
## Coordinator Node
```py
#!/usr/bin/env python3

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import KeyValue
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from cetibar_lifecycle.poll_future import poll_future
from cetibar_msgs.action import PerformAction, PerformActivity
from cetibar_msgs.error_codes.generated.error_codes import (
    EC_COORD_VALIDATION_ACTIVITY_DAG_INVALID,
    EC_COORD_VALIDATION_INVALID_STATE,
    EC_COORD_VALIDATION_RESOURCE_CONFLICT,
)
from cetibar_msgs.error_helper import make_error_result, make_success_result
from cetibar_msgs.msg import ActionRef, ActivityScaleOverride, RobotEvent
from cetibar_msgs.srv import GetActionDetail, GetActivityPlan, ValidateActivity

from coordination.scheduler import (
    CoordinatedActionSpec,
    build_adjacency,
    build_ready_groups,
    compute_indegrees,
    resource_tokens,
    select_startable_groups,
    warn_parallel_same_robot,
)


@dataclass
class ChildProgress:
    progress: float = 0.0
    current_waypoint: int = 0
    total_waypoints: int = 0


@dataclass
class ActiveChild:
    action_no: int
    spec: CoordinatedActionSpec
    goal_handle: object
    result_future: object
    started_at: float
    timeout_at: float
    group_key: Tuple[int, ...]
    progress: ChildProgress = field(default_factory=ChildProgress)


class CoordinationNode(Node):
    _MAX_ACTIVITY_VELOCITY_SCALING = 1.5

    def __init__(self) -> None:
        super().__init__('coordination_node')

        self._callback_group = ReentrantCallbackGroup()
        self._active_children_lock = threading.Lock()
        self._active_children: Dict[int, ActiveChild] = {}

        self._load_configuration()
        self._create_clients()
        self._create_publishers()
        self._create_action_server()

    def _load_configuration(self) -> None:
        default_config_file = os.path.join(
            get_package_share_directory('coordination'),
            'config',
            'coordination_params.yaml',
        )

        ros_ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        default_namespaces_file = os.path.join(ros_ws_root, 'config', 'namespaces.yaml')

        self.declare_parameter('config_file', default_config_file)
        self.declare_parameter('namespaces_config_path', default_namespaces_file)
        self.declare_parameter('db_bridge.namespace', '/db_bridge')
        self.declare_parameter('performer_helper.namespace', '/performer_helper')
        self.declare_parameter('timeouts.service_call', 5.0)
        self.declare_parameter('timeouts.action_server_wait', 5.0)
        self.declare_parameter('timeouts.action_result_wait', 300.0)
        self.declare_parameter('timeouts.cancel_wait', 5.0)
        self.declare_parameter('execution.progress_poll_sec', 0.05)
        self.declare_parameter('execution.serialize_by_robot', True)

        config_file = str(self.get_parameter('config_file').value)
        self._raw_config = {}
        try:
            with open(config_file, 'r', encoding='utf-8') as handle:
                self._raw_config = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            self.get_logger().warn(f'Config file not found, using scalar params only: {config_file}')

        namespaces_file = str(self.get_parameter('namespaces_config_path').value)
        self._shared_resources = self._load_shared_resources(namespaces_file)

        self._service_timeout = float(self.get_parameter('timeouts.service_call').value)
        self._action_server_wait = float(self.get_parameter('timeouts.action_server_wait').value)
        self._action_result_wait = float(self.get_parameter('timeouts.action_result_wait').value)
        self._cancel_wait = float(self.get_parameter('timeouts.cancel_wait').value)
        self._progress_poll_sec = float(self.get_parameter('execution.progress_poll_sec').value)
        self._serialize_by_robot = bool(self.get_parameter('execution.serialize_by_robot').value)

    def _load_shared_resources(self, file_path: str) -> Dict[str, Sequence[str]]:
        try:
            with open(file_path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            self.get_logger().warn(f'Namespace config not found: {file_path}')
            return {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to read namespace config {file_path}: {exc}')
            return {}

        resources = data.get('resources', {}) or {}
        parsed: Dict[str, Sequence[str]] = {}
        for resource_name, resource_data in resources.items():
            robots = resource_data.get('robots', []) if isinstance(resource_data, dict) else []
            parsed[str(resource_name)] = [str(robot_id) for robot_id in robots]
        return parsed

    def _create_clients(self) -> None:
        db_ns = str(self.get_parameter('db_bridge.namespace').value).rstrip('/')
        performer_ns = str(self.get_parameter('performer_helper.namespace').value).rstrip('/')

        self._get_activity_plan_client = self.create_client(
            GetActivityPlan,
            f'{db_ns}/get_activity_plan',
            callback_group=self._callback_group,
        )
        self._validate_activity_client = self.create_client(
            ValidateActivity,
            f'{db_ns}/validate_activity',
            callback_group=self._callback_group,
        )
        self._get_action_detail_client = self.create_client(
            GetActionDetail,
            f'{db_ns}/get_action_detail',
            callback_group=self._callback_group,
        )
        self._perform_action_client = ActionClient(
            self,
            PerformAction,
            f'{performer_ns}/perform',
            callback_group=self._callback_group,
        )

    def _create_publishers(self) -> None:
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._events_pub = self.create_publisher(RobotEvent, '/coord/events', qos)

    def _create_action_server(self) -> None:
        self._action_server = ActionServer(
            self,
            PerformActivity,
            '/coord/execute_activity',
            execute_callback=self._execute_activity,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

    def _goal_callback(self, goal_request: PerformActivity.Goal) -> GoalResponse:
        if not goal_request.activity_id:
            self.get_logger().warn('Rejecting execute_activity goal with empty activity_id')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Received coordinator cancel request')
        return CancelResponse.ACCEPT

    def _execute_activity(self, goal_handle) -> PerformActivity.Result:
        request = goal_handle.request
        activity_id = request.activity_id
        velocity_scaling = self._normalize_activity_scaling(request.velocity_scaling, default=1.0)
        action_scale_overrides = self._build_activity_scale_override_map(
            getattr(request, 'action_scale_overrides', []),
            default_scaling=velocity_scaling,
        )

        self._publish_event(
            RobotEvent.MOVE_STARTED,
            severity=0,
            message=f"Starting coordinated activity '{activity_id}'",
            correlation_id=activity_id,
        )

        feedback = PerformActivity.Feedback()
        feedback.status = 'loading'
        feedback.progress = 0.0
        feedback.total_actions = 0
        goal_handle.publish_feedback(feedback)

        try:
            plan_response = self._call_service(self._get_activity_plan_client, self._make_activity_plan_request(activity_id))
            if plan_response is None:
                return self._error_result(
                    EC_COORD_VALIDATION_INVALID_STATE,
                    'DB_Bridge GetActivityPlan service unavailable or timed out',
                )
            if not plan_response.result.success:
                result = PerformActivity.Result()
                result.result = plan_response.result
                result.activity_id = activity_id
                return result

            validate_request = ValidateActivity.Request()
            validate_request.nodes = list(plan_response.nodes)
            validate_request.edges = list(plan_response.edges)
            validate_response = self._call_service(self._validate_activity_client, validate_request)
            if validate_response is None:
                return self._error_result(
                    EC_COORD_VALIDATION_INVALID_STATE,
                    'DB_Bridge ValidateActivity service unavailable or timed out',
                )
            if not validate_response.result.success or not validate_response.valid:
                return self._error_result(
                    EC_COORD_VALIDATION_ACTIVITY_DAG_INVALID,
                    self._validation_message(validate_response),
                )

            specs_by_no = self._load_action_specs(plan_response.nodes)
            graph_edges = [(int(edge.src_action_no), int(edge.dst_action_no)) for edge in plan_response.edges]
            adjacency = build_adjacency(graph_edges)
            indegrees = compute_indegrees(specs_by_no.keys(), graph_edges)

            warnings = warn_parallel_same_robot(specs_by_no, graph_edges)
            for message in warnings:
                self.get_logger().warn(message)

            total_actions = len(specs_by_no)
            completed: List[int] = []
            started: Set[int] = set()
            children = adjacency

            while len(completed) < total_actions:
                if goal_handle.is_cancel_requested:
                    self._cancel_active_children()
                    goal_handle.canceled()
                    return self._canceled_result(activity_id, completed, total_actions, specs_by_no)

                failed_result = self._process_active_children(
                    goal_handle,
                    activity_id,
                    completed,
                    total_actions,
                    specs_by_no,
                    indegrees,
                    children,
                )
                if failed_result is not None:
                    return failed_result

                if len(completed) >= total_actions:
                    break

                ready_action_nos = [
                    action_no
                    for action_no, indegree in indegrees.items()
                    if indegree == 0 and action_no not in started and action_no not in completed
                ]

                blocked_tokens = self._active_resource_tokens()
                ready_groups = build_ready_groups(ready_action_nos, specs_by_no)
                startable_groups, validation_errors = select_startable_groups(
                    ready_groups,
                    specs_by_no,
                    self._shared_resources,
                    blocked_tokens=blocked_tokens,
                    serialize_by_robot=self._serialize_by_robot,
                )
                if validation_errors:
                    return self._error_result(
                        EC_COORD_VALIDATION_RESOURCE_CONFLICT,
                        '; '.join(validation_errors),
                        activity_id=activity_id,
                        completed=completed,
                        total_actions=total_actions,
                        specs_by_no=specs_by_no,
                    )

                if startable_groups:
                    started_children = self._start_groups(
                        startable_groups,
                        specs_by_no,
                        velocity_scaling,
                        action_scale_overrides,
                    )
                    started.update(started_children.keys())
                    with self._active_children_lock:
                        self._active_children.update(started_children)

                if not ready_action_nos and not self._has_active_children():
                    return self._error_result(
                        EC_COORD_VALIDATION_ACTIVITY_DAG_INVALID,
                        'Coordinator stalled: no runnable frontier and no active children',
                        activity_id=activity_id,
                        completed=completed,
                        total_actions=total_actions,
                        specs_by_no=specs_by_no,
                    )

                if not self._has_active_children() and ready_action_nos and not startable_groups:
                    return self._error_result(
                        EC_COORD_VALIDATION_RESOURCE_CONFLICT,
                        'Ready actions are blocked by coordinator resource rules',
                        activity_id=activity_id,
                        completed=completed,
                        total_actions=total_actions,
                        specs_by_no=specs_by_no,
                    )

                self._publish_progress_feedback(goal_handle, completed, total_actions)
                time.sleep(self._progress_poll_sec)

            result = PerformActivity.Result()
            result.result = make_success_result()
            result.activity_id = activity_id
            result.actions_executed = len(completed)
            result.total_actions = total_actions
            result.total_duration = 0.0
            result.action_ids_executed = [specs_by_no[action_no].action_id for action_no in sorted(completed)]

            feedback.status = 'completed'
            feedback.progress = 1.0
            feedback.total_actions = total_actions
            feedback.current_action_id = ''
            feedback.current_action_index = total_actions - 1 if total_actions else 0
            feedback.action_progress = 1.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()

            self._publish_event(
                RobotEvent.MOVE_COMPLETED,
                severity=0,
                message=f"Completed coordinated activity '{activity_id}'",
                correlation_id=activity_id,
            )
            return result

        except Exception as exc:
            self.get_logger().error(f'Coordinator execution failed: {exc}')
            self._cancel_active_children()
            self._publish_event(
                RobotEvent.MOVE_FAILED,
                severity=2,
                message=f"Coordinator failed for '{activity_id}': {exc}",
                correlation_id=activity_id,
            )
            result = PerformActivity.Result()
            result.result = make_error_result(
                EC_COORD_VALIDATION_INVALID_STATE,
                f'Coordinator execution error: {exc}',
                severity=3,
            )
            result.activity_id = activity_id
            return result

    def _make_activity_plan_request(self, activity_id: str) -> GetActivityPlan.Request:
        request = GetActivityPlan.Request()
        request.activity_id = activity_id
        return request

    def _call_service(self, client, request):
        if not client.wait_for_service(timeout_sec=min(self._service_timeout, 2.0)):
            return None
        return poll_future(client.call_async(request), self._service_timeout)

    def _load_action_specs(self, nodes) -> Dict[int, CoordinatedActionSpec]:
        specs_by_no: Dict[int, CoordinatedActionSpec] = {}
        for node in nodes:
            request = GetActionDetail.Request()
            request.action_ref = ActionRef(
                action_id=node.action_ref.action_id,
                db_rowid=getattr(node.action_ref, 'db_rowid', 0),
            )
            response = self._call_service(self._get_action_detail_client, request)
            if response is None or not response.result.success:
                message = 'unknown error'
                if response is not None and response.result.error:
                    message = response.result.error.message
                raise RuntimeError(f"Failed to load action detail for '{node.action_ref.action_id}': {message}")

            action = response.action
            specs_by_no[int(node.action_no)] = CoordinatedActionSpec(
                action_no=int(node.action_no),
                action_id=action.action_id,
                robot_id=action.robot_id,
                actiontype_id=action.actiontype_id,
                sync_flag=int(node.sync_flag) if int(node.sync_flag) >= 0 else None,
            )
        return specs_by_no

    def _start_groups(
        self,
        groups: Sequence[Tuple[int, ...]],
        specs_by_no: Dict[int, CoordinatedActionSpec],
        velocity_scaling: float,
        action_scale_overrides: Dict[int, float],
    ) -> Dict[int, ActiveChild]:
        started_children: Dict[int, ActiveChild] = {}
        if not self._perform_action_client.wait_for_server(timeout_sec=self._action_server_wait):
            raise RuntimeError('PerformerHelper perform action server unavailable')

        for group in groups:
            send_futures: List[Tuple[int, CoordinatedActionSpec, ChildProgress, object]] = []
            accepted_children: List[Tuple[str, object]] = []
            for action_no in group:
                spec = specs_by_no[action_no]
                goal = PerformAction.Goal()
                goal.action_ref = ActionRef(action_id=spec.action_id)
                goal.speed_scaling = self._resolve_activity_scaling(
                    action_no,
                    velocity_scaling,
                    action_scale_overrides,
                )
                child_progress = ChildProgress()

                def _feedback(feedback_msg, current=child_progress):
                    current.progress = float(feedback_msg.feedback.progress)
                    current.current_waypoint = int(feedback_msg.feedback.current_waypoint)
                    current.total_waypoints = int(feedback_msg.feedback.total_waypoints)

                send_future = self._perform_action_client.send_goal_async(goal, feedback_callback=_feedback)
                send_futures.append((action_no, spec, child_progress, send_future))

            started_at = time.monotonic()
            timeout_at = started_at + self._action_result_wait
            try:
                for action_no, spec, child_progress, send_future in send_futures:
                    child_goal_handle = poll_future(send_future, self._action_server_wait)
                    if child_goal_handle is None or not child_goal_handle.accepted:
                        raise RuntimeError(f"PerformerHelper rejected action '{spec.action_id}'")
                    accepted_children.append((spec.action_id, child_goal_handle))
                    started_children[action_no] = ActiveChild(
                        action_no=action_no,
                        spec=spec,
                        goal_handle=child_goal_handle,
                        result_future=child_goal_handle.get_result_async(),
                        started_at=started_at,
                        timeout_at=timeout_at,
                        group_key=tuple(group),
                        progress=child_progress,
                    )
            except Exception:
                for action_id, child_goal_handle in accepted_children:
                    try:
                        cancel_future = child_goal_handle.cancel_goal_async()
                        poll_future(cancel_future, self._cancel_wait)
                    except Exception as exc:
                        self.get_logger().warn(
                            f"Failed to cancel partially-started action '{action_id}': {exc}"
                        )
                raise
        return started_children

    @classmethod
    def _normalize_activity_scaling(cls, value: float, *, default: float) -> float:
        try:
            scaling = float(value)
        except (TypeError, ValueError):
            return float(default)
        if scaling <= 0.0:
            return float(default)
        return min(scaling, cls._MAX_ACTIVITY_VELOCITY_SCALING)

    @classmethod
    def _build_activity_scale_override_map(
        cls,
        overrides: Sequence[ActivityScaleOverride],
        *,
        default_scaling: float,
    ) -> Dict[int, float]:
        override_map: Dict[int, float] = {}
        for override in overrides:
            try:
                action_no = int(getattr(override, 'action_no', 0))
            except (TypeError, ValueError):
                continue
            if action_no <= 0:
                continue
            override_map[action_no] = cls._normalize_activity_scaling(
                getattr(override, 'velocity_scaling', 0.0),
                default=default_scaling,
            )
        return override_map

    @staticmethod
    def _resolve_activity_scaling(
        action_no: int,
        default_scaling: float,
        action_scale_overrides: Dict[int, float],
    ) -> float:
        return float(action_scale_overrides.get(int(action_no), default_scaling))

    def _process_active_children(
        self,
        goal_handle,
        activity_id: str,
        completed: List[int],
        total_actions: int,
        specs_by_no: Dict[int, CoordinatedActionSpec],
        indegrees: Dict[int, int],
        children: Dict[int, Set[int]],
    ) -> Optional[PerformActivity.Result]:
        with self._active_children_lock:
            active_children = dict(self._active_children)

        if not active_children:
            return None

        now = time.monotonic()
        done_children: List[Tuple[int, ActiveChild]] = []
        for action_no, child in active_children.items():
            if child.result_future.done():
                done_children.append((action_no, child))
                continue
            if now > child.timeout_at:
                self._cancel_active_children()
                return self._error_result(
                    EC_COORD_VALIDATION_INVALID_STATE,
                    f"Timed out waiting for action '{child.spec.action_id}'",
                    activity_id=activity_id,
                    completed=completed,
                    total_actions=total_actions,
                    specs_by_no=specs_by_no,
                )

        for action_no, child in sorted(done_children, key=lambda item: item[0]):
            try:
                wrapped_result = child.result_future.result()
            except Exception as exc:
                self._cancel_active_children(except_action_no=action_no)
                return self._error_result(
                    EC_COORD_VALIDATION_INVALID_STATE,
                    f"Action '{child.spec.action_id}' result retrieval failed: {exc}",
                    activity_id=activity_id,
                    completed=completed,
                    total_actions=total_actions,
                    specs_by_no=specs_by_no,
                )

            if not wrapped_result.result.result.success:
                self._cancel_active_children(except_action_no=action_no)
                self._publish_event(
                    RobotEvent.MOVE_FAILED,
                    severity=2,
                    message=f"Action '{child.spec.action_id}' failed during coordinated activity",
                    correlation_id=activity_id,
                )
                result = PerformActivity.Result()
                result.result = wrapped_result.result.result
                result.activity_id = activity_id
                result.actions_executed = len(completed)
                result.total_actions = total_actions
                result.total_duration = 0.0
                result.action_ids_executed = [specs_by_no[action_no].action_id for action_no in sorted(completed)]
                goal_handle.abort()
                return result

            completed.append(action_no)
            for child_action_no in children.get(action_no, set()):
                indegrees[child_action_no] -= 1
            with self._active_children_lock:
                self._active_children.pop(action_no, None)

        return None

    def _publish_progress_feedback(
        self,
        goal_handle,
        completed: Sequence[int],
        total_actions: int,
    ) -> None:
        feedback = PerformActivity.Feedback()
        with self._active_children_lock:
            active_children = list(self._active_children.values())

        feedback.status = self._feedback_status(active_children)
        feedback.current_action_id = ','.join(child.spec.action_id for child in active_children)
        feedback.current_action_index = len(completed)
        feedback.total_actions = total_actions
        progress_sum = float(len(completed))
        for child in active_children:
            progress_sum += max(0.0, min(1.0, child.progress.progress))
        feedback.progress = progress_sum / float(total_actions) if total_actions else 0.0
        feedback.action_progress = (
            sum(child.progress.progress for child in active_children) / float(len(active_children))
            if active_children else 0.0
        )
        goal_handle.publish_feedback(feedback)

    def _feedback_status(self, active_children: Sequence[ActiveChild]) -> str:
        if not active_children:
            return 'executing'
        group_sizes: Dict[Tuple[int, ...], int] = {}
        for child in active_children:
            group_sizes[child.group_key] = group_sizes.get(child.group_key, 0) + 1
        if any(size > 1 for size in group_sizes.values()):
            return 'waiting_sync'
        return 'executing'

    def _cancel_active_children(self, except_action_no: Optional[int] = None) -> None:
        with self._active_children_lock:
            active_children = list(self._active_children.values())
        for child in active_children:
            if except_action_no is not None and child.action_no == except_action_no:
                continue
            try:
                cancel_future = child.goal_handle.cancel_goal_async()
                poll_future(cancel_future, self._cancel_wait)
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel child action '{child.spec.action_id}': {exc}")

    def _active_resource_tokens(self) -> Set[str]:
        with self._active_children_lock:
            active_children = list(self._active_children.values())
        tokens: Set[str] = set()
        for child in active_children:
            tokens.update(
                resource_tokens(
                    child.spec,
                    self._shared_resources,
                    serialize_by_robot=self._serialize_by_robot,
                )
            )
        return tokens

    def _has_active_children(self) -> bool:
        with self._active_children_lock:
            return bool(self._active_children)

    def _set_active_children(self, active_children: Dict[int, ActiveChild]) -> None:
        with self._active_children_lock:
            self._active_children = dict(active_children)

    def _canceled_result(
        self,
        activity_id: str,
        completed: Sequence[int],
        total_actions: int,
        specs_by_no: Dict[int, CoordinatedActionSpec],
    ) -> PerformActivity.Result:
        self._publish_event(
            RobotEvent.MOVE_STOPPED,
            severity=1,
            message=f"Canceled coordinated activity '{activity_id}'",
            correlation_id=activity_id,
        )
        result = PerformActivity.Result()
        result.result = make_error_result(
            EC_COORD_VALIDATION_INVALID_STATE,
            f"Activity '{activity_id}' canceled",
            severity=1,
        )
        result.activity_id = activity_id
        result.actions_executed = len(completed)
        result.total_actions = total_actions
        result.total_duration = 0.0
        result.action_ids_executed = [specs_by_no[action_no].action_id for action_no in sorted(completed)]
        return result

    def _error_result(
        self,
        error_code: int,
        message: str,
        *,
        activity_id: str = '',
        completed: Optional[Sequence[int]] = None,
        total_actions: int = 0,
        specs_by_no: Optional[Dict[int, CoordinatedActionSpec]] = None,
    ) -> PerformActivity.Result:
        result = PerformActivity.Result()
        result.result = make_error_result(error_code, message, severity=3)
        result.activity_id = activity_id
        completed = completed or []
        specs_by_no = specs_by_no or {}
        result.actions_executed = len(completed)
        result.total_actions = total_actions
        result.total_duration = 0.0
        result.action_ids_executed = [specs_by_no[action_no].action_id for action_no in sorted(completed)]
        return result

    def _validation_message(self, validate_response: ValidateActivity.Response) -> str:
        if validate_response.errors:
            return '; '.join(issue.message for issue in validate_response.errors)
        if validate_response.result.error:
            return validate_response.result.error.message
        return 'Activity validation failed'

    def _publish_event(self, event_type: int, severity: int, message: str, correlation_id: str = '') -> None:
        event = RobotEvent()
        event.source = '/coord/coordination_node'
        event.type = event_type
        event.severity = int(severity)
        event.message = message
        event.correlation_id = correlation_id
        event.stamp = self.get_clock().now().to_msg()
        self._events_pub.publish(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoordinationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

```


## Scheduler
```py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


@dataclass(frozen=True)
class CoordinatedActionSpec:
    action_no: int
    action_id: str
    robot_id: str
    actiontype_id: str
    sync_flag: Optional[int] = None


def normalize_sync_flag(sync_flag: Optional[int]) -> Optional[int]:
    if sync_flag is None:
        return None
    value = int(sync_flag)
    return None if value < 0 else value


def build_adjacency(edges: Iterable[Tuple[int, int]]) -> Dict[int, Set[int]]:
    adjacency: Dict[int, Set[int]] = {}
    for src, dst in edges:
        adjacency.setdefault(int(src), set()).add(int(dst))
        adjacency.setdefault(int(dst), set())
    return adjacency


def compute_indegrees(
    action_nos: Iterable[int],
    edges: Iterable[Tuple[int, int]],
) -> Dict[int, int]:
    indegrees = {int(action_no): 0 for action_no in action_nos}
    for _, dst in edges:
        indegrees[int(dst)] += 1
    return indegrees


def reachable_from(start: int, adjacency: Dict[int, Set[int]]) -> Set[int]:
    seen: Set[int] = set()
    stack = [int(start)]
    while stack:
        current = stack.pop()
        for nxt in adjacency.get(current, set()):
            if nxt in seen:
                continue
            seen.add(nxt)
            stack.append(nxt)
    return seen


def has_path_between(u: int, v: int, adjacency: Dict[int, Set[int]]) -> bool:
    return int(v) in reachable_from(int(u), adjacency) or int(u) in reachable_from(int(v), adjacency)


def resource_tokens(
    spec: CoordinatedActionSpec,
    shared_resources: Dict[str, Sequence[str]],
    serialize_by_robot: bool = True,
) -> Set[str]:
    tokens: Set[str] = set()
    if serialize_by_robot:
        tokens.add(f"robot:{spec.robot_id}")
    for resource_name, robots in shared_resources.items():
        if spec.robot_id in robots:
            tokens.add(f"resource:{resource_name}")
    return tokens


def conflicting_pairs_in_group(
    specs: Sequence[CoordinatedActionSpec],
    shared_resources: Dict[str, Sequence[str]],
    serialize_by_robot: bool = True,
) -> List[Tuple[str, str, str]]:
    conflicts: List[Tuple[str, str, str]] = []
    seen_tokens: Dict[str, str] = {}
    ordered_specs = sorted(specs, key=lambda item: item.action_no)
    for spec in ordered_specs:
        for token in sorted(resource_tokens(spec, shared_resources, serialize_by_robot)):
            owner = seen_tokens.get(token)
            if owner is not None:
                conflicts.append((owner, spec.action_id, token))
            else:
                seen_tokens[token] = spec.action_id
    return conflicts


def build_ready_groups(
    ready_action_nos: Iterable[int],
    specs_by_no: Dict[int, CoordinatedActionSpec],
) -> List[Tuple[int, ...]]:
    ready = sorted(int(action_no) for action_no in ready_action_nos)
    ready_set = set(ready)
    sync_members = group_sync_members(specs_by_no)
    singleton_groups: List[Tuple[int, ...]] = []
    grouped_sync_flags: Set[int] = set()

    for action_no in ready:
        spec = specs_by_no[action_no]
        sync_flag = normalize_sync_flag(spec.sync_flag)
        if sync_flag is None:
            singleton_groups.append((action_no,))
            continue

        if sync_flag in grouped_sync_flags:
            continue

        group_members = sync_members.get(sync_flag, ())
        if set(group_members).issubset(ready_set):
            grouped_sync_flags.add(sync_flag)

    grouped = [sync_members[sync_flag] for sync_flag in sorted(grouped_sync_flags)]
    grouped.extend(singleton_groups)
    return sorted(grouped, key=lambda group: min(group))


def group_sync_members(
    specs_by_no: Dict[int, CoordinatedActionSpec],
) -> Dict[int, Tuple[int, ...]]:
    by_flag: Dict[int, List[int]] = {}
    for action_no, spec in specs_by_no.items():
        sync_flag = normalize_sync_flag(spec.sync_flag)
        if sync_flag is None:
            continue
        by_flag.setdefault(sync_flag, []).append(int(action_no))
    return {
        sync_flag: tuple(sorted(action_nos))
        for sync_flag, action_nos in by_flag.items()
    }


def select_startable_groups(
    candidate_groups: Sequence[Tuple[int, ...]],
    specs_by_no: Dict[int, CoordinatedActionSpec],
    shared_resources: Dict[str, Sequence[str]],
    blocked_tokens: Optional[Set[str]] = None,
    serialize_by_robot: bool = True,
) -> Tuple[List[Tuple[int, ...]], List[str]]:
    selected: List[Tuple[int, ...]] = []
    validation_errors: List[str] = []
    used_tokens: Set[str] = set(blocked_tokens or set())

    for group in candidate_groups:
        specs = [specs_by_no[action_no] for action_no in group]
        conflicts = conflicting_pairs_in_group(specs, shared_resources, serialize_by_robot)
        if conflicts:
            if len(group) > 1:
                for left_action_id, right_action_id, token in conflicts:
                    validation_errors.append(
                        f"Sync group contains conflicting actions '{left_action_id}' and '{right_action_id}' via {token}"
                    )
            continue

        group_tokens: Set[str] = set()
        for spec in specs:
            group_tokens.update(resource_tokens(spec, shared_resources, serialize_by_robot))

        if used_tokens.intersection(group_tokens):
            continue

        selected.append(group)
        used_tokens.update(group_tokens)

    return selected, validation_errors


def warn_parallel_same_robot(
    specs_by_no: Dict[int, CoordinatedActionSpec],
    edges: Iterable[Tuple[int, int]],
) -> List[str]:
    adjacency = build_adjacency(edges)
    by_robot: Dict[str, List[int]] = {}
    for action_no, spec in specs_by_no.items():
        by_robot.setdefault(spec.robot_id, []).append(action_no)

    warnings: List[str] = []
    for robot_id, action_nos in sorted(by_robot.items()):
        sorted_action_nos = sorted(action_nos)
        for index, left in enumerate(sorted_action_nos):
            for right in sorted_action_nos[index + 1:]:
                if has_path_between(left, right, adjacency):
                    continue
                warnings.append(
                    f"Robot '{robot_id}' has parallel branches at nodes {left} and {right}"
                )
    return warnings
```