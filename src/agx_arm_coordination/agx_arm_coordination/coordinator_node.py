#!/usr/bin/env python3
"""Activity-DAG coordinator for the Duo Nero system.

Loads a named activity graph (YAML-backed, decision §8), validates it, then runs
a frontier scheduler that serializes by resource token, releases ``sync_flag``
barrier groups together, and dispatches each node through its coordinator-internal
performer:

- ``Gripper`` + ``{left,right}_hand`` -> ``PerformAction`` to the side's
  ``omnihand_skill_controller``.
- ``Trajectory`` + ``{both_arms,left_arm,right_arm}`` -> the MoveIt multi-arm slice:
  anchor->anchor via ``MoveGroup`` (collision-aware plan + execute), recorded replay
  via ``ExecuteTrajectory``. MoveIt fans a both_arms plan out to the per-arm
  controllers natively, so the coordinator owns no second arm-execution path.

The coordinator never touches a vendor SDK or the hardware directly; it only
dispatches catalogue-backed actions. Child failure (or cancellation) aborts the
activity and cancels active children. Events stream on ``~/events``.

Public entry point: the ``execute_activity`` action server (PerformActivity).
"""

from __future__ import annotations

import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    RobotTrajectory,
)
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.action import PerformActivity, PerformAction
from agx_arm_msgs.msg import RobotEvent

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmTrajectoryPlanner,
    MoveGroupPlan,
    NotTaughtError,
    RecordedTrajectoryPlan,
)
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import Scheduler
from agx_arm_coordination.performer import KIND_ARM, KIND_HAND, RoutingError, route


class DispatchError(RuntimeError):
    """A node could not be dispatched (routing/planning failure)."""


class _Child:
    """Uniform handle over a dispatched child action (hand skill or arm FJT)."""

    def __init__(self, action_no: int, action_id: str) -> None:
        self.action_no = action_no
        self.action_id = action_id
        self.done = False
        self.success = False
        self.message = ""
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

    def attach_goal_future(self, future) -> None:
        self._goal_future = future

    def mark(self, success: bool, message: str) -> None:
        self.done = True
        self.success = success
        self.message = message

    def poll(self) -> None:
        if self.done:
            return
        if self._goal_future is not None and self._result_future is None:
            if not self._goal_future.done():
                return
            self._goal_handle = self._goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                self.mark(False, "goal rejected by executor")
                return
            self._result_future = self._goal_handle.get_result_async()
            return
        if self._result_future is not None and self._result_future.done():
            wrapper = self._result_future.result()
            self._interpret_result(wrapper)

    def _interpret_result(self, wrapper) -> None:  # overridden per child kind
        self.mark(False, "no result interpreter")

    def request_cancel(self) -> None:
        if self._goal_handle is not None and not self.done:
            self._goal_handle.cancel_goal_async()


class _HandChild(_Child):
    def _interpret_result(self, wrapper) -> None:
        result = wrapper.result
        self.mark(bool(result.success), result.message or result.final_state)


class _ArmChild(_Child):
    """Arm child over a MoveGroup or ExecuteTrajectory goal (both moveit_msgs).

    Both result types carry a ``moveit_msgs/MoveItErrorCodes`` ``error_code`` whose
    ``val`` is ``SUCCESS`` (1) on success.
    """

    def _interpret_result(self, wrapper) -> None:
        result = wrapper.result
        code = result.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        self.mark(ok, "" if ok else f"MoveIt error_code={code}")


class CoordinatorNode(Node):

    def __init__(self) -> None:
        super().__init__("agx_arm_coordinator")

        self.declare_parameter("config_dir", "")
        self.declare_parameter("hand_action_template", "/{side}_hand/perform")
        self.declare_parameter("arm_dry_run", False)
        self.declare_parameter("poll_period_sec", 0.05)
        self.declare_parameter("goal_accept_timeout_sec", 5.0)
        # MoveGroup planning knobs for anchor->anchor moves.
        self.declare_parameter("joint_goal_tolerance_rad", 0.01)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time_sec", 5.0)

        config_dir_param = str(self.get_parameter("config_dir").value).strip()
        if not config_dir_param:
            config_dir_param = str(
                Path(get_package_share_directory("agx_arm_coordination")) / "config"
            )
        config_dir = Path(config_dir_param)
        self.hand_action_template = str(self.get_parameter("hand_action_template").value)
        self.arm_dry_run = bool(self.get_parameter("arm_dry_run").value)
        self.poll_period = float(self.get_parameter("poll_period_sec").value)
        self.goal_accept_timeout = float(self.get_parameter("goal_accept_timeout_sec").value)
        self.joint_goal_tolerance = float(self.get_parameter("joint_goal_tolerance_rad").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time_sec").value)

        self.catalogue = ActivityCatalogue.from_config_dir(config_dir)
        arm_config_path = config_dir / "arm_config.yaml"
        self.arm_planner = ArmTrajectoryPlanner(ArmConfig.from_file(arm_config_path))

        self._cb_group = ReentrantCallbackGroup()
        self.event_pub = self.create_publisher(RobotEvent, "events", 10)

        # Child action clients (created once, reused per activity run).
        self._hand_clients: dict[str, ActionClient] = {}
        for side in ("left", "right"):
            name = self.hand_action_template.format(side=side)
            self._hand_clients[side] = ActionClient(
                self, PerformAction, name, callback_group=self._cb_group
            )
        # Arm motion goes through the MoveIt multi-arm slice: anchor->anchor via
        # MoveGroup (collision-aware plan + execute), recorded replay via
        # ExecuteTrajectory. MoveIt fans a both_arms plan out to the per-arm
        # controllers natively, so there is no per-group action client here.
        arm_cfg = self.arm_planner.config
        self._move_group_client = ActionClient(
            self, MoveGroup, arm_cfg.move_group_action, callback_group=self._cb_group
        )
        self._execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, arm_cfg.execute_trajectory_action,
            callback_group=self._cb_group,
        )

        self.action_server = ActionServer(
            self,
            PerformActivity,
            "execute_activity",
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"Coordinator up: config_dir={config_dir}, "
            f"activities={self.catalogue.available_activities()}, "
            f"arm_groups={sorted(self.arm_planner.config.groups)}, "
            f"arm_dry_run={self.arm_dry_run}"
        )

    # --- events --------------------------------------------------------------

    def _event(self, event_type: str, *, activity_id="", action_id="", robot_id="",
               state="", message="") -> None:
        event = RobotEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.source = "coordinator"
        event.robot_id = robot_id
        event.activity_id = activity_id
        event.action_id = action_id
        event.event_type = event_type
        event.state = state
        event.message = message
        self.event_pub.publish(event)

    # --- dispatch ------------------------------------------------------------

    def _dispatch(self, action_no: int, action_id: str, activity_id: str) -> _Child:
        action = self.catalogue.get_action_detail(action_id)
        try:
            decision = route(action)
        except RoutingError as exc:
            raise DispatchError(str(exc)) from exc

        if decision.kind == KIND_HAND:
            return self._dispatch_hand(action_no, action, decision, activity_id)
        if decision.kind == KIND_ARM:
            return self._dispatch_arm(action_no, action, activity_id)
        raise DispatchError(f"unhandled routing kind '{decision.kind}'")

    def _dispatch_hand(self, action_no, action, decision, activity_id) -> _Child:
        client = self._hand_clients[decision.side]
        if not client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                f"hand skill controller for {decision.robot_id} not available"
            )
        goal = PerformAction.Goal()
        goal.action_id = action.action_id
        goal.actiontype_id = action.actiontype_id
        goal.robot_id = action.robot_id
        goal.activity_id = activity_id
        goal.metadata_json = json.dumps(action.metadata)
        child = _HandChild(action_no, action.action_id)
        child.attach_goal_future(client.send_goal_async(goal))
        return child

    def _dispatch_arm(self, action_no, action, activity_id) -> _Child:
        try:
            plan = self.arm_planner.plan(action)
        except NotTaughtError as exc:
            if self.arm_dry_run:
                child = _ArmChild(action_no, action.action_id)
                child.mark(True, f"dry_run: skipped not-yet-taught trajectory ({exc})")
                return child
            raise DispatchError(str(exc)) from exc
        except ArmConfigError as exc:
            raise DispatchError(str(exc)) from exc

        if isinstance(plan, MoveGroupPlan):
            return self._dispatch_move_group(action_no, plan)
        if isinstance(plan, RecordedTrajectoryPlan):
            return self._dispatch_execute_trajectory(action_no, plan)
        raise DispatchError(f"unhandled arm plan type {type(plan).__name__}")

    def _dispatch_move_group(self, action_no, plan: MoveGroupPlan) -> _Child:
        """Anchor->anchor as a collision-aware MoveGroup plan+execute goal."""
        if self.arm_dry_run:
            child = _ArmChild(action_no, plan.action_id)
            child.mark(True, f"dry_run: would plan+execute group '{plan.planning_group}' to a "
                             f"{len(plan.joint_names)}-joint anchor via MoveGroup")
            return child
        if not self._move_group_client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                f"move_group action '{self.arm_planner.config.move_group_action}' not available"
            )
        constraints = Constraints()
        for joint_name, position in zip(plan.joint_names, plan.target_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = self.joint_goal_tolerance
            jc.tolerance_below = self.joint_goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request = MotionPlanRequest()
        request.group_name = plan.planning_group
        request.goal_constraints.append(constraints)
        request.max_velocity_scaling_factor = plan.velocity_scaling
        request.max_acceleration_scaling_factor = plan.acceleration_scaling
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.allowed_planning_time
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False  # plan AND execute
        child = _ArmChild(action_no, plan.action_id)
        child.attach_goal_future(self._move_group_client.send_goal_async(goal))
        return child

    def _dispatch_execute_trajectory(self, action_no, plan: RecordedTrajectoryPlan) -> _Child:
        """Replay a recorded trajectory through MoveIt's ExecuteTrajectory."""
        if self.arm_dry_run:
            child = _ArmChild(action_no, plan.action_id)
            child.mark(True, f"dry_run: would replay {len(plan.points)} waypoint(s) on group "
                             f"'{plan.planning_group}' via ExecuteTrajectory")
            return child
        exec_client = self._execute_trajectory_client
        if not exec_client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                "execute_trajectory action "
                f"'{self.arm_planner.config.execute_trajectory_action}' not available"
            )
        traj = JointTrajectory()
        traj.joint_names = list(plan.joint_names)
        for point in plan.points:
            jp = JointTrajectoryPoint()
            jp.positions = list(point.positions)
            sec = int(point.time_from_start_sec)
            nsec = int((point.time_from_start_sec - sec) * 1e9)
            jp.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(jp)
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj
        child = _ArmChild(action_no, plan.action_id)
        child.attach_goal_future(exec_client.send_goal_async(goal))
        return child

    # --- main execution ------------------------------------------------------

    def _execute(self, goal_handle) -> PerformActivity.Result:
        activity_id = goal_handle.request.activity_id
        result = PerformActivity.Result()

        problems = self.catalogue.validate_activity(activity_id)
        if problems:
            result.success = False
            result.message = "validation failed: " + "; ".join(problems)
            self.get_logger().error(result.message)
            self._event("failed", activity_id=activity_id, message=result.message)
            goal_handle.abort()
            return result

        graph = self.catalogue.get_activity_plan(activity_id)
        scheduler = Scheduler(graph, self.catalogue.actions)
        total = len(graph.nodes)
        result.total_nodes = total
        self.get_logger().info(f"running activity '{activity_id}' ({total} nodes)")
        self._event("started", activity_id=activity_id, message=f"{total} nodes")

        completed: set[int] = set()
        running: dict[int, _Child] = {}

        while rclpy.ok() and not scheduler.is_complete(completed):
            if goal_handle.is_cancel_requested:
                self._cancel_children(running)
                result.success = False
                result.message = "canceled"
                result.completed_nodes = len(completed)
                self._event("failed", activity_id=activity_id, message="canceled")
                goal_handle.canceled()
                return result

            # dispatch any newly ready batch
            for item in scheduler.next_batch(completed, set(running)):
                try:
                    child = self._dispatch(item.action_no, item.action_id, activity_id)
                except (DispatchError, KeyError) as exc:
                    return self._abort(
                        goal_handle, result, running, activity_id,
                        item.action_id, len(completed), str(exc),
                    )
                running[item.action_no] = child
                self.get_logger().info(f"-> dispatch {item.action_id} (#{item.action_no})")
                self._event("info", activity_id=activity_id, action_id=item.action_id,
                            robot_id=self.catalogue.actions[item.action_id].robot_id,
                            state="running", message="dispatched")
                self._publish_feedback(goal_handle, item.action_no, item.action_id,
                                       "running", len(completed), total)

            # poll running children
            for action_no in list(running):
                child = running[action_no]
                child.poll()
                if not child.done:
                    continue
                del running[action_no]
                if child.success:
                    completed.add(action_no)
                    self._event("completed", activity_id=activity_id,
                                action_id=child.action_id, state="completed",
                                message=child.message)
                    self._publish_feedback(goal_handle, action_no, child.action_id,
                                           "completed", len(completed), total)
                else:
                    return self._abort(
                        goal_handle, result, running, activity_id,
                        child.action_id, len(completed),
                        f"child failed: {child.message}",
                    )

            if not running and not scheduler.next_batch(completed, set(running)) \
                    and not scheduler.is_complete(completed):
                return self._abort(
                    goal_handle, result, running, activity_id, "",
                    len(completed), "scheduler deadlock: no runnable or running nodes",
                )

            time.sleep(self.poll_period)

        result.success = True
        result.message = f"activity '{activity_id}' complete"
        result.completed_nodes = len(completed)
        self.get_logger().info(result.message)
        self._event("completed", activity_id=activity_id, message=result.message)
        goal_handle.succeed()
        return result

    def _abort(self, goal_handle, result, running, activity_id, failed_action_id,
               completed_count, message) -> PerformActivity.Result:
        self.get_logger().error(f"aborting '{activity_id}': {message}")
        self._cancel_children(running)
        result.success = False
        result.message = message
        result.failed_action_id = failed_action_id
        result.completed_nodes = completed_count
        self._event("failed", activity_id=activity_id, action_id=failed_action_id,
                    message=message)
        goal_handle.abort()
        return result

    def _cancel_children(self, running: dict[int, _Child]) -> None:
        for child in running.values():
            child.request_cancel()
        running.clear()

    def _publish_feedback(self, goal_handle, action_no, action_id, state,
                          completed_count, total) -> None:
        feedback = PerformActivity.Feedback()
        feedback.action_no = int(action_no)
        feedback.action_id = action_id
        feedback.state = state
        feedback.completed_nodes = int(completed_count)
        feedback.total_nodes = int(total)
        goal_handle.publish_feedback(feedback)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
