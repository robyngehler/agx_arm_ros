#!/usr/bin/env python3
"""Activity-DAG coordinator for the Duo Nero system.

Loads a named activity graph (YAML-backed), validates it, then runs
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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.action import PerformActivity, PerformAction
from agx_arm_msgs.msg import RobotEvent

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmTrajectoryPlanner,
    MoveGroupPlan,
    NotTaughtError,
    PlanMergeError,
    RecordedTrajectoryPlan,
    merge_arm_plans,
)
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import ACTIONTYPE_TRAJECTORY, Scheduler
from agx_arm_coordination.performer import KIND_ARM, KIND_HAND, RoutingError, route


class DispatchError(RuntimeError):
    """A node could not be dispatched (routing/planning failure)."""


class _Child:
    """Uniform handle over a dispatched child action (hand skill or arm FJT)."""

    def __init__(self, action_no: int, action_id: str) -> None:
        self.action_no = action_no
        self.action_id = action_id
        # Action numbers this child completes. Usually just its own, but a merged
        # duo dispatch (two synced per-arm actions -> one both_arms goal) covers
        # both, so both are marked done/complete together.
        self.action_nos = [action_no]
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
    side: str = ""  # arm side whose hand window was opened for this action

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


class _PhasedArmChild(_ArmChild):
    """One catalogue action executed as an ordered chain of MoveIt goals.

    Used for a recorded replay, which is really two steps: a planned
    ``MoveGroup`` approach to the taught trajectory's first waypoint, then the
    ``ExecuteTrajectory`` replay itself. A phase that fails fails the whole
    action; the phases share one action_no, so the graph still sees a single node.

    Phases are callables returning a goal future, invoked lazily: the approach is
    sent first, and the replay goal is only built once the approach succeeded.
    """

    def __init__(self, action_no: int, action_id: str, phases: list, labels: list[str]) -> None:
        super().__init__(action_no, action_id)
        self._phases = list(phases)
        self._labels = list(labels)
        self._phase_index = -1

    def start(self) -> None:
        self._advance()

    def _advance(self) -> None:
        self._phase_index += 1
        if self._phase_index >= len(self._phases):
            self.mark(True, f"completed {len(self._phases)} phase(s)")
            return
        self._goal_future = self._phases[self._phase_index]()
        self._result_future = None
        self._goal_handle = None

    def _label(self) -> str:
        if 0 <= self._phase_index < len(self._labels):
            return self._labels[self._phase_index]
        return f"phase {self._phase_index + 1}"

    def mark(self, success: bool, message: str) -> None:
        if not success and not message.startswith("phase"):
            message = f"{self._label()}: {message}"
        super().mark(success, message)

    def _interpret_result(self, wrapper) -> None:
        code = wrapper.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.mark(False, f"MoveIt error_code={code}")
            return
        self._advance()


class CoordinatorNode(Node):

    def __init__(self) -> None:
        super().__init__("agx_arm_coordinator")

        self.declare_parameter("config_dir", "")
        self.declare_parameter("hand_action_template", "/{side}_hand/perform")
        # Driver-side step-and-settle handoff services, per arm side. Before a
        # hand action runs on a shared side bus the arm is quiesced into a
        # verified hold (prepare_hand_window) and reopened afterwards
        # (resume_arm_control), so the hand actually owns the bus.
        self.declare_parameter("arm_service_template", "/{side}_arm")
        self.declare_parameter("handoff_enabled", True)
        self.declare_parameter("handoff_timeout_sec", 5.0)
        self.declare_parameter("arm_dry_run", False)
        self.declare_parameter("poll_period_sec", 0.05)
        self.declare_parameter("goal_accept_timeout_sec", 5.0)
        # MoveGroup planning knobs for anchor->anchor moves.
        self.declare_parameter("joint_goal_tolerance_rad", 0.01)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time_sec", 5.0)
        # Scaling for the planned approach that precedes a recorded replay. Kept
        # separate from the action's velocity_scaling, which for a recorded action
        # stretches replay time rather than limiting the planner.
        self.declare_parameter("recorded_approach_scaling", 0.10)

        config_dir_param = str(self.get_parameter("config_dir").value).strip()
        if not config_dir_param:
            config_dir_param = str(
                Path(get_package_share_directory("agx_arm_coordination")) / "config"
            )
        config_dir = Path(config_dir_param)
        self.hand_action_template = str(self.get_parameter("hand_action_template").value)
        self.arm_service_template = str(self.get_parameter("arm_service_template").value)
        self.handoff_enabled = bool(self.get_parameter("handoff_enabled").value)
        self.handoff_timeout = float(self.get_parameter("handoff_timeout_sec").value)
        self.arm_dry_run = bool(self.get_parameter("arm_dry_run").value)
        self.poll_period = float(self.get_parameter("poll_period_sec").value)
        self.goal_accept_timeout = float(self.get_parameter("goal_accept_timeout_sec").value)
        self.joint_goal_tolerance = float(self.get_parameter("joint_goal_tolerance_rad").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time_sec").value)
        self.recorded_approach_scaling = min(
            max(float(self.get_parameter("recorded_approach_scaling").value), 1e-3), 1.0
        )

        self.catalogue = ActivityCatalogue.from_config_dir(config_dir)
        arm_config_path = config_dir / "arm_config.yaml"
        self.arm_planner = ArmTrajectoryPlanner(ArmConfig.from_file(arm_config_path))

        self._cb_group = ReentrantCallbackGroup()
        self.event_pub = self.create_publisher(RobotEvent, "events", 10)

        # Child action clients (created once, reused per activity run).
        self._hand_clients: dict[str, ActionClient] = {}
        self._prepare_clients: dict[str, object] = {}
        self._resume_clients: dict[str, object] = {}
        for side in ("left", "right"):
            name = self.hand_action_template.format(side=side)
            self._hand_clients[side] = ActionClient(
                self, PerformAction, name, callback_group=self._cb_group
            )
            arm_ns = self.arm_service_template.format(side=side)
            self._prepare_clients[side] = self.create_client(
                Trigger, f"{arm_ns}/prepare_hand_window", callback_group=self._cb_group
            )
            self._resume_clients[side] = self.create_client(
                Trigger, f"{arm_ns}/resume_arm_control", callback_group=self._cb_group
            )
        # Sides whose arm is currently quiesced for a hand window (prepared but
        # not yet resumed), so any exit path can reopen them.
        self._open_hand_windows: set[str] = set()
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

    def _call_trigger_sync(self, client, label: str) -> tuple[bool, str]:
        """Call a Trigger service and wait for its result (bounded)."""
        if not client.wait_for_service(timeout_sec=self.handoff_timeout):
            return False, f"{label}: service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.handoff_timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{label}: timed out"
            time.sleep(self.poll_period)
        resp = future.result()
        if resp is None:
            return False, f"{label}: no response"
        return bool(resp.success), resp.message or ""

    def _open_hand_window(self, side: str) -> None:
        """Quiesce the arm on ``side`` into a verified hold before a hand action.

        Raises DispatchError if the arm cannot be safely parked, so a hand action
        never runs while the arm still streams on the shared side bus.
        """
        if not self.handoff_enabled or side in self._open_hand_windows:
            return
        ok, msg = self._call_trigger_sync(
            self._prepare_clients[side], f"prepare_hand_window[{side}]"
        )
        if not ok:
            raise DispatchError(f"could not open hand window on {side}: {msg}")
        self._open_hand_windows.add(side)
        self.get_logger().info(f"hand window opened on {side} (arm quiesced): {msg}")

    def _resume_hand_window(self, side: str) -> None:
        """Reopen the arm side after a hand action (best-effort)."""
        if not side or side not in self._open_hand_windows:
            return
        ok, msg = self._call_trigger_sync(
            self._resume_clients[side], f"resume_arm_control[{side}]"
        )
        self._open_hand_windows.discard(side)
        if ok:
            self.get_logger().info(f"arm control resumed on {side}: {msg}")
        else:
            self.get_logger().error(f"resume_arm_control failed on {side}: {msg}")

    def _resume_all_hand_windows(self) -> None:
        for side in list(self._open_hand_windows):
            self._resume_hand_window(side)

    def _dispatch_hand(self, action_no, action, decision, activity_id) -> _Child:
        client = self._hand_clients[decision.side]
        if not client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                f"hand skill controller for {decision.robot_id} not available"
            )
        # Quiesce the arm on this side into a verified hold before the hand takes
        # the shared bus; raises if the arm cannot be safely parked.
        self._open_hand_window(decision.side)
        goal = PerformAction.Goal()
        goal.action_id = action.action_id
        goal.actiontype_id = action.actiontype_id
        goal.robot_id = action.robot_id
        goal.activity_id = activity_id
        goal.metadata_json = json.dumps(action.metadata)
        child = _HandChild(action_no, action.action_id)
        child.side = decision.side
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

    def _require_move_group(self) -> None:
        if not self._move_group_client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                f"move_group action '{self.arm_planner.config.move_group_action}' not available"
            )

    def _build_move_group_goal(
        self, planning_group, joint_names, positions, velocity_scaling, acceleration_scaling
    ) -> MoveGroup.Goal:
        """A collision-aware joint-space plan+execute goal for one planning group."""
        constraints = Constraints()
        for joint_name, position in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = self.joint_goal_tolerance
            jc.tolerance_below = self.joint_goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request = MotionPlanRequest()
        request.group_name = planning_group
        request.goal_constraints.append(constraints)
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.allowed_planning_time
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False  # plan AND execute
        return goal

    def _dispatch_move_group(self, action_no, plan: MoveGroupPlan) -> _Child:
        """Anchor->anchor as a collision-aware MoveGroup plan+execute goal."""
        if self.arm_dry_run:
            child = _ArmChild(action_no, plan.action_id)
            child.mark(True, f"dry_run: would plan+execute group '{plan.planning_group}' to a "
                             f"{len(plan.joint_names)}-joint anchor via MoveGroup")
            return child
        self._require_move_group()
        goal = self._build_move_group_goal(
            plan.planning_group, plan.joint_names, plan.target_positions,
            plan.velocity_scaling, plan.acceleration_scaling,
        )
        child = _ArmChild(action_no, plan.action_id)
        child.attach_goal_future(self._move_group_client.send_goal_async(goal))
        return child

    def _build_execute_trajectory_goal(
        self, plan: RecordedTrajectoryPlan
    ) -> ExecuteTrajectory.Goal:
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
        return goal

    def _dispatch_execute_trajectory(self, action_no, plan: RecordedTrajectoryPlan) -> _Child:
        """Approach the replay's first waypoint (planned), then replay it.

        A taught trajectory is recorded from wherever the arm happened to be, which
        is never bit-exact where the graph parks it beforehand — the preceding
        anchor is a staging pose, not a guaranteed start state. ``ExecuteTrajectory``
        only *executes*: it does not plan, and MoveIt rejects a replay whose first
        point deviates from the current state by more than
        ``trajectory_execution.allowed_start_tolerance``.

        So the offset is resolved the same way every other gap in the graph is: by
        a collision-aware ``MoveGroup`` plan to the trajectory's own first waypoint,
        run as phase 1 of the same action. That keeps taught data untouched and
        works from wherever the arm actually is, instead of assuming it is on the
        anchor. The approach runs at ``recorded_approach_scaling``, deliberately
        independent of the action's ``velocity_scaling`` (which for a recorded
        action is a replay time-stretch, not a planner limit).
        """
        if self.arm_dry_run:
            child = _ArmChild(action_no, plan.action_id)
            child.mark(True, f"dry_run: would approach waypoint 0 then replay "
                             f"{len(plan.points)} waypoint(s) on group "
                             f"'{plan.planning_group}' via MoveGroup+ExecuteTrajectory")
            return child
        self._require_move_group()
        exec_client = self._execute_trajectory_client
        if not exec_client.wait_for_server(timeout_sec=self.goal_accept_timeout):
            raise DispatchError(
                "execute_trajectory action "
                f"'{self.arm_planner.config.execute_trajectory_action}' not available"
            )

        approach_goal = self._build_move_group_goal(
            plan.planning_group, plan.joint_names, plan.points[0].positions,
            self.recorded_approach_scaling, self.recorded_approach_scaling,
        )
        child = _PhasedArmChild(
            action_no,
            plan.action_id,
            phases=[
                lambda: self._move_group_client.send_goal_async(approach_goal),
                lambda: exec_client.send_goal_async(
                    self._build_execute_trajectory_goal(plan)
                ),
            ],
            labels=["approach to waypoint 0", "recorded replay"],
        )
        child.start()
        return child

    # --- sync-group merge (Case 4) -------------------------------------------

    def _dispatch_units(self, batch, activity_id) -> list[_Child]:
        """Turn a scheduler batch into children, merging synced per-arm pairs.

        Two separate goals to move_group serialize, so per-arm Trajectory actions
        that share a ``sync_flag`` are merged into one ``both_arms`` goal here (a
        single MoveIt trajectory = genuine time-sync). Anything not mergeable
        (mixed kinds, a hand in the group, uneven coverage, not taught) falls back
        to independent dispatch — identical to the previous behaviour.
        """
        by_flag: dict[int, list] = {}
        singles: list = []
        for item in batch:
            if item.sync_flag:
                by_flag.setdefault(item.sync_flag, []).append(item)
            else:
                singles.append(item)

        children: list[_Child] = []
        for members in by_flag.values():
            merged = None
            if len(members) >= 2:
                merged = self._try_merge_sync_group(members, activity_id)
            if merged is not None:
                children.append(merged)
            else:
                children.extend(
                    self._dispatch(m.action_no, m.action_id, activity_id) for m in members
                )
        children.extend(
            self._dispatch(m.action_no, m.action_id, activity_id) for m in singles
        )
        return children

    def _try_merge_sync_group(self, members, activity_id) -> _Child | None:
        """Merge a synced left_arm+right_arm Trajectory pair into one both_arms goal.

        Returns None (caller falls back to per-action dispatch) whenever the group
        is not exactly the two arm sides, is not all Trajectory, has no both_arms
        group configured, is not yet taught, or the plans cannot be merged.
        """
        if len(members) != 2:
            return None
        actions = [self.catalogue.get_action_detail(m.action_id) for m in members]
        if not all(a.actiontype_id == ACTIONTYPE_TRAJECTORY for a in actions):
            return None
        if {a.robot_id for a in actions} != {"left_arm", "right_arm"}:
            return None
        group = self.arm_planner.config.groups.get("both_arms")
        if group is None:
            return None
        try:
            plans = [self.arm_planner.plan(a) for a in actions]
        except (NotTaughtError, ArmConfigError):
            return None
        merged_id = "+".join(a.action_id for a in actions)
        try:
            merged_plan = merge_arm_plans(plans, group, merged_id)
        except PlanMergeError as exc:
            self.get_logger().warn(f"sync-merge fallback for {merged_id}: {exc}")
            return None

        rep_no = members[0].action_no
        if isinstance(merged_plan, MoveGroupPlan):
            child = self._dispatch_move_group(rep_no, merged_plan)
        else:
            child = self._dispatch_execute_trajectory(rep_no, merged_plan)
        child.action_id = merged_id
        child.action_nos = [m.action_no for m in members]
        self.get_logger().info(
            f"-> merged synced {merged_id} into one both_arms goal (genuine dual-arm sync)"
        )
        return child

    def _child_robot_id(self, child: _Child) -> str:
        if len(child.action_nos) > 1:
            return "both_arms"
        action = self.catalogue.actions.get(child.action_id)
        return action.robot_id if action else ""

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
        self._open_hand_windows.clear()

        while rclpy.ok() and not scheduler.is_complete(completed):
            if goal_handle.is_cancel_requested:
                self._cancel_children(running)
                self._resume_all_hand_windows()
                result.success = False
                result.message = "canceled"
                result.completed_nodes = len(completed)
                self._event("failed", activity_id=activity_id, message="canceled")
                goal_handle.canceled()
                return result

            # dispatch any newly ready batch (synced per-arm pairs merged to one goal)
            try:
                units = self._dispatch_units(
                    scheduler.next_batch(completed, set(running)), activity_id
                )
            except (DispatchError, KeyError) as exc:
                return self._abort(
                    goal_handle, result, running, activity_id, "", len(completed), str(exc),
                )
            for child in units:
                for covered_no in child.action_nos:
                    running[covered_no] = child
                self.get_logger().info(f"-> dispatch {child.action_id} ({child.action_nos})")
                self._event("info", activity_id=activity_id, action_id=child.action_id,
                            robot_id=self._child_robot_id(child),
                            state="running", message="dispatched")
                self._publish_feedback(goal_handle, child.action_no, child.action_id,
                                       "running", len(completed), total)

            # poll running children (a merged child appears under both action_nos)
            polled: set[int] = set()
            for action_no in list(running):
                child = running.get(action_no)
                if child is None or id(child) in polled:
                    continue
                polled.add(id(child))
                child.poll()
                if not child.done:
                    continue
                for covered_no in child.action_nos:
                    running.pop(covered_no, None)
                if child.success:
                    for covered_no in child.action_nos:
                        completed.add(covered_no)
                    if isinstance(child, _HandChild):
                        # Hand action done: reopen the arm side it quiesced.
                        self._resume_hand_window(child.side)
                    self._event("completed", activity_id=activity_id,
                                action_id=child.action_id, state="completed",
                                message=child.message)
                    self._publish_feedback(goal_handle, child.action_no, child.action_id,
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

        self._resume_all_hand_windows()
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
        self._resume_all_hand_windows()
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
