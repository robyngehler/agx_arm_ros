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
import signal
import threading
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.action import PerformActivity, PerformAction
from agx_arm_msgs.msg import AgxUnitSafety, RobotEvent

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
from agx_arm_coordination.graph_model import (
    ACTIONTYPE_TRAJECTORY,
    Scheduler,
    robot_units,
)
from agx_arm_coordination.motion_registry import (
    assert_matches_topology,
    bus_topology,
    handshake_required,
)
from agx_arm_coordination.performer import KIND_ARM, KIND_HAND, RoutingError, route
from agx_arm_coordination.unit_activity import UnitActivity


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
        # Set by the activity loop so a completing goal wakes it immediately,
        # rather than being noticed on the next fixed-rate sweep.
        self._notify = None

    def set_notify(self, notify) -> None:
        self._notify = notify
        self._arm_notify(self._goal_future)
        self._arm_notify(self._result_future)

    def _arm_notify(self, future) -> None:
        """Wake the activity loop when this future resolves.

        Best-effort on purpose: a torn-down context can refuse the callback, and
        the loop's watchdog tick still finds the completion. Losing a wakeup
        costs latency; raising here would cost the activity.
        """
        if self._notify is None or future is None:
            return
        try:
            future.add_done_callback(lambda _f: self._notify())
        except Exception:
            pass

    def attach_goal_future(self, future) -> None:
        self._goal_future = future
        self._arm_notify(future)

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
            self._arm_notify(self._result_future)
            return
        if self._result_future is not None and self._result_future.done():
            wrapper = self._result_future.result()
            self._interpret_result(wrapper)

    def _interpret_result(self, wrapper) -> None:  # overridden per child kind
        self.mark(False, "no result interpreter")

    def request_cancel(self) -> None:
        if self._goal_handle is None or self.done:
            return
        try:
            self._goal_handle.cancel_goal_async()
        except Exception:
            # Cancelling is a stop path: a torn-down context or an already-closed
            # goal must not stop the remaining children from being cancelled.
            pass


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
        # verified hold (prepare_hand_window) and handed back afterwards
        # (resume_arm_control), so the hand actually owns the bus. On a
        # dedicated-per-device topology there is nothing to hand off.
        self.declare_parameter("arm_service_template", "/{side}_arm")
        # MIT controller namespace per side, used only by the safe-stop path
        # (cancel_trajectory + hold_current).
        self.declare_parameter("mit_controller_template", "/{side}_arm/mit_controller")
        self.declare_parameter("stop_service_timeout_sec", 3.0)
        # Derived, not configured: the same declared topology decides the
        # resource table below. Two independent switches is how a stack came up
        # quiescing an arm for every hand motion that did not need it.
        self.declare_parameter("handoff_enabled", handshake_required())
        self.declare_parameter("handoff_timeout_sec", 5.0)
        self.declare_parameter("arm_dry_run", False)
        self.declare_parameter("poll_period_sec", 0.05)
        # The activity loop is woken by whatever it is waiting for — a child
        # goal resolving, a cancel, a stop — so this is only a watchdog tick,
        # not the rate at which completion is noticed. It exists so a lost
        # wakeup costs latency instead of the activity.
        self.declare_parameter("watchdog_period_sec", 0.5)
        # How long cancellation waits for children to actually report done
        # before it reports which ones did not.
        self.declare_parameter("cleanup_timeout_sec", 3.0)
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
        self.mit_controller_template = str(self.get_parameter("mit_controller_template").value)
        self.stop_service_timeout = float(self.get_parameter("stop_service_timeout_sec").value)
        # Compatibility input only. A value that disagrees with the declared
        # topology is refused rather than obeyed — see assert_matches_topology.
        self.handoff_enabled = assert_matches_topology(
            "handoff_enabled", bool(self.get_parameter("handoff_enabled").value)
        )
        # One declared fact behind both: whether same-side arm and hand are one
        # schedulable resource, and whether a hand action has to quiesce the arm.
        self.bus_topology = bus_topology()
        self.robot_units = robot_units(self.bus_topology)
        self.handoff_timeout = float(self.get_parameter("handoff_timeout_sec").value)
        self.arm_dry_run = bool(self.get_parameter("arm_dry_run").value)
        self.poll_period = float(self.get_parameter("poll_period_sec").value)
        self.watchdog_period = max(
            0.01, float(self.get_parameter("watchdog_period_sec").value)
        )
        self.cleanup_timeout = max(
            0.0, float(self.get_parameter("cleanup_timeout_sec").value)
        )
        # Set by anything the activity loop would otherwise have to poll for.
        self._progress = threading.Event()
        self.goal_accept_timeout = float(self.get_parameter("goal_accept_timeout_sec").value)
        self.joint_goal_tolerance = float(self.get_parameter("joint_goal_tolerance_rad").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time_sec").value)
        self.recorded_approach_scaling = min(
            max(float(self.get_parameter("recorded_approach_scaling").value), 1e-3), 1.0
        )

        self.catalogue = ActivityCatalogue.from_config_dir(config_dir, self.robot_units)
        arm_config_path = config_dir / "arm_config.yaml"
        self.arm_planner = ArmTrajectoryPlanner(ArmConfig.from_file(arm_config_path))

        self._cb_group = ReentrantCallbackGroup()
        self.event_pub = self.create_publisher(RobotEvent, "events", 10)

        # Child action clients (created once, reused per activity run).
        self._hand_clients: dict[str, ActionClient] = {}
        self._prepare_clients: dict[str, object] = {}
        self._resume_clients: dict[str, object] = {}
        self._cancel_traj_clients: dict[str, object] = {}
        self._hold_clients: dict[str, object] = {}
        self._estop_clients: dict[str, object] = {}
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
            # Safe-stop path. Cancelling the MoveIt goal is the primary stop;
            # these pin the arm afterwards so "stopped" means held, not coasting.
            mit_ns = self.mit_controller_template.format(side=side)
            self._cancel_traj_clients[side] = self.create_client(
                Empty, f"{mit_ns}/cancel_trajectory", callback_group=self._cb_group
            )
            self._hold_clients[side] = self.create_client(
                Empty, f"{mit_ns}/hold_current", callback_group=self._cb_group
            )
            self._estop_clients[side] = self.create_client(
                Trigger, f"{arm_ns}/emergency_stop", callback_group=self._cb_group
            )
        # Sides whose arm is currently quiesced for a hand window (prepared but
        # not yet resumed), so any exit path can close them again.
        self._open_hand_windows: set[str] = set()

        # --- cooperative stop ------------------------------------------------
        # Ctrl+C must reach the hardware, not just this process. rclpy's default
        # SIGINT handler tears the context down immediately, which would strand a
        # running MoveIt trajectory: the goal keeps executing with no client left
        # to cancel it. main() therefore takes the signal and calls request_stop,
        # and the activity loop unwinds through the normal abort path (cancel
        # children -> resume hand windows -> pin the arms).
        self._stop_lock = threading.Lock()
        self._stop_requested = False
        self._stop_reason = ""
        # One authoritative answer to "may another activity start?". Replaces a
        # plain running-flag that nothing consulted before dispatching. Also
        # carries the unit-safety liveness rule: losing the safety writer never
        # stops a running activity, but it must stop a new one from starting.
        self.declare_parameter("require_unit_safety", True)
        self.declare_parameter("unit_safety_timeout_s", 6.0)
        self._unit_activity = UnitActivity(
            require_unit_safety=bool(
                self.get_parameter("require_unit_safety").value
            ),
            unit_safety_timeout_s=float(
                self.get_parameter("unit_safety_timeout_s").value
            ),
        )
        # Every message counts as liveness, heartbeat included: the latched
        # value outlives the writer, so a stale generation and a live one are
        # otherwise indistinguishable.
        self.create_subscription(
            AgxUnitSafety, "/unit_safety", self._unit_safety_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._cb_group,
        )
        self._shutdown_event = threading.Event()
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
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"Coordinator up: config_dir={config_dir}, "
            f"activities={self.catalogue.available_activities()}, "
            f"arm_groups={sorted(self.arm_planner.config.groups)}, "
            f"arm_dry_run={self.arm_dry_run}, "
            # Named at startup because it silently changes what may run at once:
            # under dedicated_per_device a hand action no longer waits for its
            # own arm, and no other line of output would say so.
            f"bus_topology={self.bus_topology} "
            f"(same-side arm and hand "
            f"{'serialized' if self.handoff_enabled else 'may overlap'}, "
            f"arm handoff {'on' if self.handoff_enabled else 'off'})"
        )

    def _unit_safety_callback(self, msg: AgxUnitSafety) -> None:
        """Track the unit's safety generation for activity admission."""
        self._unit_activity.observe_unit_safety(
            epoch=int(msg.epoch), stopped=bool(msg.stopped), reason=msg.reason
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

    # --- cooperative stop ----------------------------------------------------

    @property
    def stop_requested(self) -> bool:
        with self._stop_lock:
            return self._stop_requested

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Accept the cancel and wake the activity loop to act on it now.

        Without the wake, cancellation would wait out the loop's watchdog tick.
        Making that tick low-rate is only safe because every reason to look —
        a child finishing, a cancel, a stop — sets the same event.
        """
        self._progress.set()
        return CancelResponse.ACCEPT

    def request_stop(self, reason: str) -> None:
        """Ask a running activity to unwind and stop the hardware.

        Signal-handler safe: it only flips a flag and (when nothing is running)
        releases main(). The actual cancelling happens on the activity thread,
        which is still spinning, so the cancel goals can still be delivered.
        """
        with self._stop_lock:
            if self._stop_requested:
                return
            self._stop_requested = True
            self._stop_reason = reason
        # Refuses further activities and tells us whether one still has to
        # unwind before main() may release.
        running = self._unit_activity.begin_stop(reason)
        # Same reason as _on_cancel: the unwind must not wait for a tick.
        self._progress.set()
        self.get_logger().warn(
            f"stop requested ({reason}); "
            + ("unwinding the running activity" if running else "no activity running")
        )
        if not running:
            self._shutdown_event.set()

    def wait_for_shutdown(self) -> None:
        """Block main() until a requested stop has been carried out.

        Polled rather than an unbounded wait so the process stays responsive to
        the signal handler (which only ever runs on the main thread).
        """
        while not self._shutdown_event.wait(0.2):
            pass

    def _call_empty_sync(self, client, label: str) -> tuple[bool, str]:
        """Call an Empty service and wait for it (bounded); never raises."""
        try:
            if not client.wait_for_service(timeout_sec=self.stop_service_timeout):
                return False, f"{label}: service unavailable"
            future = client.call_async(Empty.Request())
            deadline = time.monotonic() + self.stop_service_timeout
            while not future.done():
                if time.monotonic() > deadline:
                    return False, f"{label}: timed out"
                time.sleep(self.poll_period)
            return True, f"{label}: ok"
        except Exception as exc:  # a stop path must never raise
            return False, f"{label}: {exc}"

    def _sides_for_robot(self, robot_id: str) -> set[str]:
        if robot_id == "both_arms":
            return {"left", "right"}
        if robot_id in ("left_arm", "right_arm"):
            return {robot_id.split("_", 1)[0]}
        return set()

    def safe_stop_arms(self, sides: set[str], reason: str) -> None:
        """Pin the given arm sides where they stand (best effort, bounded).

        Cancelling the MoveIt goal is what actually stops the motion; this runs
        after it so "stopped" means *held* rather than left at whatever the last
        streamed MIT command was. The Nero firmware has no MIT command watchdog —
        silence is not a safe state — so an explicit hold is the difference
        between stopping and merely going quiet.

        Best effort by design: a missing service is logged, not escalated. The
        escalation to ``emergency_stop`` is deliberate and separate
        (``emergency_stop_all``, second Ctrl+C).
        """
        for side in sorted(sides):
            for label, client in (
                ("cancel_trajectory", self._cancel_traj_clients[side]),
                ("hold_current", self._hold_clients[side]),
            ):
                ok, msg = self._call_empty_sync(client, f"{label}[{side}]")
                if ok:
                    self.get_logger().info(f"safe stop ({reason}): {msg}")
                else:
                    self.get_logger().warn(f"safe stop ({reason}): {msg}")

    def emergency_stop_all(self) -> None:
        """Last-resort stop on every side: damped zero, hold, electronic e-stop.

        Only used on an explicit second interrupt. The physical e-stop remains the
        only guaranteed stop while the arm firmware has no command watchdog.

        The driver answers with three outcomes, not two: a stop confirmed in
        feedback, a stop contradicted by feedback, and a stop that was commanded
        while feedback could not answer the question at all. The last two are
        different diagnoses but the same operational situation — an arm that must
        be assumed to be moving — so they are collapsed into one unmissable
        instruction here rather than left as per-side lines that scroll past.
        """
        unconfirmed: list[str] = []
        for side in ("left", "right"):
            ok, msg = self._call_trigger_sync(
                self._estop_clients[side], f"emergency_stop[{side}]"
            )
            if ok:
                self.get_logger().info(f"emergency stop [{side}]: {msg}")
            else:
                unconfirmed.append(side)
                self.get_logger().error(f"emergency stop [{side}]: {msg}")
        if unconfirmed:
            self.get_logger().error(
                "PHYSICAL E-STOP REQUIRED — software stop not confirmed on "
                f"{', '.join(unconfirmed)}. A stop that cannot be verified is not "
                "a stop: treat these arms as still in motion."
            )

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
        """Close the hand window on ``side``, handing the arm back (best-effort).

        Closing the window is what resumes the arm: `resume_arm_control` reopens
        the arm's MIT gate and restores its feedback push. The two directions
        read confusingly close together, so state which one this is — the window
        closes, the arm resumes.
        """
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
        sharing a ``sync_flag`` must become one ``both_arms`` goal — a single
        MoveIt trajectory is the only genuine time-sync available here.

        **Merge or fail.** If two arm trajectories are synchronized and cannot be
        merged, this raises rather than dispatching them independently. The old
        fallback was silent and produced the opposite of what was asked for: two
        serialized arm motions presented as a synchronized pair, with nothing in
        the log to say the synchronization had been dropped.

        Members that were never mergeable in the first place — a hand beside an
        arm, a single-member group — are dispatched independently, which is not a
        downgrade: on dedicated buses those devices genuinely run in parallel.
        """
        by_flag: dict[int, list] = {}
        singles: list = []
        for item in batch:
            if item.sync_flag:
                by_flag.setdefault(item.sync_flag, []).append(item)
            else:
                singles.append(item)

        children: list[_Child] = []
        for flag in sorted(by_flag):
            members = by_flag[flag]
            arm_traj = [m for m in members if self._is_arm_trajectory(m.action_id)]
            if len(arm_traj) >= 2:
                merged = self._try_merge_sync_group(arm_traj, activity_id)
                if merged is None:
                    names = ", ".join(m.action_id for m in arm_traj)
                    raise DispatchError(
                        f"sync_flag {flag}: arm trajectories [{names}] are "
                        "synchronized but could not be merged into one both_arms "
                        "goal. Dispatching them separately would serialize them "
                        "and silently drop the synchronization, so the activity "
                        "fails instead"
                    )
                children.append(merged)
                rest = [m for m in members if m not in arm_traj]
            else:
                rest = members
            children.extend(
                self._dispatch(m.action_no, m.action_id, activity_id) for m in rest
            )
        children.extend(
            self._dispatch(m.action_no, m.action_id, activity_id) for m in singles
        )
        return children

    def _is_arm_trajectory(self, action_id: str) -> bool:
        """True for a per-arm Trajectory action — the only mergeable shape."""
        try:
            action = self.catalogue.get_action_detail(action_id)
        except KeyError:
            return False
        return (
            action.actiontype_id == ACTIONTYPE_TRAJECTORY
            and action.robot_id in ("left_arm", "right_arm")
        )

    def _try_merge_sync_group(self, members, activity_id) -> _Child | None:
        """Merge a synced left_arm+right_arm Trajectory pair into one both_arms goal.

        Returns None whenever the group is not exactly the two arm sides, is not
        all Trajectory, has no both_arms group configured, is not yet taught, or
        the plans cannot be merged.

        None is not a fallback. The caller raises `DispatchError` on it, because
        dispatching a synchronized pair separately serializes it and drops the
        synchronization silently — the opposite of what was asked for, presented
        as success. This docstring used to say the caller falls back to
        per-action dispatch, which stopped being true when the merge was made
        strict.
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
            self.get_logger().warn(
                f"sync-merge failed for {merged_id}: {exc}; the activity will "
                "fail rather than run the pair unsynchronized"
            )
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

    def _on_goal(self, request) -> GoalResponse:
        """Refuse a second activity at the door instead of executing it.

        Every goal used to be accepted unconditionally, and the callback group
        is reentrant, so two overlapping goals would both have run — against the
        same arms. This is the cheap refusal; :meth:`_execute` still claims the
        unit authoritatively, because two goals can pass this check at once.
        """
        admission = self._unit_activity.can_accept(request.activity_id)
        if admission.accepted:
            return GoalResponse.ACCEPT
        self.get_logger().warn(f"rejecting activity goal: {admission.detail}")
        self._event(
            "rejected",
            activity_id=request.activity_id,
            state=admission.reason.value,
            message=admission.detail,
        )
        return GoalResponse.REJECT

    def _execute(self, goal_handle) -> PerformActivity.Result:
        activity_id = goal_handle.request.activity_id
        admission = self._unit_activity.try_claim(activity_id)
        if not admission.accepted:
            result = PerformActivity.Result()
            result.success = False
            result.message = admission.detail
            self.get_logger().warn(f"refusing to execute: {admission.detail}")
            self._event(
                "rejected",
                activity_id=activity_id,
                state=admission.reason.value,
                message=admission.detail,
            )
            goal_handle.abort()
            return result
        try:
            return self._execute_activity(goal_handle)
        finally:
            # Authority is released the same way on every exit — success,
            # failure, cancellation, and an exception nobody predicted. Any hand
            # window still open is CLOSED here — the arm gets its bus and its MIT
            # gate back — because a window left open keeps that gate shut and the
            # next activity would find an arm that silently refuses to move.
            try:
                self._resume_all_hand_windows()
            except Exception as exc:
                self.get_logger().error(f"closing hand windows failed: {exc}")
            self._unit_activity.release(activity_id)
            if self.stop_requested:
                self._shutdown_event.set()

    def _execute_activity(self, goal_handle) -> PerformActivity.Result:
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
        scheduler = Scheduler(graph, self.catalogue.actions, self.robot_units)
        total = len(graph.nodes)
        result.total_nodes = total
        self.get_logger().info(f"running activity '{activity_id}' ({total} nodes)")
        self._event("started", activity_id=activity_id, message=f"{total} nodes")

        completed: set[int] = set()
        running: dict[int, _Child] = {}
        self._open_hand_windows.clear()

        while rclpy.ok() and not scheduler.is_complete(completed):
            # Cleared before the work, never after: a notification arriving
            # while this pass runs then leaves the event set, and the wait at
            # the bottom returns at once instead of sleeping through news that
            # already happened.
            self._progress.clear()

            if goal_handle.is_cancel_requested or self.stop_requested:
                reason = self._stop_reason if self.stop_requested else "canceled"
                self._stop_running(running, reason)
                result.success = False
                result.message = reason
                result.completed_nodes = len(completed)
                self._event("failed", activity_id=activity_id, message=reason)
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
                child.set_notify(self._progress.set)
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
                        # Hand action done: close the window, hand the arm back.
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

            # Woken by a child result, a cancel, or a stop; the timeout is the
            # watchdog that keeps a missed wakeup from stalling the activity.
            self._progress.wait(self.watchdog_period)

        if not scheduler.is_complete(completed):
            # rclpy went down (context shutdown) while nodes were still pending.
            # Never report success here: the arm may be mid-trajectory, and a
            # silently "successful" result would hide that from the caller.
            return self._abort(
                goal_handle, result, running, activity_id, "", len(completed),
                "coordinator shut down before the activity completed",
            )

        self._resume_all_hand_windows()
        result.success = True
        result.message = f"activity '{activity_id}' complete"
        result.completed_nodes = len(completed)
        self.get_logger().info(result.message)
        self._event("completed", activity_id=activity_id, message=result.message)
        goal_handle.succeed()
        return result

    def _sides_in_flight(self, running: dict[int, _Child]) -> set[str]:
        """Arm sides with a goal actually in flight — the ones a stop must pin."""
        sides: set[str] = set()
        for child in running.values():
            if isinstance(child, _ArmChild):
                sides |= self._sides_for_robot(self._child_robot_id(child))
        return sides

    def _stop_running(self, running: dict[int, _Child], reason: str) -> None:
        """Bring everything in flight to a held stop, in the only order that works.

        1. cancel the children — this is what actually stops the motion;
        2. close any hand window, because while one is open the arm's MIT gate is
           closed and a hold command would be dropped before reaching the arm;
        3. pin the arms that were moving.
        """
        sides = self._sides_in_flight(running)
        self._cancel_children(running)
        self._resume_all_hand_windows()
        if sides:
            self.safe_stop_arms(sides, reason)

    def _abort(self, goal_handle, result, running, activity_id, failed_action_id,
               completed_count, message) -> PerformActivity.Result:
        self.get_logger().error(f"aborting '{activity_id}': {message}")
        self._stop_running(running, f"abort: {message}")
        result.success = False
        result.message = message
        result.failed_action_id = failed_action_id
        result.completed_nodes = completed_count
        self._event("failed", activity_id=activity_id, action_id=failed_action_id,
                    message=message)
        goal_handle.abort()
        return result

    def _cancel_children(self, running: dict[int, _Child]) -> None:
        """Cancel every child and wait, bounded, for it to actually stop.

        Firing the cancels and clearing the dict reported cleanup as finished
        the instant it was requested. The children were still executing on
        hardware, so "activity aborted" could be published while an arm was
        mid-trajectory, and nothing recorded that the two disagreed.

        The wait is bounded because a child that never answers must not hold the
        unit forever — but which children those were is a structured result, not
        a silence.
        """
        children = []
        seen: set[int] = set()
        for child in running.values():
            if id(child) in seen:
                continue
            seen.add(id(child))
            children.append(child)
            child.request_cancel()

        deadline = time.monotonic() + self.cleanup_timeout
        while children and time.monotonic() < deadline:
            pending = []
            for child in children:
                child.poll()
                if not child.done:
                    pending.append(child)
            if not pending:
                children = []
                break
            children = pending
            self._progress.wait(0.05)
            self._progress.clear()

        if children:
            unstopped = ", ".join(sorted(c.action_id for c in children))
            self.get_logger().error(
                f"cleanup deadline: {len(children)} child(ren) did not confirm "
                f"cancellation within {self.cleanup_timeout:.1f}s [{unstopped}]. "
                "The activity is released, but the hardware may still be moving."
            )
            self._event(
                "failed", state="cleanup_timeout",
                message=f"children did not stop: {unstopped}",
            )
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

    # Own the interrupt instead of letting rclpy tear the context down on the
    # spot. A coordinator that just exits leaves its MoveIt trajectory and hand
    # goals running on hardware with nobody left to cancel them, so the first
    # interrupt asks the activity to unwind (cancel -> close hand windows -> pin
    # the arms) while the graph is still alive to carry those messages. A second
    # interrupt means the operator wants out now: escalate to the emergency stop
    # and hand the signal back to the default handler.
    previous_sigint = signal.getsignal(signal.SIGINT)

    def _on_interrupt(signum, frame):
        if node.stop_requested:
            node.get_logger().error("second interrupt: escalating to emergency stop")
            node.emergency_stop_all()
            signal.signal(signal.SIGINT, previous_sigint)
            raise KeyboardInterrupt
        node.request_stop("interrupt (Ctrl+C)")

    signal.signal(signal.SIGINT, _on_interrupt)
    signal.signal(signal.SIGTERM, lambda signum, frame: node.request_stop("SIGTERM"))

    spin_thread = threading.Thread(target=executor.spin, name="coordinator_spin", daemon=True)
    spin_thread.start()
    try:
        # Blocks until request_stop() has been honoured: either nothing was
        # running, or the activity thread finished unwinding.
        node.wait_for_shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
