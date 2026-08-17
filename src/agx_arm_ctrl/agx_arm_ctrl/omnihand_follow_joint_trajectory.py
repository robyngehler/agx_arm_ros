from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.msg import (
    AuthorizedJointTrajectory,
    DeviceCommandStamp,
    OmniHandStatus,
)
from agx_arm_msgs.srv import ClaimDevice

from agx_arm_ctrl.motion_registry import assert_matches_topology, handshake_required
from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, get_hand_model
# Shared, model-aware joint naming — do NOT keep a second JOINT_SUFFIXES copy here
# (proposal §6/§11.3): a stale O10 list would flag every Pro-only joint as unknown.
from agx_arm_ctrl.omnihand_bridge_node import HAND_CLAIM_SERVICE, build_joint_names


def _trajectory_duration_s(msg: JointTrajectory) -> float:
    if not msg.points:
        return 0.0
    last_point = msg.points[-1].time_from_start
    return float(last_point.sec) + float(last_point.nanosec) / 1e9


class OmniHandFollowJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("omnihand_follow_joint_trajectory")

        self.declare_parameter("omnihand_type", "right")
        self.declare_parameter("hand_model", DEFAULT_HAND_MODEL)
        self.declare_parameter(
            "action_name",
            "right_omnihand_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "trajectory_topic",
            "control/omnihand/joint_trajectory",
        )
        self.declare_parameter(
            "feedback_topic",
            "feedback/omnihand/joint_states",
        )
        self.declare_parameter(
            "status_topic",
            "feedback/omnihand/status",
        )
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("goal_margin_s", 0.25)
        # Elapsed trajectory time does not mean the hand got the target: on the
        # shared bus the bridge re-sends until a readback confirms it, and that
        # loop outlives a short trajectory. Hold the window (and the goal) open
        # until the bridge has actually decided, bounded by this.
        self.declare_parameter("delivery_timeout_s", 4.0)
        # feedback/omnihand/joint_states is republished from cache at pub_rate
        # even while the backend is faulted, so a fresh header stamp proves
        # nothing. A real SDK readback must be no older than this.
        self.declare_parameter("readback_max_age_s", 1.5)
        # Step-and-settle handshake: quiesce the same-side arm into a verified
        # hold for the duration of a hand trajectory so MoveIt hand execution
        # owns the shared side bus instead of losing arbitration under arm MIT.
        #
        # Only the shared-bus topology needs it, and the default is derived from
        # the declared one rather than typed here. Hardcoded `True` meant that
        # anything starting this node outside the launch files — a test double, a
        # bare `ros2 run`, a measurement harness — quiesced an arm for a hand on
        # its own bus, and did it silently.
        self.declare_parameter("handshake_enabled", handshake_required())
        self.declare_parameter("arm_service_ns", "")
        self.declare_parameter("handshake_timeout_s", 5.0)

        hand_side = str(self.get_parameter("omnihand_type").value)
        if hand_side not in ("left", "right"):
            raise ValueError("omnihand_type must be 'left' or 'right'")

        self.hand_model = get_hand_model(str(self.get_parameter("hand_model").value))
        self.joint_names = build_joint_names(hand_side, self.hand_model)
        action_name = str(self.get_parameter("action_name").value)
        trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.goal_margin_s = float(self.get_parameter("goal_margin_s").value)
        self.delivery_timeout_s = max(
            0.0, float(self.get_parameter("delivery_timeout_s").value)
        )
        self.readback_max_age_s = max(
            0.0, float(self.get_parameter("readback_max_age_s").value)
        )
        # Compatibility input only; a launch that passes hand_bus:=shared on a
        # dedicated registry is refused here rather than quiescing an arm that
        # shares no bus with this hand.
        self.handshake_enabled = assert_matches_topology(
            "handshake_enabled", bool(self.get_parameter("handshake_enabled").value)
        )
        self.handshake_timeout_s = float(self.get_parameter("handshake_timeout_s").value)
        arm_ns = str(self.get_parameter("arm_service_ns").value).strip("/")

        self.feedback_positions: dict[str, float] = {}
        self.feedback_velocities: dict[str, float] = {}
        self.last_feedback_time = 0.0
        self._window_open = False

        self.last_status: OmniHandStatus | None = None
        self.last_status_monotonic = 0.0

        self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        # The authority-carrying surface (4D). The external interface is still
        # standard FollowJointTrajectory; this is where the accepted goal is
        # bound to the claim it runs under before it reaches the hardware.
        self.authorized_pub = self.create_publisher(
            AuthorizedJointTrajectory, "control/omnihand/authorized_trajectory", 10
        )
        # Both generations come from the claim response, so the first command
        # after a claim need not wait for the authority topic to catch up.
        self._device_epoch = 0
        self._unit_safety_epoch = 0
        self._sequence = 0
        self.create_subscription(JointState, feedback_topic, self._feedback_callback, 20)
        self.create_subscription(
            OmniHandStatus, self.status_topic, self._status_callback, 10
        )

        # Reentrant group so the handshake service futures are serviced while the
        # action execute callback is spinning on them.
        self._cb_group = ReentrantCallbackGroup()
        prepare_name = f"/{arm_ns}/prepare_hand_window" if arm_ns else "prepare_hand_window"
        resume_name = f"/{arm_ns}/resume_arm_control" if arm_ns else "resume_arm_control"
        # The owner_id declares the motion primitive, then the node. The bridge
        # tells the two production primitives apart by it, and uses the node half
        # to notice when a commander has died still holding a claim.
        self.owner_id = f"trajectory:{self.get_name()}"
        self.claim_service_name = HAND_CLAIM_SERVICE
        self.claim_client = self.create_client(
            ClaimDevice, self.claim_service_name, callback_group=self._cb_group
        )
        self.prepare_client = self.create_client(
            Trigger, prepare_name, callback_group=self._cb_group
        )
        self.resume_client = self.create_client(
            Trigger, resume_name, callback_group=self._cb_group
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

    def _feedback_callback(self, msg: JointState) -> None:
        if not msg.name:
            return
        self.feedback_positions.update(
            {name: float(value) for name, value in zip(msg.name, msg.position)}
        )
        self.feedback_velocities.update(
            {
                name: float(value)
                for name, value in zip(msg.name, msg.velocity or [0.0] * len(msg.name))
            }
        )
        self.last_feedback_time = time.monotonic()

    def _status_callback(self, msg: OmniHandStatus) -> None:
        self.last_status = msg
        self.last_status_monotonic = time.monotonic()

    def _has_fresh_feedback(self) -> bool:
        if self.last_feedback_time <= 0.0:
            return False
        return (time.monotonic() - self.last_feedback_time) <= self.feedback_timeout_s

    def _readback_is_live(self, status: OmniHandStatus) -> bool:
        """True when the hand itself answered recently, not just the cache."""
        age_s = float(status.joint_readback_age_s)
        if age_s < 0.0:
            return False
        return (age_s + (time.monotonic() - self.last_status_monotonic)) <= (
            self.readback_max_age_s
        )

    def _await_delivery(self, published_at: float) -> tuple[bool, str]:
        """Wait until the bridge has decided the fate of the published target.

        Only status samples received AFTER the publish are trusted: an older
        sample still describes the previous command. Falls back to plain
        feedback freshness when no status surface is present (older bridge or a
        rig without one), which is the pre-existing behavior.
        """
        if self.count_publishers(self.status_topic) == 0:
            return self._has_fresh_feedback(), "OmniHand feedback is stale"

        deadline = time.monotonic() + self.delivery_timeout_s
        while rclpy.ok():
            status = self.last_status
            if status is not None and self.last_status_monotonic > published_at:
                if not status.command_pending:
                    if status.command_delivery_failed:
                        return False, (
                            "bridge gave up on the hand target unverified "
                            f"after {status.command_attempts} attempts"
                        )
                    if not self._readback_is_live(status):
                        return False, (
                            "hand target reported delivered but no recent SDK "
                            f"readback backs it (age {status.joint_readback_age_s:.2f} s)"
                        )
                    return True, ""

            if time.monotonic() > deadline:
                attempts = status.command_attempts if status is not None else 0
                return False, (
                    "hand target still unverified after "
                    f"{self.delivery_timeout_s:.1f} s ({attempts} attempts)"
                )
            time.sleep(0.02)
        return False, "ROS shutdown while waiting for hand command delivery"

    def _validate_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.joint_names:
            raise ValueError("trajectory must declare joint_names")
        unknown_names = [name for name in msg.joint_names if name not in self.joint_names]
        if unknown_names:
            raise ValueError(
                "trajectory contains unknown OmniHand joints: " + ", ".join(unknown_names)
            )
        if len(set(msg.joint_names)) != len(msg.joint_names):
            raise ValueError("trajectory joint_names must be unique")
        if not msg.points:
            raise ValueError("trajectory must contain at least one point")
        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                raise ValueError("each trajectory point must match joint_names length")

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal):
        try:
            self._validate_trajectory(goal_request.trajectory)
        except ValueError as exc:
            self.get_logger().error(f"Rejected OmniHand FollowJointTrajectory goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _desired_point(self, trajectory: JointTrajectory) -> JointTrajectoryPoint:
        return trajectory.points[-1]

    def _actual_point(self, goal_joint_names: list[str]) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [self.feedback_positions.get(name, 0.0) for name in goal_joint_names]
        point.velocities = [self.feedback_velocities.get(name, 0.0) for name in goal_joint_names]
        point.effort = [0.0] * len(goal_joint_names)
        return point

    def _error_point(self, desired: JointTrajectoryPoint, goal_joint_names: list[str]) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [
            desired.positions[index] - self.feedback_positions.get(name, 0.0)
            for index, name in enumerate(goal_joint_names)
        ]
        point.velocities = [0.0] * len(goal_joint_names)
        point.effort = [0.0] * len(goal_joint_names)
        return point

    def _success_result(self) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _failed_result(self, code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _call_trigger(self, client, label: str):
        """Call a Trigger service. Returns (proceed, message).

        ``proceed`` is None when the service is absent (no arm to gate — proceed
        without a window), True on a verified success, False on a real failure.
        """
        if not client.wait_for_service(timeout_sec=self.handshake_timeout_s):
            return None, f"{label} unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.handshake_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{label} timed out"
            time.sleep(0.02)
        resp = future.result()
        if resp is None:
            return False, f"{label} returned no response"
        return bool(resp.success), resp.message or ""

    def _claim_hand(self) -> tuple[bool, str]:
        """Take the hand's device authority for the duration of a goal."""
        return self._call_claim(claim=True)

    def _release_hand(self) -> None:
        accepted, detail = self._call_claim(claim=False)
        if not accepted:
            self.get_logger().warn(f"releasing the hand failed: {detail}")

    def _call_claim(self, *, claim: bool) -> tuple[bool, str]:
        if not self.claim_client.wait_for_service(timeout_sec=self.handshake_timeout_s):
            return False, f"{self.claim_service_name} is not available"
        request = ClaimDevice.Request()
        request.owner_id = self.owner_id
        request.claim = claim
        future = self.claim_client.call_async(request)
        deadline = time.monotonic() + self.handshake_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{self.claim_service_name} did not answer"
            time.sleep(0.02)
        response = future.result()
        if response is None:
            return False, f"{self.claim_service_name} returned nothing"
        if response.accepted and claim:
            # A claim opens a new era for this device, so the sequence starts
            # again with it rather than carrying a watermark across owners.
            self._device_epoch = int(response.device_epoch)
            self._unit_safety_epoch = int(response.unit_safety_epoch)
            self._sequence = 0
        return bool(response.accepted), response.message or response.reason

    def _authority_stamp(self) -> DeviceCommandStamp:
        """The stamp for the next command issued under the current claim."""
        self._sequence += 1
        stamp = DeviceCommandStamp()
        stamp.owner_id = self.owner_id
        stamp.device_epoch = self._device_epoch
        stamp.unit_safety_epoch = self._unit_safety_epoch
        stamp.sequence = self._sequence
        return stamp

    def _open_hand_window(self) -> tuple[bool, str]:
        """Quiesce the same-side arm before commanding the hand.

        Tolerant of a hand-only bringup: if no prepare service is present there
        is no arm contending for the bus, so proceed without a window.
        """
        if not self.handshake_enabled:
            return True, "handshake disabled"
        ok, msg = self._call_trigger(self.prepare_client, "prepare_hand_window")
        if ok is None:
            self.get_logger().warn(
                f"no arm handshake ({msg}); commanding hand without quiescing an arm"
            )
            self._window_open = False
            return True, msg
        self._window_open = bool(ok)
        if ok:
            self.get_logger().info(f"hand window opened (arm quiesced): {msg}")
        return bool(ok), msg

    def _close_hand_window(self) -> None:
        if not self._window_open:
            return
        ok, msg = self._call_trigger(self.resume_client, "resume_arm_control")
        if not ok:
            self.get_logger().error(f"resume_arm_control failed: {msg}")
        self._window_open = False

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self._validate_trajectory(trajectory)
        # Own the hand for the trajectory. The bridge is fail-closed: an
        # unclaimed hand executes nothing, and a hand held by the reactive
        # primitive refuses a trajectory rather than letting the two interleave.
        # Taking the claim is therefore part of executing a goal, not setup.
        claimed, claim_msg = self._claim_hand()
        if not claimed:
            goal_handle.abort()
            return self._failed_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"could not take the hand: {claim_msg}",
            )
        # Own the shared side bus for the whole hand trajectory: quiesce the arm,
        # run, then always reopen it — so MoveIt hand execution is safe under the
        # always-on arm MIT without the caller needing to know the handshake.
        opened, msg = self._open_hand_window()
        if not opened:
            self._release_hand()
            goal_handle.abort()
            return self._failed_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"could not open hand window: {msg}",
            )
        try:
            return self._run_trajectory(goal_handle, trajectory)
        finally:
            self._close_hand_window()
            # Released on every exit, including an abort. A trajectory that ended
            # has no further claim on the hand, and holding one would block the
            # reactive primitive behind a goal that is already over.
            self._release_hand()

    def _run_trajectory(self, goal_handle, trajectory):
        # Bound to the claim this goal runs under, and carrying the trajectory
        # whole. The compatibility topic is published alongside so a subscriber
        # that has not migrated still sees the goal; it is the stamped message
        # that the bridge admits on.
        authorized = AuthorizedJointTrajectory()
        authorized.authority = self._authority_stamp()
        authorized.trajectory = trajectory
        self.authorized_pub.publish(authorized)
        self.trajectory_pub.publish(trajectory)
        published_at = time.monotonic()
        desired = self._desired_point(trajectory)
        goal_joint_names = list(trajectory.joint_names)

        # Close the window as soon as the hand command is verified DELIVERED,
        # not after the full (possibly multi-second) trajectory duration: the
        # OmniHand moves to the target autonomously once it has accepted the
        # command, so the arm's bus should come back the moment delivery is
        # confirmed. Waiting the whole duration kept the arm feedback silenced
        # far longer than needed, and under an always-on MIT hold that extra
        # silence is exactly what starved the co-located hand and tipped MoveIt
        # over its execution timeout. The window stays open across delivery
        # polling — the bridge's re-send loop needs the quiet bus — but not a
        # moment longer. `_await_delivery` polls up to `delivery_timeout_s`.
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = goal_joint_names
        feedback.desired = desired
        feedback.actual = self._actual_point(goal_joint_names)
        feedback.error = self._error_point(desired, goal_joint_names)
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._failed_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Goal canceled",
            )

        delivered, reason = self._await_delivery(published_at)
        if not delivered:
            goal_handle.abort()
            return self._failed_result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                f"OmniHand trajectory not delivered: {reason}",
            )
        goal_handle.succeed()
        return self._success_result()


def main() -> None:
    rclpy.init()
    node = OmniHandFollowJointTrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["main"]