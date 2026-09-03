#!/usr/bin/env python3
"""FollowJointTrajectory server for the AGX gripper.

The external interface is standard FollowJointTrajectory, so MoveIt needs to
know nothing about this device. Internally the accepted goal is bound to the
claim it runs under and handed to the arm driver, which owns the gripper's SDK
session and CAN socket; this node never touches the hardware.

No bus handshake: the gripper rides the arm's own bus and its transmits are
serialized onto the arm's worker, so there is no window to open. That is the
difference from the OmniHand, which has a bus of its own.

A goal succeeds on the jaws having settled after measurable travel, not on them
having reached the commanded width — a gripper closing on an object stops at the
object. Stillness alone is therefore not enough, because a gripper that never
received the command is still too.
"""
from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from agx_arm_msgs.msg import AuthorizedJointTrajectory, DeviceCommandStamp, GripperStatus
from agx_arm_msgs.srv import ClaimDevice

# Each finger travels half the opening, so either one determines the width.
FINGER_SUFFIXES = ("gripper_joint1", "gripper_joint2")
FINGER_TO_WIDTH = 2.0


class GripperFollowJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("gripper_follow_joint_trajectory")

        # No joint_prefix parameter: the driver matches finger joints by suffix
        # and each arm's node lives in that arm's namespace, so the namespace is
        # what routes a goal, not the name.
        self.declare_parameter("action_name", "gripper_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_service_ns", "")
        self.declare_parameter("status_topic", "feedback/gripper_status")
        self.declare_parameter("service_timeout_s", 5.0)
        # A grasp ends where the object is, not where the clock says, so the
        # goal succeeds on the width having settled rather than on it having
        # reached the target: a gripper closing on something never reaches its
        # commanded width, and a position tolerance would fail every grasp.
        self.declare_parameter("settle_epsilon_m", 0.0005)
        self.declare_parameter("settle_time_s", 0.15)
        self.declare_parameter("delivery_timeout_s", 4.0)
        # Settling is not enough on its own: a gripper that never received the
        # command is also perfectly still. One tolerance carries both halves of
        # that question — nearer than this to the target counts as already
        # arrived, and a command that has further to go must close at least this
        # much of the gap before the settle test may succeed.
        self.declare_parameter("progress_tolerance_m", 0.002)
        self.declare_parameter("progress_timeout_s", 1.0)
        # A status older than this is not evidence about the command just sent.
        self.declare_parameter("readback_max_age_s", 1.5)

        action_name = str(self.get_parameter("action_name").value)
        service_ns = str(self.get_parameter("gripper_service_ns").value).strip("/")
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.settle_epsilon_m = float(self.get_parameter("settle_epsilon_m").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.delivery_timeout_s = float(self.get_parameter("delivery_timeout_s").value)
        self.progress_tolerance_m = float(self.get_parameter("progress_tolerance_m").value)
        self.progress_timeout_s = float(self.get_parameter("progress_timeout_s").value)
        self.readback_max_age_s = float(self.get_parameter("readback_max_age_s").value)

        self.last_status: GripperStatus | None = None
        self.last_status_monotonic = 0.0

        def scoped(relative: str) -> str:
            return f"/{service_ns}/{relative}" if service_ns else relative

        self.authorized_pub = self.create_publisher(
            AuthorizedJointTrajectory, scoped("control/gripper/authorized_trajectory"), 10
        )
        self.create_subscription(
            GripperStatus, self.status_topic, self._status_callback, 10
        )

        # Both generations come from the claim response, so the first command
        # after a claim need not wait for the authority topic to catch up.
        self._device_epoch = 0
        self._unit_safety_epoch = 0
        self._sequence = 0

        # Reentrant so the claim future is serviced while the execute callback
        # spins on it.
        self._cb_group = ReentrantCallbackGroup()
        # The owner declares the primitive, then the node: the primitive half
        # says what kind of motion holds the device, the node half is how a
        # commander that died still holding a claim is recognised.
        self.owner_id = f"trajectory:{self.get_name()}"
        self.claim_service_name = scoped("control/gripper/claim_device")
        self.claim_client = self.create_client(
            ClaimDevice, self.claim_service_name, callback_group=self._cb_group
        )
        # A cancel has to reach the device, not only the goal: the driver keeps
        # driving the last width it was given until something replaces it.
        self.stop_service_name = scoped("control/gripper/stop")
        self.stop_client = self.create_client(
            Trigger, self.stop_service_name, callback_group=self._cb_group
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
        self.get_logger().info(
            f"gripper trajectory server on '{action_name}', "
            f"claiming '{self.claim_service_name}'"
        )

    # -- feedback -------------------------------------------------------
    def _status_callback(self, msg: GripperStatus) -> None:
        self.last_status = msg
        self.last_status_monotonic = time.monotonic()

    def _fresh_status(self) -> GripperStatus | None:
        if self.last_status is None:
            return None
        if time.monotonic() - self.last_status_monotonic > self.readback_max_age_s:
            return None
        return self.last_status

    # -- goal handling --------------------------------------------------
    def _goal_callback(self, goal_request):
        del goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _width_from(self, joint_names, positions) -> float | None:
        for suffix in FINGER_SUFFIXES:
            for index, name in enumerate(joint_names):
                if name.endswith(suffix) and index < len(positions):
                    return abs(positions[index]) * FINGER_TO_WIDTH
        return None

    def _point_for(self, width: float, joint_names) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        half = width / FINGER_TO_WIDTH
        point.positions = [
            half if name.endswith("gripper_joint1") else -half for name in joint_names
        ]
        return point

    def _result(self, code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    # -- ownership ------------------------------------------------------
    def _call_claim(self, *, claim: bool) -> tuple[bool, str]:
        if not self.claim_client.wait_for_service(timeout_sec=self.service_timeout_s):
            return False, f"{self.claim_service_name} is not available"
        request = ClaimDevice.Request()
        request.owner_id = self.owner_id
        request.claim = claim
        future = self.claim_client.call_async(request)
        deadline = time.monotonic() + self.service_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{self.claim_service_name} did not answer"
            time.sleep(0.02)
        response = future.result()
        if response is None:
            return False, f"{self.claim_service_name} returned nothing"
        if response.accepted and claim:
            # A claim opens a new era for this device, so the sequence restarts
            # rather than carrying a watermark across owners.
            self._device_epoch = int(response.device_epoch)
            self._unit_safety_epoch = int(response.unit_safety_epoch)
            self._sequence = 0
        return bool(response.accepted), response.message or response.reason

    def _release(self) -> None:
        accepted, detail = self._call_claim(claim=False)
        if not accepted:
            self.get_logger().warn(f"releasing the gripper failed: {detail}")

    def _authority_stamp(self) -> DeviceCommandStamp:
        self._sequence += 1
        stamp = DeviceCommandStamp()
        stamp.owner_id = self.owner_id
        stamp.device_epoch = self._device_epoch
        stamp.unit_safety_epoch = self._unit_safety_epoch
        stamp.sequence = self._sequence
        return stamp

    # -- execution ------------------------------------------------------
    def _health_refusal(self, status: GripperStatus) -> str:
        """Why this status says the gripper cannot execute, or ``""``.

        Read before the command and again while waiting, because these are the
        states a stillness test would otherwise report as a successful grasp.
        """
        if not status.driver_enable_status:
            return "gripper driver is not enabled"
        faults = [
            name
            for name, raised in (
                ("driver_error", status.driver_error_status),
                ("voltage_too_low", status.voltage_too_low),
                ("motor_overheating", status.motor_overheating),
                ("driver_overcurrent", status.driver_overcurrent),
                ("driver_overheating", status.driver_overheating),
            )
            if raised
        ]
        return f"gripper reports {', '.join(faults)}" if faults else ""

    def _await_outcome(
        self, goal_handle, start_width: float, target_width: float
    ) -> tuple[str, str]:
        """Watch the width until the goal is decided. Returns (outcome, detail).

        Three ways to end well and four to end badly, because *settled* alone
        cannot tell a grasp from a command that never arrived:

        - ``arrived``  — already within tolerance of the target when commanded.
        - ``settled``  — moved measurably toward the target, then stopped, which
          is what a gripper closing on an object does. The residual gap is the
          object.
        - ``canceled`` — a cancel was requested; the caller stops the device.
        - ``no_progress``/``moving``/``stale``/``faulted`` — the failures.
        """
        gap = abs(start_width - target_width)
        if gap <= self.progress_tolerance_m:
            return "arrived", (
                f"already at {start_width:.4f} m, {gap:.4f} m from the "
                f"commanded {target_width:.4f} m"
            )

        deadline = time.monotonic() + self.delivery_timeout_s
        progress_deadline = time.monotonic() + self.progress_timeout_s
        progressed = False
        held_since = None
        previous = None
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return "canceled", "cancel requested during travel"
            status = self._fresh_status()
            if status is None:
                held_since = None
                previous = None
                time.sleep(0.02)
                continue
            refusal = self._health_refusal(status)
            if refusal:
                return "faulted", refusal
            width = status.width
            if not progressed:
                # Toward the target, not merely different: a width that moved
                # for some other reason is not this command being executed.
                closed = gap - abs(width - target_width)
                if closed >= self.progress_tolerance_m:
                    progressed = True
                elif time.monotonic() >= progress_deadline:
                    return "no_progress", (
                        f"width still {width:.4f} m after "
                        f"{self.progress_timeout_s:.1f} s; commanded "
                        f"{target_width:.4f} m from {start_width:.4f} m"
                    )
            if progressed and previous is not None and (
                abs(width - previous) <= self.settle_epsilon_m
            ):
                held_since = held_since if held_since is not None else time.monotonic()
                if time.monotonic() - held_since >= self.settle_time_s:
                    residual = abs(width - target_width)
                    return "settled", (
                        f"settled at {width:.4f} m, {residual:.4f} m from the "
                        f"commanded {target_width:.4f} m, {status.force:.2f} N"
                    )
            else:
                held_since = None
            previous = width
            time.sleep(0.02)
        if self._fresh_status() is None:
            return "stale", "gripper status stopped arriving during the goal"
        return "moving", f"width still moving after {self.delivery_timeout_s:.1f} s"

    def _stop_device(self) -> None:
        """Cancel the pending width and hold where the jaws are (best effort)."""
        if not self.stop_client.wait_for_service(timeout_sec=self.service_timeout_s):
            self.get_logger().error(
                f"cancel could not reach {self.stop_service_name}; the gripper "
                "keeps driving its last commanded width"
            )
            return
        future = self.stop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.service_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{self.stop_service_name} did not answer the cancel"
                )
                return
            time.sleep(0.02)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else "no response"
            self.get_logger().error(f"gripper stop did not take: {detail}")
        else:
            self.get_logger().info(f"gripper stopped: {response.message}")

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        if not trajectory.points:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL, "trajectory has no points"
            )
        target_width = self._width_from(joint_names, trajectory.points[-1].positions)
        if target_width is None:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_JOINTS,
                f"no gripper finger joint among {joint_names}",
            )

        # Where the jaws are before the command is what makes progress
        # measurable afterwards, so a missing readback fails here rather than
        # being discovered as stillness later.
        before = self._fresh_status()
        if before is None:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"no gripper status on {self.status_topic}; nothing commanded",
            )
        refusal = self._health_refusal(before)
        if refusal:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"{refusal}; nothing commanded",
            )

        # Taking the claim is part of executing the goal, not setup: the driver
        # is fail-closed, so an unclaimed gripper executes nothing.
        claimed, detail = self._call_claim(claim=True)
        if not claimed:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"could not take the gripper: {detail}",
            )
        try:
            authorized = AuthorizedJointTrajectory()
            authorized.authority = self._authority_stamp()
            authorized.trajectory = trajectory
            self.authorized_pub.publish(authorized)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = joint_names
            feedback.desired = self._point_for(target_width, joint_names)
            feedback.actual = self._point_for(before.width, joint_names)
            goal_handle.publish_feedback(feedback)

            outcome, reason = self._await_outcome(
                goal_handle, before.width, target_width
            )
            if outcome == "canceled":
                # Stop first, then report: the firmware drives the last width it
                # was given until something replaces it, so a goal that ends
                # without this leaves the jaws travelling.
                self._stop_device()
                # CANCELED is carried by the goal state; error_code has no
                # cancellation value, and SUCCESSFUL is what ros2_control's
                # trajectory controller reports there too. Read the state.
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.SUCCESSFUL, f"canceled: {reason}"
                )
            if outcome in ("arrived", "settled"):
                goal_handle.succeed()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL, reason)
            self._stop_device()
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                f"gripper goal failed ({outcome}): {reason}",
            )
        finally:
            # Released on every exit, abort included: a goal that ended has no
            # further claim, and holding one would block the next commander.
            self._release()


def main() -> None:
    rclpy.init()
    node = GripperFollowJointTrajectoryBridge()
    # Bounded on purpose: an unbounded MultiThreadedExecutor takes cpu_count()
    # threads for a handful of callbacks, contending on the GIL without buying
    # concurrency Python can use.
    executor = MultiThreadedExecutor(num_threads=4)
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


if __name__ == "__main__":
    main()
