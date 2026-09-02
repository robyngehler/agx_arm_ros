#!/usr/bin/env python3
"""FollowJointTrajectory server for the AGX gripper.

The external interface is standard FollowJointTrajectory, so MoveIt needs to
know nothing about this device. Internally the accepted goal is bound to the
claim it runs under and handed to the arm driver, which owns the gripper's SDK
session and CAN socket; this node never touches the hardware.

No bus handshake: the gripper rides the arm's own bus and its transmits are
serialized onto the arm's worker, so there is no window to open. That is the
difference from the OmniHand, which has a bus of its own.
"""
from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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
        # A status older than this is not evidence about the command just sent.
        self.declare_parameter("readback_max_age_s", 1.5)

        action_name = str(self.get_parameter("action_name").value)
        service_ns = str(self.get_parameter("gripper_service_ns").value).strip("/")
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.settle_epsilon_m = float(self.get_parameter("settle_epsilon_m").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.delivery_timeout_s = float(self.get_parameter("delivery_timeout_s").value)
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
    def _await_settled(self, target_width: float) -> tuple[bool, str]:
        """Wait until the width stops changing, or the deadline passes.

        Settled, not arrived: a gripper closing on an object stops short of its
        commanded width and holds there with force, which is success. What
        failure looks like is a width that never stops moving, or a readback
        that stops arriving.
        """
        deadline = time.monotonic() + self.delivery_timeout_s
        held_since = None
        previous = None
        while rclpy.ok() and time.monotonic() < deadline:
            status = self._fresh_status()
            if status is None:
                held_since = None
                previous = None
                time.sleep(0.02)
                continue
            width = status.width
            if previous is not None and abs(width - previous) <= self.settle_epsilon_m:
                held_since = held_since if held_since is not None else time.monotonic()
                if time.monotonic() - held_since >= self.settle_time_s:
                    gap = abs(width - target_width)
                    return True, (
                        f"settled at {width:.4f} m, {gap:.4f} m from the "
                        f"commanded {target_width:.4f} m"
                    )
            else:
                held_since = None
            previous = width
            time.sleep(0.02)
        if self._fresh_status() is None:
            return False, "no gripper status newer than the command"
        return False, f"width still moving after {self.delivery_timeout_s:.1f} s"

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
            status = self._fresh_status()
            if status is not None:
                feedback.actual = self._point_for(status.width, joint_names)
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL, "Goal canceled"
                )

            settled, reason = self._await_settled(target_width)
            if not settled:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                    f"gripper did not settle: {reason}",
                )
            goal_handle.succeed()
            return self._result(FollowJointTrajectory.Result.SUCCESSFUL, reason)
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
