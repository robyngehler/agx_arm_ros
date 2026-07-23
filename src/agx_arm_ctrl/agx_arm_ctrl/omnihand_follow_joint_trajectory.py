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

from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, get_hand_model
# Shared, model-aware joint naming — do NOT keep a second JOINT_SUFFIXES copy here
# (proposal §6/§11.3): a stale O10 list would flag every Pro-only joint as unknown.
from agx_arm_ctrl.omnihand_bridge_node import build_joint_names


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
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("goal_margin_s", 0.25)
        # Step-and-settle handshake: quiesce the same-side arm into a verified
        # hold for the duration of a hand trajectory so MoveIt hand execution
        # owns the shared side bus instead of losing arbitration under arm MIT.
        self.declare_parameter("handshake_enabled", True)
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
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.goal_margin_s = float(self.get_parameter("goal_margin_s").value)
        self.handshake_enabled = bool(self.get_parameter("handshake_enabled").value)
        self.handshake_timeout_s = float(self.get_parameter("handshake_timeout_s").value)
        arm_ns = str(self.get_parameter("arm_service_ns").value).strip("/")

        self.feedback_positions: dict[str, float] = {}
        self.feedback_velocities: dict[str, float] = {}
        self.last_feedback_time = 0.0
        self._window_open = False

        self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.create_subscription(JointState, feedback_topic, self._feedback_callback, 20)

        # Reentrant group so the handshake service futures are serviced while the
        # action execute callback is spinning on them.
        self._cb_group = ReentrantCallbackGroup()
        prepare_name = f"/{arm_ns}/prepare_hand_window" if arm_ns else "prepare_hand_window"
        resume_name = f"/{arm_ns}/resume_arm_control" if arm_ns else "resume_arm_control"
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

    def _has_fresh_feedback(self) -> bool:
        if self.last_feedback_time <= 0.0:
            return False
        return (time.monotonic() - self.last_feedback_time) <= self.feedback_timeout_s

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
        # Own the shared side bus for the whole hand trajectory: quiesce the arm,
        # run, then always reopen it — so MoveIt hand execution is safe under the
        # always-on arm MIT without the caller needing to know the handshake.
        opened, msg = self._open_hand_window()
        if not opened:
            goal_handle.abort()
            return self._failed_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"could not open hand window: {msg}",
            )
        try:
            return self._run_trajectory(goal_handle, trajectory)
        finally:
            self._close_hand_window()

    def _run_trajectory(self, goal_handle, trajectory):
        self.trajectory_pub.publish(trajectory)
        start_time = time.monotonic()
        duration_s = _trajectory_duration_s(trajectory)
        desired = self._desired_point(trajectory)
        goal_joint_names = list(trajectory.joint_names)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._failed_result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Goal canceled",
                )

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = goal_joint_names
            feedback.desired = desired
            feedback.actual = self._actual_point(goal_joint_names)
            feedback.error = self._error_point(desired, goal_joint_names)
            goal_handle.publish_feedback(feedback)

            if time.monotonic() - start_time >= duration_s + self.goal_margin_s:
                if not self._has_fresh_feedback():
                    goal_handle.abort()
                    return self._failed_result(
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                        "OmniHand trajectory finished but hand feedback is stale",
                    )
                goal_handle.succeed()
                return self._success_result()

            time.sleep(0.05)

        goal_handle.abort()
        return self._failed_result(
            FollowJointTrajectory.Result.INVALID_GOAL,
            "ROS shutdown while executing OmniHand trajectory",
        )


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