from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_SUFFIXES = [
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "index_abad_joint",
    "index_pip_joint",
    "middle_pip_joint",
    "ring_abad_joint",
    "ring_pip_joint",
    "pinky_abad_joint",
    "pinky_pip_joint",
]


def build_joint_names(hand_side: str) -> list[str]:
    prefix = f"{hand_side}_"
    return [f"{prefix}{suffix}" for suffix in JOINT_SUFFIXES]


def _trajectory_duration_s(msg: JointTrajectory) -> float:
    if not msg.points:
        return 0.0
    last_point = msg.points[-1].time_from_start
    return float(last_point.sec) + float(last_point.nanosec) / 1e9


class OmniHandFollowJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("omnihand_follow_joint_trajectory")

        self.declare_parameter("omnihand_type", "right")
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

        hand_side = str(self.get_parameter("omnihand_type").value)
        if hand_side not in ("left", "right"):
            raise ValueError("omnihand_type must be 'left' or 'right'")

        self.joint_names = build_joint_names(hand_side)
        action_name = str(self.get_parameter("action_name").value)
        trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.goal_margin_s = float(self.get_parameter("goal_margin_s").value)

        self.feedback_positions: dict[str, float] = {}
        self.feedback_velocities: dict[str, float] = {}
        self.last_feedback_time = 0.0

        self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.create_subscription(JointState, feedback_topic, self._feedback_callback, 20)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
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

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self._validate_trajectory(trajectory)

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