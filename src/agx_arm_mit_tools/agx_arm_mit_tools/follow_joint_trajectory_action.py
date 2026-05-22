from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_mit_controller.trajectory_buffer import JointTrajectoryBuffer, SampledTrajectoryPoint

from .joint_state_trajectory_bridge import DEFAULT_JOINT_NAMES


class MitFollowJointTrajectoryActionBridge(Node):
	def __init__(self) -> None:
		super().__init__("mit_follow_joint_trajectory")

		self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
		self.declare_parameter("action_name", "arm_controller/follow_joint_trajectory")
		self.declare_parameter("trajectory_topic", "mit_controller/joint_trajectory")
		self.declare_parameter("feedback_topic", "feedback/joint_states")
		self.declare_parameter("enable_service", "mit_controller/enable")
		self.declare_parameter("cancel_service", "mit_controller/cancel_trajectory")
		self.declare_parameter("auto_enable", True)
		self.declare_parameter("feedback_timeout_s", 0.5)
		self.declare_parameter("goal_margin_s", 0.25)
		self.declare_parameter("service_timeout_s", 3.0)

		self.joint_names = [str(value) for value in self.get_parameter("joint_names").value]
		action_name = str(self.get_parameter("action_name").value)
		trajectory_topic = str(self.get_parameter("trajectory_topic").value)
		feedback_topic = str(self.get_parameter("feedback_topic").value)
		enable_service = str(self.get_parameter("enable_service").value)
		cancel_service = str(self.get_parameter("cancel_service").value)
		self.auto_enable = bool(self.get_parameter("auto_enable").value)
		self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
		self.goal_margin_s = float(self.get_parameter("goal_margin_s").value)
		self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)

		self.feedback_positions: dict[str, float] = {}
		self.feedback_velocities: dict[str, float] = {}
		self.last_feedback_time = 0.0

		self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
		self.enable_client = self.create_client(SetBool, enable_service)
		self.cancel_client = self.create_client(Empty, cancel_service)
		self.create_subscription(JointState, feedback_topic, self._feedback_callback, 20)

		self.action_server = ActionServer(
			self,
			FollowJointTrajectory,
			action_name,
			execute_callback=self._execute_callback,
			goal_callback=self._goal_callback,
			cancel_callback=self._cancel_callback,
		)
		self.get_logger().warn(
			"This bridge is deprecated. Use the MIT controller action server directly."
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

	def _wait_for_service(self, client) -> bool:
		deadline = time.monotonic() + self.service_timeout_s
		while time.monotonic() < deadline and rclpy.ok():
			if client.wait_for_service(timeout_sec=0.2):
				return True
		return False

	def _call_enable(self) -> tuple[bool, str]:
		if not self.auto_enable:
			return True, ""
		if not self._wait_for_service(self.enable_client):
			return False, "MIT enable service is unavailable"
		request = SetBool.Request()
		request.data = True
		future = self.enable_client.call_async(request)
		deadline = time.monotonic() + self.service_timeout_s
		while time.monotonic() < deadline and not future.done() and rclpy.ok():
			time.sleep(0.02)
		if not future.done() or future.result() is None:
			return False, "Timed out enabling the MIT controller"
		if not future.result().success:
			return False, future.result().message
		return True, future.result().message

	def _call_cancel(self) -> None:
		if not self._wait_for_service(self.cancel_client):
			return
		future = self.cancel_client.call_async(Empty.Request())
		deadline = time.monotonic() + self.service_timeout_s
		while time.monotonic() < deadline and not future.done() and rclpy.ok():
			time.sleep(0.02)

	def _goal_callback(self, goal_request: FollowJointTrajectory.Goal):
		try:
			JointTrajectoryBuffer.from_ros_message(
				self.joint_names,
				goal_request.trajectory,
			)
		except ValueError as exc:
			self.get_logger().error(f"Rejected FollowJointTrajectory goal: {exc}")
			return GoalResponse.REJECT
		return GoalResponse.ACCEPT

	def _cancel_callback(self, goal_handle):
		del goal_handle
		return CancelResponse.ACCEPT

	def _point_from_sample(self, sample: SampledTrajectoryPoint) -> JointTrajectoryPoint:
		point = JointTrajectoryPoint()
		point.positions = list(sample.positions)
		point.velocities = list(sample.velocities)
		point.effort = list(sample.efforts)
		return point

	def _actual_point(self) -> JointTrajectoryPoint:
		point = JointTrajectoryPoint()
		point.positions = [self.feedback_positions.get(joint, 0.0) for joint in self.joint_names]
		point.velocities = [self.feedback_velocities.get(joint, 0.0) for joint in self.joint_names]
		point.effort = [0.0] * len(self.joint_names)
		return point

	def _error_point(self, desired: SampledTrajectoryPoint) -> JointTrajectoryPoint:
		point = JointTrajectoryPoint()
		point.positions = [
			desired.positions[index] - self.feedback_positions.get(joint, 0.0)
			for index, joint in enumerate(self.joint_names)
		]
		point.velocities = [
			desired.velocities[index] - self.feedback_velocities.get(joint, 0.0)
			for index, joint in enumerate(self.joint_names)
		]
		point.effort = [0.0] * len(self.joint_names)
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
		buffer = JointTrajectoryBuffer.from_ros_message(
			self.joint_names,
			goal_handle.request.trajectory,
		)

		ok, detail = self._call_enable()
		if not ok:
			goal_handle.abort()
			return self._failed_result(FollowJointTrajectory.Result.INVALID_GOAL, detail)

		self.trajectory_pub.publish(goal_handle.request.trajectory)
		start_time = time.monotonic()

		while rclpy.ok():
			if goal_handle.is_cancel_requested:
				self._call_cancel()
				goal_handle.canceled()
				return self._failed_result(
					FollowJointTrajectory.Result.INVALID_GOAL,
					"Goal canceled",
				)

			elapsed = time.monotonic() - start_time
			desired = buffer.sample(elapsed)

			feedback = FollowJointTrajectory.Feedback()
			feedback.joint_names = list(self.joint_names)
			feedback.desired = self._point_from_sample(desired)
			feedback.actual = self._actual_point()
			feedback.error = self._error_point(desired)
			goal_handle.publish_feedback(feedback)

			if elapsed >= buffer.duration + self.goal_margin_s:
				if not self._has_fresh_feedback():
					goal_handle.abort()
					return self._failed_result(
						FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
						"MIT trajectory finished but feedback became stale",
					)
				goal_handle.succeed()
				return self._success_result()

			time.sleep(0.05)

		goal_handle.abort()
		return self._failed_result(
			FollowJointTrajectory.Result.INVALID_GOAL,
			"ROS shutdown while executing MIT trajectory",
		)


def main() -> None:
	rclpy.init()
	node = MitFollowJointTrajectoryActionBridge()
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