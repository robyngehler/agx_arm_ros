from __future__ import annotations

import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
	"joint1",
	"joint2",
	"joint3",
	"joint4",
	"joint5",
	"joint6",
	"joint7",
]


def select_target_positions(
	joint_names: Sequence[str],
	base_positions: Sequence[float],
	msg: JointState,
) -> list[float]:
	if len(base_positions) != len(joint_names):
		raise ValueError("base_positions length must match joint_names")

	target_map = {joint: float(value) for joint, value in zip(joint_names, base_positions)}
	used_joint = False
	for name, value in zip(msg.name, msg.position):
		if name in target_map:
			target_map[name] = float(value)
			used_joint = True

	if not used_joint:
		raise ValueError("JointState did not include any controlled arm joints")

	return [target_map[joint] for joint in joint_names]


def build_single_point_trajectory(
	joint_names: Sequence[str],
	positions: Sequence[float],
	duration_s: float,
) -> JointTrajectory:
	msg = JointTrajectory()
	msg.joint_names = list(joint_names)

	point = JointTrajectoryPoint()
	point.positions = [float(value) for value in positions]
	point.velocities = [0.0] * len(joint_names)
	point.effort = [0.0] * len(joint_names)
	point.time_from_start.sec = int(duration_s)
	point.time_from_start.nanosec = int((duration_s - point.time_from_start.sec) * 1e9)

	msg.points = [point]
	return msg


class JointStateTrajectoryBridge(Node):
	def __init__(self) -> None:
		super().__init__("mit_joint_state_bridge")

		self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
		self.declare_parameter("input_topic", "mit_controller/soft_target_joint_states")
		self.declare_parameter("output_topic", "mit_controller/joint_trajectory")
		self.declare_parameter("feedback_topic", "feedback/joint_states")
		self.declare_parameter("enable_service", "mit_controller/enable")
		self.declare_parameter("segment_duration_s", 0.75)
		self.declare_parameter("change_tolerance", 1e-4)
		self.declare_parameter("auto_enable", False)
		self.declare_parameter("service_timeout_s", 3.0)

		self.joint_names = [str(value) for value in self.get_parameter("joint_names").value]
		self.segment_duration_s = float(self.get_parameter("segment_duration_s").value)
		self.change_tolerance = float(self.get_parameter("change_tolerance").value)
		self.auto_enable = bool(self.get_parameter("auto_enable").value)
		self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
		self.feedback_topic = str(self.get_parameter("feedback_topic").value)

		input_topic = str(self.get_parameter("input_topic").value)
		output_topic = str(self.get_parameter("output_topic").value)
		enable_service = str(self.get_parameter("enable_service").value)

		self.current_positions: dict[str, float] = {}
		self.last_target_positions: list[float] | None = None
		self.enable_requested = False

		self.trajectory_pub = self.create_publisher(JointTrajectory, output_topic, 10)
		self.enable_client = self.create_client(SetBool, enable_service)
		self.create_subscription(JointState, self.feedback_topic, self._feedback_callback, 20)
		self.create_subscription(JointState, input_topic, self._target_callback, 10)

	def _feedback_callback(self, msg: JointState) -> None:
		if not msg.name:
			return
		self.current_positions.update(
			{name: float(value) for name, value in zip(msg.name, msg.position)}
		)

	def _call_enable(self) -> None:
		if self.enable_requested or not self.auto_enable:
			return
		if not self.enable_client.wait_for_service(timeout_sec=self.service_timeout_s):
			self.get_logger().warn("MIT enable service is not available yet")
			return

		request = SetBool.Request()
		request.data = True
		future = self.enable_client.call_async(request)
		deadline = time.monotonic() + self.service_timeout_s
		while time.monotonic() < deadline and not future.done() and rclpy.ok():
			time.sleep(0.02)
		if not future.done() or future.result() is None:
			self.get_logger().warn("Timed out enabling the MIT controller")
			return
		if not future.result().success:
			self.get_logger().warn(
				f"MIT enable request was rejected: {future.result().message}"
			)
			return
		self.enable_requested = True

	def _base_positions(self) -> list[float] | None:
		if all(joint in self.current_positions for joint in self.joint_names):
			return [self.current_positions[joint] for joint in self.joint_names]
		return self.last_target_positions

	def _target_callback(self, msg: JointState) -> None:
		base_positions = self._base_positions()
		if base_positions is None:
			self.get_logger().warn(
				"Ignoring RViz soft target because no current arm joint state is available yet"
			)
			return

		try:
			target_positions = select_target_positions(self.joint_names, base_positions, msg)
		except ValueError as exc:
			self.get_logger().warn(str(exc))
			return

		if self.last_target_positions is not None:
			max_delta = max(
				abs(current - previous)
				for current, previous in zip(target_positions, self.last_target_positions)
			)
			if max_delta <= self.change_tolerance:
				return

		self._call_enable()
		self.trajectory_pub.publish(
			build_single_point_trajectory(
				self.joint_names,
				target_positions,
				self.segment_duration_s,
			)
		)
		self.last_target_positions = list(target_positions)


def main() -> None:
	rclpy.init()
	node = JointStateTrajectoryBridge()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = ["main"]