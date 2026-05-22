from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from agx_arm_mit_controller.trajectory_io import (
	load_recorded_trajectory,
	recorded_to_joint_trajectory,
)


class SavedTrajectoryExecutorNode(Node):
	def __init__(self, node_name: str = "saved_trajectory_executor") -> None:
		super().__init__(node_name)
		self.publisher = self.create_publisher(JointTrajectory, "mit_controller/joint_trajectory", 10)
		self.trajectory_publisher = self.publisher
		self.enable_client = self.create_client(SetBool, "mit_controller/enable")
		self.set_normal_mode_client = self.create_client(Trigger, "set_normal_mode")
		self.last_joint_state_time_monotonic = 0.0
		self.create_subscription(JointState, "feedback/joint_states", self._joint_state_callback, 20)

	def _joint_state_callback(self, msg: JointState) -> None:
		if msg.name:
			self.last_joint_state_time_monotonic = time.monotonic()

	def wait_for_fresh_joint_state(self, timeout_s: float, freshness_s: float = 0.5) -> bool:
		deadline = time.monotonic() + timeout_s
		while time.monotonic() < deadline and rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			if (time.monotonic() - self.last_joint_state_time_monotonic) <= freshness_s:
				return True
		return False

	def call_set_normal_mode(self, timeout_s: float) -> bool:
		if not self.set_normal_mode_client.wait_for_service(timeout_sec=timeout_s):
			return False
		future = self.set_normal_mode_client.call_async(Trigger.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False
		return bool(future.result().success)

	def call_enable_mit(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
		if not self.enable_client.wait_for_service(timeout_sec=timeout_s):
			return False, "MIT controller enable service is not available"

		request = SetBool.Request()
		request.data = enabled
		future = self.enable_client.call_async(request)
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False, "Timed out waiting for MIT controller enable response"

		response = future.result()
		if not response.success:
			return False, response.message
		return True, response.message


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Load and execute a saved MIT trajectory recording")
	parser.add_argument("trajectory_path", help="Path to a saved recording JSON file")
	parser.add_argument("--service-timeout", type=float, default=5.0, help="Timeout for MIT enable service")
	parser.add_argument("--feedback-timeout", type=float, default=3.0, help="Timeout waiting for fresh feedback/joint_states after mode changes")
	parser.add_argument("--publish-repetitions", type=int, default=3, help="How often to republish the trajectory")
	parser.add_argument("--publish-interval", type=float, default=0.2, help="Seconds between trajectory publishes")
	return parser.parse_args()


def execute_recorded_trajectory(
	node: SavedTrajectoryExecutorNode,
	trajectory_path: Path,
	*,
	service_timeout: float,
	feedback_timeout: float,
	publish_repetitions: int,
	publish_interval: float,
) -> None:
	trajectory = load_recorded_trajectory(trajectory_path)
	joint_trajectory = recorded_to_joint_trajectory(trajectory)

	node.get_logger().info("Zeroing recorded trajectory effort feedforward for playback safety")
	if not node.call_set_normal_mode(service_timeout):
		raise RuntimeError("Failed to switch robot to normal mode before MIT playback")
	if not node.wait_for_fresh_joint_state(feedback_timeout):
		raise RuntimeError("Did not receive fresh feedback/joint_states after switching to normal mode")

	enabled, detail = node.call_enable_mit(True, service_timeout)
	if not enabled:
		suffix = f": {detail}" if detail else ""
		raise RuntimeError(f"Failed to enable MIT controller before publishing trajectory{suffix}")

	for _ in range(max(1, publish_repetitions)):
		node.publisher.publish(joint_trajectory)
		rclpy.spin_once(node, timeout_sec=0.05)
		time.sleep(max(0.0, publish_interval))

	print(
		f"Published saved trajectory '{trajectory.name}' with {len(trajectory.points)} points "
		f"and {trajectory.duration:.2f}s duration"
	)


def main() -> None:
	args = parse_args()

	rclpy.init()
	node = SavedTrajectoryExecutorNode()
	try:
		execute_recorded_trajectory(
			node,
			Path(args.trajectory_path),
			service_timeout=args.service_timeout,
			feedback_timeout=args.feedback_timeout,
			publish_repetitions=args.publish_repetitions,
			publish_interval=args.publish_interval,
		)
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = ["main"]