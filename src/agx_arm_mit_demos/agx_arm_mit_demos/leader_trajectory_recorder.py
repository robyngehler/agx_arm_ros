from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from agx_arm_mit_controller.model_metadata import (
	build_gravity_context,
	compute_flange_pose_from_mdh,
)
from agx_arm_mit_controller.trajectory_io import (
	RecordedTrajectory,
	default_recorded_at,
	sanitize_trajectory_name,
	save_recorded_trajectory,
	trim_trailing_stationary_points,
	with_finite_difference_velocities,
)


@dataclass(frozen=True)
class RecorderSnapshot:
	time_from_start: float
	positions: list[float]
	efforts: list[float]
	flange_pose: Optional[list[float]]


class LeaderTrajectoryRecorderNode(Node):
	def __init__(self) -> None:
		super().__init__("leader_trajectory_recorder")
		self.leader_joint_state: Optional[JointState] = None
		self.create_subscription(JointState, "feedback/leader_joint_angles", self._leader_callback, 20)
		self.enable_client = self.create_client(SetBool, "enable_agx_arm")
		self.set_normal_mode_client = self.create_client(Trigger, "set_normal_mode")
		self.set_leader_mode_client = self.create_client(Trigger, "set_leader_mode")

	def _leader_callback(self, msg: JointState) -> None:
		self.leader_joint_state = msg

	def wait_for_topic_feedback(self, timeout_s: float) -> bool:
		deadline = time.monotonic() + timeout_s
		while time.monotonic() < deadline and rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			if self.leader_joint_state is not None:
				return True
		return False

	def wait_for_services(self, timeout_s: float) -> bool:
		deadline = time.monotonic() + timeout_s
		clients = [self.enable_client, self.set_normal_mode_client, self.set_leader_mode_client]
		while time.monotonic() < deadline and rclpy.ok():
			if all(client.wait_for_service(timeout_sec=0.2) for client in clients):
				return True
		return False

	def call_enable(self, enabled: bool, timeout_s: float) -> bool:
		request = SetBool.Request()
		request.data = enabled
		future = self.enable_client.call_async(request)
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False
		response = future.result()
		if not response.success:
			self.get_logger().error(response.message)
		return response.success

	def call_trigger(self, client, label: str, timeout_s: float) -> bool:
		if not client.wait_for_service(timeout_sec=timeout_s):
			self.get_logger().error(f"Service {label} is not available")
			return False
		future = client.call_async(Trigger.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			self.get_logger().error(f"Timed out calling {label}")
			return False
		response = future.result()
		if not response.success:
			self.get_logger().error(f"{label} failed: {response.message}")
		return response.success

	def ensure_normal_mode(self, timeout_s: float, retries: int = 3) -> bool:
		for attempt in range(1, retries + 1):
			if self.call_trigger(self.set_normal_mode_client, "set_normal_mode", timeout_s):
				return True
			if attempt < retries:
				time.sleep(0.25)
		return False

	def current_snapshot(self, time_from_start: float, joint_names: list[str]) -> Optional[RecorderSnapshot]:
		if self.leader_joint_state is None:
			return None

		leader_position_map = {
			name: float(value)
			for name, value in zip(self.leader_joint_state.name, self.leader_joint_state.position)
		}
		if any(joint_name not in leader_position_map for joint_name in joint_names):
			return None
		positions = [leader_position_map[joint_name] for joint_name in joint_names]
		efforts = [0.0] * len(joint_names)
		flange_pose = compute_flange_pose_from_mdh(positions, robot="nero")
		return RecorderSnapshot(
			time_from_start=time_from_start,
			positions=positions,
			efforts=efforts,
			flange_pose=flange_pose,
		)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Interactive Nero leader mode trajectory recorder")
	parser.add_argument("--output-dir", default="~/agx_arm_trajectories", help="Directory for saved recordings")
	# The arm's feedback rate is the ceiling; sampling above it only repeats
	# samples (docs/sprint_refactor/reference/feedback_rate_budget.md).
	parser.add_argument("--sample-rate", type=float, default=100.0, help="Sampling rate in Hz")
	parser.add_argument("--hold-timeout", type=float, default=3.0, help="Stop after this much stationary time")
	parser.add_argument("--movement-threshold", type=float, default=0.01, help="Motion threshold in rad")
	parser.add_argument("--service-timeout", type=float, default=5.0, help="Timeout for ROS service calls")
	parser.add_argument("--feedback-timeout", type=float, default=10.0, help="Timeout waiting for feedback topics")
	parser.add_argument("--auto-enable", action="store_true", help="Enable the arm before switching modes")
	parser.add_argument("--name", default="", help="Optional save name; if omitted, the tool asks interactively")
	parser.add_argument("--urdf-path", default="", help="Optional path to a Nero URDF used for metadata")
	return parser.parse_args()


def record_trajectory(
	node: LeaderTrajectoryRecorderNode,
	*,
	sample_rate: float,
	hold_timeout: float,
	movement_threshold: float,
	wait_for_enter: bool = True,
	joint_names: Optional[list[str]] = None,
) -> tuple[list[RecorderSnapshot], int]:
	# Default to the native leader stream's joint order; callers driving a
	# software freedrive (no leader stream) pass their own joint_names instead.
	if joint_names is None:
		joint_names = list(node.leader_joint_state.name) if node.leader_joint_state is not None else []
	if not joint_names:
		raise RuntimeError("no joint names available for recording")

	if wait_for_enter:
		print("Press Enter to start recording once the arm is in leader mode.")
		input()

	samples: list[RecorderSnapshot] = []
	last_motion_time = time.monotonic()
	motion_started = False
	recording_start = time.monotonic()
	period = 1.0 / sample_rate

	while rclpy.ok():
		loop_start = time.monotonic()
		rclpy.spin_once(node, timeout_sec=min(period, 0.1))
		snapshot = node.current_snapshot(loop_start - recording_start, joint_names)
		if snapshot is None:
			continue

		if samples:
			deltas = [
				abs(current - previous)
				for current, previous in zip(snapshot.positions, samples[-1].positions)
			]
			if max(deltas, default=0.0) >= movement_threshold:
				motion_started = True
				last_motion_time = loop_start

		samples.append(snapshot)
		if motion_started and (loop_start - last_motion_time) >= hold_timeout:
			break

		sleep_time = max(0.0, period - (time.monotonic() - loop_start))
		if sleep_time > 0.0:
			time.sleep(sleep_time)

	if not motion_started:
		raise RuntimeError("No joint movement detected during recording")

	provisional_points = with_finite_difference_velocities(
		times=[sample.time_from_start for sample in samples],
		positions=[sample.positions for sample in samples],
		efforts=[sample.efforts for sample in samples],
		flange_poses=[sample.flange_pose for sample in samples],
	)
	_, last_motion_index = trim_trailing_stationary_points(provisional_points, movement_threshold)
	return samples[: last_motion_index + 1], len(samples)


def build_recorded_trajectory(
	*,
	name: str,
	joint_names: list[str],
	sample_rate: float,
	hold_timeout: float,
	movement_threshold: float,
	samples: list[RecorderSnapshot],
	raw_sample_count: int,
	urdf_path: str | None = None,
	metadata: Optional[dict[str, Any]] = None,
) -> RecordedTrajectory:
	points = with_finite_difference_velocities(
		times=[sample.time_from_start for sample in samples],
		positions=[sample.positions for sample in samples],
		efforts=[sample.efforts for sample in samples],
		flange_poses=[sample.flange_pose for sample in samples],
	)
	trimmed_points, last_motion_index = trim_trailing_stationary_points(points, movement_threshold)

	payload_metadata: dict[str, Any] = {
		"recording_mode": "leader",
		"position_source": "feedback/leader_joint_angles",
		"movement_threshold_rad": movement_threshold,
		"hold_timeout_s": hold_timeout,
		"raw_sample_count": raw_sample_count,
		"trimmed_sample_count": len(trimmed_points),
		"last_motion_index": last_motion_index,
		"gravity_context": build_gravity_context(urdf_path or None),
	}
	if metadata:
		payload_metadata.update(metadata)

	return RecordedTrajectory(
		name=name,
		robot="nero",
		joint_names=joint_names,
		sample_rate_hz=sample_rate,
		recorded_at=default_recorded_at(),
		points=trimmed_points,
		metadata=payload_metadata,
	)


def main() -> None:
	args = parse_args()
	if args.sample_rate <= 0.0:
		raise ValueError("--sample-rate must be > 0")
	if args.hold_timeout <= 0.0:
		raise ValueError("--hold-timeout must be > 0")
	if args.movement_threshold <= 0.0:
		raise ValueError("--movement-threshold must be > 0")

	rclpy.init()
	node = LeaderTrajectoryRecorderNode()
	try:
		if not node.wait_for_services(args.service_timeout):
			raise RuntimeError("Required agx_arm_ctrl services are not available")
		if args.auto_enable and not node.call_enable(True, args.service_timeout):
			raise RuntimeError("Failed to enable arm through enable_agx_arm")
		if not node.call_trigger(node.set_normal_mode_client, "set_normal_mode", args.service_timeout):
			raise RuntimeError("Failed to switch to normal mode")
		if not node.call_trigger(node.set_leader_mode_client, "set_leader_mode", args.service_timeout):
			raise RuntimeError("Failed to switch to leader mode")
		if not node.wait_for_topic_feedback(args.feedback_timeout):
			raise RuntimeError("Did not receive feedback/leader_joint_angles")

		print(
			"Leader mode is active. Move the arm by hand. Recording will stop automatically "
			f"after {args.hold_timeout:.1f}s without detected motion."
		)
		samples, raw_sample_count = record_trajectory(
			node,
			sample_rate=args.sample_rate,
			hold_timeout=args.hold_timeout,
			movement_threshold=args.movement_threshold,
		)
		joint_names = list(node.leader_joint_state.name)

		default_name = sanitize_trajectory_name(time.strftime("nero_leader_%Y%m%d_%H%M%S"))
		requested_name = args.name.strip() or input(f"Trajectory name [{default_name}]: ").strip() or default_name
		save_name = sanitize_trajectory_name(requested_name)
		output_dir = Path(args.output_dir).expanduser().resolve()
		file_path = output_dir / f"{save_name}.json"
		if file_path.exists():
			file_path = output_dir / f"{save_name}_{time.strftime('%H%M%S')}.json"

		trajectory = build_recorded_trajectory(
			name=save_name,
			joint_names=joint_names,
			sample_rate=args.sample_rate,
			hold_timeout=args.hold_timeout,
			movement_threshold=args.movement_threshold,
			samples=samples,
			raw_sample_count=raw_sample_count,
			urdf_path=args.urdf_path or None,
		)
		saved_path = save_recorded_trajectory(trajectory, file_path)
		print(f"Saved trajectory to {saved_path}")
	finally:
		try:
			if not node.ensure_normal_mode(args.service_timeout):
				node.get_logger().warn("Recorder cleanup could not switch the robot back to normal mode")
		except Exception:
			pass
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = ["main"]