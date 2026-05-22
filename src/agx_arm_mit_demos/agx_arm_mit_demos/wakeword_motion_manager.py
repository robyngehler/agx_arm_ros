from __future__ import annotations

import argparse
from enum import Enum
from pathlib import Path
import random
import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger

from agx_arm_mit_controller.model_metadata import compute_flange_pose_from_mdh
from agx_arm_mit_controller.trajectory_io import sanitize_trajectory_name, save_recorded_trajectory

from .execute_saved_trajectory import SavedTrajectoryExecutorNode, execute_recorded_trajectory
from .leader_trajectory_recorder import RecorderSnapshot, build_recorded_trajectory, record_trajectory


class ManagerState(str, Enum):
	IDLE = "idle"
	RECORD = "record"
	PLAYBACK = "playback"


TRANSIENT_MODE_STARTUP_MESSAGES = {
	"Agx_arm is not connected",
}


class TerminalKeyReader:
	def __init__(self, enabled: bool) -> None:
		self.enabled = enabled and sys.stdin.isatty()
		self.fd: Optional[int] = None
		self._settings: Optional[list] = None

	def __enter__(self) -> "TerminalKeyReader":
		if self.enabled:
			self.fd = sys.stdin.fileno()
			self._settings = termios.tcgetattr(self.fd)
			tty.setcbreak(self.fd)
		return self

	def __exit__(self, exc_type, exc, exc_tb) -> None:
		if self.enabled and self.fd is not None and self._settings is not None:
			termios.tcsetattr(self.fd, termios.TCSADRAIN, self._settings)

	def read_key(self) -> Optional[str]:
		if not self.enabled:
			return None
		ready, _, _ = select.select([sys.stdin], [], [], 0.0)
		if not ready:
			return None
		return sys.stdin.read(1)


class WakewordMotionManagerNode(SavedTrajectoryExecutorNode):
	def __init__(self, args: argparse.Namespace) -> None:
		super().__init__(node_name="agx_arm_motion_manager")
		self.args = args
		self.library_dir = Path(args.library_dir).expanduser().resolve()
		self.name_prefix = args.name_prefix.strip()
		self.random_selection = bool(args.random_selection)
		self.selected_index = 0
		self.shutdown_requested = False
		self.runtime_active = False
		self.arm_enable_verified = False
		self.external_trigger_pending = False
		self.state = ManagerState.IDLE
		self.last_triggered_path: Optional[Path] = None
		self.last_trigger_time_monotonic = 0.0
		self.random_generator = random.Random()
		self.trajectory_paths: list[Path] = []
		self.leader_joint_state: Optional[JointState] = None

		self.enable_arm_client = self.create_client(SetBool, "enable_agx_arm")
		self.set_leader_mode_client = self.create_client(Trigger, "set_leader_mode")
		self.hold_current_client = self.create_client(Empty, "mit_controller/hold_current")
		self.cancel_trajectory_client = self.create_client(Empty, "mit_controller/cancel_trajectory")
		self.create_subscription(JointState, "feedback/leader_joint_angles", self._leader_callback, 20)
		self.create_service(Trigger, "~/trigger_motion", self._trigger_motion_callback)

		self.refresh_library()

	def _leader_callback(self, msg: JointState) -> None:
		self.leader_joint_state = msg

	def _required_service_clients(self, state: ManagerState) -> list[tuple[str, object]]:
		clients: list[tuple[str, object]] = []
		if self.args.auto_enable_arm:
			clients.append(("enable_agx_arm", self.enable_arm_client))

		if state in (ManagerState.IDLE, ManagerState.RECORD):
			clients.extend(
				[
					("set_normal_mode", self.set_normal_mode_client),
					("set_leader_mode", self.set_leader_mode_client),
				]
			)
		elif state == ManagerState.PLAYBACK:
			clients.extend(
				[
					("set_normal_mode", self.set_normal_mode_client),
					("mit_controller/enable", self.enable_client),
					("mit_controller/hold_current", self.hold_current_client),
				]
			)
		return clients

	def missing_service_labels(self, state: ManagerState) -> list[str]:
		missing: list[str] = []
		for label, client in self._required_service_clients(state):
			if not client.wait_for_service(timeout_sec=0.1):
				missing.append(label)
		return missing

	def wait_for_required_services(self, state: ManagerState) -> None:
		timeout_s = float(self.args.startup_timeout)
		deadline = None if timeout_s <= 0.0 else (time.monotonic() + timeout_s)
		last_log_time = 0.0

		while rclpy.ok():
			missing = self.missing_service_labels(state)
			if not missing:
				return

			now = time.monotonic()
			if now - last_log_time >= 2.0:
				missing_list = ", ".join(missing)
				self.get_logger().warn(
					f"Waiting for services needed for {state.value} mode: {missing_list}"
				)
				last_log_time = now

			if deadline is not None and now >= deadline:
				missing_list = ", ".join(missing)
				raise RuntimeError(
					f"Timed out waiting for services needed for {state.value} mode: {missing_list}"
				)

			rclpy.spin_once(self, timeout_sec=0.1)

	def wait_for_leader_feedback(self, timeout_s: float) -> bool:
		deadline = time.monotonic() + timeout_s
		while time.monotonic() < deadline and rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			if self.leader_joint_state is not None:
				return True
		return False

	def call_enable_arm(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
		if not self.enable_arm_client.wait_for_service(timeout_sec=timeout_s):
			return False, "Service enable_agx_arm is not available"

		request = SetBool.Request()
		request.data = enabled
		future = self.enable_arm_client.call_async(request)
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False, "Timed out waiting for enable_agx_arm"
		response = future.result()
		if response.success:
			self.arm_enable_verified = enabled
		return bool(response.success), response.message

	def _retry_deadline(self, timeout_s: float) -> float:
		return time.monotonic() + max(timeout_s, 0.1)

	def _sleep_before_retry(self, deadline: float, retry_interval_s: float = 0.25) -> bool:
		remaining = deadline - time.monotonic()
		if remaining <= 0.0:
			return False
		time.sleep(min(retry_interval_s, remaining))
		return True

	def call_mode_trigger_with_auto_enable(self, client, label: str, timeout_s: float) -> tuple[bool, str]:
		deadline = self._retry_deadline(timeout_s)
		last_message = ""

		while rclpy.ok():
			remaining = max(0.1, deadline - time.monotonic())
			success, message = self.call_trigger_client(client, label, remaining)
			if success:
				self.arm_enable_verified = True
				return True, message

			last_message = message
			if message == "Agx_arm is not enabled" and self.args.auto_enable_arm:
				self.arm_enable_verified = False
				enable_success, enable_message = self.call_enable_arm(True, remaining)
				if enable_success:
					continue
				last_message = enable_message or message

			if last_message not in TRANSIENT_MODE_STARTUP_MESSAGES:
				return False, last_message

			self.get_logger().warn(
				f"{label} is waiting for arm readiness: {last_message}"
			)
			if not self._sleep_before_retry(deadline):
				return False, last_message

		return False, last_message or f"Interrupted while calling {label}"

	def call_set_normal_mode_with_detail(self, timeout_s: float) -> tuple[bool, str]:
		return self.call_mode_trigger_with_auto_enable(
			self.set_normal_mode_client,
			"set_normal_mode",
			timeout_s,
		)

	def call_set_normal_mode(self, timeout_s: float) -> bool:
		success, _ = self.call_set_normal_mode_with_detail(timeout_s)
		return success

	def call_set_leader_mode(self, timeout_s: float) -> tuple[bool, str]:
		return self.call_mode_trigger_with_auto_enable(
			self.set_leader_mode_client,
			"set_leader_mode",
			timeout_s,
		)

	def call_trigger_client(self, client, label: str, timeout_s: float) -> tuple[bool, str]:
		if not client.wait_for_service(timeout_sec=timeout_s):
			return False, f"Service {label} is not available"
		future = client.call_async(Trigger.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False, f"Timed out calling {label}"
		response = future.result()
		return bool(response.success), response.message

	def call_empty_client(self, client, label: str, timeout_s: float) -> tuple[bool, str]:
		if not client.wait_for_service(timeout_sec=timeout_s):
			return False, f"Service {label} is not available"
		future = client.call_async(Empty.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			return False, f"Timed out calling {label}"
		return True, ""

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

	def _library_pattern(self) -> str:
		if not self.name_prefix:
			return "*.json"
		return f"{sanitize_trajectory_name(self.name_prefix)}*.json"

	def refresh_library(self) -> None:
		self.library_dir.mkdir(parents=True, exist_ok=True)
		self.trajectory_paths = sorted(self.library_dir.glob(self._library_pattern()), key=lambda path: path.name)
		if not self.trajectory_paths:
			self.selected_index = 0
			return
		self.selected_index = min(self.selected_index, len(self.trajectory_paths) - 1)

	def selected_trajectory_path(self) -> Optional[Path]:
		if not self.trajectory_paths:
			return None
		return self.trajectory_paths[self.selected_index]

	def next_trajectory_name(self) -> str:
		base_prefix = sanitize_trajectory_name(self.name_prefix or "wakeword")
		existing = {path.stem for path in self.trajectory_paths}
		for index in range(1, 1000):
			candidate = sanitize_trajectory_name(f"{base_prefix}_{index:02d}")
			if candidate not in existing:
				return candidate
		return sanitize_trajectory_name(f"{base_prefix}_{int(time.time())}")

	def select_previous(self) -> None:
		if not self.trajectory_paths:
			return
		self.selected_index = (self.selected_index - 1) % len(self.trajectory_paths)
		self.print_status()

	def select_next(self) -> None:
		if not self.trajectory_paths:
			return
		self.selected_index = (self.selected_index + 1) % len(self.trajectory_paths)
		self.print_status()

	def select_by_digit(self, key: str) -> None:
		if key == "0" or not self.trajectory_paths:
			return
		requested_index = int(key) - 1
		if requested_index >= len(self.trajectory_paths):
			self.get_logger().warn(f"No trajectory slot {key}; {len(self.trajectory_paths)} sample(s) available")
			return
		self.selected_index = requested_index
		self.print_status()

	def ensure_arm_enabled(self) -> None:
		if not self.args.auto_enable_arm:
			return
		if self.arm_enable_verified:
			return
		success, message = self.call_enable_arm(True, self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to enable arm through enable_agx_arm{detail}")

	def disable_mit(self, required: bool = True) -> None:
		if not self.enable_client.wait_for_service(timeout_sec=0.0):
			if required:
				raise RuntimeError("Failed to disable MIT controller: service mit_controller/enable is not available")
			self.get_logger().info("MIT controller service is not available; skipping MIT disable")
			return

		success, message = self.call_enable_mit(False, self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to disable MIT controller{detail}")

	def enter_idle_mode(self) -> None:
		self.disable_mit(required=False)
		success, message = self.call_set_normal_mode_with_detail(self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to switch robot to normal mode before entering idle{detail}")
		success, message = self.call_set_leader_mode(self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to switch robot to leader mode for idle{detail}")
		if not self.wait_for_leader_feedback(self.args.feedback_timeout):
			raise RuntimeError("Did not receive feedback/leader_joint_angles in idle mode")
		self.state = ManagerState.IDLE
		self.get_logger().info("State changed to idle")
		self.print_status()

	def enter_record_mode(self) -> None:
		self.disable_mit(required=False)
		success, message = self.call_set_normal_mode_with_detail(self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to switch robot to normal mode before entering record mode{detail}")
		success, message = self.call_set_leader_mode(self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to switch robot to leader mode for teaching{detail}")
		if not self.wait_for_leader_feedback(self.args.feedback_timeout):
			raise RuntimeError("Did not receive feedback/leader_joint_angles in record mode")
		self.state = ManagerState.RECORD
		self.get_logger().info("State changed to record")
		self.print_status()

	def enter_playback_mode(self) -> None:
		success, message = self.call_set_normal_mode_with_detail(self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to switch robot to normal mode before entering playback mode{detail}")
		if not self.wait_for_fresh_joint_state(self.args.feedback_timeout):
			raise RuntimeError("Did not receive fresh feedback/joint_states in playback mode")
		success, message = self.call_enable_mit(True, self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to enable MIT controller for playback{detail}")
		success, message = self.call_empty_client(
			self.hold_current_client,
			"mit_controller/hold_current",
			self.args.service_timeout,
		)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to hold current pose for playback{detail}")
		self.state = ManagerState.PLAYBACK
		self.get_logger().info("State changed to playback")
		self.print_status()

	def hold_current_pose(self) -> None:
		if not self.call_set_normal_mode(self.args.service_timeout):
			raise RuntimeError("Failed to switch robot to normal mode before hold_current")
		if not self.wait_for_fresh_joint_state(self.args.feedback_timeout):
			raise RuntimeError("Did not receive fresh feedback/joint_states before hold_current")
		success, message = self.call_enable_mit(True, self.args.service_timeout)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to enable MIT controller for hold_current{detail}")
		success, message = self.call_empty_client(
			self.hold_current_client,
			"mit_controller/hold_current",
			self.args.service_timeout,
		)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to capture hold_current target{detail}")
		self.get_logger().info("MIT hold_current captured the current pose")

	def cancel_active_trajectory(self) -> None:
		success, message = self.call_empty_client(
			self.cancel_trajectory_client,
			"mit_controller/cancel_trajectory",
			self.args.service_timeout,
		)
		if not success:
			detail = f": {message}" if message else ""
			raise RuntimeError(f"Failed to cancel active MIT trajectory{detail}")
		self.get_logger().info("Cancelled active MIT trajectory")

	def record_sample(self) -> Path:
		if self.state != ManagerState.RECORD:
			raise RuntimeError("Recording is only available in record mode")
		if not self.wait_for_leader_feedback(self.args.feedback_timeout):
			raise RuntimeError("Did not receive feedback/leader_joint_angles before recording")

		trajectory_name = self.next_trajectory_name()
		self.get_logger().info(f"Recording new wakeword motion '{trajectory_name}'")
		samples, raw_sample_count = record_trajectory(
			self,
			sample_rate=self.args.sample_rate,
			hold_timeout=self.args.hold_timeout,
			movement_threshold=self.args.movement_threshold,
			wait_for_enter=False,
		)
		joint_names = list(self.leader_joint_state.name) if self.leader_joint_state is not None else []
		if not joint_names:
			raise RuntimeError("leader_joint_angles does not contain joint names after recording")

		trajectory = build_recorded_trajectory(
			name=trajectory_name,
			joint_names=joint_names,
			sample_rate=self.args.sample_rate,
			hold_timeout=self.args.hold_timeout,
			movement_threshold=self.args.movement_threshold,
			samples=samples,
			raw_sample_count=raw_sample_count,
			urdf_path=self.args.urdf_path or None,
			metadata={
				"manager": "agx_arm_motion_manager",
				"variant_group": sanitize_trajectory_name(self.name_prefix or "wakeword"),
			},
		)
		saved_path = save_recorded_trajectory(trajectory, self.library_dir / f"{trajectory_name}.json")
		self.refresh_library()
		if saved_path in self.trajectory_paths:
			self.selected_index = self.trajectory_paths.index(saved_path)
		self.get_logger().info(f"Saved wakeword motion to {saved_path}")
		self.print_status()
		return saved_path

	def delete_selected_sample(self) -> None:
		target = self.selected_trajectory_path()
		if target is None:
			raise RuntimeError("No saved wakeword motion is selected for deletion")
		target.unlink(missing_ok=False)
		self.refresh_library()
		self.get_logger().info(f"Deleted wakeword motion {target}")
		self.print_status()

	def choose_trigger_path(self) -> Path:
		self.refresh_library()
		if not self.trajectory_paths:
			raise RuntimeError(f"No recorded motions found in {self.library_dir}")
		if self.random_selection:
			return self.random_generator.choice(self.trajectory_paths)
		selected = self.selected_trajectory_path()
		if selected is None:
			raise RuntimeError("No deterministic wakeword motion is selected")
		return selected

	def validate_trigger_request(self) -> None:
		if self.state != ManagerState.PLAYBACK:
			raise RuntimeError(f"Wakeword trigger ignored while state={self.state.value}; switch to playback first")
		if self.external_trigger_pending:
			raise RuntimeError("Wakeword trigger request is already pending")

		now = time.monotonic()
		if self.args.trigger_cooldown > 0.0 and (now - self.last_trigger_time_monotonic) < self.args.trigger_cooldown:
			raise RuntimeError("Wakeword trigger cooldown is still active")

		self.refresh_library()
		if not self.trajectory_paths:
			raise RuntimeError(f"No recorded motions found in {self.library_dir}")
		if not self.random_selection and self.selected_trajectory_path() is None:
			raise RuntimeError("No deterministic wakeword motion is selected")

	def request_external_trigger(self) -> None:
		self.validate_trigger_request()
		self.external_trigger_pending = True

	def process_pending_external_trigger(self) -> None:
		if not self.external_trigger_pending:
			return

		self.external_trigger_pending = False
		try:
			trajectory_path = self.trigger_motion()
		except RuntimeError as exc:
			self.get_logger().error(f"Queued wakeword trigger failed: {exc}")
			return

		self.get_logger().info(f"Processed queued wakeword trigger for {trajectory_path.name}")

	def trigger_motion(self) -> Path:
		self.validate_trigger_request()
		now = time.monotonic()
		trajectory_path = self.choose_trigger_path()
		execute_recorded_trajectory(
			self,
			trajectory_path,
			service_timeout=self.args.service_timeout,
			feedback_timeout=self.args.feedback_timeout,
			publish_repetitions=self.args.publish_repetitions,
			publish_interval=self.args.publish_interval,
		)
		self.last_trigger_time_monotonic = now
		self.last_triggered_path = trajectory_path
		self.get_logger().info(f"Triggered wakeword motion {trajectory_path.name}")
		self.print_status()
		return trajectory_path

	def _trigger_motion_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
		del request
		try:
			self.request_external_trigger()
		except RuntimeError as exc:
			response.success = False
			response.message = str(exc)
		else:
			response.success = True
			response.message = "Wakeword trigger accepted"
		return response

	def handle_key(self, key: str) -> None:
		if key == "q":
			self.shutdown_requested = True
			return
		if key == "h":
			self.print_help()
			return
		if key == "s":
			self.print_status()
			return
		if key == "i":
			self.enter_idle_mode()
			return
		if key == "r":
			self.enter_record_mode()
			return
		if key == "p":
			self.enter_playback_mode()
			return
		if key == "[":
			self.select_previous()
			return
		if key == "]":
			self.select_next()
			return
		if key.isdigit():
			self.select_by_digit(key)
			return
		if key == "m":
			self.random_selection = not self.random_selection
			mode_label = "random" if self.random_selection else "deterministic"
			self.get_logger().info(f"Playback selection mode set to {mode_label}")
			self.print_status()
			return

		if self.state == ManagerState.RECORD:
			if key == "n":
				self.record_sample()
				return
			if key == "x":
				self.delete_selected_sample()
				return
			if key == "g":
				self.hold_current_pose()
				return
			if key == "t":
				self.enter_record_mode()
				return

		if self.state == ManagerState.PLAYBACK:
			if key == "f":
				self.trigger_motion()
				return
			if key == "c":
				self.cancel_active_trajectory()
				return
			if key == "g":
				self.hold_current_pose()
				return

		self.get_logger().warn(f"Unhandled key '{key}' in state={self.state.value}; press 'h' for help")

	def print_help(self) -> None:
		print(
			"\nKeyboard mapping:\n"
			"  i -> idle (leader mode, MIT disabled)\n"
			"  r -> record mode\n"
			"  p -> playback mode\n"
			"  h -> help\n"
			"  s -> status\n"
			"  q -> quit\n"
			"  [ / ] -> previous / next sample\n"
			"  1..9 -> select sample slot\n"
			"  m -> toggle deterministic/random selection\n"
			"Record mode:\n"
			"  n -> record a new sample immediately\n"
			"  x -> delete selected sample\n"
			"  g -> switch to MIT hold_current (gummi mode)\n"
			"  t -> return to leader teach mode\n"
			"Playback mode:\n"
			"  f -> fire the selected/random sample now\n"
			"  c -> cancel active MIT trajectory\n"
			"  g -> refresh MIT hold_current\n"
			"Service trigger:\n"
			"  ros2 service call /agx_arm_motion_manager/trigger_motion std_srvs/srv/Trigger \"{}\"\n"
		)

	def print_status(self) -> None:
		self.refresh_library()
		selected = self.selected_trajectory_path()
		selection_mode = "random" if self.random_selection else "deterministic"
		selected_label = selected.name if selected is not None else "<none>"
		last_triggered = self.last_triggered_path.name if self.last_triggered_path is not None else "<none>"
		print(
			"Status: "
			f"state={self.state.value}, "
			f"selection={selection_mode}, "
			f"samples={len(self.trajectory_paths)}, "
			f"selected={selected_label}, "
			f"last_triggered={last_triggered}, "
			f"library={self.library_dir}"
		)

	def run(self) -> None:
		start_state = ManagerState(self.args.start_mode)
		self.wait_for_required_services(start_state)
		if start_state == ManagerState.IDLE:
			self.enter_idle_mode()
		elif start_state == ManagerState.RECORD:
			self.enter_record_mode()
		else:
			self.enter_playback_mode()
		self.runtime_active = True

		self.print_help()
		with TerminalKeyReader(enabled=not self.args.no_keyboard) as key_reader:
			if not key_reader.enabled:
				self.get_logger().info("Keyboard input disabled; use the trigger service and ROS CLI to control the manager")

			while rclpy.ok() and not self.shutdown_requested:
				rclpy.spin_once(self, timeout_sec=0.1)
				if self.external_trigger_pending:
					self.process_pending_external_trigger()
					continue
				key = key_reader.read_key()
				if key is None:
					continue
				try:
					self.handle_key(key)
				except RuntimeError as exc:
					self.get_logger().error(str(exc))


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Interactive wakeword motion manager for Nero record/playback workflows")
	parser.add_argument("--library-dir", default="~/agx_arm_trajectories/wakeword", help="Directory that stores taught wakeword trajectories")
	parser.add_argument("--name-prefix", default="wakeword", help="Prefix for taught trajectory filenames")
	parser.add_argument("--start-mode", choices=[state.value for state in ManagerState], default=ManagerState.IDLE.value)
	parser.add_argument("--sample-rate", type=float, default=50.0, help="Recording sample rate in Hz")
	parser.add_argument("--hold-timeout", type=float, default=3.0, help="Stop recording after this much stationary time")
	parser.add_argument("--movement-threshold", type=float, default=0.01, help="Movement threshold in rad for recording")
	parser.add_argument("--service-timeout", type=float, default=5.0, help="Timeout for ROS service calls")
	parser.add_argument("--feedback-timeout", type=float, default=3.0, help="Timeout waiting for fresh feedback topics")
	parser.add_argument("--publish-repetitions", type=int, default=3, help="How often to republish a triggered trajectory")
	parser.add_argument("--publish-interval", type=float, default=0.2, help="Seconds between repeated trajectory publishes")
	parser.add_argument("--trigger-cooldown", type=float, default=0.0, help="Minimum time between accepted trigger requests")
	parser.add_argument("--startup-timeout", type=float, default=0.0, help="How long to wait for the services needed by the start mode; 0 waits forever")
	parser.add_argument("--random-selection", action="store_true", help="Randomize which saved sample gets played in playback mode")
	parser.add_argument("--auto-enable-arm", action="store_true", help="Call enable_agx_arm before switching controller modes")
	parser.add_argument("--no-keyboard", action="store_true", help="Disable terminal key handling and run headless")
	parser.add_argument("--urdf-path", default="", help="Optional path to a Nero URDF used for recording metadata")
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	if args.sample_rate <= 0.0:
		raise ValueError("--sample-rate must be > 0")
	if args.hold_timeout <= 0.0:
		raise ValueError("--hold-timeout must be > 0")
	if args.movement_threshold <= 0.0:
		raise ValueError("--movement-threshold must be > 0")
	if args.feedback_timeout <= 0.0:
		raise ValueError("--feedback-timeout must be > 0")
	if args.service_timeout <= 0.0:
		raise ValueError("--service-timeout must be > 0")
	if args.publish_repetitions <= 0:
		raise ValueError("--publish-repetitions must be > 0")
	if args.publish_interval < 0.0:
		raise ValueError("--publish-interval must be >= 0")
	if args.trigger_cooldown < 0.0:
		raise ValueError("--trigger-cooldown must be >= 0")
	if args.startup_timeout < 0.0:
		raise ValueError("--startup-timeout must be >= 0")

	rclpy.init()
	node = WakewordMotionManagerNode(args)
	try:
		node.run()
	finally:
		try:
			if node.runtime_active and rclpy.ok():
				node.enter_idle_mode()
		except Exception:
			pass
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = ["main"]