"""Central teach manager for the Hefeweizen teach loop (record/playback + teach).

One interactive keyboard tool — modelled on ``wakeword_motion_manager`` — that
covers the whole teach loop for the coordinated demo from a single process,
instead of three separate CLIs:

- **freedrive / leader mode** (``idle``): MIT off, the arm is back-drivable.
- **record** a leader trajectory to the library (``record`` mode, key ``n``).
- **capture** the current joint vector as a named anchor pose written straight
  into ``agx_arm_coordination/config/arm_config.yaml`` (key ``a``).
- **playback** a recorded trajectory through the MIT controller to test it
  (``playback`` mode, key ``f``).
- **convert** a recording into catalogue ``waypoints`` for a chosen action
  (key ``w`` -> ``recorded_to_catalogue``).

It reuses the proven building blocks rather than re-implementing motion code:
``leader_trajectory_recorder`` (record), ``execute_saved_trajectory`` (playback +
MIT/leader service plumbing via :class:`SavedTrajectoryExecutorNode`),
``capture_anchor_pose`` (anchor write), and ``recorded_to_catalogue`` (waypoints).

Right-side bring-up example (7-DoF right arm, real hand bus already up)::

    ros2 run agx_arm_mit_demos agx_arm_teach_manager \\
        --source-joints right_arm_joint1,right_arm_joint2,right_arm_joint3,\\
right_arm_joint4,right_arm_joint5,right_arm_joint6,right_arm_joint7 \\
        --arm-config src/agx_arm_coordination/config/arm_config.yaml

See ``docs/development/sprint6/planning/teach_and_run_right_side.md``.
"""

from __future__ import annotations

import argparse
from enum import Enum
from pathlib import Path
import time
from typing import Optional

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger

from agx_arm_mit_controller.trajectory_io import (
    load_recorded_trajectory,
    sanitize_trajectory_name,
    save_recorded_trajectory,
)

from .capture_anchor_pose import update_pose_in_config
from .execute_saved_trajectory import SavedTrajectoryExecutorNode, execute_recorded_trajectory
from .leader_trajectory_recorder import build_recorded_trajectory, record_trajectory
from .recorded_to_catalogue import format_waypoints_block, recorded_to_waypoints
from .wakeword_motion_manager import TerminalKeyReader


class ManagerState(str, Enum):
    IDLE = "idle"
    RECORD = "record"
    PLAYBACK = "playback"


class TeachManagerNode(SavedTrajectoryExecutorNode):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(node_name="agx_arm_teach_manager")
        self.args = args
        self.library_dir = Path(args.library_dir).expanduser().resolve()
        self.arm_config_path = Path(args.arm_config).expanduser().resolve() if args.arm_config else None
        self.source_joints = [j.strip() for j in args.source_joints.split(",") if j.strip()]
        self.state = ManagerState.IDLE
        self.shutdown_requested = False
        self.runtime_active = False
        self.selected_index = 0
        self.trajectory_paths: list[Path] = []
        self.leader_joint_state: Optional[JointState] = None
        self.source_joint_state: Optional[JointState] = None

        self.enable_arm_client = self.create_client(SetBool, "enable_agx_arm")
        self.set_leader_mode_client = self.create_client(Trigger, "set_leader_mode")
        self.hold_current_client = self.create_client(Empty, "mit_controller/hold_current")
        self.cancel_trajectory_client = self.create_client(Empty, "mit_controller/cancel_trajectory")
        self.create_subscription(JointState, "feedback/leader_joint_angles", self._leader_callback, 20)
        self.create_subscription(JointState, args.source_topic, self._source_callback, 20)

        self.refresh_library()

    # --- callbacks -----------------------------------------------------------

    def _leader_callback(self, msg: JointState) -> None:
        self.leader_joint_state = msg

    def _source_callback(self, msg: JointState) -> None:
        self.source_joint_state = msg

    # --- service helpers -----------------------------------------------------

    def call_trigger(self, client, label: str, timeout_s: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"service {label} not available"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, f"timed out calling {label}"
        response = future.result()
        return bool(response.success), response.message

    def call_empty(self, client, label: str, timeout_s: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"service {label} not available"
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, f"timed out calling {label}"
        return True, ""

    def call_enable_arm(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
        if not self.enable_arm_client.wait_for_service(timeout_sec=timeout_s):
            return False, "enable_agx_arm not available"
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, "timed out calling enable_agx_arm"
        response = future.result()
        return bool(response.success), response.message

    def ensure_arm_enabled(self) -> None:
        if not self.args.auto_enable_arm:
            return
        success, message = self.call_enable_arm(True, self.args.service_timeout)
        if not success:
            raise RuntimeError(f"failed to enable arm: {message}")

    def wait_for_leader_feedback(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.leader_joint_state is not None:
                return True
        return False

    # --- mode transitions ----------------------------------------------------

    def _enter_leader(self, label: str) -> None:
        self.ensure_arm_enabled()
        if not self.call_set_normal_mode(self.args.service_timeout):
            raise RuntimeError("failed to switch to normal mode")
        ok, msg = self.call_trigger(self.set_leader_mode_client, "set_leader_mode", self.args.service_timeout)
        if not ok:
            raise RuntimeError(f"failed to switch to leader mode for {label}: {msg}")
        if not self.wait_for_leader_feedback(self.args.feedback_timeout):
            raise RuntimeError("did not receive feedback/leader_joint_angles")

    def enter_idle_mode(self) -> None:
        self._enter_leader("idle")
        self.state = ManagerState.IDLE
        self.get_logger().info("State -> idle (freedrive, MIT off)")
        self.print_status()

    def enter_record_mode(self) -> None:
        self._enter_leader("record")
        self.state = ManagerState.RECORD
        self.get_logger().info("State -> record (leader mode; press 'n' to record)")
        self.print_status()

    def enter_playback_mode(self) -> None:
        if not self.call_set_normal_mode(self.args.service_timeout):
            raise RuntimeError("failed to switch to normal mode before playback")
        if not self.wait_for_fresh_joint_state(self.args.feedback_timeout):
            raise RuntimeError("did not receive fresh feedback/joint_states")
        enabled, detail = self.call_enable_mit(True, self.args.service_timeout)
        if not enabled:
            raise RuntimeError(f"failed to enable MIT controller: {detail}")
        ok, msg = self.call_empty(self.hold_current_client, "mit_controller/hold_current", self.args.service_timeout)
        if not ok:
            raise RuntimeError(f"failed to hold current pose: {msg}")
        self.state = ManagerState.PLAYBACK
        self.get_logger().info("State -> playback (MIT on, holding current)")
        self.print_status()

    # --- library -------------------------------------------------------------

    def refresh_library(self) -> None:
        self.library_dir.mkdir(parents=True, exist_ok=True)
        self.trajectory_paths = sorted(self.library_dir.glob("*.json"), key=lambda p: p.name)
        if self.trajectory_paths:
            self.selected_index = min(self.selected_index, len(self.trajectory_paths) - 1)
        else:
            self.selected_index = 0

    def selected_trajectory_path(self) -> Optional[Path]:
        if not self.trajectory_paths:
            return None
        return self.trajectory_paths[self.selected_index]

    def select_next(self, step: int) -> None:
        if not self.trajectory_paths:
            return
        self.selected_index = (self.selected_index + step) % len(self.trajectory_paths)
        self.print_status()

    # --- teach actions -------------------------------------------------------

    def record_sample(self, key_reader: TerminalKeyReader) -> None:
        if self.state != ManagerState.RECORD:
            raise RuntimeError("recording is only available in record mode")
        if not self.wait_for_leader_feedback(self.args.feedback_timeout):
            raise RuntimeError("no feedback/leader_joint_angles before recording")
        default_name = sanitize_trajectory_name(time.strftime("teach_%Y%m%d_%H%M%S"))
        name = sanitize_trajectory_name(
            self.prompt_line(key_reader, f"Trajectory name [{default_name}]: ") or default_name
        )
        self.get_logger().info(f"Recording '{name}' — move the arm; auto-stops after hold timeout")
        samples, raw_count = record_trajectory(
            self,
            sample_rate=self.args.sample_rate,
            hold_timeout=self.args.hold_timeout,
            movement_threshold=self.args.movement_threshold,
            wait_for_enter=False,
        )
        joint_names = list(self.leader_joint_state.name) if self.leader_joint_state is not None else []
        if not joint_names:
            raise RuntimeError("leader_joint_angles has no joint names after recording")
        trajectory = build_recorded_trajectory(
            name=name,
            joint_names=joint_names,
            sample_rate=self.args.sample_rate,
            hold_timeout=self.args.hold_timeout,
            movement_threshold=self.args.movement_threshold,
            samples=samples,
            raw_sample_count=raw_count,
            urdf_path=self.args.urdf_path or None,
            metadata={"manager": "agx_arm_teach_manager"},
        )
        saved = save_recorded_trajectory(trajectory, self.library_dir / f"{name}.json")
        self.refresh_library()
        if saved in self.trajectory_paths:
            self.selected_index = self.trajectory_paths.index(saved)
        self.get_logger().info(f"Saved {saved}")
        self.print_status()

    def capture_anchor(self, key_reader: TerminalKeyReader) -> None:
        if self.arm_config_path is None:
            raise RuntimeError("--arm-config not set; cannot write anchor poses")
        if not self.source_joints:
            raise RuntimeError("--source-joints not set; cannot order the captured vector")
        pose_name = self.prompt_line(key_reader, "Anchor pose name (e.g. Pre_Grip_R): ").strip()
        if not pose_name:
            self.get_logger().warn("no pose name given; capture aborted")
            return
        averaged = self._collect_source_positions()
        vector = [averaged[name] for name in self.source_joints]
        note = update_pose_in_config(self.arm_config_path, pose_name, vector, self.args.precision)
        formatted = "[" + ", ".join(f"{v:.{self.args.precision}f}" for v in vector) + "]"
        self.get_logger().info(f"{note}: {pose_name} = {formatted}")
        self.get_logger().info("rebuild agx_arm_coordination (or symlink-install) for a launched coordinator")

    def _collect_source_positions(self) -> dict[str, float]:
        want = self.source_joints
        sums = {n: 0.0 for n in want}
        counts = {n: 0 for n in want}
        seen: set[str] = set()
        deadline = time.monotonic() + self.args.feedback_timeout
        settle_end: Optional[float] = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.source_joint_state
            if msg is None:
                continue
            seen.update(msg.name)
            if settle_end is None:
                settle_end = time.monotonic() + self.args.settle_sec
            pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
            for name in want:
                if name in pos:
                    sums[name] += pos[name]
                    counts[name] += 1
            if settle_end is not None and time.monotonic() >= settle_end:
                break
        missing = [n for n in want if counts[n] == 0]
        if missing:
            raise RuntimeError(
                f"joints not on {self.args.source_topic}: {missing}; saw {sorted(seen) or '(none)'}"
            )
        return {n: sums[n] / counts[n] for n in want}

    def convert_to_waypoints(self, key_reader: TerminalKeyReader) -> None:
        path = self.selected_trajectory_path()
        if path is None:
            raise RuntimeError("no trajectory selected to convert")
        action_id = self.prompt_line(key_reader, "Catalogue action_id for the waypoints: ").strip()
        if not action_id:
            self.get_logger().warn("no action_id given; conversion aborted")
            return
        trajectory = load_recorded_trajectory(path)
        if not trajectory.points:
            raise RuntimeError(f"recording '{trajectory.name}' has no points")
        waypoints = recorded_to_waypoints(trajectory, self.args.max_waypoints, self.args.precision)
        block = format_waypoints_block(action_id, trajectory, waypoints)
        out_path = path.with_name(f"{action_id}.waypoints.yaml")
        out_path.write_text(block, encoding="utf-8")
        print("\n" + block)
        self.get_logger().info(f"wrote {out_path}; paste the block under '{action_id}' in catalogue.yaml")

    def playback_selected(self) -> None:
        if self.state != ManagerState.PLAYBACK:
            raise RuntimeError("switch to playback mode first")
        path = self.selected_trajectory_path()
        if path is None:
            raise RuntimeError(f"no recordings in {self.library_dir}")
        execute_recorded_trajectory(
            self,
            path,
            service_timeout=self.args.service_timeout,
            feedback_timeout=self.args.feedback_timeout,
            publish_repetitions=self.args.publish_repetitions,
            publish_interval=self.args.publish_interval,
        )
        self.get_logger().info(f"Played {path.name}")
        self.print_status()

    def cancel_active(self) -> None:
        ok, msg = self.call_empty(
            self.cancel_trajectory_client, "mit_controller/cancel_trajectory", self.args.service_timeout
        )
        if not ok:
            raise RuntimeError(f"failed to cancel trajectory: {msg}")
        self.get_logger().info("cancelled active MIT trajectory")

    # --- terminal ------------------------------------------------------------

    def prompt_line(self, key_reader: TerminalKeyReader, prompt: str) -> str:
        """Read a full line, temporarily leaving single-key (cbreak) mode."""
        import sys
        import termios

        if not key_reader.enabled or key_reader.fd is None:
            return ""
        termios.tcsetattr(key_reader.fd, termios.TCSADRAIN, key_reader._settings)
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            return sys.stdin.readline().rstrip("\n")
        finally:
            import tty

            tty.setcbreak(key_reader.fd)

    def handle_key(self, key: str, key_reader: TerminalKeyReader) -> None:
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
            self.select_next(-1)
            return
        if key == "]":
            self.select_next(1)
            return
        if key == "a":
            self.capture_anchor(key_reader)
            return
        if key == "w":
            self.convert_to_waypoints(key_reader)
            return
        if self.state == ManagerState.RECORD and key == "n":
            self.record_sample(key_reader)
            return
        if self.state == ManagerState.PLAYBACK and key == "f":
            self.playback_selected()
            return
        if self.state == ManagerState.PLAYBACK and key == "c":
            self.cancel_active()
            return
        self.get_logger().warn(f"unhandled key '{key}' in state={self.state.value}; press 'h' for help")

    def print_help(self) -> None:
        print(
            "\nTeach manager keys:\n"
            "  i -> idle / freedrive (leader mode, MIT off)\n"
            "  r -> record mode      p -> playback mode\n"
            "  a -> capture current pose as a named anchor (-> arm_config.yaml)\n"
            "  w -> convert selected recording -> catalogue waypoints\n"
            "  [ / ] -> select previous / next recording\n"
            "  s -> status   h -> help   q -> quit\n"
            "Record mode:   n -> record a new trajectory\n"
            "Playback mode: f -> play selected   c -> cancel active trajectory\n"
        )

    def print_status(self) -> None:
        self.refresh_library()
        selected = self.selected_trajectory_path()
        print(
            "Status: "
            f"state={self.state.value}, "
            f"recordings={len(self.trajectory_paths)}, "
            f"selected={selected.name if selected else '<none>'}, "
            f"arm_config={'set' if self.arm_config_path else '<none>'}, "
            f"library={self.library_dir}"
        )

    def run(self) -> None:
        start_state = ManagerState(self.args.start_mode)
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
                self.get_logger().info("keyboard disabled; nothing to drive headless — exiting")
                return
            while rclpy.ok() and not self.shutdown_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
                key = key_reader.read_key()
                if key is None:
                    continue
                try:
                    self.handle_key(key, key_reader)
                except RuntimeError as exc:
                    self.get_logger().error(str(exc))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Central teach manager (record/playback + anchor/waypoint teach)")
    parser.add_argument("--library-dir", default="~/agx_arm_trajectories/teach", help="Directory for taught trajectories")
    parser.add_argument("--arm-config", default="", help="Path to arm_config.yaml for anchor-pose capture")
    parser.add_argument("--source-joints", default="", help="Comma-separated joint names captured for anchor poses, in stored order")
    parser.add_argument("--source-topic", default="feedback/joint_states", help="JointState topic for anchor capture")
    parser.add_argument("--start-mode", choices=[s.value for s in ManagerState], default=ManagerState.IDLE.value)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--hold-timeout", type=float, default=3.0)
    parser.add_argument("--movement-threshold", type=float, default=0.01)
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--settle-sec", type=float, default=0.5, help="Averaging window for anchor capture")
    parser.add_argument("--precision", type=int, default=5)
    parser.add_argument("--max-waypoints", type=int, default=8, help="Downsample target for waypoint conversion")
    parser.add_argument("--publish-repetitions", type=int, default=3)
    parser.add_argument("--publish-interval", type=float, default=0.2)
    parser.add_argument("--auto-enable-arm", action="store_true", help="Call enable_agx_arm before mode switches")
    parser.add_argument("--no-keyboard", action="store_true")
    parser.add_argument("--urdf-path", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sample_rate <= 0.0:
        raise ValueError("--sample-rate must be > 0")

    rclpy.init()
    node = TeachManagerNode(args)
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


__all__ = ["main", "TeachManagerNode", "ManagerState"]
