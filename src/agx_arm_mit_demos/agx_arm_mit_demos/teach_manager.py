"""Central teach manager for the Hefeweizen teach loop (record/playback + teach).

One interactive keyboard tool — modelled on ``wakeword_motion_manager`` — that
covers the whole teach loop for the coordinated demo from a single process:

- **freedrive** (``idle``): MIT drives a zero-force, gravity-compensated command
  (mounting-pose aware), so the arm(s) are back-drivable.
- **record** a taught trajectory (``record`` mode, key ``n``).
- **capture** the current joint vector as a named anchor pose written straight
  into ``agx_arm_coordination/config/arm_config.yaml`` (key ``a``).
- **playback** a recorded trajectory to test it (``playback`` mode, key ``f``).
- **convert** a recording into catalogue ``waypoints`` (key ``w``).

Arm-count-aware: pass one namespace per arm via ``--arms``. Each arm is its own
namespaced MIT stack (``/<ns>/mit_controller/...``, ``/<ns>/feedback/...``) that
internally uses the **unprefixed** joints ``joint1..7`` — the namespace, not a
joint prefix, separates the arms. With two arms the record path captures both on
one clock; at save time you choose the resource the recording is stored as
(``both_arms`` -> merged 14-dim, or one side -> 7-dim).

Single right arm (backward-compatible default, empty namespace)::

    ros2 run agx_arm_mit_demos agx_arm_teach_manager \\
        --arm-config src/agx_arm_coordination/config/arm_config.yaml \\
        --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7

Both arms (namespaced bring-up: launch the MIT stack twice with
``namespace:=left_arm can_port:=can_nero_left`` and ``namespace:=right_arm
can_port:=can_nero_right``)::

    ros2 run agx_arm_mit_demos agx_arm_teach_manager \\
        --arm-config src/agx_arm_coordination/config/arm_config.yaml \\
        --arms left_arm right_arm \\
        --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7

See ``docs/control/bringups/teach_and_run.md``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_mit_controller.model_metadata import compute_flange_pose_from_mdh
from agx_arm_mit_controller.trajectory_io import (
    RecordedTrajectory,
    load_recorded_trajectory,
    recorded_to_joint_trajectory,
    sanitize_trajectory_name,
    save_recorded_trajectory,
    smooth_recorded_trajectory,
)
from agx_arm_coordination.arm_executor import ArmConfig

from .capture_anchor_pose import average_joint_positions, update_pose_in_config
from .leader_trajectory_recorder import RecorderSnapshot, build_recorded_trajectory
from .recorded_to_catalogue import build_duo_trajectory, format_waypoints_block, recorded_to_waypoints
from .wakeword_motion_manager import TerminalKeyReader


class ManagerState(str, Enum):
    IDLE = "idle"
    RECORD = "record"
    PLAYBACK = "playback"
    TRANSITIONS = "transitions"


@dataclass(frozen=True)
class TransitionTarget:
    label: str
    robot_id: str
    planning_group: str
    joint_names: tuple[str, ...]
    pose_names: tuple[str, ...]
    target_positions: tuple[float, ...]


# Registry side order for a both_arms concatenation (left then right).
_SIDE_ORDER = {"left_arm": 0, "right_arm": 1}


def _transition_robot_ids(namespaces: list[str]) -> tuple[str, ...]:
    ids: list[str] = []
    names = [ns for ns in namespaces if ns]
    if not names:
        return ("right_arm",)
    if {"left_arm", "right_arm"}.issubset(set(names)):
        ids.append("both_arms")
    for name in names:
        if name in {"left_arm", "right_arm"}:
            ids.append(name)
    if not ids:
        return ("right_arm",)
    return tuple(dict.fromkeys(ids))


def _pose_vector(config: ArmConfig, pose_names: tuple[str, ...]) -> tuple[float, ...]:
    values: list[float] = []
    for pose_name in pose_names:
        if pose_name not in config.poses:
            raise ValueError(f"unknown anchor pose '{pose_name}'")
        values.extend(config.poses[pose_name])
    return tuple(values)


def _build_transition_targets(config: ArmConfig, namespaces: list[str]) -> list[TransitionTarget]:
    targets: list[TransitionTarget] = []
    for robot_id in _transition_robot_ids(namespaces):
        group = config.groups.get(robot_id)
        if group is None:
            continue
        if robot_id == "both_arms":
            left = {name[:-2]: name for name in config.poses if name.endswith("_L")}
            right = {name[:-2]: name for name in config.poses if name.endswith("_R")}
            for stem in sorted(set(left) & set(right)):
                pose_names = (left[stem], right[stem])
                positions = _pose_vector(config, pose_names)
                if len(positions) != len(group.joint_names):
                    continue
                targets.append(
                    TransitionTarget(
                        label=f"both_arms:{stem}",
                        robot_id=robot_id,
                        planning_group=group.planning_group,
                        joint_names=group.joint_names,
                        pose_names=pose_names,
                        target_positions=positions,
                    )
                )
            continue

        suffix = "_L" if robot_id == "left_arm" else "_R"
        for pose_name, vector in sorted(config.poses.items()):
            if not pose_name.endswith(suffix) or len(vector) != len(group.joint_names):
                continue
            targets.append(
                TransitionTarget(
                    label=pose_name,
                    robot_id=robot_id,
                    planning_group=group.planning_group,
                    joint_names=group.joint_names,
                    pose_names=(pose_name,),
                    target_positions=tuple(float(value) for value in vector),
                )
            )
    return targets


_REQUIRED_MIT_SERVICES = (
    "set_normal_mode",
    "mit_controller/enable",
    "mit_controller/freedrive",
    "mit_controller/hold_current",
)


def _discover_mit_namespaces(node: Node) -> list[str]:
    """Namespaces that currently provide the full MIT service set.

    Returns "" for an un-namespaced stack and e.g. "left_arm"/"right_arm" for
    the namespaced Duo bring-ups (components baseline), sorted left before
    right. Used to rebind the teach manager automatically when it was started
    without --arms against a namespaced bring-up.
    """
    services = {name for name, _ in node.get_service_names_and_types()}
    marker = "/set_normal_mode"
    namespaces = []
    for service_name in services:
        if not service_name.endswith(marker):
            continue
        prefix = service_name[: -len(marker)]  # "" un-namespaced, "/left_arm" otherwise
        if all(f"{prefix}/{required}" in services for required in _REQUIRED_MIT_SERVICES):
            namespaces.append(prefix.strip("/"))
    return sorted(namespaces, key=lambda ns: (_SIDE_ORDER.get(ns, 99), ns))


def _resolve_config_path(raw: str) -> Path:
    """Resolve a possibly-relative config path independent of the launch cwd.

    The documented invocation passes ``src/agx_arm_coordination/config/...``,
    which only resolves from the workspace root. A relative path is tried
    against the cwd and each of its parents, then against the directories
    above this (installed) file, so the teach manager also works when started
    from the home directory or anywhere else inside the workspace.
    """
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path.resolve()

    search_roots = [Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents]
    for root in search_roots:
        candidate = root / path
        if candidate.exists():
            return candidate.resolve()
    return path.resolve()


class _ArmEndpoint:
    """All per-arm ROS handles for one (optionally namespaced) MIT stack.

    An empty namespace resolves to the un-namespaced single-arm graph, so the
    default one-arm case is byte-for-byte the previous topic/service layout.
    """

    def __init__(self, node: Node, namespace: str, source_joints: list[str], source_topic: str) -> None:
        self.node = node
        self.namespace = namespace.strip("/")
        self.source_joints = source_joints
        self.latest: Optional[JointState] = None

        self.set_normal_mode_client = node.create_client(Trigger, self._name("set_normal_mode"))
        self.enable_mit_client = node.create_client(SetBool, self._name("mit_controller/enable"))
        self.freedrive_client = node.create_client(SetBool, self._name("mit_controller/freedrive"))
        self.hold_current_client = node.create_client(Empty, self._name("mit_controller/hold_current"))
        self.cancel_client = node.create_client(Empty, self._name("mit_controller/cancel_trajectory"))
        self.enable_arm_client = node.create_client(SetBool, self._name("enable_agx_arm"))
        self.trajectory_pub = node.create_publisher(
            JointTrajectory, self._name("mit_controller/joint_trajectory"), 10
        )
        node.create_subscription(JointState, self._name(source_topic), self._on_feedback, 20)

    def _name(self, relative: str) -> str:
        return f"/{self.namespace}/{relative}" if self.namespace else relative

    @property
    def label(self) -> str:
        return self.namespace or "arm"

    @property
    def side_prefix(self) -> str:
        """Joint prefix this arm contributes to a duo vector (``left_arm`` -> ``left_arm_``)."""
        return f"{self.namespace}_" if self.namespace else ""

    def _on_feedback(self, msg: JointState) -> None:
        self.latest = msg

    def snapshot(self, time_from_start: float) -> Optional[RecorderSnapshot]:
        msg = self.latest
        if msg is None:
            return None
        position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
        if any(joint not in position_map for joint in self.source_joints):
            return None
        positions = [position_map[joint] for joint in self.source_joints]
        flange_pose = compute_flange_pose_from_mdh(positions, robot="nero")
        return RecorderSnapshot(
            time_from_start=time_from_start,
            positions=positions,
            efforts=[0.0] * len(self.source_joints),
            flange_pose=flange_pose,
        )

    # --- service calls (spin on the owning node) ---------------------------
    def _call(self, client, request, label: str, timeout_s: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"{self._name(label)} not available"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, f"timed out calling {self._name(label)}"
        result = future.result()
        success = getattr(result, "success", True)
        message = getattr(result, "message", "")
        return bool(success), message

    def call_set_normal_mode(self, timeout_s: float) -> tuple[bool, str]:
        return self._call(self.set_normal_mode_client, Trigger.Request(), "set_normal_mode", timeout_s)

    def call_enable_mit(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = enabled
        return self._call(self.enable_mit_client, request, "mit_controller/enable", timeout_s)

    def call_freedrive(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = enabled
        return self._call(self.freedrive_client, request, "mit_controller/freedrive", timeout_s)

    def call_hold_current(self, timeout_s: float) -> tuple[bool, str]:
        return self._call(self.hold_current_client, Empty.Request(), "mit_controller/hold_current", timeout_s)

    def call_cancel(self, timeout_s: float) -> tuple[bool, str]:
        return self._call(self.cancel_client, Empty.Request(), "mit_controller/cancel_trajectory", timeout_s)

    def call_enable_arm(self, enabled: bool, timeout_s: float) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = enabled
        return self._call(self.enable_arm_client, request, "enable_agx_arm", timeout_s)

    def required_clients(self, include_enable_arm: bool) -> list[tuple[str, object]]:
        clients = [
            (self._name("set_normal_mode"), self.set_normal_mode_client),
            (self._name("mit_controller/enable"), self.enable_mit_client),
            (self._name("mit_controller/freedrive"), self.freedrive_client),
            (self._name("mit_controller/hold_current"), self.hold_current_client),
        ]
        if include_enable_arm:
            clients.append((self._name("enable_agx_arm"), self.enable_arm_client))
        return clients


class TeachManagerNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("agx_arm_teach_manager")
        self.args = args
        self.library_dir = Path(args.library_dir).expanduser().resolve()
        self.arm_config_path = _resolve_config_path(args.arm_config) if args.arm_config else None
        self.arm_config: Optional[ArmConfig] = None
        if self.arm_config_path is not None and not self.arm_config_path.exists():
            self.get_logger().warn(
                f"--arm-config resolved to {self.arm_config_path}, which does not exist; "
                "anchor capture ('a') will fail until it does"
            )
        self.source_joints = [j.strip() for j in args.source_joints.split(",") if j.strip()]
        self.state = ManagerState.IDLE
        self.shutdown_requested = False
        self.runtime_active = False
        self.selected_index = 0
        self.trajectory_paths: list[Path] = []
        self.transition_selected_index = 0
        self.transition_targets: list[TransitionTarget] = []
        self.pending_transition_plan: Optional[RobotTrajectory] = None
        self.pending_transition_target_label: Optional[str] = None

        namespaces = args.arms if args.arms else [""]
        self.arms = [
            _ArmEndpoint(self, ns, self.source_joints, args.source_topic) for ns in namespaces
        ]
        # Deterministic duo order: left before right (registry both_arms order).
        self.arms.sort(key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))

        if self.arm_config_path is not None and self.arm_config_path.exists():
            self._reload_arm_config()

        self._move_group_client = ActionClient(self, MoveGroup, self._move_group_action_name())
        self._execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, self._execute_trajectory_action_name()
        )

        self.refresh_library()

    @property
    def is_dual(self) -> bool:
        return len(self.arms) > 1

    def _move_group_action_name(self) -> str:
        return self.arm_config.move_group_action if self.arm_config is not None else "/move_action"

    def _execute_trajectory_action_name(self) -> str:
        return (
            self.arm_config.execute_trajectory_action
            if self.arm_config is not None
            else "/execute_trajectory"
        )

    def _reload_arm_config(self) -> None:
        if self.arm_config_path is None or not self.arm_config_path.exists():
            self.arm_config = None
            self.transition_targets = []
            self.transition_selected_index = 0
            self._clear_transition_plan()
            return
        self.arm_config = ArmConfig.from_file(self.arm_config_path)
        self.transition_targets = _build_transition_targets(
            self.arm_config,
            [arm.namespace for arm in self.arms],
        )
        if self.transition_targets:
            self.transition_selected_index = min(
                self.transition_selected_index,
                len(self.transition_targets) - 1,
            )
        else:
            self.transition_selected_index = 0
        self._clear_transition_plan()

    def _clear_transition_plan(self) -> None:
        self.pending_transition_plan = None
        self.pending_transition_target_label = None

    def selected_transition_target(self) -> Optional[TransitionTarget]:
        if not self.transition_targets:
            return None
        return self.transition_targets[self.transition_selected_index]

    def select_next_transition(self, step: int) -> None:
        if not self.transition_targets:
            return
        self.transition_selected_index = (self.transition_selected_index + step) % len(self.transition_targets)
        self._clear_transition_plan()
        self.print_status()

    # --- shared spin helper --------------------------------------------------

    def wait_for_source_feedback(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(arm.latest is not None for arm in self.arms):
                return True
        return False

    # --- mode transitions ----------------------------------------------------

    def _enter_freedrive(self, label: str) -> None:
        """Software leader mode on every arm: MIT zero-force + gravity feedforward.

        Honours each arm's mounting pose (baked into its MIT gravity model) and
        keeps the arm in normal mode, so feedback/joint_states stays live for
        recording and the bus watchdog.
        """
        for arm in self.arms:
            if self.args.auto_enable_arm:
                ok, msg = arm.call_enable_arm(True, self.args.service_timeout)
                if not ok:
                    raise RuntimeError(f"[{arm.label}] failed to enable arm: {msg}")
            ok, msg = arm.call_set_normal_mode(self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to switch to normal mode: {msg}")
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError(f"did not receive feedback from all arms for {label}")
        for arm in self.arms:
            ok, msg = arm.call_enable_mit(True, self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to enable MIT controller: {msg}")
            ok, msg = arm.call_freedrive(True, self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to enter freedrive: {msg}")

    def enter_idle_mode(self) -> None:
        self._enter_freedrive("idle")
        self.state = ManagerState.IDLE
        self.get_logger().info("State -> idle (gravity-compensated freedrive)")
        self.print_status()

    def enter_record_mode(self) -> None:
        self._enter_freedrive("record")
        self._clear_transition_plan()
        self.state = ManagerState.RECORD
        self.get_logger().info("State -> record (freedrive; press 'n' to record)")
        self.print_status()

    def _enter_hold_mode(self, state: ManagerState, label: str) -> None:
        for arm in self.arms:
            ok, msg = arm.call_set_normal_mode(self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to switch to normal mode before {label}: {msg}")
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError("did not receive fresh feedback from all arms")
        for arm in self.arms:
            ok, msg = arm.call_enable_mit(True, self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to enable MIT controller: {msg}")
            ok, msg = arm.call_hold_current(self.args.service_timeout)
            if not ok:
                raise RuntimeError(f"[{arm.label}] failed to hold current pose: {msg}")
        self.state = state
        self.get_logger().info(f"State -> {state.value} ({label})")
        self.print_status()

    def enter_playback_mode(self) -> None:
        self._clear_transition_plan()
        self._enter_hold_mode(ManagerState.PLAYBACK, "playback; MIT on, holding current")

    def enter_transition_mode(self) -> None:
        self._reload_arm_config()
        if self.arm_config is None:
            raise RuntimeError("--arm-config not set or unreadable; cannot plan anchor transitions")
        if not self.transition_targets:
            raise RuntimeError(
                "arm_config has no anchor targets matching this teach session; "
                "check the active arm namespaces and taught pose dimensions"
            )
        self._enter_hold_mode(ManagerState.TRANSITIONS, "transitions; MIT on, holding current")

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

    # --- resource selection --------------------------------------------------

    def _resource_options(self) -> list[str]:
        """Resources this teach session can save an action as."""
        if not self.is_dual:
            return [self.arms[0].namespace or "nero"]
        return ["both_arms"] + [arm.namespace for arm in self.arms]

    def _prompt_resource(self, key_reader: TerminalKeyReader, purpose: str) -> Optional[str]:
        options = self._resource_options()
        if len(options) == 1:
            return options[0]
        listing = ", ".join(f"{i}={name}" for i, name in enumerate(options))
        raw = self.prompt_line(key_reader, f"Resource for this {purpose} [{listing}] (default 0): ").strip()
        if not raw:
            return options[0]
        try:
            index = int(raw)
            return options[index]
        except (ValueError, IndexError):
            self.get_logger().warn(f"invalid resource selection '{raw}'; aborting")
            return None

    # --- teach actions -------------------------------------------------------

    def _record_all_arms(self) -> dict[_ArmEndpoint, list[RecorderSnapshot]]:
        """Capture every arm on one shared clock until motion stops on all arms.

        Recording starts once any arm moves and stops after ``hold_timeout`` with
        no arm moving, so both arms share identical sample timestamps — the basis
        for a time-synced duo merge.
        """
        period = 1.0 / self.args.sample_rate
        samples: dict[_ArmEndpoint, list[RecorderSnapshot]] = {arm: [] for arm in self.arms}
        recording_start = time.monotonic()
        last_motion_time = recording_start
        motion_started = False
        while rclpy.ok():
            loop_start = time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(period, 0.1))
            frame = {arm: arm.snapshot(loop_start - recording_start) for arm in self.arms}
            if any(snap is None for snap in frame.values()):
                continue
            moved = False
            for arm in self.arms:
                if samples[arm]:
                    deltas = [
                        abs(current - previous)
                        for current, previous in zip(frame[arm].positions, samples[arm][-1].positions)
                    ]
                    if max(deltas, default=0.0) >= self.args.movement_threshold:
                        moved = True
            if moved:
                motion_started = True
                last_motion_time = loop_start
            for arm in self.arms:
                samples[arm].append(frame[arm])
            if motion_started and (loop_start - last_motion_time) >= self.args.hold_timeout:
                break
            sleep_time = max(0.0, period - (time.monotonic() - loop_start))
            if sleep_time > 0.0:
                time.sleep(sleep_time)
        if not motion_started:
            raise RuntimeError("No joint movement detected during recording")
        return samples

    def _build_arm_trajectory(self, name: str, arm: _ArmEndpoint, arm_samples: list[RecorderSnapshot]) -> RecordedTrajectory:
        return build_recorded_trajectory(
            name=name,
            joint_names=list(arm.source_joints),
            sample_rate=self.args.sample_rate,
            hold_timeout=self.args.hold_timeout,
            movement_threshold=self.args.movement_threshold,
            samples=arm_samples,
            raw_sample_count=len(arm_samples),
            urdf_path=self.args.urdf_path or None,
            metadata={"manager": "agx_arm_teach_manager", "namespace": arm.namespace},
        )

    def record_sample(self, key_reader: TerminalKeyReader) -> None:
        if self.state != ManagerState.RECORD:
            raise RuntimeError("recording is only available in record mode")
        if not self.source_joints:
            raise RuntimeError("--source-joints not set; cannot record a joint vector")
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError("no feedback from all arms before recording")
        resource = self._prompt_resource(key_reader, "recording")
        if resource is None:
            return
        default_name = sanitize_trajectory_name(time.strftime("teach_%Y%m%d_%H%M%S"))
        name = sanitize_trajectory_name(
            self.prompt_line(key_reader, f"Trajectory name [{default_name}]: ") or default_name
        )
        self.get_logger().info(f"Recording '{name}' as '{resource}' — move the arm(s); auto-stops after hold timeout")
        samples = self._record_all_arms()

        if resource == "both_arms":
            ordered = sorted(self.arms, key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))
            per_arm = [self._build_arm_trajectory(f"{name}_{arm.namespace}", arm, samples[arm]) for arm in ordered]
            trajectory = build_duo_trajectory(
                per_arm[0],
                per_arm[1],
                name=name,
                left_prefix=f"{ordered[0].namespace}_",
                right_prefix=f"{ordered[1].namespace}_",
                rate_hz=self.args.sample_rate,
            )
        else:
            arm = next((a for a in self.arms if (a.namespace or "nero") == resource), self.arms[0])
            trajectory = self._build_arm_trajectory(name, arm, samples[arm])

        saved = save_recorded_trajectory(trajectory, self.library_dir / f"{name}.json")
        self.refresh_library()
        if saved in self.trajectory_paths:
            self.selected_index = self.trajectory_paths.index(saved)
        self.get_logger().info(f"Saved {saved} ({len(trajectory.joint_names)}-dim, resource={resource})")
        self.print_status()

    def capture_anchor(self, key_reader: TerminalKeyReader) -> None:
        if self.arm_config_path is None:
            raise RuntimeError("--arm-config not set; cannot write anchor poses")
        if not self.source_joints:
            raise RuntimeError("--source-joints not set; cannot order the captured vector")
        resource = self._prompt_resource(key_reader, "anchor")
        if resource is None:
            return
        pose_name = self.prompt_line(key_reader, "Anchor pose name (e.g. Pre_Grip_R): ").strip()
        if not pose_name:
            self.get_logger().warn("no pose name given; capture aborted")
            return

        if resource == "both_arms":
            ordered = sorted(self.arms, key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))
            arms = ordered
        else:
            arms = [next((a for a in self.arms if (a.namespace or "nero") == resource), self.arms[0])]

        vector: list[float] = []
        for arm in arms:
            averaged = average_joint_positions(
                self, lambda a=arm: a.latest, arm.source_joints,
                self.args.settle_sec, self.args.feedback_timeout,
            )
            vector.extend(averaged[name] for name in arm.source_joints)

        try:
            note = update_pose_in_config(self.arm_config_path, pose_name, vector, self.args.precision)
        except OSError as exc:
            # Do not tear down the whole teach session over a bad config path;
            # the captured pose is printed so it can still be pasted manually.
            formatted = "[" + ", ".join(f"{v:.{self.args.precision}f}" for v in vector) + "]"
            self.get_logger().error(f"could not write anchor pose to {self.arm_config_path}: {exc}")
            self.get_logger().error(f"captured vector for '{pose_name}' ({resource}): {formatted}")
            return
        formatted = "[" + ", ".join(f"{v:.{self.args.precision}f}" for v in vector) + "]"
        self.get_logger().info(f"{note}: {pose_name} ({resource}, {len(vector)}-dim) = {formatted}")
        self.get_logger().info("rebuild agx_arm_coordination (or symlink-install) for a launched coordinator")
        self._reload_arm_config()

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

    # --- playback ------------------------------------------------------------

    def _dispatch_to_arm(self, arm: _ArmEndpoint, trajectory_msg: JointTrajectory, columns: list[int]) -> None:
        """Publish a per-arm slice to that arm's controller (joint1..7, unprefixed)."""
        msg = JointTrajectory()
        msg.joint_names = list(arm.source_joints)
        for point in trajectory_msg.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = [float(point.positions[i]) for i in columns]
            ros_point.velocities = [float(point.velocities[i]) for i in columns] if point.velocities else []
            ros_point.time_from_start = point.time_from_start
            msg.points.append(ros_point)
        for _ in range(max(1, self.args.publish_repetitions)):
            arm.trajectory_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(max(0.0, self.args.publish_interval))

    def _arm_columns(self, arm: _ArmEndpoint, joint_names: list[str]) -> Optional[list[int]]:
        """Column indices in ``joint_names`` this arm owns (side-prefixed match)."""
        wanted = [f"{arm.side_prefix}{joint}" for joint in arm.source_joints]
        index = {name: i for i, name in enumerate(joint_names)}
        if all(name in index for name in wanted):
            return [index[name] for name in wanted]
        # Fall back to the bare (unprefixed) names for a single-arm recording.
        if all(joint in index for joint in arm.source_joints):
            return [index[joint] for joint in arm.source_joints]
        return None

    def _current_positions_for_trajectory(
        self,
        trajectory: RecordedTrajectory,
        dispatched: list[tuple[_ArmEndpoint, list[int]]],
    ) -> list[float]:
        current_positions = [float(point) for point in trajectory.points[0].positions]
        for arm, columns in dispatched:
            msg = arm.latest
            if msg is None:
                raise RuntimeError(f"no fresh feedback for {arm.label} before playback")
            position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
            missing = [joint for joint in arm.source_joints if joint not in position_map]
            if missing:
                raise RuntimeError(
                    f"[{arm.label}] feedback/joint_states is missing {missing}; cannot build playback lead-in"
                )
            for column, joint_name in zip(columns, arm.source_joints):
                current_positions[column] = position_map[joint_name]
        return current_positions

    def playback_selected(self) -> None:
        if self.state != ManagerState.PLAYBACK:
            raise RuntimeError("switch to playback mode first")
        path = self.selected_trajectory_path()
        if path is None:
            raise RuntimeError(f"no recordings in {self.library_dir}")
        trajectory = load_recorded_trajectory(path)
        if not trajectory.points:
            raise RuntimeError(f"recording '{trajectory.name}' has no points")

        dispatched = []
        for arm in self.arms:
            columns = self._arm_columns(arm, list(trajectory.joint_names))
            if columns is not None:
                dispatched.append((arm, columns))
        if not dispatched:
            raise RuntimeError(
                f"recording joints {list(trajectory.joint_names)} match none of the arms "
                f"{[a.label for a in self.arms]}; record it with a matching resource"
            )
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError("did not receive fresh feedback from all arms before playback")

        current_positions = None
        if self.args.playback_lead_in_sec > 0.0:
            current_positions = self._current_positions_for_trajectory(trajectory, dispatched)
        # Smooth at playback time (raw recordings stay untouched): teach
        # recordings carry stale-sample staircases whose finite-difference
        # velocities chatter, which the MIT controller reproduces as judder.
        trajectory = smooth_recorded_trajectory(trajectory, self.args.playback_smoothing_window)
        full_msg = recorded_to_joint_trajectory(
            trajectory,
            time_scale=1.0 / self.args.playback_speed_scale,
            current_positions=current_positions,
            lead_in_sec=self.args.playback_lead_in_sec,
        )
        # Publish each arm's slice back-to-back (per-arm controllers run concurrently);
        # for a duo recording this is the direct-controller sync path (bypasses MoveIt).
        for arm, columns in dispatched:
            self._dispatch_to_arm(arm, full_msg, columns)
        self.get_logger().info(
            f"Played {path.name} on {[arm.label for arm, _ in dispatched]} "
            "(needs enable_debug_joint_trajectory_topic:=true on the MIT bring-up)"
        )
        self.print_status()

    def cancel_active(self) -> None:
        for arm in self.arms:
            ok, msg = arm.call_cancel(self.args.service_timeout)
            if not ok:
                self.get_logger().warn(f"[{arm.label}] failed to cancel: {msg}")
        self.get_logger().info("cancelled active MIT trajectory on all arms")

    # --- transitions ---------------------------------------------------------

    def _wait_for_moveit_server(self, client: ActionClient, label: str) -> None:
        if not client.wait_for_server(timeout_sec=self.args.moveit_timeout):
            raise RuntimeError(
                f"{label} action '{client._action_name}' not available; "
                "bring up start_agx_arm_components.launch.py mode:=moveit_mit for the matching execution_profile"
            )

    def _send_goal_and_wait(self, client: ActionClient, goal, label: str):
        self._wait_for_moveit_server(client, label)
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=self.args.moveit_timeout)
        if not goal_future.done() or goal_future.result() is None:
            raise RuntimeError(f"timed out sending {label} goal")
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.moveit_timeout)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError(f"timed out waiting for {label} result")
        return result_future.result()

    def plan_selected_transition(self) -> None:
        if self.state != ManagerState.TRANSITIONS:
            raise RuntimeError("switch to transitions mode first")
        target = self.selected_transition_target()
        if target is None:
            raise RuntimeError("no anchor transition target available")
        constraints = Constraints()
        for joint_name, position in zip(target.joint_names, target.target_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = self.args.transition_joint_tolerance
            jc.tolerance_below = self.args.transition_joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request = MotionPlanRequest()
        request.group_name = target.planning_group
        # Plan from the monitored current state. Without is_diff the default
        # start_state is an empty (non-diff) RobotState and move_group logs
        # 'Found empty JointState message' on every plan before falling back.
        request.start_state.is_diff = True
        request.goal_constraints.append(constraints)
        request.max_velocity_scaling_factor = self.args.transition_velocity_scaling
        request.max_acceleration_scaling_factor = self.args.transition_acceleration_scaling
        request.num_planning_attempts = self.args.transition_num_planning_attempts
        request.allowed_planning_time = self.args.transition_allowed_planning_time
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        wrapper = self._send_goal_and_wait(self._move_group_client, goal, "MoveGroup plan")
        result = wrapper.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._clear_transition_plan()
            raise RuntimeError(
                f"planning transition to {target.label} failed with MoveIt error_code={result.error_code.val}"
            )
        if not result.planned_trajectory.joint_trajectory.points:
            self._clear_transition_plan()
            raise RuntimeError(f"planning transition to {target.label} returned an empty trajectory")
        self.pending_transition_plan = result.planned_trajectory
        self.pending_transition_target_label = target.label
        self.get_logger().info(
            f"Planned transition to {target.label} on '{target.planning_group}' "
            f"({len(result.planned_trajectory.joint_trajectory.points)} point(s), planning_time={result.planning_time:.3f}s). "
            "Press 'f' again to execute."
        )
        self.print_status()

    def execute_planned_transition(self) -> None:
        if self.state != ManagerState.TRANSITIONS:
            raise RuntimeError("switch to transitions mode first")
        target = self.selected_transition_target()
        if target is None:
            raise RuntimeError("no anchor transition target available")
        if self.pending_transition_plan is None or self.pending_transition_target_label != target.label:
            self.plan_selected_transition()
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self.pending_transition_plan
        wrapper = self._send_goal_and_wait(self._execute_trajectory_client, goal, "ExecuteTrajectory")
        result = wrapper.result
        status = getattr(wrapper, "status", GoalStatus.STATUS_UNKNOWN)
        if status != GoalStatus.STATUS_SUCCEEDED or result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"executing transition to {target.label} failed with status={status}, "
                f"MoveIt error_code={result.error_code.val}"
            )
        self.get_logger().info(f"Executed transition to {target.label} via MoveIt")
        self._clear_transition_plan()
        self.print_status()

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
        if key == "t":
            self.enter_transition_mode()
            return
        if key == "[":
            if self.state == ManagerState.TRANSITIONS:
                self.select_next_transition(-1)
            else:
                self.select_next(-1)
            return
        if key == "]":
            if self.state == ManagerState.TRANSITIONS:
                self.select_next_transition(1)
            else:
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
        if self.state == ManagerState.TRANSITIONS and key == "f":
            self.execute_planned_transition()
            return
        if self.state == ManagerState.TRANSITIONS and key == "c":
            self._clear_transition_plan()
            self.get_logger().info("cleared cached transition plan")
            self.print_status()
            return
        self.get_logger().warn(f"unhandled key '{key}' in state={self.state.value}; press 'h' for help")

    def print_help(self) -> None:
        print(
            "\nTeach manager keys:\n"
            "  i -> idle / freedrive (MIT zero-force, gravity-compensated)\n"
            "  r -> record mode      p -> playback mode      t -> transitions mode\n"
            "  a -> capture current pose as a named anchor (-> arm_config.yaml)\n"
            "  w -> convert selected recording -> catalogue waypoints\n"
            "  [ / ] -> select previous / next item (recording or anchor target)\n"
            "  s -> status   h -> help   q -> quit\n"
            "Record mode:   n -> record a new trajectory\n"
            "Playback mode: f -> play selected   c -> cancel active trajectory\n"
            "Transitions:  f -> plan selected target, press f again -> execute cached plan, c -> clear cached plan\n"
            "With two arms, record/anchor ask which resource to save "
            "(both_arms -> merged 14-dim, or one side -> 7-dim).\n"
        )

    def print_status(self) -> None:
        self.refresh_library()
        selected = self.selected_trajectory_path()
        transition = self.selected_transition_target()
        arms = ", ".join(arm.label for arm in self.arms)
        print(
            "Status: "
            f"state={self.state.value}, "
            f"arms=[{arms}], "
            f"recordings={len(self.trajectory_paths)}, "
            f"selected={selected.name if selected else '<none>'}, "
            f"transition={transition.label if transition else '<none>'}, "
            f"transition_plan={'cached' if self.pending_transition_plan is not None else '<none>'}, "
            f"arm_config={'set' if self.arm_config_path else '<none>'}, "
            f"library={self.library_dir}"
        )

    def _rebind_arms(self, namespaces: list[str]) -> None:
        """Rebuild the arm endpoints against discovered namespaces.

        The previous endpoints' subscriptions/clients stay registered on the
        node (their un-namespaced topics simply never fire) — a one-time,
        harmless leftover in exchange for not tearing the node down.
        """
        self.arms = [
            _ArmEndpoint(self, ns, self.source_joints, self.args.source_topic) for ns in namespaces
        ]
        self.arms.sort(key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))
        # Transition targets are namespace-derived — rebuild them for the new arms.
        self._reload_arm_config()

    def wait_for_required_services(self, timeout_s: float) -> None:
        """Block until every arm's MIT services exist, pointing at the bring-up launch.

        Started without --arms against a namespaced bring-up (components
        baseline duo_arm/duo_hand, or a namespaced single side), the manager
        rebinds itself automatically to the MIT stacks it finds in the graph
        instead of waiting forever for the un-namespaced default.
        """
        clients: list[tuple[str, object]] = []
        for arm in self.arms:
            clients.extend(arm.required_clients(self.args.auto_enable_arm))
        deadline = None if timeout_s <= 0.0 else time.monotonic() + timeout_s
        warned = False
        # Only the implicit un-namespaced default may be rebound; an explicit
        # --arms choice is respected verbatim.
        auto_detect = not self.args.arms
        while rclpy.ok():
            missing = [label for label, client in clients if not client.wait_for_service(timeout_sec=0.2)]
            if not missing:
                return

            discovered = _discover_mit_namespaces(self)
            if auto_detect and "" not in discovered:
                namespaced = [ns for ns in discovered if ns]
                if namespaced:
                    self.get_logger().info(
                        "No un-namespaced MIT stack, but found namespaced stack(s) "
                        f"{namespaced} in the graph — rebinding automatically "
                        "(equivalent to --arms " + " ".join(namespaced) + ")"
                    )
                    self._rebind_arms(namespaced)
                    clients = []
                    for arm in self.arms:
                        clients.extend(arm.required_clients(self.args.auto_enable_arm))
                    auto_detect = False
                    continue

            if not warned:
                example = (
                    "  ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py "
                    "can_port:=can_nero_right\n"
                    if not self.is_dual
                    else (
                        "  # one launch per arm, each namespaced:\n"
                        "  ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py "
                        "namespace:=left_arm can_port:=can_nero_left\n"
                        "  ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py "
                        "namespace:=right_arm can_port:=can_nero_right\n"
                    )
                )
                graph_hint = (
                    "MIT stacks currently in the graph: "
                    + (", ".join(ns or "<un-namespaced>" for ns in discovered) if discovered else "none")
                )
                self.get_logger().warn(
                    "Waiting for the arm MIT services: " + ", ".join(missing)
                    + ".\nThe teach manager does not start the arm — bring it up first, e.g.:\n"
                    + example
                    + "(no input_joint_prefix for the teach loop; use --source-joints joint1,...,joint7)\n"
                    + graph_hint
                )
                warned = True
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for arm services: " + ", ".join(missing))
            rclpy.spin_once(self, timeout_sec=0.2)

    def run(self) -> None:
        self.wait_for_required_services(self.args.startup_timeout)
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
    parser.add_argument(
        "--arms",
        nargs="*",
        default=None,
        help="One ROS namespace per arm (e.g. left_arm right_arm). Omit for a single un-namespaced arm.",
    )
    parser.add_argument("--source-joints", default="", help="Comma-separated joint names per arm, in stored order (e.g. joint1,...,joint7)")
    parser.add_argument("--source-topic", default="feedback/joint_states", help="Per-arm JointState topic (namespaced automatically)")
    parser.add_argument("--start-mode", choices=[s.value for s in ManagerState], default=ManagerState.IDLE.value)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--hold-timeout", type=float, default=3.0)
    parser.add_argument("--movement-threshold", type=float, default=0.01)
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--startup-timeout", type=float, default=0.0,
                        help="Seconds to wait for the arm/MIT services at startup; 0 waits forever")
    parser.add_argument("--moveit-timeout", type=float, default=15.0,
                        help="Timeout waiting for MoveIt plan/execute action responses")
    parser.add_argument(
        "--playback-speed-scale",
        type=float,
        default=1.0,
        help="Playback speed scale (0.25 = quarter-speed, 1.0 = recorded speed)",
    )
    parser.add_argument(
        "--playback-lead-in-sec",
        type=float,
        default=0.0,
        help="Blend from the current hold pose to the first recorded waypoint over this many seconds",
    )
    parser.add_argument(
        "--playback-smoothing-window",
        type=int,
        default=9,
        help=(
            "Zero-phase moving-average window (samples) applied to recorded positions at "
            "playback, with velocities recomputed from the smoothed signal (9 at 50 Hz "
            "~ 180 ms). Removes the stale-sample staircase that makes raw playback judder; "
            "<= 1 disables smoothing."
        ),
    )
    parser.add_argument("--transition-velocity-scaling", type=float, default=0.10,
                        help="MoveIt velocity scaling for anchor transitions (0,1]")
    parser.add_argument("--transition-acceleration-scaling", type=float, default=0.10,
                        help="MoveIt acceleration scaling for anchor transitions (0,1]")
    parser.add_argument("--transition-joint-tolerance", type=float, default=0.01,
                        help="Joint target tolerance used when planning anchor transitions")
    parser.add_argument("--transition-num-planning-attempts", type=int, default=10,
                        help="MoveIt planning attempts for anchor transitions")
    parser.add_argument("--transition-allowed-planning-time", type=float, default=5.0,
                        help="MoveIt allowed planning time for anchor transitions")
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
    if args.playback_speed_scale <= 0.0:
        raise ValueError("--playback-speed-scale must be > 0")
    if args.playback_lead_in_sec < 0.0:
        raise ValueError("--playback-lead-in-sec must be >= 0")
    if not 0.0 < args.transition_velocity_scaling <= 1.0:
        raise ValueError("--transition-velocity-scaling must be in (0, 1]")
    if not 0.0 < args.transition_acceleration_scaling <= 1.0:
        raise ValueError("--transition-acceleration-scaling must be in (0, 1]")
    if args.transition_joint_tolerance <= 0.0:
        raise ValueError("--transition-joint-tolerance must be > 0")
    if args.transition_num_planning_attempts <= 0:
        raise ValueError("--transition-num-planning-attempts must be > 0")
    if args.transition_allowed_planning_time <= 0.0:
        raise ValueError("--transition-allowed-planning-time must be > 0")
    if args.moveit_timeout <= 0.0:
        raise ValueError("--moveit-timeout must be > 0")

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
