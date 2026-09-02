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
joint prefix, separates the arms. Recording is driven by each arm's feedback
callbacks, so an arm's own cadence is its recording's cadence; the two arms are
put on one time axis afterwards. At save time you choose the resource the
recording is stored as (``both_arms`` -> merged 14-dim, or one side -> 7-dim).

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
import math
from pathlib import Path
import re
import time
from typing import Optional

import yaml

from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger

from agx_arm_msgs.msg import (
    DeviceCommandStamp,
    GripperStatus,
    HandJointTarget,
    OmniHandStatus,
)
from agx_arm_msgs.srv import ClaimDevice
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_mit_controller.model_metadata import compute_flange_pose_from_mdh
from agx_arm_mit_controller.trajectory_io import (
    RecordedTrajectory,
    load_recorded_trajectory,
    reconstruct_stalled_joints,
    sanitize_trajectory_name,
    save_recorded_trajectory,
)
from agx_arm_coordination.arm_executor import ArmConfig
from agx_arm_coordination.gripper_closure import (
    ClosureError,
    closure_to_finger_positions,
    displayed_closure,
)
from agx_arm_ctrl.motion_registry import assert_matches_topology, handshake_required
from agx_arm_retiming import (
    AS_RECORDED,
    DEFAULT_RESAMPLE_DT,
    DEFAULT_SMOOTHING_WINDOW_SEC,
    MAXIMIZE_SPEED,
    MODES as RETIMING_MODES,
    NERO_MAX_VELOCITY,
    RECONSTRUCTION_WINDOW_SEC,
    SMOOTH,
    SPEED_SCALE,
    TEMPO_SCALE,
    TIMING_PRESERVING,
    RetimingError,
    default_acceleration,
    retime,
)

from .capture_anchor_pose import average_joint_positions, update_pose_in_config
from .leader_trajectory_recorder import RecorderSnapshot, build_recorded_trajectory
from .playback import retimed_to_joint_trajectory
from .recorded_to_catalogue import build_duo_trajectory, format_waypoints_block, recorded_to_waypoints
from .wakeword_motion_manager import TerminalKeyReader


#: One Nero arm. A recording's joint count divided by this is how many arms it
#: covers, which is what maps the per-joint limits onto a duo recording.
ARM_JOINT_COUNT = 7


class ManagerState(str, Enum):
    IDLE = "idle"
    RECORD = "record"
    PLAYBACK = "playback"
    TRANSITIONS = "transitions"
    HAND = "hand"
    GRIPPER = "gripper"


@dataclass
class _PlaybackPlan:
    """How the next replay is time-parameterized. Chosen per replay, not per
    bring-up, so switching between a threading motion and a transit motion does
    not need the stack restarted."""

    mode: str
    speed_scale: float
    smoothing_window_sec: float


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


def _load_hand_gestures(path: Path) -> tuple[list[str], dict[str, list[float]]]:
    """Read (active_joint_order, {gesture_name: vector}) from a gesture YAML.

    Same file the omnihand_skill_controller reads (omnihand_pro_gestures.yaml),
    so a skill captured here is immediately usable as a preset. Returns empty
    structures if the file is missing or malformed rather than raising.
    """
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return [], {}
    order = [str(j) for j in (data.get("omnihand_active_joint_order") or [])]
    gestures = {
        str(name): [float(v) for v in vec]
        for name, vec in (data.get("omnihand_gestures") or {}).items()
        if isinstance(vec, (list, tuple))
    }
    return order, gestures


def _update_gesture_in_config(path: Path, name: str, vector: list[float], precision: int) -> str:
    """Insert or replace one gesture line under ``omnihand_gestures:``.

    Line-based like ``update_pose_in_config`` so comments and the joint-order
    header are preserved; only the single gesture line is touched.
    """
    lines = path.read_text(encoding="utf-8").splitlines()
    formatted = "[" + ", ".join(f"{v:.{precision}f}" for v in vector) + "]"
    entry_re = re.compile(rf"^(\s+){re.escape(name)}:\s*.*$")
    for i, line in enumerate(lines):
        if entry_re.match(line):
            indent = entry_re.match(line).group(1)
            lines[i] = f"{indent}{name}: {formatted}"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return f"updated existing gesture '{name}'"
    block_idx = next(
        (i for i, ln in enumerate(lines) if re.match(r"^\s*omnihand_gestures:\s*$", ln)), None
    )
    if block_idx is None:
        raise RuntimeError(f"no 'omnihand_gestures:' block in {path}")
    child_indent = "  "
    for line in lines[block_idx + 1:]:
        if line.strip() and not line.lstrip().startswith("#"):
            m = re.match(r"^(\s+)\S", line)
            if m:
                child_indent = m.group(1)
            break
    lines.insert(block_idx + 1, f"{child_indent}{name}: {formatted}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return f"inserted new gesture '{name}'"


def _resource_robot_id(resource: str) -> str:
    """robot_id an anchor capture is stored under for the chosen resource.

    The resource is picked in the UI before saving, so the side is known: it is
    stored explicitly rather than encoded in an _L/_R name suffix. An
    un-namespaced single arm resolves to the right side (see
    ``_transition_robot_ids``).
    """
    if resource in ("both_arms", "left_arm", "right_arm"):
        return resource
    return "right_arm"


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
            # Explicit both_arms anchors: one 14-DoF entry (robot_id: both_arms).
            for pose_name, vector in sorted(config.poses.items()):
                if config.pose_robot_id(pose_name) != "both_arms":
                    continue
                if len(vector) != len(group.joint_names):
                    continue
                targets.append(
                    TransitionTarget(
                        label=f"both_arms:{pose_name}",
                        robot_id=robot_id,
                        planning_group=group.planning_group,
                        joint_names=group.joint_names,
                        pose_names=(pose_name,),
                        target_positions=tuple(float(v) for v in vector),
                    )
                )
            # Legacy fallback: pair bare-list _L/_R poses by stem into both_arms.
            left = {
                name[:-2]: name for name in config.poses
                if config.pose_robot_id(name) == "left_arm" and name.endswith("_L")
            }
            right = {
                name[:-2]: name for name in config.poses
                if config.pose_robot_id(name) == "right_arm" and name.endswith("_R")
            }
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

        # Single side: poses whose resolved robot_id is this side.
        for pose_name, vector in sorted(config.poses.items()):
            if config.pose_robot_id(pose_name) != robot_id:
                continue
            if len(vector) != len(group.joint_names):
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


def _resolve_topic_for_namespace(namespace: str, topic: str) -> str:
    """Apply an arm namespace to a relative topic, preserving absolute names."""
    cleaned = topic.strip()
    if not cleaned:
        return cleaned
    if cleaned.startswith("/"):
        return cleaned
    ns = namespace.strip("/")
    return f"/{ns}/{cleaned}" if ns else cleaned


#: Where the hand bridge serves its claim. Deliberately not `claim_device`: the
#: arm driver serves one of those too, and a client would reach whichever it found.
HAND_CLAIM_SERVICE = "control/omnihand/claim_device"
#: A gesture is a static target, so it travels on the reactive surface, and the
#: bridge checks the surface against the primitive the owner_id declares.
HAND_OWNER_PRIMITIVE = "reactive"


@dataclass
class _HandAuthority:
    """The claim one hand's commands are stamped with.

    Both generations come from the claim response, so the first command need not
    wait for the authority topic. The sequence restarts at each new claim.
    """

    device_epoch: int = 0
    unit_safety_epoch: int = 0
    sequence: int = 0
    held: bool = False


def _hand_side_for_arm_name(name: str) -> str:
    """Infer the hand side for an arm namespace/label.

    The teach-loop default is the un-namespaced right arm, so ambiguous names
    fall back to ``right``.
    """
    cleaned = name.strip("/").lower()
    if cleaned.startswith("left"):
        return "left"
    return "right"


def _recording_namespace(metadata: dict[str, object] | None) -> str:
    """Recorded single-arm namespace, or empty when not stored."""
    if not metadata:
        return ""
    return str(metadata.get("namespace", "")).strip()


def _allow_bare_joint_match(
    *,
    recording_namespace: str,
    arm_namespace: str,
    arm_count: int,
) -> bool:
    """Whether an unprefixed recording may bind to this arm.

    In a duo session, a 7-DoF recording with bare ``joint1..7`` names is
    ambiguous unless it carries the recorded arm namespace in metadata. Only
    that stored owner may receive the playback.
    """
    if arm_count <= 1:
        return True
    if not recording_namespace:
        return False
    return arm_namespace == recording_namespace


def _hand_delivery_verdict(
    status,
    *,
    fresh: bool,
    saw_pending: bool,
    elapsed_s: float,
    grace_s: float = 0.3,
) -> str:
    """Decide hand-command delivery from one status sample.

    Returns ``"wait"`` (no decision yet), ``"delivered"`` (bridge confirmed the
    target landed), or ``"failed"`` (bridge gave up). Only fresh, post-publish
    samples decide; a cleared ``command_pending`` counts as delivered only once
    the command was actually seen pending, or after a short grace, so the stale
    pre-command status is never mistaken for instant success.
    """
    if status is None or not fresh:
        return "wait"
    if status.command_pending:
        return "wait"
    if status.command_delivery_failed:
        return "failed"
    if saw_pending or elapsed_s > grace_s:
        return "delivered"
    return "wait"


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
        # Driver-side step-and-settle handshake: quiesce this arm into a verified
        # hold before a hand command, resume it after.
        self.prepare_hand_window_client = node.create_client(
            Trigger, self._name("prepare_hand_window")
        )
        self.resume_arm_control_client = node.create_client(
            Trigger, self._name("resume_arm_control")
        )
        self.trajectory_pub = node.create_publisher(
            JointTrajectory, self._name("mit_controller/joint_trajectory"), 10
        )
        self.feedback_messages = 0
        self.feedback_frames = 0
        self._last_feedback_stamp = None
        # Capture state, owned by the feedback callback (see start_capture).
        self._capturing = False
        self._capture: list[RecorderSnapshot] = []
        self._capture_origin: Optional[tuple[int, int]] = None
        self._capture_wall_origin = 0.0
        self._capture_uses_stamp = True
        self._capture_threshold = 0.0
        self._capture_moved = False
        self._capture_last_motion = 0.0
        self._capture_first_motion = None
        self._capture_stalled = 0
        node.create_subscription(JointState, self._name(source_topic), self._on_feedback, 50)

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
        self.node._callbacks_served += 1
        self.feedback_messages += 1
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        advanced = stamp != self._last_feedback_stamp
        if advanced:
            self._last_feedback_stamp = stamp
            self.feedback_frames += 1
        if self._capturing:
            self._capture_sample(msg, stamp, advanced)

    def reset_feedback_counters(self) -> None:
        self.feedback_messages = 0
        self.feedback_frames = 0
        self._last_feedback_stamp = None

    # --- recording -----------------------------------------------------------

    def start_capture(self, movement_threshold: float) -> None:
        """Arm this arm's feedback callback to store every new update.

        The arm's own cadence sets the sample times, so a stored sample is always
        an update the arm produced and every update it produced is stored.
        """
        self._capture = []
        self._capture_origin = None
        self._capture_wall_origin = 0.0
        self._capture_uses_stamp = True
        self._capture_threshold = float(movement_threshold)
        self._capture_moved = False
        self._capture_last_motion = time.monotonic()
        self._capture_first_motion = None
        self._capture_stalled = 0
        # Pose the threshold is measured from, re-seeded every time motion is
        # registered. See _on_feedback: a per-sample delta would make the
        # threshold a speed, and a different speed on each arm.
        self._capture_reference: Optional[list[float]] = None
        self.reset_feedback_counters()
        self._capturing = True

    def stop_capture(self) -> list[RecorderSnapshot]:
        self._capturing = False
        return self._capture

    @property
    def capture_moved(self) -> bool:
        return self._capture_moved

    @property
    def capture_last_motion(self) -> float:
        return self._capture_last_motion

    @property
    def capture_uses_stamp(self) -> bool:
        return self._capture_uses_stamp

    @property
    def capture_first_motion(self) -> Optional[float]:
        """Capture time of the first sample that crossed the movement threshold."""
        return self._capture_first_motion

    @property
    def capture_stalled(self) -> int:
        """Reads skipped because the driver's cache had not advanced."""
        return self._capture_stalled

    def _capture_sample(self, msg: JointState, stamp, advanced: bool) -> None:
        # The publisher's stamp is the arm's frame timestamp — when the data was
        # produced, not when the executor delivered it. A zero stamp means the
        # publisher does not fill it; arrival time is then the only clock, and
        # every delivered message is a sample.
        #
        # Times are stored absolute and re-based against all arms at once, so a
        # duo capture stays on one time axis even though each arm's first frame
        # lands at a different instant.
        if self._capture_origin is None:
            self._capture_uses_stamp = stamp != (0, 0)
            self._capture_origin = stamp
            self._capture_wall_origin = time.monotonic()
        elif self._capture_uses_stamp and not advanced:
            return

        if self._capture_uses_stamp:
            elapsed = stamp[0] + stamp[1] * 1e-9
        else:
            elapsed = time.monotonic()
        if self._capture and elapsed <= self._capture[-1].time_from_start:
            # A stamp that does not move forward carries no new instant, and the
            # retiming pipeline requires strictly increasing times.
            return

        position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
        if any(joint not in position_map for joint in self.source_joints):
            return
        positions = [position_map[joint] for joint in self.source_joints]
        if self._capture and positions == self._capture[-1].positions:
            # The stamp tracks the last CAN frame that touched the driver's
            # cache, and a complete joint update is four position frames, so an
            # advancing stamp does not mean advancing positions. Storing an
            # unchanged read asserts the arm was here at this instant, which
            # forces the eventual catch-up into one step: six of seven joints
            # once moved 3-7x their typical sample together, at 4.4 rad/s on a
            # 3.93 rad/s joint. Dropping it lets playback interpolate across the
            # stall instead, and a genuinely still arm interpolates flat.
            self._capture_stalled += 1
            return
        # Displacement from a reference pose, not from the previous sample. A
        # per-sample delta divided by nothing is a speed, and the two arms
        # sample at different rates (~100/s right, ~137/s left), so one number
        # meant 1.0 rad/s on one arm and 1.37 rad/s on the other — a hand-guided
        # teach motion cleared neither. Re-seeding the reference at each
        # registration keeps a slow move registering as it accumulates, so the
        # hold timeout still fires only when the arm is actually held still.
        if self._capture_reference is None:
            self._capture_reference = list(positions)
        deltas = [
            abs(current - reference)
            for current, reference in zip(positions, self._capture_reference)
        ]
        if max(deltas, default=0.0) >= self._capture_threshold:
            self._capture_moved = True
            self._capture_last_motion = time.monotonic()
            self._capture_reference = list(positions)
            if self._capture_first_motion is None:
                self._capture_first_motion = elapsed
        self._capture.append(
            RecorderSnapshot(
                time_from_start=elapsed,
                positions=positions,
                efforts=[0.0] * len(self.source_joints),
                flange_pose=compute_flange_pose_from_mdh(positions, robot="nero"),
            )
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

    def call_prepare_hand_window(self, timeout_s: float) -> tuple[bool, str]:
        return self._call(
            self.prepare_hand_window_client, Trigger.Request(), "prepare_hand_window", timeout_s
        )

    def call_resume_arm_control(self, timeout_s: float) -> tuple[bool, str]:
        return self._call(
            self.resume_arm_control_client, Trigger.Request(), "resume_arm_control", timeout_s
        )

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
    #: Callbacks drained per recording cycle after the blocking spin. Bounded so
    #: a publisher faster than the loop cannot stall it; well above the handful
    #: of subscriptions one bring-up has.
    RECORD_DRAIN_LIMIT = 32

    #: Largest gap between the arm and a replay's first waypoint this will
    #: bridge. Beyond it the arm is somewhere the replay was not taught from and
    #: a planned move belongs first.
    PLAYBACK_MAX_START_OFFSET = 0.35
    #: Joint speed used to size that bridge, well under the limit: the lead-in
    #: is a positioning move, not part of the taught motion.
    PLAYBACK_LEAD_IN_SPEED = 0.25
    #: Re-timed replays kept in memory, keyed on recording and settings.
    PLAYBACK_CACHE_ENTRIES = 8

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("agx_arm_teach_manager")
        self.args = args
        # Advanced by every subscription callback, so the recording loop can see
        # whether a spin actually served anything.
        self._callbacks_served = 0
        self._playback_cache: dict[tuple, object] = {}
        self._playback_plan = _PlaybackPlan(
            mode=args.playback_mode,
            speed_scale=args.playback_speed_scale,
            smoothing_window_sec=args.playback_smoothing_sec,
        )
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

        # Hand mode: capture/replay OmniHand skills from the gesture catalogue.
        raw_hand = args.hand_gestures or "src/agx_arm_ctrl/config/omnihand_pro_gestures.yaml"
        self.hand_gesture_path = _resolve_config_path(raw_hand)
        self.hand_joint_order: list[str] = []
        self.hand_gestures: dict[str, list[float]] = {}
        self.hand_names: list[str] = []
        self.hand_selected_index = 0
        self._reload_hand_gestures()
        self.hand_feedback_by_arm: dict[str, Optional[JointState]] = {}
        self.hand_feedback_topics: dict[str, str] = {}
        self.hand_command_topics: dict[str, str] = {}
        self.hand_command_pubs = {}
        # Per-arm OmniHand delivery status, so a hand op can hold its window open
        # until the bridge confirms the command landed instead of guessing with a
        # fixed dwell (which closes the window mid-retry on a busy shared bus).
        # Gravity residual capture ('v' in freedrive): one model per arm, built
        # on first use, and how many rows each arm's CSV has grown by this session.
        self._gravity_models: dict[str, object] = {}
        self._gravity_rows: dict[str, int] = {}
        self.hand_status_by_arm: dict[str, Optional[OmniHandStatus]] = {}
        self.hand_status_monotonic: dict[str, float] = {}
        self.hand_status_topics: dict[str, str] = {}
        self._hand_arm_label = ""
        self._setup_hand_io()
        # Per-arm parallel gripper: status for the closure display, one action
        # client per arm for commanding. Which arms actually carry one is
        # answered by the hardware at mode entry, not by a flag.
        self.gripper_status_by_arm: dict[str, Optional[GripperStatus]] = {}
        self.gripper_status_monotonic: dict[str, float] = {}
        self.gripper_status_topics: dict[str, str] = {}
        self.gripper_clients: dict[str, ActionClient] = {}
        self.gripper_selected_index = 0
        self._setup_gripper_io()

        self.refresh_library()

    # --- gripper mode --------------------------------------------------------

    def _setup_gripper_io(self) -> None:
        """One status subscription and one trajectory client per arm.

        Created for every arm regardless: an arm without a gripper simply never
        publishes status and never answers on the action, which is exactly how
        :meth:`_grippers_present` tells them apart.
        """
        self.gripper_status_by_arm = {}
        self.gripper_status_monotonic = {}
        self.gripper_status_topics = {}
        self.gripper_clients = {}
        for arm in self.arms:
            label = arm.label
            status_topic = _resolve_topic_for_namespace(
                arm.namespace, self.args.gripper_status_topic
            )
            self.gripper_status_by_arm[label] = None
            self.gripper_status_monotonic[label] = 0.0
            self.gripper_status_topics[label] = status_topic
            self.create_subscription(
                GripperStatus, status_topic,
                lambda msg, key=label: self._on_gripper_status(key, msg),
                1,
            )
            self.gripper_clients[label] = ActionClient(
                self,
                FollowJointTrajectory,
                _resolve_topic_for_namespace(
                    arm.namespace, self.args.gripper_action_topic
                ),
            )

    def _on_gripper_status(self, label: str, msg: GripperStatus) -> None:
        self.gripper_status_by_arm[label] = msg
        self.gripper_status_monotonic[label] = time.monotonic()

    def _gripper_closure_for(self, label: str) -> Optional[float]:
        """Displayed closure for one arm's gripper, or None without a readback."""
        status = self.gripper_status_by_arm.get(label)
        if status is None:
            return None
        try:
            return displayed_closure(status.width)
        except ClosureError:
            return None

    def _grippers_present(self) -> list[_ArmEndpoint]:
        """Arms whose gripper is answering — status seen, or server up."""
        present = []
        for arm in self.arms:
            client = self.gripper_clients.get(arm.label)
            has_status = self.gripper_status_by_arm.get(arm.label) is not None
            if has_status or (client is not None and client.server_is_ready()):
                present.append(arm)
        return present

    def selected_gripper_arm(self) -> Optional[_ArmEndpoint]:
        present = self._grippers_present()
        if not present:
            return None
        return present[self.gripper_selected_index % len(present)]

    def select_next_gripper(self, step: int) -> None:
        present = self._grippers_present()
        if len(present) < 2:
            return
        self.gripper_selected_index = (self.gripper_selected_index + step) % len(present)
        arm = self.selected_gripper_arm()
        if arm is not None:
            self.get_logger().info(f"gripper selected: {arm.label}")

    def enter_gripper_mode(self) -> None:
        # A short spin so a gripper that is publishing gets a chance to be seen
        # before the mode reports there is none.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._grippers_present():
                break
        present = self._grippers_present()
        if not present:
            self.get_logger().warn(
                "no parallel gripper found: no status on "
                f"{sorted(set(self.gripper_status_topics.values()))} and no "
                "trajectory server. Bring the stack up with "
                "effector_type:=agx_gripper."
            )
            return
        self.state = ManagerState.GRIPPER
        arm = self.selected_gripper_arm()
        listing = ", ".join(a.label for a in present)
        self.get_logger().info(
            f"GRIPPER mode (selected={arm.label if arm else '<none>'}, "
            f"available=[{listing}]): 'f' set closure, 'o' open, 'c' close, "
            "'[' ']' select"
        )
        self.print_status()

    def command_gripper_closure(self, closure: float) -> None:
        """Send one normalized closure to the selected gripper and report it.

        Through the same FollowJointTrajectory server MoveIt and the coordinator
        use — never a bare command on control/joint_states, which carries no
        owner and no generation and which the driver refuses by default.
        """
        arm = self.selected_gripper_arm()
        if arm is None:
            self.get_logger().warn("no gripper available to command")
            return
        client = self.gripper_clients[arm.label]
        if not client.wait_for_server(timeout_sec=self.args.service_timeout):
            self.get_logger().error(
                f"gripper trajectory server not available on {arm.label} "
                f"({client._action_name})"
            )
            return
        joint_names = [
            f"{arm.side_prefix}gripper_joint1",
            f"{arm.side_prefix}gripper_joint2",
        ]
        try:
            positions = closure_to_finger_positions(joint_names, closure)
        except ClosureError as exc:
            self.get_logger().warn(str(exc))
            return

        point = JointTrajectoryPoint()
        point.positions = positions
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = [point]

        self.get_logger().info(f"{arm.label} -> closure {closure:.2f}")
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, goal_future, timeout_sec=self.args.service_timeout
        )
        goal_handle = goal_future.result() if goal_future.done() else None
        if goal_handle is None:
            self.get_logger().error("gripper goal was not answered")
            return
        if not goal_handle.accepted:
            self.get_logger().error("gripper goal was rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.args.gripper_timeout_sec
        )
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(
                f"no gripper result within {self.args.gripper_timeout_sec:.1f} s"
            )
            return
        wrapper = result_future.result()
        reached = self._gripper_closure_for(arm.label)
        reached_text = "unknown" if reached is None else f"{reached:.2f}"
        if wrapper.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"{arm.label} canceled at closure {reached_text}: "
                f"{wrapper.result.error_string}"
            )
        elif wrapper.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(
                f"{arm.label} settled at closure {reached_text} "
                f"({wrapper.result.error_string})"
            )
        else:
            self.get_logger().error(
                f"{arm.label} failed at closure {reached_text}: "
                f"{wrapper.result.error_string}"
            )
        self.print_status()

    def prompt_gripper_closure(self, key_reader: TerminalKeyReader) -> None:
        arm = self.selected_gripper_arm()
        if arm is None:
            self.get_logger().warn("no gripper available to command")
            return
        current = self._gripper_closure_for(arm.label)
        current_text = "unknown" if current is None else f"{current:.2f}"
        raw = self.prompt_line(
            key_reader,
            f"[{arm.label}] current closure {current_text}. "
            "Target closure [0.0=open, 1.0=closed]: ",
        ).strip()
        if not raw:
            self.get_logger().info("no closure given; nothing commanded")
            return
        try:
            closure = float(raw)
        except ValueError:
            self.get_logger().warn(f"'{raw}' is not a number; nothing commanded")
            return
        self.command_gripper_closure(closure)

    # --- hand mode -----------------------------------------------------------

    def _setup_hand_io(self) -> None:
        preferred = self.args.hand_arm.strip("/")
        self.hand_feedback_by_arm = {}
        self.hand_feedback_topics = {}
        self.hand_command_topics = {}
        self.hand_command_pubs = {}
        self.hand_status_by_arm = {}
        self.hand_status_monotonic = {}
        self.hand_status_topics = {}
        self.hand_claim_clients = {}
        self.hand_authority = {}
        self.hand_owner_id = f"{HAND_OWNER_PRIMITIVE}:{self.get_name()}"
        for arm in self.arms:
            label = arm.label
            feedback_topic = _resolve_topic_for_namespace(
                arm.namespace, self.args.hand_feedback_topic
            )
            command_topic = _resolve_topic_for_namespace(
                arm.namespace, self.args.hand_command_topic
            )
            status_topic = _resolve_topic_for_namespace(
                arm.namespace, self.args.hand_status_topic
            )
            self.hand_feedback_by_arm[label] = None
            self.hand_feedback_topics[label] = feedback_topic
            self.hand_command_topics[label] = command_topic
            self.hand_status_topics[label] = status_topic
            self.hand_status_by_arm[label] = None
            self.hand_status_monotonic[label] = 0.0
            # A gesture is one static target, so it goes out as HandJointTarget
            # on the reactive surface, stamped with the claim it was issued
            # under. The bare-JointState surface this used to publish on is not
            # subscribed unless the bridge was started with
            # allow_legacy_hand_command_ingress, so those commands reached nobody.
            self.hand_command_pubs[label] = self.create_publisher(
                HandJointTarget, command_topic, 10
            )
            self.hand_claim_clients[label] = self.create_client(
                ClaimDevice,
                _resolve_topic_for_namespace(arm.namespace, HAND_CLAIM_SERVICE),
            )
            self.hand_authority[label] = _HandAuthority()
            self.create_subscription(
                JointState,
                feedback_topic,
                lambda msg, arm_label=label: self._on_hand_feedback(arm_label, msg),
                20,
            )
            self.create_subscription(
                OmniHandStatus,
                status_topic,
                lambda msg, arm_label=label: self._on_hand_status(arm_label, msg),
                10,
            )

        selected = next((arm.label for arm in self.arms if arm.namespace == preferred), "")
        if not selected and self.arms:
            selected = self.arms[0].label
        self._hand_arm_label = selected

    def _on_hand_feedback(self, arm_label: str, msg: JointState) -> None:
        self.hand_feedback_by_arm[arm_label] = msg
        self._callbacks_served += 1

    def _on_hand_status(self, arm_label: str, msg: OmniHandStatus) -> None:
        self.hand_status_by_arm[arm_label] = msg
        self.hand_status_monotonic[arm_label] = time.monotonic()
        self._callbacks_served += 1

    def _await_hand_delivery(self, arm_label: str, published_at: float) -> None:
        """Hold the caller (and thus the open window) until the OmniHand bridge
        confirms the command landed, or a timeout.

        Only status samples received AFTER the command was published are trusted;
        an older one still describes the previous command. Falls back to a fixed
        dwell when no bridge status is present (older bridge / mock), which keeps
        the previous behaviour. Returning promptly on delivery is the point: the
        window closes right after the hand has the target instead of after a
        blind fixed settle that can close mid-retry on a busy shared bus.
        """
        status_topic = self.hand_status_topics.get(arm_label, "")
        if not status_topic or self.count_publishers(status_topic) == 0:
            # No delivery surface (mock / older bridge): keep the previous
            # fixed-dwell behaviour so the hand still gets time to act.
            self._spin_for(self.args.hand_settle_sec)
            return

        timeout_s = max(0.0, float(self.args.hand_delivery_timeout_sec))
        deadline = time.monotonic() + timeout_s
        saw_pending = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status = self.hand_status_by_arm.get(arm_label)
            fresh = self.hand_status_monotonic.get(arm_label, 0.0) > published_at
            if status is not None and fresh and status.command_pending:
                saw_pending = True
            verdict = _hand_delivery_verdict(
                status,
                fresh=fresh,
                saw_pending=saw_pending,
                elapsed_s=time.monotonic() - published_at,
            )
            if verdict == "failed":
                self.get_logger().error(
                    f"hand command on {arm_label} not delivered "
                    f"(bridge gave up after {status.command_attempts} attempts)"
                )
                return
            if verdict == "delivered":
                self.get_logger().info(
                    f"hand command on {arm_label} delivered "
                    f"({status.command_attempts} attempts)"
                )
                return
        self.get_logger().warn(
            f"hand command on {arm_label} not confirmed within "
            f"{timeout_s:.1f} s; closing the window anyway"
        )

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _reload_hand_gestures(self) -> None:
        order, gestures = _load_hand_gestures(self.hand_gesture_path)
        self.hand_joint_order = order
        self.hand_gestures = gestures
        self.hand_names = sorted(gestures)
        self.hand_selected_index = min(self.hand_selected_index, max(0, len(self.hand_names) - 1))

    @property
    def hand_enabled(self) -> bool:
        return bool(self.hand_joint_order) and self.hand_gesture_path.exists()

    def _current_hand_arm(self) -> _ArmEndpoint:
        return next((arm for arm in self.arms if arm.label == self._hand_arm_label), self.arms[0])

    def _hand_joint_names_for_arm(self, arm: _ArmEndpoint) -> list[str]:
        side = _hand_side_for_arm_name(arm.namespace or arm.label)
        return [f"{side}_{joint_name}" for joint_name in self.hand_joint_order]

    def _prompt_hand_arm(self, key_reader: TerminalKeyReader, purpose: str) -> _ArmEndpoint:
        if len(self.arms) == 1:
            self._hand_arm_label = self.arms[0].label
            return self.arms[0]

        current = self._current_hand_arm().label
        listing = ", ".join(f"{i}={arm.label}" for i, arm in enumerate(self.arms))
        default_idx = next(
            (i for i, arm in enumerate(self.arms) if arm.label == current),
            0,
        )
        raw = self.prompt_line(
            key_reader,
            f"Hand resource for {purpose} [{listing}] (default {default_idx}): ",
        ).strip()
        if not raw:
            return self.arms[default_idx]
        try:
            selected = self.arms[int(raw)]
        except (ValueError, IndexError):
            self.get_logger().warn(f"invalid hand resource selection '{raw}'; using {current}")
            return self.arms[default_idx]
        self._hand_arm_label = selected.label
        return selected

    def selected_hand_skill(self) -> Optional[str]:
        if not self.hand_names:
            return None
        return self.hand_names[self.hand_selected_index % len(self.hand_names)]

    def select_next_hand_skill(self, step: int) -> None:
        if not self.hand_names:
            return
        self.hand_selected_index = (self.hand_selected_index + step) % len(self.hand_names)
        self.get_logger().info(f"hand skill selected: {self.selected_hand_skill()}")

    def enter_hand_mode(self) -> None:
        if not self.hand_enabled:
            self.get_logger().warn(
                f"hand mode unavailable: no gestures loaded from {self.hand_gesture_path} "
                "(set --hand-gestures)"
            )
            return
        self.state = ManagerState.HAND
        current = self._current_hand_arm()
        self.get_logger().info(
            f"HAND mode (default source/gate={current.label}, "
            f"feedback={self.hand_feedback_topics.get(current.label, self.args.hand_feedback_topic)}): "
            f"'c' capture, 'f' replay, '[' ']' select. skills: {self.hand_names}"
        )
        self.print_status()

    def _with_hand_window(self, arm: _ArmEndpoint, label: str, fn) -> None:
        """Run a hand op inside a prepare/resume handshake on the hand's arm.

        With ``--no-hand-window`` (dedicated hand bus / parallel operation) the
        handshake is skipped entirely and the hand op runs directly, with the
        arm still under its normal MIT control.
        """
        if not self.args.hand_window:
            fn()
            return
        ok, msg = arm.call_prepare_hand_window(self.args.service_timeout)
        if not ok:
            self.get_logger().error(f"{label} aborted: prepare_hand_window failed ({msg})")
            return
        try:
            fn()
        finally:
            rok, rmsg = arm.call_resume_arm_control(self.args.service_timeout)
            if not rok:
                self.get_logger().error(f"resume_arm_control failed after {label}: {rmsg}")

    def capture_hand_skill(self, key_reader: TerminalKeyReader) -> None:
        if not self.hand_enabled:
            self.get_logger().warn("hand mode not available; cannot capture")
            return
        hand_arm = self._prompt_hand_arm(key_reader, "this hand pose")
        name = self.prompt_line(key_reader, "Hand skill name (e.g. open_flat): ").strip()
        if not name:
            self.get_logger().warn("no skill name given; capture aborted")
            return

        def _do() -> None:
            topic_joint_names = self._hand_joint_names_for_arm(hand_arm)
            try:
                averaged = average_joint_positions(
                    self,
                    lambda: self.hand_feedback_by_arm.get(hand_arm.label),
                    topic_joint_names,
                    self.args.settle_sec, self.args.feedback_timeout,
                )
            except RuntimeError as exc:
                topic = self.hand_feedback_topics.get(hand_arm.label, self.args.hand_feedback_topic)
                self.get_logger().error(
                    f"hand capture failed on {hand_arm.label} ({topic}): {exc}"
                )
                return
            vector = [averaged[joint_name] for joint_name in topic_joint_names]
            try:
                note = _update_gesture_in_config(
                    self.hand_gesture_path, name, vector, self.args.precision
                )
            except (OSError, RuntimeError) as exc:
                formatted = "[" + ", ".join(f"{v:.{self.args.precision}f}" for v in vector) + "]"
                self.get_logger().error(
                    f"could not write gesture to {self.hand_gesture_path}: {exc}"
                )
                self.get_logger().error(f"captured '{name}': {formatted}")
                return
            self.get_logger().info(f"{note} ({len(vector)}-dim); rebuild agx_arm_ctrl to install")
            self._reload_hand_gestures()

        self._with_hand_window(hand_arm, f"capture '{name}' on {hand_arm.label}", _do)

    def _call_hand_claim(self, arm: _ArmEndpoint, *, claim: bool) -> tuple[bool, str]:
        """Take or give up this hand's device authority.

        The claim response carries both generations, which is what the next
        command is stamped with; the bridge is fail-closed, so an unclaimed hand
        executes nothing.
        """
        client = self.hand_claim_clients.get(arm.label)
        if client is None:
            return False, f"no claim client for {arm.label}"
        service = HAND_CLAIM_SERVICE
        if not client.wait_for_service(timeout_sec=self.args.service_timeout):
            return False, f"{service} is not available on {arm.label}"
        request = ClaimDevice.Request()
        request.owner_id = self.hand_owner_id
        request.claim = claim
        future = client.call_async(request)
        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{service} did not answer"
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result()
        if response is None:
            return False, f"{service} returned nothing"
        authority = self.hand_authority[arm.label]
        if response.accepted:
            if claim:
                authority.device_epoch = int(response.device_epoch)
                authority.unit_safety_epoch = int(response.unit_safety_epoch)
                authority.sequence = 0
            authority.held = bool(claim)
        return bool(response.accepted), response.message or response.reason

    def play_hand_skill(self, key_reader: TerminalKeyReader) -> None:
        skill = self.selected_hand_skill()
        if skill is None:
            self.get_logger().warn("no hand skill selected")
            return
        vector = self.hand_gestures.get(skill)
        if not vector or len(vector) != len(self.hand_joint_order):
            self.get_logger().error(f"skill '{skill}' has no vector matching the joint order")
            return
        hand_arm = self._prompt_hand_arm(key_reader, f"replay '{skill}'")

        def _do() -> None:
            accepted, detail = self._call_hand_claim(hand_arm, claim=True)
            if not accepted:
                self.get_logger().error(
                    f"cannot replay '{skill}' on {hand_arm.label}: the hand refused "
                    f"the claim ({detail}). Another commander holds it, or the "
                    "bridge is not up."
                )
                return
            try:
                authority = self.hand_authority[hand_arm.label]
                authority.sequence += 1
                stamp = DeviceCommandStamp()
                stamp.owner_id = self.hand_owner_id
                stamp.device_epoch = authority.device_epoch
                stamp.unit_safety_epoch = authority.unit_safety_epoch
                stamp.sequence = authority.sequence
                msg = HandJointTarget()
                msg.authority = stamp
                msg.joint_names = self._hand_joint_names_for_arm(hand_arm)
                msg.positions = [float(v) for v in vector]
                published_at = time.monotonic()
                self.hand_command_pubs[hand_arm.label].publish(msg)
                topic = self.hand_command_topics.get(
                    hand_arm.label, self.args.hand_command_topic
                )
                self.get_logger().info(
                    f"published hand skill '{skill}' on {hand_arm.label} ({topic}, "
                    f"epoch {stamp.device_epoch} seq {stamp.sequence}); "
                    "holding the window until the bridge confirms delivery"
                )
                # Keep the window open until the hand actually has the target, not
                # for a blind fixed dwell — on a shared bus a fixed dwell closes
                # the window mid-retry and the remaining attempts hit the arm flood.
                self._await_hand_delivery(hand_arm.label, published_at)
            finally:
                # Released even when the publish or the wait failed: a claim left
                # behind blocks every later replay, and releasing advances the
                # epoch so anything still in flight is stale.
                released, release_detail = self._call_hand_claim(hand_arm, claim=False)
                if not released:
                    self.get_logger().warn(
                        f"could not release {hand_arm.label}'s hand: {release_detail}"
                    )

        self._with_hand_window(hand_arm, f"replay '{skill}' on {hand_arm.label}", _do)

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

    def _record_all_arms(
        self, recorded: "list[_ArmEndpoint] | None" = None
    ) -> dict[_ArmEndpoint, list[RecorderSnapshot]]:
        """Capture every arm from its own feedback callbacks until motion stops.

        Each arm stores one sample per feedback update it receives, so the arm's
        cadence is the recording's cadence: nothing is stored twice, and the
        stored rate is the rate the arm delivered. Recording stops after
        ``hold_timeout`` with no arm moving.

        ``recorded`` names the arms the take is actually for. Every arm is still
        captured — a duo merge needs both — but only these decide when the take
        starts, when it ends, and whether it is usable. A single-side take beside
        a deliberately still arm was otherwise refused for the still arm having
        produced nothing to store.
        """
        watched = list(recorded) if recorded else list(self.arms)
        for arm in self.arms:
            arm.start_capture(self.args.movement_threshold)
        recording_start = time.monotonic()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                # Serve everything ready before looping: the subscription queue
                # is finite, and a message dropped there is a sample the arm
                # produced and the recording will not contain. Stop as soon as a
                # spin serves nothing — every spin checks the node's whole wait
                # set, so a fixed drain count is paid even on empty queues.
                for _ in range(self.RECORD_DRAIN_LIMIT):
                    served = self._callbacks_served
                    rclpy.spin_once(self, timeout_sec=0.0)
                    if self._callbacks_served == served:
                        break
                if not any(arm.capture_moved for arm in watched):
                    continue
                idle = time.monotonic() - max(arm.capture_last_motion for arm in watched)
                if idle >= self.args.hold_timeout:
                    break
        finally:
            samples = {arm: arm.stop_capture() for arm in self.arms}

        if not any(arm.capture_moved for arm in watched):
            raise RuntimeError(
                "No joint movement detected on "
                f"{[arm.label for arm in watched]} during recording"
            )
        thin = [arm.label for arm in watched if len(samples[arm]) < 4]
        if thin:
            raise RuntimeError(
                f"too few feedback updates captured on {thin}; the arm is not "
                "publishing feedback/joint_states with an advancing header stamp"
            )
        samples = self._trim_pre_motion(samples)
        samples = self._rebase_capture_times(samples)
        self._report_capture(samples, time.monotonic() - recording_start)
        return samples

    def _trim_pre_motion(
        self, samples: dict[_ArmEndpoint, list[RecorderSnapshot]]
    ) -> dict[_ArmEndpoint, list[RecorderSnapshot]]:
        """Drop the still interval between arming the recorder and the first move.

        One cut instant for every arm — the earliest onset across them, less the
        pre-roll — so a duo recording keeps the relative phase between its arms.
        The pre-roll keeps the physical start of the motion, which a cut at the
        threshold crossing itself would clip.
        """
        onsets = [
            arm.capture_first_motion
            for arm in self.arms
            if arm.capture_first_motion is not None
        ]
        if not onsets:
            return samples
        cut = min(onsets) - max(0.0, float(self.args.pre_roll_sec))
        trimmed: dict[_ArmEndpoint, list[RecorderSnapshot]] = {}
        for arm, arm_samples in samples.items():
            kept = [s for s in arm_samples if s.time_from_start >= cut]
            # Never trim an arm down to a stub: the retiming pipeline needs four
            # samples, and an arm that only moved late still has to start where
            # it was standing.
            trimmed[arm] = kept if len(kept) >= 4 else arm_samples[-4:]
        dropped = sum(len(samples[arm]) - len(trimmed[arm]) for arm in samples)
        if dropped:
            self.get_logger().info(
                f"trimmed {dropped} pre-motion sample(s) across {len(samples)} arm(s); "
                f"recording starts {self.args.pre_roll_sec:.2f}s before the first move"
            )
        return trimmed

    def _rebase_capture_times(
        self, samples: dict[_ArmEndpoint, list[RecorderSnapshot]]
    ) -> dict[_ArmEndpoint, list[RecorderSnapshot]]:
        """Put every arm's absolute capture times on one axis starting at zero.

        Both clocks a capture can use are process-global, so arms are comparable
        as long as they chose the same one. Mixing them would silently skew a duo
        recording against itself.
        """
        # Only arms that captured something have a clock: the flag is decided on
        # the first message received, so an arm held still still carries the
        # start_capture default and comparing it would be a false mismatch.
        clocks = {arm.capture_uses_stamp for arm in self.arms if samples[arm]}
        if len(clocks) > 1:
            raise RuntimeError(
                "arms captured on different clocks (one publishes a header stamp, "
                "one does not); a duo recording cannot be put on one time axis"
            )
        # An arm held still stores nothing, so it contributes no origin. Only the
        # arms that captured something define the shared axis.
        starts = [samples[arm][0].time_from_start for arm in self.arms if samples[arm]]
        if not starts:
            return samples
        origin = min(starts)
        return {
            arm: [
                RecorderSnapshot(
                    time_from_start=sample.time_from_start - origin,
                    positions=sample.positions,
                    efforts=sample.efforts,
                    flange_pose=sample.flange_pose,
                )
                for sample in arm_samples
            ]
            for arm, arm_samples in samples.items()
        }

    @staticmethod
    def _achieved_rate(arm_samples: list[RecorderSnapshot]) -> float:
        if len(arm_samples) < 2:
            return 0.0
        span = arm_samples[-1].time_from_start - arm_samples[0].time_from_start
        return (len(arm_samples) - 1) / span if span > 0.0 else 0.0

    @staticmethod
    def _worst_implied_velocity(
        arm_samples: list[RecorderSnapshot],
    ) -> tuple[float, int, float]:
        """Fastest joint speed the stored samples imply, and where."""
        worst, joint, when = 0.0, 0, 0.0
        for previous, current in zip(arm_samples, arm_samples[1:]):
            span = current.time_from_start - previous.time_from_start
            if span <= 0.0:
                continue
            for index, (a, b) in enumerate(zip(previous.positions, current.positions)):
                speed = abs(b - a) / span
                if speed > worst:
                    worst, joint, when = speed, index, current.time_from_start
        return worst, joint, when

    def _report_capture(
        self, samples: dict[_ArmEndpoint, list[RecorderSnapshot]], elapsed: float
    ) -> None:
        """State the cadence each arm delivered, how even it was, and how clean.

        A recording cannot be read back for this: a stall and a still arm look
        the same in the file, and the implied velocity is the only thing that
        separates a taught motion from a cache catching up.
        """
        if elapsed <= 0.0:
            return
        for arm in self.arms:
            arm_samples = samples[arm]
            gaps = sorted(
                b.time_from_start - a.time_from_start
                for a, b in zip(arm_samples, arm_samples[1:])
            )
            clock = "frame stamp" if arm.capture_uses_stamp else "arrival time"
            if not gaps:
                # An arm held still stores nothing: capture refuses a read whose
                # positions have not changed. It is not part of the take, so this
                # is a statement about it, not a fault.
                self.get_logger().info(
                    f"[{arm.label}] captured {len(arm_samples)} update(s) over "
                    f"{elapsed:.1f}s ({arm.capture_stalled} stalled read(s)); "
                    "no interval to report"
                )
                continue
            median = gaps[len(gaps) // 2]
            self.get_logger().info(
                f"[{arm.label}] captured {len(arm_samples)} updates at "
                f"{self._achieved_rate(arm_samples):.1f} Hz over {elapsed:.1f}s "
                f"({clock}); interval median {median * 1000:.1f} ms, "
                f"max {gaps[-1] * 1000:.1f} ms, {arm.capture_stalled} stalled read(s)"
            )
            worst, joint, when = self._worst_implied_velocity(arm_samples)
            limit = NERO_MAX_VELOCITY[joint] if joint < len(NERO_MAX_VELOCITY) else 0.0
            if limit and worst > limit:
                # Freedrive back-drives the arm by hand, and a hand can move a
                # joint faster than the joint can be commanded, so speed alone
                # does not say which of the two this was. Name both and let the
                # replay's velocity utilisation settle it.
                self.get_logger().warn(
                    f"[{arm.label}] joint{joint + 1} reaches {worst:.2f} rad/s at "
                    f"t={when:.2f}s, past the {limit:.2f} rad/s it can be commanded "
                    "at; the replay will have to slow down or smooth it away"
                )

    def _build_arm_trajectory(self, name: str, arm: _ArmEndpoint, arm_samples: list[RecorderSnapshot]) -> RecordedTrajectory:
        # A whole-vector stall is already refused at capture, but the driver
        # assembles a joint vector from several CAN frames, so one joint can
        # stall while its neighbours update — a read that is genuinely new for
        # the rest of the arm and cannot be dropped.
        times = [sample.time_from_start for sample in arm_samples]
        corrected, spread = reconstruct_stalled_joints(
            times, [list(sample.positions) for sample in arm_samples]
        )
        if any(spread):
            self.get_logger().info(
                f"[{arm.label}] spread {sum(spread)} single-joint stall(s) back over "
                f"the hold that produced them: per joint {spread}"
            )
            arm_samples = [
                RecorderSnapshot(
                    time_from_start=sample.time_from_start,
                    positions=positions,
                    efforts=sample.efforts,
                    flange_pose=sample.flange_pose,
                )
                for sample, positions in zip(arm_samples, corrected)
            ]
        return build_recorded_trajectory(
            name=name,
            # The rate the arm delivered, not one that was configured: with
            # callback-driven capture there is no configured rate to store.
            sample_rate=round(self._achieved_rate(arm_samples), 3),
            joint_names=list(arm.source_joints),
            hold_timeout=self.args.hold_timeout,
            movement_threshold=self.args.movement_threshold,
            samples=arm_samples,
            raw_sample_count=len(arm_samples),
            urdf_path=self.args.urdf_path or None,
            metadata={
                "manager": "agx_arm_teach_manager",
                "namespace": arm.namespace,
                "capture": "event_driven",
                "capture_clock": "frame_stamp" if arm.capture_uses_stamp else "arrival_time",
            },
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
        if resource == "both_arms":
            recorded = sorted(self.arms, key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))
        else:
            recorded = [
                next((a for a in self.arms if (a.namespace or "nero") == resource), self.arms[0])
            ]
        self.get_logger().info(
            f"Recording '{name}' as '{resource}' — move "
            f"{', '.join(arm.label for arm in recorded)}; auto-stops after hold timeout"
        )
        samples = self._record_all_arms(recorded)

        if resource == "both_arms":
            ordered = recorded
            per_arm = [self._build_arm_trajectory(f"{name}_{arm.namespace}", arm, samples[arm]) for arm in ordered]
            # The arms deliver at different rates, so the merge grid takes the
            # faster one: resampling the slower arm up interpolates, resampling
            # the faster one down discards updates it did deliver.
            trajectory = build_duo_trajectory(
                per_arm[0],
                per_arm[1],
                name=name,
                left_prefix=f"{ordered[0].namespace}_",
                right_prefix=f"{ordered[1].namespace}_",
                rate_hz=max(recording.sample_rate_hz for recording in per_arm),
            )
        else:
            arm = recorded[0]
            trajectory = self._build_arm_trajectory(name, arm, samples[arm])

        # No de-duplication pass here: capture already refuses a read whose
        # positions have not changed, and a duo merge output is a uniform grid
        # that removing rows could only make uneven.
        saved = save_recorded_trajectory(trajectory, self.library_dir / f"{name}.json")
        self.refresh_library()
        if saved in self.trajectory_paths:
            self.selected_index = self.trajectory_paths.index(saved)
        self.get_logger().info(f"Saved {saved} ({len(trajectory.joint_names)}-dim, resource={resource})")
        self.print_status()

    # --- gravity residuals ---------------------------------------------------

    def _gravity_model_for(self, arm: _ArmEndpoint):
        """The gravity model for one arm, built once and cached.

        Derived through the same `resolve_gravity_urdf_path` the MIT controller
        uses at bring-up, so the residual is measured against the model that is
        actually running — body mount baked in, hand subtree frozen.
        """
        cached = self._gravity_models.get(arm.label)
        if cached is not None:
            return cached
        from agx_arm_mit_controller.gravity_launch_utils import resolve_gravity_urdf_path
        from agx_arm_mit_controller.gravity_model import create_gravity_model

        urdf_path = resolve_gravity_urdf_path(
            custom_model=self.args.gravity_custom_model,
            explicit_gravity_urdf_path=self.args.gravity_urdf,
            duo_side=_hand_side_for_arm_name(arm.namespace or arm.label),
            effector_type=self.args.gravity_effector_type,
        )
        if not urdf_path:
            raise RuntimeError(
                "no gravity URDF: pass --gravity-urdf, or --gravity-custom-model "
                "to derive one per side. Without it the comparison would run "
                "against the upright hand-less Nero."
            )
        model = create_gravity_model("pinocchio", urdf_path)
        self._gravity_models[arm.label] = model
        self.get_logger().info(f"[{arm.label}] gravity model from {urdf_path}")
        return model

    def _collect_feedback_samples(self, arm: _ArmEndpoint, dwell_sec: float) -> list[JointState]:
        """Every distinct feedback message this arm delivers over ``dwell_sec``.

        Keyed on message identity, not on the header stamp: the stamp carries the
        receive time of the last CAN frame to touch the driver cache and advances
        while the positions need not, so it cannot tell a new reading from a
        stalled one. A message the callback did not replace is not a new sample.
        """
        samples: list[JointState] = []
        last = arm.latest
        deadline = time.monotonic() + dwell_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            # Drain what else is ready, and stop as soon as a spin serves nothing:
            # one spin_once delivers one message from one subscription.
            served = self._callbacks_served
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                if self._callbacks_served == served:
                    break
                served = self._callbacks_served
            current = arm.latest
            if current is not None and current is not last:
                samples.append(current)
                last = current
        return samples

    def capture_gravity_sample(self, key_reader: TerminalKeyReader) -> None:
        """Log the gravity residual at wherever the arm has been guided to.

        Freedrive is MIT zero-force plus the gravity feedforward, so the measured
        joint torque is the compensation the model asked for. Its difference from
        the model is what `fit_gravity_calibration` fits a per-joint scale and
        bias to.
        """
        if self.state != ManagerState.IDLE:
            self.get_logger().warn(
                "gravity capture needs freedrive; press 'i' first so the arm is "
                "hand-guidable and the measured torque is the compensation torque"
            )
            return
        label = self.prompt_line(
            key_reader, "Pose label for this sample (blank = auto): "
        ).strip()

        for arm in self.arms:
            try:
                model = self._gravity_model_for(arm)
            except (RuntimeError, ImportError) as exc:
                self.get_logger().error(f"[{arm.label}] {exc}")
                return
            samples = self._collect_feedback_samples(arm, self.args.gravity_dwell)
            if not samples:
                self.get_logger().error(
                    f"[{arm.label}] no feedback over {self.args.gravity_dwell:.1f}s; "
                    "nothing captured"
                )
                continue
            written = self._write_gravity_samples(arm, model, samples, label)
            self.get_logger().info(
                f"[{arm.label}] {written} sample(s) at '{label or 'auto'}' -> "
                f"{self._gravity_csv_path(arm)} ({self._gravity_rows.get(arm.label, 0)} total)"
            )
        self.print_status()

    def _gravity_csv_path(self, arm: _ArmEndpoint) -> Path:
        directory = Path(self.args.gravity_csv_dir).expanduser()
        return directory / f"{sanitize_trajectory_name(arm.label)}_gravity_freedrive.csv"

    def _write_gravity_samples(self, arm, model, samples: list[JointState], label: str) -> int:
        """Append samples in the schema `fit_gravity_calibration` reads.

        The measured torque is converted into the gravity model's sign
        convention before it is stored, by dividing out the controller's
        `gravity_feedforward_sign`. The controller commands
        `sign * scale * compute_gravity(q)`, so the motor reports the negative
        of the model at the default sign of -1; comparing the raw reading
        against the model made the residual twice the gravity torque and the
        fitted scale -1. `tau_raw_*` keeps the untouched reading.
        """
        import csv

        sign = float(self.args.gravity_feedforward_sign)
        if sign == 0.0:
            raise RuntimeError("--gravity-feedforward-sign must not be zero")

        path = self._gravity_csv_path(arm)
        path.parent.mkdir(parents=True, exist_ok=True)
        count = ARM_JOINT_COUNT
        field_names = [
            "time", "pose",
            *[f"q{i}" for i in range(1, count + 1)],
            *[f"tau_measured_{i}" for i in range(1, count + 1)],
            *[f"tau_g_urdf_{i}" for i in range(1, count + 1)],
            *[f"tau_error_{i}" for i in range(1, count + 1)],
            *[f"tau_raw_{i}" for i in range(1, count + 1)],
        ]
        write_header = not path.exists() or path.stat().st_size == 0
        written = 0
        with path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=field_names)
            if write_header:
                writer.writeheader()
            for sample in samples:
                by_name = dict(zip(sample.name, sample.position))
                effort_by_name = dict(zip(sample.name, sample.effort))
                if not all(joint in by_name for joint in arm.source_joints):
                    continue
                if not all(joint in effort_by_name for joint in arm.source_joints):
                    # The driver publishes motor torque as JointState.effort; a
                    # feedback surface without it cannot answer this question.
                    self.get_logger().error(
                        f"[{arm.label}] feedback carries no effort for every joint; "
                        "the measured torque is what this capture exists to read"
                    )
                    return written
                q = [float(by_name[joint]) for joint in arm.source_joints]
                tau_raw = [float(effort_by_name[joint]) for joint in arm.source_joints]
                tau_measured = [value / sign for value in tau_raw]
                tau_model = model.compute_gravity(q)
                row = {"time": time.monotonic(), "pose": label or "auto"}
                for index in range(count):
                    row[f"q{index + 1}"] = q[index]
                    row[f"tau_measured_{index + 1}"] = tau_measured[index]
                    row[f"tau_g_urdf_{index + 1}"] = tau_model[index]
                    row[f"tau_error_{index + 1}"] = tau_measured[index] - tau_model[index]
                    row[f"tau_raw_{index + 1}"] = tau_raw[index]
                writer.writerow(row)
                written += 1
        self._gravity_rows[arm.label] = self._gravity_rows.get(arm.label, 0) + written
        return written

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

        # Store the resource EXPLICITLY (robot_id: {both_arms|left_arm|right_arm})
        # instead of encoding the side in an _L/_R name suffix. A both_arms
        # capture is one 14-DoF entry (left then right), a single side is one
        # 7-DoF entry — the resource is known from the UI selection, so both the
        # transition builder here and the coordinator's arm_executor read it back
        # by robot_id, and renaming the pose no longer breaks side detection.
        if resource == "both_arms":
            arms = sorted(self.arms, key=lambda arm: _SIDE_ORDER.get(arm.namespace, 99))
        else:
            arms = [next((a for a in self.arms if (a.namespace or "nero") == resource), self.arms[0])]
        robot_id = _resource_robot_id(resource)

        vector: list[float] = []
        for arm in arms:
            averaged = average_joint_positions(
                self, lambda a=arm: a.latest, arm.source_joints,
                self.args.settle_sec, self.args.feedback_timeout,
            )
            vector.extend(averaged[joint] for joint in arm.source_joints)

        formatted = "[" + ", ".join(f"{v:.{self.args.precision}f}" for v in vector) + "]"
        try:
            note = update_pose_in_config(
                self.arm_config_path, pose_name, vector, self.args.precision, robot_id=robot_id
            )
        except OSError as exc:
            # Do not tear down the whole teach session over a bad config path;
            # the captured pose is printed so it can still be pasted manually.
            self.get_logger().error(f"could not write anchor pose to {self.arm_config_path}: {exc}")
            self.get_logger().error(f"captured vector for '{pose_name}' ({robot_id}): {formatted}")
            return
        self.get_logger().info(f"{note}: {pose_name} ({robot_id}, {len(vector)}-dim) = {formatted}")
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

    def _prompt_playback_plan(self, key_reader: TerminalKeyReader) -> Optional[_PlaybackPlan]:
        """Ask how to replay, every time.

        Which mode a recording wants is a property of the motion, not of the
        bring-up: threading into a handle and crossing the table want different
        answers on the same running stack. The previous choice is the default so
        repeating a replay stays one keypress.
        """
        plan = self._playback_plan
        listing = ", ".join(f"{i}={name}" for i, name in enumerate(RETIMING_MODES))
        default_index = RETIMING_MODES.index(plan.mode)
        raw = self.prompt_line(
            key_reader,
            f"Playback mode [{listing}] (default {default_index}={plan.mode}): ",
        ).strip()
        mode = plan.mode
        if raw:
            try:
                mode = RETIMING_MODES[int(raw)]
            except (ValueError, IndexError):
                self.get_logger().warn(f"invalid mode '{raw}'; playback aborted")
                return None

        window = plan.smoothing_window_sec
        if mode in TIMING_PRESERVING and mode != AS_RECORDED:
            raw = self.prompt_line(
                key_reader,
                f"Smoothing window in seconds (default {window:g}, floor "
                f"{RECONSTRUCTION_WINDOW_SEC:g}): ",
            ).strip()
            if raw:
                try:
                    window = float(raw)
                except ValueError:
                    self.get_logger().warn(f"invalid window '{raw}'; playback aborted")
                    return None
                if window < 0.0 or not math.isfinite(window):
                    self.get_logger().warn("window must be finite and >= 0; playback aborted")
                    return None

        speed_scale = plan.speed_scale
        if mode in (SPEED_SCALE, TEMPO_SCALE):
            what = (
                "Tempo relative to the recording, taught timing kept"
                if mode == TEMPO_SCALE
                else "Speed relative to the recording, taught timing discarded"
            )
            raw = self.prompt_line(
                key_reader,
                f"{what} (default {speed_scale:g}, 2 = twice as fast): ",
            ).strip()
            if raw:
                try:
                    speed_scale = float(raw)
                except ValueError:
                    self.get_logger().warn(f"invalid speed '{raw}'; playback aborted")
                    return None
                if not (speed_scale > 0.0 and math.isfinite(speed_scale)):
                    self.get_logger().warn("speed must be finite and > 0; playback aborted")
                    return None

        self._playback_plan = _PlaybackPlan(
            mode=mode, speed_scale=speed_scale, smoothing_window_sec=window
        )
        return self._playback_plan

    def _plan_playback(self, trajectory, plan: _PlaybackPlan, path=None):
        """Re-time one recording under the chosen mode, against the arm limits.

        Cached on the recording and the settings: the time-optimal search runs
        the parameterization several times and takes 0.5-2.3 s on a 20 s
        recording, which is a visible pause between the keypress and the motion.
        Repeating a replay, or stepping a speed back and forth, then costs
        nothing. The file's modification time is part of the key, so a re-recorded
        trajectory is re-planned.
        """
        key = None
        if path is not None:
            try:
                key = (
                    str(path), path.stat().st_mtime_ns, plan.mode, plan.speed_scale,
                    plan.smoothing_window_sec, self.args.playback_resample_dt,
                )
            except OSError:
                key = None
        if key is not None and key in self._playback_cache:
            return self._playback_cache[key]

        result = self._retime_trajectory(trajectory, plan)
        if key is not None:
            # One entry per recording+settings; a handful covers a teach session
            # and each holds a few thousand points.
            if len(self._playback_cache) >= self.PLAYBACK_CACHE_ENTRIES:
                self._playback_cache.pop(next(iter(self._playback_cache)))
            self._playback_cache[key] = result
        return result

    def _retime_trajectory(self, trajectory, plan: _PlaybackPlan):
        joint_count = len(trajectory.joint_names)
        if joint_count % ARM_JOINT_COUNT:
            raise RuntimeError(
                f"recording has {joint_count} joints, not a multiple of {ARM_JOINT_COUNT}; "
                "cannot map the per-joint limits onto it"
            )
        max_velocity = list(NERO_MAX_VELOCITY) * (joint_count // ARM_JOINT_COUNT)
        times = [float(point.time_from_start) for point in trajectory.points]
        positions = [list(point.positions) for point in trajectory.points]
        return retime(
            times,
            positions,
            plan.mode,
            max_velocity=max_velocity,
            max_acceleration=default_acceleration(max_velocity),
            speed_scale=plan.speed_scale,
            smoothing_window_sec=plan.smoothing_window_sec,
            resample_dt=self.args.playback_resample_dt,
        )

    def _lead_in_for_start_offset(self, current_positions, result, offsets=()) -> float:
        """Seconds to blend from where the arm is into the replay's first point.

        A replay begins at the pose it was taught from. Commanding that pose at
        t=0 from somewhere else makes the controller close the whole gap in one
        cycle. The gap is measured here and covered by a lead-in long enough to
        cross it well under the joint speed limit, so the arm travels there
        instead of lunging.
        """
        first = result.positions[0]
        offset = max(abs(a - b) for a, b in zip(current_positions, first))
        if offset > self.PLAYBACK_MAX_START_OFFSET:
            # A duo recording takes this maximum over fourteen joints, so the
            # number alone does not say which arm to move.
            worst = max(offsets, key=lambda item: item[2]) if offsets else None
            where = (
                f"{worst[0]} joint{worst[1] + 1} is {worst[2]:.3f} rad"
                if worst
                else f"an arm is {offset:.3f} rad"
            )
            raise RuntimeError(
                f"{where} from the replay's first waypoint, more than the "
                f"{self.PLAYBACK_MAX_START_OFFSET:.2f} rad this will bridge; press 'm' "
                "to plan a move to the start pose, then replay"
            )
        requested = max(0.0, float(self.args.playback_lead_in_sec))
        needed = offset / self.PLAYBACK_LEAD_IN_SPEED
        lead_in = max(requested, needed)
        if lead_in > 0.0:
            self.get_logger().info(
                f"blending {offset:.3f} rad into the first waypoint over {lead_in:.2f}s"
            )
        return lead_in

    def _report_playback_plan(self, name: str, result) -> None:
        self.get_logger().info(
            f"{name}: {result.mode} -> {result.duration:.2f}s "
            f"({result.speed_achieved:.2f}x recorded), path deviation "
            f"{result.path_deviation:.4f} rad, velocity {result.velocity_utilisation:.2f} "
            f"and acceleration {result.acceleration_utilisation:.2f} of limit"
        )
        for note in result.notes:
            self.get_logger().info(f"  {note}")
        if result.velocity_utilisation > 1.0:
            # Not refused: the operator chose this replay and is at the keyboard,
            # and a taught motion at its taught speed is the conservative option.
            # But the controller clamps what it cannot command, and a large
            # enough mismatch shows up as a joint dropping to hold mid-motion.
            self.get_logger().warn(
                f"commanded velocity reaches {result.velocity_utilisation:.2f}x the joint "
                "limit; the controller will clamp it and tracking error may trip the "
                "per-joint hold. A slower 'tempo_scale' keeps the taught shape and "
                "brings it under; a wider window or 'speed_scale' also works."
            )

    def _build_arm_slice(
        self, arm: _ArmEndpoint, trajectory_msg: JointTrajectory, columns: list[int]
    ) -> JointTrajectory:
        """One arm's columns of a merged recording (joint1..7, unprefixed)."""
        msg = JointTrajectory()
        msg.joint_names = list(arm.source_joints)
        for point in trajectory_msg.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = [float(point.positions[i]) for i in columns]
            ros_point.velocities = [float(point.velocities[i]) for i in columns] if point.velocities else []
            ros_point.time_from_start = point.time_from_start
            msg.points.append(ros_point)
        return msg

    def _dispatch_slices(self, slices: list[tuple[_ArmEndpoint, JointTrajectory]]) -> None:
        """Publish every arm's slice back to back — this loop *is* the duo sync.

        A controller starts the trajectory when the message arrives, so anything
        between two publishes becomes a start-time offset between the arms. The
        slices are therefore all built before the first one goes out, and
        nothing sleeps or spins in between.

        A repetition restarts the trajectory rather than reinforcing it, so it
        repeats the whole set: both arms restart together or not at all.
        """
        for _ in range(max(1, self.args.publish_repetitions)):
            for arm, msg in slices:
                arm.trajectory_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.args.publish_interval > 0.0:
                time.sleep(self.args.publish_interval)

    def _arm_columns(
        self,
        arm: _ArmEndpoint,
        joint_names: list[str],
        *,
        allow_bare_names: bool,
    ) -> Optional[list[int]]:
        """Column indices in ``joint_names`` this arm owns (side-prefixed match)."""
        wanted = [f"{arm.side_prefix}{joint}" for joint in arm.source_joints]
        index = {name: i for i, name in enumerate(joint_names)}
        if all(name in index for name in wanted):
            return [index[name] for name in wanted]
        # Fall back to the bare (unprefixed) names for a single-arm recording.
        if allow_bare_names and all(joint in index for joint in arm.source_joints):
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

    def _start_pose_target(self, trajectory, dispatched) -> TransitionTarget:
        """A MoveIt target at the recording's first waypoint.

        Segments are concatenated in the registry's side order, which is the
        order the planning group declares its joints in, so a duo recording
        fills ``both_arms`` and a single-arm one fills that side's group.
        """
        if self.arm_config is None:
            raise RuntimeError("--arm-config not set; cannot plan a move to the start pose")
        robot_id = (
            "both_arms"
            if len(dispatched) > 1
            else _resource_robot_id(dispatched[0][0].namespace or "right_arm")
        )
        group = self.arm_config.groups.get(robot_id)
        if group is None:
            raise RuntimeError(
                f"arm_config declares no '{robot_id}' group; cannot plan the start pose"
            )
        first = trajectory.points[0].positions
        ordered = sorted(dispatched, key=lambda pair: _SIDE_ORDER.get(pair[0].namespace, 99))
        positions = [float(first[column]) for _, columns in ordered for column in columns]
        if len(positions) != len(group.joint_names):
            raise RuntimeError(
                f"recording covers {len(positions)} joints but planning group "
                f"'{group.planning_group}' has {len(group.joint_names)}"
            )
        return TransitionTarget(
            label=f"{trajectory.name} start pose",
            robot_id=robot_id,
            planning_group=group.planning_group,
            joint_names=group.joint_names,
            pose_names=(trajectory.name,),
            target_positions=tuple(positions),
        )

    def move_to_recording_start(self) -> None:
        """Plan and run a collision-checked move to the selected replay's start.

        Separate from playback on purpose: this is a move through free space that
        was never taught, so it is planned rather than bridged, and it happens
        only when asked for.
        """
        if self.state != ManagerState.PLAYBACK:
            raise RuntimeError("switch to playback mode first")
        _, trajectory, dispatched = self._resolve_selected_recording()
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError("did not receive fresh feedback from all arms")
        target = self._start_pose_target(trajectory, dispatched)
        offsets = self._start_offsets(trajectory, dispatched)
        worst = max(offsets, key=lambda item: item[2])
        self.get_logger().info(
            f"moving to '{trajectory.name}' start pose on '{target.planning_group}'; "
            f"furthest is {worst[0]} joint{worst[1] + 1} at {worst[2]:.3f} rad"
        )
        self._execute_plan(self._plan_to_target(target), target.label)
        self.print_status()

    def _start_offsets(self, trajectory, dispatched) -> list[tuple[str, int, float]]:
        """Per-arm, per-joint gap between where each arm is and the replay start."""
        first = trajectory.points[0].positions
        out: list[tuple[str, int, float]] = []
        for arm, columns in dispatched:
            msg = arm.latest
            if msg is None:
                raise RuntimeError(f"no feedback for {arm.label}")
            position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
            for index, (joint, column) in enumerate(zip(arm.source_joints, columns)):
                if joint not in position_map:
                    raise RuntimeError(f"{arm.label} feedback is missing {joint}")
                out.append((arm.label, index, abs(position_map[joint] - float(first[column]))))
        return out

    def _resolve_selected_recording(self):
        """The selected recording and which arm owns which of its columns."""
        path = self.selected_trajectory_path()
        if path is None:
            raise RuntimeError(f"no recordings in {self.library_dir}")
        trajectory = load_recorded_trajectory(path)
        if not trajectory.points:
            raise RuntimeError(f"recording '{trajectory.name}' has no points")

        recording_namespace = _recording_namespace(trajectory.metadata)
        dispatched: list[tuple[_ArmEndpoint, list[int]]] = []
        for arm in self.arms:
            columns = self._arm_columns(
                arm,
                list(trajectory.joint_names),
                allow_bare_names=_allow_bare_joint_match(
                    recording_namespace=recording_namespace,
                    arm_namespace=arm.namespace,
                    arm_count=len(self.arms),
                ),
            )
            if columns is not None:
                dispatched.append((arm, columns))
        if not dispatched:
            if len(self.arms) > 1 and all(
                joint_name in set(trajectory.joint_names) for joint_name in self.source_joints
            ):
                raise RuntimeError(
                    f"recording '{trajectory.name}' has bare joint names but no usable arm owner in "
                    f"metadata (namespace={recording_namespace or '<missing>'}); refusing ambiguous duo playback"
                )
            raise RuntimeError(
                f"recording joints {list(trajectory.joint_names)} match none of the arms "
                f"{[a.label for a in self.arms]}; record it with a matching resource"
            )
        return path, trajectory, dispatched

    def playback_selected(self, key_reader: Optional[TerminalKeyReader] = None) -> None:
        if self.state != ManagerState.PLAYBACK:
            raise RuntimeError("switch to playback mode first")
        plan = self._playback_plan
        if key_reader is not None:
            plan = self._prompt_playback_plan(key_reader)
            if plan is None:
                return
        path, trajectory, dispatched = self._resolve_selected_recording()
        if not self.wait_for_source_feedback(self.args.feedback_timeout):
            raise RuntimeError("did not receive fresh feedback from all arms before playback")

        # Re-timed at playback (raw recordings stay untouched): the taught
        # samples carry no usable derivatives, and the mode decides whether the
        # taught timing survives or the path is traversed on a new one.
        try:
            result = self._plan_playback(trajectory, plan, path)
        except RetimingError as exc:
            raise RuntimeError(f"cannot replay '{trajectory.name}': {exc}") from None
        self._report_playback_plan(trajectory.name, result)

        current_positions = self._current_positions_for_trajectory(trajectory, dispatched)
        lead_in_sec = self._lead_in_for_start_offset(
            current_positions, result, self._start_offsets(trajectory, dispatched)
        )
        full_msg = retimed_to_joint_trajectory(
            result,
            trajectory.joint_names,
            current_positions=current_positions,
            lead_in_sec=lead_in_sec,
        )
        # Build every slice before publishing any of it; for a duo recording
        # this is the direct-controller sync path (bypasses MoveIt).
        self._dispatch_slices([
            (arm, self._build_arm_slice(arm, full_msg, columns))
            for arm, columns in dispatched
        ])
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

    def _plan_to_target(self, target: TransitionTarget) -> RobotTrajectory:
        """Plan a joint-space move to ``target`` through MoveIt.

        Shared by anchor transitions and the move to a replay's start pose: both
        need a collision-checked path, which a straight joint-space bridge is
        not.
        """
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
            raise RuntimeError(
                f"planning a move to {target.label} failed with MoveIt "
                f"error_code={result.error_code.val}"
            )
        if not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError(f"planning a move to {target.label} returned an empty trajectory")
        self.get_logger().info(
            f"planned a move to {target.label} on '{target.planning_group}' "
            f"({len(result.planned_trajectory.joint_trajectory.points)} point(s), "
            f"planning_time={result.planning_time:.3f}s)"
        )
        return result.planned_trajectory

    def _execute_plan(self, plan: RobotTrajectory, label: str) -> None:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan
        wrapper = self._send_goal_and_wait(self._execute_trajectory_client, goal, "ExecuteTrajectory")
        result = wrapper.result
        status = getattr(wrapper, "status", GoalStatus.STATUS_UNKNOWN)
        if status != GoalStatus.STATUS_SUCCEEDED or result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"executing the move to {label} failed with status={status}, "
                f"MoveIt error_code={result.error_code.val}"
            )
        self.get_logger().info(f"executed the move to {label} via MoveIt")

    def plan_selected_transition(self) -> None:
        if self.state != ManagerState.TRANSITIONS:
            raise RuntimeError("switch to transitions mode first")
        target = self.selected_transition_target()
        if target is None:
            raise RuntimeError("no anchor transition target available")
        try:
            plan = self._plan_to_target(target)
        except RuntimeError:
            self._clear_transition_plan()
            raise
        self.pending_transition_plan = plan
        self.pending_transition_target_label = target.label
        self.get_logger().info("Press 'f' again to execute.")
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
        self._execute_plan(self.pending_transition_plan, target.label)
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
        if key == "g":
            self.enter_hand_mode()
            return
        if key == "e":
            self.enter_gripper_mode()
            return
        if key == "[":
            if self.state == ManagerState.TRANSITIONS:
                self.select_next_transition(-1)
            elif self.state == ManagerState.HAND:
                self.select_next_hand_skill(-1)
            elif self.state == ManagerState.GRIPPER:
                self.select_next_gripper(-1)
            else:
                self.select_next(-1)
            return
        if key == "]":
            if self.state == ManagerState.TRANSITIONS:
                self.select_next_transition(1)
            elif self.state == ManagerState.HAND:
                self.select_next_hand_skill(1)
            elif self.state == ManagerState.GRIPPER:
                self.select_next_gripper(1)
            else:
                self.select_next(1)
            return
        if key == "a":
            self.capture_anchor(key_reader)
            return
        if key == "w":
            self.convert_to_waypoints(key_reader)
            return
        if key == "v":
            self.capture_gravity_sample(key_reader)
            return
        if self.state == ManagerState.RECORD and key == "n":
            self.record_sample(key_reader)
            return
        if self.state == ManagerState.HAND and key == "c":
            self.capture_hand_skill(key_reader)
            return
        if self.state == ManagerState.HAND and key == "f":
            self.play_hand_skill(key_reader)
            return
        if self.state == ManagerState.GRIPPER and key == "f":
            self.prompt_gripper_closure(key_reader)
            return
        if self.state == ManagerState.GRIPPER and key == "o":
            self.command_gripper_closure(0.0)
            return
        if self.state == ManagerState.GRIPPER and key == "c":
            self.command_gripper_closure(1.0)
            return
        if self.state == ManagerState.PLAYBACK and key == "f":
            self.playback_selected(key_reader)
            return
        if self.state == ManagerState.PLAYBACK and key == "m":
            self.move_to_recording_start()
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
            "  g -> hand mode        e -> gripper mode (parallel jaw)\n"
            "  a -> capture current pose as a named anchor\n"
            "  w -> convert selected recording -> catalogue waypoints\n"
            "  v -> log the gravity residual here (freedrive; guide the arm, press v per pose)\n"
            "  [ / ] -> select previous / next item (recording, anchor, or hand skill)\n"
            "  s -> status   h -> help   q -> quit\n"
            "Record mode:   n -> record a new trajectory\n"
            "Playback mode: f -> play selected   m -> move to its start pose (MoveIt)\n"
            "               c -> cancel active trajectory\n"
            "Transitions:  f -> plan selected target, press f again -> execute cached plan, c -> clear cached plan\n"
            "Gripper mode: f -> enter a closure (0.0 open .. 1.0 closed), o -> open, c -> close\n"
            "              (normalized closure, not metres; goes through the same\n"
            "               FollowJointTrajectory server MoveIt and the coordinator use)\n"
            "Hand mode:    c -> capture current hand pose as a skill, f -> replay selected skill\n"
            "              (wrapped in a prepare_hand_window/resume_arm_control handshake\n"
            "               only on the shared-bus topology; parallel otherwise)\n"
            "With two arms, record/anchor ask which resource to save "
            "(both_arms -> merged 14-dim, or one side -> 7-dim).\n"
        )

    def print_status(self) -> None:
        self.refresh_library()
        selected = self.selected_trajectory_path()
        transition = self.selected_transition_target()
        arms = ", ".join(arm.label for arm in self.arms)
        gripper_arm = self.selected_gripper_arm()
        if gripper_arm is None:
            gripper = "<none>"
        else:
            closure = self._gripper_closure_for(gripper_arm.label)
            gripper = (
                f"{gripper_arm.label}@"
                f"{'?' if closure is None else f'{closure:.2f}'}"
            )
        print(
            "Status: "
            f"state={self.state.value}, "
            f"arms=[{arms}], "
            f"hand_source={self._current_hand_arm().label}, "
            f"gripper={gripper}, "
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
        self._setup_hand_io()
        self._setup_gripper_io()
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
    # Recording has no rate argument: a capture stores one sample per feedback
    # update, so the arm's cadence is the recording's cadence and the stored
    # sample_rate_hz is what it delivered. The recording is in turn the ceiling
    # on what playback can reproduce — the controller interpolates between
    # samples, it cannot invent detail the capture never took.
    # See docs/sprint_refactor/reference/feedback_rate_budget.md.
    parser.add_argument("--hold-timeout", type=float, default=3.0)
    parser.add_argument(
        "--pre-roll-sec", type=float, default=0.25,
        help=(
            "Seconds of stillness kept before the first detected movement. The "
            "interval between arming the recorder and moving is dropped; the "
            "pre-roll keeps the physical start that a cut at the threshold "
            "crossing would clip. One cut instant for all arms, so a duo "
            "recording keeps its relative phase."
        ),
    )
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
        "--playback-smoothing-sec", type=float, default=DEFAULT_SMOOTHING_WINDOW_SEC,
        help=(
            "Starting default for the moving-average window asked for in 'smooth' "
            f"mode. A blunt local filter, not a fit. Values below the "
            f"{RECONSTRUCTION_WINDOW_SEC:.2f}s reconstruction floor have no effect: "
            "every timing-preserving replay filters at least that much."
        ),
    )
    parser.add_argument(
        "--playback-mode",
        choices=list(RETIMING_MODES),
        default=SMOOTH,
        help=(
            "Starting default for the mode asked at each replay: 'as_recorded' keeps the "
            "taught path and pace at the smallest filter that executes, 'smooth' widens "
            "that filter to trade path deviation for quieter derivatives, 'speed_scale' "
            "re-times to a multiple of the recorded duration, 'maximize_speed' runs the "
            "path as fast as the joint limits allow. 'tempo_scale' keeps the taught "
            "timing structure and only stretches the clock, which is what a take that "
            "was taught too fast needs"
        ),
    )
    parser.add_argument(
        "--playback-resample-dt",
        type=float,
        default=DEFAULT_RESAMPLE_DT,
        help=(
            "Output sample period for a replay, in every mode. Match it to the MIT "
            "control period: the controller interpolates linearly between the points "
            "it is given, so this is the grid the commanded motion is built on."
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
    # A trajectory publish is not idempotent: the controller restarts execution
    # from t=0 on every message, so a repetition replays the start rather than
    # reinforcing delivery. The topic is RELIABLE; one publish is a delivered
    # publish. Above 1 only as a deliberate diagnostic.
    parser.add_argument("--publish-repetitions", type=int, default=1)
    parser.add_argument("--publish-interval", type=float, default=0.0)
    parser.add_argument("--auto-enable-arm", action="store_true", help="Call enable_agx_arm before mode switches")
    parser.add_argument("--no-keyboard", action="store_true")
    parser.add_argument("--urdf-path", default="")
    # Gravity residual capture ('v'): compared against the same model the MIT
    # controller runs, so the fit corrects what is actually commanding the arm.
    parser.add_argument(
        "--gravity-urdf", default="",
        help="Gravity URDF the residual is measured against. Wins over --gravity-custom-model.",
    )
    parser.add_argument(
        "--gravity-custom-model", default="",
        help="Xacro the gravity URDF is derived from per arm side "
             "(e.g. src/duo_body_description/urdf/duo_system.urdf.xacro)",
    )
    parser.add_argument("--gravity-effector-type", default="omnihand")
    parser.add_argument(
        "--gravity-dwell", type=float, default=1.0,
        help="Seconds of feedback logged per 'v' press",
    )
    parser.add_argument("--gravity-csv-dir", default="logs")
    parser.add_argument(
        "--gravity-feedforward-sign", type=float, default=-1.0,
        help="The MIT controller's gravity_feedforward_sign. The measured torque "
             "is divided by it so the logged residual is in the model's own sign "
             "convention; a mismatch shows up as a fitted scale of -1.",
    )
    # Hand mode ('g'): capture ('c') / replay ('f') OmniHand skills. Whether each
    # is wrapped in a prepare_hand_window/resume_arm_control handshake follows
    # the declared bus topology; see --hand-window below.
    parser.add_argument(
        "--gripper-action-topic",
        default="gripper_controller/follow_joint_trajectory",
        help="Parallel-gripper FollowJointTrajectory action (namespaced per arm); "
             "the same surface MoveIt and the coordinator use",
    )
    parser.add_argument(
        "--gripper-status-topic", default="feedback/gripper_status",
        help="Parallel-gripper status topic (namespaced per arm) read for the "
             "normalized closure display",
    )
    parser.add_argument(
        "--gripper-timeout-sec", type=float, default=10.0,
        help="Max time to wait for a gripper trajectory goal to finish",
    )
    parser.add_argument(
        "--hand-gestures", default="",
        help="Path to omnihand_pro_gestures.yaml; enables hand mode ('g') when set/resolvable",
    )
    parser.add_argument(
        "--hand-feedback-topic", default="feedback/omnihand/joint_states",
        help="OmniHand JointState feedback topic read for skill capture",
    )
    parser.add_argument(
        "--hand-command-topic", default="control/omnihand/joint_target",
        help="Authority-carrying HandJointTarget topic a replayed skill is "
             "published to, after claiming the hand",
    )
    parser.add_argument(
        "--hand-arm", default="",
        help="Arm namespace whose prepare/resume services gate the window (default: first arm)",
    )
    parser.add_argument(
        "--hand-settle-sec", type=float, default=2.0,
        help="Fallback dwell after publishing a hand skill when the bridge has "
             "no delivery status (mock/older bridge)",
    )
    parser.add_argument(
        "--hand-status-topic", default="feedback/omnihand/status",
        help="OmniHand bridge status topic (namespaced per arm) used to hold the "
             "window open until a hand command is confirmed delivered",
    )
    parser.add_argument(
        "--hand-delivery-timeout-sec", type=float, default=4.0,
        help="Max time to hold the hand window waiting for delivery confirmation "
             "before closing it anyway",
    )
    # Derived from the one declared topology, never typed here: on dedicated
    # per-device buses the handshake quiesces an arm for a hand that shares no
    # bus with it, and does it silently. The explicit flags stay as
    # compatibility inputs and are refused when they contradict the registry.
    window = parser.add_mutually_exclusive_group()
    window.add_argument(
        "--hand-window", dest="hand_window", action="store_true",
        help="Force the arm<->hand prepare/resume handshake (shared-bus topology only)",
    )
    window.add_argument(
        "--no-hand-window", dest="hand_window", action="store_false",
        help="Skip the arm<->hand prepare/resume handshake and command the hand "
             "directly. Only safe when the hand has its own CAN bus (parallel "
             "operation); on a shared bus the hand needs the window.",
    )
    parser.set_defaults(hand_window=None)
    args = parser.parse_args()
    args.hand_window = (
        handshake_required()
        if args.hand_window is None
        else assert_matches_topology("hand_window", args.hand_window)
    )
    return args


def main() -> None:
    args = parse_args()
    if args.pre_roll_sec < 0.0:
        raise ValueError("--pre-roll-sec must be >= 0")
    if args.playback_speed_scale <= 0.0:
        raise ValueError("--playback-speed-scale must be > 0")
    if args.playback_lead_in_sec < 0.0:
        raise ValueError("--playback-lead-in-sec must be >= 0")
    if args.playback_resample_dt <= 0.0:
        raise ValueError("--playback-resample-dt must be > 0")
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
                node.cancel_active()
                node.enter_idle_mode()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["main", "TeachManagerNode", "ManagerState"]
