#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
import yaml

from agx_arm_ctrl.device_authority import (
    CommandStamp,
    DeviceAuthority,
    DeviceState,
    UnitSafety,
    UnitSafetySnapshot,
)
from agx_arm_msgs.srv import ClaimDevice
from agx_arm_msgs.msg import (
    AgxDeviceAuthority,
    AgxUnitSafety,
    AuthorizedJointTrajectory,
    HandJointTarget,
    OmniHandStatus,
    OmniHandTactileRaw,
)

from agx_arm_ctrl.motion_registry import bus_topology, hand_sides
from agx_arm_ctrl.runtime_metrics import MeasuredSdk, RuntimeMetrics, name_os_thread
from agx_arm_ctrl.sdk_worker import CallOutcome, Lane, SdkWorker
from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, HandModel, get_hand_model


def resolve_can_interface(hand_side: str) -> tuple[str, str]:
    """Return (interface_name, source) for this hand's OWN bus.

    Read from ``omnihand.sides.<side>.can_port``. It used to come from
    ``arm.sides.<side>.can_port`` with a built-in fallback to the arm buses,
    which is how a full four-bus bring-up ran with both hand bridges pointed at
    the arm interfaces: they timed out continuously, published nothing, and cost
    290 % of a core doing it. Nothing in the logs said "wrong bus", because from
    the bridge's point of view it had resolved an interface successfully.

    Returns an empty name rather than guessing. The caller fails closed for a
    hardware backend; a silently wrong interface is worse than a refusal,
    because the arm bus will happily accept frames no hand will ever answer.
    """
    try:
        side_cfg = hand_sides().get(hand_side, {})
        interface = str(side_cfg.get("can_port", "")).strip()
        if interface:
            return interface, "duo_motion_registry.yaml omnihand.sides"
    except Exception:
        pass
    return "", "not declared"


# The legacy O10 joint set / limits / mirror / tactile map come from the O10 hand
# model, which is itself registry-driven (duo_motion_registry.yaml omnihand.o10) —
# so this module no longer keeps a second hardcoded O10 list. These constants are
# the model=None fallback used by build_joint_names/load_gesture_presets; the O10
# SDK-internal motor/actuator calibration below stays here (not a description asset).
_O10_MODEL = get_hand_model("o10")
JOINT_SUFFIXES = list(_O10_MODEL.joint_suffixes)

TACTILE_FINGERS = [tuple(finger) for finger in _O10_MODEL.tactile_fingers]

SDK_TACTILE_LAYOUT_NAME = ",".join(name for name, _ in TACTILE_FINGERS)

SDK_ACTIVE_JOINT_COUNT = len(JOINT_SUFFIXES)
SDK_PADDED_VECTOR_COUNT = SDK_ACTIVE_JOINT_COUNT + 2
SDK_LEFT_POS_DIRECTION = list(_O10_MODEL.left_pos_direction)
SDK_ACTIVE_JOINT_MAX_RIGHT = list(_O10_MODEL.active_joint_max_right)
SDK_ACTIVE_JOINT_MIN_RIGHT = list(_O10_MODEL.active_joint_min_right)
SDK_MOTOR_MAX_RIGHT = [1.12, 0.05, 1.33, 0.0, 1.43, 1.43, 0.17, 1.43, 0.19, 1.43]
SDK_MOTOR_MIN_RIGHT = [-0.03, -1.64, 0.0, -0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SDK_ACTUATOR_MAX_RIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4096.0, 0.0, 0.0, 0.0]
SDK_ACTUATOR_MIN_RIGHT = [4096.0, 4096.0, 4096.0, 4096.0, 4096.0, 4096.0, 0.0, 4096.0, 4096.0, 4096.0]
SDK_ACTUATOR_MAX_LEFT = [4096.0, 0.0, 4096.0, 0.0, 0.0, 0.0, 4096.0, 0.0, 0.0, 0.0]
SDK_ACTUATOR_MIN_LEFT = [0.0, 4096.0, 0.0, 4096.0, 4096.0, 4096.0, 0.0, 4096.0, 4096.0, 4096.0]
SDK_FINGER_MOTOR2MCP_POLY = [
    -0.000257594494466942,
    1.57144033291557,
    0.217395210463076,
    -0.768328304426314,
    0.248168989312469,
]
SDK_RIGHT_THUMB_MOTOR2MCP_POLY = [
    -0.000677604838762652,
    1.05175893483608,
    -0.280133575638901,
    -0.115384415912668,
    0.0676128925382166,
]
SDK_LEFT_THUMB_MOTOR2MCP_POLY = [
    0.000677604838762652,
    1.05175893483608,
    0.280133575638901,
    -0.115384415912668,
    -0.0676128925382166,
]
SDK_STATUS_READ_INTERVAL_S = 1.0
SDK_TACTILE_READ_INTERVAL_S = 1.0

# A hand has two legitimate production motion primitives, and they may never
# command it at the same time. They are told apart by the surface a command
# arrives on, because `sensor_msgs/JointState` and `trajectory_msgs/JointTrajectory`
# carry no sender identity and ROS does not reveal a publisher to a subscriber.
#
# So an owner declares its primitive in the owner_id it claims with,
# `<primitive>:<node_name>`, and the bridge checks the surface against it. The
# node half is not decoration: it is how a crashed owner is detected, since a
# claim outlives the process that took it.
#
# The limit, stated rather than papered over: surface is a proxy for identity.
# Anything else publishing on the reactive surface looks like the reactive
# primitive. Only identity carried *per command* closes that, and the message
# that can carry it is the consolidated hand contract (phase 4D).
# A hand's claim service must NOT be called `claim_device`: the arm driver serves
# that name in the same per-side namespace, so both resolve to the identical
# `/<side>_arm/claim_device` and a client silently reaches whichever it found
# first. On hardware that meant a hand trajectory took the ARM's authority, left
# the hand unclaimed, and then failed delivery having sent nothing — with the
# refusal naming `arm_right`, which is what gave it away.
HAND_CLAIM_SERVICE = "control/omnihand/claim_device"

PRIMITIVE_TRAJECTORY = "trajectory"
PRIMITIVE_REACTIVE = "reactive"
SURFACE_PRIMITIVES = {
    "joint_state": PRIMITIVE_REACTIVE,
    "joint_trajectory": PRIMITIVE_TRAJECTORY,
    # The authority-carrying surfaces (4D). Same two primitives, but the command
    # brings its own identity instead of the bridge inventing one.
    "hand_joint_target": PRIMITIVE_REACTIVE,
    "authorized_trajectory": PRIMITIVE_TRAJECTORY,
}


def owner_primitive(owner_id: str) -> str:
    """The motion primitive an owner_id declares, or "" when it declares none."""
    primitive, separator, _node = owner_id.partition(":")
    if not separator:
        return ""
    return primitive if primitive in SURFACE_PRIMITIVES.values() else ""


def owner_node_name(owner_id: str) -> str:
    """The node half of a structured owner_id, used to detect a dead owner."""
    _primitive, separator, node = owner_id.partition(":")
    return node if separator else ""


def build_joint_names(hand_side: str, model: HandModel | None = None) -> list[str]:
    """Return prefixed active-joint names for the side.

    Model-aware: when a HandModel is given, its joint_suffixes win (e.g. the 12
    o12_pro joints). model=None keeps the legacy O10 module layout for backward
    compatibility (the O10 SDK backend and older callers).
    """
    if model is not None:
        return model.build_joint_names(hand_side)
    prefix = f"{hand_side}_"
    return [f"{prefix}{suffix}" for suffix in JOINT_SUFFIXES]


# Named active-joint presets live in config/omnihand_gestures.yaml (installed to
# the package share); that file is the single source of truth. This dict is only
# a last-resort fallback if the file cannot be read. Keep both in sync.
GESTURE_CONFIG = "omnihand_gestures.yaml"
FALLBACK_GESTURE_PRESETS: dict[str, list[float]] = {
    "open": [0.58, -0.21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def load_gesture_presets(model: HandModel | None = None) -> dict[str, list[float]]:
    """Return named OmniHand active-joint presets from the package config.

    Model-aware: each model carries its own preset file (model.gesture_config_file,
    e.g. omnihand_pro_gestures.yaml for o12_pro) and its own active-joint order and
    count. model=None keeps the legacy O10 file/layout for backward compatibility.
    The matching config file is the single source of truth; callers (the exerciser,
    future skill controllers) read from here instead of carrying their own copies.
    Every preset is validated to carry exactly the model's active-joint count,
    ordered to match its joint_suffixes. Falls back to a small built-in set only if
    the config file cannot be read.
    """
    config_file = model.gesture_config_file if model is not None else GESTURE_CONFIG
    expected_order = list(model.joint_suffixes) if model is not None else list(JOINT_SUFFIXES)

    try:
        config_path = (
            Path(get_package_share_directory("agx_arm_ctrl"))
            / "config"
            / config_file
        )
        data = yaml.safe_load(config_path.read_text()) or {}
    except Exception:
        # The built-in fallback only covers the legacy O10 layout; for any other
        # model an unreadable config is a hard error rather than a wrong-shape pose.
        if model is not None and len(expected_order) != len(JOINT_SUFFIXES):
            raise RuntimeError(
                f"could not read gesture config {config_file} for model "
                f"'{model.name}'; no safe built-in fallback for its joint layout"
            )
        return {name: list(values) for name, values in FALLBACK_GESTURE_PRESETS.items()}

    declared_order = data.get("omnihand_active_joint_order")
    if declared_order is not None and list(declared_order) != expected_order:
        raise RuntimeError(
            f"omnihand_active_joint_order in {config_file} does not match the "
            "model's joint order; gesture vectors would be misordered"
        )

    raw_gestures = data.get("omnihand_gestures") or {}
    presets: dict[str, list[float]] = {}
    for name, values in raw_gestures.items():
        vector = [float(value) for value in values]
        if len(vector) != len(expected_order):
            raise RuntimeError(
                f"gesture '{name}' in {config_file} has {len(vector)} values, "
                f"expected {len(expected_order)}"
            )
        presets[str(name)] = vector

    if not presets and model is None:
        return {name: list(values) for name, values in FALLBACK_GESTURE_PRESETS.items()}
    return presets


def mirror_active_joint_vector(
    values: list[float], model: HandModel | None = None
) -> list[float]:
    """Mirror a right-hand active-joint vector into the left-hand convention.

    The vendor presets are calibrated for the right hand (every value fits the
    right-hand limits and is out of range for the left). The left hand uses the
    mirrored sign convention captured by the model's left_pos_direction (the same
    direction vector its joint limits mirror with), so a component-wise multiply
    maps a right-hand pose to the matching left-hand pose. model=None keeps the
    legacy O10 direction vector for backward compatibility.
    """
    direction_vector = (
        list(model.left_pos_direction) if model is not None else SDK_LEFT_POS_DIRECTION
    )
    return [
        direction * float(value)
        for direction, value in zip(direction_vector, values, strict=True)
    ]


def resolve_gesture_presets(
    hand_side: str, model: HandModel | None = None
) -> dict[str, list[float]]:
    """Return the named presets in the convention of the selected hand side.

    The config file is the single source of truth and stores the canonical
    right-hand vectors; the left hand is derived by mirroring so there is no
    second copy to keep in sync. The bridge still clamps every target to the
    side's joint limits as a final safety net. Pass the HandModel to resolve the
    correct per-model preset file and mirror convention.
    """
    presets = load_gesture_presets(model)
    if hand_side == "left":
        return {name: mirror_active_joint_vector(vec, model) for name, vec in presets.items()}
    return {name: list(vec) for name, vec in presets.items()}


# Built vendor package, relative to the repo root. It carries the compiled
# agibot_hand_core .so, whose RUNPATH is $ORIGIN, so no LD_LIBRARY_PATH is
# needed — getting this directory onto sys.path is sufficient to import the SDK.
_VENDOR_PKG_REL = Path("vendor") / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"


def _locate_builtin_vendor_pkg() -> str | None:
    """Search upward from this file for the repo's built agibot_hand package."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _VENDOR_PKG_REL
        if (candidate / "agibot_hand" / "__init__.py").exists():
            return str(candidate)
    return None


def _ensure_omnihand_importable(sdk_python_dir: str = "") -> None:
    """Make agibot_hand importable, locating the built package if needed.

    Tries the ambient environment first (an already-set PYTHONPATH wins), then
    an explicit dir, the AGX_ARM_OMNIHAND_SDK_DIR env var, and finally an upward
    search for the repo's built package. This lets `ros2 launch ... backend_type:=sdk`
    work without manually exporting PYTHONPATH/LD_LIBRARY_PATH.
    """
    try:
        import_module("agibot_hand")
        return
    except ImportError:
        pass

    candidates = [
        sdk_python_dir,
        os.environ.get("AGX_ARM_OMNIHAND_SDK_DIR", ""),
        _locate_builtin_vendor_pkg() or "",
    ]
    last_error: ImportError | None = None
    for candidate in candidates:
        if not candidate or not Path(candidate).is_dir():
            continue
        if candidate not in sys.path:
            sys.path.insert(0, candidate)
        sys.modules.pop("agibot_hand", None)
        try:
            import_module("agibot_hand")
            return
        except ImportError as exc:
            last_error = exc

    raise RuntimeError(
        "backend_type=sdk requires the agibot_hand vendor package, which was not "
        "on PYTHONPATH and could not be located automatically. Set the bridge "
        "'sdk_python_dir' parameter or the AGX_ARM_OMNIHAND_SDK_DIR env var to the "
        "built package (vendor/OmniHand-Pro-2025/build/agibot_hand_pkg)."
    ) from last_error


def _load_sdk_symbols(sdk_python_dir: str = "") -> tuple[type[Any], Any, Any]:
    _ensure_omnihand_importable(sdk_python_dir)
    try:
        module = import_module("agibot_hand")
    except ImportError as exc:
        raise RuntimeError(
            "backend_type=sdk could not import agibot_hand even after the repo auto-discovery path ran; set the bridge 'sdk_python_dir' parameter or AGX_ARM_OMNIHAND_SDK_DIR only if the built vendor package lives outside the repo checkout"
        ) from exc

    missing = [
        symbol_name
        for symbol_name in ("AgibotHandO12", "EFinger", "EHandType")
        if not hasattr(module, symbol_name)
    ]
    if missing:
        raise RuntimeError(
            "Installed agibot_hand package is missing required symbols: "
            + ", ".join(missing)
        )

    return module.AgibotHandO12, module.EFinger, module.EHandType


def _create_sdk_hand(
    sdk_class: type[Any],
    *,
    device_id: int,
    hand_type: Any,
) -> Any:
    return sdk_class(device_id=device_id, hand_type=hand_type)


def _coerce_float_list(values: object, expected_count: int, value_label: str) -> list[float]:
    normalized = [float(value) for value in list(values)]
    if len(normalized) != expected_count:
        raise RuntimeError(
            f"{value_label} length mismatch: expected {expected_count}, got {len(normalized)}"
        )
    return normalized


def _coerce_bool_list(values: object, expected_count: int, attr_name: str) -> list[bool]:
    normalized = [bool(getattr(value, attr_name, False)) for value in list(values)]
    if len(normalized) != expected_count:
        raise RuntimeError(
            f"{attr_name} length mismatch: expected {expected_count}, got {len(normalized)}"
        )
    return normalized


def _evaluate_polynomial(value: float, coefficients: list[float]) -> float:
    result = 0.0
    power = 1.0
    for coefficient in coefficients:
        result += coefficient * power
        power *= value
    return result


def _mirror_joint_limits(
    max_values: list[float],
    min_values: list[float],
) -> tuple[list[float], list[float]]:
    mirrored_max = list(max_values)
    mirrored_min = list(min_values)
    for index, direction in enumerate(SDK_LEFT_POS_DIRECTION):
        if direction == -1:
            mirrored_max[index] = -min_values[index]
            mirrored_min[index] = -max_values[index]
    return mirrored_max, mirrored_min


def _sdk_joint_model(hand_side: str) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]:
    if hand_side == "left":
        active_joint_max, active_joint_min = _mirror_joint_limits(
            SDK_ACTIVE_JOINT_MAX_RIGHT,
            SDK_ACTIVE_JOINT_MIN_RIGHT,
        )
        motor_max, motor_min = _mirror_joint_limits(
            SDK_MOTOR_MAX_RIGHT,
            SDK_MOTOR_MIN_RIGHT,
        )
        return (
            active_joint_min,
            active_joint_max,
            motor_min,
            motor_max,
            SDK_ACTUATOR_MIN_LEFT,
            SDK_ACTUATOR_MAX_LEFT,
            SDK_LEFT_THUMB_MOTOR2MCP_POLY,
        )

    return (
        SDK_ACTIVE_JOINT_MIN_RIGHT,
        SDK_ACTIVE_JOINT_MAX_RIGHT,
        SDK_MOTOR_MIN_RIGHT,
        SDK_MOTOR_MAX_RIGHT,
        SDK_ACTUATOR_MIN_RIGHT,
        SDK_ACTUATOR_MAX_RIGHT,
        SDK_RIGHT_THUMB_MOTOR2MCP_POLY,
    )


def _trim_sdk_vector(values: object, expected_count: int, value_label: str) -> tuple[list[Any], bool]:
    normalized = list(values)
    actual_count = len(normalized)
    if actual_count < expected_count:
        raise RuntimeError(
            f"{value_label} length mismatch: expected at least {expected_count}, got {actual_count}"
        )
    trimmed = actual_count > expected_count
    return normalized[:expected_count], trimmed


def _clamp_sdk_joint_targets(
    values: list[float],
    min_values: list[float],
    max_values: list[float],
) -> list[float]:
    return [
        min(max(min_values[index], float(value)), max_values[index])
        for index, value in enumerate(values)
    ]


@dataclass
class OmniHandStatusSnapshot:
    backend_name: str
    control_mode: str
    connected: bool
    initialized: bool
    is_mock: bool
    communication_fault: bool
    active_joint_temperatures_c: list[float]
    active_joint_currents_a: list[float]
    active_joint_stalled: list[bool]
    active_joint_over_temperature: list[bool]
    active_joint_over_current: list[bool]
    status_text: str


@dataclass
class OmniHandTactileSnapshot:
    backend_name: str
    layout_name: str
    values: list[float]


class MockOmniHandBackend:

    def __init__(
        self,
        hand_side: str,
        tactile_sample_count: int,
        joint_names: list[str] | None = None,
    ) -> None:
        self.hand_side = hand_side
        self.backend_name = "mock_backend"
        self.control_mode = "idle"
        self.connected = True
        self.initialized = True
        self.is_mock = True
        self.communication_fault = False
        self.status_text = "mock backend ready"
        # Model-aware joint set when provided (e.g. 12 joints for o12_pro);
        # falls back to the O10 joint set for backward compatibility.
        self.joint_names = list(joint_names) if joint_names is not None else build_joint_names(hand_side)
        self.positions = [0.0] * len(self.joint_names)
        self.temperatures_c = [25.0] * len(self.joint_names)
        self.currents_a = [0.0] * len(self.joint_names)
        self.stalled = [False] * len(self.joint_names)
        self.over_temperature = [False] * len(self.joint_names)
        self.over_current = [False] * len(self.joint_names)
        self.tactile_values = [0.0] * max(0, tactile_sample_count)

    def get_joint_names(self) -> list[str]:
        return list(self.joint_names)

    def apply_joint_targets(self, target_map: dict[str, float], control_mode: str) -> int:
        matched_joint_count = 0
        for index, joint_name in enumerate(self.joint_names):
            if joint_name in target_map:
                self.positions[index] = float(target_map[joint_name])
                matched_joint_count += 1

        if matched_joint_count == 0:
            raise ValueError("received command with no recognized OmniHand joints")

        self.control_mode = control_mode
        self.status_text = (
            f"applied mock {control_mode} command with {matched_joint_count} commanded joints"
        )
        return matched_joint_count

    def apply_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            raise ValueError("received JointTrajectory with no points")

        final_point = msg.points[-1]
        if len(final_point.positions) != len(msg.joint_names):
            raise ValueError("joint_names and final point positions length mismatch")

        target_map = dict(zip(msg.joint_names, final_point.positions, strict=True))
        self.apply_joint_targets(target_map, "joint_trajectory")

    def stop(self) -> None:
        self.control_mode = "stopped"
        self.currents_a = [0.0] * len(self.joint_names)
        self.status_text = "mock stop requested"

    def read_joint_state(self) -> list[float]:
        return list(self.positions)

    def read_status(self) -> OmniHandStatusSnapshot:
        return OmniHandStatusSnapshot(
            backend_name=self.backend_name,
            control_mode=self.control_mode,
            connected=self.connected,
            initialized=self.initialized,
            is_mock=self.is_mock,
            communication_fault=self.communication_fault,
            active_joint_temperatures_c=list(self.temperatures_c),
            active_joint_currents_a=list(self.currents_a),
            active_joint_stalled=list(self.stalled),
            active_joint_over_temperature=list(self.over_temperature),
            active_joint_over_current=list(self.over_current),
            status_text=self.status_text,
        )

    def read_tactile(self) -> OmniHandTactileSnapshot:
        return OmniHandTactileSnapshot(
            backend_name=self.backend_name,
            layout_name="flat_array",
            values=list(self.tactile_values),
        )


class SdkOmniHandBackend:

    def __init__(self, hand_side: str, device_id: int, canfd_id: int, cfg_path: str, sdk_python_dir: str = "", can_interface: str = "", metrics: RuntimeMetrics | None = None) -> None:
        self.hand_side = hand_side
        self._metrics = metrics or RuntimeMetrics(enabled=False)
        self.device_id = device_id
        self.canfd_id = canfd_id
        self.cfg_path = cfg_path
        self.sdk_python_dir = sdk_python_dir
        self.can_interface = can_interface
        # The vendor SocketCAN backend reads the interface ONLY from this env var.
        # Export the repo-owned side-bus name explicitly so the hand opens on the
        # public runtime interface (for example can_nero_right) instead of a legacy
        # kernel-facing alias such as can0.
        if can_interface:
            os.environ["OMNIHAND_SOCKETCAN_IFACE"] = can_interface
        self.backend_name = "vendor_sdk"
        self.control_mode = "active_joint_control"
        self.connected = False
        self.initialized = False
        self.is_mock = False
        self.communication_fault = False
        self.status_text = "sdk backend initializing"
        self.joint_names = build_joint_names(hand_side)
        self.positions = [0.0] * len(self.joint_names)
        self.temperatures_c = [0.0] * len(self.joint_names)
        self.currents_a = [0.0] * len(self.joint_names)
        self.stalled = [False] * len(self.joint_names)
        self.over_temperature = [False] * len(self.joint_names)
        self.over_current = [False] * len(self.joint_names)
        self.tactile_values = [0.0] * len(TACTILE_FINGERS)
        self._active_joint_min, self._active_joint_max, self._motor_min, self._motor_max, self._actuator_min, self._actuator_max, self._thumb_motor2mcp_poly = _sdk_joint_model(hand_side)
        self._saw_padded_vectors = False
        self._last_status_read_s = 0.0
        self._last_tactile_read_s = 0.0
        # How often the vendor tactile sample is actually refreshed. A
        # diagnostic cadence by default; the bridge raises it while a
        # reactive owner holds the hand, because contact-seeking motion
        # ends where this sensor says and cannot wait a second to hear it.
        self.tactile_read_interval_s = SDK_TACTILE_READ_INTERVAL_S
        self._temperature_reports_supported = False
        self._current_reports_supported = False

        sdk_class, finger_enum, hand_type_enum = _load_sdk_symbols(sdk_python_dir)
        hand_type = getattr(hand_type_enum, hand_side.upper())
        self._tactile_finger_entries = [
            (layout_name, getattr(finger_enum, enum_name))
            for layout_name, enum_name in TACTILE_FINGERS
        ]
        # Wrapping the session rather than each call site is what makes the
        # coverage complete: a call nobody thought to measure is still measured,
        # and that is exactly the one that turns out to dominate. ~150 % of a
        # core per hand lives behind this object and has never been decomposed.
        self.hand = MeasuredSdk(
            _create_sdk_hand(
                sdk_class,
                device_id=device_id,
                hand_type=hand_type,
            ),
            self._metrics,
        )
        if hasattr(self.hand, "show_data_details"):
            self.hand.show_data_details(False)

        self.connected = True
        self.initialized = True
        self.status_text = (
            f"sdk backend ready (active joint control, can_interface={can_interface or 'can_nero_right(default)'}, "
            f"device_id={device_id})"
        )
        try:
            self.positions = self.read_joint_state()
            self.control_mode = "active_joint_hold"
        except Exception:
            # Keep startup tolerant; the periodic feedback timer will continue retrying.
            pass

    def _status_suffix(self) -> str:
        if not self._saw_padded_vectors:
            return ""
        return " (trimmed padded 12-value SDK vectors to 10 active channels)"

    def _set_fault(self, message: str, exc: Exception) -> None:
        self.communication_fault = True
        self.connected = False
        self.status_text = f"{message}: {exc}{self._status_suffix()}"

    def _clear_fault(self, message: str) -> None:
        self.communication_fault = False
        self.connected = True
        self.status_text = f"{message}{self._status_suffix()}"

    def _current_active_joint_targets(self) -> list[float]:
        if any(self.positions):
            return list(self.positions)
        return self.read_joint_state()

    def _normalize_numeric_vector(self, values: object, value_label: str) -> list[float]:
        trimmed_values, trimmed = _trim_sdk_vector(values, len(self.joint_names), value_label)
        if trimmed:
            self._saw_padded_vectors = True
        return [float(value) for value in trimmed_values]

    def _normalize_report_vector(self, values: object, value_label: str) -> tuple[list[Any], bool]:
        trimmed_values, trimmed = _trim_sdk_vector(values, len(self.joint_names), value_label)
        if trimmed:
            self._saw_padded_vectors = True
        return trimmed_values, trimmed

    def _convert_motor_positions_to_active_joint_angles(self, motor_positions: object) -> list[float]:
        trimmed_positions, trimmed = _trim_sdk_vector(
            motor_positions,
            len(self.joint_names),
            "motor positions",
        )
        if trimmed:
            self._saw_padded_vectors = True

        active_joint_pos: list[float] = []
        for index, raw_position in enumerate(trimmed_positions):
            denominator = self._actuator_max[index] - self._actuator_min[index]
            if denominator == 0.0:
                raise RuntimeError(f"invalid actuator range at index {index}")
            active_joint_pos.append(
                (float(raw_position) - self._actuator_min[index])
                * (self._motor_max[index] - self._motor_min[index])
                / denominator
                + self._motor_min[index]
            )

        active_joint_pos[2] = _evaluate_polynomial(
            active_joint_pos[2],
            self._thumb_motor2mcp_poly,
        )
        for finger_index in (4, 5, 7, 9):
            active_joint_pos[finger_index] = _evaluate_polynomial(
                active_joint_pos[finger_index],
                SDK_FINGER_MOTOR2MCP_POLY,
            )

        return [
            min(max(self._active_joint_min[index], value), self._active_joint_max[index])
            for index, value in enumerate(active_joint_pos)
        ]

    def get_joint_names(self) -> list[str]:
        return list(self.joint_names)

    def apply_joint_targets(self, target_map: dict[str, float], control_mode: str) -> int:
        matched_joint_count = sum(1 for joint_name in self.joint_names if joint_name in target_map)
        if matched_joint_count == 0:
            raise ValueError("received command with no recognized OmniHand joints")

        target_positions = self._current_active_joint_targets()
        for index, joint_name in enumerate(self.joint_names):
            if joint_name in target_map:
                target_positions[index] = float(target_map[joint_name])

        target_positions = _clamp_sdk_joint_targets(
            target_positions,
            self._active_joint_min,
            self._active_joint_max,
        )

        try:
            self.hand.set_all_active_joint_angles(target_positions)
        except Exception as exc:
            self._set_fault(f"sdk {control_mode} command failed", exc)
            raise RuntimeError("sdk backend rejected active joint command") from exc

        self.positions = list(target_positions)
        self.control_mode = control_mode
        self._clear_fault(
            f"applied sdk {control_mode} command with {matched_joint_count} active joints"
        )
        return matched_joint_count

    def apply_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            raise ValueError("received JointTrajectory with no points")

        final_point = msg.points[-1]
        if len(final_point.positions) != len(msg.joint_names):
            raise ValueError("joint_names and final point positions length mismatch")

        target_map = dict(zip(msg.joint_names, final_point.positions, strict=True))
        self.apply_joint_targets(target_map, "joint_trajectory")

    def stop(self) -> None:
        """Cancel the pending target and hold where the hand ACTUALLY is.

        The pose is taken from a fresh readback, not from the cached one. The
        cache holds the last *commanded* target: apply_joint_targets writes it
        optimistically, before the hand has moved. Commanding that back is not a
        stop, it re-sends the destination the hand is already travelling to.

        Whether that happened used to depend on timing. A readback between the
        command and the stop overwrote the cache and the hand held its position;
        without one, it kept closing.
        """
        try:
            hold_positions = self.read_joint_state()
            self.hand.set_all_active_joint_angles(hold_positions)
            self.positions = list(hold_positions)
            self.control_mode = "stopped"
            self._clear_fault("sdk stop requested; holding current active joint pose")
        except Exception as exc:
            self._set_fault("sdk stop request failed", exc)

    def read_joint_state(self) -> list[float]:
        try:
            if hasattr(self.hand, "get_all_joint_positions"):
                self.positions = self._convert_motor_positions_to_active_joint_angles(
                    self.hand.get_all_joint_positions()
                )
                self._clear_fault("sdk readback active via motor-position compatibility")
            else:
                self.positions = self._normalize_numeric_vector(
                    self.hand.get_all_active_joint_angles(),
                    "active joint angles",
                )
                self._clear_fault("sdk readback active")
        except Exception as exc:
            try:
                self.positions = self._normalize_numeric_vector(
                    self.hand.get_all_active_joint_angles(),
                    "active joint angles",
                )
                self._clear_fault("sdk readback active")
            except Exception as fallback_exc:
                self._set_fault("sdk joint readback failed", fallback_exc)

        return list(self.positions)

    def read_status(self) -> OmniHandStatusSnapshot:
        now = time.monotonic()
        if now - self._last_status_read_s >= SDK_STATUS_READ_INTERVAL_S:
            try:
                error_reports, _ = self._normalize_report_vector(
                    self.hand.get_all_error_reports(),
                    "error reports",
                )
                if self._current_reports_supported:
                    try:
                        self.currents_a = self._normalize_numeric_vector(
                            self.hand.get_all_current_reports(),
                            "current reports",
                        )
                    except Exception:
                        self._current_reports_supported = False
                if self._temperature_reports_supported:
                    try:
                        self.temperatures_c = self._normalize_numeric_vector(
                            self.hand.get_all_temperature_reports(),
                            "temperature reports",
                        )
                    except Exception:
                        self._temperature_reports_supported = False
                self.stalled = [bool(getattr(report, "stalled", False)) for report in error_reports]
                self.over_temperature = [bool(getattr(report, "overheat", False)) for report in error_reports]
                self.over_current = [bool(getattr(report, "over_current", False)) for report in error_reports]
                self._last_status_read_s = now
                self._clear_fault("sdk status readback active")
            except Exception as exc:
                self._last_status_read_s = now
                self._set_fault("sdk status readback failed", exc)

        return OmniHandStatusSnapshot(
            backend_name=self.backend_name,
            control_mode=self.control_mode,
            connected=self.connected,
            initialized=self.initialized,
            is_mock=self.is_mock,
            communication_fault=self.communication_fault,
            active_joint_temperatures_c=list(self.temperatures_c),
            active_joint_currents_a=list(self.currents_a),
            active_joint_stalled=list(self.stalled),
            active_joint_over_temperature=list(self.over_temperature),
            active_joint_over_current=list(self.over_current),
            status_text=self.status_text,
        )

    def read_tactile(self) -> OmniHandTactileSnapshot:
        now = time.monotonic()
        if now - self._last_tactile_read_s >= self.tactile_read_interval_s:
            tactile_values: list[float] = []
            supported_finger_entries: list[tuple[str, Any]] = []
            for layout_name, finger_value in self._tactile_finger_entries:
                try:
                    finger_samples = [float(value) for value in self.hand.get_tactile_sensor_data(finger_value)]
                except Exception:
                    continue
                if not finger_samples:
                    continue
                tactile_values.extend(finger_samples)
                supported_finger_entries.append((layout_name, finger_value))

            if supported_finger_entries:
                self._tactile_finger_entries = supported_finger_entries
                self.tactile_values = tactile_values
            self._last_tactile_read_s = now

        return OmniHandTactileSnapshot(
            backend_name=self.backend_name,
            layout_name=",".join(name for name, _ in self._tactile_finger_entries),
            values=list(self.tactile_values),
        )


class OmniHandBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("omnihand_bridge_node")

        self.declare_parameter("omnihand_type", "right")
        self.declare_parameter("hand_model", DEFAULT_HAND_MODEL)
        self.declare_parameter("backend_type", "mock")
        # A CEILING on publication, not the rate anything runs at. It used to
        # drive the one timer that did everything, and every bringup passes the
        # ARM's pub_rate (200) into it — so the hand published three messages
        # 200 times a second while its joints changed 20 times and its status and
        # tactile once. Measured cost of that mistake: 41.5 % of a core against
        # 4.5 % at 20 Hz, on the mock backend where no CAN is involved at all.
        # Publication is now driven by new data; this only throttles it further.
        self.declare_parameter("pub_rate", 50.0)
        # Status carries the command-delivery verdict the FollowJointTrajectory
        # action waits on, so it is published the moment that verdict changes.
        # This is the floor underneath it: what a consumer gets when nothing is
        # happening, keeping joint_readback_age_s and liveness observable.
        self.declare_parameter("status_heartbeat_rate", 2.0)
        # Hand joint readback is a real CAN request per poll; on the shared
        # arm+hand bus this competes with the 50 Hz MIT command stream, so the
        # SDK poll rate is decoupled from the ROS publish rate. <= 0 polls on
        # every publish tick (legacy behavior).
        self.declare_parameter("joint_read_rate", 20.0)
        self.declare_parameter("tactile_sample_count", 32)
        # Tactile has two cadences because it serves two different consumers.
        # As diagnostics it is a once-a-second sample nobody is waiting for. To
        # the reactive primitive it is the signal that ENDS the motion — a hand
        # closing at 20 Hz cannot act on a reading up to a second old, and the
        # skill controller rejects one that is (`tactile_stale_sec`).
        self.declare_parameter("tactile_read_rate", 1.0)
        self.declare_parameter("tactile_reactive_read_rate", 20.0)
        self.declare_parameter("joint_states_command_topic", "control/joint_states")
        self.declare_parameter("device_id", 1)
        self.declare_parameter("canfd_id", 0)
        self.declare_parameter("sdk_cfg_path", "")
        self.declare_parameter("sdk_python_dir", "")
        self.declare_parameter("can_interface", "")
        # Command delivery on the congested shared bus is lossy (one-shot CAN +
        # low hand arbitration priority drops frames under arm load). Commands
        # are idempotent position setpoints, so the bridge re-sends the latest
        # target until the readback confirms it, up to max_attempts. Eventual
        # delivery matters more than latency here, hence the generous budget
        # (8 x 0.3 s ~ 2.4 s worst case).
        self.declare_parameter("command_retry_enabled", True)
        self.declare_parameter("command_retry_max_attempts", 8)
        self.declare_parameter("command_retry_period_s", 0.3)
        self.declare_parameter("command_verify_tolerance_rad", 0.10)
        # When the backend reports a communication fault (请求超时 storms), every
        # further periodic SDK poll is a real CANFD request that feeds the bus
        # error flood — with one-shot off a failing FD frame already keeps the
        # CAN controller retransmitting. Back off to a slow single-probe cadence
        # until one readback succeeds, instead of hammering at joint_read_rate
        # plus status plus tactile.
        self.declare_parameter("fault_poll_interval_s", 2.0)
        # Per-SDK-call attribution: which vendor calls, how many a second, how
        # long each blocks. The bridge's process cost is known and its ROS half
        # is now measured; this is what decomposes the rest. Off by default
        # because it costs CPU on the Jetson, and the point is to measure the
        # bridge, not the instrument.
        self.declare_parameter("runtime_metrics_enabled", False)
        self.declare_parameter("runtime_metrics_period_s", 10.0)
        # Fail-closed command admission. Off only for a rig that deliberately
        # drives the bridge with no authority layer above it; production and
        # every supported bring-up leave it on.
        self.declare_parameter("command_admission_enforced", True)
        # A claim outlives the process that took it, so a crashed owner would
        # hold a hand nobody can command. The bridge watches for the owner's node
        # leaving the graph and revokes after this grace period. Revoking clears
        # the owner and bumps the device epoch, which is what makes the hand
        # non-commandable until someone claims it again explicitly — deliberately
        # not a forced STANDBY, because `_sync_authority` derives the state every
        # tick and would overwrite anything written behind its back.
        self.declare_parameter("owner_liveness_grace_s", 3.0)
        # Bare JointState / JointTrajectory ingress, development only. Those
        # surfaces carry no identity, so the bridge would stamp them from its own
        # current state and every staleness check would pass by construction.
        # Not an authority-safe path.
        self.declare_parameter("allow_legacy_hand_command_ingress", False)

        self.hand_side = str(self.get_parameter("omnihand_type").value)
        # This device in the authority contract. Note it is *not* the
        # scheduler's resource name: the coordinator's resource for the left
        # hand is "left_hand" while its device is "hand_left". Two contracts,
        # deliberately not derived from one another.
        self.device_id = f"hand_{self.hand_side}"
        # NOTE: this hand has no device-level emergency stop of its own.
        # `control/omnihand/stop` cancels the pending target — it is a
        # cancel, not a latching stop, and the skill flow relies on that.
        # So a hand goes STOPPED only through the unit generation, unlike an
        # arm, which can latch its own. Closing that asymmetry belongs with
        # the consolidated hand contract (Phase 4D), and is recorded rather
        # than faked here.
        # A transport authority: what it owns is this hand's SDK session and
        # CAN transport, not the semantics of a grasp, which stays with the
        # skill controller.
        self._unit_safety = UnitSafety(self.device_id, writer=False)
        self._authority = DeviceAuthority(self.device_id, self._unit_safety)
        self.metrics = RuntimeMetrics(
            enabled=bool(self.get_parameter("runtime_metrics_enabled").value),
            report_period_s=float(self.get_parameter("runtime_metrics_period_s").value),
        )
        self.hand_model = get_hand_model(str(self.get_parameter("hand_model").value))
        self.backend_type = str(self.get_parameter("backend_type").value)
        self.pub_rate = float(self.get_parameter("pub_rate").value)
        self.status_heartbeat_rate = float(self.get_parameter("status_heartbeat_rate").value)
        self.joint_read_rate = float(self.get_parameter("joint_read_rate").value)
        self.command_retry_enabled = bool(self.get_parameter("command_retry_enabled").value)
        self.command_retry_max_attempts = max(
            1, int(self.get_parameter("command_retry_max_attempts").value)
        )
        self.command_retry_period_s = max(
            0.05, float(self.get_parameter("command_retry_period_s").value)
        )
        self.command_verify_tolerance_rad = float(
            self.get_parameter("command_verify_tolerance_rad").value
        )
        self.fault_poll_interval_s = max(
            0.0, float(self.get_parameter("fault_poll_interval_s").value)
        )
        self._fault_backoff_active = False
        # Hysteresis: one lucky probe must not flip the bridge back to full-rate
        # polling on a bus that is still congested (fault -> recover -> burst ->
        # fault oscillation). Full rate resumes only after this many consecutive
        # successful probes at the slow cadence.
        self._fault_recovery_streak = 0
        self._fault_recovery_streak_needed = 3
        # A hand that stays unreachable turns every probe into another frame
        # that may sit in the shared TX path retransmitting (one-shot off).
        # After this many consecutive failed probes the cadence is stretched
        # 5x and a single ERROR points at the hand instead of the bus.
        self._failed_probe_streak = 0
        self._failed_probe_escalation_threshold = 10
        self._probe_escalated = False
        self._last_status_snapshot: OmniHandStatusSnapshot | None = None
        self._last_tactile_snapshot: OmniHandTactileSnapshot | None = None
        self.tactile_sample_count = int(self.get_parameter("tactile_sample_count").value)
        tactile_rate = float(self.get_parameter("tactile_read_rate").value)
        self.tactile_read_interval_s = 1.0 / tactile_rate if tactile_rate > 0.0 else 0.0
        reactive_rate = float(self.get_parameter("tactile_reactive_read_rate").value)
        self.tactile_reactive_interval_s = (
            1.0 / reactive_rate if reactive_rate > 0.0 else 0.0
        )
        self.joint_states_command_topic = str(
            self.get_parameter("joint_states_command_topic").value
        )
        self.allow_legacy_hand_command_ingress = bool(
            self.get_parameter("allow_legacy_hand_command_ingress").value
        )
        self.device_id = int(self.get_parameter("device_id").value)
        self.canfd_id = int(self.get_parameter("canfd_id").value)
        self.sdk_cfg_path = str(self.get_parameter("sdk_cfg_path").value)
        self.sdk_python_dir = str(self.get_parameter("sdk_python_dir").value)
        if self.hand_side not in ("left", "right"):
            raise ValueError("omnihand_type must be 'left' or 'right'")

        # Resolve the native SocketCAN interface: explicit param wins, else the
        # side -> interface mapping from the motion registry (arm.sides.*.can_port).
        self.can_interface = str(self.get_parameter("can_interface").value).strip()
        interface_source = "can_interface parameter"
        if not self.can_interface:
            self.can_interface, interface_source = resolve_can_interface(self.hand_side)

        if self.backend_type == "sdk":
            if not self.can_interface:
                raise ValueError(
                    f"backend_type=sdk needs this hand's own SocketCAN interface and "
                    f"none is declared for side '{self.hand_side}'. Add it under "
                    "omnihand.sides.<side>.can_port in duo_motion_registry.yaml, or "
                    "pass can_interface for this run. Refusing rather than falling "
                    "back to the arm bus, where a hand is never reached."
                )
            self.get_logger().info(
                f"OmniHand SocketCAN interface: {self.can_interface} "
                f"(from {interface_source}; bus topology {bus_topology()})"
            )
            if self.hand_model.name == "o12_pro":
                # Lazy import: only pull in the agibot_hand SDK when actually
                # driving the Pro hand over SDK.
                from agx_arm_ctrl.omnihand.sdk_o12_pro import O12ProSdkBackend

                self.backend = O12ProSdkBackend(
                    model=self.hand_model,
                    hand_side=self.hand_side,
                    device_id=self.device_id,
                    sdk_python_dir=self.sdk_python_dir,
                    can_interface=self.can_interface,
                    metrics=self.metrics,
                )
            else:
                self.backend = SdkOmniHandBackend(
                    hand_side=self.hand_side,
                    device_id=self.device_id,
                    canfd_id=self.canfd_id,
                    cfg_path=self.sdk_cfg_path,
                    sdk_python_dir=self.sdk_python_dir,
                    can_interface=self.can_interface,
                    metrics=self.metrics,
                )
        else:
            if self.backend_type != "mock":
                self.get_logger().warn(
                    f"Unsupported backend_type={self.backend_type}; falling back to mock backend"
                )
            self.backend = MockOmniHandBackend(
                hand_side=self.hand_side,
                tactile_sample_count=self.tactile_sample_count,
                joint_names=self.hand_model.build_joint_names(self.hand_side),
            )

        self.backend_type = self.backend.backend_name
        self.joint_names = self.backend.get_joint_names()
        self.joint_name_set = frozenset(self.joint_names)

        # Latest verified-delivery state: the newest hand target replaces any
        # older pending one (latest wins), and cached_positions holds the last
        # real SDK readback used both for publishing and for verification.
        self.pending_command: dict[str, Any] | None = None
        self.cached_positions: list[float] = list(
            getattr(self.backend, "positions", [0.0] * len(self.joint_names))
        )
        self.last_joint_read_monotonic = 0.0
        # Distinct from the above: that one advances on every probe, including
        # failed ones (it paces the poll). This one marks the last readback the
        # hand actually answered, and is what feedback/omnihand/status reports —
        # joint_states is republished from cache regardless, so its stamp cannot
        # tell a caller whether the hand is still there.
        self.last_good_joint_read_monotonic = 0.0
        # Latched when a target was given up on unverified; cleared by the next
        # command or by a stop. Lets the FollowJointTrajectory bridge fail the
        # goal instead of reporting SUCCEEDED on an undelivered pose.
        self._command_delivery_failed = False
        self.joint_read_min_interval_s = (
            1.0 / self.joint_read_rate if self.joint_read_rate > 0.0 else 0.0
        )
        # Publication is gated on new data. These record what was last put on the
        # wire so a tick with nothing new stays silent instead of re-serializing
        # the same three messages.
        # Generous against the measured worst case (a status read peaked at
        # 17.7 ms) but bounded, so a wedged SDK call cannot stall acquisition
        # for ever.
        self._sdk_read_timeout_s = 2.0
        self._admission_enforced = bool(
            self.get_parameter("command_admission_enforced").value
        )
        self._owner_liveness_grace_s = max(
            0.0, float(self.get_parameter("owner_liveness_grace_s").value)
        )
        # Per-epoch monotonic sequence. The authority rejects a stamp whose
        # sequence does not advance, so this is reset whenever the epoch moves —
        # a new owner starts a fresh sequence rather than inheriting a watermark
        # it never wrote.
        self._command_sequence = 0
        self._sequence_epoch = -1
        self._owner_missing_since = 0.0
        self._last_liveness_check = 0.0
        # Half the grace, so a dead owner is still noticed within it, bounded
        # below so a small grace cannot turn this back into a per-tick query.
        self._liveness_check_interval_s = max(0.5, self._owner_liveness_grace_s / 2.0)
        self._last_refusal = ""
        self._last_refusal_monotonic = 0.0
        self._tick_thread_named = False
        self._published_read_monotonic = 0.0
        self._last_joint_publish_monotonic = 0.0
        self._last_status_publish_monotonic = 0.0
        self._last_status_signature: tuple | None = None
        self._last_tactile_publish_monotonic = 0.0
        self._publish_min_interval_s = 1.0 / self.pub_rate if self.pub_rate > 0.0 else 0.0
        self._status_heartbeat_period_s = (
            1.0 / self.status_heartbeat_rate if self.status_heartbeat_rate > 0.0 else 0.0
        )
        # Tactile cannot change faster than the SDK is read, so publishing faster
        # only re-serializes the same array.
        # Tactile is published when a new sample lands, like the joints, rather
        # than on a fixed interval. The interval was equal to the read interval,
        # so at the diagnostic cadence a consumer saw every message arrive
        # exactly one staleness limit after the last one.
        self._tactile_acquired_monotonic = 0.0
        self._published_tactile_monotonic = 0.0
        self._last_tactile_acquire_monotonic = 0.0

        self.hand_joint_states_pub = self.create_publisher(
            JointState, "feedback/omnihand/joint_states", 10
        )
        self.status_pub = self.create_publisher(
            OmniHandStatus, "feedback/omnihand/status", 10
        )
        self.tactile_pub = self.create_publisher(
            OmniHandTactileRaw, "feedback/omnihand/tactile_raw", 10
        )

        if self.allow_legacy_hand_command_ingress:
            self.get_logger().warn(
                "LEGACY HAND COMMAND INGRESS ENABLED: bare JointState on "
                f"'{self.joint_states_command_topic}' and bare JointTrajectory on "
                "'control/omnihand/joint_trajectory' will move this hand. Those "
                "surfaces carry no commander, no generations and no sequence, so "
                "a stale or reordered command CANNOT be refused on them — the "
                "bridge has to stamp them from its own current state. "
                "Development only; never a production path."
            )
            self.create_subscription(
                JointState,
                self.joint_states_command_topic,
                self._joint_states_command_callback,
                10,
            )
            self.create_subscription(
                JointTrajectory,
                "control/omnihand/joint_trajectory",
                self._joint_trajectory_callback,
                10,
            )
        # The authority-carrying surfaces (4D), and by default the only way to
        # move this hand. Both feed one admission path; only the message shape
        # differs — a trajectory for planned execution, a target for the
        # reactive loop, which cannot be time-parameterized.
        self.create_subscription(
            AuthorizedJointTrajectory,
            "control/omnihand/authorized_trajectory",
            self._authorized_trajectory_callback,
            10,
        )
        self.create_subscription(
            HandJointTarget,
            "control/omnihand/joint_target",
            self._hand_joint_target_callback,
            10,
        )
        self.create_service(Trigger, "control/omnihand/stop", self._stop_callback)
        self.create_service(
            ClaimDevice, HAND_CLAIM_SERVICE, self._claim_device_callback
        )
        self.authority_pub = self.create_publisher(
            AgxDeviceAuthority, "feedback/authority",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._authority.set_on_change(self._publish_authority)
        self.create_subscription(
            AgxUnitSafety, "/unit_safety", self._unit_safety_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        # The timer paces ACQUISITION, which is the only thing here with a real
        # periodic need; publication rides along when there is something new.
        # Oversampled 2x against the readback interval on purpose: with a timer
        # period equal to the interval, ordinary jitter makes roughly every other
        # tick miss the `now - last >= interval` gate and the effective readback
        # rate lands nearer 13 Hz than the 20 Hz that was asked for.
        if self.joint_read_min_interval_s > 0.0:
            timer_period = self.joint_read_min_interval_s / 2.0
        else:
            timer_period = 1.0 / self.pub_rate if self.pub_rate > 0.0 else 0.02
        # Half a tick, so the readback gate rounds to the nearest tick instead of
        # always rounding up. Bounded by the interval itself so a degenerate
        # configuration cannot turn the gate into "always read".
        self._read_gate_tolerance_s = min(
            timer_period / 2.0, self.joint_read_min_interval_s / 2.0
        )

        # One owner for this hand's SDK session.
        #
        # The calls were already serialized, but only because the bridge spins
        # single-threaded. That is an accident: one edit to a
        # MultiThreadedExecutor ends it silently, and both sibling nodes in this
        # package already use one. It also bought nothing. A stop queued behind
        # whatever the executor was doing, and a blocking read sat on the
        # executor thread, so a 17 ms status read stopped the node answering its
        # own claim service. That was observed as a service "not answering".
        self._sdk = SdkWorker(
            self._authority.device_id, metrics=self.metrics, logger=self.get_logger()
        )
        # Guards the acquisition results the publication timer reads. The
        # acquisition thread writes them; the executor only reads.
        self._snapshot_lock = threading.Lock()
        self._acquisition_stop = threading.Event()
        self._acquisition_thread = threading.Thread(
            target=self._acquisition_loop,
            name=f"hand-acq-{self.hand_side}",
            daemon=True,
        )
        self._acquisition_thread.start()

        # Publication only. No SDK call reaches the executor from here.
        self.create_timer(timer_period, self._publication_tick)
        if self.command_retry_enabled:
            self.create_timer(self.command_retry_period_s, self._command_retry_tick)

        self.get_logger().info(
            "OmniHand bridge started with "
            f"hand_side={self.hand_side}, hand_model={self.hand_model.name} "
            f"({len(self.joint_names)} joints), backend_type={self.backend_type}, "
            f"joint_states_command_topic={self.joint_states_command_topic}"
        )

    def _joint_states_command_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        target_map = {
            joint_name: float(msg.position[index])
            for index, joint_name in enumerate(msg.name)
            if index < len(msg.position) and joint_name in self.joint_name_set
        }
        if not target_map:
            # Shared control/joint_states frequently contains arm-only updates.
            return

        self._submit_command(target_map, "joint_state")

    def _joint_trajectory_callback(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warn("Rejected OmniHand JointTrajectory: no points")
            return

        final_point = msg.points[-1]
        if len(final_point.positions) != len(msg.joint_names):
            self.get_logger().warn(
                "Rejected OmniHand JointTrajectory: joint_names and final point "
                "positions length mismatch"
            )
            return

        unknown_names = [name for name in msg.joint_names if name not in self.joint_name_set]
        if unknown_names:
            self.get_logger().warn(
                f"Ignored unknown OmniHand joints in trajectory: {', '.join(unknown_names)}"
            )

        target_map = {
            joint_name: float(position)
            for joint_name, position in zip(msg.joint_names, final_point.positions, strict=True)
            if joint_name in self.joint_name_set
        }
        if not target_map:
            self.get_logger().warn(
                "Rejected OmniHand JointTrajectory: no recognized OmniHand joints"
            )
            return

        self._submit_command(target_map, "joint_trajectory")

    def _stamp_from(self, authority) -> CommandStamp:
        """The authority the command arrived with, taken verbatim.

        Nothing is defaulted or repaired here. A commander that leaves a field
        empty is refused by admission rather than silently completed from the
        bridge's own state, which is the behaviour 4D exists to end.
        """
        return CommandStamp(
            owner_id=authority.owner_id,
            device_epoch=int(authority.device_epoch),
            unit_safety_epoch=int(authority.unit_safety_epoch),
            sequence=int(authority.sequence),
        )

    def _authorized_trajectory_callback(self, msg: AuthorizedJointTrajectory) -> None:
        """A trajectory that carries the authority it was issued under (4D).

        The MVP keeps the known-good vendor integration: the final point drives
        the existing position path, exactly as the compatibility topic does. What
        changes is that the trajectory reaches this boundary whole and the
        identity travels with it, so a goal issued under a superseded claim is
        refused here instead of executing.
        """
        trajectory = msg.trajectory
        if not trajectory.points:
            self.get_logger().warn(
                "Rejected AuthorizedJointTrajectory: no points"
            )
            return

        final_point = trajectory.points[-1]
        if len(final_point.positions) != len(trajectory.joint_names):
            self.get_logger().warn(
                "Rejected AuthorizedJointTrajectory: joint_names and final point "
                "positions length mismatch"
            )
            return

        target_map = {
            joint_name: float(position)
            for joint_name, position in zip(
                trajectory.joint_names, final_point.positions, strict=True
            )
            if joint_name in self.joint_name_set
        }
        if not target_map:
            self.get_logger().warn(
                "Rejected AuthorizedJointTrajectory: no recognized OmniHand joints"
            )
            return

        self._submit_command(
            target_map, "authorized_trajectory", self._stamp_from(msg.authority)
        )

    def _hand_joint_target_callback(self, msg: HandJointTarget) -> None:
        """One authority-stamped target from the reactive primitive (4D)."""
        if len(msg.positions) != len(msg.joint_names):
            self.get_logger().warn(
                "Rejected HandJointTarget: joint_names and positions length mismatch"
            )
            return

        target_map = {
            joint_name: float(position)
            for joint_name, position in zip(
                msg.joint_names, msg.positions, strict=True
            )
            if joint_name in self.joint_name_set
        }
        if not target_map:
            self.get_logger().warn(
                "Rejected HandJointTarget: no recognized OmniHand joints"
            )
            return

        self._submit_command(
            target_map, "hand_joint_target", self._stamp_from(msg.authority)
        )

    def _log_refusal(self, detail: str) -> None:
        """Say why a command was refused, without a flood.

        A refused stream is usually a stream: a controller that lost the claim
        keeps publishing at its control rate, and logging each one buries the
        first, which is the one that says what happened.
        """
        now = time.monotonic()
        if detail == self._last_refusal and now - self._last_refusal_monotonic < 5.0:
            return
        self._last_refusal = detail
        self._last_refusal_monotonic = now
        self.get_logger().warn(f"refused hand command: {detail}")

    def _check_owner_liveness(self) -> None:
        """Revoke a claim whose owner is no longer in the graph.

        A claim is state on this side, so a commander that crashes mid-grasp
        leaves the hand owned by nobody and commandable by no one. Recovering
        that by hand is not something an operator should have to know about.

        Revoking clears the owner and bumps the device epoch, so a new commander
        must claim explicitly; nothing is auto-transferred, because inheriting a
        device whose last commander died is exactly the case where the next one
        should have to say so.
        """
        if not self._admission_enforced or self._owner_liveness_grace_s <= 0.0:
            return
        # Rate-limited hard, because `get_node_names_and_namespaces` is a graph
        # query, not a field read. Running it on every feedback tick starved the
        # bridge's single-threaded executor badly enough that its own claim
        # service stopped answering within 5 s — the watchdog took the hand out
        # of service to check whether the hand was in service.
        now = time.monotonic()
        if now - self._last_liveness_check < self._liveness_check_interval_s:
            return
        self._last_liveness_check = now

        owner = self._authority.snapshot().owner_id
        node = owner_node_name(owner)
        if not node:
            self._owner_missing_since = 0.0
            return
        try:
            alive = any(name == node for name, _ns in self.get_node_names_and_namespaces())
        except Exception:
            return  # graph query failed; never revoke on a missing answer
        if alive:
            self._owner_missing_since = 0.0
            return

        now = time.monotonic()
        if self._owner_missing_since <= 0.0:
            self._owner_missing_since = now
            return
        if now - self._owner_missing_since < self._owner_liveness_grace_s:
            return

        self._owner_missing_since = 0.0
        self.get_logger().error(
            f"commander '{owner}' left the graph; revoking its claim on "
            f"{self._authority.device_id}. The hand accepts nothing until a "
            "new owner "
            "claims it."
        )
        self._authority.revoke(f"owner '{owner}' no longer present")
        self.pending_command = None

    def _admit_command(
        self, control_mode: str, stamp: "CommandStamp | None" = None
    ) -> tuple[bool, str]:
        """Decide whether a command from this surface may reach the hand.

        Fail-closed: an unclaimed hand executes nothing. That costs a migration —
        every caller now has to claim — and it is the point. A default-open gate
        would have left the two-commander race open for exactly the callers
        nobody remembered to convert, which is the set that causes the incident.

        ``stamp`` is the authority the command arrived with. When present it is
        judged as given and nothing here substitutes for a missing field: the
        bridge is not entitled to decide, on a commander's behalf, which era its
        command belongs to.
        """
        if not self._admission_enforced:
            return True, ""

        snapshot = self._authority.snapshot()
        owner = snapshot.owner_id
        if not owner:
            return False, (
                f"{self._authority.device_id} has no commander; claim it before "
                "commanding "
                "(claim_device)"
            )

        surface_primitive = SURFACE_PRIMITIVES.get(control_mode, "")
        declared = owner_primitive(stamp.owner_id if stamp is not None else owner)
        if declared and surface_primitive and declared != surface_primitive:
            return False, (
                f"{self._authority.device_id} is held by '{owner}' ({declared}); a "
                f"{surface_primitive} command may not preempt it"
            )

        if stamp is not None:
            # The command brought its own identity, so judge THAT. This is the
            # only form in which a stale epoch or an out-of-order sequence can
            # actually be refused: a stamp the bridge builds itself is always
            # current by construction, which is why those two checks passed
            # unconditionally before 4D.
            verdict = self._authority.admit(stamp)
            if not verdict.accepted:
                return False, f"{verdict.reason.value}: {verdict.detail}"
            return True, ""

        # Legacy self-stamped path (shared control/joint_states and the
        # bridge-local trajectory topic). Retained for migration only: it cannot
        # reject a stale or reordered command, because it invents the identity it
        # then checks. Ownership and surface are all that refuse anything here.
        if snapshot.device_epoch != self._sequence_epoch:
            # A new era: ownership changed, or the device rearmed. Start the
            # sequence again rather than carrying a watermark set by whoever held
            # it before.
            self._sequence_epoch = snapshot.device_epoch
            self._command_sequence = 0

        self._command_sequence += 1
        verdict = self._authority.admit(
            CommandStamp(
                owner_id=owner,
                device_epoch=snapshot.device_epoch,
                unit_safety_epoch=snapshot.unit_safety_epoch,
                sequence=self._command_sequence,
            )
        )
        if not verdict.accepted:
            return False, f"{verdict.reason.value}: {verdict.detail}"
        return True, ""

    def _submit_command(
        self,
        target_map: dict[str, float],
        control_mode: str,
        stamp: "CommandStamp | None" = None,
    ) -> None:
        """Send a hand target and keep it pending until the readback confirms it.

        On the shared CAN bus the hand's frames lose arbitration under arm load
        and get dropped silently (one-shot mode), so a single send is unreliable.
        Targets are absolute setpoints, so re-sending the latest one is safe.
        """
        admitted, refusal = self._admit_command(control_mode, stamp)
        if not admitted:
            self._log_refusal(refusal)
            return

        self._command_delivery_failed = False
        self.pending_command = {
            "targets": dict(target_map),
            "control_mode": control_mode,
            "attempts": 0,
            "last_send_monotonic": 0.0,
        }
        self._send_pending_command()
        if not self.command_retry_enabled:
            self.pending_command = None
        self._publish_command_verdict()

    def _send_pending_command(self) -> None:
        pending = self.pending_command
        if pending is None:
            return

        pending["attempts"] += 1
        pending["last_send_monotonic"] = time.monotonic()
        targets = dict(pending["targets"])
        mode = pending["control_mode"]
        # CONTROL lane, superseding, and deliberately NOT waited on. A newer
        # target makes an older queued one pointless, and delivering it late
        # would move the hand back; the epoch stamp drops whatever a previous
        # owner had queued. Waiting for the result here would put SDK latency
        # back onto the executor, which is what this whole path exists to avoid,
        # so the verdict is picked up by the retry tick instead.
        pending["call"] = self._sdk.submit(
            "apply_joint_targets",
            lambda: self.backend.apply_joint_targets(targets, mode),
            lane=Lane.CONTROL,
            epoch=self._authority.snapshot().device_epoch,
            replace_key="hand_target",
        )
        try:
            self._raise_if_send_failed(pending)
        except (ValueError, RuntimeError) as exc:
            attempts_left = pending["attempts"] < self.command_retry_max_attempts
            if self.command_retry_enabled and attempts_left:
                self.get_logger().warn(
                    f"OmniHand {pending['control_mode']} command send failed "
                    f"(attempt {pending['attempts']}/{self.command_retry_max_attempts}), "
                    f"retrying: {exc}"
                )
            else:
                self.get_logger().warn(
                    f"Ignoring OmniHand {pending['control_mode']} command: {exc}"
                )
                self._command_delivery_failed = True
                self.pending_command = None

    def _raise_if_send_failed(self, pending: dict) -> None:
        """Re-raise a send that the worker has already finished and failed.

        The submit does not block, so the failure surfaces one tick later. A call
        still PENDING is not a failure and not a success — it is in flight, and
        saying anything about the hand on that basis would be a guess.
        """
        call = pending.get("call")
        if call is None:
            return
        if call.outcome is CallOutcome.FAILED and call.error is not None:
            raise RuntimeError(str(call.error))
        if call.outcome in (CallOutcome.DROPPED, CallOutcome.REJECTED):
            raise ValueError(f"hand command {call.outcome.value}: {call.detail}")

    def _pending_command_verified(self, pending: dict[str, Any]) -> bool:
        if self.backend.communication_fault:
            return False
        # Only trust a real SDK readback taken after the last send; right after
        # apply_joint_targets the backend caches the target optimistically.
        if self.last_joint_read_monotonic <= pending["last_send_monotonic"]:
            return False
        positions = dict(zip(self.joint_names, self.cached_positions))
        return all(
            joint_name in positions
            and abs(positions[joint_name] - target) <= self.command_verify_tolerance_rad
            for joint_name, target in pending["targets"].items()
        )

    def _command_retry_tick(self) -> None:
        pending = self.pending_command
        if pending is None:
            return

        # The previous send was submitted, not awaited. This is where its verdict
        # arrives.
        try:
            self._raise_if_send_failed(pending)
        except (ValueError, RuntimeError) as exc:
            attempts_left = pending["attempts"] < self.command_retry_max_attempts
            if not (self.command_retry_enabled and attempts_left):
                self.get_logger().warn(
                    f"OmniHand {pending['control_mode']} command failed: {exc}"
                )
                self._command_delivery_failed = True
                self.pending_command = None
                self._publish_command_verdict()
                return
            pending["call"] = None

        if self._pending_command_verified(pending):
            if pending["attempts"] > 1:
                self.get_logger().info(
                    f"OmniHand {pending['control_mode']} command verified after "
                    f"{pending['attempts']} attempts"
                )
            self.pending_command = None
            self._publish_command_verdict()
            return

        # An attempt may only be spent once the previous send actually had a
        # chance to be judged, i.e. a real readback landed after it. Without
        # this the budget is consumed by the clock instead of by evidence: a
        # single 请求超时 puts the backend into fault backoff (one probe every
        # fault_poll_interval_s), while this tick keeps firing every
        # command_retry_period_s — 8 attempts burn in 2.4 s with at most one
        # readback in between, and the command is declared lost inside a hand
        # window that was opened precisely to deliver it.
        if self.last_joint_read_monotonic <= pending["last_send_monotonic"]:
            return

        if pending["attempts"] >= self.command_retry_max_attempts:
            self.get_logger().warn(
                f"OmniHand {pending['control_mode']} command not verified within "
                f"{pending['attempts']} attempts (tolerance "
                f"{self.command_verify_tolerance_rad:.3f} rad); giving up — fingers may be "
                "in contact or the bus is congested"
            )
            self._command_delivery_failed = True
            self.pending_command = None
            self._publish_command_verdict()
            return

        if time.monotonic() - pending["last_send_monotonic"] >= self.command_retry_period_s:
            self._send_pending_command()
            # A send can itself exhaust the budget and settle the command; the
            # signature gate makes a repeated verdict a no-op.
            self._publish_command_verdict()

    _AUTHORITY_STATE_CODES = {
        DeviceState.OFFLINE: AgxDeviceAuthority.STATE_OFFLINE,
        DeviceState.STANDBY: AgxDeviceAuthority.STATE_STANDBY,
        DeviceState.READY: AgxDeviceAuthority.STATE_READY,
        DeviceState.RECOVERING: AgxDeviceAuthority.STATE_RECOVERING,
        DeviceState.FAULTED: AgxDeviceAuthority.STATE_FAULTED,
        DeviceState.STOPPED: AgxDeviceAuthority.STATE_STOPPED,
    }

    def _publish_authority(self, snapshot) -> None:
        """Publish one transport-authority transition. Never breaks the caller."""
        try:
            msg = AgxDeviceAuthority()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.device_id = snapshot.device_id
            msg.state = self._AUTHORITY_STATE_CODES[snapshot.state]
            msg.device_epoch = snapshot.device_epoch
            msg.unit_safety_epoch = snapshot.unit_safety_epoch
            msg.unit_stopped = snapshot.unit_stopped
            msg.motion_ready = snapshot.motion_ready
            msg.owner_id = snapshot.owner_id
            msg.reason = snapshot.reason
            self.authority_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"publishing hand authority failed: {exc}")

    def _unit_safety_callback(self, msg: AgxUnitSafety) -> None:
        """Adopt a generation from the one writer that may allocate them."""
        adopted = self._unit_safety.observe(
            UnitSafetySnapshot(
                epoch=int(msg.epoch),
                stopped=bool(msg.stopped),
                reason=msg.reason,
                writer_id=msg.writer_id,
                incarnation=msg.writer_incarnation,
                started_ns=int(msg.writer_started_ns),
            )
        )
        if adopted:
            self.get_logger().warn(
                f"unit safety generation {msg.epoch}: stopped={msg.stopped} "
                f"({msg.reason})"
            )
        if not msg.stopped:
            return

        # Stop the hardware, do not just close the gate. The authority goes
        # STOPPED and refuses further commands, but a target already delivered
        # keeps executing: this hand drives to a position on its own once it has
        # accepted one. Without this, a unit stop during a closing grasp left the
        # hand closing.
        #
        # Unlike an arm, a hand has no local e-stop that would already have
        # stopped the hardware before the unit generation arrived, so the unit
        # stop is the only signal there is.
        self.pending_command = None
        self._command_delivery_failed = False
        try:
            self._sdk.submit_safety("stop", self.backend.stop)
        except Exception as exc:
            self.get_logger().error(
                f"unit stop reached {self._authority.device_id} but stopping the "
                f"hand failed: {exc}"
            )
            return
        self.get_logger().warn(
            f"unit stop: {self._authority.device_id} holding its measured pose"
        )

    def _claim_device_callback(self, request, response):
        """Take or give up the transport session for this hand."""
        verdict = (
            self._authority.claim(request.owner_id)
            if request.claim
            else self._authority.release(request.owner_id)
        )
        snapshot = self._authority.snapshot()
        response.accepted = verdict.accepted
        response.reason = "" if verdict.accepted else verdict.reason.value
        response.device_epoch = snapshot.device_epoch
        response.unit_safety_epoch = snapshot.unit_safety_epoch
        response.message = (
            f"{self._authority.device_id} transport "
            f"{'claimed by' if request.claim else 'released by'} "
            f"'{request.owner_id}'"
            if verdict.accepted
            else verdict.detail
        )
        # Two call sites on purpose. rclpy caches a logger's severity per call
        # site and raises if it ever changes, so a single site that logs INFO on
        # success and WARN on refusal throws the first time a claim is refused —
        # out of a service callback, killing the node. It survived review because
        # nothing ever refused a claim until the hand had two commanders.
        if verdict.accepted:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response

    def _sync_authority(self, reason: str) -> None:
        """Map the bridge's gates onto this hand's transport authority.

        Derived, like the arm driver's: these gates are already what the bridge
        acts on. The epochs are not derived — they come from the authority's own
        transitions.

        A communication fault maps to STANDBY rather than FAULTED on purpose.
        The bridge clears it by itself on the next successful call, and FAULTED
        is for something an operator has to acknowledge; using it for a
        self-clearing condition would leave a latch nothing here can release.
        """
        # Deliberately no local stop latch — see the note in __init__.
        authority = self._authority
        if self._unit_safety.stopped:
            return
        if authority.state is DeviceState.FAULTED:
            authority.acknowledge_fault(reason)

        snapshot = self._last_status_snapshot
        if snapshot is None or not snapshot.connected:
            authority.go_offline(f"{reason}: hand not connected")
            return
        if not snapshot.initialized:
            authority.go_standby(f"{reason}: hand not initialized")
            return
        if snapshot.communication_fault:
            authority.go_standby(f"{reason}: hand communication fault")
            return
        authority.rearm(verified=True, detail=reason)

    def _stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        # A stop supersedes any in-flight target; never re-send it afterwards,
        # and do not report the superseded target as a delivery failure.
        self.pending_command = None
        self._command_delivery_failed = False
        # SAFETY lane: ahead of everything queued, including a control transmit
        # that has not started. Not awaited — the caller gets the acknowledgement,
        # and blocking a service handler on the SDK is what used to stop this node
        # answering at all.
        self._sdk.submit_safety("stop", self.backend.stop)
        self._publish_command_verdict()
        response.success = True
        response.message = f"OmniHand {self.backend.backend_name} stop requested"
        return response

    def _reactive_owner_holds(self) -> bool:
        """True while the contact-seeking primitive owns this hand.

        The bridge already knows which primitive claimed it, so the cost of a
        fast tactile read is paid only when something is actually waiting on the
        sensor. A standing 20 Hz read is five vendor SDK calls per cycle on the
        O12 — about a fifth of a core, permanently, for a signal nobody reads
        between grasps.
        """
        return owner_primitive(self._authority.snapshot().owner_id) == PRIMITIVE_REACTIVE

    def _backend_faulted(self) -> bool:
        return bool(getattr(self.backend, "communication_fault", False))

    def _effective_read_interval(self) -> float:
        """Seconds between SDK joint readbacks under the current conditions."""
        read_interval = self.joint_read_min_interval_s
        if not self._fault_backoff_active:
            return read_interval
        read_interval = max(read_interval, self.fault_poll_interval_s)
        if self._probe_escalated:
            read_interval *= 5.0
        if self.pending_command is not None:
            # The one case where a probe is not pointless traffic during an
            # error storm: an undelivered command is already re-sending at
            # command_retry_period_s, and this readback is the only thing that
            # can confirm it and STOP those re-sends. Probing at the retry
            # cadence adds no meaningful load and keeps the attempt budget
            # bounded at the documented 8 x 0.3 s instead of 8 x 2 s.
            read_interval = min(
                read_interval,
                max(self.joint_read_min_interval_s, self.command_retry_period_s),
            )
        return read_interval

    def shutdown(self) -> None:
        """Stop every thread this node started, in dependency order.

        Producers first, then the worker they submit to: stopping the worker
        while acquisition is still running would leave that thread waiting on
        calls that can no longer be executed. Idempotent, because teardown
        arrives by more than one path.
        """
        self._acquisition_stop.set()
        thread = getattr(self, "_acquisition_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        # Only after its producer is gone. Previously this was never called at
        # all: the worker thread outlived the node, and being a daemon it was
        # invisible until something kept the process alive.
        worker = getattr(self, "_sdk", None)
        if worker is not None:
            worker.shutdown()

    def destroy_node(self) -> bool:
        """Never outlive the node with a thread that still touches it.

        The acquisition thread holds a reference to this node and to its logger.
        Left running past destruction it keeps calling into a dead context, which
        shows up as unrelated teardown errors elsewhere — it surfaced first as
        tests that passed alone and failed together.
        """
        self.shutdown()
        return super().destroy_node()

    def _sdk_read(self, name: str, fn, *, lane: Lane = Lane.ACQUISITION):
        """Run one SDK read on the worker and wait for it, off the executor.

        Blocking here is fine and deliberate: this is the acquisition thread, and
        waiting on the worker is how it stays in step with the hand. What must
        never block is the executor, which is why no SDK call is reachable from a
        subscription, a service handler or the publication timer any more.

        Returns None when the call did not produce a value. A read that timed out
        has an unknown outcome — it may still be in flight — so the caller keeps
        the previous sample rather than inventing one.
        """
        try:
            return self._sdk.call(name, fn, timeout=self._sdk_read_timeout_s, lane=lane)
        except Exception as exc:
            self.get_logger().warn(f"hand SDK read '{name}' did not complete: {exc}")
            return None

    def _acquisition_loop(self) -> None:
        """Pace SDK reads on their own thread, off the ROS executor.

        Monotonic, without burst catch-up: after a slow read the next one is due
        one interval from now, not immediately. A rate loop that repays missed
        ticks turns a hiccup into a burst on the bus.
        """
        name_os_thread(f"hand_{self.hand_side[:4]}_acq")
        next_tick = time.monotonic()
        while not self._acquisition_stop.is_set():
            period = self._effective_read_interval() or 0.05
            next_tick = self._pace(next_tick, period)
            if self._acquisition_stop.is_set():
                break
            try:
                self._acquire_once()
            except Exception as exc:  # never let one bad read end acquisition
                self.get_logger().error(f"hand acquisition cycle failed: {exc}")

    @staticmethod
    def _pace(next_tick: float, period_s: float) -> float:
        now = time.monotonic()
        if next_tick <= now:
            next_tick = now + period_s
        else:
            time.sleep(next_tick - now)
            next_tick += period_s
        return next_tick

    def _acquire_once(self) -> None:
        """One acquisition cycle: reads go through the worker, results to state."""
        now = time.monotonic()
        # Fault backoff: while the backend is faulted, only a slow joint-read
        # probe goes onto the bus; status/tactile reads are skipped entirely and
        # the cached snapshots are republished. A single successful probe clears
        # the backend fault and normal polling resumes on the next tick.
        faulted = self._backend_faulted()
        if faulted:
            self._fault_recovery_streak = 0
            if not self._fault_backoff_active:
                self._fault_backoff_active = True
                if self.fault_poll_interval_s > 0.0:
                    self.get_logger().warn(
                        "OmniHand backend communication fault; backing off SDK polling "
                        f"to one probe every {self.fault_poll_interval_s:.1f} s until "
                        f"{self._fault_recovery_streak_needed} consecutive readbacks succeed"
                    )

        # No gate here any more. The loop is already paced at the read interval,
        # so a tick IS the read. The old gate existed because acquisition rode on
        # the publish timer and had to decide per tick whether it was due; that
        # rounding is what made an intended 20 Hz measure 15.4 Hz.
        positions = self._sdk_read("read_joint_state", self.backend.read_joint_state)
        if positions is not None:
            with self._snapshot_lock:
                self.cached_positions = positions
        self.last_joint_read_monotonic = now
        if positions is not None:
            if not self._backend_faulted():
                self.last_good_joint_read_monotonic = now
            if self._fault_backoff_active:
                if self._backend_faulted():
                    self._failed_probe_streak += 1
                    if (
                        not self._probe_escalated
                        and self._failed_probe_streak
                        >= self._failed_probe_escalation_threshold
                    ):
                        self._probe_escalated = True
                        self.get_logger().error(
                            f"OmniHand unreachable for {self._failed_probe_streak} "
                            "consecutive probes; stretching probe cadence 5x. Check the "
                            "hand (power/connection) — with one-shot off its unacked "
                            "frames keep retransmitting and congest the shared bus TX "
                            "path until an `ip link down/up`"
                        )
                else:
                    # Hysteresis: stay at the slow probe cadence until the bus
                    # has proven itself with consecutive clean readbacks.
                    self._failed_probe_streak = 0
                    self._fault_recovery_streak += 1
                    if self._fault_recovery_streak >= self._fault_recovery_streak_needed:
                        self._fault_backoff_active = False
                        self._fault_recovery_streak = 0
                        self._probe_escalated = False
                        self.get_logger().info(
                            "OmniHand backend recovered "
                            f"({self._fault_recovery_streak_needed} consecutive clean "
                            "readbacks); normal SDK polling resumed"
                        )

        if not self._fault_backoff_active or self._last_status_snapshot is None:
            status = self._sdk_read(
                "read_status", self.backend.read_status, lane=Lane.DIAGNOSTIC
            )
            if status is not None:
                with self._snapshot_lock:
                    self._last_status_snapshot = status

        # Tactile: cadence and lane both depend on who holds the hand. For the
        # reactive primitive this is control-critical acquisition; for anyone
        # else it is a diagnostic nobody is waiting on.
        reactive = self._reactive_owner_holds()
        interval = (
            self.tactile_reactive_interval_s if reactive else self.tactile_read_interval_s
        )
        # A gate at or below the loop period is not a gate, it is a rounding
        # error: ordinary jitter makes cycles miss `now - last >= interval` and
        # the effective rate collapses. Measured 2026-08-15 — a 20 Hz reactive
        # tactile rate on a 20 Hz loop delivered 6.5 Hz. The joint readback lost
        # 20 Hz to 15.4 Hz the same way before the loop was paced instead of
        # gated. So: at or under one period means every cycle, and a slower
        # cadence is compared with half a period of tolerance.
        period = self._effective_read_interval()
        due = interval <= period or (
            now - self._last_tactile_acquire_monotonic >= interval - 0.5 * period
        )
        if due and (not self._fault_backoff_active or self._last_tactile_snapshot is None):
            self._last_tactile_acquire_monotonic = now
            # The backend caches the vendor sample behind the same interval, so
            # raising the cadence here without raising it there would re-serve
            # the stale one and change nothing.
            self.backend.tactile_read_interval_s = interval
            tactile = self._sdk_read(
                "read_tactile",
                self.backend.read_tactile,
                lane=Lane.ACQUISITION if reactive else Lane.DIAGNOSTIC,
            )
            if tactile is not None:
                with self._snapshot_lock:
                    self._last_tactile_snapshot = tactile
                    self._tactile_acquired_monotonic = now

    def _publication_tick(self) -> None:
        """Publish from the acquired snapshot. Reaches no SDK."""
        stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        if not self._tick_thread_named:
            # Python thread names never reach the kernel, so a per-thread CPU
            # census would otherwise show this as another copy of the process
            # name and attribute nothing.
            name_os_thread(f"hand_{self.hand_side[:4]}_pub")
            self._tick_thread_named = True
        if self.metrics.due():
            report = self.metrics.report()
            if report:
                self.get_logger().info(report)

        # Do not publish a fabricated zero/default hand pose before the first
        # successful SDK readback. MoveIt otherwise latches that fake pose as
        # the current hand state and plans from it, which later fails execute
        # validation once the real readback arrives.
        #
        # Beyond that, a joint sample goes out when the hand answered, not when a
        # timer fired. Republishing the cache gave every sample a fresh header
        # stamp while the values were minutes old under a fault.
        with self._snapshot_lock:
            positions = list(self.cached_positions)
            acquired_at = self.last_joint_read_monotonic
            good_at = self.last_good_joint_read_monotonic
        if (
            good_at > 0.0
            and acquired_at > self._published_read_monotonic
            and now - self._last_joint_publish_monotonic >= self._publish_min_interval_s
        ):
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = list(self.joint_names)
            joint_state.position = positions
            self.hand_joint_states_pub.publish(joint_state)
            self._published_read_monotonic = acquired_at
            self._last_joint_publish_monotonic = now

        self._sync_authority("publication tick")
        self._check_owner_liveness()
        self._publish_status_if_due(stamp, now)

        if self._tactile_acquired_monotonic > self._published_tactile_monotonic:
            self._published_tactile_monotonic = self._tactile_acquired_monotonic
            tactile_snapshot = self._last_tactile_snapshot
            tactile_msg = OmniHandTactileRaw()
            tactile_msg.header.stamp = stamp
            tactile_msg.hand_side = self.hand_side
            tactile_msg.backend_name = tactile_snapshot.backend_name
            tactile_msg.layout_name = tactile_snapshot.layout_name
            tactile_msg.values = tactile_snapshot.values
            self.tactile_pub.publish(tactile_msg)
            self._last_tactile_publish_monotonic = now

    def _status_signature(self, snapshot: OmniHandStatusSnapshot) -> tuple:
        """What has to change before a status message is worth sending.

        Deliberately excludes `joint_readback_age_s`: it advances every tick by
        construction, so including it would make every status "changed" and undo
        the gate. The heartbeat is what keeps it current.
        """
        pending = self.pending_command
        return (
            snapshot.control_mode,
            snapshot.connected,
            snapshot.initialized,
            snapshot.communication_fault,
            snapshot.status_text,
            pending is not None,
            int(pending["attempts"]) if pending is not None else 0,
            self._command_delivery_failed,
        )

    def _publish_status_if_due(self, stamp, now: float) -> None:
        snapshot = self._last_status_snapshot
        if snapshot is None:
            return
        signature = self._status_signature(snapshot)
        heartbeat_due = (
            self._status_heartbeat_period_s > 0.0
            and now - self._last_status_publish_monotonic >= self._status_heartbeat_period_s
        )
        if signature == self._last_status_signature and not heartbeat_due:
            return

        status_msg = OmniHandStatus()
        status_msg.header.stamp = stamp
        status_msg.hand_side = self.hand_side
        status_msg.backend_name = snapshot.backend_name
        status_msg.control_mode = snapshot.control_mode
        status_msg.connected = snapshot.connected
        status_msg.initialized = snapshot.initialized
        status_msg.is_mock = snapshot.is_mock
        status_msg.communication_fault = snapshot.communication_fault
        pending = self.pending_command
        status_msg.command_pending = pending is not None
        status_msg.command_delivery_failed = self._command_delivery_failed
        status_msg.command_attempts = min(
            0xFFFF, int(pending["attempts"]) if pending is not None else 0
        )
        status_msg.joint_readback_age_s = (
            float(now - self.last_good_joint_read_monotonic)
            if self.last_good_joint_read_monotonic > 0.0
            else -1.0
        )
        status_msg.active_joint_temperatures_c = snapshot.active_joint_temperatures_c
        status_msg.active_joint_currents_a = snapshot.active_joint_currents_a
        status_msg.active_joint_stalled = snapshot.active_joint_stalled
        status_msg.active_joint_over_temperature = snapshot.active_joint_over_temperature
        status_msg.active_joint_over_current = snapshot.active_joint_over_current
        status_msg.status_text = snapshot.status_text
        if pending is not None:
            status_msg.status_text += (
                f"; command_retry {pending['attempts']}"
                f"/{self.command_retry_max_attempts} pending"
            )
        self.status_pub.publish(status_msg)
        self._last_status_signature = signature
        self._last_status_publish_monotonic = now

    def _publish_command_verdict(self) -> None:
        """Announce a settled command immediately, without waiting for a tick.

        `FollowJointTrajectory` holds its goal — and, on a shared bus, the arm's
        quiesce window — until it sees a status sample published *after* the
        command saying the target is no longer pending. Emitting that here makes
        the verdict reach it in the retry tick that decided it, which is sooner
        than the old fixed cadence delivered it.
        """
        self._publish_status_if_due(self.get_clock().now().to_msg(), time.monotonic())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = OmniHandBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()