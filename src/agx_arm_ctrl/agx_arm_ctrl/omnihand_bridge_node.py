#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
import os
from pathlib import Path
import sys
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
    DeviceAuthority,
    DeviceState,
    UnitSafety,
    UnitSafetySnapshot,
)
from agx_arm_msgs.srv import ClaimDevice
from agx_arm_msgs.msg import (
    AgxDeviceAuthority,
    AgxUnitSafety,
    OmniHandStatus,
    OmniHandTactileRaw,
)

from agx_arm_ctrl.motion_registry import bus_topology, hand_sides
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

    def __init__(self, hand_side: str, device_id: int, canfd_id: int, cfg_path: str, sdk_python_dir: str = "", can_interface: str = "") -> None:
        self.hand_side = hand_side
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
        self._temperature_reports_supported = False
        self._current_reports_supported = False

        sdk_class, finger_enum, hand_type_enum = _load_sdk_symbols(sdk_python_dir)
        hand_type = getattr(hand_type_enum, hand_side.upper())
        self._tactile_finger_entries = [
            (layout_name, getattr(finger_enum, enum_name))
            for layout_name, enum_name in TACTILE_FINGERS
        ]
        self.hand = _create_sdk_hand(
            sdk_class,
            device_id=device_id,
            hand_type=hand_type,
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
        try:
            hold_positions = self._current_active_joint_targets()
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
        if now - self._last_tactile_read_s >= SDK_TACTILE_READ_INTERVAL_S:
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
        self.declare_parameter("pub_rate", 50.0)
        # Hand joint readback is a real CAN request per poll; on the shared
        # arm+hand bus this competes with the 50 Hz MIT command stream, so the
        # SDK poll rate is decoupled from the ROS publish rate. <= 0 polls on
        # every publish tick (legacy behavior).
        self.declare_parameter("joint_read_rate", 20.0)
        self.declare_parameter("tactile_sample_count", 32)
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
        self.hand_model = get_hand_model(str(self.get_parameter("hand_model").value))
        self.backend_type = str(self.get_parameter("backend_type").value)
        self.pub_rate = float(self.get_parameter("pub_rate").value)
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
        self.joint_states_command_topic = str(
            self.get_parameter("joint_states_command_topic").value
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
                )
            else:
                self.backend = SdkOmniHandBackend(
                    hand_side=self.hand_side,
                    device_id=self.device_id,
                    canfd_id=self.canfd_id,
                    cfg_path=self.sdk_cfg_path,
                    sdk_python_dir=self.sdk_python_dir,
                    can_interface=self.can_interface,
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

        self.hand_joint_states_pub = self.create_publisher(
            JointState, "feedback/omnihand/joint_states", 10
        )
        self.status_pub = self.create_publisher(
            OmniHandStatus, "feedback/omnihand/status", 10
        )
        self.tactile_pub = self.create_publisher(
            OmniHandTactileRaw, "feedback/omnihand/tactile_raw", 10
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
        self.create_service(Trigger, "control/omnihand/stop", self._stop_callback)
        self.create_service(ClaimDevice, "claim_device", self._claim_device_callback)
        self.authority_pub = self.create_publisher(
            AgxDeviceAuthority, "feedback/authority",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._authority.set_on_change(self._publish_authority)
        self.create_subscription(
            AgxUnitSafety, "/unit_safety", self._unit_safety_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        timer_period = 1.0 / self.pub_rate if self.pub_rate > 0.0 else 0.02
        self.create_timer(timer_period, self._publish_feedback)
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

    def _submit_command(self, target_map: dict[str, float], control_mode: str) -> None:
        """Send a hand target and keep it pending until the readback confirms it.

        On the shared CAN bus the hand's frames lose arbitration under arm load
        and get dropped silently (one-shot mode), so a single send is unreliable.
        Targets are absolute setpoints, so re-sending the latest one is safe.
        """
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

    def _send_pending_command(self) -> None:
        pending = self.pending_command
        if pending is None:
            return

        pending["attempts"] += 1
        pending["last_send_monotonic"] = time.monotonic()
        try:
            self.backend.apply_joint_targets(pending["targets"], pending["control_mode"])
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

        if self._pending_command_verified(pending):
            if pending["attempts"] > 1:
                self.get_logger().info(
                    f"OmniHand {pending['control_mode']} command verified after "
                    f"{pending['attempts']} attempts"
                )
            self.pending_command = None
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
            return

        if time.monotonic() - pending["last_send_monotonic"] >= self.command_retry_period_s:
            self._send_pending_command()

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
        if self._unit_safety.observe(
            UnitSafetySnapshot(
                epoch=int(msg.epoch),
                stopped=bool(msg.stopped),
                reason=msg.reason,
                writer_id=msg.writer_id,
            )
        ):
            self.get_logger().warn(
                f"unit safety generation {msg.epoch}: stopped={msg.stopped} "
                f"({msg.reason})"
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
            f"{self.device_id} transport "
            f"{'claimed by' if request.claim else 'released by'} "
            f"'{request.owner_id}'"
            if verdict.accepted
            else verdict.detail
        )
        (self.get_logger().info if verdict.accepted else self.get_logger().warn)(
            response.message
        )
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
        self.backend.stop()
        response.success = True
        response.message = f"OmniHand {self.backend.backend_name} stop requested"
        return response

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

    def _publish_feedback(self) -> None:
        stamp = self.get_clock().now().to_msg()

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

        read_interval = self._effective_read_interval()
        if read_interval <= 0.0 or now - self.last_joint_read_monotonic >= read_interval:
            self.cached_positions = self.backend.read_joint_state()
            self.last_joint_read_monotonic = now
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

        # Do not publish a fabricated zero/default hand pose before the first
        # successful SDK readback. MoveIt otherwise latches that fake pose as
        # the current hand state and plans from it, which later fails execute
        # validation once the real readback arrives.
        if self.last_good_joint_read_monotonic > 0.0:
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = list(self.joint_names)
            joint_state.position = list(self.cached_positions)
            self.hand_joint_states_pub.publish(joint_state)

        if not self._fault_backoff_active or self._last_status_snapshot is None:
            self._last_status_snapshot = self.backend.read_status()
        status_snapshot = self._last_status_snapshot
        status_msg = OmniHandStatus()
        status_msg.header.stamp = stamp
        status_msg.hand_side = self.hand_side
        status_msg.backend_name = status_snapshot.backend_name
        status_msg.control_mode = status_snapshot.control_mode
        status_msg.connected = status_snapshot.connected
        status_msg.initialized = status_snapshot.initialized
        status_msg.is_mock = status_snapshot.is_mock
        status_msg.communication_fault = status_snapshot.communication_fault
        pending = self.pending_command
        status_msg.command_pending = pending is not None
        status_msg.command_delivery_failed = self._command_delivery_failed
        status_msg.command_attempts = min(
            0xFFFF, int(pending["attempts"]) if pending is not None else 0
        )
        self._sync_authority("publish tick")
        status_msg.joint_readback_age_s = (
            float(now - self.last_good_joint_read_monotonic)
            if self.last_good_joint_read_monotonic > 0.0
            else -1.0
        )
        status_msg.active_joint_temperatures_c = status_snapshot.active_joint_temperatures_c
        status_msg.active_joint_currents_a = status_snapshot.active_joint_currents_a
        status_msg.active_joint_stalled = status_snapshot.active_joint_stalled
        status_msg.active_joint_over_temperature = status_snapshot.active_joint_over_temperature
        status_msg.active_joint_over_current = status_snapshot.active_joint_over_current
        status_msg.status_text = status_snapshot.status_text
        if self.pending_command is not None:
            status_msg.status_text += (
                f"; command_retry {self.pending_command['attempts']}"
                f"/{self.command_retry_max_attempts} pending"
            )
        self.status_pub.publish(status_msg)

        if not self._fault_backoff_active or self._last_tactile_snapshot is None:
            self._last_tactile_snapshot = self.backend.read_tactile()
        tactile_snapshot = self._last_tactile_snapshot
        tactile_msg = OmniHandTactileRaw()
        tactile_msg.header.stamp = stamp
        tactile_msg.hand_side = self.hand_side
        tactile_msg.backend_name = tactile_snapshot.backend_name
        tactile_msg.layout_name = tactile_snapshot.layout_name
        tactile_msg.values = tactile_snapshot.values
        self.tactile_pub.publish(tactile_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    try:
        node = OmniHandBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()