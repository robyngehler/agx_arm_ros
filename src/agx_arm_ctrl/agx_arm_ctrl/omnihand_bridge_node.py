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
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
import yaml

from agx_arm_msgs.msg import OmniHandStatus, OmniHandTactileRaw

from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, get_hand_model


# Side -> native SocketCAN interface. The authoritative mapping lives in
# config/omnihand_can_interfaces.yaml (installed to the package share); this dict
# is only a last-resort fallback if that file cannot be read. Keep both in sync.
CAN_INTERFACE_CONFIG = "omnihand_can_interfaces.yaml"
FALLBACK_CAN_INTERFACES = {"right": "can_nero_right", "left": "can_nero_left"}


def resolve_can_interface(hand_side: str) -> tuple[str, str]:
    """Return (interface_name, source) for the side, preferring the config file."""
    try:
        config_path = (
            Path(get_package_share_directory("agx_arm_ctrl"))
            / "config"
            / CAN_INTERFACE_CONFIG
        )
        data = yaml.safe_load(config_path.read_text()) or {}
        mapping = data.get("omnihand_can_interfaces", {})
        interface = str(mapping.get(hand_side, "")).strip()
        if interface:
            return interface, str(config_path)
    except Exception:
        pass
    return FALLBACK_CAN_INTERFACES.get(hand_side, ""), "built-in fallback"


JOINT_SUFFIXES = [
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "index_abad_joint",
    "index_pip_joint",
    "middle_pip_joint",
    "ring_abad_joint",
    "ring_pip_joint",
    "pinky_abad_joint",
    "pinky_pip_joint",
]

TACTILE_FINGERS = [
    ("thumb_tip", "THUMB"),
    ("index_tip", "INDEX"),
    ("middle_tip", "MIDDLE"),
    ("ring_tip", "RING"),
    ("little_tip", "LITTLE"),
]

SDK_TACTILE_LAYOUT_NAME = ",".join(name for name, _ in TACTILE_FINGERS)

SDK_ACTIVE_JOINT_COUNT = len(JOINT_SUFFIXES)
SDK_PADDED_VECTOR_COUNT = SDK_ACTIVE_JOINT_COUNT + 2
SDK_LEFT_POS_DIRECTION = [-1, -1, -1, -1, 1, 1, -1, 1, -1, 1]
SDK_ACTIVE_JOINT_MAX_RIGHT = [1.12, 0.05, 0.8416, 0.0, 1.48, 1.48, 0.17, 1.48, 0.19, 1.48]
SDK_ACTIVE_JOINT_MIN_RIGHT = [-0.03, -1.64, 0.0, -0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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


def build_joint_names(hand_side: str) -> list[str]:
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


def load_gesture_presets() -> dict[str, list[float]]:
    """Return named OmniHand active-joint presets from the package config.

    config/omnihand_gestures.yaml is the single source of truth; callers (the
    exerciser, future skill controllers) read from here instead of carrying their
    own copies. Every preset is validated to carry exactly len(JOINT_SUFFIXES)
    values, ordered to match JOINT_SUFFIXES. Falls back to a small built-in set
    only if the config file cannot be read.
    """
    try:
        config_path = (
            Path(get_package_share_directory("agx_arm_ctrl"))
            / "config"
            / GESTURE_CONFIG
        )
        data = yaml.safe_load(config_path.read_text()) or {}
    except Exception:
        return {name: list(values) for name, values in FALLBACK_GESTURE_PRESETS.items()}

    declared_order = data.get("omnihand_active_joint_order")
    if declared_order is not None and list(declared_order) != JOINT_SUFFIXES:
        raise RuntimeError(
            f"omnihand_active_joint_order in {GESTURE_CONFIG} does not match "
            "JOINT_SUFFIXES; gesture vectors would be misordered"
        )

    raw_gestures = data.get("omnihand_gestures") or {}
    presets: dict[str, list[float]] = {}
    for name, values in raw_gestures.items():
        vector = [float(value) for value in values]
        if len(vector) != len(JOINT_SUFFIXES):
            raise RuntimeError(
                f"gesture '{name}' in {GESTURE_CONFIG} has {len(vector)} values, "
                f"expected {len(JOINT_SUFFIXES)}"
            )
        presets[str(name)] = vector

    if not presets:
        return {name: list(values) for name, values in FALLBACK_GESTURE_PRESETS.items()}
    return presets


def mirror_active_joint_vector(values: list[float]) -> list[float]:
    """Mirror a right-hand active-joint vector into the left-hand convention.

    The vendor presets are calibrated for the right hand (every value fits the
    right-hand limits and is out of range for the left). The left hand uses the
    mirrored sign convention captured by SDK_LEFT_POS_DIRECTION, the same
    direction vector _mirror_joint_limits uses for the joint limits, so a
    component-wise multiply maps a right-hand pose to the matching left-hand pose.
    """
    return [
        direction * float(value)
        for direction, value in zip(SDK_LEFT_POS_DIRECTION, values, strict=True)
    ]


def resolve_gesture_presets(hand_side: str) -> dict[str, list[float]]:
    """Return the named presets in the convention of the selected hand side.

    The config file is the single source of truth and stores the canonical
    right-hand vectors; the left hand is derived by mirroring so there is no
    second copy to keep in sync. The bridge still clamps every target to the
    side's joint limits as a final safety net.
    """
    presets = load_gesture_presets()
    if hand_side == "left":
        return {name: mirror_active_joint_vector(vec) for name, vec in presets.items()}
    return {name: list(vec) for name, vec in presets.items()}


# Built vendor package, relative to the repo root. It carries the compiled
# omnihand_2025_core .so, whose RUNPATH is $ORIGIN, so no LD_LIBRARY_PATH is
# needed — getting this directory onto sys.path is sufficient to import the SDK.
_VENDOR_PKG_REL = Path("vendor") / "Omnihand-2025-SDK" / "build_phase1_socket" / "omnihand_2025_pkg"


def _locate_builtin_vendor_pkg() -> str | None:
    """Search upward from this file for the repo's built omnihand_2025 package."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _VENDOR_PKG_REL
        if (candidate / "omnihand_2025" / "__init__.py").exists():
            return str(candidate)
    return None


def _ensure_omnihand_importable(sdk_python_dir: str = "") -> None:
    """Make omnihand_2025 importable, locating the built package if needed.

    Tries the ambient environment first (an already-set PYTHONPATH wins), then
    an explicit dir, the AGX_ARM_OMNIHAND_SDK_DIR env var, and finally an upward
    search for the repo's built package. This lets `ros2 launch ... backend_type:=sdk`
    work without manually exporting PYTHONPATH/LD_LIBRARY_PATH.
    """
    try:
        import_module("omnihand_2025")
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
        sys.modules.pop("omnihand_2025", None)
        try:
            import_module("omnihand_2025")
            return
        except ImportError as exc:
            last_error = exc

    raise RuntimeError(
        "backend_type=sdk requires the omnihand_2025 vendor package, which was not "
        "on PYTHONPATH and could not be located automatically. Set the bridge "
        "'sdk_python_dir' parameter or the AGX_ARM_OMNIHAND_SDK_DIR env var to the "
        "built package (vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg)."
    ) from last_error


def _load_sdk_symbols(sdk_python_dir: str = "") -> tuple[type[Any], Any, Any]:
    _ensure_omnihand_importable(sdk_python_dir)
    try:
        module = import_module("omnihand_2025")
    except ImportError as exc:
        raise RuntimeError(
            "backend_type=sdk requires omnihand_2025 on PYTHONPATH and the vendor library path to be set"
        ) from exc

    missing = [
        symbol_name
        for symbol_name in ("AgibotHandO10", "EFinger", "EHandType")
        if not hasattr(module, symbol_name)
    ]
    if missing:
        raise RuntimeError(
            "Installed omnihand_2025 package is missing required symbols: "
            + ", ".join(missing)
        )

    return module.AgibotHandO10, module.EFinger, module.EHandType


def _create_sdk_hand(
    sdk_class: type[Any],
    *,
    device_id: int,
    canfd_id: int,
    hand_type: Any,
    cfg_path: str,
) -> Any:
    attempts: list[dict[str, Any]] = []
    if cfg_path:
        attempts.append(
            {
                "device_id": device_id,
                "canfd_id": canfd_id,
                "hand_type": hand_type,
                "cfg_path": cfg_path,
            }
        )
    attempts.append(
        {
            "device_id": device_id,
            "canfd_id": canfd_id,
            "hand_type": hand_type,
        }
    )
    if cfg_path:
        attempts.append(
            {
                "device_id": device_id,
                "hand_type": hand_type,
                "cfg_path": cfg_path,
            }
        )
    attempts.append(
        {
            "device_id": device_id,
            "hand_type": hand_type,
        }
    )

    last_error: TypeError | None = None
    for kwargs in attempts:
        try:
            return sdk_class.create_hand(**kwargs)
        except TypeError as exc:
            last_error = exc

    raise RuntimeError(
        "Unsupported omnihand_2025 create_hand signature for the current vendor package"
    ) from last_error


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
        # The vendor SocketCAN backend reads the interface ONLY from this env var
        # (default "can0"). Export it before create_hand so the hand opens on the
        # native side bus (e.g. can_nero_right) instead of can0.
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
            canfd_id=canfd_id,
            hand_type=hand_type,
            cfg_path=cfg_path,
        )
        if hasattr(self.hand, "show_data_details"):
            self.hand.show_data_details(False)

        self.connected = True
        self.initialized = True
        self.status_text = (
            f"sdk backend ready (active joint control, can_interface={can_interface or 'can0(default)'}, "
            f"device_id={device_id}, canfd_id={canfd_id})"
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
        self.declare_parameter("tactile_sample_count", 32)
        self.declare_parameter("joint_states_command_topic", "control/joint_states")
        self.declare_parameter("device_id", 1)
        self.declare_parameter("canfd_id", 0)
        self.declare_parameter("sdk_cfg_path", "")
        self.declare_parameter("sdk_python_dir", "")
        self.declare_parameter("can_interface", "")

        self.hand_side = str(self.get_parameter("omnihand_type").value)
        self.hand_model = get_hand_model(str(self.get_parameter("hand_model").value))
        self.backend_type = str(self.get_parameter("backend_type").value)
        self.pub_rate = float(self.get_parameter("pub_rate").value)
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
        # side -> interface mapping from config/omnihand_can_interfaces.yaml.
        self.can_interface = str(self.get_parameter("can_interface").value).strip()
        interface_source = "can_interface parameter"
        if not self.can_interface:
            self.can_interface, interface_source = resolve_can_interface(self.hand_side)

        if self.backend_type == "sdk":
            if not self.can_interface:
                raise ValueError(
                    "backend_type=sdk needs a SocketCAN interface. Set 'can_interface' "
                    f"or add '{self.hand_side}' to config/{CAN_INTERFACE_CONFIG}."
                )
            self.get_logger().info(
                f"OmniHand SocketCAN interface: {self.can_interface} (from {interface_source})"
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

        timer_period = 1.0 / self.pub_rate if self.pub_rate > 0.0 else 0.02
        self.create_timer(timer_period, self._publish_feedback)

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
            if index < len(msg.position)
        }
        if not target_map:
            return

        try:
            self.backend.apply_joint_targets(target_map, "joint_state")
        except ValueError:
            # Shared control/joint_states frequently contains arm-only updates.
            return
        except RuntimeError as exc:
            self.get_logger().warn(f"Ignoring OmniHand joint_state command: {exc}")
            return

    def _joint_trajectory_callback(self, msg: JointTrajectory) -> None:
        try:
            self.backend.apply_trajectory(msg)
        except ValueError as exc:
            self.get_logger().warn(f"Rejected OmniHand JointTrajectory: {exc}")
            return
        except RuntimeError as exc:
            self.get_logger().warn(f"Ignoring OmniHand joint_trajectory command: {exc}")
            return

        unknown_names = [name for name in msg.joint_names if name not in self.joint_names]
        if unknown_names:
            self.get_logger().warn(
                f"Ignored unknown OmniHand joints in trajectory: {', '.join(unknown_names)}"
            )

    def _stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.backend.stop()
        response.success = True
        response.message = f"OmniHand {self.backend.backend_name} stop requested"
        return response

    def _publish_feedback(self) -> None:
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self.joint_names)
        joint_state.position = self.backend.read_joint_state()
        self.hand_joint_states_pub.publish(joint_state)

        status_snapshot = self.backend.read_status()
        status_msg = OmniHandStatus()
        status_msg.header.stamp = stamp
        status_msg.hand_side = self.hand_side
        status_msg.backend_name = status_snapshot.backend_name
        status_msg.control_mode = status_snapshot.control_mode
        status_msg.connected = status_snapshot.connected
        status_msg.initialized = status_snapshot.initialized
        status_msg.is_mock = status_snapshot.is_mock
        status_msg.communication_fault = status_snapshot.communication_fault
        status_msg.active_joint_temperatures_c = status_snapshot.active_joint_temperatures_c
        status_msg.active_joint_currents_a = status_snapshot.active_joint_currents_a
        status_msg.active_joint_stalled = status_snapshot.active_joint_stalled
        status_msg.active_joint_over_temperature = status_snapshot.active_joint_over_temperature
        status_msg.active_joint_over_current = status_snapshot.active_joint_over_current
        status_msg.status_text = status_snapshot.status_text
        self.status_pub.publish(status_msg)

        tactile_snapshot = self.backend.read_tactile()
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