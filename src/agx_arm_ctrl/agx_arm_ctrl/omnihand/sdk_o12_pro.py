"""OmniHand Pro 2025 (O12) SDK backend.

Drives the real Pro hand via the vendor ``AgibotHandO12`` class from the built
``agibot_hand`` package (``vendor/OmniHand-Pro-2025/build/agibot_hand_pkg``). The
CAN interface is passed to the backend and held in ``OMNIHAND_SOCKETCAN_IFACE``
(our SocketCAN fork patch) only for the SDK construction that reads it — see
``agx_arm_ctrl.omnihand.socketcan_iface``.

Differences from the O10 backend that matter:
- 12 active joints (model registry ``o12_pro``); no 12->10 trimming.
- direct active-angle readback (``get_all_active_joint_angles``); NO motor->angle
  polynomial conversion (that was O10-specific calibration).
- richer tactile payload (online/normal/tangent + channels + capacitive),
  flattened per finger into the existing OmniHandTactileRaw float vector.

This backend exposes the same duck-typed surface the bridge expects from a
backend (get_joint_names / apply_joint_targets / apply_trajectory / stop /
read_joint_state / read_status / read_tactile, plus the snapshot fields).
"""

from __future__ import annotations

from importlib import import_module
import os
from pathlib import Path
import sys
import time
from typing import Any

from trajectory_msgs.msg import JointTrajectory

from agx_arm_ctrl.omnihand.models import HandModel
from agx_arm_ctrl.omnihand.socketcan_iface import socketcan_interface
# Snapshot dataclasses live in the bridge module; importing them here is safe
# because this module is only imported lazily, after omnihand_bridge_node has
# finished loading (see OmniHandBridgeNode.__init__).
from agx_arm_ctrl.omnihand_bridge_node import (
    OmniHandStatusSnapshot,
    OmniHandTactileSnapshot,
)
from agx_arm_ctrl.runtime_metrics import MeasuredSdk, RuntimeMetrics


SDK_STATUS_READ_INTERVAL_S = 1.0
SDK_TACTILE_READ_INTERVAL_S = 1.0

# Built Pro package, relative to the repo root. Its compiled core .so has an
# $ORIGIN RUNPATH and sits next to libomniHandPro25Can.so, so adding this
# directory to sys.path is enough to import the SDK.
_VENDOR_O12_PRO_PKG_REL = Path("vendor") / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"


def _locate_builtin_o12_pro_pkg() -> str | None:
    """Search upward from this file for the repo's built agibot_hand package."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _VENDOR_O12_PRO_PKG_REL
        if (candidate / "agibot_hand" / "__init__.py").exists():
            return str(candidate)
    return None


def _ensure_agibot_hand_importable(sdk_python_dir: str = "") -> None:
    """Make ``agibot_hand`` importable, locating the built package if needed."""
    try:
        import_module("agibot_hand")
        return
    except ImportError:
        pass

    candidates = [
        sdk_python_dir,
        os.environ.get("AGX_ARM_OMNIHAND_PRO_SDK_DIR", ""),
        _locate_builtin_o12_pro_pkg() or "",
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
        "hand_model=o12_pro backend_type=sdk requires the agibot_hand package, "
        "which was not on PYTHONPATH and could not be located automatically. Build "
        "vendor/OmniHand-Pro-2025 (SocketCAN, Python 3.10) or set the bridge "
        "'sdk_python_dir' parameter / AGX_ARM_OMNIHAND_PRO_SDK_DIR to its "
        "build/agibot_hand_pkg directory."
    ) from last_error


def _load_o12_pro_symbols(sdk_python_dir: str = "") -> tuple[type[Any], Any, Any, Any]:
    _ensure_agibot_hand_importable(sdk_python_dir)
    module = import_module("agibot_hand")
    missing = [
        symbol
        for symbol in ("AgibotHandO12", "EFinger", "EHandType", "EControlMode")
        if not hasattr(module, symbol)
    ]
    if missing:
        raise RuntimeError(
            "Installed agibot_hand package is missing required symbols: "
            + ", ".join(missing)
        )
    return module.AgibotHandO12, module.EFinger, module.EHandType, module.EControlMode


class O12ProSdkBackend:
    """Repo-owned backend wrapping the vendor AgibotHandO12 over SocketCAN."""

    def __init__(
        self,
        model: HandModel,
        hand_side: str,
        device_id: int = 1,
        sdk_python_dir: str = "",
        can_interface: str = "",
        metrics: RuntimeMetrics | None = None,
    ) -> None:
        self.model = model
        self._metrics = metrics or RuntimeMetrics(enabled=False)
        self.hand_side = hand_side
        self.device_id = device_id
        self.backend_name = "vendor_sdk_o12_pro"
        self.control_mode = "active_joint_control"
        self.connected = False
        self.initialized = False
        self.is_mock = False
        self.communication_fault = False
        self.status_text = "o12 pro sdk backend initializing"

        self.joint_names = model.build_joint_names(hand_side)
        joint_count = len(self.joint_names)
        self.positions = [0.0] * joint_count
        self.temperatures_c = [0.0] * joint_count
        self.currents_a = [0.0] * joint_count
        self.stalled = [False] * joint_count
        self.over_temperature = [False] * joint_count
        self.over_current = [False] * joint_count
        self.tactile_values: list[float] = []
        self._active_joint_min, self._active_joint_max = model.joint_limits(hand_side)
        self._last_status_read_s = 0.0
        self._last_tactile_read_s = 0.0
        # How often the vendor tactile sample is actually refreshed. A
        # diagnostic cadence by default; the bridge raises it while a
        # reactive owner holds the hand, because contact-seeking motion
        # ends where this sensor says and cannot wait a second to hear it.
        self.tactile_read_interval_s = SDK_TACTILE_READ_INTERVAL_S
        self._extra_fault_text = ""

        sdk_class, finger_enum, hand_type_enum, _control_mode_enum = _load_o12_pro_symbols(
            sdk_python_dir
        )
        hand_type = getattr(hand_type_enum, hand_side.upper())
        self._tactile_finger_entries = [
            (layout_name, getattr(finger_enum, enum_name))
            for layout_name, enum_name in model.tactile_fingers
        ]

        # AgibotHandO12 opens the CAN session in its constructor, and the vendor
        # SocketCAN backend picks its interface from the environment at that
        # moment. Scoped to this call so the interface stays an argument to this
        # backend rather than becoming state of the whole process.
        #
        # Wrapped so every vendor call is counted and timed by name, from the
        # thread that made it. Wrapping the session rather than each call site is
        # what makes the coverage complete: the call nobody thought to measure is
        # the one that turns out to dominate. ~150 % of a core per hand lives
        # behind this object and has never been decomposed.
        with socketcan_interface(can_interface):
            self.hand = MeasuredSdk(
                sdk_class(device_id=device_id, hand_type=hand_type),
                self._metrics,
            )
        if hasattr(self.hand, "show_data_details"):
            self.hand.show_data_details(False)

        self.connected = True
        self.initialized = True
        self.status_text = (
            f"o12 pro sdk backend ready (can_interface={can_interface or 'can_nero_right(default)'}, "
            f"device_id={device_id})"
        )
        try:
            self.positions = self.read_joint_state()
            self.control_mode = "active_joint_hold"
        except Exception:
            # Keep startup tolerant; the periodic feedback timer keeps retrying.
            pass

    # --- fault helpers -------------------------------------------------------
    def _set_fault(self, message: str, exc: Exception) -> None:
        self.communication_fault = True
        self.connected = False
        self.status_text = f"{message}: {exc}"

    def _clear_fault(self, message: str) -> None:
        self.communication_fault = False
        self.connected = True
        self.status_text = message

    def _clamp(self, values: list[float]) -> list[float]:
        return [
            min(max(self._active_joint_min[index], float(value)), self._active_joint_max[index])
            for index, value in enumerate(values)
        ]

    def _current_active_joint_targets(self) -> list[float]:
        if any(self.positions):
            return list(self.positions)
        return self.read_joint_state()

    # --- backend surface -----------------------------------------------------
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
        target_positions = self._clamp(target_positions)

        try:
            self.hand.set_all_active_joint_angles(target_positions)
        except Exception as exc:
            self._set_fault(f"o12 pro {control_mode} command failed", exc)
            raise RuntimeError("o12 pro backend rejected active joint command") from exc

        self.positions = list(target_positions)
        self.control_mode = control_mode
        self._clear_fault(
            f"applied o12 pro {control_mode} command with {matched_joint_count} active joints"
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
            self._clear_fault("o12 pro stop requested; holding current active joint pose")
        except Exception as exc:
            self._set_fault("o12 pro stop request failed", exc)

    def read_joint_state(self) -> list[float]:
        try:
            raw = list(self.hand.get_all_active_joint_angles())
            if len(raw) != len(self.joint_names):
                raise RuntimeError(
                    "active joint angles length mismatch: expected "
                    f"{len(self.joint_names)}, got {len(raw)}"
                )
            self.positions = self._clamp([float(value) for value in raw])
            self._clear_fault("o12 pro readback active")
        except Exception as exc:
            self._set_fault("o12 pro joint readback failed", exc)
        return list(self.positions)

    def read_status(self) -> OmniHandStatusSnapshot:
        now = time.monotonic()
        if now - self._last_status_read_s >= SDK_STATUS_READ_INTERVAL_S:
            try:
                reports = list(self.hand.get_all_error_reports())
                self.stalled = [bool(getattr(r, "stalled", False)) for r in reports]
                self.over_temperature = [bool(getattr(r, "overheat", False)) for r in reports]
                self.over_current = [bool(getattr(r, "over_current", False)) for r in reports]
                motor_except = [bool(getattr(r, "motor_except", False)) for r in reports]
                commu_except = [bool(getattr(r, "commu_except", False)) for r in reports]

                try:
                    self.temperatures_c = [float(t) for t in self.hand.get_all_temperature_reports()]
                except Exception:
                    pass
                try:
                    self.currents_a = [float(c) for c in self.hand.get_all_current_reports()]
                except Exception:
                    pass

                # OmniHandStatus has no motor_except/commu_except fields; surface
                # them in status_text for now (proposal §5.7).
                extras: list[str] = []
                if any(motor_except):
                    extras.append(
                        "motor_except@" + str([i for i, v in enumerate(motor_except) if v])
                    )
                if any(commu_except):
                    extras.append(
                        "commu_except@" + str([i for i, v in enumerate(commu_except) if v])
                    )
                self._extra_fault_text = ("; " + ", ".join(extras)) if extras else ""

                self._last_status_read_s = now
                self._clear_fault("o12 pro status readback active")
            except Exception as exc:
                self._last_status_read_s = now
                self._set_fault("o12 pro status readback failed", exc)

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
            status_text=self.status_text + self._extra_fault_text,
        )

    def read_tactile(self) -> OmniHandTactileSnapshot:
        now = time.monotonic()
        if now - self._last_tactile_read_s >= self.tactile_read_interval_s:
            tactile_values: list[float] = []
            supported_finger_entries: list[tuple[str, Any]] = []
            for layout_name, finger_value in self._tactile_finger_entries:
                try:
                    data = self.hand.get_tactile_sensor_data(finger_value)
                    row = [
                        float(data.online_state),
                        float(data.normal_force),
                        float(data.tangent_force),
                        float(data.tangent_force_angle),
                    ]
                    row.extend(float(value) for value in data.channel_values)
                    row.extend(float(value) for value in data.capacitive_approach)
                except Exception:
                    continue
                tactile_values.extend(row)
                supported_finger_entries.append((layout_name, finger_value))

            if supported_finger_entries:
                self._tactile_finger_entries = supported_finger_entries
                self.tactile_values = tactile_values
            self._last_tactile_read_s = now

        finger_names = ",".join(name for name, _ in self._tactile_finger_entries)
        layout_name = (
            f"o12_pro:v1:{finger_names}:online,normal,tangent,tangent_angle,channels,capacitive"
        )
        return OmniHandTactileSnapshot(
            backend_name=self.backend_name,
            layout_name=layout_name,
            values=list(self.tactile_values),
        )
