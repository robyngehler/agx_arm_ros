#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any


VENDOR_PYTHON_DIR = (
    Path(__file__).resolve().parents[2] / "vendor" / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"
)

DECLARED_JOINTS: dict[str, list[dict[str, float | int | str]]] = {
    "right": [
        {"index": 1, "name": "R_thumb_roll_joint", "min_rad": -0.17453292519943295, "max_rad": 0.8726646259971648, "min_deg": -10.0, "max_deg": 50.0, "velocity_limit_rad_s": 0.164},
        {"index": 2, "name": "R_thumb_abad_joint", "min_rad": -1.7453292519943295, "max_rad": 0.0, "min_deg": -100.0, "max_deg": 0.0, "velocity_limit_rad_s": 0.164},
        {"index": 3, "name": "R_thumb_mcp_joint", "min_rad": 0.0, "max_rad": 0.8552113334772214, "min_deg": 0.0, "max_deg": 49.0, "velocity_limit_rad_s": 0.308},
        {"index": 4, "name": "R_index_abad_joint", "min_rad": -0.20943951023931953, "max_rad": 0.0, "min_deg": -12.0, "max_deg": 0.0, "velocity_limit_rad_s": 0.164},
        {"index": 5, "name": "R_index_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 6, "name": "R_middle_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 7, "name": "R_ring_abad_joint", "min_rad": 0.0, "max_rad": 0.17453292519943295, "min_deg": 0.0, "max_deg": 10.0, "velocity_limit_rad_s": 0.164},
        {"index": 8, "name": "R_ring_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 9, "name": "R_pinky_abad_joint", "min_rad": 0.0, "max_rad": 0.17453292519943295, "min_deg": 0.0, "max_deg": 10.0, "velocity_limit_rad_s": 0.164},
        {"index": 10, "name": "R_pinky_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
    ],
    "left": [
        {"index": 1, "name": "L_thumb_roll_joint", "min_rad": -0.8726646259971648, "max_rad": 0.17453292519943295, "min_deg": -50.0, "max_deg": 10.0, "velocity_limit_rad_s": 0.164},
        {"index": 2, "name": "L_thumb_abad_joint", "min_rad": 0.0, "max_rad": 1.7453292519943295, "min_deg": 0.0, "max_deg": 100.0, "velocity_limit_rad_s": 0.164},
        {"index": 3, "name": "L_thumb_mcp_joint", "min_rad": -0.8552113334772214, "max_rad": 0.0, "min_deg": -49.0, "max_deg": 0.0, "velocity_limit_rad_s": 0.308},
        {"index": 4, "name": "L_index_abad_joint", "min_rad": 0.0, "max_rad": 0.20943951023931953, "min_deg": 0.0, "max_deg": 12.0, "velocity_limit_rad_s": 0.164},
        {"index": 5, "name": "L_index_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 6, "name": "L_middle_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 7, "name": "L_ring_abad_joint", "min_rad": -0.17453292519943295, "max_rad": 0.0, "min_deg": -10.0, "max_deg": 0.0, "velocity_limit_rad_s": 0.164},
        {"index": 8, "name": "L_ring_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
        {"index": 9, "name": "L_pinky_abad_joint", "min_rad": -0.17453292519943295, "max_rad": 0.0, "min_deg": -10.0, "max_deg": 0.0, "velocity_limit_rad_s": 0.164},
        {"index": 10, "name": "L_pinky_pip_joint", "min_rad": 0.0, "max_rad": 1.5707963267948966, "min_deg": 0.0, "max_deg": 90.0, "velocity_limit_rad_s": 0.308},
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 1 OmniHand smoke test using the vendored Python SDK."
    )
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument(
        "--canfd-id",
        type=int,
        default=0,
        help="CANFD adapter index passed to the current vendor Python binding.",
    )
    parser.add_argument(
        "--hand-type",
        choices=("left", "right"),
        default="left",
        help="Select the hand variant declared in the vendor SDK.",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="Enable one small command-and-restore step. Disabled by default for safety.",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        default=5,
        help="Joint index used for the optional command-and-restore step.",
    )
    parser.add_argument(
        "--delta-rad",
        type=float,
        default=0.05,
        help="Requested angle delta in radians for the optional command step.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.75,
        help="Wait time after each command during the optional motion step.",
    )
    parser.add_argument(
        "--skip-tactile",
        action="store_true",
        help="Skip tactile reads if they are not needed for the current probe.",
    )
    parser.add_argument(
        "--sdk-python-dir",
        type=Path,
        default=VENDOR_PYTHON_DIR,
        help="Python package root to probe. Point this at the built agibot_hand_pkg directory.",
    )
    parser.add_argument(
        "--print-declared-joint-map",
        action="store_true",
        help="Print the repo-side vendor-declared joint map before any runtime probe.",
    )
    parser.add_argument(
        "--json-output",
        type=Path,
        help="Write the structured result to a JSON file.",
    )
    parser.add_argument(
        "--child-runtime-probe",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--child-result-json",
        type=Path,
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def print_declared_joint_map(hand_type: str) -> None:
    joints = DECLARED_JOINTS[hand_type]
    print(f"Declared {hand_type} hand active-joint map:")
    for joint in joints:
        print(
            f"  {joint['index']:>2}: {joint['name']}  "
            f"[{joint['min_rad']:.6f}, {joint['max_rad']:.6f}] rad"
        )


def to_jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(item) for item in value]
    if isinstance(value, SimpleNamespace):
        return to_jsonable(vars(value))

    public_attrs: dict[str, Any] = {}
    for attr_name in dir(value):
        if attr_name.startswith("_"):
            continue
        try:
            attr_value = getattr(value, attr_name)
        except Exception:  # noqa: BLE001
            continue
        if callable(attr_value):
            continue
        public_attrs[attr_name] = to_jsonable(attr_value)
    if public_attrs:
        return public_attrs

    return str(value)


def json_dumps(data: dict[str, Any]) -> str:
    return json.dumps(to_jsonable(data), indent=2, sort_keys=True)


def write_json_output(output_path: Path, data: dict[str, Any]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json_dumps(data) + "\n")


def safe_call(target: Any, method_name: str, *args: Any) -> dict[str, Any]:
    method = getattr(target, method_name)
    try:
        return {"ok": True, "value": method(*args)}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


def serialize_error_reports(reports: list[Any]) -> list[dict[str, bool]]:
    serialized: list[dict[str, bool]] = []
    for report in reports:
        serialized.append(
            {
                "stalled": bool(getattr(report, "stalled", False)),
                "overheat": bool(getattr(report, "overheat", False)),
                "over_current": bool(getattr(report, "over_current", False)),
                "motor_except": bool(getattr(report, "motor_except", False)),
                "commu_except": bool(getattr(report, "commu_except", False)),
            }
        )
    return serialized


def has_fault(error_reports: list[dict[str, bool]]) -> bool:
    return any(any(report.values()) for report in error_reports)


def has_complete_joint_result(call_result: dict[str, Any], expected_size: int = 10) -> bool:
    if not call_result.get("ok"):
        return False
    value = call_result.get("value")
    return isinstance(value, list) and len(value) == expected_size


def compute_target_angles(
    hand_type: str,
    current_angles: list[float],
    joint_index: int,
    delta_rad: float,
) -> list[float]:
    if len(current_angles) != 10:
        raise ValueError(f"Expected 10 active-joint angles, got {len(current_angles)}")
    if not 1 <= joint_index <= 10:
        raise ValueError(f"Joint index must be in [1, 10], got {joint_index}")

    target_angles = list(current_angles)
    joint_spec = DECLARED_JOINTS[hand_type][joint_index - 1]
    min_rad = float(joint_spec["min_rad"])
    max_rad = float(joint_spec["max_rad"])
    desired = current_angles[joint_index - 1] + delta_rad
    target_angles[joint_index - 1] = min(max(desired, min_rad), max_rad)
    return target_angles


def load_vendor_sdk(sdk_python_dir: Path) -> tuple[Any | None, str | None, str | None]:
    sys.path.insert(0, str(sdk_python_dir))
    try:
        import agibot_hand  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}", None

    if hasattr(agibot_hand, "AgibotHandO12"):
        return agibot_hand, None, "built_package"

    return None, "agibot_hand imported but AgibotHandO12 symbol is missing", None


def collect_runtime_data(args: argparse.Namespace, agibot_hand: Any, hand: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
    baseline_angles: list[float] | None = None

    result["vendor_info"] = safe_call(hand, "get_vendor_info")
    result["device_info"] = safe_call(hand, "get_device_info")
    result["active_joint_angles"] = safe_call(hand, "get_all_active_joint_angles")
    result["all_joint_angles"] = safe_call(hand, "get_all_joint_angles")
    result["control_modes"] = safe_call(hand, "get_all_control_modes")
    result["temperature_reports"] = safe_call(hand, "get_all_temperature_reports")
    result["current_reports"] = safe_call(hand, "get_all_current_reports")

    error_reports_result = safe_call(hand, "get_all_error_reports")
    if error_reports_result["ok"]:
        serialized_errors = serialize_error_reports(error_reports_result["value"])
        result["error_reports"] = {"ok": True, "value": serialized_errors}
        result["fault_present"] = has_fault(serialized_errors)
    else:
        result["error_reports"] = error_reports_result
        result["fault_present"] = None

    tactile_summary: dict[str, Any] = {}
    if not args.skip_tactile:
        for finger_name in ("THUMB", "INDEX", "MIDDLE", "RING", "LITTLE"):
            finger_enum = getattr(agibot_hand.EFinger, finger_name)
            tactile_result = safe_call(hand, "get_tactile_sensor_data", finger_enum)
            if tactile_result["ok"]:
                tactile_summary[finger_name.lower()] = {
                    "sample_count": len(tactile_result["value"]),
                }
            else:
                tactile_summary[finger_name.lower()] = tactile_result
    result["tactile_summary"] = tactile_summary

    if result["active_joint_angles"]["ok"]:
        baseline_angles = list(result["active_joint_angles"]["value"])

    if args.allow_motion:
        if baseline_angles is None:
            raise RuntimeError(
                "Cannot run the motion step without a successful active-joint readback."
            )

        target_angles = compute_target_angles(
            args.hand_type,
            baseline_angles,
            args.joint_index,
            args.delta_rad,
        )
        result["motion_step"] = {
            "joint_index": args.joint_index,
            "requested_delta_rad": args.delta_rad,
            "target_angles": target_angles,
        }
        hand.set_all_active_joint_angles(target_angles)
        time.sleep(args.settle_s)
        result["motion_step"]["after_command"] = safe_call(
            hand, "get_all_active_joint_angles"
        )
        hand.set_all_active_joint_angles(baseline_angles)
        time.sleep(args.settle_s)
        result["motion_step"]["after_restore"] = safe_call(
            hand, "get_all_active_joint_angles"
        )

    result["shutdown_note"] = (
        "The vendor Python API does not expose an explicit close method. "
        "This script restores the baseline angles when motion is enabled, then exits."
    )

    if not has_complete_joint_result(result["active_joint_angles"]):
        result["status"] = "runtime_probe_incomplete"
        result["notes"] = [
            "Transport initialized, but the SDK did not return a complete 10-joint active-angle vector.",
            "Treat this as a live runtime or device-path failure rather than a successful enumeration.",
        ]
        return result

    if not has_complete_joint_result(result["all_joint_angles"], expected_size=16):
        result["status"] = "runtime_probe_incomplete"
        result["notes"] = [
            "The SDK did not return a complete full-joint vector after the initial transport setup.",
            "Do not treat this run as a safe command-response validation.",
        ]
        return result

    result["status"] = "runtime_probe_completed"
    return result


def run_child_runtime_probe(args: argparse.Namespace) -> int:
    result: dict[str, Any] = {}
    agibot_hand, import_error, sdk_layout = load_vendor_sdk(args.sdk_python_dir)
    result["sdk_layout"] = sdk_layout
    if import_error is not None:
        result["status"] = "blocked_before_runtime"
        result["import_error"] = import_error
        if args.child_result_json is not None:
            write_json_output(args.child_result_json, result)
        return 3

    hand_type_enum = (
        agibot_hand.EHandType.LEFT
        if args.hand_type == "left"
        else agibot_hand.EHandType.RIGHT
    )
    hand = None
    try:
        hand = agibot_hand.AgibotHandO12(
            device_id=args.device_id,
            hand_type=hand_type_enum,
        )
        result.update(collect_runtime_data(args, agibot_hand, hand))
    except Exception as exc:  # noqa: BLE001
        result["status"] = "runtime_probe_failed"
        result["runtime_error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if args.child_result_json is not None:
            write_json_output(args.child_result_json, result)
        if hand is not None:
            del hand

    return 0 if result.get("status") == "runtime_probe_completed" else 4


def run_child_runtime_probe_subprocess(args: argparse.Namespace) -> tuple[dict[str, Any] | None, int]:
    with tempfile.NamedTemporaryFile(prefix="omnihand_phase1_", suffix=".json", delete=False) as handle:
        child_result_path = Path(handle.name)

    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--child-runtime-probe",
        "--child-result-json",
        str(child_result_path),
        "--device-id",
        str(args.device_id),
        "--canfd-id",
        str(args.canfd_id),
        "--hand-type",
        args.hand_type,
        "--sdk-python-dir",
        str(args.sdk_python_dir),
        "--joint-index",
        str(args.joint_index),
        "--delta-rad",
        str(args.delta_rad),
        "--settle-s",
        str(args.settle_s),
    ]
    if args.skip_tactile:
        command.append("--skip-tactile")
    if args.allow_motion:
        command.append("--allow-motion")

    completed = subprocess.run(command, check=False)
    child_result: dict[str, Any] | None = None
    if child_result_path.exists() and child_result_path.stat().st_size > 0:
        child_result = json.loads(child_result_path.read_text())
    child_result_path.unlink(missing_ok=True)
    return child_result, completed.returncode


def main() -> int:
    args = parse_args()
    if args.child_runtime_probe:
        return run_child_runtime_probe(args)

    result: dict[str, Any] = {
        "phase": 1,
        "hand_type": args.hand_type,
        "device_id": args.device_id,
        "canfd_id": args.canfd_id,
        "platform": {
            "machine": platform.machine(),
            "system": platform.system(),
            "python_version": sys.version.split()[0],
        },
        "vendor_python_dir": str(args.sdk_python_dir),
        "declared_joint_map": DECLARED_JOINTS[args.hand_type],
    }

    if args.print_declared_joint_map:
        print_declared_joint_map(args.hand_type)

    agibot_hand, import_error, sdk_layout = load_vendor_sdk(args.sdk_python_dir)
    result["sdk_layout"] = sdk_layout
    if import_error is not None:
        result["status"] = "blocked_before_runtime"
        result["import_error"] = import_error
        result["notes"] = [
            "The vendor Python package could not be imported on this host.",
            "Check whether the native module has been built and whether the vendored CAN userspace library matches the host architecture.",
        ]
        print("OmniHand Phase 1 probe blocked before runtime:")
        print(f"  host architecture: {result['platform']['machine']}")
        print(f"  import error: {import_error}")
        if args.json_output is not None:
            write_json_output(args.json_output, result)
        return 3

    hand_type_enum = (
        agibot_hand.EHandType.LEFT
        if args.hand_type == "left"
        else agibot_hand.EHandType.RIGHT
    )

    hand = None
    try:
        hand = agibot_hand.AgibotHandO12(
            device_id=args.device_id,
            hand_type=hand_type_enum,
        )
        result["status"] = "runtime_probe_started"
        init_result = safe_call(hand, "is_initialized")
        result["is_initialized"] = init_result
        if init_result["ok"] and not init_result["value"]:
            result["status"] = "runtime_transport_unavailable"
            result["notes"] = [
                "The SDK object was created, but the transport backend did not initialize successfully.",
                "Do not issue vendor-info or motion calls until the transport path and hardware interface are confirmed.",
            ]
            print(json.dumps(result, indent=2, sort_keys=True))
            return 5
        del hand
        hand = None
        child_result, child_return_code = run_child_runtime_probe_subprocess(args)
        result["child_return_code"] = child_return_code
        if child_result is None:
            result["status"] = "runtime_probe_crashed"
            result["notes"] = [
                "The vendor SDK crashed during the runtime RPC phase.",
                "This usually means the transport came up far enough to issue requests, but the device path is still not safe to probe directly from the parent process.",
            ]
            print(json.dumps(result, indent=2, sort_keys=True))
            return 6

        result.update(child_result)
        if child_return_code != 0 and result.get("status") == "runtime_probe_completed":
            result["status"] = "runtime_probe_failed"
            result["notes"] = [
                "The child runtime probe exited with a non-zero status despite reporting completion.",
                "Treat this as a probe failure and inspect the child output before trusting the result.",
            ]
    except Exception as exc:  # noqa: BLE001
        result["status"] = "runtime_probe_failed"
        result["runtime_error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if args.json_output is not None:
            write_json_output(args.json_output, result)
        if hand is not None:
            del hand

    print(json_dumps(result))
    if result["status"] == "runtime_probe_completed":
        return 0
    if result["status"] == "blocked_before_runtime":
        return 3
    if result["status"] == "runtime_probe_failed":
        return 4
    if result["status"] == "runtime_transport_unavailable":
        return 5
    if result["status"] == "runtime_probe_crashed":
        return 6
    return 4


if __name__ == "__main__":
    raise SystemExit(main())