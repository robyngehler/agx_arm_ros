#!/usr/bin/env python3.10
"""Read-only OmniHand Pro (O12) hardware probe.

Opens the Pro hand over native SocketCAN and prints device facts and live
readback WITHOUT commanding any motion. Use this as the first hardware check
after a green SDK build, before the ROS bridge drives the hand.

IMPORTANT: do not run this while the ROS bridge (backend_type:=sdk) is also
holding the same hand — only one process may own the CAN session.

    python3.10 scripts/omnihand_pro/pro_hardware_probe.py --side right --iface can_nero_right

Active joint order (12, AgibotHandO12 / OmniHand Pro manual §2.4):
    thumb_roll, thumb_abad, thumb_mcp, thumb_pip,
    index_abad, index_mcp, index_pip,
    middle_abad, middle_mcp, middle_pip,
    ring_mcp, pinky_mcp
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

_VENDOR_PKG_REL = Path("vendor") / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"


def _ensure_on_path() -> None:
    try:
        import agibot_hand  # noqa: F401
        return
    except ImportError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _VENDOR_PKG_REL
        if (candidate / "agibot_hand" / "__init__.py").exists():
            sys.path.insert(0, str(candidate))
            return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--iface", default="can_nero_right", help="SocketCAN interface (OMNIHAND_SOCKETCAN_IFACE)")
    parser.add_argument("--device-id", type=int, default=1)
    args = parser.parse_args(argv)

    os.environ["OMNIHAND_SOCKETCAN_IFACE"] = args.iface

    _ensure_on_path()
    try:
        from agibot_hand import AgibotHandO12, EFinger, EHandType
    except ImportError as exc:
        print(f"import FAILED: {exc}; build vendor/OmniHand-Pro-2025 first.")
        return 1

    hand_type = EHandType.RIGHT if args.side == "right" else EHandType.LEFT
    print(f"Opening OmniHand Pro ({args.side}, device_id={args.device_id}, iface={args.iface})")
    hand = AgibotHandO12(device_id=args.device_id, hand_type=hand_type)
    if hasattr(hand, "show_data_details"):
        hand.show_data_details(False)

    def _safe(label, fn):
        try:
            value = fn()
            print(f"{label}: {value}")
        except Exception as exc:  # noqa: BLE001
            print(f"{label}: <error: {exc}>")

    _safe("vendor_info", lambda: hand.get_vendor_info())
    _safe("device_info", lambda: hand.get_device_info())

    active = None
    try:
        active = list(hand.get_all_active_joint_angles())
        print(f"active_joint_angles ({len(active)}): {[round(v, 4) for v in active]}")
    except Exception as exc:  # noqa: BLE001
        print(f"active_joint_angles: <error: {exc}>")

    _safe("all_joint_angles", lambda: [round(v, 4) for v in hand.get_all_joint_angles()])
    _safe("joint_positions (raw motor)", lambda: list(hand.get_all_joint_positions()))
    _safe("temperature_reports", lambda: list(hand.get_all_temperature_reports()))
    _safe("current_reports", lambda: list(hand.get_all_current_reports()))

    try:
        reports = list(hand.get_all_error_reports())
        flagged = [
            (i, [f for f in ("stalled", "overheat", "over_current", "motor_except", "commu_except")
                 if getattr(r, f, False)])
            for i, r in enumerate(reports)
        ]
        flagged = [(i, fs) for i, fs in flagged if fs]
        print(f"error_reports ({len(reports)}): flagged={flagged or 'none'}")
    except Exception as exc:  # noqa: BLE001
        print(f"error_reports: <error: {exc}>")

    for finger in (EFinger.THUMB, EFinger.INDEX, EFinger.MIDDLE, EFinger.RING, EFinger.LITTLE):
        try:
            d = hand.get_tactile_sensor_data(finger)
            print(
                f"tactile[{finger.name}]: online={d.online_state} normal={d.normal_force} "
                f"tangent={d.tangent_force} angle={d.tangent_force_angle} "
                f"channels={len(list(d.channel_values))} capacitive={len(list(d.capacitive_approach))}"
            )
        except Exception as exc:  # noqa: BLE001
            print(f"tactile[{finger.name}]: <error: {exc}>")

    if active is not None and len(active) != 12:
        print(f"WARNING: expected 12 active joints, got {len(active)} — check hand model.")
    print("probe done (read-only; no motion commanded).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
