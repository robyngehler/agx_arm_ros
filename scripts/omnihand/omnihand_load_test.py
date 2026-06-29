#!/usr/bin/env python3
"""Vendor-level OmniHand communication load / stress test (below ROS).

The vendor SDK ships only one-shot demos (create_hand -> a few calls -> exit).
None of them hold a persistent connection the way normal operation does, so
there was no ready way to measure the OmniHand's steady-state CAN FD load.

This script opens ONE persistent SDK connection and drives the same call mix
the ROS bridge (SdkOmniHandBackend) uses in normal operation:

  - read joint positions every tick   (default 50 Hz, like bridge pub_rate)
  - read error reports at status rate  (default 1 Hz)
  - optionally read tactile per finger (default off; adds 5 calls/cycle)
  - optionally send a gentle joint sweep at command rate (default off, safe)

It runs for a fixed duration and prints achieved call rates and per-call
latency, so you can capture the bus alongside it, e.g.:

    # terminal 1 — capture the side bus while the test runs
    candump -l can_nero_right            # writes candump-*.log
    # or a pcap for Wireshark:
    sudo tcpdump -i can_nero_right -w ~/omnihand_load.pcap

    # terminal 2 — run the sustained load
    cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025
    PYTHONPATH=$PWD/build/agibot_hand_pkg \
    LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \
    OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
    python3.10 ~/workspace/agx_arm_ros/scripts/omnihand/omnihand_load_test.py \
        --hand-type right --duration 30

Read-only by default. Pass --with-commands to also exercise the write path
(a small, clamped sweep around the current pose).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable


VENDOR_ROOT = Path(__file__).resolve().parents[2] / "vendor" / "OmniHand-Pro-2025"
# The BUILT package carries the compiled agibot_hand_core .so; the source tree
# under python/ does NOT. Only the built package can actually talk to the hand.
VENDOR_BUILT_PKG = VENDOR_ROOT / "build" / "agibot_hand_pkg"

# The OmniHand exposes 10 active joints. The SDK sometimes returns a padded
# 12-value vector (the bridge trims it the same way); set_all_active_joint_angles
# requires exactly 10. Index 4 is the index PIP (range ~0..1.48), safe to flex.
ACTIVE_JOINT_COUNT = 10
SWEEP_JOINT_INDEX = 4
SWEEP_JOINT_MIN = 0.0
SWEEP_JOINT_MAX = 1.4
SAFE_OPEN_POSE = [0.0] * ACTIVE_JOINT_COUNT


@dataclass
class CallStats:
    """Latency and count bookkeeping for one SDK call kind."""

    name: str
    count: int = 0
    errors: int = 0
    latencies_ms: list[float] = field(default_factory=list)

    def record(self, latency_ms: float) -> None:
        self.count += 1
        self.latencies_ms.append(latency_ms)

    def record_error(self) -> None:
        self.errors += 1

    def summary(self, elapsed_s: float) -> str:
        if not self.latencies_ms:
            return f"  {self.name:<26} count=0 errors={self.errors}"
        rate = self.count / elapsed_s if elapsed_s > 0 else 0.0
        mean_ms = statistics.fmean(self.latencies_ms)
        p95_ms = (
            statistics.quantiles(self.latencies_ms, n=20)[18]
            if len(self.latencies_ms) >= 20
            else max(self.latencies_ms)
        )
        max_ms = max(self.latencies_ms)
        return (
            f"  {self.name:<26} count={self.count:<7} errors={self.errors:<4} "
            f"rate={rate:6.1f}/s  mean={mean_ms:5.2f}ms p95={p95_ms:5.2f}ms max={max_ms:6.2f}ms"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sustained vendor-level OmniHand communication load test.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--canfd-id", type=int, default=0)
    parser.add_argument(
        "--hand-type", choices=("left", "right"), default="right"
    )
    parser.add_argument(
        "--duration", type=float, default=30.0, help="Test duration in seconds."
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Joint-readback rate in Hz (matches bridge pub_rate).",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=1.0,
        help="Error-report readback rate in Hz.",
    )
    parser.add_argument(
        "--tactile",
        action="store_true",
        help="Also read tactile per finger at status rate (adds 5 calls/cycle).",
    )
    parser.add_argument(
        "--with-commands",
        action="store_true",
        help="Also send a small clamped joint sweep at the joint rate (write path).",
    )
    parser.add_argument(
        "--sweep-amplitude-rad",
        type=float,
        default=0.05,
        help="Peak amplitude of the optional command sweep, in radians.",
    )
    parser.add_argument(
        "--report-interval-s",
        type=float,
        default=5.0,
        help="How often to print a running summary.",
    )
    parser.add_argument(
        "--sdk-python-dir",
        type=Path,
        default=None,
        help=(
            "Explicit built agibot_hand_pkg directory to prepend to sys.path. "
            "By default the existing PYTHONPATH is respected (do not point this at "
            "the source python/ tree — it lacks the compiled agibot_hand_core)."
        ),
    )
    return parser.parse_args()


def load_sdk(sdk_python_dir: Path | None) -> Any:
    """Import the vendor SDK, respecting an already-set PYTHONPATH.

    The built package (build/agibot_hand_pkg) carries the compiled
    agibot_hand_core .so; the source python/ tree does not. We must NOT shadow
    the user's PYTHONPATH with the source tree, or the import resolves to a
    core-less package and fails. So: only an explicit --sdk-python-dir is
    prepended; otherwise we trust PYTHONPATH and fall back to the built package.
    """
    if sdk_python_dir is not None:
        sys.path.insert(0, str(sdk_python_dir))

    try:
        import agibot_hand  # type: ignore
    except ModuleNotFoundError as exc:
        # Two recoverable cases: the package is not on the path at all, or
        # PYTHONPATH points at the source tree (package present but no compiled
        # core). Either way, retry with the built package prepended.
        recoverable = exc.name in ("agibot_hand", "agibot_hand.agibot_hand_core")
        if recoverable and VENDOR_BUILT_PKG.is_dir():
            sys.modules.pop("agibot_hand", None)
            sys.path.insert(0, str(VENDOR_BUILT_PKG))
            try:
                import agibot_hand  # type: ignore  # noqa: F811
            except Exception as retry_exc:  # noqa: BLE001
                raise SystemExit(_import_hint(retry_exc))
        else:
            raise SystemExit(_import_hint(exc))
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(_import_hint(exc))

    if not hasattr(agibot_hand, "AgibotHandO12"):
        raise SystemExit(
            "Imported agibot_hand is missing AgibotHandO12 — PYTHONPATH is "
            "probably pointing at the source python/ tree instead of the built "
            f"package ({VENDOR_BUILT_PKG})."
        )
    return agibot_hand


def _import_hint(exc: Exception) -> str:
    return (
        f"Failed to import agibot_hand: {type(exc).__name__}: {exc}\n"
        "The compiled SDK lives in the BUILT package, not the source python/ tree.\n"
        "Set the environment to the built package (note: PYTHONPATH must point at\n"
        "the package ROOT, LD_LIBRARY_PATH at the agibot_hand subdir):\n\n"
        f"  cd {VENDOR_ROOT}\n"
        "  PYTHONPATH=$PWD/build/agibot_hand_pkg \\\n"
        "  LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \\\n"
        "  OMNIHAND_SOCKETCAN_IFACE=can_nero_right \\\n"
        "  python3.10 <this-script> --hand-type right"
    )


def create_hand(sdk: Any, device_id: int, canfd_id: int, hand_type: Any) -> Any:
    """Create a hand instance using the O12 Pro SDK constructor."""
    return sdk.AgibotHandO12(device_id=device_id, hand_type=hand_type)


def timed(stats: CallStats, fn: Callable[[], Any]) -> Any:
    start = time.perf_counter()
    try:
        result = fn()
    except Exception:  # noqa: BLE001
        stats.record_error()
        return None
    stats.record((time.perf_counter() - start) * 1000.0)
    return result


def read_joint_positions(hand: Any) -> Callable[[], Any]:
    """Pick the readback call the bridge prefers, falling back like it does."""
    if hasattr(hand, "get_all_joint_positions"):
        return hand.get_all_joint_positions
    return hand.get_all_active_joint_angles


def acquire_reference_pose(hand: Any) -> tuple[list[float], bool]:
    """Return a valid 10-value active-joint pose to sweep around.

    Tries the current active-joint angles, trimming a padded 12-value vector to
    10 (as the bridge does). Some firmware fails that readback and returns a
    short/empty vector; in that case fall back to the safe open pose. The bool is
    True when a real current pose was obtained, False when the open fallback is
    used (which means the hand will first move to open).
    """
    try:
        raw = list(hand.get_all_active_joint_angles())
    except Exception:  # noqa: BLE001
        raw = []
    pose = [float(value) for value in raw[:ACTIVE_JOINT_COUNT]]
    if len(pose) == ACTIVE_JOINT_COUNT and all(math.isfinite(value) for value in pose):
        return pose, True
    return list(SAFE_OPEN_POSE), False


def main() -> int:
    args = parse_args()
    sdk = load_sdk(args.sdk_python_dir)
    hand_type = (
        sdk.EHandType.LEFT if args.hand_type == "left" else sdk.EHandType.RIGHT
    )

    print(
        f"Opening OmniHand ({args.hand_type}, device_id={args.device_id}, "
        f"canfd_id={args.canfd_id}) ..."
    )
    hand = create_hand(sdk, args.device_id, args.canfd_id, hand_type)
    if hasattr(hand, "show_data_details"):
        hand.show_data_details(False)

    joint_read = read_joint_positions(hand)
    baseline: list[float] | None = None
    if args.with_commands:
        baseline, from_hand = acquire_reference_pose(hand)
        if from_hand:
            print("  commands: sweeping the index PIP around the current pose.")
        else:
            print(
                "  commands: active-joint readback was incomplete; using the OPEN "
                "pose as reference. THE HAND WILL MOVE TO OPEN, then flex one joint."
            )

    tactile_fingers = (
        [sdk.EFinger.THUMB, sdk.EFinger.INDEX, sdk.EFinger.MIDDLE,
         sdk.EFinger.RING, sdk.EFinger.LITTLE]
        if args.tactile
        else []
    )

    joint_stats = CallStats("get_joint_positions")
    status_stats = CallStats("get_all_error_reports")
    tactile_stats = CallStats("get_tactile_sensor_data")
    command_stats = CallStats("set_active_joint_angles")

    joint_period = 1.0 / args.rate if args.rate > 0 else 0.02
    status_period = 1.0 / args.status_rate if args.status_rate > 0 else 1.0

    print(
        f"Running for {args.duration:.0f}s: joint {args.rate:.0f} Hz, "
        f"status {args.status_rate:.0f} Hz, tactile={'on' if args.tactile else 'off'}, "
        f"commands={'on' if args.with_commands else 'off'}.\n"
        "Capture the bus now (candump / tcpdump). Ctrl-C to stop early.\n"
    )

    start = time.monotonic()
    end = start + args.duration
    next_status = start
    next_report = start + args.report_interval_s
    next_tick = start

    try:
        while True:
            now = time.monotonic()
            if now >= end:
                break

            timed(joint_stats, joint_read)

            if args.with_commands and baseline is not None:
                # One-directional sweep in [0, amplitude] so it never goes
                # negative; flex only the index PIP, clamped to a safe range.
                phase = (now - start) * 2.0  # ~0.3 Hz sweep
                offset = args.sweep_amplitude_rad * 0.5 * (1.0 - math.cos(phase))
                target = list(baseline)
                swept = target[SWEEP_JOINT_INDEX] + offset
                target[SWEEP_JOINT_INDEX] = min(max(swept, SWEEP_JOINT_MIN), SWEEP_JOINT_MAX)
                timed(command_stats, lambda t=target: hand.set_all_active_joint_angles(t))

            if now >= next_status:
                timed(status_stats, hand.get_all_error_reports)
                for finger in tactile_fingers:
                    timed(tactile_stats, lambda f=finger: hand.get_tactile_sensor_data(f))
                next_status += status_period

            if now >= next_report:
                _print_running(now - start, joint_stats, status_stats,
                               tactile_stats, command_stats, args)
                next_report += args.report_interval_s

            next_tick += joint_period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()  # fell behind; resync
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if args.with_commands and baseline is not None:
            try:
                hand.set_all_active_joint_angles(baseline)
            except Exception:  # noqa: BLE001
                pass

    elapsed = time.monotonic() - start
    print("\n=== Final summary ===")
    print(f"  elapsed: {elapsed:.2f}s")
    for stats in (joint_stats, command_stats, status_stats, tactile_stats):
        if stats.count or stats.errors:
            print(stats.summary(elapsed))
    total_calls = sum(s.count for s in (joint_stats, command_stats, status_stats, tactile_stats))
    print(f"  total SDK calls: {total_calls}  ({total_calls / elapsed:.1f}/s)")
    print(
        "\nCross-check against the captured bus: compare the candump/pcap frame\n"
        "rate and bus utilization with the SDK call rate above. See\n"
        "docs/development/sprint5/planning/can_transport_decision.md for the\n"
        "arm-side budget this hand load adds to."
    )
    return 0


def _print_running(
    elapsed: float,
    joint_stats: CallStats,
    status_stats: CallStats,
    tactile_stats: CallStats,
    command_stats: CallStats,
    args: argparse.Namespace,
) -> None:
    print(f"[t={elapsed:5.1f}s]")
    for stats in (joint_stats, command_stats, status_stats, tactile_stats):
        if stats.count or stats.errors:
            print(stats.summary(elapsed))


if __name__ == "__main__":
    raise SystemExit(main())
