#!/usr/bin/env python3
"""Automated TDC (transmitter delay compensation) sweep for the mttcan side buses.

Finds the tdc_offset register value with the most reliable OmniHand CANFD
request/response link on can_nero_left / can_nero_right. For every candidate
value the script:

  1. takes the interface down, writes the mttcan sysfs ``tdc_offset``, and
     brings the bus back up with the validated 1M/5M FD timing
     (same parameters as scripts/activate_native_can.sh),
  2. snapshots the SocketCAN error counters (bus_error, berr-counter, drops),
  3. runs a fresh vendor-SDK worker process that issues N joint readback
     requests (the exact request/response cycle that fails with 请求超时) and
     optionally a fist -> zero motion cycle with readback verification,
  4. scores the value: success rate first, then bus-error delta, then latency.

Results land in a CSV plus a ranked summary. The pre-sweep tdc_offset is
restored unless --apply-best is given.

mttcan/M_CAN background: tdc_offset is the raw TDCR register value, TDCO lives
in bits [14:8] in units of one CAN-clock tick (20 ns at the 50 MHz mttcan
clock). One data bit at 5 Mbit is 10 ticks, so TDCO=8 (register 0x800) puts the
secondary sample point at 80 % of the bit — matching dsample-point 0.8. The
default sweep therefore covers TDCO 0..15 (0x000..0xF00).

MUST run as root. The OmniHand bridge (or any other SDK session) must be
stopped — the vendor SDK needs the hand exclusively. The ARM stack is handled
in two stages:

  Stage 1 (idle bus): run with everything stopped. This maps the raw TDCO
  window in which the PHY works at all. Expect a wide flat plateau — on an
  idle bus almost every in-window value scores 100 %.
  Stage 2 (--arm-load): start the arm bringup WITHOUT the hand bridges
  (e.g. execution_profile:=duo_arm or launch_omnihand_bridge:=false) and put
  the MIT controller into ACTIVE regulation — merely launching it is not load
  (it stays DISABLED and sends nothing; the >2000 rx pkt/s you still see is
  only the firmware feedback push). Activation per arm namespace — no manual
  set_motion_mode needed, the driver handshakes MIT mode on the first command:

    ros2 service call /left_arm/mit_controller/enable std_srvs/srv/SetBool "{data: true}"
    ros2 service call /left_arm/mit_controller/hold_current std_srvs/srv/Empty
    (same for /right_arm; agx_arm_test_position_hold --leave-mit-enabled does
    the full sequence including enable_agx_arm/set_normal_mode)

  The controller then streams 7 move_mit frames per tick at control_rate_hz
  (~700 tx/s per arm at 100 Hz). The sweep records the per-trial TX rate and
  flags trials below ~150 tx/s as 'low-tx-load(mit-not-streaming?)'. Disable
  afterwards with the same service and {data: false}.

The final recommendation is the CENTER of the longest clean TDCO window, not
the lowest passing value — borderline values (TDCO 0/1 here) are flaky.

Examples:
    sudo python3.10 scripts/can_tdc_sweep.py --motion none            # stage 1, TDCO 0..15
    sudo python3.10 scripts/can_tdc_sweep.py --arm-load --passes 3 \
        --values 0x200,0x300,...,0xd00 --requests 30 --motion fist    # stage 2
    sudo python3.10 scripts/can_tdc_sweep.py --dry-run                # print plan only
    sudo python3.10 scripts/can_tdc_sweep.py --apply-best             # keep the winner configured
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
VENDOR_PKG = REPO_ROOT / "vendor" / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"

SIDE_IFACES = {"left": "can_nero_left", "right": "can_nero_right"}

# Same bus timing as scripts/activate_native_can.sh (validated OmniHand timing).
BITRATE = "1000000"
SAMPLE_POINT = "0.8"
DBITRATE = "5000000"
DSAMPLE_POINT = "0.8"
RESTART_MS = "100"
TX_QUEUE_LEN = "1000"

# Canonical RIGHT-hand active-joint vectors from
# src/agx_arm_ctrl/config/omnihand_pro_gestures.yaml; the left hand mirrors
# thumb_roll and thumb_abad (indices 0 and 1).
GESTURE_ZERO = [0.0] * 12
GESTURE_FIST = [0.5, -0.2, 0.0, -1.2, 0.0, 1.35, 1.53, 0.0, 1.36, 1.82, 1.55, 1.54]
MIRRORED_INDICES = (0, 1)

# The vendor SDK needs an exclusive hand session — these always conflict.
HAND_SESSION_PATTERN = (
    "omnihand_bridge|omnihand_follow_joint_trajectory|pro_hardware_probe|omnihand_load_test"
)
# The arm stack shares the bus but not the hand session. Without --arm-load it
# aborts the sweep (idle-bus PHY measurement); with --arm-load it is the
# intended live load and its absence is flagged instead.
ARM_STACK_PATTERN = "agx_arm_ctrl_single|agx_arm_mit_controller|agx_arm_teach_manager"

# Minimum TRANSMIT rate (frames/s) for an --arm-load trial to count as loaded.
# The discriminating load is the MIT command stream we transmit (7 move_mit
# frames per control tick, ~700 tx/s per arm at 100 Hz) — the firmware's
# autonomous feedback push alone produces >2000 rx pkt/s on an otherwise idle
# bus and must not satisfy the check.
LOADED_BUS_MIN_TX_PPS = 150.0


def _run(cmd: list[str], check: bool = True, capture: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=check, capture_output=capture, text=True)


def _mirror_for_left(vector: list[float]) -> list[float]:
    mirrored = list(vector)
    for i in MIRRORED_INDICES:
        mirrored[i] = -mirrored[i]
    return mirrored


# --------------------------------------------------------------------------
# Worker mode: one fresh process per (tdc value, side) so a hung SDK session
# can be killed from the parent and no CAN/session state leaks between trials.
# --------------------------------------------------------------------------

def _worker_main(args: argparse.Namespace) -> int:
    os.environ["OMNIHAND_SOCKETCAN_IFACE"] = args.iface
    if VENDOR_PKG.joinpath("agibot_hand", "__init__.py").exists():
        sys.path.insert(0, str(VENDOR_PKG))

    result: dict = {
        "connect_ok": False,
        "connect_error": "",
        "requests": args.requests,
        "ok": 0,
        "latencies_ms": [],
        "motion": None,
    }

    def _write_and_exit(code: int) -> int:
        Path(args.out).write_text(json.dumps(result), encoding="utf-8")
        return code

    try:
        from agibot_hand import AgibotHandO12, EHandType  # noqa: PLC0415
    except ImportError as exc:
        result["connect_error"] = f"agibot_hand import failed: {exc}"
        return _write_and_exit(1)

    try:
        hand = AgibotHandO12(
            device_id=args.device_id,
            hand_type=EHandType.RIGHT if args.side == "right" else EHandType.LEFT,
        )
        if hasattr(hand, "show_data_details"):
            hand.show_data_details(False)
        result["connect_ok"] = True
    except Exception as exc:  # noqa: BLE001
        result["connect_error"] = str(exc)
        return _write_and_exit(1)

    def _read_angles() -> list[float] | None:
        try:
            angles = list(hand.get_all_active_joint_angles())
        except Exception:  # noqa: BLE001
            return None
        return angles if len(angles) == 12 else None

    for _ in range(args.requests):
        start = time.perf_counter()
        angles = _read_angles()
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if angles is not None:
            result["ok"] += 1
            result["latencies_ms"].append(round(elapsed_ms, 2))
        time.sleep(args.request_gap_s)

    if args.motion != "none" and result["ok"] > 0:
        target = GESTURE_FIST if args.motion == "fist" else GESTURE_ZERO
        if args.side == "left":
            target = _mirror_for_left(target)
        home = _mirror_for_left(GESTURE_ZERO) if args.side == "left" else GESTURE_ZERO

        def _command_until_confirmed(vector: list[float]) -> tuple[bool, int, float]:
            # Same eventual-delivery idea as the bridge command retry: re-send
            # until the readback confirms, up to max attempts. Delivery is
            # judged by PROGRESS from the pre-command baseline toward the
            # target, not by absolute per-joint tolerance — command==readback
            # is not calibrated on the Pro (couplings, unreachable extremes),
            # so an absolute check fails even when the hand visibly moves.
            baseline = _read_angles()
            if baseline is None:
                return False, 0, 0.0
            moving = [
                i for i in range(12)
                if abs(vector[i] - baseline[i]) > args.motion_tolerance_rad
            ]
            if not moving:
                return True, 0, 1.0  # already at target; nothing to prove
            best_progress = 0.0
            for attempt in range(1, args.motion_max_attempts + 1):
                try:
                    hand.set_all_active_joint_angles(vector)
                except Exception:  # noqa: BLE001
                    time.sleep(args.motion_verify_period_s)
                    continue
                time.sleep(args.motion_verify_period_s)
                angles = _read_angles()
                if angles is not None:
                    ratios = [
                        max(0.0, min(1.0, (angles[i] - baseline[i]) / (vector[i] - baseline[i])))
                        for i in moving
                    ]
                    best_progress = max(best_progress, sum(ratios) / len(ratios))
                if best_progress >= args.motion_min_progress:
                    return True, attempt, round(best_progress, 2)
            return False, args.motion_max_attempts, round(best_progress, 2)

        target_ok, target_attempts, target_progress = _command_until_confirmed(target)
        home_ok, home_attempts, home_progress = _command_until_confirmed(home)
        result["motion"] = {
            "gesture": args.motion,
            "target_confirmed": target_ok,
            "target_attempts": target_attempts,
            "target_progress": target_progress,
            "return_confirmed": home_ok,
            "return_attempts": home_attempts,
            "return_progress": home_progress,
        }

    return _write_and_exit(0)


# --------------------------------------------------------------------------
# Parent mode: bus reconfiguration, stats, orchestration, scoring.
# --------------------------------------------------------------------------

def _tdc_sysfs_path(iface: str) -> str | None:
    # /sys/class/net/<iface> symlinks into the mttcan platform device, so the
    # attribute is reachable without walking /sys/devices/platform.
    path = Path("/sys/class/net") / iface / "tdc_offset"
    return str(path) if path.exists() else None


def _read_tdc_offset(iface: str) -> str:
    path = _tdc_sysfs_path(iface)
    if not path:
        return ""
    raw = Path(path).read_text(encoding="utf-8").strip()
    # Format: "tdc_offset=0x200, DBTP.tdc=1"
    for token in raw.replace(",", " ").split():
        if token.startswith("tdc_offset="):
            return token.split("=", 1)[1]
    return raw


def _write_tdc_offset(iface: str, reg_value: int) -> None:
    path = _tdc_sysfs_path(iface)
    if not path:
        raise RuntimeError(f"tdc_offset sysfs entry not found for '{iface}' (mttcan only)")
    Path(path).write_text(f"{reg_value:#x}\n", encoding="utf-8")


def _reconfigure_iface(iface: str, reg_value: int, one_shot: str) -> None:
    _run(["ip", "link", "set", iface, "down"])
    _write_tdc_offset(iface, reg_value)
    type_cmd = [
        "ip", "link", "set", iface, "type", "can",
        "bitrate", BITRATE, "sample-point", SAMPLE_POINT,
        "dbitrate", DBITRATE, "dsample-point", DSAMPLE_POINT, "fd", "on",
        "restart-ms", RESTART_MS, "one-shot", one_shot,
    ]
    # berr-reporting is best-effort, mirroring activate_native_can.sh.
    if subprocess.run(type_cmd + ["berr-reporting", "on"], capture_output=True).returncode != 0:
        _run(type_cmd)
    _run(["ip", "link", "set", iface, "txqueuelen", TX_QUEUE_LEN])
    _run(["ip", "link", "set", iface, "up"])

    applied = _read_tdc_offset(iface)
    if f"{reg_value:#x}" not in applied:
        raise RuntimeError(f"{iface}: wrote tdc_offset {reg_value:#x} but sysfs reports '{applied}'")


def _can_stats(iface: str) -> dict:
    data = json.loads(_run(["ip", "-j", "-d", "-s", "link", "show", iface]).stdout)[0]
    xstats = data.get("linkinfo", {}).get("info_xstats", {})
    stats = data.get("stats64", {})
    return {
        "bus_error": xstats.get("bus_error", 0),
        "restarts": xstats.get("restarts", 0),
        "rx_errors": stats.get("rx", {}).get("errors", 0),
        "rx_dropped": stats.get("rx", {}).get("dropped", 0),
        "rx_packets": stats.get("rx", {}).get("packets", 0),
        "tx_errors": stats.get("tx", {}).get("errors", 0),
        "tx_dropped": stats.get("tx", {}).get("dropped", 0),
        "tx_packets": stats.get("tx", {}).get("packets", 0),
    }


def _matching_processes(pattern: str) -> list[str]:
    probe = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    own_pid = str(os.getpid())
    return [
        line for line in probe.stdout.strip().splitlines()
        if line and not line.startswith(own_pid + " ") and "can_tdc_sweep" not in line
    ]


def _check_processes(arm_load: bool, force: bool) -> None:
    hand_owners = _matching_processes(HAND_SESSION_PATTERN)
    if hand_owners:
        print("These processes hold the exclusive OmniHand SDK session:", file=sys.stderr)
        for line in hand_owners:
            print(f"  {line}", file=sys.stderr)
        if not force:
            print("Stop them first — two SDK sessions on one hand corrupt both. "
                  "(--force overrides at your own risk.)", file=sys.stderr)
            raise SystemExit(2)

    arm_stack = _matching_processes(ARM_STACK_PATTERN)
    if arm_load:
        if not arm_stack:
            print("WARNING: --arm-load given but no arm stack process is running — "
                  "the sweep will effectively measure an idle bus. Start the duo_arm "
                  "bringup (launch_omnihand_bridge:=false) with an active hold/stream "
                  "first.", file=sys.stderr)
        else:
            print("Arm stack detected as live load:", file=sys.stderr)
            for line in arm_stack:
                print(f"  {line}", file=sys.stderr)
            print("NOTE: each TDC value briefly takes the interface down; the arm "
                  "driver will see a short frame gap per value. Watch the arms.",
                  file=sys.stderr)
    elif arm_stack:
        print("Arm stack processes are running:", file=sys.stderr)
        for line in arm_stack:
            print(f"  {line}", file=sys.stderr)
        if not force:
            print("Use --arm-load to sweep under this load deliberately, or stop the "
                  "stack for a clean idle-bus sweep.", file=sys.stderr)
            raise SystemExit(2)


def _worker_interpreter() -> str:
    # The vendor .so is built for CPython 3.10 (agibot_hand_core.cpython-310-*).
    return shutil.which("python3.10") or "/usr/bin/python3.10"


def _run_worker(args: argparse.Namespace, side: str, iface: str) -> dict:
    with tempfile.NamedTemporaryFile(mode="r", suffix=".json", delete=False) as tmp:
        out_path = tmp.name
    worker_cmd = [
        _worker_interpreter(), str(Path(__file__).resolve()), "--worker",
        "--side", side, "--iface", iface, "--out", out_path,
        "--device-id", str(args.device_id),
        "--requests", str(args.requests),
        "--request-gap-s", str(args.request_gap_s),
        "--motion", args.motion,
        "--motion-tolerance-rad", str(args.motion_tolerance_rad),
        "--motion-min-progress", str(args.motion_min_progress),
        "--motion-verify-period-s", str(args.motion_verify_period_s),
        "--motion-max-attempts", str(args.motion_max_attempts),
    ]
    # Generous ceiling: SDK-internal timeouts already bound each request.
    timeout_s = 30 + args.requests * (args.request_gap_s + 2.0) + (
        0 if args.motion == "none" else 2 * args.motion_max_attempts * (args.motion_verify_period_s + 2.0)
    )
    try:
        proc = subprocess.run(worker_cmd, capture_output=True, text=True, timeout=timeout_s)
        note = "" if proc.returncode == 0 else "worker-nonzero-exit"
    except subprocess.TimeoutExpired:
        note = "worker-timeout-killed"
    try:
        result = json.loads(Path(out_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        result = {"connect_ok": False, "connect_error": note or "no worker output",
                  "requests": args.requests, "ok": 0, "latencies_ms": [], "motion": None}
    finally:
        Path(out_path).unlink(missing_ok=True)
    result["note"] = note
    return result


def _aggregate_by_value(side_rows: list[dict]) -> list[dict]:
    """Merge repeated passes of the same tdc value into one aggregate row."""
    grouped: dict[int, list[dict]] = {}
    for row in side_rows:
        grouped.setdefault(row["tdco"], []).append(row)
    aggregates = []
    for tdco in sorted(grouped):
        rows = grouped[tdco]
        requests = sum(r["requests"] for r in rows)
        ok = sum(r["ok"] for r in rows)
        motions = [r["motion_confirmed"] for r in rows if r["motion_confirmed"] is not None]
        p95s = [r["p95_ms"] for r in rows if r["p95_ms"] is not None]
        aggregates.append({
            "tdco": tdco,
            "tdc_reg": rows[0]["tdc_reg"],
            "trials": len(rows),
            "requests": requests,
            "ok": ok,
            "success_rate": ok / max(requests, 1),
            "bus_error_delta": sum(r["bus_error_delta"] for r in rows),
            "motion_confirmed": None if not motions else all(motions),
            "p95_ms": max(p95s) if p95s else None,
            "loaded": all(r.get("tx_pps", 0.0) >= LOADED_BUS_MIN_TX_PPS for r in rows),
        })
    return aggregates


def _score_key(row: dict) -> tuple:
    motion = row["motion_confirmed"]
    return (
        -row["success_rate"],
        0 if motion in (True, None) else 1,   # confirmed or not tested beats failed
        row["bus_error_delta"],
        row["p95_ms"] if row["p95_ms"] is not None else 1e9,
    )


def _recommended_value(aggregates: list[dict]) -> dict | None:
    """Center of the longest contiguous passing window, not merely the top rank.

    All-pass values tie under _score_key; picking the smallest would sit at the
    edge of the working TDCO window (seen live: TDCO 0 fails, 1 is flaky). The
    center of the widest fully-clean stretch keeps the most margin both ways.
    """
    if not aggregates:
        return None
    best = sorted(aggregates, key=_score_key)[0]
    if best["success_rate"] <= 0.0:
        # Nothing worked at any value: TDC is not the differentiator here, and
        # recommending the "least bad" 0 % row would be misleading noise.
        return None
    passing = [
        a["tdco"] for a in aggregates
        if a["success_rate"] >= best["success_rate"]
        and a["bus_error_delta"] <= best["bus_error_delta"]
        and a["motion_confirmed"] in (True, None)
    ]
    if not passing:
        return best
    runs: list[list[int]] = [[passing[0]]]
    for tdco in passing[1:]:
        if tdco == runs[-1][-1] + 1:
            runs[-1].append(tdco)
        else:
            runs.append([tdco])
    window = max(runs, key=len)
    center = window[len(window) // 2]
    return next(a for a in aggregates if a["tdco"] == center)


def _parse_values(args: argparse.Namespace) -> list[int]:
    if args.values:
        return [int(v.strip(), 0) for v in args.values.split(",") if v.strip()]
    return [tdco << 8 for tdco in range(args.tdco_min, args.tdco_max + 1, args.tdco_step)]


def _parent_main(args: argparse.Namespace) -> int:
    sides = ["left", "right"] if args.side == "both" else [args.side]
    ifaces = {side: (args.iface or SIDE_IFACES[side]) for side in sides}
    values = _parse_values(args)

    if args.dry_run:
        print(f"Plan: sides={sides}, ifaces={ifaces}, one_shot={args.one_shot}, "
              f"passes={args.passes}, arm_load={args.arm_load}")
        print(f"TDC register values: {[hex(v) for v in values]}")
        print(f"Per value: reconfigure bus, {args.requests} SDK readback requests, motion={args.motion}")
        print(f"CSV: {args.csv}")
        return 0

    if os.geteuid() != 0:
        print("Run with sudo (ip link + sysfs writes).", file=sys.stderr)
        return 1
    _check_processes(args.arm_load, args.force)

    original = {side: _read_tdc_offset(ifaces[side]) for side in sides}
    print(f"Original tdc_offset: { {ifaces[s]: original[s] for s in sides} }")

    rows: list[dict] = []
    csv_fields = [
        "timestamp", "side", "iface", "tdc_reg", "tdco", "pass",
        "requests", "ok", "success_rate", "mean_ms", "p95_ms",
        "motion_confirmed", "motion_attempts", "motion_progress",
        "bus_error_delta", "restarts_delta", "rx_errors_delta",
        "rx_dropped_delta", "tx_dropped_delta",
        "duration_s", "bus_pps", "tx_pps", "note",
    ]
    csv_path = Path(args.csv)
    if csv_path.exists():
        first_line = csv_path.read_text(encoding="utf-8").split("\n", 1)[0]
        if first_line.strip() != ",".join(csv_fields):
            stamped = csv_path.with_name(
                f"{csv_path.stem}_{_dt.datetime.now():%Y%m%d_%H%M%S}{csv_path.suffix}"
            )
            print(f"Existing CSV has an older column layout; writing to {stamped} instead.")
            csv_path = stamped
    write_header = not csv_path.exists()

    try:
        with csv_path.open("a", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=csv_fields)
            if write_header:
                writer.writeheader()

            for pass_index in range(1, args.passes + 1):
                for reg in values:
                    for side in sides:
                        iface = ifaces[side]
                        print(f"\n=== {iface} tdc_offset={reg:#x} (TDCO={reg >> 8}) "
                              f"pass {pass_index}/{args.passes} ===")
                        _reconfigure_iface(iface, reg, args.one_shot)
                        time.sleep(args.settle_s)

                        before = _can_stats(iface)
                        trial_start = time.monotonic()
                        result = _run_worker(args, side, iface)
                        duration_s = time.monotonic() - trial_start
                        after = _can_stats(iface)

                        latencies = result["latencies_ms"]
                        motion = result.get("motion")
                        tx_pps = (after["tx_packets"] - before["tx_packets"]) / max(duration_s, 0.001)
                        bus_pps = tx_pps + (
                            after["rx_packets"] - before["rx_packets"]
                        ) / max(duration_s, 0.001)
                        note = (result["note"] + " " + result.get("connect_error", "")).strip()
                        if args.arm_load and tx_pps < LOADED_BUS_MIN_TX_PPS:
                            note = (note + " low-tx-load(mit-not-streaming?)").strip()
                        row = {
                            "timestamp": _dt.datetime.now().isoformat(timespec="seconds"),
                            "side": side,
                            "iface": iface,
                            "tdc_reg": f"{reg:#x}",
                            "tdco": reg >> 8,
                            "pass": pass_index,
                            "requests": result["requests"],
                            "ok": result["ok"],
                            "success_rate": round(result["ok"] / max(result["requests"], 1), 3),
                            "mean_ms": round(statistics.mean(latencies), 1) if latencies else None,
                            "p95_ms": round(sorted(latencies)[int(0.95 * (len(latencies) - 1))], 1) if latencies else None,
                            "motion_confirmed": None if motion is None else (
                                motion["target_confirmed"] and motion["return_confirmed"]
                            ),
                            "motion_attempts": None if motion is None else (
                                motion["target_attempts"] + motion["return_attempts"]
                            ),
                            "motion_progress": None if motion is None else motion["target_progress"],
                            "bus_error_delta": after["bus_error"] - before["bus_error"],
                            "restarts_delta": after["restarts"] - before["restarts"],
                            "rx_errors_delta": after["rx_errors"] - before["rx_errors"],
                            "rx_dropped_delta": after["rx_dropped"] - before["rx_dropped"],
                            "tx_dropped_delta": after["tx_dropped"] - before["tx_dropped"],
                            "duration_s": round(duration_s, 1),
                            "bus_pps": round(bus_pps, 1),
                            "tx_pps": round(tx_pps, 1),
                            "note": note,
                        }
                        rows.append(row)
                        writer.writerow(row)
                        csv_file.flush()
                        print(
                            f"  readback {row['ok']}/{row['requests']} ok "
                            f"(mean {row['mean_ms']} ms, p95 {row['p95_ms']} ms), "
                            f"motion={row['motion_confirmed']} "
                            f"(progress {row['motion_progress']}, {row['motion_attempts']} attempts), "
                            f"bus_error +{row['bus_error_delta']}, "
                            f"load {row['bus_pps']:.0f} pkt/s (tx {row['tx_pps']:.0f}/s)"
                            + (f"  [{row['note']}]" if row["note"] else "")
                        )
    finally:
        recommended: dict[str, dict] = {}
        for side in sides:
            aggregates = _aggregate_by_value([r for r in rows if r["side"] == side])
            pick = _recommended_value(aggregates)
            if pick is not None:
                recommended[side] = pick

        for side in sides:
            iface = ifaces[side]
            if args.apply_best and side in recommended:
                reg = int(recommended[side]["tdc_reg"], 16)
                print(f"\nApplying recommended value on {iface}: {reg:#x}")
                _reconfigure_iface(iface, reg, args.one_shot)
            elif original[side]:
                print(f"\nRestoring original tdc_offset on {iface}: {original[side]}")
                _reconfigure_iface(iface, int(original[side], 16), args.one_shot)

    print("\n================ RANKING ================")
    for side in sides:
        aggregates = sorted(
            _aggregate_by_value([r for r in rows if r["side"] == side]), key=_score_key
        )
        print(f"\n{ifaces[side]}:")
        for rank, agg in enumerate(aggregates, 1):
            marker = ""
            if side in recommended and agg["tdco"] == recommended[side]["tdco"]:
                marker = " <-- RECOMMENDED (center of passing window)"
            load_flag = "" if agg["loaded"] or not args.arm_load else " LOW-LOAD"
            print(
                f"  {rank:2d}. tdc={agg['tdc_reg']:>6} (TDCO {agg['tdco']:3d})  "
                f"readback {agg['success_rate']:5.0%} ({agg['ok']}/{agg['requests']})  "
                f"bus_err +{agg['bus_error_delta']:<5d} motion={agg['motion_confirmed']}"
                f"{load_flag}{marker}"
            )
        if side in recommended:
            print(
                f"  -> persist with: TDCR_VALUE={recommended[side]['tdc_reg']} "
                f"sudo bash scripts/activate_native_can.sh {side}"
            )
        elif aggregates:
            print(
                "  -> no recommendation: 0% success at every value — the failure "
                "is not TDC-dependent (check bus contention / FD tolerance of the "
                "other nodes instead)."
            )
    print(f"\nCSV: {csv_path.resolve()}")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--iface", default="", help="Override interface (single --side only).")
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--values", default="",
                        help="Comma-separated raw tdc_offset register values (e.g. 0x600,0x800). "
                             "Overrides the TDCO range.")
    parser.add_argument("--tdco-min", type=int, default=0)
    parser.add_argument("--tdco-max", type=int, default=15)
    parser.add_argument("--tdco-step", type=int, default=1)
    parser.add_argument("--requests", type=int, default=25,
                        help="SDK joint-readback requests per value (default 25).")
    parser.add_argument("--request-gap-s", type=float, default=0.05)
    parser.add_argument("--motion", choices=("none", "zero", "fist"), default="fist",
                        help="Motion check per value: command gesture, verify readback, return to zero. "
                             "'fist' proves delivery via a visible state change (default). THE HAND MOVES.")
    parser.add_argument("--motion-tolerance-rad", type=float, default=0.20,
                        help="Joints whose commanded change exceeds this count as 'moving' "
                             "for the progress check.")
    parser.add_argument("--motion-min-progress", type=float, default=0.6,
                        help="Mean fraction of the commanded joint deltas the readback must "
                             "cover to count as delivered (default 0.6).")
    parser.add_argument("--motion-verify-period-s", type=float, default=0.3)
    parser.add_argument("--motion-max-attempts", type=int, default=8)
    parser.add_argument("--passes", type=int, default=1,
                        help="Repeat the whole value list N times; the ranking aggregates "
                             "all passes (borderline TDCO values are flaky — one pass can lie).")
    parser.add_argument("--arm-load", action="store_true",
                        help="Sweep while the arm stack runs as real bus load (start the "
                             "duo_arm bringup with launch_omnihand_bridge:=false and keep the "
                             "MIT controller actively commanding). Only bridge-type processes "
                             "still block the sweep; per-trial bus load is measured and "
                             "low-load trials are flagged.")
    parser.add_argument("--one-shot", choices=("on", "off"), default="off",
                        help="Bus one-shot mode during the sweep (default off = allows retransmits, "
                             "cleanest PHY signal; validate the winner with your production setting too).")
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--csv", default="tdc_sweep_results.csv")
    parser.add_argument("--apply-best", action="store_true",
                        help="Leave the best value configured instead of restoring the original.")
    parser.add_argument("--force", action="store_true",
                        help="Override ALL process checks, including a live hand-SDK session "
                             "(dangerous; prefer --arm-load for deliberate load sweeps).")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan, touch nothing.")
    # Internal worker mode.
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--out", default="", help=argparse.SUPPRESS)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.worker:
        if args.side not in ("left", "right") or not args.iface or not args.out:
            print("--worker needs --side left|right, --iface and --out", file=sys.stderr)
            return 2
        return _worker_main(args)
    if args.iface and args.side == "both":
        print("--iface requires --side left or --side right", file=sys.stderr)
        return 2
    return _parent_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
