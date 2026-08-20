#!/usr/bin/env python3
"""Analyse a SocketCAN pcap: frame rates, direction split, and Nero feedback fields.

Written for the Phase 0 baseline, and kept because the same numbers are what the
post-refactor comparison needs. Reads the pcap directly — no tshark on the
Jetson — and understands the Nero high-speed feedback layout, so it can answer
the question that motivated it: does the firmware report joint velocity at all?

Usage:
    ./scripts/analyze_can_pcap.py <file.pcap> [more.pcap ...]
    ./scripts/analyze_can_pcap.py --stop-at <unix_ts> <file.pcap> [more.pcap ...]

``--stop-at`` adds the emergency-stop signature: what crossed the wire around a
stop, and above all whether the vendor electronic stop did. See
docs/sprint_refactor/reference/emergency_stop_ladder.md.

Reference: docs/sprint_refactor/reference/phase0_baseline.md
"""

from __future__ import annotations

import argparse
import collections
import struct
import sys

# Nero CAN layout (vendor protocol docs, ArmMsgFeedbackHighSpd):
#   0x251..0x257  per-joint high-speed feedback
#       bytes 0-1  velocity, int16, 0.001 rad/s
#       bytes 2-3  current,  uint16, 0.001 A
#       bytes 4-7  position, int32
#   0x15A..0x160  per-joint MIT command
FEEDBACK_HIGH_SPD = range(0x251, 0x258)
MIT_COMMAND = range(0x15A, 0x161)
SLL_OUTGOING = 4  # Linux cooked-capture packet type for a frame we sent

# The stop-relevant transmit frames.
#   0x150  motion ctrl — byte 0: 0x01 electronic emergency stop, 0x02 reset
#   0x151  mode ctrl   — byte 1: MOVE mode, byte 3: 0xAD MIT
#   0x155, 0x156, 0x157, 0x170  joint position control, the MOVE-J payload
MOTION_CTRL = 0x150
MODE_CTRL = 0x151
JOINT_CTRL = (0x155, 0x156, 0x157, 0x170)
ELECTRONIC_STOP = 0x01
MOTION_RESET = 0x02
# 0x04 on the default tier, 0x06 on v111. Neither may appear as a hold.
MIT_MOVE_MODES = (0x04, 0x06)
MOVE_J = 0x01


def frames(path):
    """Yield (timestamp, sll_pkttype, can_id, payload) from a LINKTYPE_LINUX_SLL pcap."""
    with open(path, "rb") as handle:
        header = handle.read(24)
        if len(header) < 24:
            raise ValueError(f"{path}: truncated pcap header")
        magic = struct.unpack("<I", header[:4])[0]
        if magic not in (0xA1B2C3D4, 0xA1B23C4D):
            raise ValueError(f"{path}: unsupported pcap magic 0x{magic:08x}")
        linktype = struct.unpack("<I", header[20:24])[0]
        if linktype != 113:
            raise ValueError(f"{path}: expected LINKTYPE_LINUX_SLL (113), got {linktype}")

        while True:
            record = handle.read(16)
            if len(record) < 16:
                return
            secs, usecs, incl, _orig = struct.unpack("<IIII", record)
            data = handle.read(incl)
            if len(data) < incl:
                return
            if incl < 32:
                continue  # too short to hold an SLL header plus a CAN frame
            pkttype = struct.unpack(">H", data[:2])[0]
            can = data[16:32]
            can_id = struct.unpack("<I", can[:4])[0] & 0x1FFFFFFF
            yield secs + usecs / 1e6, pkttype, can_id, can[8:16]


def analyse(path: str) -> None:
    stamps: list[float] = []
    ids: collections.Counter = collections.Counter()
    directions: collections.Counter = collections.Counter()
    joints: dict[int, list] = collections.defaultdict(list)

    for timestamp, pkttype, can_id, payload in frames(path):
        stamps.append(timestamp)
        ids[can_id] += 1
        directions[pkttype] += 1
        if can_id in FEEDBACK_HIGH_SPD and len(payload) == 8:
            velocity = struct.unpack(">h", payload[0:2])[0]
            position = struct.unpack(">i", payload[4:8])[0]
            joints[can_id - 0x250].append((timestamp, velocity, position))

    if len(stamps) < 2:
        print(f"{path}: fewer than two frames")
        return

    duration = stamps[-1] - stamps[0]
    tx = directions.get(SLL_OUTGOING, 0)
    rx = sum(directions.values()) - tx
    mit = sum(count for can_id, count in ids.items() if can_id in MIT_COMMAND)

    print(f"=== {path} ===")
    print(f"  {len(stamps)} frames over {duration:.2f}s = {len(stamps) / duration:.0f} f/s")
    print(f"  RX {rx / duration:.0f} f/s, TX {tx / duration:.0f} f/s, {len(ids)} distinct IDs")
    if mit:
        per_joint = mit / duration / len(MIT_COMMAND)
        print(f"  MIT command frames: {mit} = {mit / duration:.0f} f/s "
              f"({per_joint:.0f} Hz per joint)")

    if not joints:
        print("  no high-speed joint feedback in this capture")
        return

    print("  joint feedback — reported velocity vs velocity derived from the same frames:")
    for joint in sorted(joints):
        samples = joints[joint]
        derived = [
            abs(p1 - p0) / (t1 - t0)
            for (t0, _v0, p0), (t1, _v1, p1) in zip(samples, samples[1:])
            if t1 - t0 > 1e-4
        ]
        reported_peak = max(abs(v) for _t, v, _p in samples)
        span = max(p for _t, _v, p in samples) - min(p for _t, _v, p in samples)
        if not derived:
            continue
        verdict = "FLAT ZERO" if reported_peak == 0 else f"peak {reported_peak}"
        print(
            f"    joint {joint}: reported {verdict:>10}   "
            f"derived peak={max(derived):8.1f} mean={sum(derived) / len(derived):7.1f}   "
            f"position span={span}"
        )

    everything_flat = all(
        max(abs(v) for _t, v, _p in samples) <= 1 for samples in joints.values()
    )
    moved = any(
        (max(p for _t, _v, p in s) - min(p for _t, _v, p in s)) > 50
        for s in joints.values()
    )
    if everything_flat and moved:
        print("  VERDICT: joints moved while the velocity field stayed at 0 (+/-1).")
        print("           The firmware does not report usable velocity — deriving it")
        print("           from positions is the only available source, and removing")
        print("           the vendor's zeroing would expose nothing but zeros.")


def _move_mode_name(value: int) -> str:
    if value == MOVE_J:
        return "MOVE-J"
    if value in MIT_MOVE_MODES:
        return "MIT"
    return f"0x{value:02x}"


def stop_signature(path: str, stop_ts: float, window: float = 5.0) -> bool:
    """Report what the wire shows around a stop. Returns True when it is clean.

    Clean means: no vendor electronic stop, MIT command traffic ends, and a
    MOVE-J hold is commanded. The first of the three is the one that decides —
    the electronic stop is a damped descent, so a single such frame contradicts
    the whole point of the hold.
    """
    electronic: list[float] = []
    resets: list[float] = []
    modes_after: list[tuple[float, int]] = []
    joint_ctrl_after = 0
    mit_before = 0
    mit_after = 0
    first = last = None

    for timestamp, _pkttype, can_id, payload in frames(path):
        if first is None:
            first = timestamp
        last = timestamp
        after = timestamp >= stop_ts
        near = abs(timestamp - stop_ts) <= window
        if can_id == MOTION_CTRL and payload:
            if payload[0] == ELECTRONIC_STOP:
                electronic.append(timestamp)
            elif payload[0] == MOTION_RESET:
                resets.append(timestamp)
        elif can_id == MODE_CTRL and after and near and len(payload) >= 4:
            modes_after.append((timestamp, payload[1]))
        elif can_id in JOINT_CTRL and after and near:
            joint_ctrl_after += 1
        elif can_id in MIT_COMMAND:
            if after:
                mit_after += 1
            elif near:
                mit_before += 1

    print(f"=== stop signature: {path} ===")
    if first is None:
        print("  empty capture")
        return False
    if not (first <= stop_ts <= last):
        print(f"  WARNING: stop timestamp {stop_ts:.3f} is outside the capture "
              f"({first:.3f}..{last:.3f}) — the numbers below mean nothing")
        return False

    print(f"  MIT command frames: {mit_before} in the {window:.0f}s before the "
          f"stop, {mit_after} after it")
    print(f"  joint position control frames after the stop: {joint_ctrl_after}")
    if modes_after:
        seen = collections.Counter(_move_mode_name(mode) for _t, mode in modes_after)
        summary = ", ".join(f"{name} x{count}" for name, count in seen.items())
        print(f"  MOVE modes commanded after the stop: {summary}")
    else:
        print("  no mode frame after the stop")

    clean = True
    if electronic:
        offsets = ", ".join(f"{t - stop_ts:+.3f}s" for t in electronic[:5])
        print(f"  FAIL: {len(electronic)} electronic emergency stop frame(s) "
              f"(0x150 byte0=0x01) at {offsets}")
        print("        That command damps without stiffness — a raised arm "
              "descends. No safety path may issue it.")
        clean = False
    else:
        print("  PASS: no electronic emergency stop frame anywhere in the capture")
    if resets:
        print(f"  note: {len(resets)} motion reset frame(s) (0x150 byte0=0x02)")
    if mit_after:
        print(f"  FAIL: {mit_after} MIT command frames after the stop — the "
              "control stream did not end")
        clean = False
    if not joint_ctrl_after:
        print("  FAIL: no joint position control after the stop — no MOVE-J "
              "hold reached the arm")
        clean = False
    if any(mode in MIT_MOVE_MODES for _t, mode in modes_after):
        print("  FAIL: a MIT move mode was commanded after the stop")
        clean = False

    print(f"  VERDICT: {'clean' if clean else 'NOT clean'}")
    return clean


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyse SocketCAN pcaps captured from the Nero arms.",
    )
    parser.add_argument("pcaps", nargs="+", help="pcap files to read")
    parser.add_argument(
        "--stop-at", type=float, default=None, metavar="UNIX_TS",
        help="also report the emergency-stop signature around this instant",
    )
    parser.add_argument(
        "--window", type=float, default=5.0, metavar="SECONDS",
        help="how far either side of the stop to look (default 5)",
    )
    args = parser.parse_args()

    clean = True
    for path in args.pcaps:
        try:
            analyse(path)
            if args.stop_at is not None:
                clean &= stop_signature(path, args.stop_at, args.window)
        except (OSError, ValueError) as exc:
            print(f"{path}: {exc}", file=sys.stderr)
            return 1
    return 0 if clean else 3


if __name__ == "__main__":
    raise SystemExit(main())
