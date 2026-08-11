#!/usr/bin/env python3
"""Analyse a SocketCAN pcap: frame rates, direction split, and Nero feedback fields.

Written for the Phase 0 baseline, and kept because the same numbers are what the
post-refactor comparison needs. Reads the pcap directly — no tshark on the
Jetson — and understands the Nero high-speed feedback layout, so it can answer
the question that motivated it: does the firmware report joint velocity at all?

Usage:
    ./scripts/analyze_can_pcap.py <file.pcap> [more.pcap ...]

Reference: docs/sprint_refactor/reference/phase0_baseline.md
"""

from __future__ import annotations

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


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for path in sys.argv[1:]:
        try:
            analyse(path)
        except (OSError, ValueError) as exc:
            print(f"{path}: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
