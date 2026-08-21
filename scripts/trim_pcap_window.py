#!/usr/bin/env python3
"""Cut a SocketCAN pcap down to a time window, so evidence can be committed.

A stop capture is minutes of healthy traffic around the few milliseconds that
decide anything. Keeping the whole file in git costs tens of megabytes forever;
keeping a window around the stop preserves every frame
``analyze_can_pcap.py --stop-at`` reads, at a fraction of the size.

The global header is copied byte for byte, so the result stays the same
link type and timestamp resolution as the input.

Usage:
    ./scripts/trim_pcap_window.py --at <unix_ts> [--before 2] [--after 2] \\
        --out-dir docs/sprint6/evidence/<run> <file.pcap> ...

Reference: docs/sprint_refactor/reference/emergency_stop_ladder.md
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys

MICROSECOND_MAGICS = (0xA1B2C3D4,)
NANOSECOND_MAGICS = (0xA1B23C4D,)


def trim(src: pathlib.Path, dst: pathlib.Path, start: float, end: float) -> tuple:
    """Copy records inside [start, end]. Returns (kept, total, out_bytes)."""
    with src.open("rb") as handle:
        header = handle.read(24)
        if len(header) < 24:
            raise ValueError(f"{src}: truncated pcap header")
        magic = struct.unpack("<I", header[:4])[0]
        if magic in MICROSECOND_MAGICS:
            divisor = 1e6
        elif magic in NANOSECOND_MAGICS:
            divisor = 1e9
        else:
            raise ValueError(f"{src}: unsupported pcap magic 0x{magic:08x}")

        kept = total = 0
        chunks = [header]
        while True:
            record = handle.read(16)
            if len(record) < 16:
                break
            secs, fraction, incl, _orig = struct.unpack("<IIII", record)
            data = handle.read(incl)
            if len(data) < incl:
                break
            total += 1
            timestamp = secs + fraction / divisor
            if start <= timestamp <= end:
                chunks.append(record)
                chunks.append(data)
                kept += 1

    payload = b"".join(chunks)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(payload)
    return kept, total, len(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("pcaps", nargs="+", type=pathlib.Path)
    parser.add_argument("--at", type=float, required=True, metavar="UNIX_TS",
                        help="the instant to centre the window on")
    parser.add_argument("--before", type=float, default=2.0, metavar="SECONDS")
    parser.add_argument("--after", type=float, default=2.0, metavar="SECONDS")
    parser.add_argument("--out-dir", type=pathlib.Path, required=True)
    args = parser.parse_args()

    start, end = args.at - args.before, args.at + args.after
    for src in args.pcaps:
        dst = args.out_dir / src.name
        if dst.resolve() == src.resolve():
            print(f"{src}: refusing to trim a file onto itself", file=sys.stderr)
            return 2
        try:
            kept, total, size = trim(src, dst, start, end)
        except (OSError, ValueError) as exc:
            print(f"{src}: {exc}", file=sys.stderr)
            return 1
        print(f"  {dst}: {kept}/{total} frames, {size / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
