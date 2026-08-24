#!/usr/bin/env python3
"""Drop the repeated samples a teach recording carries, in place.

A recording samples the feedback cache on a fixed clock, so wherever the clock
is faster than new data arrives it stores the previous sample again. Those rows
carry no information and actively harm what reads them: a finite difference
across a repeat alternates between zero and twice the true value, and a spline
fitted through them reproduces that as acceleration noise.

Removing them makes the sampling non-uniform, which every consumer in this
repository already handles -- the trajectory buffer interpolates on
``time_from_start`` and the retiming pipeline fits over recorded time.

Velocities are recomputed as central differences over the surviving, unevenly
spaced samples. Positions, times, efforts and flange poses are untouched, so the
path is exactly the one that was taught.

    python3 scripts/clean_recording.py ~/agx_arm_trajectories/teach/*.json

The original is moved to a ``raw/`` subdirectory next to the file before the
cleaned version is written, so nothing is destroyed.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

# Two samples count as the same when every joint is identical to this tolerance.
# 0.0 keeps only byte-identical repeats, which is what a stale cache produces;
# a small positive value also drops encoder dither.
DEFAULT_TOLERANCE = 0.0


def deduplicate(points: list[dict], tolerance: float) -> list[dict]:
    """Keep the first sample and every later one that actually moved."""
    if not points:
        return []
    kept = [points[0]]
    for point in points[1:]:
        previous = kept[-1]["positions"]
        current = point["positions"]
        if max(abs(a - b) for a, b in zip(current, previous)) > tolerance:
            kept.append(point)
    # The final pose is where the motion ended; dropping it as a repeat would
    # shorten the trajectory and leave the arm short of its taught end.
    if kept[-1] is not points[-1]:
        kept.append(points[-1])
    return kept


def recompute_velocities(points: list[dict]) -> None:
    """Central differences over unevenly spaced samples, endpoints at rest."""
    count = len(points)
    width = len(points[0]["positions"])
    for index, point in enumerate(points):
        if index == 0 or index == count - 1:
            point["velocities"] = [0.0] * width
            continue
        before = points[index - 1]
        after = points[index + 1]
        span = after["time_from_start"] - before["time_from_start"]
        if span <= 0.0:
            point["velocities"] = [0.0] * width
            continue
        point["velocities"] = [
            (after["positions"][j] - before["positions"][j]) / span for j in range(width)
        ]


def clean_file(path: Path, tolerance: float, dry_run: bool) -> None:
    payload = json.loads(path.read_text(encoding="utf-8"))
    points = payload.get("points", [])
    if len(points) < 2:
        print(f"{path.name}: fewer than two samples, skipped")
        return

    kept = deduplicate(points, tolerance)
    duration = points[-1]["time_from_start"] - points[0]["time_from_start"]
    removed = len(points) - len(kept)
    effective = (len(kept) - 1) / duration if duration > 0 else 0.0

    print(
        f"{path.name}: {len(points)} -> {len(kept)} samples "
        f"({removed} repeats, {100.0 * removed / len(points):.1f}%), "
        f"{payload.get('sample_rate_hz', 0.0):.0f} Hz nominal -> {effective:.1f} Hz of real content"
    )
    if dry_run:
        return

    recompute_velocities(kept)
    payload["points"] = kept
    # The stored rate is what a reader believes about the data, so it has to
    # become what survived rather than what the clock was set to.
    payload["sample_rate_hz"] = round(effective, 3)
    metadata = payload.setdefault("metadata", {})
    metadata["cleaned"] = {
        "removed_repeats": removed,
        "original_samples": len(points),
        "original_sample_rate_hz": json.loads(path.read_text(encoding="utf-8")).get("sample_rate_hz"),
        "tolerance_rad": tolerance,
        "velocities": "recomputed as central differences over the surviving samples",
    }

    backup_dir = path.parent / "raw"
    backup_dir.mkdir(exist_ok=True)
    shutil.copy2(path, backup_dir / path.name)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(f"    original kept at {backup_dir / path.name}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                        help="Joint delta below which a sample counts as a repeat (rad)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Report what would be removed and change nothing")
    args = parser.parse_args(argv)

    for path in args.paths:
        if not path.is_file():
            print(f"{path}: not a file", file=sys.stderr)
            return 1
        clean_file(path, args.tolerance, args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
