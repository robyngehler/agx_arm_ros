#!/usr/bin/env python3
"""Drop the repeated samples an existing teach recording carries, in place.

New recordings are de-duplicated before they are saved. This is for files taken
before that, or for re-running with a different tolerance. The logic is the
shared one in ``agx_arm_mit_controller.trajectory_io`` -- the reasoning for it
lives in that docstring, not here.

    python3 scripts/clean_recording.py ~/agx_arm_trajectories/teach/*.json

The original is copied into a ``raw/`` subdirectory before the cleaned version
is written, so nothing is destroyed.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from agx_arm_mit_controller.trajectory_io import (
    deduplicate_recorded_trajectory,
    load_recorded_trajectory,
    save_recorded_trajectory,
)


def clean_file(path: Path, tolerance: float, dry_run: bool) -> None:
    trajectory = load_recorded_trajectory(path)
    original_samples = len(trajectory.points)
    cleaned, removed = deduplicate_recorded_trajectory(trajectory, tolerance)

    if not removed:
        print(f"{path.name}: {original_samples} samples, no repeats")
        return

    print(
        f"{path.name}: {original_samples} -> {len(cleaned.points)} samples "
        f"({removed} repeats, {100.0 * removed / original_samples:.1f}%), "
        f"{trajectory.sample_rate_hz:.0f} Hz nominal -> "
        f"{cleaned.sample_rate_hz:.1f} Hz of real content"
    )
    if dry_run:
        return

    backup_dir = path.parent / "raw"
    backup_dir.mkdir(exist_ok=True)
    shutil.copy2(path, backup_dir / path.name)
    save_recorded_trajectory(cleaned, path)
    print(f"    original kept at {backup_dir / path.name}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.0,
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
