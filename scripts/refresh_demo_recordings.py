#!/usr/bin/env python3
"""Regenerate a demo's recording sidecars from the teach library.

A catalogue action references ``config/recordings/<name>.json``, a lean sidecar
(joint names, times, positions) carrying the full taught density. That file is a
copy, so re-teaching a take on hardware does **not** update it — the activity
keeps replaying the older motion, and nothing says so. This closes that gap:

    scripts/refresh_demo_recordings.py --check     # report what is stale, exit 1
    scripts/refresh_demo_recordings.py             # rewrite the stale ones

The side prefix comes from the action's ``robot_id``, so a sidecar always states
which arm it was taught on and the planner checks that against the group instead
of taking the catalogue's word for it. A ``both_arms`` recording is already
prefixed by the duo merge and is copied as it is.

Run it from the workspace root. Needs no ROS environment.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONFIG = REPO / "src/agx_arm_coordination/config"
DEFAULT_TEACH_DIR = Path.home() / "agx_arm_trajectories" / "teach"

sys.path.insert(0, str(REPO / "src/agx_arm_mit_demos"))
sys.path.insert(0, str(REPO / "src/agx_arm_mit_controller"))


def references() -> list[tuple[str, Path, str]]:
    """Every ``recording:`` reference in the catalogue: action, sidecar, prefix."""
    import yaml

    prefixes = {"left_arm": "left_arm_", "right_arm": "right_arm_", "both_arms": ""}
    found = []
    sources = [CONFIG / "catalogue.yaml", *sorted((CONFIG / "catalogue.d").glob("*.yaml"))]
    for path in sources:
        actions = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("actions") or {}
        for action_id, spec in actions.items():
            reference = ((spec or {}).get("metadata") or {}).get("recording")
            if not reference:
                continue
            robot_id = str((spec or {}).get("robot_id", ""))
            if robot_id not in prefixes:
                raise SystemExit(f"{action_id}: cannot derive a side from robot_id '{robot_id}'")
            found.append((action_id, CONFIG / reference, prefixes[robot_id]))
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--teach-dir", type=Path, default=DEFAULT_TEACH_DIR,
        help=f"Teach library the sidecars are generated from (default {DEFAULT_TEACH_DIR})",
    )
    parser.add_argument(
        "--check", action="store_true",
        help="Report which sidecars differ from the teach library and exit 1; write nothing.",
    )
    parser.add_argument("--decimals", type=int, default=5, help="Rounding for positions")
    args = parser.parse_args()

    from agx_arm_mit_controller.trajectory_io import load_recorded_trajectory
    from agx_arm_mit_demos.recorded_to_catalogue import recording_sidecar

    stale = 0
    for action_id, sidecar, prefix in references():
        source = args.teach_dir / sidecar.name
        if not source.is_file():
            print(f"MISSING {action_id}: no teach recording at {source}")
            stale += 1
            continue
        fresh = recording_sidecar(
            load_recorded_trajectory(source), args.decimals, prefix
        )
        current = json.loads(sidecar.read_text()) if sidecar.is_file() else None
        if current is not None and current.get("positions") == fresh["positions"]:
            print(f"ok      {action_id}: {len(fresh['times'])} pts, {fresh['times'][-1]:.2f} s")
            continue
        stale += 1
        was = len(current["positions"]) if current else 0
        verb = "STALE  " if args.check else "UPDATED"
        print(f"{verb} {action_id}: {was} -> {len(fresh['times'])} pts, "
              f"{fresh['times'][-1]:.2f} s (taught {fresh['recorded_at']})")
        if not args.check:
            sidecar.parent.mkdir(parents=True, exist_ok=True)
            sidecar.write_text(json.dumps(fresh, separators=(",", ":")), encoding="utf-8")

    if args.check and stale:
        print(f"\n{stale} sidecar(s) out of date; run without --check, then rebuild the package.")
        return 1
    if stale:
        print(f"\n{stale} sidecar(s) rewritten. Rebuild agx_arm_coordination before running.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
