"""Convert a recorded leader trajectory into catalogue ``waypoints``.

Closes the teach-loop gap noted in
``docs/control/bringups/teach_and_run.md`` (Step C): the
leader recorder saves a dense ``RecordedTrajectory`` JSON, but the coordinator's
``arm_executor`` replays a *catalogue* action's ``waypoints:`` (a short list of
``positions`` + ``time_from_start_sec``). This tool downsamples a recording to a
handful of timed waypoints and emits a ready-to-paste YAML block for the matching
``Trajectory`` action in ``agx_arm_coordination/config/catalogue.yaml``.

It deliberately does NOT rewrite ``catalogue.yaml`` in place: that file uses
flow-style ``metadata: { ... }`` with comments that a YAML round-trip would
clobber. Instead it writes a ``<action_id>.waypoints.yaml`` sidecar and prints
the block; paste it under the action's ``metadata`` (replacing/adding
``waypoints:``). ``arm_executor`` validates that each waypoint's length matches
the group's ``joint_names``, so the recorded joint order is echoed in a comment
for a quick cross-check against the ``both_arms`` / per-arm group order.

Example::

    ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue \\
        ~/agx_arm_trajectories/pour_profile_right.json \\
        --action-id both_arms_pour_profile_v1 --max-points 8
"""

from __future__ import annotations

import argparse
from pathlib import Path

from agx_arm_mit_controller.duo_trajectory import ArmSegment, merge_arm_recordings
from agx_arm_mit_controller.trajectory_io import (
    RecordedTrajectory,
    load_recorded_trajectory,
)


def build_duo_trajectory(
    left: RecordedTrajectory,
    right: RecordedTrajectory,
    *,
    name: str,
    left_prefix: str = "left_arm_",
    right_prefix: str = "right_arm_",
    rate_hz: float | None = None,
) -> RecordedTrajectory:
    """Merge a left + right recording into one ``both_arms`` (14-dim) trajectory.

    Output joint order is ``left_prefix``-joints then ``right_prefix``-joints,
    matching the motion registry's ``both_arms`` side order (``[left, right]``).
    The emitted waypoint block echoes this order so it can be cross-checked
    against the catalogue group's ``joint_names`` before pasting.
    """
    segments = [
        ArmSegment(left, tuple(f"{left_prefix}{joint}" for joint in left.joint_names)),
        ArmSegment(right, tuple(f"{right_prefix}{joint}" for joint in right.joint_names)),
    ]
    return merge_arm_recordings(segments, name=name, rate_hz=rate_hz, robot="duo")


def downsample_indices(count: int, max_points: int) -> list[int]:
    """Evenly spaced sample indices over ``range(count)``, keeping first+last."""
    if count <= 0:
        return []
    if max_points <= 0 or count <= max_points:
        return list(range(count))
    if max_points == 1:
        return [count - 1]
    step = (count - 1) / (max_points - 1)
    picked = sorted({round(i * step) for i in range(max_points)})
    # Rounding collisions can drop a point; backfill to keep endpoints + count.
    if picked[0] != 0:
        picked.insert(0, 0)
    if picked[-1] != count - 1:
        picked.append(count - 1)
    return picked


def recorded_to_waypoints(
    trajectory: RecordedTrajectory, max_points: int, decimals: int = 5
) -> list[dict]:
    """Downsample a recording to ``[{positions, time_from_start_sec}, ...]``."""
    points = trajectory.points
    waypoints: list[dict] = []
    for index in downsample_indices(len(points), max_points):
        point = points[index]
        waypoints.append(
            {
                "positions": [round(float(p), decimals) for p in point.positions],
                "time_from_start_sec": round(float(point.time_from_start), 3),
            }
        )
    return waypoints


def format_waypoints_block(
    action_id: str, trajectory: RecordedTrajectory, waypoints: list[dict], indent: int = 6
) -> str:
    """Render a paste-ready ``waypoints:`` block (block style, with provenance)."""
    pad = " " * indent
    item_pad = " " * (indent + 2)
    lines = [
        f"{pad}# waypoints generated from recording '{trajectory.name}' "
        f"({len(trajectory.points)} pts -> {len(waypoints)})",
        f"{pad}# recorded joint order: {list(trajectory.joint_names)}",
        f"{pad}# -> must match the catalogue group's joint_names for '{action_id}'",
        f"{pad}waypoints:",
    ]
    for wp in waypoints:
        positions = ", ".join(f"{v}" for v in wp["positions"])
        lines.append(f"{item_pad}- positions: [{positions}]")
        lines.append(f"{item_pad}  time_from_start_sec: {wp['time_from_start_sec']}")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a recorded leader trajectory into catalogue waypoints"
    )
    parser.add_argument("trajectory_path", help="Path to a RecordedTrajectory JSON file (the left arm when merging)")
    parser.add_argument(
        "--action-id", required=True, help="Catalogue action_id the waypoints belong to"
    )
    parser.add_argument(
        "--merge-with",
        default="",
        help="Second RecordedTrajectory (the right arm) to merge into one both_arms (14-dim) action",
    )
    parser.add_argument(
        "--left-prefix", default="left_arm_", help="Joint prefix for the first recording in the duo merge"
    )
    parser.add_argument(
        "--right-prefix", default="right_arm_", help="Joint prefix for the --merge-with recording in the duo merge"
    )
    parser.add_argument(
        "--merge-rate-hz",
        type=float,
        default=0.0,
        help="Common resample rate for the duo merge (0 = max of the two recordings' rates)",
    )
    parser.add_argument(
        "--max-points", type=int, default=8, help="Downsample to at most this many waypoints"
    )
    parser.add_argument("--decimals", type=int, default=5, help="Rounding for positions")
    parser.add_argument(
        "--out",
        default="",
        help="Sidecar YAML path to write (default: <trajectory dir>/<action_id>.waypoints.yaml)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trajectory_path = Path(args.trajectory_path).expanduser().resolve()
    if not trajectory_path.is_file():
        raise SystemExit(f"recording not found: {trajectory_path}")

    trajectory = load_recorded_trajectory(trajectory_path)
    if not trajectory.points:
        raise SystemExit(f"recording '{trajectory.name}' has no points")

    if args.merge_with:
        right_path = Path(args.merge_with).expanduser().resolve()
        if not right_path.is_file():
            raise SystemExit(f"--merge-with recording not found: {right_path}")
        right = load_recorded_trajectory(right_path)
        if not right.points:
            raise SystemExit(f"recording '{right.name}' has no points")
        trajectory = build_duo_trajectory(
            trajectory,
            right,
            name=args.action_id,
            left_prefix=args.left_prefix,
            right_prefix=args.right_prefix,
            rate_hz=args.merge_rate_hz or None,
        )
        print(
            f"merged '{Path(args.trajectory_path).name}' + '{right_path.name}' -> "
            f"{len(trajectory.joint_names)}-dim both_arms trajectory "
            f"({trajectory.duration:.2f}s @ {trajectory.sample_rate_hz:.0f} Hz)"
        )

    waypoints = recorded_to_waypoints(trajectory, args.max_points, args.decimals)
    block = format_waypoints_block(args.action_id, trajectory, waypoints)

    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else trajectory_path.with_name(f"{args.action_id}.waypoints.yaml")
    )
    out_path.write_text(block, encoding="utf-8")

    print(block)
    print(f"wrote {out_path}")
    print(
        f"paste the 'waypoints:' block under action '{args.action_id}' metadata in "
        "agx_arm_coordination/config/catalogue.yaml, then rebuild the package"
    )


__all__ = [
    "downsample_indices",
    "recorded_to_waypoints",
    "format_waypoints_block",
    "build_duo_trajectory",
    "main",
]
