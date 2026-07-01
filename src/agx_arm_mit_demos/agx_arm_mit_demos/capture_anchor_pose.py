"""Capture the current arm joint configuration as a named anchor pose.

Companion to ``leader_trajectory_recorder`` (full trajectories) for the
single-pose half of the teach loop: drive the arm where you want it (freedrive /
leader mode, or any other way), then snapshot the live joint vector and store it
as a named anchor pose that the coordinator's ``arm_executor`` reads from
``agx_arm_coordination/config/arm_config.yaml`` (``arm_executor.poses``).

It only writes the single ``poses:`` line for the captured name, so the rest of
the config (groups, action servers, documentation comments) is preserved. After
capturing, rebuild ``agx_arm_coordination`` (or use a symlink-install) so a
launched coordinator picks up the new value.

Example (right arm, 7 DoF):

    ros2 run agx_arm_mit_demos agx_arm_capture_anchor_pose \\
        --pose-name Pre_Grip_R \\
        --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7 \\
        --config src/agx_arm_coordination/config/arm_config.yaml

The captured vector order matches ``--source-joints``; align that with the
target group's ``joint_names`` (e.g. the ``_R`` half of ``both_arms``).
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def average_joint_positions(node, latest_getter, want, settle_sec, timeout_sec) -> dict[str, float]:
    """Average each requested joint over a short settle window once data arrives.

    Spins ``node``; ``latest_getter()`` returns the most recent ``JointState`` (or
    ``None``). This is the single implementation shared by ``capture_anchor_pose``
    and the teach manager, so anchor-pose capture behaves identically in both.
    """
    deadline = time.monotonic() + timeout_sec
    sums: dict[str, float] = {name: 0.0 for name in want}
    counts: dict[str, int] = {name: 0 for name in want}
    seen_names: set[str] = set()
    settle_end: Optional[float] = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        msg = latest_getter()
        if msg is None:
            continue
        seen_names.update(msg.name)
        if settle_end is None:
            settle_end = time.monotonic() + settle_sec
        pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
        for name in want:
            if name in pos:
                sums[name] += pos[name]
                counts[name] += 1
        if settle_end is not None and time.monotonic() >= settle_end:
            break
    missing = [n for n in want if counts[n] == 0]
    if missing:
        raise RuntimeError(
            f"joints not found on the source topic: {missing}\n"
            f"joint names seen on the topic: {sorted(seen_names) or '(none received)'}"
        )
    return {name: sums[name] / counts[name] for name in want}


class _JointStateCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("capture_anchor_pose")
        self.latest: Optional[JointState] = None
        self.create_subscription(JointState, topic, self._cb, 20)

    def _cb(self, msg: JointState) -> None:
        self.latest = msg

    def collect(self, want: list[str], settle_sec: float, timeout_sec: float) -> dict[str, float]:
        """Average each requested joint over a short settle window."""
        return average_joint_positions(self, lambda: self.latest, want, settle_sec, timeout_sec)


def _format_vector(values: list[float], precision: int) -> str:
    return "[" + ", ".join(f"{v:.{precision}f}" for v in values) + "]"


def update_pose_in_config(config_path: Path, pose_name: str, vector: list[float], precision: int) -> str:
    """Replace (or insert) a single anchor pose line under ``poses:``.

    Returns a short human-readable note about what changed. Only the one pose
    line is touched; comments and the rest of the file are preserved.
    """
    text = config_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    formatted = _format_vector(vector, precision)

    # Existing entry: "<indent><pose_name>: <anything>"
    entry_re = re.compile(rf"^(\s+){re.escape(pose_name)}:\s*.*$")
    for i, line in enumerate(lines):
        if entry_re.match(line):
            indent = entry_re.match(line).group(1)
            lines[i] = f"{indent}{pose_name}: {formatted}"
            config_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return f"updated existing pose '{pose_name}'"

    # Insert under the 'poses:' key, matching the indentation of existing entries.
    poses_idx = next(
        (i for i, line in enumerate(lines) if re.match(r"^\s*poses:\s*$", line)), None
    )
    if poses_idx is None:
        raise RuntimeError(
            f"could not find a 'poses:' block in {config_path}; add one under arm_executor:"
        )
    # Indentation: reuse the next non-empty child line's indent, else poses+2.
    child_indent = "    "
    for line in lines[poses_idx + 1 :]:
        if line.strip():
            m = re.match(r"^(\s+)\S", line)
            if m and len(m.group(1)) > (len(lines[poses_idx]) - len(lines[poses_idx].lstrip())):
                child_indent = m.group(1)
            break
    lines.insert(poses_idx + 1, f"{child_indent}{pose_name}: {formatted}")
    config_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return f"inserted new pose '{pose_name}'"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture current arm joints as a named anchor pose")
    parser.add_argument("--pose-name", required=True, help="Anchor pose name to write (e.g. Pre_Grip_R)")
    parser.add_argument(
        "--source-joints",
        required=True,
        help="Comma-separated joint names to capture, in stored order (e.g. joint1,...,joint7)",
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Path to arm_config.yaml whose arm_executor.poses block is updated",
    )
    parser.add_argument("--source-topic", default="feedback/joint_states", help="JointState topic to read")
    parser.add_argument("--settle-sec", type=float, default=0.5, help="Averaging window once data arrives")
    parser.add_argument("--timeout-sec", type=float, default=10.0, help="Give up if no data within this time")
    parser.add_argument("--precision", type=int, default=5, help="Decimal places stored per joint")
    parser.add_argument("--dry-run", action="store_true", help="Print the captured vector but do not write")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    want = [j.strip() for j in args.source_joints.split(",") if j.strip()]
    if not want:
        raise SystemExit("--source-joints is empty")
    config_path = Path(args.config).expanduser().resolve()
    if not config_path.is_file():
        raise SystemExit(f"config not found: {config_path}")

    rclpy.init()
    node = _JointStateCapture(args.source_topic)
    try:
        averaged = node.collect(want, args.settle_sec, args.timeout_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    vector = [averaged[name] for name in want]
    print(f"captured {args.pose_name}: {_format_vector(vector, args.precision)}")
    print(f"  joints: {want}")
    if args.dry_run:
        print("dry-run: not writing")
        return
    note = update_pose_in_config(config_path, args.pose_name, vector, args.precision)
    print(f"{note} in {config_path}")
    print("rebuild agx_arm_coordination (or symlink-install) for a launched coordinator to use it")


__all__ = ["main", "update_pose_in_config", "average_joint_positions"]
