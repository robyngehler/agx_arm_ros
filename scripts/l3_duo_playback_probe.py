#!/usr/bin/env python3
"""Replay a duo teach recording and measure what the two arms actually got.

Answers three questions a playback cannot answer from the outside:

* **Did both arms start together?** Read from the controllers' own
  ``Accepted debug trajectory`` lines on ``/rosout``. A duo playback that
  dispatches per arm with a sleep in between put 671 ms between them.
* **Was the trajectory started once?** Each publish restarts execution at
  t=0, so more than one acceptance per arm means the arm replayed its start.
* **Did the control loop hold its rate?** Counts ``control/move_mit`` per arm
  during execution and reports the gap distribution. A configured 200 Hz that
  lands at 90 Hz with 30 ms gaps is what a lagging, sample-skipping arm looks
  like from here.

The replay itself mirrors the teach manager: same loader, same smoothing, both
slices built before either is published, one publish each. A lead-in is used by
default so the arms blend from wherever they are into the first recorded point.

**This moves both arms.** Bring them to the recording's start anchor first.

Usage:
  python3 scripts/l3_duo_playback_probe.py ~/agx_arm_trajectories/teach/Wave_Both_V01.json
  python3 scripts/l3_duo_playback_probe.py <file.json> --lead-in-sec 3.0 --dry-run
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.msg import MoveMITMsg
from agx_arm_mit_controller.trajectory_io import (
    load_recorded_trajectory,
    recorded_to_joint_trajectory,
    smooth_recorded_trajectory_seconds,
)

ARM_JOINTS = [f"joint{index}" for index in range(1, 8)]
ACCEPT_MARKER = "Accepted debug trajectory"


class PlaybackProbe(Node):
    def __init__(self, sides: list[str]) -> None:
        super().__init__("l3_duo_playback_probe")
        self.sides = sides
        self.cmd_stamps: dict[str, list[float]] = {side: [] for side in sides}
        self.feedback: dict[str, JointState] = {}
        self.accepts: list[tuple[float, str, str]] = []
        self._collecting = False

        self.trajectory_pubs = {}
        for side in sides:
            self.trajectory_pubs[side] = self.create_publisher(
                JointTrajectory, f"/{side}/mit_controller/joint_trajectory", 10
            )
            self.create_subscription(
                MoveMITMsg, f"/{side}/control/move_mit",
                self._make_cmd_cb(side), 50,
            )
            self.create_subscription(
                JointState, f"/{side}/feedback/joint_states",
                self._make_fb_cb(side), 20,
            )
        self.create_subscription(Log, "/rosout", self._on_log, 100)

    def _make_cmd_cb(self, side: str):
        def callback(_msg) -> None:
            if self._collecting:
                self.cmd_stamps[side].append(time.monotonic())
        return callback

    def _make_fb_cb(self, side: str):
        def callback(msg: JointState) -> None:
            self.feedback[side] = msg
        return callback

    def _on_log(self, msg: Log) -> None:
        if ACCEPT_MARKER in msg.msg and "mit_controller" in msg.name:
            self.accepts.append((time.monotonic(), msg.name, msg.msg))

    def start_collecting(self) -> None:
        for stamps in self.cmd_stamps.values():
            stamps.clear()
        self.accepts.clear()
        self._collecting = True

    def stop_collecting(self) -> None:
        self._collecting = False

    def wait_for_feedback(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(side in self.feedback for side in self.sides):
                return True
            time.sleep(0.05)
        return False

    def current_positions(self, joint_names: list[str]) -> list[float]:
        """Live positions in the recording's column order (side-prefixed names)."""
        merged: dict[str, float] = {}
        for side, msg in self.feedback.items():
            prefix = f"{side}_"
            for name, position in zip(msg.name, msg.position):
                merged[name if name.startswith(prefix) else prefix + name] = float(position)
        missing = [name for name in joint_names if name not in merged]
        if missing:
            raise RuntimeError(f"feedback does not cover {missing}")
        return [merged[name] for name in joint_names]


def arm_slice(side: str, full: JointTrajectory, columns: list[int]) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(ARM_JOINTS)
    for point in full.points:
        sliced = JointTrajectoryPoint()
        sliced.positions = [float(point.positions[i]) for i in columns]
        sliced.velocities = [float(point.velocities[i]) for i in columns] if point.velocities else []
        sliced.time_from_start = point.time_from_start
        msg.points.append(sliced)
    return msg


def gaps_ms(stamps: list[float]) -> list[float]:
    return [(b - a) * 1e3 for a, b in zip(stamps, stamps[1:])]


def report(side: str, stamps: list[float], window_s: float) -> None:
    if len(stamps) < 3:
        print(f"  {side:<10} {len(stamps)} commands — too few to characterise")
        return
    gaps = gaps_ms(stamps)
    gaps_sorted = sorted(gaps)
    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span if span > 0 else 0.0
    over_10 = sum(1 for gap in gaps if gap > 10.0)
    over_20 = sum(1 for gap in gaps if gap > 20.0)
    print(
        f"  {side:<10} {len(stamps):>5} cmds  {rate:>6.1f}/s   "
        f"gap p50 {statistics.median(gaps):>5.1f}  "
        f"p95 {gaps_sorted[int(0.95 * len(gaps_sorted))]:>5.1f}  "
        f"p99 {gaps_sorted[int(0.99 * len(gaps_sorted))]:>5.1f}  "
        f"max {max(gaps):>6.1f} ms   "
        f">10ms {over_10:>4} ({100.0 * over_10 / len(gaps):.1f}%)  >20ms {over_20}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("recording", type=Path)
    parser.add_argument("--sides", default="left_arm,right_arm")
    parser.add_argument("--smoothing-sec", type=float, default=0.30)
    parser.add_argument("--lead-in-sec", type=float, default=2.0)
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--tail-sec", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="prepare and report, publish nothing")
    args = parser.parse_args()

    sides = [side.strip() for side in args.sides.split(",") if side.strip()]
    trajectory = load_recorded_trajectory(args.recording)
    trajectory = smooth_recorded_trajectory_seconds(trajectory, args.smoothing_sec)
    names = list(trajectory.joint_names)
    columns = {}
    for side in sides:
        wanted = [f"{side}_{joint}" for joint in ARM_JOINTS]
        if not all(name in names for name in wanted):
            print(f"recording does not carry {side} columns ({wanted[0]} ...)", file=sys.stderr)
            return 2
        columns[side] = [names.index(name) for name in wanted]

    rclpy.init()
    node = PlaybackProbe(sides)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        if not node.wait_for_feedback(10.0):
            print("no feedback from every side", file=sys.stderr)
            return 2

        full = recorded_to_joint_trajectory(
            trajectory,
            time_scale=1.0 / args.speed_scale,
            current_positions=node.current_positions(names) if args.lead_in_sec > 0 else None,
            lead_in_sec=args.lead_in_sec,
        )
        slices = [(side, arm_slice(side, full, columns[side])) for side in sides]
        duration = full.points[-1].time_from_start.sec + \
            full.points[-1].time_from_start.nanosec / 1e9
        print(f"recording : {args.recording.name}, {len(trajectory.points)} points, "
              f"{duration:.2f}s after lead-in")
        if args.dry_run:
            print("DRY RUN — nothing published")
            return 0

        node.start_collecting()
        published = time.monotonic()
        for side, msg in slices:
            node.trajectory_pubs[side].publish(msg)
        print(f"published both slices in {(time.monotonic() - published) * 1e3:.2f} ms; "
              f"executing for {duration:.1f}s ...")
        time.sleep(duration + args.tail_sec)
        node.stop_collecting()

        print()
        print("=== trajectory acceptance (from the controllers' own logs) ===")
        if not node.accepts:
            print("  no acceptance logged — did the controllers take the trajectory?")
        first_by_node: dict[str, float] = {}
        for stamp, name, text in node.accepts:
            count = sum(1 for _s, n, _t in node.accepts if n == name)
            first_by_node.setdefault(name, stamp)
            print(f"  {(stamp - published) * 1e3:+9.1f} ms  {name}")
        for name, count in sorted(
            (n, sum(1 for _s, m, _t in node.accepts if m == n))
            for n in {n for _s, n, _t in node.accepts}
        ):
            verdict = "OK" if count == 1 else f"RESTARTED {count - 1}x"
            print(f"  {name}: {count} acceptance(s) — {verdict}")
        if len(first_by_node) == 2:
            offset = abs(max(first_by_node.values()) - min(first_by_node.values()))
            print(f"  duo start offset: {offset * 1e3:.1f} ms")

        print()
        print("=== control loop during execution ===")
        for side in sides:
            report(side, node.cmd_stamps[side], duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=5.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
