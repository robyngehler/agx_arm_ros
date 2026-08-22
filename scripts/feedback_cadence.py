#!/usr/bin/env python3
"""Separate the cadence of a feedback SOURCE from its TRANSPORT.

Written because a message count cannot tell "the publisher is slow" from "the
data is not changing". `feedback/joint_states` carries the arm's frame
timestamp in its header, not the instant the driver read it, so one
subscription answers three questions a count conflates:

  * how fast distinct frames arrive   (header-stamp gaps, duplicate stamps)
  * how fast the driver publishes     (receive-time gaps)
  * whether the publication loop      (receive gaps quantized to the
    aliases                            publication period)

That distinction is what showed a 200 Hz recording carrying ~100 Hz of real
content: the acquisition loop ran at ~180 Hz while the frames it read updated
at ~100 Hz (`docs/sprint_refactor/reference/feedback_rate_budget.md`).

    python3 scripts/feedback_cadence.py /right_arm/feedback/joint_states \
        --seconds 20 --label right

Prints one block at the end -- nothing to flush early, nothing to interleave.
For the rate below the SDK use `candump` on the raw socket instead, and take TX
from /sys/class/net/<iface>/statistics/tx_packets, which candump does not show.

Exit status is 1 when no messages arrived, so it can gate a check directly.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# How long to wait for the first message before giving up on the window.
DISCOVERY_TIMEOUT_S = 10.0
GAP_EDGES_MS = [3, 4, 5, 6, 7, 8, 10, 12, 16, 25]


def histogram(gaps_ms, edges):
    """Bucket counts for the given upper edges, plus an overflow bucket."""
    counts = [0] * (len(edges) + 1)
    for gap in gaps_ms:
        for index, edge in enumerate(edges):
            if gap < edge:
                counts[index] += 1
                break
        else:
            counts[-1] += 1
    return counts


def render_histogram(title, gaps_ms, edges):
    if not gaps_ms:
        print(f"  {title}: no samples")
        return
    counts = histogram(gaps_ms, edges)
    total = len(gaps_ms)
    labels = [f"<{edge:g}ms" for edge in edges] + [f">={edges[-1]:g}ms"]
    print(f"  {title}  (n={total}, "
          f"median {statistics.median(gaps_ms):.2f}ms, "
          f"mean {statistics.fmean(gaps_ms):.2f}ms)")
    for label, count in zip(labels, counts):
        if not count:
            continue
        share = 100.0 * count / total
        print(f"    {label:>9} {count:6d}  {share:5.1f}%  "
              + "#" * int(round(share / 2.0)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--label", default="")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("feedback_cadence_probe")
    recv: list[float] = []
    stamps: list[float] = []
    positions: list[list[float]] = []

    def on_msg(msg: JointState) -> None:
        recv.append(time.monotonic())
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        positions.append(list(msg.position))

    node.create_subscription(
        JointState, args.topic, on_msg,
        QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE),
    )

    # rclpy.spin on a thread, not a spin_once loop: a spin_once loop charges its
    # own scheduling to the publisher and reported 121 Hz for a 198 Hz topic.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while not recv and time.monotonic() < deadline:
        time.sleep(0.01)
    if not recv:
        print(f"{args.label or args.topic}: no messages within "
              f"{DISCOVERY_TIMEOUT_S:.0f}s")
        rclpy.shutdown()
        return 1

    time.sleep(args.seconds)
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    # Rate over the span the messages actually covered, so neither discovery nor
    # subscription matching is charged to the publisher.
    span = recv[-1] - recv[0]
    count = len(recv)
    recv_gaps = [1000.0 * (b - a) for a, b in zip(recv, recv[1:])]
    stamp_gaps = [1000.0 * (b - a) for a, b in zip(stamps, stamps[1:])]

    duplicate_stamps = sum(1 for gap in stamp_gaps if gap <= 0.0)
    distinct_stamps = len(set(stamps))
    identical_rows = sum(1 for a, b in zip(positions, positions[1:]) if a == b)

    print(f"\n=== {args.label or args.topic} ===")
    print(f"  {count} messages over {span:.1f}s -> {count / span:.1f}/s published")
    print(f"  distinct source frames:   {distinct_stamps} "
          f"-> {distinct_stamps / span:.1f}/s at the source")
    print(f"  republished same frame:   {duplicate_stamps} "
          f"({100.0 * duplicate_stamps / max(1, len(stamp_gaps)):.1f}% of gaps)")
    # Only meaningful while the arm is moving: a holding arm repeats positions
    # for real, and that is not evidence about the rate.
    print(f"  identical position rows:  {identical_rows} "
          f"({100.0 * identical_rows / max(1, count - 1):.1f}%)")
    render_histogram("receive gaps ", recv_gaps, GAP_EDGES_MS)
    render_histogram("frame  gaps  ", [g for g in stamp_gaps if g > 0.0],
                     GAP_EDGES_MS)
    return 0


if __name__ == "__main__":
    sys.exit(main())
