#!/usr/bin/env python3
"""Count messages on one topic over a fixed window and print once at the end.

Written because `ros2 topic hz` was not a trustworthy witness for "did this
node stop publishing?": its output is block-buffered when redirected, so the
last seconds are lost when the process is killed, and a shell marker appended
to the same file has no defined position relative to those buffered flushes.
Twice during Phase 1A validation that produced a confident-looking zero that
was an artifact.

This subscribes, counts, and prints a single line at the end — nothing to
flush early, nothing to interleave.

    python3 scripts/count_topic_messages.py /control/move_mit \
        agx_arm_msgs/msg/MoveMITMsg --seconds 8

Exit status is 0 when messages arrived and 1 when none did, so it can gate a
check directly.
"""

from __future__ import annotations

import argparse
import importlib
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# How long to wait for the first message before giving up on the window.
DISCOVERY_TIMEOUT_S = 10.0


def _import_message(type_name: str):
    """Turn 'pkg/msg/Type' into the message class."""
    package, kind, name = type_name.split("/")
    module = importlib.import_module(f"{package}.{kind}")
    return getattr(module, name)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic")
    parser.add_argument("message_type", help="e.g. agx_arm_msgs/msg/MoveMITMsg")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--label", default="")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("topic_message_counter")
    # Timestamps, not a bare count: the rate is then taken over the span the
    # messages actually spanned, so neither discovery nor subscription matching
    # is charged to the publisher. Counting over a wall-clock window that starts
    # at rclpy.init() reported 168/s for a topic running at 197/s.
    stamps: list[float] = []
    node.create_subscription(
        _import_message(args.message_type),
        args.topic,
        lambda _msg: stamps.append(time.monotonic()),
        QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE),
    )

    # Spin properly, on its own thread. A `spin_once` loop handles ONE callback
    # per iteration and pays the wait-set cost every time, so above ~100 Hz it
    # cannot keep up and reports its own deafness as the publication rate: it
    # measured 91-172/s on a topic running at 197/s. That is the failure this
    # script exists to avoid, so it must not commit it.
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while not stamps and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(args.seconds)
    rclpy.shutdown()
    spinner.join(timeout=5.0)
    node.destroy_node()

    count = len(stamps)
    label = f"{args.label}: " if args.label else ""
    span = (stamps[-1] - stamps[0]) if count > 1 else 0.0
    if span <= 0.0:
        print(f"{label}{count} messages on {args.topic} (no measurable span)")
        return 0 if count else 1
    print(
        f"{label}{count} messages on {args.topic} over {span:.1f}s "
        f"({(count - 1) / span:.1f}/s)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
