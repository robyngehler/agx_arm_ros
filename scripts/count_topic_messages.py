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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


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
    counter = {"n": 0}
    node.create_subscription(
        _import_message(args.message_type),
        args.topic,
        lambda _msg: counter.__setitem__("n", counter["n"] + 1),
        QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE),
    )

    deadline = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    count = counter["n"]
    label = f"{args.label}: " if args.label else ""
    print(
        f"{label}{count} messages on {args.topic} in {args.seconds:.0f}s "
        f"({count / args.seconds:.1f}/s)"
    )
    return 0 if count else 1


if __name__ == "__main__":
    sys.exit(main())
