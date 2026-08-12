#!/usr/bin/env python3
"""Send two overlapping activity goals from one process, and report both.

The L2 exclusivity check cannot be done with two ``run_activity`` processes:
the mock activity finishes in well under a second, and process startup and
discovery jitter is larger than that, so the second client's goal lands after
the first activity has already completed and nothing overlaps.

From one process the second goal is sent as soon as the first is accepted —
microseconds, not seconds — so the overlap is real. The result is printed as one
JSON object on stdout; the test does the asserting.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from agx_arm_msgs.action import PerformActivity


class DoubleGoalProbe(Node):
    def __init__(self, server: str) -> None:
        super().__init__("l2_double_goal_probe")
        self._client = ActionClient(self, PerformActivity, server)

    def _spin_until(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _send(self, activity_id: str, timeout_sec: float):
        goal = PerformActivity.Goal()
        goal.activity_id = activity_id
        future = self._client.send_goal_async(goal)
        if not self._spin_until(future, timeout_sec):
            return None
        return future.result()

    def _collect(self, handle, timeout_sec: float) -> dict:
        if handle is None:
            return {"accepted": False, "reason": "no response to the goal request"}
        if not handle.accepted:
            return {"accepted": False, "reason": "rejected by the goal callback"}
        result_future = handle.get_result_async()
        if not self._spin_until(result_future, timeout_sec):
            return {"accepted": True, "timed_out": True}
        result = result_future.result().result
        return {
            "accepted": True,
            "success": bool(result.success),
            "message": result.message,
            "completed_nodes": int(result.completed_nodes),
        }

    def run(self, activity_id: str, timeout_sec: float) -> dict:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            return {"error": "execute_activity action not available"}

        first_handle = self._send(activity_id, timeout_sec)
        # Sent while the first is still executing: that is the whole point.
        second_handle = self._send(activity_id, timeout_sec)

        # The second is collected first — it is the one expected to be refused,
        # and waiting on the first would hide a second that never settles.
        second = self._collect(second_handle, timeout_sec)
        first = self._collect(first_handle, timeout_sec)
        return {"first": first, "second": second}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--activity", required=True)
    parser.add_argument("--server", default="execute_activity")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    args = parser.parse_args(argv)

    rclpy.init()
    probe = DoubleGoalProbe(args.server)
    try:
        report = probe.run(args.activity, args.timeout_sec)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    print(json.dumps(report))
    return 0 if "error" not in report else 2


if __name__ == "__main__":
    sys.exit(main())
