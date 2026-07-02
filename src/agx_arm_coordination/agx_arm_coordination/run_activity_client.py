#!/usr/bin/env python3
"""Trigger one activity on the coordinator from the CLI (validation helper).

    ros2 run agx_arm_coordination run_activity --activity hands_open_close_release_v1

Sends a PerformActivity goal to ``execute_activity``, streams feedback, and
prints the structured result. Handy for coordinator bring-up without writing a
bespoke client each time.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from agx_arm_msgs.action import PerformActivity


class RunActivityClient(Node):
    def __init__(self, server: str) -> None:
        super().__init__("run_activity_client")
        self._client = ActionClient(self, PerformActivity, server)

    def run(self, activity_id: str, timeout_sec: float) -> int:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("coordinator execute_activity action not available")
            return 2
        goal = PerformActivity.Goal()
        goal.activity_id = activity_id
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return 3
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"result: success={result.success} "
            f"({result.completed_nodes}/{result.total_nodes}) "
            f"failed_action='{result.failed_action_id}' msg='{result.message}'"
        )
        return 0 if result.success else 1

    def _on_feedback(self, feedback) -> None:
        fb = feedback.feedback
        self.get_logger().info(
            f"[{fb.completed_nodes}/{fb.total_nodes}] "
            f"#{fb.action_no} {fb.action_id} -> {fb.state}"
        )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--activity", required=True, help="activity_id to run")
    parser.add_argument("--server", default="execute_activity", help="action server name")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = RunActivityClient(args.server)
        code = node.run(args.activity, args.timeout_sec)
    except KeyboardInterrupt:
        code = 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
