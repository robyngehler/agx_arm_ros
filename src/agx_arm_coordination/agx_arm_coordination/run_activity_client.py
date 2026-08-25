#!/usr/bin/env python3
"""Trigger one activity on the coordinator from the CLI (validation helper).

    ros2 run agx_arm_coordination run_activity --activity hands_open_close_release_v1

Sends a PerformActivity goal to ``execute_activity``, streams feedback, and
prints the structured result. Handy for coordinator bring-up without writing a
bespoke client each time.

Ctrl+C **cancels the activity** rather than just exiting: killing the client does
not stop the robot, since the goal keeps executing server-side. The first
interrupt sends a cancel and waits for the coordinator to confirm it unwound; a
second one gives up on waiting (the coordinator still unwinds on its own, the
cancel having been delivered).
"""

from __future__ import annotations

import argparse
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from agx_arm_msgs.action import PerformActivity


class RunActivityClient(Node):
    def __init__(self, server: str) -> None:
        super().__init__("run_activity_client")
        self._client = ActionClient(self, PerformActivity, server)
        self._interrupted = False

    # --- interrupt handling --------------------------------------------------

    def install_interrupt_handler(self) -> None:
        """Take SIGINT from rclpy so the context survives long enough to cancel.

        rclpy's default handler shuts the context down immediately, which would
        leave no working graph to send the cancel request over — the activity
        would keep running on hardware with its client gone.
        """
        previous = signal.getsignal(signal.SIGINT)

        def _on_interrupt(signum, frame):
            if self._interrupted:
                signal.signal(signal.SIGINT, previous)
                raise KeyboardInterrupt
            self._interrupted = True

        signal.signal(signal.SIGINT, _on_interrupt)

    def _spin_until(self, future, timeout_sec: float | None = None) -> bool:
        """Spin until ``future`` completes, an interrupt arrives, or time runs out."""
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if self._interrupted:
                return False
            if deadline is not None and time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    # --- run -----------------------------------------------------------------

    def run(self, activity_id: str, timeout_sec: float, metadata_json: str = "") -> int:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("coordinator execute_activity action not available")
            return 2
        goal = PerformActivity.Goal()
        goal.activity_id = activity_id
        goal.metadata_json = metadata_json
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not self._spin_until(send_future, timeout_sec=timeout_sec):
            self.get_logger().error("goal was never accepted or rejected")
            return 3
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return 3

        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future):
            return self._cancel_and_wait(goal_handle, result_future)

        result = result_future.result().result
        self.get_logger().info(
            f"result: success={result.success} "
            f"({result.completed_nodes}/{result.total_nodes}) "
            f"failed_action='{result.failed_action_id}' msg='{result.message}'"
        )
        return 0 if result.success else 1

    def _cancel_and_wait(self, goal_handle, result_future, timeout_sec: float = 20.0) -> int:
        """Cancel the activity and wait for the coordinator to finish unwinding.

        Exiting early is the dangerous move: the coordinator's stop path (cancel
        children -> reopen hand windows -> pin the arms) needs this client only
        for the cancel request, but staying until the result arrives is what makes
        "the client exited" mean "the robot is stopped".
        """
        self.get_logger().warn("interrupted: cancelling the activity")
        self._interrupted = False  # so _spin_until keeps spinning through the cancel
        try:
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until(cancel_future, timeout_sec=timeout_sec)
            done = self._spin_until(result_future, timeout_sec=timeout_sec)
        except KeyboardInterrupt:
            done = False

        if not done:
            self.get_logger().error(
                "no result after cancelling — do NOT assume the arm stopped. Check "
                "the coordinator log, and if it is still moving use the emergency "
                "stop: ros2 service call /left_arm/emergency_stop std_srvs/srv/Trigger"
            )
            return 130
        result = result_future.result().result
        self.get_logger().warn(
            f"activity canceled after {result.completed_nodes}/{result.total_nodes} "
            f"nodes: {result.message}"
        )
        return 130

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
    parser.add_argument(
        "--metadata-json",
        default="",
        help=(
            "Run-time overrides as a JSON object, applied to this run only. "
            "Playback: --metadata-json "
            "'{\"playback\": {\"mode\": \"tempo_scale\", \"speed_scale\": 0.6}}'"
        ),
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = None
    try:
        node = RunActivityClient(args.server)
        node.install_interrupt_handler()
        code = node.run(args.activity, args.timeout_sec, args.metadata_json)
    except KeyboardInterrupt:
        code = 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
