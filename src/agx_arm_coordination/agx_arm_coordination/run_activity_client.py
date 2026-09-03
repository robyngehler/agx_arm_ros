#!/usr/bin/env python3
"""Trigger one activity on the coordinator from the CLI (validation helper).

    ros2 run agx_arm_coordination run_activity --activity hands_open_close_release_v1

Sends a PerformActivity goal to ``execute_activity``, streams feedback, and
prints the structured result. Handy for coordinator bring-up without writing a
bespoke client each time.

Ctrl+C **cancels the activity** rather than just exiting: killing the client does
not stop the robot, since the goal keeps executing server-side. The first
interrupt sends a cancel and waits for the coordinator to confirm it unwound. A
second one, or a cancel that produced no result, escalates to the unit emergency
stop — the same ladder the coordinator's own second interrupt takes. Exiting with
the arm unaccounted for is not one of the options.
"""

from __future__ import annotations

import argparse
import json
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from agx_arm_msgs.action import PerformActivity

#: Unit-wide stop hosted by ``agx_arm_duo_soft_estop``: soft hold on every arm,
#: then the verified per-side escalation. The per-arm services are the fallback
#: for a stack brought up without that node.
DUO_ESTOP_SERVICE = "/emergency_stop"
ARM_ESTOP_SERVICES = ("/left_arm/emergency_stop", "/right_arm/emergency_stop")


class RunActivityClient(Node):
    def __init__(self, server: str) -> None:
        super().__init__("run_activity_client")
        self._client = ActionClient(self, PerformActivity, server)
        self._interrupted = False
        self._interrupt_count = 0

    # --- interrupt handling --------------------------------------------------

    def install_interrupt_handler(self) -> None:
        """Take SIGINT from rclpy so the context survives long enough to cancel.

        rclpy's default handler shuts the context down immediately, which would
        leave no working graph to send the cancel request over — the activity
        would keep running on hardware with its client gone.

        The interrupts are counted, not just latched: the first cancels, the
        second escalates to the emergency stop, and only a third leaves through
        ``KeyboardInterrupt``. rclpy installs its handler below Python, so
        ``getsignal`` can answer ``None`` — restoring that would raise inside the
        handler, which is the one place an exception helps nobody.
        """
        previous = signal.getsignal(signal.SIGINT)
        if not callable(previous):
            previous = signal.default_int_handler

        def _on_interrupt(signum, frame):
            self._interrupt_count += 1
            if self._interrupt_count >= 3:
                signal.signal(signal.SIGINT, previous)
                raise KeyboardInterrupt
            self._interrupted = True

        signal.signal(signal.SIGINT, _on_interrupt)

    def _spin_until(self, future, timeout_sec: float | None = None,
                    *, interruptible: bool = True) -> bool:
        """Spin until ``future`` completes, an interrupt arrives, or time runs out.

        ``interruptible=False`` for the emergency stop: it is what an interrupt
        asked for, so it must not be abandoned by the next one.
        """
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if interruptible and self._interrupted:
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
            if self._interrupted:
                # The goal can still be accepted and run with nobody holding its
                # handle, so this is not a quiet exit.
                return self._emergency_stop("interrupted before the goal was accepted")
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

        Exiting early is the dangerous move: the coordinator's stop path (drop the
        MIT trajectories -> cancel children -> reopen hand windows -> pin the arms)
        needs this client only for the cancel request, but staying until the result
        arrives is what makes "the client exited" mean "the robot is stopped".

        Where that confirmation does not arrive — a second interrupt, or a cancel
        that produced no result — the fallback is the emergency stop, not advice
        printed to an operator who is already holding Ctrl+C.
        """
        self.get_logger().warn(
            "interrupted: cancelling the activity "
            "(Ctrl+C again to force the emergency stop)"
        )
        self._interrupted = False  # so _spin_until keeps spinning through the cancel
        try:
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until(cancel_future, timeout_sec=timeout_sec)
            done = self._spin_until(result_future, timeout_sec=timeout_sec)
        except KeyboardInterrupt:
            done = False

        if self._interrupted:
            return self._emergency_stop("second interrupt while the activity unwound")
        if not done:
            return self._emergency_stop(
                f"no result {timeout_sec:.0f}s after cancelling the activity"
            )
        result = result_future.result().result
        self.get_logger().warn(
            f"activity canceled after {result.completed_nodes}/{result.total_nodes} "
            f"nodes: {result.message}"
        )
        return 130

    # --- escalation ----------------------------------------------------------

    def _emergency_stop(self, reason: str, timeout_sec: float = 10.0) -> int:
        """Call the unit emergency stop and report what it verified.

        The duo service holds every arm and then escalates each side to its
        verified stop; the per-arm services are the fallback when the duo e-stop
        node is not running. An unverified stop is reported as an unverified
        stop: this unit has no mechanical emergency stop, so the only remaining
        one is cutting arm power, and that drops the arm.
        """
        self.get_logger().error(f"ESCALATING TO EMERGENCY STOP: {reason}")
        outcomes = self._call_estop(DUO_ESTOP_SERVICE, timeout_sec)
        if outcomes is None:
            self.get_logger().warn(
                f"{DUO_ESTOP_SERVICE} not available — falling back to the per-arm stops"
            )
            outcomes = []
            for service in ARM_ESTOP_SERVICES:
                result = self._call_estop(service, timeout_sec)
                if result is None:
                    outcomes.append((service, False, "service not available"))
                else:
                    outcomes.extend(result)

        unverified = [service for service, ok, _ in outcomes if not ok]
        for service, ok, message in outcomes:
            if ok:
                self.get_logger().warn(f"emergency stop verified [{service}]: {message}")
            else:
                self.get_logger().error(f"emergency stop NOT verified [{service}]: {message}")
        if not outcomes or unverified:
            self.get_logger().error(
                "CUT ARM POWER — the software stop was not confirmed. Treat the "
                "arms as still in motion. This unit has no mechanical emergency "
                "stop, and removing power drops the arm because a de-energized "
                "Nero has no brakes."
            )
        return 130

    def _call_estop(self, service: str, timeout_sec: float):
        """Call one Trigger stop. ``None`` means the service was not there at all."""
        client = self.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(Trigger.Request())
        # Not interruptible: this call IS what the interrupt asked for.
        if not self._spin_until(future, timeout_sec=timeout_sec, interruptible=False):
            return [(service, False, f"no response within {timeout_sec:.0f}s")]
        response = future.result()
        if response is None:
            return [(service, False, "no response")]
        return [(service, bool(response.success), response.message or "")]

    def _on_feedback(self, feedback) -> None:
        fb = feedback.feedback
        self.get_logger().info(
            f"[{fb.completed_nodes}/{fb.total_nodes}] "
            f"#{fb.action_no} {fb.action_id} -> {fb.state}"
        )


def _with_resume(metadata_json: str, from_id) -> str:
    """Fold ``--from-id`` into the metadata object the goal carries.

    Refused rather than merged when the metadata already declares a resume: two
    step numbers for one run is a contradiction, and silently preferring one is
    how a run starts somewhere the operator did not ask for.
    """
    if from_id is None:
        return metadata_json
    payload = {}
    if metadata_json and metadata_json.strip():
        try:
            payload = json.loads(metadata_json)
        except json.JSONDecodeError as exc:
            raise ValueError(f"--metadata-json is not JSON: {exc}") from None
        if not isinstance(payload, dict):
            raise ValueError("--metadata-json must be a JSON object")
    if "resume" in payload:
        raise ValueError(
            "--from-id and a 'resume' block in --metadata-json both set the "
            "start step; declare one"
        )
    if from_id < 1:
        raise ValueError(f"--from-id is 1-based; got {from_id}")
    payload["resume"] = {"from_step": from_id}
    return json.dumps(payload)


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
    parser.add_argument(
        "--from-id", "--from_id",
        dest="from_id",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Resume at operator step N (1-based). A step is one dispatch batch, "
            "so a synchronized pair is one step. Sugar for --metadata-json "
            "'{\"resume\": {\"from_step\": N}}'; declaring both is refused."
        ),
    )
    args = parser.parse_args(argv)

    try:
        metadata_json = _with_resume(args.metadata_json, args.from_id)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init()
    node = None
    try:
        node = RunActivityClient(args.server)
        node.install_interrupt_handler()
        code = node.run(args.activity, args.timeout_sec, metadata_json)
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
