#!/usr/bin/env python3
"""Capture a tea-pour run and the emergency stop that interrupts it (L3).

Records both arm buses to pcap, runs ``tea_pour_left_v1``, fires the arm
emergency stop while a recorded trajectory is still replaying, and keeps
recording long enough to show what the arm did afterwards.

What the capture is for: the stop ladder ends at the firmware ``MOVE-J`` hold
and must never issue the vendor electronic stop, which damps without stiffness
and lets a raised arm descend. That is a claim about frames on the wire, so it
is settled on the wire — ``analyze_can_pcap.py --stop-at`` reads the capture
back and reports the signature. See
``docs/sprint_refactor/reference/emergency_stop_ladder.md``.

**This moves the robot and then stops it.** The stop latches: each arm refuses
motion until ``clear_fault_lockout``, and the unit holds a safety generation
until ``unit_safety/rearm``. Both commands are printed when the run ends.

The default trigger is node 160 (``left_arm_teapot_handle_release``): a recorded
trajectory replaying with the teapot already set down and released, so the arm
is moving and empty. Node 110 (``left_arm_pour_tea``) is the more demanding
case — payload at height — and is one flag away.

Usage:
  python3 scripts/l3_estop_pcap_run.py
  python3 scripts/l3_estop_pcap_run.py --trigger-action-no 110 --trigger-delay 8
  python3 scripts/l3_estop_pcap_run.py --dry-run
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from agx_arm_msgs.action import PerformActivity

ARM_IFACE = {"left": "can_nero_left", "right": "can_nero_right"}
REPO_ROOT = Path(__file__).resolve().parent.parent


def sudo_prefix() -> list[str]:
    """Empty when already root. ``-n`` so a password prompt fails instead of hanging."""
    return [] if os.geteuid() == 0 else ["sudo", "-n"]


class Capture:
    """One tcpdump per interface, stopped by an interrupt so the tail is flushed."""

    def __init__(self, ifaces: list[str], out_dir: Path) -> None:
        self.out_dir = out_dir
        self.paths = {iface: out_dir / f"{iface}.pcap" for iface in ifaces}
        self._procs: dict[str, subprocess.Popen] = {}

    def start(self) -> None:
        for iface, path in self.paths.items():
            # -U writes each frame straight through, so an interrupted capture
            # keeps the seconds that matter most: the ones just before it.
            cmd = sudo_prefix() + [
                "tcpdump", "-i", iface, "-U", "-w", str(path),
            ]
            self._procs[iface] = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
            )
        time.sleep(1.0)
        for iface, proc in self._procs.items():
            if proc.poll() is not None:
                error = proc.stderr.read().decode(errors="replace").strip()
                raise RuntimeError(f"tcpdump on {iface} exited immediately: {error}")
            if not self.paths[iface].exists():
                raise RuntimeError(f"tcpdump on {iface} wrote no file")

    def stop(self) -> None:
        # SIGINT by write path, not by the Popen handle: that handle is sudo's,
        # and sudo does not reliably forward a signal to the child.
        for path in self.paths.values():
            subprocess.run(
                sudo_prefix() + ["pkill", "-INT", "-f", f"tcpdump.*-w {path}"],
                check=False,
            )
        deadline = time.monotonic() + 10.0
        for iface, proc in self._procs.items():
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if proc.poll() is None:
                proc.kill()
                print(f"  WARNING: tcpdump on {iface} had to be killed; "
                      "its capture may be truncated")

    def report(self) -> None:
        for iface, path in self.paths.items():
            size = path.stat().st_size if path.exists() else 0
            print(f"  {iface}: {path} ({size / 1e6:.1f} MB)")


class EstopRun(Node):
    def __init__(self, server: str) -> None:
        super().__init__("l3_estop_pcap_run")
        self._group = ReentrantCallbackGroup()
        self._activity = ActionClient(
            self, PerformActivity, server, callback_group=self._group
        )
        self._estop = {
            side: self.create_client(
                Trigger, f"/{side}_arm/emergency_stop", callback_group=self._group
            )
            for side in ("left", "right")
        }
        self._reached = threading.Event()
        self._target_action_no: int | None = None
        self.feedback_log: list[dict] = []

    # -- readiness ------------------------------------------------------

    def wait_for_stack(self, timeout_s: float) -> bool:
        ok = True
        if not self._activity.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error("coordinator execute_activity is not available")
            ok = False
        for side, client in self._estop.items():
            if not client.wait_for_service(timeout_sec=timeout_s):
                self.get_logger().error(
                    f"/{side}_arm/emergency_stop is not available — the stop this "
                    "run exists to capture cannot be issued"
                )
                ok = False
        return ok

    # -- the activity ---------------------------------------------------

    def start_activity(self, activity_id: str, target_action_no: int):
        self._target_action_no = target_action_no
        goal = PerformActivity.Goal()
        goal.activity_id = activity_id
        future = self._activity.send_goal_async(goal, feedback_callback=self._on_feedback)
        handle = _await(future, 15.0)
        if handle is None or not handle.accepted:
            return None
        return handle

    def _on_feedback(self, message) -> None:
        feedback = message.feedback
        entry = {
            "t": time.time(),
            "action_no": int(feedback.action_no),
            "action_id": str(feedback.action_id),
            "state": str(feedback.state),
        }
        self.feedback_log.append(entry)
        print(f"  [{entry['action_no']:>4}] {entry['action_id']} -> {entry['state']}")
        if (
            entry["action_no"] == self._target_action_no
            and entry["state"] == "running"
        ):
            self._reached.set()

    def wait_for_target(self, timeout_s: float) -> bool:
        return self._reached.wait(timeout_s)

    # -- the stop -------------------------------------------------------

    def fire_emergency_stop(self) -> tuple[float, dict]:
        """Call both arms' stop, as ``emergency_stop_all`` does. Returns (ts, results).

        The timestamp is taken immediately before the calls go out, and is what
        the pcap is read against — the driver's own latching happens first, so
        anything on the wire after it belongs to the stop.
        """
        futures = {}
        stop_ts = time.time()
        for side, client in self._estop.items():
            futures[side] = client.call_async(Trigger.Request())
        results = {}
        for side, future in futures.items():
            response = _await(future, 30.0)
            if response is None:
                results[side] = {"success": None, "message": "no response in 30s"}
            else:
                results[side] = {
                    "success": bool(response.success),
                    "message": str(response.message),
                }
        return stop_ts, results


def _await(future, timeout_s: float):
    """Wait on a future driven by a background executor."""
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--activity", default="tea_pour_left_v1")
    parser.add_argument("--server", default="execute_activity")
    parser.add_argument(
        "--trigger-action-no", type=int, default=160,
        help="fire the stop while this graph node is running (default 160, "
             "left_arm_teapot_handle_release — a recorded trajectory, empty hand)",
    )
    parser.add_argument(
        "--trigger-delay", type=float, default=5.0,
        help="seconds into that node before the stop is fired (default 5)",
    )
    parser.add_argument(
        "--tail", type=float, default=8.0,
        help="seconds to keep capturing after the stop (default 8)",
    )
    parser.add_argument(
        "--ifaces", default=",".join(ARM_IFACE.values()),
        help="comma-separated CAN interfaces to capture",
    )
    parser.add_argument(
        "--out-dir", default=None,
        help="where to write the capture (default logs/estop_<timestamp>)",
    )
    parser.add_argument(
        "--activity-timeout", type=float, default=600.0,
        help="give up if the trigger node is not reached within this (default 600)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="run the activity and report when the trigger node starts, but "
             "capture nothing and fire no stop",
    )
    parser.add_argument("--no-analyse", action="store_true")
    args = parser.parse_args()

    ifaces = [name.strip() for name in args.ifaces.split(",") if name.strip()]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) if args.out_dir else REPO_ROOT / "logs" / f"estop_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"run:      {args.activity}, stop during node {args.trigger_action_no} "
          f"+{args.trigger_delay:.1f}s")
    print(f"capture:  {', '.join(ifaces)}")
    print(f"output:   {out_dir}")
    if args.dry_run:
        print("DRY RUN — nothing is captured and no stop is fired")
    print()

    rclpy.init()
    node = EstopRun(args.server)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    capture = Capture(ifaces, out_dir)
    capture_started = False
    stop_ts = None
    results: dict = {}
    exit_code = 0
    handle = None

    try:
        if not node.wait_for_stack(timeout_s=20.0):
            return 2

        if not args.dry_run:
            print("starting capture ...")
            capture.start()
            capture_started = True
            time.sleep(2.0)  # pre-roll, so the healthy MIT stream is in the file

        print("sending the activity goal ...")
        handle = node.start_activity(args.activity, args.trigger_action_no)
        if handle is None:
            print("ERROR: the coordinator rejected or never answered the goal")
            return 2

        print(f"waiting for node {args.trigger_action_no} to start ...")
        if not node.wait_for_target(args.activity_timeout):
            print(f"ERROR: node {args.trigger_action_no} never started — no stop "
                  "was fired. Check the feedback above.")
            exit_code = 3
        else:
            print(f"node {args.trigger_action_no} is running; "
                  f"stopping in {args.trigger_delay:.1f}s")
            time.sleep(args.trigger_delay)
            if args.dry_run:
                print("DRY RUN — this is where the stop would be fired")
            else:
                print(">>> EMERGENCY STOP")
                stop_ts, results = node.fire_emergency_stop()
                for side, result in sorted(results.items()):
                    print(f"  {side}: success={result['success']} {result['message']}")
                if not all(r["success"] for r in results.values()):
                    exit_code = 4
    except KeyboardInterrupt:
        print("\ninterrupted — unwinding")
        exit_code = 130
    except Exception as exc:  # a capture run must always close its capture
        print(f"ERROR: {exc}")
        exit_code = 1
    finally:
        if handle is not None:
            try:
                _await(handle.cancel_goal_async(), 15.0)
            except Exception as exc:
                print(f"  note: cancelling the activity failed: {exc}")
        if capture_started:
            print(f"holding the capture open for {args.tail:.1f}s ...")
            time.sleep(args.tail)
            capture.stop()
            print("capture stopped:")
            capture.report()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=5.0)

    if not capture_started:
        return exit_code

    run = {
        "activity": args.activity,
        "trigger_action_no": args.trigger_action_no,
        "trigger_delay_s": args.trigger_delay,
        "stop_unix_ts": stop_ts,
        "stop_results": results,
        "pcaps": {iface: str(path) for iface, path in capture.paths.items()},
        "feedback": node.feedback_log,
    }
    (out_dir / "run.json").write_text(json.dumps(run, indent=2))

    if stop_ts is None:
        print("\nNo stop was fired, so there is no signature to check.")
        return exit_code

    analyse = [
        sys.executable, str(REPO_ROOT / "scripts" / "analyze_can_pcap.py"),
        "--stop-at", f"{stop_ts:.6f}",
    ] + [str(path) for path in capture.paths.values()]
    print("\n" + " ".join(analyse))
    if not args.no_analyse:
        print()
        analysis = subprocess.run(analyse)
        if analysis.returncode == 3 and exit_code == 0:
            exit_code = 3

    print()
    print("The stop latched. Before running anything else:")
    print("  ros2 service call /left_arm/clear_fault_lockout std_srvs/srv/Trigger {}")
    print("  ros2 service call /right_arm/clear_fault_lockout std_srvs/srv/Trigger {}")
    print("  ros2 service call /unit_safety/rearm std_srvs/srv/Trigger {}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
