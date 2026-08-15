#!/usr/bin/env python3
"""Does the hand bridge still answer while its SDK worker is busy? (L3)

The bridge used to call the vendor SDK from its ROS callbacks. A status read
that takes 11 ms on real hardware — and a tactile read that has been measured at
37 ms — therefore sat on the single-threaded executor, and while it sat there the
node could not answer its own claim or stop service. That was observed as a
service "not answering", which is the worst possible failure for a stop.

SDK calls now go through one serialized worker on a dedicated thread, and
acquisition runs on its own paced thread. This script measures whether that
actually holds on hardware, by asking the one question that distinguishes the two
designs:

    if the SDK work per second is multiplied, does service latency follow?

With reads on the executor it must: every read is time the executor is not
answering. With reads on the worker it must not: the executor never blocks on
them, so its latency is set by its own loop and stays flat while the worker gets
busier. Running the same probe at several `joint_read_rate` values and comparing
the curve is the evidence — it needs no second build of the old code.

It also reads the per-thread SDK call attribution out of the bridge's own runtime
metrics, which is how the one-owner invariant is checked: a call made from
anywhere but the worker thread shows up under a different thread name.

Hardware: this drives a real hand over its own CAN bus. It sends exactly one
command, to the pose the hand is already in, so nothing moves.

Usage:
  python3 scripts/measure_hand_executor_latency.py [--side right]
      [--read-rates 20 100 200] [--samples 40] [--window 8]
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import statistics
import subprocess
import sys
import time

CONSOLE_SCRIPT = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_bridge"
METRICS_PERIOD_S = 5.0


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def process_cpu_percent(pid: int, window_s: float) -> tuple[float, int]:
    """Percent of ONE core, plus thread count. One core is a Python node's ceiling."""
    ticks = os.sysconf("SC_CLK_TCK")

    def cpu_ticks() -> int:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
            fields = handle.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])

    before = cpu_ticks()
    time.sleep(window_s)
    after = cpu_ticks()
    return (after - before) / ticks / window_s * 100.0, len(os.listdir(f"/proc/{pid}/task"))


class Probe:
    """A commander that claims the hand, times the services, and gives it back."""

    def __init__(self, side: str, namespace: str, node_name: str = "l3_executor_probe") -> None:
        import rclpy
        from agx_arm_msgs.msg import OmniHandStatus
        from agx_arm_msgs.srv import ClaimDevice
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.node = Node(node_name)
        # The owner id names this node, and the bridge revokes an owner it cannot
        # find in the graph. A probe that lies about who it is gets revoked
        # mid-measurement and every later claim reads as a spurious success.
        self.owner_id = f"reactive:{node_name}"
        ns = namespace.rstrip("/")
        self.claim = self.node.create_client(ClaimDevice, f"{ns}/control/omnihand/claim_device")
        self.stop = self.node.create_client(Trigger, f"{ns}/control/omnihand/stop")
        self.command_pub = self.node.create_publisher(JointState, f"{ns}/control/joint_states", 10)
        self.joint_names: list[str] = []
        self.positions: list[float] = []
        self.status_seen = 0
        self.node.create_subscription(
            JointState, f"{ns}/feedback/omnihand/joint_states", self._on_joints, 10
        )
        self.node.create_subscription(
            OmniHandStatus, f"{ns}/feedback/omnihand/status", self._on_status, 10
        )

    def _on_joints(self, msg) -> None:
        self.joint_names = list(msg.name)
        self.positions = list(msg.position)

    def _on_status(self, msg) -> None:
        self.status_seen += 1

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.positions and self.claim.service_is_ready() and self.stop.service_is_ready():
                return True
        return False

    def _call(self, client, request, timeout: float = 5.0) -> tuple[float, object]:
        """One service round trip, timed from send to response."""
        start = time.perf_counter()
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        return elapsed_ms, (future.result() if future.done() else None)

    def claim_hand(self, take: bool) -> tuple[float, object]:
        from agx_arm_msgs.srv import ClaimDevice

        request = ClaimDevice.Request()
        request.owner_id = self.owner_id
        request.claim = take
        return self._call(self.claim, request)

    def stop_hand(self) -> tuple[float, object]:
        from std_srvs.srv import Trigger

        return self._call(self.stop, Trigger.Request())

    def command_current_pose(self) -> None:
        """Command the pose the hand already holds: the path is exercised, nothing moves."""
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        self.command_pub.publish(msg)

    def destroy(self) -> None:
        self.node.destroy_node()


def measure_one_rate(args, read_rate: float) -> dict | None:
    env = dict(os.environ)
    proc = subprocess.Popen(
        [
            CONSOLE_SCRIPT, "--ros-args",
            "-p", "backend_type:=sdk",
            "-p", f"omnihand_type:={args.side}",
            "-p", "hand_model:=o12_pro",
            "-p", f"can_interface:=hand_{args.side}",
            "-p", f"joint_read_rate:={read_rate}",
            "-p", "runtime_metrics_enabled:=true",
            "-p", f"runtime_metrics_period_s:={METRICS_PERIOD_S}",
            "-r", f"__ns:={args.namespace}" if args.namespace else "__ns:=/",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    output: list[str] = []
    try:
        probe = Probe(args.side, args.namespace)
        if not probe.wait_ready():
            print(f"  joint_read_rate={read_rate:<6g} bridge did not become ready")
            return None

        # Let the worker reach steady state before anything is timed.
        probe.spin(2.0)

        claim_ms: list[float] = []
        release_ms: list[float] = []
        refusals = 0
        for _ in range(args.samples):
            elapsed, response = probe.claim_hand(True)
            if response is None or not response.accepted:
                refusals += 1
            claim_ms.append(elapsed)
            probe.spin(0.02)
            elapsed, _ = probe.claim_hand(False)
            release_ms.append(elapsed)
            probe.spin(0.02)

        # Command and stop are measured while the hand is genuinely claimed:
        # a fail-closed bridge would otherwise answer the cheap rejection path.
        _, response = probe.claim_hand(True)
        held = response is not None and response.accepted
        probe.command_current_pose()
        probe.spin(0.5)
        stop_ms = []
        for _ in range(args.stop_samples):
            elapsed, _ = probe.stop_hand()
            stop_ms.append(elapsed)
            probe.spin(0.05)
        probe.claim_hand(False)

        cpu, threads = process_cpu_percent(proc.pid, args.window)
        probe.destroy()
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            output = proc.communicate(timeout=8)[0].splitlines()
        except subprocess.TimeoutExpired:
            proc.kill()
            output = proc.communicate()[0].splitlines()

    # Attribution is read per metrics window, and the FIRST window is discarded.
    # The backend is constructed on the main thread before the worker exists, so
    # that window legitimately names two callers; counting it would hide a real
    # second caller behind a known-benign one.
    windows: list[set[str]] = []
    lane_wait: dict[str, dict[str, float]] = {}
    worst_sdk_ms = 0.0
    for line in output:
        if "runtime metrics over" in line:
            windows.append(set())
        for thread in re.findall(r"\[([a-zA-Z0-9_\-]+)\]:", line):
            if windows:
                windows[-1].add(thread)
        match = re.search(r"^\s*sdk\.(\S+): n=\d+ mean=\S+ min=\S+ max=([\d.]+)ms", line)
        if match:
            worst_sdk_ms = max(worst_sdk_ms, float(match.group(2)))
        # How long a submission waited before the worker got to it. For the
        # safety lane this is the real distance between "stop acknowledged" and
        # "stop on the wire": the lane jumps the queue, but nothing preempts the
        # call already executing, so this is the honest bound on a stop.
        match = re.search(
            r"^\s*sdk_queue_wait\.(\w+): n=(\d+) mean=([\d.]+)ms min=\S+ max=([\d.]+)ms",
            line,
        )
        if match:
            lane = match.group(1)
            entry = lane_wait.setdefault(lane, {"n": 0, "mean_ms": 0.0, "max_ms": 0.0})
            entry["n"] += int(match.group(2))
            entry["mean_ms"] = max(entry["mean_ms"], float(match.group(3)))
            entry["max_ms"] = max(entry["max_ms"], float(match.group(4)))
    sdk_threads: set[str] = set().union(*windows[1:]) if len(windows) > 1 else set()

    return {
        "read_rate": read_rate,
        "claim_ms": claim_ms,
        "release_ms": release_ms,
        "stop_ms": stop_ms,
        "refusals": refusals,
        "held": held,
        "cpu": cpu,
        "threads": threads,
        "sdk_threads": sdk_threads,
        "metrics_windows": len(windows),
        "lane_wait": lane_wait,
        "worst_sdk_ms": worst_sdk_ms,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="right", choices=["right", "left"])
    parser.add_argument("--read-rates", type=float, nargs="+", default=[20.0, 100.0, 200.0])
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument(
        "--stop-samples",
        type=int,
        default=10,
        help="Stops per rate. The safety-lane wait is bounded by the call in "
        "flight, so its tail only shows up with enough stops to land on a slow one.",
    )
    parser.add_argument("--window", type=float, default=8.0)
    parser.add_argument(
        "--namespace",
        default=None,
        help="Bridge namespace. Defaults to the side's Duo namespace "
        "(<side>_arm in duo_motion_registry.yaml), which is where the real "
        "bringup puts it; a probe pointed at the root namespace finds no "
        "services and reports a timeout as a latency.",
    )
    args = parser.parse_args()
    if args.namespace is None:
        args.namespace = f"/{args.side}_arm"

    if not os.path.exists(CONSOLE_SCRIPT):
        print(f"!! {CONSOLE_SCRIPT} not found — build agx_arm_ctrl first")
        return 1

    import rclpy

    rclpy.init()
    results = []
    try:
        for rate in args.read_rates:
            print(f"--- joint_read_rate={rate:g} Hz ---", flush=True)
            result = measure_one_rate(args, rate)
            if result is not None:
                results.append(result)
    finally:
        rclpy.shutdown()

    if not results:
        return 1

    print()
    print(f"=== {args.side} hand: service latency against SDK load (L3, real CAN) ===")
    print(
        f"{'read Hz':>8} {'claim med':>10} {'claim p95':>10} {'claim max':>10} "
        f"{'stop med':>9} {'stop max':>9} {'worst SDK':>10} {'CPU/core':>9} {'thr':>4}"
    )
    for r in results:
        print(
            f"{r['read_rate']:>8g} "
            f"{statistics.median(r['claim_ms']):>9.1f}ms "
            f"{percentile(r['claim_ms'], 0.95):>9.1f}ms "
            f"{max(r['claim_ms']):>9.1f}ms "
            f"{statistics.median(r['stop_ms']):>8.1f}ms "
            f"{max(r['stop_ms']):>8.1f}ms "
            f"{r['worst_sdk_ms']:>9.1f}ms "
            f"{r['cpu']:>8.1f}% {r['threads']:>4d}"
        )
    print()
    for r in results:
        print(
            f"read {r['read_rate']:g} Hz: SDK callers in steady state = "
            f"{', '.join(sorted(r['sdk_threads'])) or 'not reported'}"
            f" [{r['metrics_windows']} metrics windows, first discarded]"
            f"   (claims refused: {r['refusals']}/{len(r['claim_ms'])}, "
            f"held for command: {r['held']})"
        )
    print()
    print("=== how long a submission waited for the worker, by lane ===")
    for r in results:
        lanes = r["lane_wait"]
        if not lanes:
            print(f"read {r['read_rate']:g} Hz: no lane waits reported")
            continue
        rendered = "  ".join(
            f"{lane}: n={int(entry['n'])} mean<={entry['mean_ms']:.2f}ms "
            f"max={entry['max_ms']:.2f}ms"
            for lane, entry in sorted(lanes.items())
        )
        print(f"read {r['read_rate']:g} Hz: {rendered}")
    print()
    print(
        "Read it this way: the worst SDK call is the longest the worker was busy in\n"
        "one call. If service latency tracks that, the executor is still doing SDK\n"
        "work. If it stays flat while SDK load multiplies, it is not. The safety\n"
        "lane wait is separate and is what a stop actually costs: the lane jumps the\n"
        "queue, but the call already running finishes first."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
