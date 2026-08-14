#!/usr/bin/env python3
"""Split the OmniHand bridge's per-tick cost into ROS publication and SDK polling.

The bridge's process cost is known (~160 % of a core per hand on the four-bus
stack) but that number is a sum: the same process pays for vendor-SDK round
trips over CANFD *and* for constructing and publishing three ROS messages on
every timer tick. Cutting the wrong one wastes the effort.

The mock backend is what separates them. It answers every read from a Python
list with no I/O at all, so a mock run measures the publication side alone, and
the hardware figure minus this one is the SDK side.

What it reports:

  * process CPU of a real spinning bridge on mock, per `pub_rate` — the whole
    ROS side including the executor;
  * wall-clock cost of one full `_feedback_tick` — acquisition and all three
    publications — and what that implies at a given rate;
  * the cumulative-time breakdown inside the tick, so the split between message
    construction, list copying, authority sync and rclpy publish is visible
    rather than assumed.

This is an L2 measurement: no hardware, no CAN. It bounds what the bridge can
possibly cost with a perfect device, which is the useful half of the question.

The node is started as the installed console script, not through `ros2 run`:
that wrapper forks the node into a child, so sampling the wrapper's pid measures
an idle process. Timing is done in Python rather than with shell `sleep`, which
some sandboxes neuter — a skipped sleep silently turns a 15 s window into a 0 s
one and reports whatever noise is left.

Usage:
  python3 scripts/profile_hand_bridge.py [--ticks 2000] [--side right]
                                         [--model o12_pro] [--top 25]
                                         [--process-rates 200 50 20]
"""

from __future__ import annotations

import argparse
import cProfile
import io
import os
import pstats
import signal
import subprocess
import time

CONSOLE_SCRIPT = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_bridge"


def _process_cpu_percent(pid: int, window_s: float) -> tuple[float, int]:
    """Percent of ONE core used by `pid` over `window_s`, plus its thread count.

    Percent of a core, not of the machine: these are GIL-bound Python nodes, so
    one core is the ceiling a single node can hit however many are idle.
    """
    ticks = os.sysconf("SC_CLK_TCK")

    def cpu_ticks() -> int:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
            fields = handle.read().rsplit(")", 1)[1].split()
        # utime and stime, offset by the two fields consumed by the split above.
        return int(fields[11]) + int(fields[12])

    before = cpu_ticks()
    time.sleep(window_s)
    after = cpu_ticks()
    threads = len(os.listdir(f"/proc/{pid}/task"))
    return (after - before) / ticks / window_s * 100.0, threads


def measure_process(rates: list[float], side: str, model: str, window_s: float) -> None:
    if not os.path.exists(CONSOLE_SCRIPT):
        print(f"!! {CONSOLE_SCRIPT} not found — build the package first")
        return

    print("=== spinning bridge on mock: the whole ROS side, executor included ===")
    for rate in rates:
        proc = subprocess.Popen(
            [
                CONSOLE_SCRIPT, "--ros-args",
                "-p", "backend_type:=mock",
                "-p", f"hand_model:={model}",
                "-p", f"omnihand_type:={side}",
                "-p", f"pub_rate:={rate}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            time.sleep(4.0)  # discovery and the first ticks
            if proc.poll() is not None:
                print(f"  pub_rate={rate:<6g} node exited early")
                continue
            percent, threads = _process_cpu_percent(proc.pid, window_s)
            print(
                f"  pub_rate={rate:<6g} {percent:6.1f} % of a core   ({threads} threads)"
            )
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=2000)
    parser.add_argument("--side", default="right")
    parser.add_argument("--model", default="o12_pro")
    parser.add_argument("--top", type=int, default=25)
    parser.add_argument(
        "--rate",
        type=float,
        default=200.0,
        help="Rate to project the per-tick cost onto. 200 is what every bringup "
        "passes today (the arm's pub_rate); the bridge's own default is 50.",
    )
    parser.add_argument(
        "--process-rates",
        type=float,
        nargs="*",
        default=[200.0, 50.0, 20.0],
        help="pub_rate values to spin a real node at. Empty to skip.",
    )
    parser.add_argument("--window", type=float, default=15.0)
    args = parser.parse_args()

    if args.process_rates:
        measure_process(args.process_rates, args.side, args.model, args.window)

    import rclpy

    from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode

    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", f"omnihand_type:={args.side}",
            "-p", f"hand_model:={args.model}",
            # The timer is driven by hand below, so the measurement is per tick
            # and not at the mercy of executor scheduling. Both gates are opened
            # (`joint_read_rate` <= 0 reads on every tick, `pub_rate` <= 0 sets no
            # ceiling) so this measures a tick doing its full work — otherwise
            # most hand-driven ticks would skip everything and report near zero.
            "-p", "joint_read_rate:=0.0",
            "-p", "pub_rate:=0.0",
            "-p", "command_retry_enabled:=false",
        ]
    )

    node = OmniHandBridgeNode()

    # Warm up: first calls pay one-off import and allocation costs that would
    # otherwise be charged to the steady-state figure.
    for _ in range(200):
        node._feedback_tick()

    start = time.perf_counter()
    for _ in range(args.ticks):
        node._feedback_tick()
    elapsed = time.perf_counter() - start

    per_tick_ms = elapsed / args.ticks * 1000.0
    print("=== OmniHand bridge publication cost (mock backend, no CAN) ===")
    print(f"hand:            {args.side} {args.model} ({len(node.joint_names)} joints)")
    print(f"ticks:           {args.ticks}")
    print(f"per tick:        {per_tick_ms:.3f} ms")
    print(
        f"at {args.rate:g} Hz:      {per_tick_ms * args.rate / 10.0:.1f} % of one core "
        "(publication only)"
    )
    for rate in (50.0, 20.0):
        print(f"at {rate:g} Hz:       {per_tick_ms * rate / 10.0:.1f} % of one core")

    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(args.ticks):
        node._feedback_tick()
    profiler.disable()

    stream = io.StringIO()
    stats = pstats.Stats(profiler, stream=stream).sort_stats("cumulative")
    stats.print_stats(args.top)
    print()
    print("=== where the tick goes (cumulative, profiler overhead included) ===")
    print(stream.getvalue())

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
