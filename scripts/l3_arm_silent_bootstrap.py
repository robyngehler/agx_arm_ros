#!/usr/bin/env python3
"""An arm booted with its feedback push off is woken by the bootstrap (L3).

Puts the arm into the mute state deliberately — the push bit off, which is what
a persisted leader/follower configuration leaves behind — then reports whether
startup restores feedback without commanding motion.

**This run commands no motion.** It sends the feedback-push mode frame and, in
the enable phase, the joint enable request. Neither is a joint command.

Usage:
  python3 scripts/l3_arm_silent_bootstrap.py --iface can_nero_right [--silence-only]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time


def tx_rx(iface: str) -> tuple[int, int]:
    """Return (rx_packets, tx_packets) for one interface."""
    out = subprocess.run(
        ["ip", "-s", "link", "show", iface], capture_output=True, text=True
    ).stdout.splitlines()
    rx = tx = -1
    for index, line in enumerate(out):
        if line.strip().startswith("RX:"):
            rx = int(out[index + 1].split()[1])
        if line.strip().startswith("TX:"):
            tx = int(out[index + 1].split()[1])
    return rx, tx


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can_nero_right")
    parser.add_argument("--robot", default="nero")
    parser.add_argument("--observe-s", type=float, default=3.0)
    parser.add_argument(
        "--silence-only",
        action="store_true",
        help="silence the push and exit, leaving the arm mute for a node restart",
    )
    args = parser.parse_args()

    from agx_arm_ctrl import nero_can_push
    from pyAgxArm import AgxArmFactory, create_agx_arm_config

    config = create_agx_arm_config(robot=args.robot, comm="can", channel=args.iface)
    arm = AgxArmFactory.create_arm(config)
    arm.connect()

    if not nero_can_push.supports_can_push(arm):
        print(f"FAIL  {nero_can_push.UNSUPPORTED_MESSAGE}")
        return 2

    def rx_rate(label: str) -> float:
        before, _ = tx_rx(args.iface)
        time.sleep(args.observe_s)
        after, _ = tx_rx(args.iface)
        rate = (after - before) / args.observe_s
        print(f"  {label:<28} {rate:8.1f} RX/s")
        return rate

    print(f"[{args.iface}] baseline")
    baseline = rx_rate("before silencing")

    print(f"[{args.iface}] sending feedback-push DISABLE")
    nero_can_push.set_can_push(arm, False)
    silenced = rx_rate("after silencing")

    if args.silence_only:
        arm.disconnect()
        ok = silenced < max(1.0, baseline * 0.1)
        print(
            f"\n{'OK  ' if ok else 'FAIL'} arm left "
            f"{'MUTE' if ok else 'STILL PUSHING'} for the node restart"
        )
        return 0 if ok else 1

    print(f"[{args.iface}] sending feedback-push ENABLE (the bootstrap primitive)")
    nero_can_push.set_can_push(arm, True)
    restored = rx_rate("after bootstrap")
    arm.disconnect()

    muted = silenced < max(1.0, baseline * 0.1)
    woken = restored > max(1.0, baseline * 0.5)
    print()
    print(f"{'OK  ' if muted else 'FAIL'} push DISABLE silenced the arm")
    print(f"{'OK  ' if woken else 'FAIL'} push ENABLE alone restored feedback "
          f"(no mode switch, no motion command)")
    return 0 if (muted and woken) else 1


if __name__ == "__main__":
    sys.exit(main())
