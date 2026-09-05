#!/usr/bin/env python3
"""Ask this unit's demo stack supervisor to shut down.

Signals the supervisor named in the state file and waits for it to go, so
coordination unwinds before its arms are taken away. It does not search for or
kill ROS processes itself: anything not started by that supervisor is not its to
end, and a stack whose supervisor is gone is reported rather than guessed at.

No --top/--bottom: a unit is one machine and holds one stack. ``--unit`` exists
only for a machine whose ``AGX_UNIT`` is wrong.

    ./scripts/stop_demo_stack.py
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from demo_stack import UNIT_NAMES, UNIT_ENV_VAR, StackState, resolve_unit, running_supervisor


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--unit", choices=UNIT_NAMES, default=None,
        help=f"override {UNIT_ENV_VAR} for this command",
    )
    parser.add_argument(
        "--timeout-sec", type=float, default=90.0,
        help="how long to wait for the supervisor to finish its teardown",
    )
    return parser


def _wait_for_exit(state: StackState, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    last_report = 0.0
    while time.monotonic() < deadline:
        if not state.alive():
            return True
        now = time.monotonic()
        if now - last_report > 10.0:
            last_report = now
            print("  still shutting down", flush=True)
        time.sleep(0.5)
    return not state.alive()


def main() -> int:
    args = build_parser().parse_args()
    unit = resolve_unit(args.unit)

    state = running_supervisor(unit)
    if state is None:
        print(f"no {unit} demo stack supervisor is running.")
        # A stale file is already cleared by running_supervisor; say so only if
        # something is left that this cannot account for.
        return 0

    print(f"stopping the {unit} demo stack (supervisor pid {state.pid})")
    print(f"  logs: {state.log_dir}")
    try:
        os.kill(state.pid, signal.SIGTERM)
    except ProcessLookupError:
        state.remove()
        print("the supervisor was already gone; cleared its state file")
        return 0
    except PermissionError:
        print(
            f"not allowed to signal pid {state.pid} — it belongs to another user.",
            file=sys.stderr,
        )
        return 1

    if not _wait_for_exit(state, args.timeout_sec):
        print(
            f"the supervisor is still running after {args.timeout_sec:.0f}s.\n"
            f"  its teardown gives each launch 20s on SIGINT before escalating, so\n"
            f"  give it longer with --timeout-sec, or look at {state.log_dir}.",
            file=sys.stderr,
        )
        return 1

    print("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
