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

from demo_stack import (
    UNIT_NAMES,
    UNIT_ENV_VAR,
    StackState,
    refuse_root,
    resolve_unit,
    running_supervisor,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--unit", choices=UNIT_NAMES, default=None,
        help=f"override {UNIT_ENV_VAR} for this command",
    )
    parser.add_argument(
        # The supervisor's own ladder bounds this: two launches, each 30s on
        # SIGINT then 10s on SIGTERM. Giving up earlier reports a teardown that
        # is still running correctly as a failure.
        "--timeout-sec", type=float, default=120.0,
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
    refuse_root()
    unit = resolve_unit(args.unit)

    state = running_supervisor(unit)
    if state is None:
        print(f"no {unit} demo stack supervisor is running.")
        # Name the path: "nothing is running" and "I looked in the wrong home"
        # are the same sentence otherwise, and only one of them means the arms
        # are safe.
        print(f"  (looked for {StackState.path(unit)})")
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
            f"  its teardown gives each launch 30s on SIGINT then 10s on SIGTERM, so\n"
            f"  give it longer with --timeout-sec, or look at {state.log_dir}.\n"
            f"  Its pane says which phase it is in: 'all launches are down' means the\n"
            f"  launches stopped and the supervisor itself is not exiting.",
            file=sys.stderr,
        )
        return 1

    print("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
