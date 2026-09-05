#!/usr/bin/env python3
"""Bring this unit's demo stack up and stay alive owning it.

Which unit this is comes from ``AGX_UNIT``; ``--unit`` overrides it. Components
are brought up and waited for before coordination is started, so the coordinator
never comes up against arms that are not there.

Run it in its own tmux pane and leave it. Activities run from another pane
against the stack this one holds. Stop it with Ctrl+C here or
``./scripts/stop_demo_stack.py`` from anywhere.

    tmux new -A -s demo
    ./scripts/start_demo_stack.py
"""
from demo_stack import resolve_unit, run_supervisor, supervisor_parser, unit_stack


def main() -> None:
    args = supervisor_parser(__doc__).parse_args()
    spec = unit_stack(resolve_unit(args.unit), stack=args.stack, grippers=args.grippers)
    raise SystemExit(run_supervisor(spec, args))


if __name__ == "__main__":
    main()
