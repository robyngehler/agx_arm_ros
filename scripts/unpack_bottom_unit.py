#!/usr/bin/env python3
"""Unfold the bottom unit out of its packing pose into the presentation pose.

Arms only. Runs from wherever the arms stand: the first step moves to the packing
pose rather than assuming it. Every step is a planned anchor move, so any step is
a valid --from-id.
"""
from demo_stack import ActivitySpec, main_for


def spec(args):
    return ActivitySpec(
        name="unpack_bottom_unit",
        unit="bottom",
        activity=f"unit_unpack_bottom_{args.speed}_v1",
        description=f"bottom unit: packing pose -> presentation pose ({args.speed})",
    )


if __name__ == "__main__":
    main_for(spec, __doc__, with_speed=True)
