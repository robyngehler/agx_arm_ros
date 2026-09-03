#!/usr/bin/env python3
"""Fold the bottom unit out of the presentation pose back into its packing pose.

Arms only. The first step moves to the presentation pose, so the fold into the
packing pose — the transition with no clearance margin — always starts from a
known pose.
"""
from demo_stack import arm_flow, main_for


def spec(args):
    return arm_flow(
        name="pack_bottom_unit",
        activity=f"unit_pack_bottom_{args.speed}_v1",
        description=f"bottom unit: presentation pose -> packing pose ({args.speed})",
    )


if __name__ == "__main__":
    main_for(spec, __doc__, with_speed=True)
