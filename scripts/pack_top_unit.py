#!/usr/bin/env python3
"""Fold the top unit out of the working pose back into its packing pose.

Arms only, two steps. The first step moves to the working pose, so the fold starts
from a known pose rather than from wherever a cancelled demo stopped.
"""
from demo_stack import arm_flow, main_for

SPEC = arm_flow(
    name="pack_top_unit",
    activity="unit_pack_top_v1",
    description="top unit: Functional_Init_Both_V03 -> packing pose",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
