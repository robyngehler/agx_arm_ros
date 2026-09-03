#!/usr/bin/env python3
"""Unfold the top unit out of its packing pose into the working pose.

Arms only, two steps. The same motion the tea demo makes on its way in, available
without running a demo.
"""
from demo_stack import arm_flow, main_for

SPEC = arm_flow(
    name="unpack_top_unit",
    activity="unit_unpack_top_v1",
    description="top unit: packing pose -> Functional_Init_Both_V03",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
