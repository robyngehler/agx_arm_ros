#!/usr/bin/env python3
"""Unfold the top unit out of its packing pose into the working pose.

Arms only, two steps. Needs the top unit's stack up:

    ./scripts/start_demo_stack.py
"""
from demo_stack import ActivitySpec, main_for

SPEC = ActivitySpec(
    name="unpack_top_unit",
    unit="top",
    activity="unit_unpack_top_v1",
    description="top unit: packing pose -> Functional_Init_Both_V03",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
