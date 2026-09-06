#!/usr/bin/env python3
"""Fold the stacking unit out of the working pose back into its packing pose.

Arms only, two steps. The first step moves to Boxing_Both_Idle_V01, so the fold
starts from a known pose rather than from wherever a cancelled demo stopped. Both
steps are planned anchor moves, so either is a valid --from-id.
"""
from demo_stack import ActivitySpec, main_for

SPEC = ActivitySpec(
    name="pack_stacking_unit",
    unit="stacking",
    activity="unit_pack_stacking_v1",
    description="stacking unit: Boxing_Both_Idle_V01 -> Packing_Pose_Podest",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
