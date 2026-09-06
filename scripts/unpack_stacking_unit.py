#!/usr/bin/env python3
"""Unfold the stacking unit out of its packing pose into the working pose.

Arms only, two steps. Runs from wherever the arms stand: the first step moves to
Packing_Pose_Podest rather than assuming it, so a cancelled demo does not have to
be tidied up by hand first. Both steps are planned anchor moves, so either is a
valid --from-id.

Leaves the arms where block_restack_v1 starts:

    ./scripts/start_block_restack.py
"""
from demo_stack import ActivitySpec, main_for

SPEC = ActivitySpec(
    name="unpack_stacking_unit",
    unit="stacking",
    activity="unit_unpack_stacking_v1",
    description="stacking unit: Packing_Pose_Podest -> Boxing_Both_Idle_V01",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
