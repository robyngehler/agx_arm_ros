#!/usr/bin/env python3
"""Run block_restack_v1 on the stacking unit.

Sixty-three operator steps: four blocks moved one at a time from the left gripper
to the right and placed, then a closing gripper sequence. Every step is a planned
anchor move or a gripper command, so any step is a valid --from-id.

Needs a parallel gripper on both arms, which is what the stacking unit is — its
stack comes up on `duo_gripper` with no flag:

    ./scripts/start_demo_stack.py
"""
from demo_stack import GRIPPER_ACTIONS, ActivitySpec, main_for

SPEC = ActivitySpec(
    name="block_restack",
    activity="block_restack_v1",
    unit="stacking",
    description="Restack four blocks, handing each from the left gripper to the right.",
    extra_actions=GRIPPER_ACTIONS,
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
