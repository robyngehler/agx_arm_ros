#!/usr/bin/env python3
"""Bring up the gripper stack and run block_restack_v1.

Sixty-three operator steps: four blocks moved one at a time from the left gripper
to the right and placed, then a closing gripper sequence. Every step is a planned
anchor move or a gripper command, so any step is a valid --from-id.

Needs a parallel gripper on both arms — the stack it starts is `duo_gripper`, not
the `duo_hand` the pack and unpack flows use.
"""
from demo_stack import BLOCK_RESTACK, main_for

if __name__ == "__main__":
    main_for(BLOCK_RESTACK, __doc__)
