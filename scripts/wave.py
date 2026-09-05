#!/usr/bin/env python3
"""Wave with both arms, then return to the working pose.

Top unit. Enters and leaves on Functional_Init_Both_V03, so it runs directly
after unpack_top_unit.py and before pack_top_unit.py — and on its own, because
its first step moves to that pose rather than assuming it.

Two waves with an anchor move and a hold between them; about a minute and a half.
"""
from demo_stack import ActivitySpec, main_for

SPEC = ActivitySpec(
    name="wave",
    unit="top",
    activity="wave_after_unpack_v1",
    description="top unit: wave with both arms from and back to the working pose",
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
