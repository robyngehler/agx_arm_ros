#!/usr/bin/env python3
"""Run tea_pour_duo_v2 against a running tea stack.

Twenty-one operator steps; seven of them replay a taught path and are therefore
not resume points. The right hand is out of service and is commanded nowhere.

Needs the tea stack, which drives both OmniHands and declares the teapot mass:

    ./scripts/start_demo_stack.py --stack tea

That supervisor starts agx_arm_coordination/launch/start_tea_demo.launch.py,
which owns the hand bridges and the skill controllers.
"""
from demo_stack import HAND_PERFORM_ACTIONS, ActivitySpec, main_for

SPEC = ActivitySpec(
    name="tea_demo",
    activity="tea_pour_duo_v2",
    stack="tea",
    description="Grip a tea can with the left arm and hand, pour it, put it back.",
    extra_actions=HAND_PERFORM_ACTIONS,
)

if __name__ == "__main__":
    main_for(SPEC, __doc__)
