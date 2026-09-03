#!/usr/bin/env python3
"""Bring up the tea-demo stack and run tea_pour_duo_v2.

Twenty-one operator steps; seven of them replay a taught path and are therefore
not resume points. The right hand is out of service and is commanded nowhere.

Not to be confused with the ROS launch file of the same name, which this starts:
agx_arm_coordination/launch/start_tea_demo.launch.py owns the hand bridges and
the skill controllers.
"""
from demo_stack import TEA_DEMO, main_for

if __name__ == "__main__":
    main_for(TEA_DEMO, __doc__)
