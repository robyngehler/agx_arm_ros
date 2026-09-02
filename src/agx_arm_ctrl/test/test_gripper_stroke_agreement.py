"""The declared stroke and the range the driver enforces are one fact.

The registry carries the closure endpoints the catalogue and the teach manager
convert against; the wrapper carries the range the driver refuses outside of.
They are written in two files, so the agreement is checked rather than assumed —
a registry that drifted wider would send the driver a width it rejects, and one
that drifted narrower would silently shorten the stroke.
"""
from agx_arm_ctrl.effector.agx_gripper import AgxGripperWrapper
from agx_arm_ctrl.motion_registry import load_motion_registry


def test_the_registry_stroke_matches_the_range_the_driver_enforces():
    gripper = load_motion_registry().get("gripper", {})
    assert float(gripper["width_open_m"]) == AgxGripperWrapper.WIDTH_MAX
    assert float(gripper["width_closed_m"]) == AgxGripperWrapper.WIDTH_MIN


def test_the_finger_joints_are_declared_once_and_come_from_the_registry():
    gripper = load_motion_registry().get("gripper", {})
    assert gripper["canonical_joints"] == ["gripper_joint1", "gripper_joint2"]
