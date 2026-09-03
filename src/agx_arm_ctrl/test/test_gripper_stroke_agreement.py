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


def test_the_bridge_maps_the_fingers_the_registry_declares():
    """The trajectory server spells the finger joints itself.

    It cannot import the coordination helper — agx_arm_ctrl does not depend on
    agx_arm_coordination — so the two write the same mapping twice. The stroke
    is checked above; this checks the other half, the joints it applies to.
    """
    from agx_arm_ctrl.gripper_follow_joint_trajectory import (
        FINGER_SUFFIXES,
        FINGER_TO_WIDTH,
    )

    gripper = load_motion_registry().get("gripper", {})
    assert list(FINGER_SUFFIXES) == list(gripper["canonical_joints"])
    # Each finger travels half the opening, and the second mirrors the first.
    assert FINGER_TO_WIDTH == 2.0
