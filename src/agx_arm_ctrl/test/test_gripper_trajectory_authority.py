"""Gripper trajectory admission and the width the driver derives from a goal.

Covers the pure parts: what a trajectory means as an opening width, and what the
gripper's own device authority admits. The action server's ROS plumbing is not
exercised here — it needs a live driver on the other end.
"""
import pytest

from agx_arm_ctrl.device_authority import CommandStamp, DeviceAuthority, UnitSafety
from agx_arm_ctrl.gripper_follow_joint_trajectory import (
    GripperFollowJointTrajectoryBridge as Bridge,
)


def _width_from(names, positions):
    return Bridge._width_from(None, names, positions)


def test_width_comes_from_either_finger():
    """Each finger travels half the opening and the second mirrors the first."""
    assert _width_from(["right_arm_gripper_joint1"], [0.035]) == pytest.approx(0.07)
    assert _width_from(["right_arm_gripper_joint2"], [-0.035]) == pytest.approx(0.07)


def test_width_ignores_joints_it_does_not_know():
    assert _width_from(["right_arm_joint1", "some_other_joint"], [0.1, 0.2]) is None


def test_width_survives_a_prefix_it_was_not_told_about():
    """The driver matches by suffix, so a goal from any arm resolves."""
    assert _width_from(["left_arm_gripper_joint1"], [0.025]) == pytest.approx(0.05)


def _ready_authority():
    unit = UnitSafety("test_unit", writer=True)
    authority = DeviceAuthority("test_gripper", unit)
    authority.rearm(verified=True, detail="test")
    return authority


def test_unclaimed_gripper_admits_nothing():
    """Fail-closed: a gripper nobody holds executes nothing."""
    authority = _ready_authority()
    stamp = CommandStamp("someone", authority.device_epoch, 0, 1)
    assert not authority.admit(stamp)


def test_a_command_from_the_previous_owner_is_refused_after_handover():
    authority = _ready_authority()
    assert authority.claim("trajectory:a")
    stale = CommandStamp("trajectory:a", authority.device_epoch, 0, 1)

    authority.release("trajectory:a")
    assert authority.claim("trajectory:b")
    # The claim advanced the generation, so the stamp the first owner would have
    # sent under is no longer executable.
    assert not authority.admit(stale)


def test_a_reordered_command_is_refused():
    authority = _ready_authority()
    assert authority.claim("trajectory:a")
    epoch = authority.device_epoch
    assert authority.admit(CommandStamp("trajectory:a", epoch, 0, 2))
    assert not authority.admit(CommandStamp("trajectory:a", epoch, 0, 1))
