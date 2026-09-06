"""The coordinator refuses to dispatch into an arm that is not accepting motion.

A driver-side pose hold or fault makes the arm refuse every setpoint, and MoveIt
only discovers that as a tracking error partway through the motion — the arm has
been commanded by then, and the activity fails with ``error_code=-4`` on a goal
that never could have run. The node needs ROS to construct, so these build a
bare instance via ``__new__`` and feed the authority messages in directly.
"""

import pytest

from agx_arm_msgs.msg import AgxDeviceAuthority

from agx_arm_coordination.arm_executor import MoveGroupPlan
from agx_arm_coordination.coordinator_node import CoordinatorNode, DispatchError


def _coord():
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._arm_authority = {}
    node.arm_dry_run = False
    return node


def _authority(state, *, reason=""):
    msg = AgxDeviceAuthority()
    msg.state = state
    msg.motion_ready = state == AgxDeviceAuthority.STATE_READY
    msg.reason = reason
    return msg


def _plan(*joint_names):
    return MoveGroupPlan(
        action_id="test",
        robot_id="both_arms",
        planning_group="both_arms",
        joint_names=tuple(joint_names),
        target_positions=(0.0,) * len(joint_names),
        velocity_scaling=1.0,
        acceleration_scaling=1.0,
    )


_BOTH = ("left_arm_joint1", "right_arm_joint1")


def test_two_ready_arms_dispatch():
    node = _coord()
    node._on_arm_authority("left", _authority(AgxDeviceAuthority.STATE_READY))
    node._on_arm_authority("right", _authority(AgxDeviceAuthority.STATE_READY))

    node._require_arms_ready(_plan(*_BOTH))


def test_a_held_arm_refuses_the_dispatch():
    """The refusal the -4 replaced: before the arm is commanded, not after."""
    node = _coord()
    node._on_arm_authority("left", _authority(AgxDeviceAuthority.STATE_READY))
    node._on_arm_authority(
        "right",
        _authority(AgxDeviceAuthority.STATE_STANDBY, reason="pose hold (stale feedback)"),
    )

    with pytest.raises(DispatchError) as excinfo:
        node._require_arms_ready(_plan(*_BOTH))

    assert "right arm is in standby" in str(excinfo.value)
    assert "pose hold (stale feedback)" in str(excinfo.value)


def test_the_refusal_names_every_arm_that_is_not_ready():
    node = _coord()
    node._on_arm_authority("left", _authority(AgxDeviceAuthority.STATE_FAULTED))
    node._on_arm_authority("right", _authority(AgxDeviceAuthority.STATE_STANDBY))

    with pytest.raises(DispatchError) as excinfo:
        node._require_arms_ready(_plan(*_BOTH))

    assert "left arm is faulted" in str(excinfo.value)
    assert "right arm is in standby" in str(excinfo.value)


def test_an_arm_the_plan_does_not_command_is_not_checked():
    """A left-arm hold must not refuse a right-arm action."""
    node = _coord()
    node._on_arm_authority("left", _authority(AgxDeviceAuthority.STATE_STANDBY))
    node._on_arm_authority("right", _authority(AgxDeviceAuthority.STATE_READY))

    node._require_arms_ready(_plan("right_arm_joint1", "right_arm_joint2"))


def test_a_driver_that_has_not_reported_is_left_to_the_action_client():
    """No message on a latched topic means the driver is not up.

    Refusing here would report it as a hold; the action client's own "server not
    available" says what actually happened.
    """
    node = _coord()

    node._require_arms_ready(_plan(*_BOTH))
