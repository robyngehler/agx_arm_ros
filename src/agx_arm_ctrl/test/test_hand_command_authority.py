"""The hand bridge admits on the authority a command arrived with (phase 4D).

Before this, the bridge built the stamp from its OWN current epoch and its OWN
counter at the moment a command arrived, then checked that stamp against the
same state it came from. The stale-epoch and out-of-order checks therefore
compared each value with itself and passed unconditionally: the code was present
and could never refuse anything. Identity has to travel with the command.
"""

from __future__ import annotations

import pytest
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode
from agx_arm_msgs.msg import (
    AuthorizedJointTrajectory,
    DeviceCommandStamp,
    HandJointTarget,
)


OWNER = "reactive:test_owner"
TRAJ_OWNER = "trajectory:test_owner"


@pytest.fixture()
def bridge():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "joint_read_rate:=20.0",
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _claim(node, owner=OWNER):
    """Bring the device to READY under ``owner`` and return its generations.

    Read after the rearm, not after the claim: rearming advances the device
    epoch too, so a stamp built from the claim's value would be stale before the
    first command — which is exactly what the bridge is now able to notice.
    """
    node._authority.go_standby("connected")
    assert node._authority.claim(owner).accepted
    assert node._authority.rearm(verified=True, detail="test").accepted
    snap = node._authority.snapshot()
    return snap.device_epoch, snap.unit_safety_epoch


def _stamp(owner, device_epoch, unit_epoch, sequence):
    stamp = DeviceCommandStamp()
    stamp.owner_id = owner
    stamp.device_epoch = device_epoch
    stamp.unit_safety_epoch = unit_epoch
    stamp.sequence = sequence
    return stamp


def _target(node, owner, device_epoch, unit_epoch, sequence):
    msg = HandJointTarget()
    msg.authority = _stamp(owner, device_epoch, unit_epoch, sequence)
    msg.joint_names = list(node.joint_names)
    msg.positions = [0.1] * len(node.joint_names)
    return msg


def _trajectory(node, owner, device_epoch, unit_epoch, sequence):
    msg = AuthorizedJointTrajectory()
    msg.authority = _stamp(owner, device_epoch, unit_epoch, sequence)
    traj = JointTrajectory()
    traj.joint_names = list(node.joint_names)
    point = JointTrajectoryPoint()
    point.positions = [0.1] * len(node.joint_names)
    traj.points = [point]
    msg.trajectory = traj
    return msg


def test_a_correctly_stamped_target_is_admitted(bridge):
    device_epoch, unit_epoch = _claim(bridge)

    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch, 1)
    )

    assert bridge.pending_command is not None, "a valid stamped command was refused"


def test_a_stale_device_epoch_is_refused(bridge):
    """The check that could never fire before: identity from a previous era."""
    device_epoch, unit_epoch = _claim(bridge)
    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch, 1)
    )
    bridge.pending_command = None

    # The device changes hands, which advances the epoch under the old owner.
    bridge._authority.release(OWNER)
    new_epoch, unit_epoch = _claim(bridge)
    assert new_epoch != device_epoch

    # A command issued under the previous claim arrives late.
    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch, 2)
    )

    assert bridge.pending_command is None, (
        "a command stamped under a superseded device epoch reached the hand"
    )


def test_an_out_of_order_sequence_is_refused(bridge):
    """A reordered command must not overwrite a newer target."""
    device_epoch, unit_epoch = _claim(bridge)
    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch, 5)
    )
    assert bridge.pending_command is not None
    bridge.pending_command = None

    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch, 4)
    )

    assert bridge.pending_command is None, (
        "a command that arrived out of order was executed"
    )


def test_a_foreign_owner_is_refused(bridge):
    device_epoch, unit_epoch = _claim(bridge)

    bridge._hand_joint_target_callback(
        _target(bridge, "reactive:someone_else", device_epoch, unit_epoch, 1)
    )

    assert bridge.pending_command is None, "a command from a non-owner was executed"


def test_a_stale_unit_safety_epoch_is_refused(bridge):
    device_epoch, unit_epoch = _claim(bridge)

    bridge._hand_joint_target_callback(
        _target(bridge, OWNER, device_epoch, unit_epoch + 1, 1)
    )

    assert bridge.pending_command is None, (
        "a command carrying an unknown unit-safety generation was executed"
    )


def test_an_unclaimed_hand_executes_nothing(bridge):
    """Fail-closed still holds for the stamped surfaces."""
    bridge._hand_joint_target_callback(_target(bridge, OWNER, 0, 0, 1))

    assert bridge.pending_command is None


def test_a_trajectory_carries_its_authority_through_the_same_gate(bridge):
    device_epoch, unit_epoch = _claim(bridge, TRAJ_OWNER)

    bridge._authorized_trajectory_callback(
        _trajectory(bridge, TRAJ_OWNER, device_epoch, unit_epoch, 1)
    )
    assert bridge.pending_command is not None
    bridge.pending_command = None

    bridge._authorized_trajectory_callback(
        _trajectory(bridge, TRAJ_OWNER, device_epoch - 1 if device_epoch else 99,
                    unit_epoch, 2)
    )
    assert bridge.pending_command is None, (
        "a trajectory stamped under the wrong device epoch reached the hand"
    )


def test_a_reactive_stamp_may_not_preempt_a_trajectory_owner(bridge):
    """Exclusivity is by device authority, and the surface must agree with it."""
    device_epoch, unit_epoch = _claim(bridge, TRAJ_OWNER)

    bridge._hand_joint_target_callback(
        _target(bridge, TRAJ_OWNER, device_epoch, unit_epoch, 1)
    )

    assert bridge.pending_command is None, (
        "a reactive surface command executed while a trajectory owner held the hand"
    )


def test_the_legacy_surfaces_are_not_subscribed_by_default():
    """A production bridge offers no unstamped way to move the hand.

    The bare surfaces cannot refuse a stale or reordered command — the bridge
    has to invent the identity it then checks — so leaving them subscribed meant
    every authority guarantee had a documented bypass sitting next to it.
    """
    rclpy.init(args=["--ros-args", "-p", "backend_type:=mock"])
    try:
        node = OmniHandBridgeNode()
        topics = {name for name, _types in node.get_topic_names_and_types()}
        subscribed = {
            name
            for name in topics
            if node.count_subscribers(name) and node.count_publishers(name) == 0
        }
        assert "/control/omnihand/joint_trajectory" not in subscribed
        assert "/control/joint_states" not in subscribed
        assert node.allow_legacy_hand_command_ingress is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_legacy_ingress_is_available_only_when_asked_for_and_says_so():
    """Kept for manual development, behind one explicit flag and a loud warning."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "allow_legacy_hand_command_ingress:=true",
        ]
    )
    try:
        node = OmniHandBridgeNode()
        assert node.allow_legacy_hand_command_ingress is True
        topics = {name for name, _types in node.get_topic_names_and_types()}
        assert "/control/omnihand/joint_trajectory" in topics
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_one_stamped_trajectory_produces_exactly_one_command(bridge):
    """One logical motion, one admission.

    While the executor published both the stamped and the bare copy, the bridge
    admitted the same motion twice — and the self-stamped copy advanced the very
    sequence watermark the stamped copy is judged against.
    """
    device_epoch, unit_epoch = _claim(bridge, TRAJ_OWNER)
    submitted = []
    bridge._submit_command = lambda *args, **kwargs: submitted.append(args)

    bridge._authorized_trajectory_callback(
        _trajectory(bridge, TRAJ_OWNER, device_epoch, unit_epoch, 1)
    )

    assert len(submitted) == 1


def test_a_self_stamped_copy_can_starve_the_stamped_path(bridge):
    """Why the dual publish had to go, stated as the failure it caused.

    The bridge's own counter and the executor's counter feed one watermark. A
    self-stamped command that lands first takes the sequence the real command
    was going to use, and the real one is then refused as out of order — a
    motion silently dropped by the mechanism meant to protect it.
    """
    device_epoch, unit_epoch = _claim(bridge, TRAJ_OWNER)

    # The legacy path, stamping itself from the bridge's current state.
    assert bridge._admit_command("joint_trajectory")[0] is True

    # The executor's own first command, at its own sequence 1.
    admitted, refusal = bridge._admit_command(
        "authorized_trajectory", _stamp(TRAJ_OWNER, device_epoch, unit_epoch, 1)
    )
    assert admitted is False
    assert "sequence" in refusal
