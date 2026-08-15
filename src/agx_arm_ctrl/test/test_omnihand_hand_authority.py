"""A hand has two production primitives and exactly one owner at a time.

Trajectory execution and reactive contact-seeking motion are both legitimate,
and neither may command a hand while the other holds it. The bridge is the
enforcement boundary because topic separation is not protection: two publishers
can reach one subscriber, and the bridge keeps a single pending target, so an
interleaved command replaces another commander's target and lets that commander
read the wrong verification as its own delivery.

Admission is fail-closed. An unclaimed hand executes nothing — which costs a
migration and is the point, because a default-open gate stays open for exactly
the callers nobody remembered to convert.
"""

from __future__ import annotations

import pytest
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_ctrl.omnihand_bridge_node import (
    OmniHandBridgeNode,
    owner_node_name,
    owner_primitive,
)

REACTIVE_OWNER = "reactive:omnihand_skill_controller"
TRAJECTORY_OWNER = "trajectory:omnihand_follow_joint_trajectory"


def _cycle(node) -> None:
    """One acquisition plus one publication.

    They used to be a single timer callback. Acquisition now runs on its own
    thread so no SDK call sits on the ROS executor, so a test that wants both
    halves asks for both — and a test about publication alone does not call the
    acquiring half.
    """
    node._acquire_once()
    node._publication_tick()


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "joint_read_rate:=20.0",
            # The owners named here are not running nodes; the liveness watchdog
            # has its own test that drives it explicitly.
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    _cycle(node)  # reach READY the way the runtime does
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _joint_command(node: OmniHandBridgeNode, value: float = 0.3) -> JointState:
    msg = JointState()
    msg.name = list(node.joint_names[:2])
    msg.position = [value, value]
    return msg


def _trajectory_command(node: OmniHandBridgeNode, value: float = 0.2) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(node.joint_names[:2])
    point = JointTrajectoryPoint()
    point.positions = [value, value]
    msg.points = [point]
    return msg


# --- the owner_id contract ---------------------------------------------------

def test_an_owner_id_declares_its_primitive_and_its_node():
    assert owner_primitive(TRAJECTORY_OWNER) == "trajectory"
    assert owner_node_name(TRAJECTORY_OWNER) == "omnihand_follow_joint_trajectory"
    assert owner_primitive(REACTIVE_OWNER) == "reactive"


def test_an_unstructured_owner_id_declares_no_primitive():
    """Nothing is inferred from a bare name; it simply claims no primitive."""
    assert owner_primitive("some_tool") == ""
    assert owner_node_name("some_tool") == ""
    assert owner_primitive("nonsense:node") == ""


# --- fail-closed admission ---------------------------------------------------

def test_an_unclaimed_hand_executes_nothing(bridge_node):
    bridge_node._joint_states_command_callback(_joint_command(bridge_node))

    assert bridge_node.pending_command is None


def test_the_owner_may_command(bridge_node):
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted

    bridge_node._joint_states_command_callback(_joint_command(bridge_node))

    assert bridge_node.pending_command is not None


# --- the two primitives never overlap ----------------------------------------

def test_a_trajectory_may_not_preempt_a_reactive_grasp(bridge_node):
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted

    bridge_node._joint_trajectory_callback(_trajectory_command(bridge_node))

    assert bridge_node.pending_command is None


def test_a_reactive_command_may_not_preempt_a_trajectory(bridge_node):
    assert bridge_node._authority.claim(TRAJECTORY_OWNER).accepted

    bridge_node._joint_states_command_callback(_joint_command(bridge_node))

    assert bridge_node.pending_command is None


def test_a_second_claim_is_refused_rather_than_queued(bridge_node):
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted

    verdict = bridge_node._authority.claim(TRAJECTORY_OWNER)

    assert not verdict.accepted


# --- handover is an epoch boundary -------------------------------------------

def test_a_handover_advances_the_device_epoch(bridge_node):
    before = bridge_node._authority.snapshot().device_epoch
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted
    assert bridge_node._authority.release(REACTIVE_OWNER).accepted
    assert bridge_node._authority.claim(TRAJECTORY_OWNER).accepted

    after = bridge_node._authority.snapshot().device_epoch
    assert after > before


def test_a_handover_drops_the_previous_owners_pending_command(bridge_node):
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted
    bridge_node._joint_states_command_callback(_joint_command(bridge_node))
    assert bridge_node.pending_command is not None

    bridge_node._authority.release(REACTIVE_OWNER)
    bridge_node._authority.claim(TRAJECTORY_OWNER)
    # The new owner's first command must not inherit the old one's retry state.
    bridge_node._joint_trajectory_callback(_trajectory_command(bridge_node))

    assert bridge_node.pending_command is not None
    assert bridge_node.pending_command["control_mode"] == "joint_trajectory"


def test_the_sequence_restarts_on_a_new_epoch(bridge_node):
    """A new owner starts its own sequence, not the watermark someone else set."""
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted
    for _ in range(5):
        bridge_node._joint_states_command_callback(_joint_command(bridge_node))
    assert bridge_node._command_sequence > 1

    bridge_node._authority.release(REACTIVE_OWNER)
    bridge_node._authority.claim(TRAJECTORY_OWNER)
    bridge_node._joint_trajectory_callback(_trajectory_command(bridge_node))

    assert bridge_node._command_sequence == 1


# --- a crashed owner ---------------------------------------------------------

def test_a_vanished_owner_is_revoked_and_the_hand_goes_uncommandable(bridge_node):
    """A claim outlives the process that took it; nothing else would free it."""
    bridge_node._owner_liveness_grace_s = 0.001
    # The check is rate-limited because it is a graph query; drive it directly.
    bridge_node._liveness_check_interval_s = 0.0
    assert bridge_node._authority.claim(REACTIVE_OWNER).accepted

    # The owner names a node that is not in the graph — a commander that died.
    bridge_node._check_owner_liveness()   # first sighting starts the grace
    bridge_node._owner_missing_since -= 1.0
    bridge_node._check_owner_liveness()   # grace elapsed

    assert bridge_node._authority.snapshot().owner_id == ""

    # And nothing is auto-transferred: the next commander must say so.
    bridge_node._joint_states_command_callback(_joint_command(bridge_node))
    assert bridge_node.pending_command is None

    assert bridge_node._authority.claim(TRAJECTORY_OWNER).accepted
    bridge_node._joint_trajectory_callback(_trajectory_command(bridge_node))
    assert bridge_node.pending_command is not None


def test_a_live_owner_is_never_revoked(bridge_node):
    """The bridge's own node name stands in for a commander that is present."""
    bridge_node._owner_liveness_grace_s = 0.001
    bridge_node._liveness_check_interval_s = 0.0
    live_owner = f"reactive:{bridge_node.get_name()}"
    assert bridge_node._authority.claim(live_owner).accepted

    for _ in range(5):
        bridge_node._check_owner_liveness()

    assert bridge_node._authority.snapshot().owner_id == live_owner
