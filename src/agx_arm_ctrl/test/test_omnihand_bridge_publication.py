"""Publication is driven by new data, not by a timer (mock backend).

The bridge used to rebuild and publish all three feedback messages on every
timer tick, at whatever rate the caller passed — and every bringup passed the
*arm's* 200 Hz. Its joints change at the readback rate (20 Hz) and its status
and tactile once a second, so nine out of ten messages carried nothing new.

These tests pin the three rules that replaced that, including the one that
protects the FollowJointTrajectory path: a settled command is announced
immediately, because the action holds its goal until it sees that verdict.
"""

from __future__ import annotations

import time

import pytest
import rclpy
from sensor_msgs.msg import JointState

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode


class _Recorder:
    """Counts what a publisher was asked to send."""

    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def __len__(self) -> int:
        return len(self.messages)


def _pretend_time_passed(node: OmniHandBridgeNode, seconds: float) -> None:
    """Shift every monotonic marker back, which is what elapsed time does.

    Moving one marker alone would test a state the node can never reach: the
    readback gate and the publication ceiling both measure against the same
    clock, so a test that opens one and not the other proves nothing.
    """
    for attr in (
        "last_joint_read_monotonic",
        "last_good_joint_read_monotonic",
        "_published_read_monotonic",
        "_last_joint_publish_monotonic",
        "_last_status_publish_monotonic",
        "_last_tactile_publish_monotonic",
    ):
        setattr(node, attr, getattr(node, attr) - seconds)


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "joint_read_rate:=20.0",
            "-p", "status_heartbeat_rate:=2.0",
            "-p", "command_retry_period_s:=0.05",
            # No graph to look the owner up in: an in-process node is not a
            # running commander, and the liveness watchdog would revoke it.
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    # Fail-closed admission: a hand with no commander executes nothing, and one
    # that has not reported itself connected is not READY. One tick brings it
    # there; the claim is what the reactive primitive would take.
    node._feedback_tick()
    assert node._authority.claim("reactive:test_owner").accepted
    node.hand_joint_states_pub = _Recorder()
    node.status_pub = _Recorder()
    node.tactile_pub = _Recorder()
    # That priming tick also consumed a readback and a publication slot. Wind the
    # bookkeeping back so each test starts from a bridge with something to say,
    # rather than one that happens to be mid-interval.
    _pretend_time_passed(node, 10.0)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _command_msg(node: OmniHandBridgeNode, value: float = 0.3) -> JointState:
    msg = JointState()
    msg.name = list(node.joint_names[:2])
    msg.position = [value, value]
    return msg


def test_a_tick_without_a_new_readback_publishes_no_joint_state(bridge_node):
    bridge_node._feedback_tick()
    assert len(bridge_node.hand_joint_states_pub) == 1

    # Ticks inside the readback interval have nothing new to say. Republishing
    # here is what gave a stale cache a fresh header stamp.
    for _ in range(20):
        bridge_node._feedback_tick()

    assert len(bridge_node.hand_joint_states_pub) == 1


def test_a_new_readback_publishes_exactly_one_joint_state(bridge_node):
    bridge_node._feedback_tick()
    published = len(bridge_node.hand_joint_states_pub)

    _pretend_time_passed(bridge_node, 0.06)  # past the 20 Hz readback interval
    bridge_node._feedback_tick()

    assert len(bridge_node.hand_joint_states_pub) == published + 1


def test_the_publish_ceiling_throttles_below_the_readback_rate(bridge_node):
    """`pub_rate` is a ceiling: it can throttle publication, never drive it."""
    bridge_node._publish_min_interval_s = 1.0  # 1 Hz ceiling
    bridge_node._feedback_tick()
    published = len(bridge_node.hand_joint_states_pub)

    _pretend_time_passed(bridge_node, 0.06)
    bridge_node._feedback_tick()

    assert len(bridge_node.hand_joint_states_pub) == published


def test_a_settled_command_is_announced_without_waiting_for_a_tick(bridge_node):
    bridge_node._feedback_tick()
    before = len(bridge_node.status_pub)

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command is not None
    # The command going pending is itself a change worth publishing: the action
    # only trusts a status sample stamped after its own publish.
    assert len(bridge_node.status_pub) > before
    assert bridge_node.status_pub.messages[-1].command_pending is True

    pending_announced = len(bridge_node.status_pub)
    _pretend_time_passed(bridge_node, 0.06)
    bridge_node._feedback_tick()           # readback lands, verifying the target
    bridge_node._command_retry_tick()      # verdict: delivered

    assert bridge_node.pending_command is None
    assert len(bridge_node.status_pub) > pending_announced
    assert bridge_node.status_pub.messages[-1].command_pending is False


def test_status_is_not_republished_while_nothing_changes(bridge_node):
    bridge_node._feedback_tick()
    published = len(bridge_node.status_pub)

    # Well inside the 2 Hz heartbeat, with no command and no fault.
    for _ in range(20):
        bridge_node._feedback_tick()

    assert len(bridge_node.status_pub) == published


def test_the_heartbeat_still_publishes_status_when_nothing_changes(bridge_node):
    bridge_node._feedback_tick()
    published = len(bridge_node.status_pub)

    bridge_node._last_status_publish_monotonic = time.monotonic() - 10.0
    bridge_node._feedback_tick()

    assert len(bridge_node.status_pub) == published + 1


def test_tactile_is_published_at_its_read_interval_not_every_tick(bridge_node):
    bridge_node._feedback_tick()
    published = len(bridge_node.tactile_pub)
    assert published == 1

    for _ in range(50):
        bridge_node._feedback_tick()

    assert len(bridge_node.tactile_pub) == published


def test_the_timer_is_paced_by_acquisition_not_by_the_publish_ceiling():
    """A caller passing the arm's 200 Hz must not make the hand run at 200 Hz."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "pub_rate:=200.0",
            "-p", "joint_read_rate:=20.0",
        ]
    )
    node = OmniHandBridgeNode()
    try:
        periods = [timer.timer_period_ns / 1e9 for timer in node.timers]
        # 2x oversampling of the 20 Hz readback, not 1/200.
        assert min(periods) == pytest.approx(0.025)
    finally:
        node.destroy_node()
        rclpy.shutdown()
