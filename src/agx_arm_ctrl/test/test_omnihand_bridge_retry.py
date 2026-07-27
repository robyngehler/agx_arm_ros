"""Unit tests for the bridge command verify-and-retry layer (mock backend)."""

from __future__ import annotations

import pytest
import rclpy
from sensor_msgs.msg import JointState

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode


@pytest.fixture()
def bridge_node():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "backend_type:=mock",
            "-p", "command_retry_period_s:=0.05",
            "-p", "joint_read_rate:=1000.0",
        ]
    )
    node = OmniHandBridgeNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _command_msg(node: OmniHandBridgeNode, value: float = 0.3) -> JointState:
    msg = JointState()
    msg.name = list(node.joint_names[:2])
    msg.position = [value, value]
    return msg


def test_command_verifies_and_clears_pending(bridge_node):
    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command is not None
    assert bridge_node.pending_command["attempts"] == 1

    # mock backend applies targets instantly; a fresh readback verifies them
    bridge_node._publish_feedback()
    bridge_node._command_retry_tick()

    assert bridge_node.pending_command is None


def test_arm_only_joint_state_is_ignored(bridge_node):
    msg = JointState()
    msg.name = ["joint1", "joint2"]
    msg.position = [0.1, 0.2]

    bridge_node._joint_states_command_callback(msg)

    assert bridge_node.pending_command is None


def test_failed_sends_retry_until_attempts_exhausted(bridge_node):
    def rejecting_apply(target_map, control_mode):
        raise RuntimeError("bus congested")

    bridge_node.backend.apply_joint_targets = rejecting_apply

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command is not None

    for expected_attempts in range(2, bridge_node.command_retry_max_attempts + 1):
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        # An attempt is only spent once a readback could have judged the last
        # send, so the poll has to run for the retry loop to advance.
        bridge_node._publish_feedback()
        bridge_node._command_retry_tick()
        if bridge_node.pending_command is not None:
            assert bridge_node.pending_command["attempts"] == expected_attempts

    # exhausted: one more tick gives up instead of re-sending forever
    if bridge_node.pending_command is not None:
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        bridge_node._publish_feedback()
        bridge_node._command_retry_tick()
    assert bridge_node.pending_command is None
    assert bridge_node._command_delivery_failed is True


def test_attempt_is_not_spent_without_a_readback_opportunity(bridge_node):
    """The attempt budget is spent on evidence, not on the clock.

    Regression for the hardware failure of 2026-07-24: one 请求超时 put the
    backend into fault backoff (one probe every fault_poll_interval_s) while
    the retry timer kept firing every command_retry_period_s. All 8 attempts
    burned in 2.4 s with at most one readback in between, so the target was
    declared lost inside the very hand window opened to deliver it.
    """
    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node.pending_command["attempts"] == 1

    # No _publish_feedback(): no readback has landed since the send.
    for _ in range(20):
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        bridge_node._command_retry_tick()

    assert bridge_node.pending_command is not None
    assert bridge_node.pending_command["attempts"] == 1
    assert bridge_node._command_delivery_failed is False


def test_pending_command_keeps_the_probe_at_retry_cadence_under_fault_backoff(
    bridge_node,
):
    """Fault backoff must not starve the readback that ends the retries.

    Backing off is right for idle polling during an error storm, but a pending
    command is already re-sending — the probe is the only thing that can
    confirm it and STOP that traffic.
    """
    bridge_node.fault_poll_interval_s = 2.0
    bridge_node._fault_backoff_active = True
    bridge_node.joint_read_min_interval_s = 0.05
    bridge_node.command_retry_period_s = 0.3

    bridge_node.pending_command = None
    assert bridge_node._effective_read_interval() == 2.0

    bridge_node._joint_states_command_callback(_command_msg(bridge_node))
    assert bridge_node._effective_read_interval() == 0.3


def test_unverified_target_retries_then_gives_up(bridge_node):
    bridge_node._joint_states_command_callback(_command_msg(bridge_node))

    # feedback never reaches the target (e.g. fingers blocked by contact)
    bridge_node.backend.positions = [99.0] * len(bridge_node.joint_names)

    attempts_seen = []
    while bridge_node.pending_command is not None:
        bridge_node.pending_command["last_send_monotonic"] -= 10.0
        bridge_node._publish_feedback()
        attempts_seen.append(bridge_node.pending_command["attempts"])
        bridge_node._command_retry_tick()
        # backend caches the commanded target optimistically; force the
        # "still not there" readback again for the next round
        bridge_node.backend.positions = [99.0] * len(bridge_node.joint_names)

    assert max(attempts_seen) == bridge_node.command_retry_max_attempts


def test_joint_read_rate_throttles_backend_polling(bridge_node):
    read_calls = []
    original_read = bridge_node.backend.read_joint_state

    def counting_read():
        read_calls.append(1)
        return original_read()

    import time

    bridge_node.backend.read_joint_state = counting_read
    bridge_node.joint_read_min_interval_s = 3600.0
    # force the first poll to be due regardless of system uptime (monotonic
    # starts near 0 after boot, so `last = 0.0` alone is not reliably stale)
    bridge_node.last_joint_read_monotonic = time.monotonic() - 7200.0

    bridge_node._publish_feedback()  # stale -> reads
    bridge_node._publish_feedback()  # within the 1 h window -> cached
    bridge_node._publish_feedback()

    assert len(read_calls) == 1


def test_no_joint_state_published_before_first_successful_readback(bridge_node):
    published = []

    class FaultedBackend:
        communication_fault = True

        def read_joint_state(self):
            return [0.0] * len(bridge_node.joint_names)

        def read_status(self):
            return bridge_node.backend.read_status()

        def read_tactile(self):
            return bridge_node.backend.read_tactile()

    original_backend = bridge_node.backend
    faulted = FaultedBackend()
    faulted.read_status = original_backend.read_status
    faulted.read_tactile = original_backend.read_tactile
    bridge_node.backend = faulted
    bridge_node.last_good_joint_read_monotonic = 0.0
    bridge_node.hand_joint_states_pub.publish = published.append

    bridge_node._publish_feedback()

    assert published == []
