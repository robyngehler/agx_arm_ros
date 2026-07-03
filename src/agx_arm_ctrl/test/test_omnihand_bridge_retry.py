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
        bridge_node._command_retry_tick()
        if bridge_node.pending_command is not None:
            assert bridge_node.pending_command["attempts"] == expected_attempts

    # exhausted: one more tick gives up instead of re-sending forever
    if bridge_node.pending_command is not None:
        bridge_node._command_retry_tick()
    assert bridge_node.pending_command is None


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

    bridge_node.backend.read_joint_state = counting_read
    bridge_node.joint_read_min_interval_s = 3600.0
    bridge_node.last_joint_read_monotonic = 0.0

    bridge_node._publish_feedback()  # first poll always reads
    bridge_node._publish_feedback()
    bridge_node._publish_feedback()

    assert len(read_calls) == 1
