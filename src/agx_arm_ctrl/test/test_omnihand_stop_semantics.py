"""A stop must stop the hand, not re-send where it was going.

Two defects sat here together. The stop commanded the cached pose, and the cache
holds the last *commanded* target rather than the measured one — so a stop during
motion re-sent the destination. And a unit safety stop closed the command gate
without touching the hardware, while this hand drives to a position on its own
once it has accepted one.

Both only show up mid-motion, which no test covered.
"""

from __future__ import annotations

import pytest
import rclpy
from agx_arm_msgs.msg import AgxUnitSafety

from agx_arm_ctrl.omnihand_bridge_node import OmniHandBridgeNode


class _MovingHand:
    """A hand that is travelling: commanded target ahead of measured pose."""

    def __init__(self, joint_count: int) -> None:
        self.commanded: list[list[float]] = []
        self.measured = [0.1] * joint_count

    def set_all_active_joint_angles(self, values) -> None:
        self.commanded.append(list(values))

    def get_all_active_joint_angles(self):
        return list(self.measured)


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
            "-p", "owner_liveness_grace_s:=0.0",
        ]
    )
    node = OmniHandBridgeNode()
    _cycle(node)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _unit_stop(stopped: bool = True, epoch: int = 1) -> AgxUnitSafety:
    msg = AgxUnitSafety()
    msg.epoch = epoch
    msg.stopped = stopped
    msg.reason = "test"
    msg.writer_id = "test_writer"
    return msg


def test_a_unit_stop_stops_the_hand_not_only_the_gate(bridge_node):
    stopped: list[bool] = []
    bridge_node.backend.stop = lambda: stopped.append(True)
    assert bridge_node._authority.claim("reactive:owner_a").accepted
    bridge_node._joint_states_command_callback(_command(bridge_node))
    assert bridge_node.pending_command is not None

    bridge_node._unit_safety_callback(_unit_stop())

    assert stopped == [True]
    assert bridge_node.pending_command is None


def test_a_unit_rearm_does_not_stop_anything(bridge_node):
    stopped: list[bool] = []
    bridge_node.backend.stop = lambda: stopped.append(True)

    bridge_node._unit_safety_callback(_unit_stop(stopped=False))

    assert stopped == []


def test_a_failing_stop_is_reported_not_swallowed(bridge_node):
    def boom() -> None:
        raise RuntimeError("bus down")

    bridge_node.backend.stop = boom

    bridge_node._unit_safety_callback(_unit_stop())  # must not raise


def _command(node, value: float = 0.4):
    from sensor_msgs.msg import JointState

    msg = JointState()
    msg.name = list(node.joint_names[:2])
    msg.position = [value, value]
    return msg


def test_the_sdk_stop_commands_the_measured_pose_not_the_target():
    """The defect in isolation: mid-motion, cache != reality."""
    from agx_arm_ctrl.omnihand_bridge_node import SdkOmniHandBackend

    backend = SdkOmniHandBackend.__new__(SdkOmniHandBackend)
    joint_count = 3
    backend.joint_names = ["a", "b", "c"]
    backend.hand = _MovingHand(joint_count)
    backend.positions = [0.9] * joint_count      # last commanded target
    backend.hand.measured = [0.2] * joint_count  # where the hand actually is
    backend.control_mode = "joint_state"
    backend.communication_fault = False
    backend.status_text = ""
    backend._saw_padded_vectors = False

    backend.stop()

    assert backend.hand.commanded, "stop sent nothing"
    sent = backend.hand.commanded[-1]
    assert sent == pytest.approx([0.2] * joint_count), (
        "stop re-sent the travel target instead of holding the measured pose"
    )
