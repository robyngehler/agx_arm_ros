"""Unit tests for the feedback-verified emergency stop.

Guards the shared-CAN step-and-settle plan section 1.3.2 / Phase 2 item 4: a
stop command alone proves nothing (the SDK silently drops it under ENOBUFS and
still returns success), so the e-stop must confirm the arm actually settled in
feedback and never report a phantom success.

The node connects to hardware in ``__init__``, so these tests build a bare
instance via ``__new__`` and drive the pure verification helper directly.
"""

from types import SimpleNamespace

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _MotorArm:
    """Driver stub returning a fixed per-joint velocity (or no feedback)."""

    def __init__(self, velocity):
        # velocity: float applied to every joint, or None to simulate a dead bus
        self._velocity = velocity

    def get_motor_states(self, _joint_index):
        if self._velocity is None:
            return None
        return SimpleNamespace(msg=SimpleNamespace(velocity=self._velocity))


def _node(velocity) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.arm_joint_count = 7
    node.agx_arm = _MotorArm(velocity)
    return node


def test_settled_when_velocities_below_threshold():
    node = _node(velocity=0.0)
    assert node._arm_velocities_settled(timeout_s=0.1, poll_s=0.01) is True


def test_not_settled_when_joint_still_moving():
    node = _node(velocity=0.5)  # well above the 0.05 rad/s threshold
    assert node._arm_velocities_settled(timeout_s=0.1, poll_s=0.01) is False


def test_not_settled_when_feedback_missing():
    # A dead bus (get_motor_states -> None) can never be confirmed stopped.
    node = _node(velocity=None)
    assert node._arm_velocities_settled(timeout_s=0.1, poll_s=0.01) is False
