"""Unit tests for the feedback-verified emergency stop.

A stop command alone proves nothing: the SDK silently drops it under ENOBUFS and
still returns success, so the e-stop must confirm the arm actually settled and
must never report a phantom success.

Until 2026-08-11 these tests asserted against ``get_motor_states().msg.velocity``
— the field every Nero driver tier overwrites with ``0.0`` before returning it.
The suite therefore proved the check worked on a value that is never real, while
the live path reported "confirmed stopped" on the first poll no matter how fast
the arm was moving. Speed is now differentiated from timestamped joint
positions, and the tests drive that path.

Test level: **L1**. Nothing here says the derived speed matches the arm's true
motion — that comparison needs hardware (0E).

The node connects to hardware in ``__init__``, so these tests build a bare
instance via ``__new__`` and drive the pure verification helpers directly.
"""

from types import SimpleNamespace

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.sdk_worker import SdkWorker as _SdkWorker


class _PositionArm:
    """Driver stub advancing joint positions at a fixed speed.

    ``speed_rad_s=0`` holds still; ``timestamp_advances=False`` reproduces a
    stalled bus that keeps handing back the same frame.
    """

    def __init__(self, speed_rad_s, *, timestamp_advances=True, joints=7,
                 feedback=True, frame_period_s=0.02):
        self._speed = speed_rad_s
        self._advances = timestamp_advances
        self._joints = joints
        self._feedback = feedback
        self._period = frame_period_s
        self._frame = 0

    def get_joint_angles(self):
        if not self._feedback:
            return None
        self._frame += 1
        elapsed = self._frame * self._period
        timestamp = elapsed if self._advances else 0.0
        position = self._speed * elapsed
        return SimpleNamespace(
            timestamp=timestamp,
            msg=[position] * self._joints,
            hz=1.0 / self._period,
        )


def _node(arm) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.arm_joint_count = 7
    node.agx_arm = arm
    # Stop verification reads the arm on the safety lane now, so it needs the
    # session owner rather than a direct handle: a verification read queued
    # behind the control stream would report "cannot tell" for the wrong reason.
    node._sdk = _SdkWorker("arm_test")
    node.feedback_timeout = 1.0
    return node


def test_settled_when_arm_is_actually_still():
    node = _node(_PositionArm(speed_rad_s=0.0))
    result = node._arm_velocities_settled(timeout_s=0.3, poll_s=0.01)
    assert result.verified is True
    assert result.evidence is True


def test_not_settled_when_joint_is_moving():
    # 0.5 rad/s is well above the 0.05 rad/s threshold.
    node = _node(_PositionArm(speed_rad_s=0.5))
    result = node._arm_velocities_settled(timeout_s=0.3, poll_s=0.01)
    assert result.verified is False
    assert result.evidence is True, "a moving arm is evidence, not absence of it"


def test_moving_arm_is_not_reported_settled_by_the_sdk_velocity_field():
    """The regression that motivated this rework.

    The arm below moves at 2 rad/s. The old check read the SDK's velocity
    field, which is hardcoded to 0.0, and would have called this settled.
    """
    node = _node(_PositionArm(speed_rad_s=2.0))
    result = node._arm_velocities_settled(timeout_s=0.3, poll_s=0.01)
    assert result.settled is False
    assert result.verified is False


def test_no_evidence_when_feedback_missing():
    # A dead bus can never be confirmed stopped, and must not look like one.
    node = _node(_PositionArm(speed_rad_s=0.0, feedback=False))
    result = node._arm_velocities_settled(timeout_s=0.1, poll_s=0.01)
    assert result.verified is False
    assert result.evidence is False
    assert "no joint feedback" in result.detail


def test_no_evidence_when_feedback_timestamp_is_frozen():
    """A stalled frame must not be differentiated into a confident zero.

    Positions stop changing and the timestamp stops advancing — exactly what a
    dead bus looks like while the arm may still be coasting.
    """
    node = _node(_PositionArm(speed_rad_s=0.0, timestamp_advances=False))
    result = node._arm_velocities_settled(timeout_s=0.15, poll_s=0.01)
    assert result.verified is False
    assert result.evidence is False
    assert "did not advance" in result.detail


def test_verified_requires_both_settled_and_evidence():
    from agx_arm_ctrl.agx_arm_ctrl_single_node import StopVerification

    assert StopVerification(True, True, "").verified is True
    assert StopVerification(True, False, "").verified is False
    assert StopVerification(False, True, "").verified is False
