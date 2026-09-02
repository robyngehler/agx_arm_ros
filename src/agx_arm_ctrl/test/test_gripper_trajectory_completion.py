"""What the gripper trajectory server calls success, and what it refuses.

The device closes its own loop on width, so the server decides the goal from the
readback. Settling alone cannot carry that decision: a gripper that never got the
command is still too. These drive ``_await_outcome`` over scripted status
sequences, without a ROS graph.
"""
import pytest

from agx_arm_ctrl import gripper_follow_joint_trajectory as module
from agx_arm_ctrl.gripper_follow_joint_trajectory import (
    GripperFollowJointTrajectoryBridge as Bridge,
)


class _Status:
    """One GripperStatus sample, healthy unless told otherwise."""

    def __init__(self, width, *, enabled=True, error=False, force=0.5):
        self.width = width
        self.force = force
        self.driver_enable_status = enabled
        self.driver_error_status = error
        self.voltage_too_low = False
        self.motor_overheating = False
        self.driver_overcurrent = False
        self.driver_overheating = False


class _Goal:
    def __init__(self, cancel_after=None):
        self._cancel_after = cancel_after
        self._reads = 0

    @property
    def is_cancel_requested(self):
        self._reads += 1
        return self._cancel_after is not None and self._reads > self._cancel_after


class _Bridge(Bridge):
    """The bridge's decision logic over a scripted readback, no node."""

    def __init__(self, widths, *, status_factory=_Status):
        # Deliberately not calling Node.__init__: this exercises the completion
        # rule, which needs parameters and a status source and nothing else.
        self.progress_tolerance_m = 0.002
        self.progress_timeout_s = 0.3
        self.delivery_timeout_s = 1.0
        self.settle_epsilon_m = 0.0005
        self.settle_time_s = 0.06
        self._widths = list(widths)
        self._status_factory = status_factory

    def _fresh_status(self):
        if not self._widths:
            return None
        width = self._widths[0] if len(self._widths) == 1 else self._widths.pop(0)
        return self._status_factory(width)


@pytest.fixture(autouse=True)
def _ros_is_up(monkeypatch):
    """``rclpy.ok()`` gates the wait loop; there is no context in a unit test."""
    monkeypatch.setattr(module.rclpy, "ok", lambda *a, **k: True)


def _outcome(widths, start, target, **kwargs):
    bridge = _Bridge(widths)
    for key, value in kwargs.items():
        setattr(bridge, key, value)
    outcome, detail = bridge._await_outcome(_Goal(), start, target)
    return outcome, detail


def test_a_target_already_reached_succeeds_without_waiting():
    outcome, detail = _outcome([0.0500], 0.0500, 0.0505)
    assert outcome == "arrived"
    assert "already at" in detail


def test_travel_then_standstill_is_a_grasp():
    """Moved measurably toward the target, then stopped: the object is there."""
    widths = [0.050, 0.040, 0.031, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300]
    outcome, detail = _outcome(widths, 0.050, 0.0)
    assert outcome == "settled"
    assert "0.0300 m" in detail


def test_a_command_that_never_moved_the_jaws_is_not_a_grasp():
    """The failure the settle test alone reported as success."""
    outcome, detail = _outcome([0.050] * 40, 0.050, 0.0)
    assert outcome == "no_progress"
    assert "still" in detail


def test_motion_away_from_the_target_is_not_progress():
    """Something else moved the jaws; that is not this command executing."""
    outcome, _ = _outcome([0.050, 0.056, 0.060, 0.060, 0.060, 0.060], 0.050, 0.0)
    assert outcome == "no_progress"


def test_a_disabled_driver_fails_instead_of_reading_as_still():
    bridge = _Bridge([0.050], status_factory=lambda w: _Status(w, enabled=False))
    outcome, detail = bridge._await_outcome(_Goal(), 0.050, 0.0)
    assert outcome == "faulted"
    assert "not enabled" in detail


def test_a_fault_bit_fails_and_names_itself():
    bridge = _Bridge([0.050], status_factory=lambda w: _Status(w, error=True))
    outcome, detail = bridge._await_outcome(_Goal(), 0.050, 0.0)
    assert outcome == "faulted"
    assert "driver_error" in detail


def test_a_readback_that_never_arrives_fails_as_stale():
    outcome, detail = _outcome([], 0.050, 0.0)
    assert outcome == "stale"


def test_jaws_that_never_stop_moving_fail_on_the_deadline():
    class _Ramp(_Bridge):
        def _fresh_status(self):
            self._widths.append(self._widths[-1] - 0.001)
            return _Status(self._widths[-1])

    bridge = _Ramp([0.050])
    outcome, detail = bridge._await_outcome(_Goal(), 0.050, -1.0)
    assert outcome == "moving"
    assert "still moving" in detail


def test_a_cancel_during_travel_ends_the_goal_as_canceled():
    """Observable while waiting, not only before: the wait is where a cancel lands."""
    bridge = _Bridge([0.050] * 40)
    outcome, detail = bridge._await_outcome(_Goal(cancel_after=2), 0.050, 0.0)
    assert outcome == "canceled"
    assert "cancel" in detail


def test_a_wider_tolerance_accepts_a_shorter_travel():
    """The two halves are one number, so they cannot leave a gap between them."""
    widths = [0.050, 0.0495, 0.0490, 0.0490, 0.0490, 0.0490, 0.0490]
    assert _outcome(widths, 0.050, 0.0)[0] == "no_progress"
    assert _outcome(widths, 0.050, 0.0, progress_tolerance_m=0.0005)[0] == "settled"
