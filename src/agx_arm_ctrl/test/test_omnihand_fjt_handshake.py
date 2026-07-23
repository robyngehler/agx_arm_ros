"""Unit tests for the OmniHand FollowJointTrajectory step-and-settle handshake.

MoveIt hand execution goes straight to the FJT bridge, so wrapping every goal in
prepare_hand_window/resume_arm_control is what makes the hand own the shared side
bus under the always-on arm MIT (plan section 3). The node needs ROS to
construct, so tests build a bare instance via __new__ and stub the trigger call.
"""

from agx_arm_ctrl.omnihand_follow_joint_trajectory import (
    OmniHandFollowJointTrajectoryBridge,
)


class _FakeLogger:
    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


def _node(trigger_result):
    node = OmniHandFollowJointTrajectoryBridge.__new__(OmniHandFollowJointTrajectoryBridge)
    node.get_logger = lambda: _FakeLogger()
    node.handshake_enabled = True
    node._window_open = False
    node.prepare_client = object()
    node.resume_client = object()
    node._calls = []

    def _fake(_client, label):
        node._calls.append(label)
        return trigger_result

    node._call_trigger = _fake
    return node


def test_disabled_handshake_proceeds_without_window():
    node = _node((True, "ok"))
    node.handshake_enabled = False
    proceed, _ = node._open_hand_window()
    assert proceed is True
    assert node._window_open is False
    assert node._calls == []  # no service call when disabled


def test_absent_arm_service_proceeds_without_window():
    # ok is None -> service unavailable (hand-only bringup): proceed, no window.
    node = _node((None, "unavailable"))
    proceed, _ = node._open_hand_window()
    assert proceed is True
    assert node._window_open is False


def test_verified_hold_opens_the_window():
    node = _node((True, "held"))
    proceed, _ = node._open_hand_window()
    assert proceed is True
    assert node._window_open is True


def test_failed_prepare_aborts_and_leaves_window_closed():
    node = _node((False, "hold not verified"))
    proceed, _ = node._open_hand_window()
    assert proceed is False
    assert node._window_open is False


def test_close_resumes_only_when_a_window_was_opened():
    node = _node((True, "resumed"))
    # No window open -> no resume call.
    node._close_hand_window()
    assert node._calls == []
    # Window open -> resume is called and the flag clears.
    node._window_open = True
    node._close_hand_window()
    assert node._calls == ["resume_arm_control"]
    assert node._window_open is False
