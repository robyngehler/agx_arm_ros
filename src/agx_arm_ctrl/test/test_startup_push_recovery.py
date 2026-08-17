"""Startup recovery for an arm that boots with its CAN feedback push disabled.

Hardware finding (2026-07-24, left arm): the arm persists its linkage config
across power cycles, and both ``set_leader_mode`` and ``set_follower_mode``
leave ``enable_can_push`` DISABLED. Such an arm boots mute — it acknowledges
frames but pushes no feedback — so the startup firmware query times out and the
node used to ``exit(1)``, taking its own ``set_normal_mode`` service down with
it. Nothing could then bring the arm back through ROS.

``set_normal_mode`` is no longer how startup asks for feedback. Turning the push
on is a transport/reporting operation and has its own primitive; re-asserting
the linkage is the escalation for the case the push-only frame does not fix.
The two are tested apart here, because conflating them is what made a silent arm
depend on a mode switch to become observable.
"""

import pytest

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.sdk_worker import SdkWorker


_WORKERS: list[SdkWorker] = []


@pytest.fixture(autouse=True)
def _shut_down_workers():
    """Every worker a test starts is joined again, so none leak into the next."""
    yield
    while _WORKERS:
        _WORKERS.pop().shutdown()


class _FakeLogger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def warn(self, msg):
        self.warns.append(msg)

    def error(self, msg):
        self.errors.append(msg)

    def info(self, *_a, **_k):
        pass


class _Reporting:
    """The push-bit enum the mode message carries."""

    ENABLE = 1
    DISABLE = 0
    INVALID = 255


class _ModeMsg:
    class Enums:
        CanActiveMsgReporting = _Reporting

    def __init__(self):
        self.enable_can_push = _Reporting.INVALID
        self.move_mode = 0
        self.ctrl_mode = 1


class _MuteArm:
    """Answers nothing until the feedback push is turned back on.

    ``push_only_works`` decides which of the two bootstrap steps is enough: an
    arm that merely has the push bit off recovers from the push-only frame,
    while one holding a persisted leader/follower linkage needs the mode
    re-assert. Both exist on hardware and they are not the same failure.
    """

    def __init__(self, *, recovers=True, raises=False, push_only_works=False,
                 supports_push=True):
        self.pushing = False
        self.recovers = recovers
        self.raises = raises
        self.push_only_works = push_only_works
        self.normal_mode_calls = 0
        self.push_frames = []
        if supports_push:
            self._msg_mode = _ModeMsg()

    def _set_mode(self):
        self.push_frames.append(self._msg_mode.enable_can_push)
        if self._msg_mode.enable_can_push == _Reporting.ENABLE and self.push_only_works:
            self.pushing = True

    def get_firmware(self):
        return {"software_version": "1.11"} if self.pushing else None

    def set_normal_mode(self):
        self.normal_mode_calls += 1
        if self.raises:
            raise RuntimeError("bus error")
        if self.recovers:
            self.pushing = True


def _node(arm, *, is_nero=True, transport=True) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    node.is_nero = is_nero
    node.enable_timeout = 0.05
    node.feedback_timeout = 1.0
    node.firmware = None
    node._leader_mode_active = True
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = 123.0
    node._transport_connected = transport
    node._last_good_feedback_monotonic = 0.0
    node._last_feedback_advance_monotonic = 0.0
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    # The bootstrap frame is a real SDK write and goes through the session
    # owner like every other one.
    node._sdk = SdkWorker("arm_test")
    _WORKERS.append(node._sdk)
    return node


def test_mute_arm_is_recovered_and_firmware_read_on_retry():
    arm = _MuteArm()
    node = _node(arm)

    node._wait_for_firmware()
    assert node.firmware is None          # boots mute: nothing answers

    node._recover_silent_arm()
    node._wait_for_firmware()

    assert arm.normal_mode_calls == 1
    assert node.firmware == {"software_version": "1.11"}
    assert node._leader_mode_active is False
    assert node._hand_window_push_silenced is False
    assert any("linkage" in w for w in node.logger.warns)


def test_push_only_bootstrap_wakes_an_arm_without_a_mode_switch():
    """The primary path: feedback comes back without commanding a mode."""
    arm = _MuteArm(push_only_works=True)
    node = _node(arm)

    assert node._ensure_feedback_push_enabled("startup", force=True) is True
    node._wait_for_firmware()

    assert node.firmware == {"software_version": "1.11"}
    assert arm.push_frames == [_Reporting.ENABLE]
    assert arm.normal_mode_calls == 0, "a mode switch is not how feedback starts"


def test_bootstrap_does_not_depend_on_the_local_hand_window_flag():
    """§7.1: the flag records OUR silencing, not the firmware's actual state.

    An arm that booted mute has its push off while this node's flag says it
    silenced nothing, so a bootstrap that trusted the flag would send no frame
    at all — which is the state the whole path exists to leave.
    """
    arm = _MuteArm(push_only_works=True)
    node = _node(arm)
    node._hand_window_push_silenced = False

    assert node._ensure_feedback_push_enabled("startup", force=True) is True
    assert arm.push_frames == [_Reporting.ENABLE]


def test_bootstrap_without_force_leaves_an_unsilenced_push_alone():
    """The hand-window caller keeps its no-op: nothing silenced, nothing sent."""
    arm = _MuteArm(push_only_works=True)
    node = _node(arm)
    node._hand_window_push_silenced = False

    assert node._ensure_feedback_push_enabled("resume") is True
    assert arm.push_frames == []


def test_bootstrap_needs_a_transport_and_says_so_without_one():
    arm = _MuteArm(push_only_works=True)
    node = _node(arm, transport=False)

    assert node._ensure_feedback_push_enabled("startup", force=True) is False
    assert arm.push_frames == []
    assert any("no CAN transport" in w for w in node.logger.warns)


def test_bootstrap_reports_an_sdk_that_cannot_carry_the_push_bit():
    arm = _MuteArm(supports_push=False)
    node = _node(arm)

    assert node._ensure_feedback_push_enabled("startup", force=True) is False
    assert any("cached mode message" in w for w in node.logger.warns)


def test_recovery_is_skipped_for_non_nero_arms():
    # set_normal_mode is a Nero-only API; a Piper must not be poked with it.
    arm = _MuteArm()
    node = _node(arm, is_nero=False)
    node._recover_silent_arm()
    assert arm.normal_mode_calls == 0


def test_a_failing_recovery_is_reported_and_does_not_raise():
    # Startup must still reach its honest "not answering on CAN" exit path.
    arm = _MuteArm(raises=True)
    node = _node(arm)
    node._recover_silent_arm()
    node._wait_for_firmware()
    assert node.firmware is None
    assert any("startup recovery failed" in e for e in node.logger.errors)


def test_wait_for_firmware_does_not_keep_a_stale_result():
    arm = _MuteArm(recovers=False)
    node = _node(arm)
    node.firmware = {"software_version": "stale"}
    node._wait_for_firmware()
    assert node.firmware is None
