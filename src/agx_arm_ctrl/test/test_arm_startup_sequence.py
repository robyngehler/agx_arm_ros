"""The order startup asks the arm for things, on an arm that starts mute.

Proposal §3. Startup used to be ordered around readiness: enable, then verify
the enable through feedback, then query the firmware — every step depending on a
feedback stream that a mute arm has not started yet, and only after all of it
had failed was anything sent that could start it.

It is now ordered around bootstrap. The push is asserted first and on the
transport alone; the enable request follows without waiting for feedback; the
push is asserted once more afterwards because on old firmware the push can
depend on the enabled state; the linkage re-assert is the last escalation. What
must NOT move with it is READY: no step here may make the arm commandable
without evidence.
"""

import pytest

from agx_arm_ctrl import agx_arm_ctrl_single_node as node_module
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.sdk_worker import SdkWorker


_WORKERS: list[SdkWorker] = []


@pytest.fixture(autouse=True)
def _shut_down_workers():
    yield
    while _WORKERS:
        _WORKERS.pop().shutdown()


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warns = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _Reporting:
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


class _StartupArm:
    """Records the order in which startup touches the arm.

    ``wakes_on`` names the step after which the firmware starts answering:
    ``"push"`` for an arm that only had the push bit off, ``"mode"`` for one
    holding a persisted leader/follower linkage, ``None`` for one that never
    answers at all.
    """

    joint_nums = 7

    def __init__(self, *, wakes_on="push"):
        self.wakes_on = wakes_on
        self.awake = False
        self.calls = []
        self._msg_mode = _ModeMsg()

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def _set_mode(self):
        bit = self._msg_mode.enable_can_push
        self.calls.append("push_on" if bit == _Reporting.ENABLE else "push_off")
        if bit == _Reporting.ENABLE and self.wakes_on == "push":
            self.awake = True

    def enable(self):
        self.calls.append("enable")
        return True

    def get_joint_enable_status(self, _index):
        self.calls.append("enable_readback")
        return self.awake

    def set_normal_mode(self):
        self.calls.append("set_normal_mode")
        if self.wakes_on == "mode":
            self.awake = True

    def get_firmware(self):
        self.calls.append("get_firmware")
        return {"software_version": "1.11"} if self.awake else None

    def set_speed_percent(self, _v):
        self.calls.append("set_speed_percent")

    def set_tcp_offset(self, _v):
        self.calls.append("set_tcp_offset")


def _node(arm, monkeypatch, *, auto_enable=True) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.arm_type = "nero"
    node.can_port = "can_test"
    node.is_nero = True
    node.is_piper = False
    node.auto_enable = auto_enable
    node.enable_timeout = 0.1
    node.feedback_timeout = 1.0
    node.speed_percent = 50
    node.tcp_offset = 0.0
    node.is_switch_seamlessly = True
    node.enable_flag = False
    node._enable_verified = False
    node.control_ready = False
    node._control_ready_logged = False
    node._transport_connected = False
    node._leader_mode_active = False
    node._hand_window_push_silenced = False
    node._hand_window_silence_started = 0.0
    node._last_good_feedback_monotonic = 0.0
    node._last_feedback_advance_monotonic = 0.0
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._measured = lambda a: a
    node._sdk = SdkWorker("arm_test")
    _WORKERS.append(node._sdk)

    monkeypatch.setattr(
        node_module, "create_agx_arm_config",
        lambda **_kw: {"joint_limits": {f"joint{i}": (-3.0, 3.0) for i in range(1, 8)}},
    )
    monkeypatch.setattr(
        node_module.AgxArmFactory, "create_arm", staticmethod(lambda _config: arm)
    )
    return node


def test_the_push_is_asserted_before_anything_is_asked_of_the_arm():
    """The bootstrap frame precedes every feedback-dependent step."""
    arm = _StartupArm(wakes_on="push")
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        node._init_agx_arm()

    assert arm.calls[0] == "connect"
    assert arm.calls[1] == "push_on", f"push must come first, got {arm.calls}"
    assert arm.calls.index("push_on") < arm.calls.index("enable")
    assert arm.calls.index("push_on") < arm.calls.index("get_firmware")


def test_a_push_only_wake_never_escalates_to_a_mode_switch():
    arm = _StartupArm(wakes_on="push")
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        node._init_agx_arm()

    assert "set_normal_mode" not in arm.calls
    assert node.firmware == {"software_version": "1.11"}


def test_a_persisted_linkage_escalates_only_after_a_second_push():
    """Both bootstrap steps are tried before the mode is touched."""
    arm = _StartupArm(wakes_on="mode")
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        node._init_agx_arm()

    assert arm.calls.count("push_on") >= 2, (
        f"the push is re-asserted after the enable, got {arm.calls}"
    )
    assert arm.calls.index("push_on") < arm.calls.index("set_normal_mode")
    assert node.firmware == {"software_version": "1.11"}


def test_the_enable_request_goes_out_before_any_feedback_exists():
    arm = _StartupArm(wakes_on="mode")   # still mute when the enable is sent
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        node._init_agx_arm()

    assert arm.calls.index("enable") < arm.calls.index("set_normal_mode")


def test_a_read_only_bringup_still_gets_its_feedback_push():
    """auto_enable=false inspects an arm without energising it — and still needs
    feedback to inspect it with."""
    arm = _StartupArm(wakes_on="push")
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp, auto_enable=False)
        node._init_agx_arm()

    assert "push_on" in arm.calls
    assert "enable" not in arm.calls


def test_an_arm_that_never_answers_does_not_become_ready():
    """§7.1: bootstrap commands execute, and READY still does not follow."""
    arm = _StartupArm(wakes_on=None)
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        with pytest.raises(SystemExit):
            node._init_agx_arm()

    assert "push_on" in arm.calls, "the bootstrap was attempted"
    assert node.firmware is None
    assert node.control_ready is False
    assert node._enable_verified is False
    assert node.enable_flag is False


def test_the_failure_says_which_half_did_not_happen():
    """A transport that exists and feedback that never came are different facts.

    Measured on hardware 2026-08-17: the arm bus showed TX=0 while a hand bus on
    the same host transmitted normally, so "the arm is not answering" and "our
    stack never transmitted" have to be distinguishable from the message.
    """
    arm = _StartupArm(wakes_on=None)
    with pytest.MonkeyPatch.context() as mp:
        node = _node(arm, mp)
        with pytest.raises(SystemExit):
            node._init_agx_arm()

    diagnostic = "\n".join(node.logger.errors)
    assert "Transport session: present" in diagnostic
    assert "feedback: none" in diagnostic
    assert "enable: unverified" in diagnostic
