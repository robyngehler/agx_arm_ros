"""Startup recovery for an arm that boots with its CAN feedback push disabled.

Hardware finding (2026-07-24, left arm): the arm persists its linkage config
across power cycles, and both ``set_leader_mode`` and ``set_follower_mode``
leave ``enable_can_push`` DISABLED. Such an arm boots mute — it acknowledges
frames but pushes no feedback — so the startup firmware query times out and the
node used to ``exit(1)``, taking its own ``set_normal_mode`` service down with
it. Nothing could then bring the arm back through ROS.
"""

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


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


class _MuteArm:
    """Answers nothing until set_normal_mode re-enables the push."""

    def __init__(self, *, recovers=True, raises=False):
        self.pushing = False
        self.recovers = recovers
        self.raises = raises
        self.normal_mode_calls = 0

    def get_firmware(self):
        return {"software_version": "1.11"} if self.pushing else None

    def set_normal_mode(self):
        self.normal_mode_calls += 1
        if self.raises:
            raise RuntimeError("bus error")
        if self.recovers:
            self.pushing = True


def _node(arm, *, is_nero=True) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    node.is_nero = is_nero
    node.enable_timeout = 0.05
    node.firmware = None
    node._leader_mode_active = True
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = 123.0
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
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
    assert any("push disabled" in w for w in node.logger.warns)


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
