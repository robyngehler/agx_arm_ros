"""Unit tests for the step-and-settle arm<->hand window handoff.

Covers the driver-level ``prepare_hand_window`` / ``resume_arm_control``
services (plan section 3): quiescing the arm into a verified normal-mode hold so
the OmniHand owns the shared side bus, and reopening the side afterwards. The
node connects to hardware in ``__init__``, so tests build a bare instance via
``__new__`` and mock the driver.
"""

import time
from types import SimpleNamespace

from std_srvs.srv import Trigger

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode

_CAN_CTRL = 0x01   # active holding mode (verified hold)
_STANDBY = 0x00    # idle — NOT a hold (e.g. V112 set_normal_mode no-op leaves it here)
_TEACHING = 0x02   # backdrivable

_MOVE_J = 0x01     # firmware runs its own position controller
_MOVE_MIT = 0x06   # firmware waits for host MIT commands (v111+; 0x04 before)

_PUSH_INVALID = 0x00   # mode frame byte 6: "do not touch the push"
_PUSH_ENABLE = 0x01
_PUSH_DISABLE = 0x02
_MOVE_MODE_NO_CHANGE = 255


class _Reporting:
    INVALID = _PUSH_INVALID
    ENABLE = _PUSH_ENABLE
    DISABLE = _PUSH_DISABLE


class _MotionMode:
    # Firmware-dependent: 0x06 from v111 on, 0x04 below. The fake mirrors a
    # v111+ driver so the code under test must ask the driver, not guess.
    MIT = _MOVE_MIT


class _FakeModeMsg:
    """The driver's cached mode frame (0x151) the push bit rides on."""

    class Enums:
        CanActiveMsgReporting = _Reporting
        MotionMode = _MotionMode

    def __init__(self):
        self.ctrl_mode = _CAN_CTRL
        self.move_mode = 0x01
        self.enable_can_push = _PUSH_INVALID


class _FakeLogger:
    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _FakeArm:
    def __init__(self, *, velocity=0.0, ctrl_mode=_CAN_CTRL, hz=1.0,
                 frame_ts=1.0, comm_err=None, mode_feedback=_MOVE_J,
                 push_live=False, disables_until_silent=1,
                 move_j_lands_after=0):
        # push_live models the real firmware: while the push runs, every read
        # sees a NEW frame timestamp, and it only stops once a DISABLE mode
        # frame actually lands. disables_until_silent > 1 models the frame
        # being dropped on a saturated bus, which is why silence is verified.
        self.push_live = push_live
        self.push_enabled = True
        self.disables_until_silent = disables_until_silent
        self._disable_count = 0
        self.velocity = velocity
        self.ctrl_mode = ctrl_mode
        self.mode_feedback = mode_feedback
        self.auto_motion_mode = False
        self.hz = hz
        self.frame_ts = frame_ts
        self.comm_err = comm_err
        self.ARM_STATUS = SimpleNamespace(
            CtrlMode=SimpleNamespace(TEACHING_MODE=_TEACHING)
        )
        self.normal_mode_called = False
        self.move_j_arg = None
        self.move_j_calls = 0
        # Lossy bus: the first `move_j_lands_after` MOVE-J frames are dropped
        # (mode_feedback stays as-is); the next one lands and flips it to J.
        self.move_j_lands_after = move_j_lands_after
        self.move_mit_calls = 0
        self._msg_mode = _FakeModeMsg()
        # (enable_can_push, move_mode) of every mode frame actually sent.
        self.mode_frames = []

    def _set_mode(self):
        push_bit = self._msg_mode.enable_can_push
        self.mode_frames.append((push_bit, self._msg_mode.move_mode))
        if not self.push_live:
            return
        if push_bit == _PUSH_DISABLE:
            self._disable_count += 1
            if self._disable_count >= self.disables_until_silent:
                self.push_enabled = False
        elif push_bit == _PUSH_ENABLE:
            self.push_enabled = True

    @property
    def push_frames(self):
        """Mode frames that actually changed the push bit."""
        return [f for f in self.mode_frames if f[0] != _PUSH_INVALID]

    def get_joint_angles(self):
        if self.push_live and self.push_enabled:
            self.frame_ts += 0.005
        return SimpleNamespace(msg=[0.1] * 7, hz=self.hz, timestamp=self.frame_ts)

    def get_motor_states(self, _i):
        return SimpleNamespace(msg=SimpleNamespace(velocity=self.velocity))

    def get_arm_status(self):
        return SimpleNamespace(
            msg=SimpleNamespace(
                ctrl_mode=self.ctrl_mode, mode_feedback=self.mode_feedback
            )
        )

    def set_normal_mode(self):
        self.normal_mode_called = True

    def move_j(self, q):
        self.move_j_arg = list(q)
        self.move_j_calls += 1
        if self.move_j_calls > self.move_j_lands_after:
            self.mode_feedback = _MOVE_J

    def set_motion_mode(self, *_a):
        pass

    def set_auto_set_motion_mode_enabled(self, enabled):
        self.auto_motion_mode = enabled

    def move_mit(self, **_k):
        self.move_mit_calls += 1

    def has_comm_error(self):
        return self.comm_err is not None

    def get_comm_error(self):
        return self.comm_err


def _node(arm: _FakeArm) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _FakeLogger()
    node.agx_arm = arm
    node.is_nero = True
    node.arm_joint_count = 7
    node._recovery_in_progress = False
    node._force_recovery = False
    node.enable_flag = True
    node._hand_window_active = False
    node.is_mit_mode = True
    node._current_motion_mode = 'mit'
    node._leader_mode_active = False
    node._fault_lockout = False
    node._fault_lockout_logged = False
    node._last_good_feedback_monotonic = time.monotonic()
    node._last_feedback_frame_ts = None
    node._last_feedback_advance_monotonic = time.monotonic()
    node._check_arm_connected = lambda: True
    node.feedback_timeout = 0.1
    node.hand_window_silence_feedback = True
    node.hand_window_max_silence_s = 10.0
    # Short but real: the silence verification is a timed observation, so the
    # tests pay it — just not 0.4 s per call.
    node.hand_window_silence_verify_s = 0.15
    node.hand_window_silence_quiet_s = 0.02
    # MOVE-J hold re-assertion: short budget/poll so the retry runs for real
    # without slowing the suite.
    node.hand_window_hold_assert_s = 0.05
    node.hand_window_hold_poll_s = 0.005
    node._hand_window_push_silenced = False
    node._hand_window_silence_started = 0.0
    return node


def test_prepare_hand_window_opens_on_verified_hold():
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_active is True
    assert arm.normal_mode_called is True
    assert arm.move_j_arg == [0.1] * 7


def test_prepare_hand_window_fails_and_reverts_when_not_settled():
    arm = _FakeArm(velocity=0.5, ctrl_mode=_CAN_CTRL)  # never settles
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is False  # reverted, hand window not opened


def test_prepare_hand_window_fails_if_left_in_teaching_mode():
    arm = _FakeArm(velocity=0.0, ctrl_mode=_TEACHING)  # backdrivable
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is False


def test_prepare_hand_window_fails_if_ctrl_mode_not_a_hold():
    # V112 honesty: set_normal_mode is a firmware no-op, so if the arm is left in
    # STANDBY (or any non-CAN_CTRL mode) the readback must fail verification
    # instead of claiming a hold the firmware never entered.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_STANDBY)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is False
    assert "hold NOT verified" in resp.message


def test_prepare_hand_window_rejected_while_recovering():
    arm = _FakeArm()
    node = _node(arm)
    node._recovery_in_progress = True
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert arm.normal_mode_called is False


def test_move_mit_dropped_while_hand_window_active():
    arm = _FakeArm()
    node = _node(arm)
    node._hand_window_active = True
    # The gate returns before touching msg fields, so a dummy is fine.
    node._move_mit_callback(object())
    assert arm.move_mit_calls == 0


def test_all_arm_ingress_gated_only_by_hand_window():
    # Every arm command callback funnels through _check_can_control(), so gating
    # there blocks MIT AND move_j/js/pose/line/circle and the follow path. Prove
    # the hand window is the sole difference on an otherwise-controllable node.
    node = _node(_FakeArm())
    node.control_ready = True
    node.enable_flag = True
    node.is_switch_seamlessly = True
    node._check_arm_ready = lambda: True

    node._hand_window_active = False
    assert node._check_can_control() is True

    node._hand_window_active = True
    assert node._check_can_control() is False


def test_resume_arm_control_reopens_side():
    arm = _FakeArm(frame_ts=5.0)
    node = _node(arm)
    node._hand_window_active = True
    resp = node._resume_arm_control_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_active is False


def test_open_window_silences_the_feedback_push_without_a_mode_switch():
    # The whole point of the window: the arm's own feedback push is what floods
    # the shared side bus, so it must go quiet — while the arm stays in its
    # CAN_CTRL hold (no leader/drag switch, which would drop the arm).
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_push_silenced is True
    assert arm.push_frames == [(_PUSH_DISABLE, _MOVE_MODE_NO_CHANGE)]
    # Cached mode message left neutral again so later motion-mode frames do not
    # re-toggle the push as a side effect.
    assert arm._msg_mode.enable_can_push == _PUSH_INVALID
    assert arm._msg_mode.ctrl_mode == _CAN_CTRL
    assert "feedback push silenced" in resp.message


def test_silence_is_verified_by_feedback_actually_stopping():
    # Silencing is a mode frame like any other and the SDK drops mode frames
    # under bus saturation — which is exactly when a window runs. Claiming
    # "silenced" from the send alone left the bus flooded while the log said
    # otherwise, so the absence of frames is measured.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL, push_live=True)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_push_silenced is True
    assert arm.push_enabled is False
    assert "verified" in resp.message


def test_dropped_silence_frame_is_re_sent_before_the_window_is_trusted():
    arm = _FakeArm(
        velocity=0.0, ctrl_mode=_CAN_CTRL, push_live=True, disables_until_silent=2
    )
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_push_silenced is True
    assert arm.push_enabled is False
    assert arm.push_frames == [
        (_PUSH_DISABLE, _MOVE_MODE_NO_CHANGE),
        (_PUSH_DISABLE, _MOVE_MODE_NO_CHANGE),
    ]
    assert "re-send" in resp.message


def test_unsilenceable_push_opens_the_window_but_re_enables_and_says_so():
    # Design choice kept from the original window: the arm IS held and MIT IS
    # gated, so the window is valid — it just did not free the bus. The state
    # must not claim a silence that never happened, or the bus-recovery
    # watchdog stays blind for nothing.
    arm = _FakeArm(
        velocity=0.0, ctrl_mode=_CAN_CTRL, push_live=True, disables_until_silent=99
    )
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True          # window open: arm held, MIT gated
    assert node._hand_window_push_silenced is False
    assert arm.push_enabled is True      # explicit ENABLE cancels a late DISABLE
    assert arm.push_frames[-1] == (_PUSH_ENABLE, _MOVE_MODE_NO_CHANGE)
    assert "NOT silenced" in resp.message


def test_hold_is_executed_by_the_firmware_not_by_the_gated_mit_loop():
    # The window silences feedback, so a host-side loop could not correct any
    # drift: the arm's own position controller must own the hold. Auto
    # mode-setting is forced on so the MOVE-J mode frame really goes out, and
    # the move mode is read back to prove the firmware left MIT.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL, mode_feedback=_MOVE_J)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert arm.auto_motion_mode is True
    assert arm.move_j_arg == [0.1] * 7
    assert "move_mode=1" in resp.message


def test_window_refused_when_move_j_never_lands_and_firmware_stays_in_mit():
    # Every MOVE-J mode frame is dropped on the flooded bus (measured failure:
    # the arm stayed in MOVE_MIT after prepare). The window must be refused
    # rather than silence the push against an arm the host still has to hold.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL, mode_feedback=_MOVE_MIT,
                   move_j_lands_after=10 ** 6)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is False
    assert node._hand_window_push_silenced is False
    assert arm.push_frames == []
    assert arm.move_j_calls > 1        # re-asserted, not tried once and trusted
    assert "firmware_holds=False" in resp.message


def test_dropped_move_j_is_re_asserted_until_the_firmware_confirms_the_hold():
    # First two MOVE-J frames are dropped (firmware stays MIT); the third lands.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL, mode_feedback=_MOVE_MIT,
                   move_j_lands_after=2)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert arm.move_j_calls == 3
    assert node._hand_window_push_silenced is True
    assert "move_j x3" in resp.message


def test_mit_move_mode_code_is_read_from_the_driver_not_hardcoded():
    # 0x04 is MIT below firmware v111 but UNASSIGNED from v111 on. The MIT code
    # must come from the active driver: a 1.06 arm legitimately reports 0x04 as
    # MIT, while on a v111+ arm 0x04 is not MIT and 0x06 is.
    v111 = _node(_FakeArm())  # fake driver reports MIT == 0x06
    assert v111._move_mode_is_mit(_MOVE_MIT) is True
    assert v111._move_mode_is_mit(0x04) is False

    legacy_arm = _FakeArm()
    legacy_arm._msg_mode.Enums.MotionMode = type("M", (), {"MIT": 0x04})
    legacy = _node(legacy_arm)
    assert legacy._move_mode_is_mit(0x04) is True
    assert legacy._move_mode_is_mit(_MOVE_MIT) is False


def test_push_is_not_silenced_before_the_hold_is_verified():
    # Verifying the hold reads feedback, so silencing may only happen after.
    arm = _FakeArm(velocity=0.0, ctrl_mode=_STANDBY)  # hold never verified
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_push_silenced is False
    assert arm.push_frames == []


def test_window_still_opens_but_warns_when_the_push_cannot_be_silenced():
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL)
    del arm._msg_mode  # SDK without the cached mode message
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True          # arm IS held and MIT IS gated
    assert node._hand_window_active is True
    assert node._hand_window_push_silenced is False
    assert "could not silence the feedback push" in resp.message


def test_silencing_can_be_disabled_by_parameter():
    arm = _FakeArm(velocity=0.0, ctrl_mode=_CAN_CTRL)
    node = _node(arm)
    node.hand_window_silence_feedback = False
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert arm.push_frames == []
    assert node._hand_window_push_silenced is False


def test_resume_restores_the_feedback_push():
    arm = _FakeArm(frame_ts=5.0)
    node = _node(arm)
    node._hand_window_active = True
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = time.monotonic()
    resp = node._resume_arm_control_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_push_silenced is False
    assert arm.push_frames == [(_PUSH_ENABLE, _MOVE_MODE_NO_CHANGE)]


def test_resume_restores_the_push_even_when_feedback_stays_stale():
    # The push must come back before the health checks; a failed resume must
    # never leave the arm silent on CAN.
    arm = _FakeArm(frame_ts=5.0, hz=0.0)
    node = _node(arm)
    node._hand_window_active = True
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = time.monotonic()
    node._last_feedback_frame_ts = 5.0
    node._last_feedback_advance_monotonic = time.monotonic() - 10.0
    node.feedback_timeout = 0.1
    resp = node._resume_arm_control_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is True   # side stays closed
    assert arm.push_frames == [(_PUSH_ENABLE, _MOVE_MODE_NO_CHANGE)]
    assert node._hand_window_push_silenced is False


def test_watchdog_does_not_recover_on_requested_silence():
    arm = _FakeArm(frame_ts=5.0, hz=0.0)
    node = _node(arm)
    node.bus_recovery_enabled = True
    node._had_control_ready = True
    node._recovery_in_progress = False
    node._last_good_feedback_monotonic = time.monotonic() - 30.0
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = time.monotonic()
    assert node._should_recover_bus() is False
    assert node._hand_window_push_silenced is True   # silence still in effect
    assert arm.push_frames == []


def test_watchdog_restores_the_push_when_the_silence_outlives_its_bound():
    # The watchdog is blind while the push is off, so the silence is bounded:
    # a window that never resumes gets the push (and the watchdog) back.
    arm = _FakeArm(frame_ts=5.0, hz=0.0)
    node = _node(arm)
    node.bus_recovery_enabled = True
    node._had_control_ready = True
    node._recovery_in_progress = False
    node.hand_window_max_silence_s = 0.5
    node._hand_window_push_silenced = True
    node._hand_window_silence_started = time.monotonic() - 5.0
    assert node._should_recover_bus() is False
    assert node._hand_window_push_silenced is False
    assert arm.push_frames == [(_PUSH_ENABLE, _MOVE_MODE_NO_CHANGE)]


def test_resume_arm_control_rejected_on_stale_feedback():
    arm = _FakeArm(frame_ts=5.0, hz=0.0)
    node = _node(arm)
    node._hand_window_active = True
    # Frame timestamp frozen and advance window already expired -> stale.
    node._last_feedback_frame_ts = 5.0
    node._last_feedback_advance_monotonic = time.monotonic() - 10.0
    node.feedback_timeout = 0.25
    resp = node._resume_arm_control_callback(None, Trigger.Response())
    assert resp.success is False
    assert node._hand_window_active is True  # left closed until feedback is healthy
