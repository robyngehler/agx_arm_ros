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

_NOT_TEACHING = 0x00
_TEACHING = 0x02


class _FakeLogger:
    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _FakeArm:
    def __init__(self, *, velocity=0.0, ctrl_mode=_NOT_TEACHING, hz=1.0,
                 frame_ts=1.0, comm_err=None):
        self.velocity = velocity
        self.ctrl_mode = ctrl_mode
        self.hz = hz
        self.frame_ts = frame_ts
        self.comm_err = comm_err
        self.ARM_STATUS = SimpleNamespace(
            CtrlMode=SimpleNamespace(TEACHING_MODE=_TEACHING)
        )
        self.normal_mode_called = False
        self.move_j_arg = None
        self.move_mit_calls = 0

    def get_joint_angles(self):
        return SimpleNamespace(msg=[0.1] * 7, hz=self.hz, timestamp=self.frame_ts)

    def get_motor_states(self, _i):
        return SimpleNamespace(msg=SimpleNamespace(velocity=self.velocity))

    def get_arm_status(self):
        return SimpleNamespace(msg=SimpleNamespace(ctrl_mode=self.ctrl_mode))

    def set_normal_mode(self):
        self.normal_mode_called = True

    def move_j(self, q):
        self.move_j_arg = list(q)

    def set_motion_mode(self, *_a):
        pass

    def set_auto_set_motion_mode_enabled(self, *_a):
        pass

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
    node._last_good_feedback_monotonic = time.monotonic()
    node._last_feedback_frame_ts = None
    node._last_feedback_advance_monotonic = time.monotonic()
    node._check_arm_connected = lambda: True
    return node


def test_prepare_hand_window_opens_on_verified_hold():
    arm = _FakeArm(velocity=0.0, ctrl_mode=_NOT_TEACHING)
    node = _node(arm)
    resp = node._prepare_hand_window_callback(None, Trigger.Response())
    assert resp.success is True
    assert node._hand_window_active is True
    assert arm.normal_mode_called is True
    assert arm.move_j_arg == [0.1] * 7


def test_prepare_hand_window_fails_and_reverts_when_not_settled():
    arm = _FakeArm(velocity=0.5, ctrl_mode=_NOT_TEACHING)  # never settles
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
