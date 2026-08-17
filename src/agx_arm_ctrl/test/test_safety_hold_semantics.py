"""The stopped arm holds its pose; it does not go limp.

The desired stopped state is: motors enabled, firmware position controller
active, current pose held, motion authority revoked. A kp=0 damped MIT command
stops a moving arm but has no stiffness, so leaving it as the terminal state
sags the arm — it is a braking transient before MOVE-J, never the end state.
"""

import threading

import pytest
from std_srvs.srv import Trigger

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode
from agx_arm_ctrl.device_authority import DeviceAuthority, UnitSafety
from agx_arm_ctrl.sdk_worker import Lane, SdkWorker


_WORKERS: list[SdkWorker] = []


@pytest.fixture(autouse=True)
def _shut_down_workers():
    yield
    while _WORKERS:
        _WORKERS.pop().shutdown()


class _Logger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _Angles:
    hz = 100.0
    timestamp = 1.0
    msg = [0.0] * 7


class _HoldArm:
    """Reports MIT until ``mit_reads_before_release`` move-mode reads elapse."""

    ARM_STATUS = type("S", (), {"CtrlMode": type("C", (), {"TEACHING_MODE": 9})})

    def __init__(self, *, mit_reads_before_release=0):
        self.calls = []
        self.mit_reads_before_release = mit_reads_before_release
        self.mode_reads = 0

    def get_joint_angles(self):
        return _Angles()

    def move_j(self, q):
        self.calls.append("move_j")

    def move_js(self, q):
        self.calls.append("move_js")

    def disable(self):
        self.calls.append("disable")

    def enable(self):
        self.calls.append("enable")
        return True

    def electronic_emergency_stop(self):
        self.calls.append("electronic_emergency_stop")

    def set_auto_set_motion_mode_enabled(self, _v):
        self.calls.append("set_auto_set_motion_mode_enabled")

    def get_arm_status(self):
        self.mode_reads += 1
        mit = self.mode_reads <= self.mit_reads_before_release
        return type("S", (), {"msg": type("M", (), {
            "ctrl_mode": 0x01, "mode_feedback": 0x04 if mit else 0x01,
        })()})()


def _node(arm, *, recovering=False, estop=False, unit_stopped=False,
          auto_enable=True) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.arm_type = "nero"
    node.device_id = "arm_right"
    node.agx_arm = arm
    node.arm_joint_count = 7
    node.auto_enable = auto_enable
    node.enable_timeout = 1.0
    node.feedback_timeout = 1.0
    node.is_mit_mode = False
    node._current_motion_mode = None
    node.is_switch_seamlessly = True
    node._estop_latched = estop
    node._recovery_in_progress = recovering
    node._recovery_lock = threading.Lock()
    node._recovery_started_monotonic = 0.0
    node._hand_window_push_silenced = False
    node._last_feedback_frame_ts = None
    node._last_feedback_advance_monotonic = 0.0
    node._last_good_feedback_monotonic = 0.0
    node.hand_window_hold_assert_s = 0.3
    node.hand_window_hold_poll_s = 0.01
    node._sdk = SdkWorker("arm_test")
    _WORKERS.append(node._sdk)
    node._unit_safety = UnitSafety("arm_right", writer=True)
    if unit_stopped:
        node._unit_safety.stop("test")
    node._authority = DeviceAuthority("arm_right", node._unit_safety)
    node._authority.go_standby("connected")
    node._authority.rearm(verified=True, detail="test")
    node.unit_stop_requests = []
    node._request_unit_stop = node.unit_stop_requests.append
    return node


def test_the_stop_carries_its_generation_to_the_worker():
    """The safety lane outranks queued work; it does not invalidate it.

    Without the epoch update a MIT cycle queued before the stop still executes
    after the hold — the arm moves again from a state that was declared stopped.
    """
    arm = _HoldArm()
    node = _node(arm)
    before = node._sdk._epoch

    node._emergency_stop_callback(None, Trigger.Response())

    assert node._sdk._epoch is not None
    assert node._sdk._epoch == node._authority.device_epoch
    assert node._sdk._epoch != before


def test_stale_control_queued_before_the_stop_is_dropped():
    arm = _HoldArm()
    node = _node(arm)
    stale_epoch = node._authority.device_epoch
    node._sdk.quiesce(timeout=1.0)
    ran = []
    call = node._sdk.submit(
        "joint_ctrl", lambda: ran.append("ran"), epoch=stale_epoch
    )

    node._emergency_stop_callback(None, Trigger.Response())
    node._sdk.resume()
    call.wait(timeout=1.0)

    assert ran == [], "control queued under the pre-stop generation still ran"


def test_the_stop_reasserts_move_j_until_the_firmware_leaves_mit():
    """One dropped mode frame must not be reported as a hold.

    The firmware would still be in MIT executing the kp=0 damped stop, which has
    no stiffness — the arm sags while the software calls it stopped.
    """
    arm = _HoldArm(mit_reads_before_release=2)
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert arm.calls.count("move_j") >= 3, (
        f"expected repeated MOVE-J assertion, got {arm.calls}"
    )


def test_a_hold_the_firmware_never_confirms_is_reported_not_claimed():
    arm = _HoldArm(mit_reads_before_release=10_000)
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert any("NOT in a firmware hold" in e for e in node.logger.errors)


def test_no_safety_path_ever_disables_the_motors():
    """disable() is a lifecycle command, not a synonym for STOP.

    A disabled Nero has no brakes and drops under gravity, which is the opposite
    of the intended stopped state.
    """
    arm = _HoldArm()
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert "disable" not in arm.calls


def test_recovery_does_not_rearm_an_arm_that_was_stopped():
    arm = _HoldArm()
    node = _node(arm, estop=True)

    assert node._may_auto_enable_after_recovery() is False
    assert any("emergency stop is latched" in w for w in node.logger.warns)


def test_recovery_does_not_rearm_while_the_unit_is_stopped():
    arm = _HoldArm()
    node = _node(arm, unit_stopped=True)

    assert node._may_auto_enable_after_recovery() is False
    assert any("unit is STOPPED" in w for w in node.logger.warns)


def test_recovery_rearms_normally_when_nothing_is_stopped():
    arm = _HoldArm()
    node = _node(arm)

    assert node._may_auto_enable_after_recovery() is True


def test_a_pre_recovery_hold_without_trustworthy_feedback_is_not_claimed():
    """A pose synthesised from stale data would be a wrong hold, not a missing one."""
    arm = _HoldArm()
    node = _node(arm)
    node._capture_hold_pose = lambda: None

    node._hold_before_teardown()

    assert any("UNAVAILABLE" in e for e in node.logger.errors)
    assert "move_j" not in arm.calls


def test_the_firmware_hold_runs_entirely_on_the_safety_lane():
    """Every part of it, the mode confirmation included.

    On the default lane the confirming read waits behind the very control
    stream the hold is displacing, and the bounded assertion window expires
    without ever seeing the mode change.
    """
    arm = _HoldArm()
    node = _node(arm)
    lanes = []
    original = node._sdk.call
    node._sdk.call = lambda name, fn, **kw: (
        lanes.append(kw.get("lane")), original(name, fn, **kw)
    )[1]

    node._assert_firmware_hold([0.0] * 7)

    assert lanes and all(lane is Lane.SAFETY for lane in lanes), lanes
