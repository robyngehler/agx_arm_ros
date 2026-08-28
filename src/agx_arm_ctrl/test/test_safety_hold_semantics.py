"""The stopped arm holds its pose; it does not go limp.

The desired stopped state is: motors enabled, firmware position controller
active, current pose held, motion authority revoked. A kp=0 damped MIT command
stops a moving arm but has no stiffness, so leaving it as the terminal state
sags the arm — it is a braking transient before MOVE-J, never the end state.

The MOVE-J hold is also the whole ladder: no safety path issues the vendor
``electronic_emergency_stop``, which is a damped descent rather than a hold.
An unverified stop re-asserts the hold; the external CAN watchdog is the
boundary beyond it.
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
    node._force_recovery = False
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


class _SettlingArm(_HoldArm):
    """Feedback whose timestamps advance while the joints stay put.

    Enough for the settle check to reach a verdict: ``_HoldArm`` freezes its
    timestamp, which yields "no evidence" rather than "settled".
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._ts = 0.0

    def get_joint_angles(self):
        self._ts += 0.05
        return type("A", (), {
            "hz": 100.0, "timestamp": self._ts, "msg": [0.0] * 7,
        })()


def test_no_safety_path_ever_sends_the_vendor_electronic_stop():
    """It is a damped descent, not a hold.

    The vendor call releases the stiffness that keeps a raised arm up, which is
    the state the MOVE-J hold exists to produce. The ladder ends at the hold.
    """
    arm = _HoldArm()
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert "electronic_emergency_stop" not in arm.calls


def test_the_pre_recovery_hold_never_sends_the_vendor_electronic_stop():
    arm = _SettlingArm()
    node = _node(arm)

    node._hold_before_teardown()

    assert "electronic_emergency_stop" not in arm.calls


def test_an_unverified_stop_reasserts_the_hold_instead_of_escalating():
    """Feedback that cannot answer is not evidence that the arm is running away.

    ``_HoldArm`` never advances its feedback timestamp, so every settle check
    returns "no evidence" — the exact case that used to command a descent on
    top of a hold the firmware had already confirmed.
    """
    arm = _HoldArm()
    node = _node(arm)

    response = Trigger.Response()
    node._emergency_stop_callback(None, response)

    assert "electronic_emergency_stop" not in arm.calls
    assert arm.calls.count("move_j") >= node.ESTOP_HOLD_ATTEMPTS, (
        f"expected one MOVE-J hold per attempt, got {arm.calls}"
    )
    assert response.success is False
    assert node._force_recovery is True, (
        "transport repair is the only thing left after the hold attempts"
    )


def test_a_verified_stop_holds_once_and_does_not_retry():
    arm = _SettlingArm()
    node = _node(arm)

    response = Trigger.Response()
    node._emergency_stop_callback(None, response)

    assert response.success is True
    assert "stop=verified" in response.message
    assert arm.calls.count("move_j") == 1, arm.calls
    assert node._force_recovery is False


def test_a_stop_without_a_trustworthy_pose_commands_nothing():
    """No pose means no hold, and no hold means no motion command at all.

    The vendor electronic stop used to fill this gap. Commanding nothing and
    saying so is better than commanding a descent: the external CAN watchdog
    owns this regime.
    """
    arm = _HoldArm()
    node = _node(arm)
    node._capture_hold_pose = lambda **_kwargs: None

    response = Trigger.Response()
    node._emergency_stop_callback(None, response)

    assert arm.calls == [], f"nothing may be commanded without a pose, got {arm.calls}"
    assert response.success is False
    assert "no_hold_commanded" in response.message
    assert any("watchdog" in e for e in node.logger.errors)


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


class _UnreadableModeArm(_HoldArm):
    """Answers joint angles but never a usable move mode.

    The realistic shape of a sick bus: enough feedback to capture a pose, not
    enough to confirm what the firmware did with it.
    """

    def __init__(self, *, mode=None):
        super().__init__()
        self._mode = mode

    def get_arm_status(self):
        self.mode_reads += 1
        if self._mode is None:
            return None
        return type("S", (), {"msg": type("M", (), {
            "ctrl_mode": 0x01, "mode_feedback": self._mode,
        })()})()


def test_an_unreadable_move_mode_does_not_confirm_the_hold():
    """Absence of a MIT reading is not a non-MIT reading.

    The verifier asked "is this not MIT?", so a status read that answered
    nothing passed — precisely when the hold most needed checking.
    """
    node = _node(_UnreadableModeArm(mode=None))

    left_mit, move_mode, _attempts = node._assert_firmware_hold([0.0] * 7)

    assert left_mit is False
    assert move_mode is None


def test_an_unknown_move_mode_does_not_confirm_the_hold():
    node = _node(_UnreadableModeArm(mode=0xFF))

    left_mit, move_mode, _attempts = node._assert_firmware_hold([0.0] * 7)

    assert left_mit is False
    assert move_mode == 0xFF


def test_a_positive_non_mit_move_mode_confirms_the_hold():
    node = _node(_UnreadableModeArm(mode=0x01))   # MOVE-J

    left_mit, move_mode, _attempts = node._assert_firmware_hold([0.0] * 7)

    assert left_mit is True
    assert move_mode == 0x01


def test_the_stop_reports_an_unconfirmable_hold_rather_than_claiming_one():
    node = _node(_UnreadableModeArm(mode=None))

    node._emergency_stop_callback(None, Trigger.Response())

    assert any("NOT in a firmware hold" in e for e in node.logger.errors)


def test_neither_tier_s_mit_code_can_pass_as_a_hold():
    """0x04 is MIT below v111 and unassigned on it; 0x06 is the reverse.

    Neither may ever read as a confirmed hold, whichever driver is loaded.
    """
    for mit_code in (0x04, 0x06):
        node = _node(_UnreadableModeArm(mode=mit_code))
        left_mit, _mode, _attempts = node._assert_firmware_hold([0.0] * 7)
        assert left_mit is False, f"move mode {mit_code:#04x} passed as a hold"


# --- shutdown ----------------------------------------------------------------

def _shutdown_node(arm, **kwargs) -> AgxArmRosNode:
    node = _node(arm, **kwargs)
    node.enable_flag = True
    return node


def test_shutdown_parks_the_arm_in_the_firmware_hold():
    """Exiting is not a reason to leave the arm on the last streamed setpoint.

    The firmware executes the last MIT command it received indefinitely, so a
    process that just goes away hands the arm to whatever was mid-trajectory.
    """
    arm = _HoldArm()
    node = _shutdown_node(arm)

    assert node.hold_on_shutdown() is True
    assert "move_j" in arm.calls
    assert "electronic_emergency_stop" not in arm.calls, (
        "the ladder ends at MOVE-J; a damped descent is not a hold"
    )


def test_shutdown_latches_nothing():
    """An ordinary exit must not cost the next bring-up a lockout to clear."""
    arm = _HoldArm()
    node = _shutdown_node(arm)
    node.hold_on_shutdown()

    assert node._estop_latched is False
    assert node.unit_stop_requests == []


def test_shutdown_commands_no_hold_while_recovery_owns_the_session():
    arm = _HoldArm()
    node = _shutdown_node(arm, recovering=True)

    assert node.hold_on_shutdown() is False
    assert arm.calls == []


class _MutePoseArm(_HoldArm):
    """Answers no joint angles, so no pose hold can be built."""

    def get_joint_angles(self):
        return None

    def set_normal_mode(self):
        self.calls.append("set_normal_mode")


def test_shutdown_falls_through_to_the_mode_frame_without_a_pose():
    """No pose, no MOVE-J — but the rung below is a mode frame, not a MIT command.

    ``set_normal_mode`` needs neither pose nor feedback, ends the MIT setpoint the
    firmware would keep executing, and leaves the arm to its own position
    controller. It cannot be verified, so nothing is claimed.
    """
    arm = _MutePoseArm()
    node = _shutdown_node(arm)

    assert node.hold_on_shutdown() is False
    assert "move_j" not in arm.calls
    assert "set_normal_mode" in arm.calls
    assert any("cut arm power" in e.lower() for e in node.logger.errors)


# --- the prohibition ---------------------------------------------------------

def test_no_kp_zero_mit_command_is_reachable_on_this_driver():
    """Removed, not merely unused: an escalation step that exists gets called.

    A kp=0 MIT command ends a moving setpoint without stiffness, so it trades a
    runaway for a sag. It was the emergency stop's braking transient, the
    pre-recovery quiesce, and the hand window's mode change; all three now go
    straight to the hold that keeps the arm up.
    """
    for name in ("_submit_damped_stop_mit", "_send_damped_stop_mit",
                 "_send_damped_stop_joint"):
        assert not hasattr(AgxArmRosNode, name), f"{name} is still reachable"


def test_the_emergency_stop_sends_no_mit_command():
    class _MitWatchingArm(_HoldArm):
        def move_mit(self, **kwargs):
            self.calls.append(f"move_mit(kp={kwargs.get('kp')})")

    arm = _MitWatchingArm()
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert not any(call.startswith("move_mit") for call in arm.calls), arm.calls
    assert "move_j" in arm.calls, "the stop still has to command its hold"


def test_the_emergency_stop_without_a_pose_sends_the_mode_frame_not_a_setpoint():
    arm = _MutePoseArm()
    node = _node(arm)

    node._emergency_stop_callback(None, Trigger.Response())

    assert "set_normal_mode" in arm.calls
    assert "move_j" not in arm.calls
    assert not any(call.startswith("move_mit") for call in arm.calls)


def test_the_hold_service_latches_nothing():
    """The MOVE-J rung on its own, for a controller that lost its feedback.

    Sharing the emergency stop's fault latch would make an ordinary escalation
    cost the next bring-up a lockout to clear.
    """
    arm = _HoldArm()
    node = _shutdown_node(arm)

    ok, message = node.hold_current_pose("test")

    assert ok is True
    assert "move_j" in arm.calls
    assert node._estop_latched is False
    assert node.unit_stop_requests == []
    assert message
