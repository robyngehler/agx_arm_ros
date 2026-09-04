"""Unit tests for the CAN bus-recovery watchdog decision logic.

Regression guard for the shared-CAN step-and-settle plan (Phase 1 item 4,
section 1.3.3): recovery must be driven by the kernel RX timestamp, not by the
FPS-based ``is_ok()`` or the node-observed feedback clock, both of which go stale
under local CPU/GIL starvation while the bus is still delivering frames.

The node connects to hardware in ``__init__``, so these tests build a bare
instance via ``__new__`` and exercise the pure predicate methods directly.
"""

import time
from types import SimpleNamespace

from agx_arm_ctrl.sdk_worker import SdkWorker as _SdkWorker
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _FakeLogger:
    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _FakeArm:
    """Minimal stand-in for the pyAgxArm driver."""

    def __init__(self, *, is_ok: bool, frame_ts: float):
        self._is_ok = is_ok
        self._frame_ts = frame_ts

    def is_ok(self) -> bool:
        return self._is_ok

    def has_comm_error(self) -> bool:
        return False

    def get_comm_error(self):
        return None

    def get_joint_angles(self):
        return SimpleNamespace(timestamp=self._frame_ts)


def _make_node(*, is_ok: bool, frame_ts: float, node_feedback_age_s: float,
               frames_have_advanced: bool) -> AgxArmRosNode:
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _FakeLogger()
    node.agx_arm = _FakeArm(is_ok=is_ok, frame_ts=frame_ts)

    now = time.monotonic()
    node.bus_recovery_enabled = True
    node._recovery_in_progress = False
    node._had_control_ready = True
    node.bus_recovery_cooldown_s = 0.0
    node._last_recovery_end_monotonic = now - 100.0
    node._recovery_cooldown_logged = False
    node._force_recovery = False
    node._tx_stall_detected = False
    node._tx_stall_count = 0
    node.feedback_timeout = 0.25
    node._last_good_feedback_monotonic = now - node_feedback_age_s
    node._last_tx_congestion_log = 0.0
    node._recover_reason = ""
    node._recovery_reason_counts = {}
    node._loop_overrun_suppressions = 0
    node._last_loop_gap_s = 0.5
    node._last_overrun_log_monotonic = now  # rate-limit: suppress the log call
    # No hand window is silencing the feedback push in these scenarios.
    node._hand_window_push_silenced = False
    node._hand_window_silence_started = 0.0
    node.hand_window_max_silence_s = 10.0

    # Frame-advance tracking: seed so that a matching frame_ts read now counts as
    # "advanced recently" or "stale" depending on frames_have_advanced.
    node._last_feedback_frame_ts = frame_ts
    node._last_feedback_advance_monotonic = (
        now if frames_have_advanced else now - 10.0
    )
    return node


def _snap(node):
    """The watchdog now judges a snapshot, not the SDK directly.

    Acquisition happens first and the decision second, so the health values it
    reads come from the same instant as the feedback they are judging.
    """
    from agx_arm_ctrl.agx_arm_ctrl_single_node import FeedbackSnapshot

    return FeedbackSnapshot(
        joint_angles=node.agx_arm.get_joint_angles(),
        motor_states=(),
        flange_pose=None,
        tcp_pose=None,
        arm_status=None,
        leader_joint_angles=None,
        is_ok=bool(node.agx_arm.is_ok()),
        send_error_count=-1,
        acquired_at=0.0,
    )


def test_not_ok_but_frames_advancing_is_suppressed():
    # is_ok() reads false but a NEW kernel frame arrives -> local starvation.
    node = _make_node(
        is_ok=False, frame_ts=101.0, node_feedback_age_s=0.0,
        frames_have_advanced=False,  # last seen ts differs below
    )
    node._last_feedback_frame_ts = 100.0  # 101.0 != 100.0 -> advanced
    assert node._should_recover_bus(_snap(node)) is False
    assert node._loop_overrun_suppressions == 1
    assert node._recovery_reason_counts == {}


def test_not_ok_and_frames_frozen_recovers():
    # is_ok() false AND the kernel frame timestamp stopped advancing -> real loss.
    node = _make_node(
        is_ok=False, frame_ts=100.0, node_feedback_age_s=0.0,
        frames_have_advanced=False,
    )
    assert node._should_recover_bus(_snap(node)) is True
    assert node._recovery_reason_counts.get("not_ok") == 1


def test_node_clock_stale_but_frames_advancing_is_suppressed():
    # is_ok() healthy, node-observed clock aged out by a publish-loop stall, but
    # kernel frames kept advancing -> suppress, do not recover.
    node = _make_node(
        is_ok=True, frame_ts=205.0, node_feedback_age_s=10.0,
        frames_have_advanced=False,
    )
    node._last_feedback_frame_ts = 204.0  # 205.0 != 204.0 -> advanced
    assert node._should_recover_bus(_snap(node)) is False
    assert node._loop_overrun_suppressions == 1
    assert node._recovery_reason_counts == {}


def test_node_clock_stale_and_frames_frozen_recovers():
    # is_ok() healthy but both the node clock and the kernel timestamp are stale.
    node = _make_node(
        is_ok=True, frame_ts=200.0, node_feedback_age_s=10.0,
        frames_have_advanced=False,
    )
    assert node._should_recover_bus(_snap(node)) is True
    assert node._recovery_reason_counts.get("stale_feedback") == 1


# --- recovery runs off the acquisition path ----------------------------------

def test_requesting_recovery_returns_immediately_and_latches_the_authority():
    """Measured on hardware: inline recovery cost 13.1 s of publish loop.

    During it nothing published state, nothing drained the CAN RX socket, and
    there was no way to see how long recovery had been running. The acquisition
    path now only detects, latches and requests.
    """
    import threading as _threading
    import time as _time

    from agx_arm_ctrl.device_authority import (
        DeviceAuthority,
        DeviceState,
        UnitSafety,
    )

    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _RecoveryLogger()
    node.device_id = "arm_left"
    node._recovery_in_progress = False
    node._recovery_lock = _threading.Lock()
    node._recovery_started_monotonic = 0.0
    node._recover_reason = "not_ok"
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_timeout = 1.0
    # A real worker: recovery has to take the SDK session from it and give it
    # back, which is the property under test, not a detail to stub away.
    node._sdk = _SdkWorker("arm_left")
    # Recovery now commands a firmware MOVE-J hold before it takes the session:
    # a host-side MIT command is not a hold, and the arm must not sag through
    # the teardown. These are what that attempt reads.
    node.feedback_timeout = 1.0
    node.is_mit_mode = False
    node._current_motion_mode = None
    node._hand_window_push_silenced = False
    node._capture_hold_pose = lambda: None
    node._unit_safety = UnitSafety("arm_left", writer=False)
    node._authority = DeviceAuthority("arm_left", node._unit_safety)
    node._authority.go_standby("connected")
    node._authority.rearm(verified=True, detail="test")

    released = _threading.Event()
    node._recover_bus = lambda: released.wait(5.0)

    started = _time.monotonic()
    node._request_recovery()
    elapsed = _time.monotonic() - started

    try:
        assert elapsed < 0.5, f"_request_recovery blocked for {elapsed:.2f}s"
        assert node._recovery_in_progress is True
        assert node._authority.state is DeviceState.RECOVERING
        assert not node._authority.snapshot().motion_ready
        # How long it has been running is observable, not hidden.
        assert node.recovery_active_s > 0.0
    finally:
        released.set()
        _time.sleep(0.2)

    assert node._recovery_in_progress is False
    assert node.recovery_active_s == 0.0


def test_a_second_request_does_not_start_a_second_recovery():
    import threading as _threading
    import time as _time

    from agx_arm_ctrl.device_authority import DeviceAuthority, UnitSafety

    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _RecoveryLogger()
    node.device_id = "arm_left"
    node._recovery_in_progress = False
    node._recovery_lock = _threading.Lock()
    node._recovery_started_monotonic = 0.0
    node._recover_reason = "not_ok"
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_timeout = 1.0
    # A real worker: recovery has to take the SDK session from it and give it
    # back, which is the property under test, not a detail to stub away.
    node._sdk = _SdkWorker("arm_left")
    # Recovery now commands a firmware MOVE-J hold before it takes the session:
    # a host-side MIT command is not a hold, and the arm must not sag through
    # the teardown. These are what that attempt reads.
    node.feedback_timeout = 1.0
    node.is_mit_mode = False
    node._current_motion_mode = None
    node._hand_window_push_silenced = False
    node._capture_hold_pose = lambda: None
    node._unit_safety = UnitSafety("arm_left", writer=False)
    node._authority = DeviceAuthority("arm_left", node._unit_safety)

    released = _threading.Event()
    runs = []
    node._recover_bus = lambda: (runs.append(1), released.wait(5.0))

    node._request_recovery()
    node._request_recovery()
    node._request_recovery()
    _time.sleep(0.2)
    released.set()
    _time.sleep(0.3)

    assert len(runs) == 1, f"recovery ran {len(runs)} times concurrently"


class _RecoveryLogger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


def test_a_forced_estop_recovery_survives_the_watchdog_being_disabled():
    """Two decisions that used to share one switch.

    `bus_recovery_enabled` turns off the *watchdog* — the automatic reaction to
    a stalled bus. It used to also disable the forced recovery an emergency stop
    escalates to when it cannot confirm the arm stopped, so an operator who
    turned off the watchdog silently removed the last resort of the stop path.
    """
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _RecoveryLogger()
    node._recovery_in_progress = False
    node.bus_recovery_enabled = False
    node._force_recovery = True
    triggered = []
    node._trigger_recovery = lambda category, reason: (
        triggered.append(category) or True
    )

    assert node._should_recover_bus(None) is True
    assert triggered == ["forced_estop"]
    # Consumed once, so it does not re-fire on the next tick.
    assert node._force_recovery is False


def test_the_watchdog_itself_still_honours_its_switch():
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _RecoveryLogger()
    node._recovery_in_progress = False
    node.bus_recovery_enabled = False
    node._force_recovery = False

    assert node._should_recover_bus(None) is False


def test_recovery_takes_the_sdk_session_and_gives_it_back():
    """The handover, in the order that keeps exactly one owner throughout."""
    import threading as _threading
    import time as _time

    from agx_arm_ctrl.device_authority import DeviceAuthority, UnitSafety
    from agx_arm_ctrl.sdk_worker import SdkWorker as _W

    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _RecoveryLogger()
    node.device_id = "arm_left"
    node._recovery_in_progress = False
    node._recovery_lock = _threading.Lock()
    node._recovery_started_monotonic = 0.0
    node._recover_reason = "not_ok"
    node.control_ready = True
    node._control_ready_logged = True
    node.enable_timeout = 1.0
    node._sdk = _W("arm_left")
    node.feedback_timeout = 1.0
    node.is_mit_mode = False
    node._current_motion_mode = None
    node._hand_window_push_silenced = False
    node._unit_safety = UnitSafety("arm_left", writer=False)
    node._authority = DeviceAuthority("arm_left", node._unit_safety)
    node._authority.go_standby("connected")
    node._authority.rearm(verified=True, detail="test")

    released = _threading.Event()
    owned_during_recovery = []
    order = []

    # The hold has to be commanded while the worker still dequeues; once the
    # session is handed over, nothing can command the arm at all. A pose that
    # cannot be trusted yields no hold rather than a synthesised one.
    node._capture_hold_pose = lambda: (order.append("capture"), [0.0] * 7)[1]
    node._assert_firmware_hold = lambda q: (
        order.append("hold"), (True, 0x01, 1)
    )[1]

    def recovery():
        # While recovery owns the session the worker must be quiesced.
        owned_during_recovery.append(node._sdk.quiesced)
        order.append("teardown")
        released.wait(5.0)

    node._recover_bus = recovery

    try:
        node._request_recovery()
        _time.sleep(0.3)
        assert node._sdk.quiesced is True, "worker still owns the session"
        assert order[:3] == ["capture", "hold", "teardown"], (
            f"the arm must be held before the session is taken away, got {order}"
        )

        # Work submitted during the handover does not execute meanwhile.
        ran = []
        node._sdk.submit("joint_ctrl", lambda: ran.append("ran"))
        _time.sleep(0.2)
        assert ran == []
    finally:
        released.set()
        _time.sleep(0.4)

    assert owned_during_recovery == [True]
    assert node._sdk.quiesced is False, "session was not handed back"
    node._sdk.shutdown()


# --------------------------------------------------------------------------
# Recovery owns the session it quiesced
#
# Recovery quiesces the worker and then has to re-arm and confirm feedback
# through something. Submitting either to that worker cannot complete: a
# quiesced worker keeps accepting submissions and dequeues none, so the re-arm
# never reaches the wire and a live bus reads as "not ready" for as long as
# recovery runs.
# --------------------------------------------------------------------------


class _SessionArm:
    """Records which calls actually reached the SDK. Healthy throughout."""

    def __init__(self):
        self.calls = []
        self._ts = 1000.0

    def enable(self):
        self.calls.append("enable")
        return True

    def get_joint_enable_status(self, _joint):
        self.calls.append("enable_readback")
        return True

    def is_ok(self):
        return True

    def get_joint_angles(self):
        self.calls.append("joint_angles")
        self._ts += 0.01
        return SimpleNamespace(timestamp=self._ts, hz=137.0, msg=[0.0] * 7)


def _session_node(*, quiesced: bool):
    from agx_arm_ctrl.sdk_worker import SdkWorker as _W

    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.get_logger = lambda: _FakeLogger()
    node.agx_arm = _SessionArm()
    node._sdk = _W("arm_left")
    node.feedback_timeout = 0.2
    node.enable_timeout = 1.0
    node.enable_flag = False
    node._transport_available = lambda: True
    node._last_feedback_frame_ts = None
    node._last_feedback_advance_monotonic = time.monotonic()
    if quiesced:
        assert node._sdk.quiesce(timeout=1.0)
    return node


def test_the_rearm_reaches_the_arm_while_recovery_owns_the_session():
    """The re-arm must reach the wire, not the queue recovery just stopped."""
    node = _session_node(quiesced=True)
    try:
        assert node._enable_arm(True, 1.0, direct=True) is True
        assert "enable" in node.agx_arm.calls
    finally:
        node._sdk.resume()
        node._sdk.shutdown()


def test_a_rearm_submitted_to_the_quiesced_worker_never_reaches_the_arm():
    """The shape the direct path exists to avoid, pinned so it cannot return."""
    from agx_arm_ctrl.sdk_worker import CallOutcomeUnknown

    node = _session_node(quiesced=True)
    try:
        try:
            node._request_enable(True, 0.3)
        except CallOutcomeUnknown:
            pass
        assert node.agx_arm.calls == [], (
            f"the enable must not have reached the arm, got {node.agx_arm.calls}"
        )
    finally:
        node._sdk.resume()
        node._sdk.shutdown()


def test_recovery_confirms_a_live_bus_while_it_owns_the_session():
    """Confirming the bus came back is the whole exit condition of recovery."""
    node = _session_node(quiesced=True)
    try:
        assert node._wait_for_feedback(1.0, direct=True) is True
        assert "joint_angles" in node.agx_arm.calls
    finally:
        node._sdk.resume()
        node._sdk.shutdown()


def test_a_feedback_wait_on_the_quiesced_worker_reads_a_live_bus_as_dead():
    """Why recovery reported 'did not restore feedback' on a healthy bus."""
    node = _session_node(quiesced=True)
    try:
        assert node._wait_for_feedback(0.4) is False
        assert node.agx_arm.calls == []
    finally:
        node._sdk.resume()
        node._sdk.shutdown()


def test_the_ordinary_paths_still_go_through_the_worker():
    """`direct` is the recovery exception, not a new default."""
    node = _session_node(quiesced=False)
    try:
        assert node._wait_for_feedback(1.0) is True
        assert "joint_angles" in node.agx_arm.calls
    finally:
        node._sdk.shutdown()
