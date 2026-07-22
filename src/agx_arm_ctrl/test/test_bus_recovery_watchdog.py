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

    # Frame-advance tracking: seed so that a matching frame_ts read now counts as
    # "advanced recently" or "stale" depending on frames_have_advanced.
    node._last_feedback_frame_ts = frame_ts
    node._last_feedback_advance_monotonic = (
        now if frames_have_advanced else now - 10.0
    )
    return node


def test_not_ok_but_frames_advancing_is_suppressed():
    # is_ok() reads false but a NEW kernel frame arrives -> local starvation.
    node = _make_node(
        is_ok=False, frame_ts=101.0, node_feedback_age_s=0.0,
        frames_have_advanced=False,  # last seen ts differs below
    )
    node._last_feedback_frame_ts = 100.0  # 101.0 != 100.0 -> advanced
    assert node._should_recover_bus() is False
    assert node._loop_overrun_suppressions == 1
    assert node._recovery_reason_counts == {}


def test_not_ok_and_frames_frozen_recovers():
    # is_ok() false AND the kernel frame timestamp stopped advancing -> real loss.
    node = _make_node(
        is_ok=False, frame_ts=100.0, node_feedback_age_s=0.0,
        frames_have_advanced=False,
    )
    assert node._should_recover_bus() is True
    assert node._recovery_reason_counts.get("not_ok") == 1


def test_node_clock_stale_but_frames_advancing_is_suppressed():
    # is_ok() healthy, node-observed clock aged out by a publish-loop stall, but
    # kernel frames kept advancing -> suppress, do not recover.
    node = _make_node(
        is_ok=True, frame_ts=205.0, node_feedback_age_s=10.0,
        frames_have_advanced=False,
    )
    node._last_feedback_frame_ts = 204.0  # 205.0 != 204.0 -> advanced
    assert node._should_recover_bus() is False
    assert node._loop_overrun_suppressions == 1
    assert node._recovery_reason_counts == {}


def test_node_clock_stale_and_frames_frozen_recovers():
    # is_ok() healthy but both the node clock and the kernel timestamp are stale.
    node = _make_node(
        is_ok=True, frame_ts=200.0, node_feedback_age_s=10.0,
        frames_have_advanced=False,
    )
    assert node._should_recover_bus() is True
    assert node._recovery_reason_counts.get("stale_feedback") == 1
