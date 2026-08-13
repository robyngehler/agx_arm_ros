"""L1 tests for the one-activity-per-unit guard.

The coordinator used to accept every goal unconditionally on a reentrant
callback group, so two overlapping activities would both have executed against
the same arms. These tests hold the guard to the rule the refactor states:
``READY`` takes one activity, ``EXECUTING`` refuses the rest with a reason.
"""

import threading

from agx_arm_coordination.unit_activity import (
    RejectReason,
    UnitActivity,
    UnitActivityState,
)


def _ready_unit():
    """A unit whose safety generation is established.

    The exclusivity tests below are about one activity at a time, not about
    safety, so they satisfy that precondition rather than opting out of it —
    which keeps them running against the real fail-closed default.
    """
    unit = UnitActivity()
    unit.observe_unit_safety(epoch=1, stopped=False, reason="test")
    return unit


def test_a_ready_unit_takes_one_activity():
    unit = _ready_unit()
    assert unit.state is UnitActivityState.READY

    assert unit.try_claim("tea_pour_left_v1").accepted
    assert unit.state is UnitActivityState.EXECUTING
    assert unit.activity_id == "tea_pour_left_v1"


def test_a_second_activity_is_refused_with_a_reason_naming_both():
    unit = _ready_unit()
    unit.try_claim("tea_pour_left_v1")

    admission = unit.try_claim("handover_right_v1")
    assert not admission.accepted
    assert admission.reason is RejectReason.UNIT_BUSY
    assert "tea_pour_left_v1" in admission.detail
    assert "handover_right_v1" in admission.detail
    assert unit.rejected_count == 1


def test_the_unit_is_free_again_after_release():
    unit = _ready_unit()
    unit.try_claim("tea_pour_left_v1")
    unit.release("tea_pour_left_v1")

    assert unit.state is UnitActivityState.READY
    assert unit.try_claim("handover_right_v1").accepted


def test_a_late_unwind_cannot_free_a_slot_it_no_longer_holds():
    """A refused activity releasing on its way out must not open the door."""
    unit = _ready_unit()
    unit.try_claim("tea_pour_left_v1")
    assert not unit.try_claim("handover_right_v1").accepted

    unit.release("handover_right_v1")

    assert unit.activity_id == "tea_pour_left_v1"
    assert not unit.try_claim("something_else").accepted


def test_only_one_of_many_concurrent_claims_wins():
    """try_claim decides and takes the slot in one step, on purpose."""
    unit = _ready_unit()
    start = threading.Event()
    won = []

    def claim(index):
        start.wait(2.0)
        if unit.try_claim(f"activity_{index}").accepted:
            won.append(index)

    threads = [threading.Thread(target=claim, args=(i,)) for i in range(16)]
    for thread in threads:
        thread.start()
    start.set()
    for thread in threads:
        thread.join(2.0)

    assert len(won) == 1
    assert unit.rejected_count == 15


def test_a_stopping_unit_refuses_new_activities():
    unit = _ready_unit()
    unit.begin_stop("interrupt")

    admission = unit.try_claim("tea_pour_left_v1")
    assert not admission.accepted
    assert admission.reason is RejectReason.UNIT_STOPPING
    assert "interrupt" in admission.detail


def test_begin_stop_reports_whether_something_still_has_to_unwind():
    idle = _ready_unit()
    assert idle.begin_stop("interrupt") is False

    busy = _ready_unit()
    busy.try_claim("tea_pour_left_v1")
    assert busy.begin_stop("interrupt") is True


def test_the_first_stop_reason_is_the_one_kept():
    unit = _ready_unit()
    unit.begin_stop("interrupt")
    unit.begin_stop("second thoughts")
    assert unit.stop_reason == "interrupt"


def test_can_accept_does_not_take_the_slot():
    unit = _ready_unit()
    assert unit.can_accept("tea_pour_left_v1").accepted
    assert unit.state is UnitActivityState.READY
    assert unit.can_accept("tea_pour_left_v1").accepted


# --- coordinator wiring ------------------------------------------------------

from rclpy.action import GoalResponse  # noqa: E402

from agx_arm_coordination.coordinator_node import CoordinatorNode  # noqa: E402


class _RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, *_a, **_k):
        self.messages.append(("info", str(msg)))

    def warn(self, msg, *_a, **_k):
        self.messages.append(("warn", str(msg)))

    def error(self, msg, *_a, **_k):
        self.messages.append(("error", str(msg)))


class _Request:
    def __init__(self, activity_id):
        self.activity_id = activity_id


class _GoalHandle:
    def __init__(self, activity_id):
        self.request = _Request(activity_id)
        self.aborted = False

    def abort(self):
        self.aborted = True


def _coord():
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node._unit_activity = _ready_unit()
    node.events = []
    node._event = lambda event_type, **kw: node.events.append((event_type, kw))
    return node


def test_the_goal_callback_refuses_a_second_activity():
    node = _coord()
    assert node._on_goal(_Request("tea_pour_left_v1")) is GoalResponse.ACCEPT

    node._unit_activity.try_claim("tea_pour_left_v1")
    assert node._on_goal(_Request("handover_right_v1")) is GoalResponse.REJECT

    event_type, payload = node.events[-1]
    assert event_type == "rejected"
    assert payload["state"] == RejectReason.UNIT_BUSY.value


def test_execute_still_refuses_a_goal_that_slipped_past_the_callback():
    """Two goals can pass the non-mutating check at once; the claim decides."""
    node = _coord()
    node._unit_activity.try_claim("tea_pour_left_v1")

    goal = _GoalHandle("handover_right_v1")
    result = node._execute(goal)

    assert goal.aborted
    assert result.success is False
    assert "tea_pour_left_v1" in result.message


def test_a_rejected_goal_is_not_reported_as_a_failed_activity():
    """Nothing ran, so a consumer counting failures must not count it."""
    node = _coord()
    node._unit_activity.try_claim("tea_pour_left_v1")
    node._on_goal(_Request("handover_right_v1"))

    assert all(event_type != "failed" for event_type, _ in node.events)


# --- unit safety liveness ----------------------------------------------------
#
# The rule the split produces: losing the safety writer never stops what is
# already authorised, but it must stop new work from starting. A fresh activity
# looks harmless because nothing is moving yet, and it is exactly when the unit
# commits to motion it could not afterwards invalidate.

class _Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def _guarded(**kwargs):
    clock = _Clock()
    unit = UnitActivity(clock=clock, unit_safety_timeout_s=6.0, **kwargs)
    return unit, clock


def test_no_safety_state_at_all_refuses_new_work():
    unit, _ = _guarded()
    admission = unit.try_claim("tea_pour_left_v1")
    assert not admission.accepted
    assert admission.reason is RejectReason.UNIT_SAFETY_UNKNOWN
    assert "none has ever arrived" in admission.detail


def test_a_live_generation_admits_work():
    unit, _ = _guarded()
    unit.observe_unit_safety(epoch=3, stopped=False, reason="operator rearm")
    assert unit.try_claim("tea_pour_left_v1").accepted


def test_a_unit_stop_refuses_new_work_with_its_own_reason():
    unit, _ = _guarded()
    unit.observe_unit_safety(epoch=4, stopped=True, reason="emergency stop")
    admission = unit.try_claim("tea_pour_left_v1")
    assert admission.reason is RejectReason.UNIT_STOPPED
    assert "emergency stop" in admission.detail


def test_the_writer_going_quiet_refuses_new_work():
    """The latched value outlives the writer, so staleness is the only signal."""
    unit, clock = _guarded()
    unit.observe_unit_safety(epoch=3, stopped=False, reason="ok")
    assert unit.can_accept("a").accepted

    clock.advance(7.0)
    admission = unit.can_accept("tea_pour_left_v1")
    assert admission.reason is RejectReason.UNIT_SAFETY_UNKNOWN
    assert "7.0s ago" in admission.detail


def test_a_running_activity_is_untouched_when_the_writer_goes_quiet():
    """The whole point of the split: this must not stop an arm mid-trajectory."""
    unit, clock = _guarded()
    unit.observe_unit_safety(epoch=3, stopped=False, reason="ok")
    assert unit.try_claim("tea_pour_left_v1").accepted

    clock.advance(60.0)

    assert unit.is_running
    assert unit.activity_id == "tea_pour_left_v1"
    assert unit.state is UnitActivityState.EXECUTING
    # And it can still be released normally when it finishes on its own.
    unit.release("tea_pour_left_v1")
    assert not unit.is_running


def test_a_heartbeat_keeps_the_unit_admissible():
    unit, clock = _guarded()
    for _ in range(5):
        unit.observe_unit_safety(epoch=3, stopped=False, reason="heartbeat")
        clock.advance(2.0)
    assert unit.can_accept("tea_pour_left_v1").accepted


def test_the_development_profile_can_run_without_a_safety_writer():
    unit, _ = _guarded(require_unit_safety=False)
    assert unit.try_claim("tea_pour_left_v1").accepted


def test_a_stop_is_honoured_even_in_the_development_profile():
    """Opting out of *requiring* safety is not opting out of obeying it."""
    unit, _ = _guarded(require_unit_safety=False)
    unit.observe_unit_safety(epoch=4, stopped=True, reason="emergency stop")
    assert unit.try_claim("tea_pour_left_v1").reason is RejectReason.UNIT_STOPPED
