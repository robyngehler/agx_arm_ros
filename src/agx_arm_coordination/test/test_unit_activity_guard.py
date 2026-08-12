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


def test_a_ready_unit_takes_one_activity():
    unit = UnitActivity()
    assert unit.state is UnitActivityState.READY

    assert unit.try_claim("tea_pour_left_v1").accepted
    assert unit.state is UnitActivityState.EXECUTING
    assert unit.activity_id == "tea_pour_left_v1"


def test_a_second_activity_is_refused_with_a_reason_naming_both():
    unit = UnitActivity()
    unit.try_claim("tea_pour_left_v1")

    admission = unit.try_claim("handover_right_v1")
    assert not admission.accepted
    assert admission.reason is RejectReason.UNIT_BUSY
    assert "tea_pour_left_v1" in admission.detail
    assert "handover_right_v1" in admission.detail
    assert unit.rejected_count == 1


def test_the_unit_is_free_again_after_release():
    unit = UnitActivity()
    unit.try_claim("tea_pour_left_v1")
    unit.release("tea_pour_left_v1")

    assert unit.state is UnitActivityState.READY
    assert unit.try_claim("handover_right_v1").accepted


def test_a_late_unwind_cannot_free_a_slot_it_no_longer_holds():
    """A refused activity releasing on its way out must not open the door."""
    unit = UnitActivity()
    unit.try_claim("tea_pour_left_v1")
    assert not unit.try_claim("handover_right_v1").accepted

    unit.release("handover_right_v1")

    assert unit.activity_id == "tea_pour_left_v1"
    assert not unit.try_claim("something_else").accepted


def test_only_one_of_many_concurrent_claims_wins():
    """try_claim decides and takes the slot in one step, on purpose."""
    unit = UnitActivity()
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
    unit = UnitActivity()
    unit.begin_stop("interrupt")

    admission = unit.try_claim("tea_pour_left_v1")
    assert not admission.accepted
    assert admission.reason is RejectReason.UNIT_STOPPING
    assert "interrupt" in admission.detail


def test_begin_stop_reports_whether_something_still_has_to_unwind():
    idle = UnitActivity()
    assert idle.begin_stop("interrupt") is False

    busy = UnitActivity()
    busy.try_claim("tea_pour_left_v1")
    assert busy.begin_stop("interrupt") is True


def test_the_first_stop_reason_is_the_one_kept():
    unit = UnitActivity()
    unit.begin_stop("interrupt")
    unit.begin_stop("second thoughts")
    assert unit.stop_reason == "interrupt"


def test_can_accept_does_not_take_the_slot():
    unit = UnitActivity()
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
    node._unit_activity = UnitActivity()
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
