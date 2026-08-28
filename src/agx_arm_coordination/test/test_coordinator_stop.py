"""Unit tests for the coordinator's stop path and phased arm dispatch.

Two things are guarded here, both safety properties rather than features:

- a recorded replay is dispatched as an *ordered pair* of MoveIt goals (planned
  approach to waypoint 0, then the replay). If phase 1 fails, the replay must not
  be sent -- that would run a taught motion from the wrong place.
- Ctrl+C must reach the hardware. rclpy's default SIGINT handler drops the
  context immediately, which strands a running MoveIt goal with no client left to
  cancel it, so the coordinator takes the signal itself and unwinds: cancel
  children -> reopen hand windows -> pin the arms, in that order.

The node needs ROS to construct, so tests build a bare instance via ``__new__``.
"""

import threading

import pytest

from agx_arm_coordination.coordinator_node import CoordinatorNode, _PhasedArmChild
from agx_arm_coordination.unit_activity import UnitActivity


SUCCESS = 1
FAILURE = -1


# --- fakes -------------------------------------------------------------------

class _Future:
    def __init__(self, value):
        self._value = value

    def done(self):
        return True

    def result(self):
        return self._value


class _ResultWrapper:
    def __init__(self, code):
        self.result = type("R", (), {"error_code": type("E", (), {"val": code})()})()


class _GoalHandle:
    def __init__(self, code, accepted=True):
        self.accepted = accepted
        self._code = code
        self.cancelled = False

    def get_result_async(self):
        return _Future(_ResultWrapper(self._code))

    def cancel_goal_async(self):
        self.cancelled = True


def _phase(code, log, name):
    def send():
        log.append(name)
        return _Future(_GoalHandle(code))
    return send


def _drain(child, limit=20):
    for _ in range(limit):
        if child.done:
            return
        child.poll()
    raise AssertionError("child never completed")


class _RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, *_a, **_k):
        self.messages.append(("info", str(msg)))

    def warn(self, msg, *_a, **_k):
        self.messages.append(("warn", str(msg)))

    def error(self, msg, *_a, **_k):
        self.messages.append(("error", str(msg)))


def _coord():
    node = CoordinatorNode.__new__(CoordinatorNode)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node._stop_lock = threading.Lock()
    node._stop_requested = False
    node._stop_reason = ""
    node._unit_activity = UnitActivity()
    # Admission is fail-closed on unit safety; this test is about the stop
    # path, so it satisfies that precondition rather than opting out.
    node._unit_activity.observe_unit_safety(epoch=1, stopped=False, reason='test')
    node._shutdown_event = threading.Event()
    # A stop wakes the activity loop rather than letting it wait out a watchdog
    # tick, so the bare node needs the same event the real one has.
    node._progress = threading.Event()
    node._open_hand_windows = set()
    return node


# --- phased arm dispatch -----------------------------------------------------

def test_phased_child_runs_phases_in_order_and_completes():
    log = []
    child = _PhasedArmChild(
        10, "left_arm_pour_tea",
        phases=[_phase(SUCCESS, log, "approach"), _phase(SUCCESS, log, "replay")],
        labels=["approach to waypoint 0", "recorded replay"],
    )
    child.start()
    _drain(child)
    assert log == ["approach", "replay"]
    assert child.success


def test_phased_child_does_not_replay_when_the_approach_fails():
    # The whole point of the approach phase: if the arm did not get to waypoint 0,
    # replaying the taught motion would run it from somewhere it was never taught.
    log = []
    child = _PhasedArmChild(
        10, "left_arm_pour_tea",
        phases=[_phase(FAILURE, log, "approach"), _phase(SUCCESS, log, "replay")],
        labels=["approach to waypoint 0", "recorded replay"],
    )
    child.start()
    _drain(child)
    assert log == ["approach"], "replay must not be dispatched after a failed approach"
    assert not child.success
    assert "approach to waypoint 0" in child.message


def test_phased_child_reports_which_phase_failed():
    log = []
    child = _PhasedArmChild(
        10, "left_arm_pour_tea",
        phases=[_phase(SUCCESS, log, "approach"), _phase(FAILURE, log, "replay")],
        labels=["approach to waypoint 0", "recorded replay"],
    )
    child.start()
    _drain(child)
    assert not child.success
    assert "recorded replay" in child.message


def test_phased_child_cancel_targets_the_current_phase():
    log = []
    child = _PhasedArmChild(
        10, "left_arm_pour_tea",
        phases=[_phase(SUCCESS, log, "approach"), _phase(SUCCESS, log, "replay")],
        labels=["approach", "replay"],
    )
    child.start()
    child.poll()  # resolve the goal handle for phase 1
    handle = child._goal_handle
    child.request_cancel()
    assert handle.cancelled


# --- cooperative stop --------------------------------------------------------

def test_request_stop_releases_main_when_no_activity_runs():
    node = _coord()
    node.request_stop("interrupt")
    assert node.stop_requested
    assert node._shutdown_event.is_set()


def test_request_stop_waits_for_a_running_activity():
    # Releasing main() here would tear the context down mid-trajectory, before the
    # activity thread has had a chance to cancel anything.
    node = _coord()
    node._unit_activity.try_claim("tea_pour_left_v1")
    node.request_stop("interrupt")
    assert node.stop_requested
    assert not node._shutdown_event.is_set()


def test_request_stop_is_idempotent():
    node = _coord()
    node.request_stop("first")
    node.request_stop("second")
    assert node._stop_reason == "first"


@pytest.mark.parametrize("robot_id,expected", [
    ("left_arm", {"left"}),
    ("right_arm", {"right"}),
    ("both_arms", {"left", "right"}),
    ("left_hand", set()),
    ("", set()),
])
def test_sides_for_robot(robot_id, expected):
    assert _coord()._sides_for_robot(robot_id) == expected


def _stop_order_coord(sides):
    """Coordinator stub recording the order of its stop steps."""
    node = _coord()
    calls = []
    node.cancel_arm_trajectories = lambda s, reason: calls.append(f"drop:{sorted(s)}")
    node._cancel_children = lambda running: calls.append("cancel_children")
    node._resume_all_hand_windows = lambda: calls.append("resume")
    node.safe_stop_arms = lambda s, reason: calls.append(f"pin:{sorted(s)}")
    node._sides_in_flight = lambda running: set(sides)
    return node, calls


def test_stop_running_drops_the_trajectory_before_waiting_on_the_children():
    # Order is load bearing twice over. Dropping the MIT trajectory needs nothing
    # from MoveIt, so it must not sit behind _cancel_children, which waits out
    # cleanup_timeout when a MoveGroup goal does not answer its cancel — measured
    # at 3.1 s of continued duo motion after a Ctrl+C. And the hold has to come
    # after resume_arm_control, because an open hand window closes the arm's MIT
    # gate and the hold would be dropped before reaching the arm.
    node, calls = _stop_order_coord({"left"})

    class _Child:
        action_nos = [1]
        done = False

    node._stop_running({1: _Child()}, "interrupt")
    assert calls == ["drop:['left']", "cancel_children", "resume", "pin:['left']"]


def test_stop_running_skips_arm_commands_when_no_arm_was_moving():
    node, calls = _stop_order_coord(set())
    node._stop_running({}, "interrupt")
    assert calls == ["cancel_children", "resume"]


def test_request_cancel_arms_the_cancel_on_a_goal_still_being_accepted():
    """A goal sent but not yet accepted still has to be stopped.

    ``request_cancel`` used to return quietly when the handle was not resolved
    yet, which let a dispatched motion run on through the stop that was supposed
    to end it.
    """
    from agx_arm_coordination.coordinator_node import _Child as Child

    class _PendingFuture:
        def __init__(self):
            self._callback = None
            self._handle = None

        def done(self):
            return self._handle is not None

        def result(self):
            return self._handle

        def add_done_callback(self, callback):
            self._callback = callback

        def accept(self, handle):
            self._handle = handle
            if self._callback is not None:
                self._callback(self)

    future = _PendingFuture()
    child = Child(10, "both_arms_can_pour")
    child.attach_goal_future(future)

    child.request_cancel()          # acceptance has not come back yet
    handle = _GoalHandle(SUCCESS)
    future.accept(handle)           # ... and now it does
    assert handle.cancelled, "a goal accepted after the stop must still be cancelled"


# --- emergency stop outcome reporting ----------------------------------------

def _estop_coord(results):
    """Coordinator stub whose per-side e-stop returns (ok, message)."""
    node = _coord()
    node._estop_clients = {"left": object(), "right": object()}
    node._call_trigger_sync = lambda client, label: results[
        "left" if "left" in label else "right"
    ]
    return node


def test_emergency_stop_demands_cutting_power_when_a_side_is_unverified():
    """An unverified software stop must not scroll past as one error line.

    The driver reports three outcomes — verified, contradicted by feedback, and
    commanded-while-unverifiable. The last two differ as diagnoses but not as
    situations: the arm must be assumed to be moving.
    """
    node = _estop_coord({
        "left": (True, "left_arm stop=verified — confirmed stopped (peak 0.001 rad/s)"),
        "right": (False, "right_arm stop=commanded_unverifiable (no joint feedback)"),
    })
    node.emergency_stop_all()

    errors = [m for level, m in node._logger.messages if level == "error"]
    assert any("CUT ARM POWER" in m for m in errors)
    demand = next(m for m in errors if "CUT ARM POWER" in m)
    assert "right" in demand
    assert "left" not in demand, "a verified side must not be named as unconfirmed"


def test_emergency_stop_stays_quiet_when_every_side_is_verified():
    node = _estop_coord({
        "left": (True, "left_arm stop=verified — confirmed stopped (peak 0.000 rad/s)"),
        "right": (True, "right_arm stop=verified — confirmed stopped (peak 0.002 rad/s)"),
    })
    node.emergency_stop_all()

    errors = [m for level, m in node._logger.messages if level == "error"]
    assert not errors, f"verified stops must not raise an alarm: {errors}"
