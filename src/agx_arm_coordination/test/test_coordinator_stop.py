"""Unit tests for the coordinator's phased arm dispatch.

A recorded replay is dispatched as an *ordered pair* of MoveIt goals (planned
approach to waypoint 0, then the replay). If phase 1 fails, the replay must not
be sent -- that would run a taught motion from the wrong place.
"""

from agx_arm_coordination.coordinator_node import _PhasedArmChild


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
