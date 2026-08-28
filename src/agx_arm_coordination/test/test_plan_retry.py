"""Replanning after a MoveIt failure a fresh goal may get past.

A goal state that only marginally clears the collision model fails
intermittently, and MoveIt's own ``num_planning_attempts`` cannot help: the
attempts share one goal sampler, so once it has failed to find valid goal states
the rest return in under a millisecond. Only a **new goal** rebuilds it, which is
what these tests are about.

What must not be retried is as much of the contract as what must: an execution
failure leaves the arm somewhere unknown, and a configuration error would just
be refused three times more slowly.

Test level: **L1**.
"""

from moveit_msgs.msg import MoveItErrorCodes

from agx_arm_coordination.coordinator_node import (
    RETRYABLE_MOVEIT_CODES,
    CoordinatorNode,
    _ArmChild,
    _HandChild,
    _PhasedArmChild,
)


def _node(attempts=2):
    node = CoordinatorNode.__new__(CoordinatorNode)
    node.plan_retry_attempts = attempts
    node.get_logger = lambda: type("L", (), {
        "warn": staticmethod(lambda *a, **k: None),
        "error": staticmethod(lambda *a, **k: None),
        "info": staticmethod(lambda *a, **k: None),
    })()
    node._event = lambda *a, **k: None
    return node


def _failed(code, action_id="anchor_move"):
    child = _ArmChild(10, action_id)
    child.error_code = code
    child.mark(False, f"MoveIt error_code={code}")
    child.respawn = lambda: _ArmChild(10, action_id)
    return child


def test_a_marginal_planning_failure_is_replanned():
    """99999 is what a goal at the edge of the collision model returns."""
    node = _node()
    replacement = node._retry_child(_failed(MoveItErrorCodes.FAILURE), "demo")
    assert replacement is not None
    assert replacement.attempt == 2
    assert replacement.action_no == 10


def test_the_retry_budget_is_finite():
    node = _node(attempts=2)
    child = _failed(MoveItErrorCodes.FAILURE)
    seen = []
    for _ in range(5):
        replacement = node._retry_child(child, "demo")
        if replacement is None:
            break
        seen.append(replacement.attempt)
        replacement.error_code = MoveItErrorCodes.FAILURE
        replacement.mark(False, "MoveIt error_code=99999")
        replacement.respawn = child.respawn
        child = replacement
    assert seen == [2, 3]          # the original try plus two replans


def test_retries_can_be_switched_off():
    assert _node(attempts=0)._retry_child(_failed(MoveItErrorCodes.FAILURE), "demo") is None


def test_an_execution_failure_is_never_replanned():
    """CONTROL_FAILED means the motion started and stopped somewhere unknown.

    Re-sending the same goal would plan from a state nobody has established.
    """
    node = _node()
    assert node._retry_child(_failed(MoveItErrorCodes.CONTROL_FAILED), "demo") is None


def test_a_cancel_is_never_replanned():
    node = _node()
    assert node._retry_child(_failed(MoveItErrorCodes.PREEMPTED), "demo") is None


def test_a_configuration_error_is_not_retried():
    """Deterministic: retrying makes the same refusal three times slower."""
    node = _node()
    for code in (MoveItErrorCodes.INVALID_GROUP_NAME,
                 MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
                 MoveItErrorCodes.INVALID_LINK_NAME):
        assert node._retry_child(_failed(code), "demo") is None


def test_a_hand_failure_is_not_replanned():
    """The hand path is fail-closed on device authority, not a sampler."""
    node = _node()
    child = _HandChild(20, "left_hand_can_grip")
    child.mark(False, "claim refused")
    child.respawn = lambda: _HandChild(20, "left_hand_can_grip")
    assert node._retry_child(child, "demo") is None


def test_a_replay_is_retried_only_while_its_approach_is_what_failed():
    """Past the approach the arm is somewhere along a taught path; re-running it
    is a motion decision, not a retry."""
    node = _node()

    def phased(phase_index):
        child = _PhasedArmChild(30, "left_arm_can_pour", phases=[lambda: None] * 2,
                                labels=["approach to waypoint 0", "recorded replay"])
        child._phase_index = phase_index
        child.error_code = MoveItErrorCodes.FAILURE
        child.mark(False, "MoveIt error_code=99999")
        child.respawn = lambda: _PhasedArmChild(
            30, "left_arm_can_pour", phases=[lambda: None] * 2, labels=["a", "b"]
        )
        return child

    assert node._retry_child(phased(0), "demo") is not None    # the planned approach
    assert node._retry_child(phased(1), "demo") is None        # the replay itself


def test_a_child_with_no_way_to_dispatch_again_is_not_retried():
    node = _node()
    child = _failed(MoveItErrorCodes.FAILURE)
    child.respawn = None
    assert node._retry_child(child, "demo") is None


def test_a_respawn_that_cannot_dispatch_fails_the_activity():
    """A merged sync group whose plans no longer merge returns None rather than
    raising; the activity must then fail instead of looping."""
    node = _node()
    child = _failed(MoveItErrorCodes.FAILURE)
    child.respawn = lambda: None
    assert node._retry_child(child, "demo") is None


def test_the_retryable_set_is_planning_side_only():
    assert MoveItErrorCodes.SUCCESS not in RETRYABLE_MOVEIT_CODES
    assert MoveItErrorCodes.CONTROL_FAILED not in RETRYABLE_MOVEIT_CODES
    assert MoveItErrorCodes.PREEMPTED not in RETRYABLE_MOVEIT_CODES
    # The two that made this demo fail: a goal at the edge of the model, and the
    # generic failure MoveIt reports when its goal sampler comes up empty.
    assert MoveItErrorCodes.GOAL_IN_COLLISION in RETRYABLE_MOVEIT_CODES
    assert MoveItErrorCodes.FAILURE in RETRYABLE_MOVEIT_CODES
