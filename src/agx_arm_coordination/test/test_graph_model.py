"""Unit tests for the ROS-free graph model, validation, and scheduler."""

from agx_arm_coordination.graph_model import (
    Scheduler,
    conflicts,
    parse_activity,
    parse_catalogue,
    units_for,
    validate_activity,
)


CATALOGUE = parse_catalogue({
    "actions": {
        "left_hand_open": {"actiontype_id": "Gripper", "robot_id": "left_hand"},
        "right_hand_open": {"actiontype_id": "Gripper", "robot_id": "right_hand"},
        "both_arms_move": {"actiontype_id": "Trajectory", "robot_id": "both_arms"},
        "left_arm_move": {"actiontype_id": "Trajectory", "robot_id": "left_arm"},
    }
})


def _activity(nodes, edges):
    return parse_activity("t", {"nodes": nodes, "edges": edges})


# --- resources ---------------------------------------------------------------

def test_both_arms_conflicts_with_each_per_arm():
    assert conflicts("both_arms", "left_arm")
    assert conflicts("both_arms", "right_arm")
    assert not conflicts("left_arm", "right_arm")
    assert not conflicts("left_hand", "right_hand")
    assert not conflicts("both_arms", "left_hand")


def test_units_for_unknown_is_empty():
    assert units_for("nope") == frozenset()


# --- validation --------------------------------------------------------------

def test_valid_activity_has_no_problems():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open"},
         {"action_no": 20, "action_id": "both_arms_move"}],
        [[10, 20]],
    )
    assert validate_activity(graph, CATALOGUE) == []


def test_unknown_action_is_flagged():
    graph = _activity([{"action_no": 10, "action_id": "ghost"}], [])
    problems = validate_activity(graph, CATALOGUE)
    assert any("ghost" in p for p in problems)


def test_cycle_is_flagged():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open"},
         {"action_no": 20, "action_id": "right_hand_open"}],
        [[10, 20], [20, 10]],
    )
    assert any("cyclic" in p for p in validate_activity(graph, CATALOGUE))


def test_sync_group_resource_conflict_is_flagged():
    # both_arms and left_arm in the same sync group cannot run in parallel.
    graph = _activity(
        [{"action_no": 10, "action_id": "both_arms_move", "sync_flag": 1},
         {"action_no": 11, "action_id": "left_arm_move", "sync_flag": 1}],
        [],
    )
    assert any("cannot run in parallel" in p for p in validate_activity(graph, CATALOGUE))


# --- scheduler ---------------------------------------------------------------

def test_scheduler_parallel_independent_hands():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open", "sync_flag": 1},
         {"action_no": 11, "action_id": "right_hand_open", "sync_flag": 1}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE)
    batch = sched.next_batch(set(), set())
    assert {item.action_no for item in batch} == {10, 11}


def test_scheduler_serializes_both_arms_against_per_arm():
    graph = _activity(
        [{"action_no": 10, "action_id": "both_arms_move"},
         {"action_no": 20, "action_id": "left_arm_move"}],
        [],  # no edge => both have satisfied predecessors
    )
    sched = Scheduler(graph, CATALOGUE)
    batch = sched.next_batch(set(), set())
    # both are ready, but they conflict; only the lower action_no is admitted.
    assert [item.action_no for item in batch] == [10]
    # while both_arms runs, left_arm stays blocked
    assert sched.next_batch(set(), {10}) == []
    # once both_arms completes, left_arm is dispatchable
    assert [i.action_no for i in sched.next_batch({10}, set())] == [20]


def test_scheduler_sync_barrier_waits_for_whole_group():
    # 10 has no predecessor; 11 waits on 99 -> the sync group is not all ready,
    # so neither member dispatches until 11 is also ready.
    graph = _activity(
        [{"action_no": 5, "action_id": "both_arms_move"},
         {"action_no": 10, "action_id": "left_hand_open", "sync_flag": 1},
         {"action_no": 11, "action_id": "right_hand_open", "sync_flag": 1}],
        [[5, 11]],
    )
    sched = Scheduler(graph, CATALOGUE)
    # initially only 5 is ready; 10 is held by its sync sibling 11 (blocked on 5)
    assert [i.action_no for i in sched.next_batch(set(), set())] == [5]
    # after 5 completes, both 10 and 11 become ready and release together
    assert {i.action_no for i in sched.next_batch({5}, set())} == {10, 11}


def test_scheduler_respects_predecessors():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open"},
         {"action_no": 20, "action_id": "right_hand_open"}],
        [[10, 20]],
    )
    sched = Scheduler(graph, CATALOGUE)
    assert [i.action_no for i in sched.next_batch(set(), set())] == [10]
    assert sched.next_batch(set(), {10}) == []
    assert [i.action_no for i in sched.next_batch({10}, set())] == [20]
    assert sched.is_complete({10, 20})
