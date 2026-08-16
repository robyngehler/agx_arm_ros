"""Unit tests for the ROS-free graph model, validation, and scheduler."""

from agx_arm_coordination.graph_model import (
    ROBOT_UNITS_DEDICATED,
    ROBOT_UNITS_SHARED,
    Scheduler,
    conflicts,
    parse_activity,
    parse_catalogue,
    robot_units,
    units_for,
    validate_activity,
)


CATALOGUE = parse_catalogue({
    "actions": {
        "left_hand_open": {"actiontype_id": "Gripper", "robot_id": "left_hand"},
        "right_hand_open": {"actiontype_id": "Gripper", "robot_id": "right_hand"},
        "both_arms_move": {"actiontype_id": "Trajectory", "robot_id": "both_arms"},
        "left_arm_move": {"actiontype_id": "Trajectory", "robot_id": "left_arm"},
        "right_arm_move": {"actiontype_id": "Trajectory", "robot_id": "right_arm"},
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
    # both_arms owns both side CAN buses, so it now blocks either hand too.
    assert conflicts("both_arms", "left_hand")


def test_same_side_arm_and_hand_share_can_bus():
    # The default table is the shared one: each side's arm and hand hold one
    # physical CAN bus, so they conflict and cannot run concurrently
    # (Step-and-Settle: the arm owns the side bus, the hand only gets explicit
    # windows). Opposite sides stay independent.
    assert conflicts("right_arm", "right_hand")
    assert conflicts("left_arm", "left_hand")
    assert not conflicts("right_arm", "left_hand")
    assert not conflicts("left_arm", "right_hand")
    assert conflicts("both_arms", "right_hand")


# --- the topology decides the resource model ---------------------------------

def test_a_dedicated_bus_frees_a_hand_from_its_own_arm():
    """The whole point of four buses: a hand no longer waits for its arm."""
    units = robot_units("dedicated_per_device")

    assert not conflicts("right_arm", "right_hand", units)
    assert not conflicts("left_arm", "left_hand", units)
    # A device is still serialized against itself, and both_arms still takes
    # both arms — only the arm/hand coupling is gone.
    assert conflicts("both_arms", "left_arm", units)
    assert conflicts("both_arms", "right_arm", units)
    assert not conflicts("left_hand", "right_hand", units)


def test_both_arms_no_longer_blocks_a_hand_on_dedicated_buses():
    """Under the shared table it did, because it held both side buses."""
    units = robot_units("dedicated_per_device")

    assert conflicts("both_arms", "right_hand", ROBOT_UNITS_SHARED)
    assert not conflicts("both_arms", "right_hand", units)


def test_an_unknown_topology_reads_as_shared():
    """The conservative direction. Parallel operation is never the fallback."""
    assert robot_units("shared_per_side") is ROBOT_UNITS_SHARED
    assert robot_units("") is ROBOT_UNITS_SHARED
    assert robot_units("four_buses_probably") is ROBOT_UNITS_SHARED
    assert robot_units("dedicated_per_device") is ROBOT_UNITS_DEDICATED


def test_a_scheduler_given_no_table_serializes():
    """A caller that never thought about the topology must not get parallelism."""
    graph = _activity(
        [{"action_no": 10, "action_id": "left_arm_move"},
         {"action_no": 20, "action_id": "left_hand_open"}],
        [],
    )

    assert [i.action_no for i in Scheduler(graph, CATALOGUE).next_batch(set(), set())] == [10]


def test_a_scheduler_on_dedicated_buses_runs_arm_and_hand_together():
    """The interleaving 2B makes reachable, and nothing else pins it."""
    graph = _activity(
        [{"action_no": 10, "action_id": "left_arm_move"},
         {"action_no": 20, "action_id": "left_hand_open"}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE, ROBOT_UNITS_DEDICATED)

    assert {i.action_no for i in sched.next_batch(set(), set())} == {10, 20}
    # And a hand action still starts while the same side's arm is already running.
    assert [i.action_no for i in sched.next_batch(set(), {10})] == [20]


def test_units_for_unknown_is_empty():
    assert units_for("nope") == frozenset()


# --- validation --------------------------------------------------------------

def test_valid_activity_has_no_problems():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open"},
         {"action_no": 20, "action_id": "both_arms_move"}],
        [[10, 20]],
    )
    assert validate_activity(graph, CATALOGUE, ROBOT_UNITS_SHARED) == []


def test_unknown_action_is_flagged():
    graph = _activity([{"action_no": 10, "action_id": "ghost"}], [])
    problems = validate_activity(graph, CATALOGUE, ROBOT_UNITS_SHARED)
    assert any("ghost" in p for p in problems)


def test_cycle_is_flagged():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_hand_open"},
         {"action_no": 20, "action_id": "right_hand_open"}],
        [[10, 20], [20, 10]],
    )
    assert any("cyclic" in p for p in validate_activity(graph, CATALOGUE, ROBOT_UNITS_SHARED))


def test_sync_group_resource_conflict_is_flagged():
    # both_arms and left_arm in the same sync group cannot run in parallel.
    graph = _activity(
        [{"action_no": 10, "action_id": "both_arms_move", "sync_flag": 1},
         {"action_no": 11, "action_id": "left_arm_move", "sync_flag": 1}],
        [],
    )
    problems = validate_activity(graph, CATALOGUE, ROBOT_UNITS_SHARED)
    assert any("cannot run in parallel" in p for p in problems)


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


# --- validation and scheduling must read the same topology -------------------

def test_a_synced_arm_and_hand_are_valid_on_dedicated_buses():
    """Validation used to reject what the scheduler would happily have run.

    Under ``dedicated_per_device`` a side's arm and hand hold different bus
    tokens, so synchronizing them is exactly the parallelism the four-bus
    rewiring was for. Validation defaulted to the shared table and called it a
    resource conflict, so the activity was refused before the scheduler — which
    was configured for dedicated buses — ever saw it.
    """
    for side in ("left", "right"):
        graph = _activity(
            [{"action_no": 10, "action_id": f"{side}_arm_move", "sync_flag": 5},
             {"action_no": 11, "action_id": f"{side}_hand_open", "sync_flag": 5}],
            [],
        )
        assert validate_activity(graph, CATALOGUE, ROBOT_UNITS_DEDICATED) == [], (
            f"{side} arm+hand rejected on dedicated buses"
        )


def test_a_synced_arm_and_hand_are_a_conflict_on_a_shared_side_bus():
    """The same activity, the other wiring: one bus, so the barrier is a lie."""
    for side in ("left", "right"):
        graph = _activity(
            [{"action_no": 10, "action_id": f"{side}_arm_move", "sync_flag": 5},
             {"action_no": 11, "action_id": f"{side}_hand_open", "sync_flag": 5}],
            [],
        )
        problems = validate_activity(graph, CATALOGUE, ROBOT_UNITS_SHARED)
        assert any("cannot run in parallel" in p for p in problems), (
            f"{side} arm+hand accepted on a shared side bus"
        )


# --- sync groups are admitted whole or not at all ----------------------------

def test_an_independent_action_cannot_half_release_a_sync_group():
    """The defect: greedy per-action admission split a synchronization barrier.

    ``A`` (independent, left arm) and the synced pair ``B``/``C`` are all ready.
    Admitting one action at a time let ``A`` take the left arm, skip ``B`` for
    conflicting, and dispatch ``C`` alone — half a barrier.
    """
    graph = _activity(
        [{"action_no": 10, "action_id": "left_arm_move"},
         {"action_no": 20, "action_id": "left_arm_move", "sync_flag": 5},
         {"action_no": 21, "action_id": "right_hand_open", "sync_flag": 5}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE, ROBOT_UNITS_DEDICATED)
    batch = {i.action_no for i in sched.next_batch(set(), set())}

    assert 21 not in batch or 20 in batch, (
        "a sync group member was dispatched without the rest of its group"
    )
    assert batch == {10}, batch

    # Once the independent action finishes, the whole group goes together.
    assert {i.action_no for i in sched.next_batch({10}, set())} == {20, 21}


def test_two_sync_groups_competing_for_one_device_admit_one_whole_group():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_arm_move", "sync_flag": 1},
         {"action_no": 11, "action_id": "right_hand_open", "sync_flag": 1},
         {"action_no": 20, "action_id": "left_arm_move", "sync_flag": 2},
         {"action_no": 21, "action_id": "left_hand_open", "sync_flag": 2}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE, ROBOT_UNITS_DEDICATED)
    batch = {i.action_no for i in sched.next_batch(set(), set())}

    # Both groups want the left arm, so exactly one of them runs — whole.
    assert batch == {10, 11}, batch
    assert {i.action_no for i in sched.next_batch({10, 11}, set())} == {20, 21}


def test_a_sync_group_blocked_by_a_running_action_admits_none_of_itself():
    graph = _activity(
        [{"action_no": 10, "action_id": "left_arm_move"},
         {"action_no": 20, "action_id": "left_arm_move", "sync_flag": 7},
         {"action_no": 21, "action_id": "right_hand_open", "sync_flag": 7}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE, ROBOT_UNITS_DEDICATED)

    assert sched.next_batch(set(), {10}) == [], (
        "the free member of a blocked sync group was dispatched on its own"
    )


def test_a_sync_group_that_contends_with_itself_is_never_admitted():
    """validate_activity rejects this; the scheduler must not run it anyway."""
    graph = _activity(
        [{"action_no": 10, "action_id": "both_arms_move", "sync_flag": 3},
         {"action_no": 11, "action_id": "left_arm_move", "sync_flag": 3}],
        [],
    )
    sched = Scheduler(graph, CATALOGUE, ROBOT_UNITS_DEDICATED)

    assert sched.next_batch(set(), set()) == [], (
        "a self-conflicting sync group was dispatched, putting two commanders "
        "on one device"
    )
