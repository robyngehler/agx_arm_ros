"""Resuming an activity from an operator step.

The step model and what it refuses, against the shipped activities rather than a
fixture: the numbers an operator types are only as good as the graphs they count.

Test level: **L1**. It proves the step mapping, the refusals and the metadata
plumbing. It proves nothing about whether resuming is safe on hardware — that is
what the replay refusal exists to bound.
"""

from pathlib import Path

import pytest

from agx_arm_coordination.coordinator_node import (
    _playback_override,
    _resume_from_step,
)
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import ROBOT_UNITS_DEDICATED, operator_steps
from agx_arm_coordination.operator_resume import (
    ResumeError,
    next_resume_step,
    parse_from_step,
    resumable_steps,
    resume_seed,
    step_of,
)
from agx_arm_coordination.run_activity_client import _with_resume


CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

TEA = "tea_pour_duo_v2"
#: The tea demo's recorded steps; a resume may not start on one.
TEA_RECORDED = {3, 6, 9, 10, 12, 18, 19}
TEA_STEPS = 21


def _catalogue() -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, ROBOT_UNITS_DEDICATED)


def _seed(activity: str, from_step: int):
    cat = _catalogue()
    return resume_seed(
        cat.get_activity_plan(activity),
        cat.actions,
        ROBOT_UNITS_DEDICATED,
        from_step,
    )


# --- the step number itself -------------------------------------------------

@pytest.mark.parametrize("given,expected", [(1, 1), ("7", 7), (21, 21)])
def test_a_step_number_is_read_as_written(given, expected):
    assert parse_from_step(given) == expected


@pytest.mark.parametrize("bad", [0, -1, "", "eight", None, True, 1.5])
def test_a_value_that_is_not_a_step_number_is_refused(bad):
    """1-based, so 0 is a mistake rather than "the whole activity".

    An operator who means the whole activity does not pass a resume at all, and
    reading 0 as 1 would hide a typo in a number that decides where an arm
    starts moving.
    """
    with pytest.raises(ResumeError):
        parse_from_step(bad)


# --- what a seed contains ---------------------------------------------------

def test_step_one_seeds_nothing():
    seed, total = _seed(TEA, 1)
    assert seed == set()
    assert total == TEA_STEPS


def test_a_later_step_seeds_every_node_of_every_earlier_step():
    """Nodes, not steps: a synchronized pair contributes both of its nodes."""
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(TEA), cat.actions, ROBOT_UNITS_DEDICATED
    )
    seed, _ = _seed(TEA, 5)
    expected = {item.action_no for batch in steps[:4] for item in batch}
    assert seed == expected
    # Step 4 is a sync group, so the seed is one node larger than the step count.
    assert len(seed) == 5


def test_the_last_step_seeds_all_but_itself():
    cat = _catalogue()
    graph = cat.get_activity_plan(TEA)
    seed, total = _seed(TEA, TEA_STEPS)
    assert total == TEA_STEPS
    assert len(seed) == len(graph.nodes) - 1


def test_a_step_past_the_end_is_refused_with_the_count():
    with pytest.raises(ResumeError, match=f"{TEA_STEPS} operator steps"):
        _seed(TEA, TEA_STEPS + 1)


# --- the replay refusal -----------------------------------------------------

@pytest.mark.parametrize("step", sorted(TEA_RECORDED))
def test_resuming_onto_a_replay_is_refused(step):
    """A replay commands taught joint angles from wherever the arm stands."""
    with pytest.raises(ResumeError, match="replays a taught path"):
        _seed(TEA, step)


@pytest.mark.parametrize("step", sorted(set(range(1, TEA_STEPS + 1)) - TEA_RECORDED))
def test_every_planned_step_is_a_valid_resume_point(step):
    seed, total = _seed(TEA, step)
    assert total == TEA_STEPS
    assert len(seed) >= step - 1


def test_the_refusal_names_a_step_the_operator_can_actually_use():
    """A rejection without a usable number just moves the guessing.

    Step 12 is the pour replay; the nearest earlier step that plans its own
    approach is 11, and that is what the message has to say.
    """
    with pytest.raises(ResumeError, match="earlier step that plans its own approach is 11"):
        _seed(TEA, 12)


def test_resumable_steps_are_the_complement_of_the_recorded_ones():
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(TEA), cat.actions, ROBOT_UNITS_DEDICATED
    )
    allowed = set(resumable_steps(steps, cat.actions))
    assert allowed == set(range(1, TEA_STEPS + 1)) - TEA_RECORDED


def test_a_pack_flow_is_resumable_at_every_step():
    """Which is the point of the one-way flows: no replay, so no refusal."""
    for step in (1, 2, 3):
        seed, total = _seed("unit_pack_bottom_fast_v1", step)
        assert total == 3
        assert len(seed) == step - 1


# --- mapping a reported node back to a step ---------------------------------

def test_a_reported_node_maps_back_to_its_step():
    """What a script needs to turn a failure into the next --from-id.

    Node 100 is the pour approach, dispatched tenth; node 171 is half of the
    closing sync pair, so it reports as step 20 like its partner.
    """
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(TEA), cat.actions, ROBOT_UNITS_DEDICATED
    )
    assert step_of(steps, 10) == 1
    assert step_of(steps, 100) == 10
    assert step_of(steps, 170) == 20
    assert step_of(steps, 171) == 20
    assert step_of(steps, 9999) == 0


# --- the metadata plumbing --------------------------------------------------

def test_a_goal_without_metadata_asks_for_no_resume():
    assert _resume_from_step("") is None
    assert _resume_from_step('{"playback": {"mode": "smooth"}}') is None


def test_a_resume_block_is_read_and_a_malformed_one_is_refused():
    assert _resume_from_step('{"resume": {"from_step": 8}}') == 8
    for payload in (
        '{"resume": 8}',
        '{"resume": {"step": 8}}',
        '{"resume": {"from_step": 0}}',
        '{"resume": {"from_step": "soon"}}',
    ):
        with pytest.raises(ValueError):
            _resume_from_step(payload)


def test_playback_and_resume_travel_in_the_same_object():
    payload = '{"playback": {"mode": "tempo_scale"}, "resume": {"from_step": 4}}'
    assert _playback_override(payload) == {"mode": "tempo_scale"}
    assert _resume_from_step(payload) == 4


# --- the CLI ----------------------------------------------------------------

def test_from_id_becomes_a_resume_block():
    import json

    assert json.loads(_with_resume("", 8)) == {"resume": {"from_step": 8}}
    assert json.loads(_with_resume('{"playback": {"mode": "smooth"}}', 2)) == {
        "playback": {"mode": "smooth"},
        "resume": {"from_step": 2},
    }


def test_without_from_id_the_metadata_passes_through_untouched():
    assert _with_resume("", None) == ""
    assert _with_resume('{"playback": {}}', None) == '{"playback": {}}'


def test_two_start_steps_are_refused_rather_than_one_of_them_winning():
    with pytest.raises(ValueError, match="declare one"):
        _with_resume('{"resume": {"from_step": 3}}', 8)


@pytest.mark.parametrize("bad", ["not json", '["a"]'])
def test_from_id_over_unusable_metadata_is_refused(bad):
    with pytest.raises(ValueError):
        _with_resume(bad, 2)


# --- the wiring: what a resumed run plans -----------------------------------

class _RecordingPlanner:
    """Counts what the coordinator asked it to plan, and plans nothing."""

    def __init__(self):
        self.planned: list[str] = []
        self.playback_override: dict = {}

    def plan(self, action):
        self.planned.append(action.action_id)
        return object()


def _bare_node(catalogue, planner):
    """A CoordinatorNode carrying only what _prewarm_arm_actions touches."""
    import threading

    from agx_arm_coordination.coordinator_node import CoordinatorNode

    node = CoordinatorNode.__new__(CoordinatorNode)
    node._stop_lock = threading.Lock()
    node._stop_requested = False
    node.arm_planner = planner
    node.arm_dry_run = False
    node.catalogue = catalogue
    node.get_logger = lambda: type("L", (), {
        "info": staticmethod(lambda *a, **k: None),
        "error": staticmethod(lambda *a, **k: None),
    })()
    return node


def test_a_resumed_run_does_not_plan_the_steps_it_skips():
    """Retiming one recording costs seconds, so a late resume must not pay for
    the whole graph. Also the reason the skip is by node and not by action_id:
    both_arms_to_functional_init is step 1 and step 20, and step 20 runs."""
    cat = _catalogue()
    graph = cat.get_activity_plan(TEA)

    whole = _RecordingPlanner()
    _bare_node(cat, whole)._prewarm_arm_actions(graph, TEA)

    seed, _ = _seed(TEA, 20)
    resumed = _RecordingPlanner()
    _bare_node(cat, resumed)._prewarm_arm_actions(graph, TEA, skip=seed)

    assert len(resumed.planned) < len(whole.planned)
    # Step 20 moves both arms to functional init and step 21 to the packing
    # pose; nothing else is dispatched, so nothing else is planned.
    assert set(resumed.planned) == {
        "both_arms_to_functional_init",
        "both_arms_to_packing_pose",
    }


def test_an_unresumed_run_plans_everything_it_did_before():
    """The skip is opt-in: a normal run is unchanged."""
    cat = _catalogue()
    graph = cat.get_activity_plan(TEA)
    planner = _RecordingPlanner()
    _bare_node(cat, planner)._prewarm_arm_actions(graph, TEA)
    arm_actions = {
        node.action_id
        for node in graph.nodes.values()
        if cat.get_action_detail(node.action_id).actiontype_id == "Trajectory"
    }
    assert set(planner.planned) == arm_actions


# --- picking a stopped run back up ------------------------------------------

def _tea_steps():
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(TEA), cat.actions, ROBOT_UNITS_DEDICATED
    )
    return steps, resumable_steps(steps, cat.actions)


def test_the_next_resume_step_skips_a_replay_that_follows():
    """Step 8 completed, 9 and 10 are replays, so the operator is sent to 11."""
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "both_arms_to_can_adjust_while_grip") == (8, 11)


def test_the_next_resume_step_is_the_one_immediately_after_when_it_is_planned():
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "both_arms_to_can_post_grip") == (15, 16)


def test_a_completed_replay_still_reports_the_step_it_was():
    """Reporting where the run got to is not the same question as where it may
    restart, so a replay is a legitimate answer to the first."""
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "both_arms_can_pour") == (12, 13)


def test_a_run_that_completed_nothing_has_no_resume_point():
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "") == (0, None)
    assert next_resume_step(steps, allowed, "an_action_from_another_activity") == (0, None)


def test_the_last_step_leaves_nothing_to_resume():
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "both_arms_to_packing_pose") == (21, None)


def test_a_sync_pair_reports_one_step_whichever_half_completed_last():
    steps, allowed = _tea_steps()
    assert next_resume_step(steps, allowed, "both_arms_to_functional_init")[0] == 20
    assert next_resume_step(steps, allowed, "left_hand_fist")[0] == 20
