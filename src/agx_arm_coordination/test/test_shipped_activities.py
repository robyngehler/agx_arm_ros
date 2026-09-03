"""Every activity in config/activities/, checked the way the coordinator loads it.

A sweep rather than a list: an activity added to the directory is picked up here
without anyone remembering to add it, which is the case this file exists for. An
anchor naming a pose that is not configured, an action id that is not in the
catalogue, or a graph that deadlocks fails here rather than with the arms part
way through the sequence.

Per-activity contracts — which step is which, what may be resumed onto — belong
in the file for that activity. This one only asserts what must hold for all of
them.

Test level: **L1**.
"""

from pathlib import Path

import pytest
import yaml

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmConfigError,
    ArmTrajectoryPlanner,
    NotTaughtError,
    is_replay,
)
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import (
    ACTIONTYPE_TRAJECTORY,
    ROBOT_UNITS_DEDICATED,
    operator_steps,
)
from agx_arm_coordination.operator_resume import resumable_steps
from agx_arm_coordination.performer import route


CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

ALL_SHIPPED = sorted(path.stem for path in (CONFIG_DIR / "activities").glob("*.yaml"))

#: Activities that still load and schedule but name anchor poses that are no
#: longer in arm_config.yaml, so they cannot be planned and cannot be run. Kept
#: for their taught waypoints and their sequence, not because they work. Each is
#: asserted below to still be broken, so an entry that gets fixed fails here
#: instead of quietly staying quarantined.
KNOWN_UNPLANNABLE = {
    "tea_pour_left_v1":
        "its eight anchors were re-captured under Tee-Can_* names "
        "(see docs/control/bringups/tea_demo.md)",
    "hefeweizen_pour_v1":
        "names Pre_Grip_L / grasp_L, from the pose set that predates the "
        "Tee-Can_* re-capture",
    "both_arms_pregrasp_grasp_retract_v1":
        "same pose set as hefeweizen_pour_v1",
}

SHIPPED = [name for name in ALL_SHIPPED if name not in KNOWN_UNPLANNABLE]


def _catalogue() -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, ROBOT_UNITS_DEDICATED)


def _planner() -> ArmTrajectoryPlanner:
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    return ArmTrajectoryPlanner(ArmConfig.from_dict(data["arm_executor"]))


def test_the_directory_is_not_empty():
    """A glob that matches nothing would make every test below vacuously pass."""
    assert len(SHIPPED) >= 8


@pytest.mark.parametrize("activity", sorted(KNOWN_UNPLANNABLE))
def test_a_quarantined_activity_is_still_the_one_that_is_broken(activity):
    """The quarantine list is checked, not trusted.

    An activity that was re-anchored must leave this list rather than sit in it
    claiming to be broken; one that breaks later must be added deliberately.
    """
    cat = _catalogue()
    planner = _planner()
    unplannable = []
    for node in cat.get_activity_plan(activity).nodes.values():
        action = cat.get_action_detail(node.action_id)
        if action.actiontype_id != ACTIONTYPE_TRAJECTORY:
            continue
        try:
            planner.plan(action)
        except NotTaughtError:
            continue
        except ArmConfigError as exc:
            unplannable.append(str(exc))
    assert unplannable, (
        f"'{activity}' now plans; remove it from KNOWN_UNPLANNABLE "
        f"({KNOWN_UNPLANNABLE[activity]})"
    )
    assert any("unknown anchor pose" in problem for problem in unplannable)


@pytest.mark.parametrize("activity", ALL_SHIPPED)
def test_the_activity_validates(activity):
    assert _catalogue().validate_activity(activity) == []


@pytest.mark.parametrize("activity", ALL_SHIPPED)
def test_every_action_routes_to_an_executor(activity):
    cat = _catalogue()
    for node in cat.get_activity_plan(activity).nodes.values():
        route(cat.get_action_detail(node.action_id))


@pytest.mark.parametrize("activity", ALL_SHIPPED)
def test_the_graph_schedules_to_completion(activity):
    """Every node reachable, no deadlock, and the step count is the batch count."""
    cat = _catalogue()
    graph = cat.get_activity_plan(activity)
    steps = operator_steps(graph, cat.actions, ROBOT_UNITS_DEDICATED)
    assert steps
    assert sum(len(batch) for batch in steps) == len(graph.nodes)


@pytest.mark.parametrize("activity", SHIPPED)
def test_every_arm_action_plans_offline(activity):
    """Anchors resolve and recordings load, before anything is dispatched.

    A not-yet-taught recording is allowed through: the catalogue keeps those on
    purpose and a dry run reports them at dispatch.
    """
    cat = _catalogue()
    planner = _planner()
    for node in cat.get_activity_plan(activity).nodes.values():
        action = cat.get_action_detail(node.action_id)
        if action.actiontype_id != ACTIONTYPE_TRAJECTORY:
            continue
        try:
            planner.plan(action)
        except NotTaughtError:
            continue


@pytest.mark.parametrize("activity", ALL_SHIPPED)
def test_the_steps_that_cannot_be_resumed_onto_are_exactly_the_replays(activity):
    """The resume refusal is a property of the batch, not a separate list.

    Running from the start is never refused — no --from-id means no resume to
    check — but `--from-id 1` on an activity whose first step is a replay is,
    and both_arms_lift_pour_return_v1 is such an activity.
    """
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(activity), cat.actions, ROBOT_UNITS_DEDICATED
    )
    allowed = set(resumable_steps(steps, cat.actions))
    replays = {
        index
        for index, batch in enumerate(steps, 1)
        if any(is_replay(cat.get_action_detail(item.action_id)) for item in batch)
    }
    assert allowed == set(range(1, len(steps) + 1)) - replays
