"""The pack and unpack activities for both units, checked as the coordinator runs them.

Six one-way activities plus the two round trips they were split out of. Every
step is a MoveIt-planned anchor move, so everything the coordinator does before
the arms move is offline: load the catalogue, validate the graph, plan each
action, ask the scheduler what it would dispatch.

These carry an operator contract the tea demo does not: an operator script names
a step number, so the step count and the order are asserted, and each activity is
one node per step with no sync group to collapse.

Test level: **L1**. It proves the graphs, the anchors and the scheduler agree. It
proves nothing about whether the motion is safe on hardware.
"""

from pathlib import Path

import pytest
import yaml

from agx_arm_coordination.arm_executor import (
    ArmConfig,
    ArmTrajectoryPlanner,
    MoveGroupPlan,
)
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import (
    ROBOT_UNITS_DEDICATED,
    ROBOT_UNITS_SHARED,
    operator_steps,
)
from agx_arm_coordination.performer import KIND_ARM, route


CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

#: The one-way flows an operator script runs, and the steps each one takes. The
#: first step of every flow moves TO its start pose rather than assuming the arms
#: are in it, which is what makes the activity runnable after an unclean stop and
#: gives a resume a stable step 1.
FLOWS = {
    "unit_unpack_bottom_fast_v1": [
        "both_arms_to_packing_side_v02",
        "both_arms_to_transit_packing_side_v02",
        "both_arms_to_tee_cup_hold_init_v01",
    ],
    "unit_pack_bottom_fast_v1": [
        "both_arms_to_tee_cup_hold_init_v01",
        "both_arms_to_transit_packing_side_v02",
        "both_arms_to_packing_side_v02",
    ],
    "unit_unpack_bottom_slow_v1": [
        "both_arms_to_packing_side_v02",
        "both_arms_to_transit_packing_side_v02",
        "both_arms_to_transit_functional_init_v02",
        "both_arms_to_functional_init_hold_v02",
        "both_arms_to_tee_cup_hold_init_v01",
    ],
    "unit_pack_bottom_slow_v1": [
        "both_arms_to_tee_cup_hold_init_v01",
        "both_arms_to_functional_init_hold_v02",
        "both_arms_to_transit_functional_init_v02",
        "both_arms_to_transit_packing_side_v02",
        "both_arms_to_packing_side_v02",
    ],
    "unit_unpack_top_v1": [
        "both_arms_to_packing_pose",
        "both_arms_to_functional_init",
    ],
    "unit_pack_top_v1": [
        "both_arms_to_functional_init",
        "both_arms_to_packing_pose",
    ],
}

#: Each unpack flow and the pack flow that undoes it.
REVERSE_PAIRS = [
    ("unit_unpack_bottom_fast_v1", "unit_pack_bottom_fast_v1"),
    ("unit_unpack_bottom_slow_v1", "unit_pack_bottom_slow_v1"),
    ("unit_unpack_top_v1", "unit_pack_top_v1"),
]

#: The round trips the one-way flows were split out of. Still shipped and still
#: referenced, so they are checked alongside rather than left to rot.
ROUND_TRIPS = {"unit_unpack_fast_v1": 5, "unit_unpack_slow_v1": 9}


def _catalogue(units=ROBOT_UNITS_DEDICATED) -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, units)


def _planner() -> ArmTrajectoryPlanner:
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    return ArmTrajectoryPlanner(ArmConfig.from_dict(data["arm_executor"]))


def _steps(cat: ActivityCatalogue, activity: str, units=ROBOT_UNITS_DEDICATED):
    return operator_steps(cat.get_activity_plan(activity), cat.actions, units)


ALL_ACTIVITIES = sorted(FLOWS) + sorted(ROUND_TRIPS)


@pytest.mark.parametrize("activity", ALL_ACTIVITIES)
def test_the_activity_validates(activity):
    assert _catalogue().validate_activity(activity) == []


@pytest.mark.parametrize("activity", ALL_ACTIVITIES)
def test_the_activity_validates_on_shared_buses_too(activity):
    """Arms only, so unlike the tea demo these do not need the dedicated topology.

    Nothing here overlaps, so there is no sync group a shared side bus could
    fail to satisfy. A pack flow must stay runnable on a degraded stack.
    """
    assert _catalogue(ROBOT_UNITS_SHARED).validate_activity(activity) == []


@pytest.mark.parametrize("activity,expected", sorted(FLOWS.items()))
def test_the_operator_steps_are_the_declared_sequence(activity, expected):
    cat = _catalogue()
    steps = [{item.action_id for item in batch} for batch in _steps(cat, activity)]
    assert steps == [{action_id} for action_id in expected]


@pytest.mark.parametrize("activity,expected", sorted(FLOWS.items()))
def test_every_step_is_a_single_node(activity, expected):
    """No sync group here, so a step number and a node are one to one.

    Worth asserting rather than assuming: it is what lets an operator read a
    failure reported by action_id and know which step to resume from.
    """
    cat = _catalogue()
    batches = _steps(cat, activity)
    assert [len(batch) for batch in batches] == [1] * len(expected)


@pytest.mark.parametrize("unpack,pack", REVERSE_PAIRS)
def test_the_pack_flow_walks_the_unpack_flow_backwards(unpack, pack):
    """The pair is one path travelled in two directions, so it is checked as one.

    An anchor added to the unpack side and forgotten on the pack side would
    otherwise leave the unit folding along a path it never unfolded along.
    """
    assert FLOWS[pack] == list(reversed(FLOWS[unpack]))


@pytest.mark.parametrize("activity", ALL_ACTIVITIES)
def test_every_step_is_a_planned_anchor_move_not_a_replay(activity):
    """Which is what makes every step of these flows resumable.

    A recorded replay commands taught joint angles from wherever the arm stands;
    a planned anchor move plans from the current state and is collision-checked.
    These flows contain only the second kind, so `--from-id` may name any step.
    """
    cat = _catalogue()
    planner = _planner()
    for batch in _steps(cat, activity):
        for item in batch:
            action = cat.get_action_detail(item.action_id)
            assert route(action).kind == KIND_ARM
            assert "recording" not in action.metadata
            assert isinstance(planner.plan(action), MoveGroupPlan)


@pytest.mark.parametrize("activity,steps", sorted(ROUND_TRIPS.items()))
def test_the_round_trips_still_run(activity, steps):
    assert len(_steps(_catalogue(), activity)) == steps


def test_the_two_units_fold_into_different_packing_poses():
    """The bottom and top units share the word "packing" and nothing else.

    `Packing_Pose_Side_V02` and `Packing_Pose_Side_Both` are separate anchors in
    separate catalogue fragments; sending one unit to the other's pose is the
    mistake the action names exist to prevent.
    """
    cat = _catalogue()
    bottom = cat.get_action_detail("both_arms_to_packing_side_v02")
    top = cat.get_action_detail("both_arms_to_packing_pose")
    assert bottom.metadata["to_pose"] != top.metadata["to_pose"]
