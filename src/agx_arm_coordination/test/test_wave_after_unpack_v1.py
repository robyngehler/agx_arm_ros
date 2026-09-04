"""The wave demo, checked as the coordinator would run it.

Eight steps, arms only: two taught waves separated by a hold, entered and left on
the pose unit_unpack_top_v1 ends on.

What this file guards is the seam between the taught data and the anchors around
it: a replay starts from wherever the preceding anchor move left the arm, and
MoveIt refuses a trajectory whose first point is further than
``allowed_start_tolerance`` from the current state. The hold repeats an anchor's
joint vector, so it is asserted against the anchor rather than trusted.

Test level: **L1**. `test_shipped_activities` covers what must hold for every
activity; this covers what is true of this one.
"""

from pathlib import Path

import pytest
import yaml

from agx_arm_coordination.arm_executor import ArmConfig, ArmTrajectoryPlanner
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import ROBOT_UNITS_DEDICATED, operator_steps
from agx_arm_coordination.performer import KIND_ARM, route


ACTIVITY = "wave_after_unpack_v1"
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

#: The pose the activity enters and leaves on, and the pose the hold sits at.
ENTRY_POSE = "Functional_Init_Both_V03"

#: MoveIt's start tolerance for this stack, raised from the 0.01 rad default in
#: agx_arm_moveit/launch/_moveit_config_builder.py. A replay whose first taught
#: point is further than this from the anchor before it is rejected before it
#: moves, so the pairing is checked here rather than on the arm.
START_TOLERANCE_RAD = 0.05

#: (replay action, the anchor action whose pose it must start from).
REPLAY_AFTER_ANCHOR = [
    ("both_arms_wave_both_v01", "both_arms_to_wave_both_init_v01"),
    ("right_arm_wave_right_v01", "both_arms_to_wave_right_init_v01"),
]

EXPECTED_STEPS = [
    "both_arms_to_functional_init",
    "both_arms_to_wave_both_init_v01",
    "both_arms_wave_both_v01",
    "both_arms_to_functional_init",
    "both_arms_hold_functional_init_2s",
    "both_arms_to_wave_right_init_v01",
    "right_arm_wave_right_v01",
    "both_arms_to_functional_init",
]


def _catalogue() -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, ROBOT_UNITS_DEDICATED)


def _planner() -> ArmTrajectoryPlanner:
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    return ArmTrajectoryPlanner(ArmConfig.from_dict(data["arm_executor"]))


def _anchor(name: str) -> list[float]:
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    return [float(v) for v in data["arm_executor"]["poses"][name]["q"]]


def test_the_demo_is_eight_serial_steps():
    """No sync group and no parallel branch: one action per step, in order."""
    cat = _catalogue()
    steps = operator_steps(
        cat.get_activity_plan(ACTIVITY), cat.actions, ROBOT_UNITS_DEDICATED
    )
    assert [item.action_id for batch in steps for item in batch] == EXPECTED_STEPS
    assert all(len(batch) == 1 for batch in steps)


def test_it_enters_and_leaves_on_the_unpack_pose():
    """The pose unit_unpack_top_v1 ends on, so the two chain without a gap."""
    cat = _catalogue()
    for action_id in (EXPECTED_STEPS[0], EXPECTED_STEPS[-1]):
        assert cat.get_action_detail(action_id).metadata["to_pose"] == ENTRY_POSE


def test_every_step_is_an_arm_action():
    """Neither hand is addressed, so neither has to be in service to run this."""
    cat = _catalogue()
    for node in cat.get_activity_plan(ACTIVITY).nodes.values():
        assert route(cat.get_action_detail(node.action_id)).kind == KIND_ARM


def test_the_hold_stands_still_at_the_entry_pose():
    """The dwell repeats an anchor, so it is checked against that anchor.

    Two points at one pose, two seconds apart: no motion, and the coordinator
    gives a plan of under four points zero velocities, which is what a hold
    commands.
    """
    plan = _planner().plan(
        _catalogue().get_action_detail("both_arms_hold_functional_init_2s")
    )
    assert len(plan.points) == 2
    assert plan.points[0].positions == plan.points[1].positions
    assert plan.points[1].time_from_start_sec > plan.points[0].time_from_start_sec
    assert list(plan.points[0].positions) == pytest.approx(_anchor(ENTRY_POSE))


@pytest.mark.parametrize("replay,anchor_action", REPLAY_AFTER_ANCHOR)
def test_a_replay_starts_where_the_anchor_before_it_ends(replay, anchor_action):
    """Each taught wave begins at the pose the step before it moves to.

    The right-arm replay commands seven joints against a 14-DoF anchor, so it is
    compared against that anchor's right half — the side order of the both_arms
    group (registry: left then right).
    """
    cat = _catalogue()
    planner = _planner()
    start = list(planner.plan(cat.get_action_detail(replay)).points[0].positions)
    target = _anchor(cat.get_action_detail(anchor_action).metadata["to_pose"])
    if len(start) != len(target):
        target = target[len(target) - len(start):]
    assert start == pytest.approx(target, abs=START_TOLERANCE_RAD)


def test_both_recordings_carry_the_side_they_were_taught_on():
    """The sidecars name prefixed joints, so the side is checked, not assumed."""
    cat = _catalogue()
    for replay, _ in REPLAY_AFTER_ANCHOR:
        action = cat.get_action_detail(replay)
        names = action.metadata["recording_joint_names"]
        assert names
        assert all(name.startswith(("left_arm_", "right_arm_")) for name in names)
