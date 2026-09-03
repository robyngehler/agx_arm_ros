"""The block-restack demo, checked as the coordinator would run it.

Four blocks moved one at a time from the left gripper to the right and placed,
then a closing gripper sequence. Structural rather than exhaustive: the four
per-block sections are the same fourteen steps with different anchors, so the
repetition is asserted once instead of written out four times.

Test level: **L1**. `test_shipped_activities` covers what must hold for every
activity; this covers what is true of this one.
"""

from pathlib import Path

import yaml

from agx_arm_coordination.arm_executor import ArmConfig, ArmTrajectoryPlanner
from agx_arm_coordination.graph_loader import ActivityCatalogue
from agx_arm_coordination.graph_model import ROBOT_UNITS_DEDICATED, operator_steps
from agx_arm_coordination.operator_resume import resumable_steps
from agx_arm_coordination.performer import KIND_ARM, KIND_GRIPPER, route


ACTIVITY = "block_restack_v1"
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

TOTAL_STEPS = 63
BLOCKS = 4
STEPS_PER_BLOCK = 14

#: One block's cycle, as action ids with the per-block anchor left out. The grip
#: and place anchors differ per block; everything else repeats verbatim.
BLOCK_CYCLE = [
    {"left_gripper_open", "right_gripper_open"},
    {"both_arms_to_grip_block_lifted"},
    None,                                        # both_arms_to_grip_block_0X
    {"left_gripper_close"},
    {"both_arms_to_grip_block_lifted"},
    {"both_arms_to_pre_block_handover"},
    {"both_arms_to_block_handover"},
    {"right_gripper_close"},
    {"left_gripper_open"},
    {"both_arms_to_backoff_block_handover"},
    {"both_arms_to_post_block_handover"},
    None,                                        # both_arms_to_place_block_0X
    {"right_gripper_open"},
    {"both_arms_to_post_block_handover"},
]


def _catalogue() -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, ROBOT_UNITS_DEDICATED)


def _steps(cat: ActivityCatalogue):
    return operator_steps(
        cat.get_activity_plan(ACTIVITY), cat.actions, ROBOT_UNITS_DEDICATED
    )


def _ids(steps):
    return [{item.action_id for item in batch} for batch in steps]


def test_the_demo_is_sixty_three_operator_steps():
    """One init step, four blocks of fourteen, one idle move, five gripper pairs."""
    steps = _ids(_steps(_catalogue()))
    assert len(steps) == TOTAL_STEPS
    assert 1 + BLOCKS * STEPS_PER_BLOCK + 1 + 5 == TOTAL_STEPS
    assert steps[0] == {"both_arms_to_boxing_idle"}


def test_every_block_runs_the_same_fourteen_steps():
    """A cycle that drifted between blocks is a hand-edit nobody meant."""
    steps = _ids(_steps(_catalogue()))
    for block in range(BLOCKS):
        start = 1 + block * STEPS_PER_BLOCK
        section = steps[start:start + STEPS_PER_BLOCK]
        for index, expected in enumerate(BLOCK_CYCLE):
            if expected is None:
                continue
            assert section[index] == expected, (
                f"block {block + 1}, step {index + 1} of the cycle"
            )


def test_each_block_grips_and_places_its_own_anchor():
    """The two per-block steps the cycle leaves open, in order 1..4."""
    steps = _ids(_steps(_catalogue()))
    for block in range(BLOCKS):
        start = 1 + block * STEPS_PER_BLOCK
        assert steps[start + 2] == {f"both_arms_to_grip_block_0{block + 1}"}
        assert steps[start + 11] == {f"both_arms_to_place_block_0{block + 1}"}


def test_the_closing_sequence_is_five_gripper_pairs():
    """Closed, open, closed, open, closed — each pair one step."""
    steps = _ids(_steps(_catalogue()))
    tail = steps[-5:]
    assert tail == [
        {"left_gripper_close", "right_gripper_close"},
        {"left_gripper_open", "right_gripper_open"},
        {"left_gripper_close", "right_gripper_close"},
        {"left_gripper_open", "right_gripper_open"},
        {"left_gripper_close", "right_gripper_close"},
    ]


def test_the_grippers_run_in_parallel_only_with_each_other():
    """Every multi-node step is a left/right gripper pair.

    They hold different bus tokens, which is what lets the scheduler admit them
    together. An arm move never shares a step with anything.
    """
    cat = _catalogue()
    for batch in _steps(cat):
        if len(batch) == 1:
            continue
        kinds = {route(cat.get_action_detail(item.action_id)).kind for item in batch}
        sides = {route(cat.get_action_detail(item.action_id)).side for item in batch}
        assert kinds == {KIND_GRIPPER}
        assert sides == {"left", "right"}


def test_only_grippers_and_planned_arm_moves_appear():
    """No hand action and no taught replay, which is why every step resumes."""
    cat = _catalogue()
    steps = _steps(cat)
    for batch in steps:
        for item in batch:
            assert route(cat.get_action_detail(item.action_id)).kind in (
                KIND_ARM, KIND_GRIPPER
            )
    assert resumable_steps(steps, cat.actions) == list(range(1, TOTAL_STEPS + 1))


def test_every_anchor_resolves():
    """Fourteen distinct anchors: thirteen captured for this demo, plus the idle
    pose it starts and ends on, which already existed."""
    cat = _catalogue()
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    planner = ArmTrajectoryPlanner(ArmConfig.from_dict(data["arm_executor"]))
    planned = set()
    for batch in _steps(cat):
        for item in batch:
            action = cat.get_action_detail(item.action_id)
            if route(action).kind != KIND_ARM:
                continue
            planner.plan(action)
            planned.add(action.metadata["to_pose"])
    assert len(planned) == 14
