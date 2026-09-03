"""The shipped tea-pour v2 demo, checked the way the coordinator would run it.

Every step the coordinator takes before the first motion is offline and cheap:
load the catalogue, validate the graph, plan every arm action, ask the scheduler
what it would dispatch. Running them here is what turns a hand-written activity
into something known to be runnable — an anchor naming a pose that no longer
exists, a recording taught on the other arm, or a sync group that cannot overlap
all fail in this file rather than mid-sequence with the can in the air.

Test level: **L1**. It proves the graph, the catalogue and the retiming agree.
It proves nothing about whether the taught motion is safe on hardware.
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
    ACTIONTYPE_TRAJECTORY,
    ROBOT_UNITS_DEDICATED,
    ROBOT_UNITS_SHARED,
    operator_steps,
)
from agx_arm_coordination.performer import route


ACTIVITY = "tea_pour_duo_v2"
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
CTRL_CONFIG_DIR = (
    Path(__file__).resolve().parents[3] / "agx_arm_ctrl" / "config"
)

#: Steps the flow requires to overlap an arm move with a hand shape, keyed by
#: sync_flag. 1 was the right hand's `can_prep` beside the staging move and is
#: gone; 4 is held for the right hand beside `both_arms_can_goto_pour_init` and
#: is currently commented out in the activity, so node 100 carries it alone.
SYNC_GROUPS = {
    2: {"both_arms_to_can_grip_idle", "left_hand_can_pre_grip"},
    3: {"left_hand_can_grip", "both_arms_to_can_adjust_while_grip"},
    5: {"both_arms_to_functional_init", "left_hand_fist"},
}

#: Hand actions the activity dispatches. The right hand is out of service, so it
#: is commanded nowhere — not even to the `zero` it is assumed to already hold.
HAND_ACTIONS = {
    "left_hand_can_pre_grip",
    "left_hand_can_grip",
    "left_hand_can_release",
    "left_hand_fist",
}

#: Defined but deliberately unreferenced: the retired right hand, and the closing
#: heart the activity no longer ends with.
RETIRED_ACTIONS = {
    "right_hand_can_prep",
    "right_hand_zero",
    "right_hand_heart",
    "left_hand_zero",
    "left_hand_heart",
    "both_arms_to_heart_top",
}


def _catalogue(units=ROBOT_UNITS_DEDICATED) -> ActivityCatalogue:
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, units)


def _planner() -> ArmTrajectoryPlanner:
    data = yaml.safe_load((CONFIG_DIR / "arm_config.yaml").read_text(encoding="utf-8"))
    return ArmTrajectoryPlanner(ArmConfig.from_dict(data["arm_executor"]))


def _dispatch_order(cat: ActivityCatalogue, units) -> list[list]:
    """What the scheduler would dispatch, batch by batch, with nothing running."""
    return operator_steps(cat.get_activity_plan(ACTIVITY), cat.actions, units)


def test_the_demo_validates_on_dedicated_buses():
    assert _catalogue().validate_activity(ACTIVITY) == []


def test_the_demo_is_refused_on_shared_buses():
    """The overlap is the activity, not an optimisation of it.

    Three of its steps move an arm and shape a hand at once. On a shared side bus
    those cannot overlap, and a run that quietly serialized them would be a
    different activity wearing this one's name — so validation refuses it and
    names every group that cannot be satisfied.
    """
    problems = _catalogue(ROBOT_UNITS_SHARED).validate_activity(ACTIVITY)
    assert len(problems) == len(SYNC_GROUPS)
    assert all("cannot run in parallel" in problem for problem in problems)


def test_every_arm_action_plans_before_anything_moves():
    """Anchors resolve, recordings load, and every side matches its group."""
    cat = _catalogue()
    planner = _planner()
    planned = 0
    for node in cat.get_activity_plan(ACTIVITY).nodes.values():
        action = cat.get_action_detail(node.action_id)
        if action.actiontype_id != ACTIONTYPE_TRAJECTORY:
            continue
        planner.plan(action)
        planned += 1
    nodes = len(cat.get_activity_plan(ACTIVITY).nodes)
    assert planned == nodes - len(HAND_ACTIONS)


def test_the_right_hand_is_never_commanded():
    """It is out of service and assumed to sit at `zero`.

    Not even a `zero` command: the bridge is fail-closed, so a hand that cannot
    execute would fail the claim or time out and take the activity with it. The
    right ARM still moves — only the hand is out.
    """
    cat = _catalogue()
    commanded = {
        cat.get_action_detail(node.action_id).robot_id
        for node in cat.get_activity_plan(ACTIVITY).nodes.values()
    }
    assert "right_hand" not in commanded
    assert {"left_hand", "right_arm", "both_arms"} <= commanded


def test_the_retired_actions_are_defined_but_unreferenced():
    """Restoring one is an edge change in the activity, not a re-authoring."""
    cat = _catalogue()
    referenced = {
        node.action_id for node in cat.get_activity_plan(ACTIVITY).nodes.values()
    }
    for action_id in RETIRED_ACTIONS:
        cat.get_action_detail(action_id)          # still in the catalogue
        assert action_id not in referenced        # and not in this activity


def test_every_action_routes_to_an_executor():
    cat = _catalogue()
    for node in cat.get_activity_plan(ACTIVITY).nodes.values():
        route(cat.get_action_detail(node.action_id))


def test_the_overlapping_steps_are_dispatched_together():
    cat = _catalogue()
    grouped = {
        batch[0].sync_flag: {item.action_id for item in batch}
        for batch in _dispatch_order(cat, ROBOT_UNITS_DEDICATED)
        if len(batch) > 1
    }
    assert grouped == SYNC_GROUPS


#: What one operator step is, in order: a dispatch batch, not a graph node. A
#: sync group is one step even though it is two nodes, which is why `--from-id N`
#: counts batches — resuming into half of a synchronized pair is not a state the
#: activity has. Pinned here because the operator step numbers are a contract:
#: an edit that inserts a node renumbers every step after it.
OPERATOR_STEPS = [
    {"both_arms_to_functional_init"},
    {"both_arms_to_can_prep_grip"},
    {"right_arm_can_prep_4grip"},
    {"both_arms_to_can_grip_idle", "left_hand_can_pre_grip"},
    {"both_arms_to_can_pre_grip"},
    {"left_arm_can_grip_move"},
    {"both_arms_to_can_pre_grip_adjust"},
    {"both_arms_to_can_adjust_while_grip", "left_hand_can_grip"},
    {"left_arm_can_lift_post_grip"},
    {"both_arms_can_goto_pour_init"},
    {"both_arms_to_pour_init"},
    {"both_arms_can_pour"},
    {"both_arms_to_pour_finish"},
    {"both_arms_to_pre_place"},
    {"both_arms_to_can_post_grip"},
    {"both_arms_to_can_place"},
    {"left_hand_can_release"},
    {"left_arm_can_release_motion"},
    {"left_arm_can_post_place_adjust"},
    {"both_arms_to_functional_init", "left_hand_fist"},
    {"both_arms_to_packing_pose"},
]

#: Steps whose batch is a recorded replay. A replay starts from wherever the arm
#: stands, so resuming onto one commands taught joint angles from an unknown
#: pose; the operator resumes at the nearest earlier planned step instead.
RECORDED_STEPS = {3, 6, 9, 10, 12, 18, 19}


def test_the_operator_steps_are_the_dispatch_batches_in_order():
    cat = _catalogue()
    steps = [
        {item.action_id for item in batch}
        for batch in _dispatch_order(cat, ROBOT_UNITS_DEDICATED)
    ]
    assert steps == OPERATOR_STEPS
    assert len(steps) == 21


def test_the_recorded_steps_are_the_ones_a_resume_may_not_start_on():
    """Which step numbers carry a replay, so the resume rule has something to
    check against rather than re-deriving it per caller."""
    cat = _catalogue()
    recorded = {
        index
        for index, batch in enumerate(_dispatch_order(cat, ROBOT_UNITS_DEDICATED), 1)
        if any(
            "recording" in cat.get_action_detail(item.action_id).metadata
            or "waypoints" in cat.get_action_detail(item.action_id).metadata
            for item in batch
        )
    }
    assert recorded == RECORDED_STEPS


def test_the_whole_graph_is_reachable():
    cat = _catalogue()
    batches = _dispatch_order(cat, ROBOT_UNITS_DEDICATED)
    dispatched = sum(len(batch) for batch in batches)
    assert dispatched == len(cat.get_activity_plan(ACTIVITY).nodes)


def test_the_grip_attaches_the_payload_and_the_release_detaches_it():
    """The lift must not run under the unloaded gravity model, and the return
    trip must not run under the loaded one."""
    cat = _catalogue()
    assert cat.get_action_detail("left_hand_can_grip").payload_update == "attach"
    assert cat.get_action_detail("left_hand_can_release").payload_update == "detach"
    # Both name one arm side, which is what the transition is applied to.
    assert cat.get_action_detail("left_hand_can_grip").robot_id == "left_hand"


def test_the_lift_is_admitted_only_after_the_grip_completed():
    """payload_update is applied on the grip's success, before the node counts
    as completed, so an ordering that could start the lift first would defeat it."""
    cat = _catalogue()
    order = [
        {item.action_id for item in batch}
        for batch in _dispatch_order(cat, ROBOT_UNITS_DEDICATED)
    ]
    grip = next(i for i, ids in enumerate(order) if "left_hand_can_grip" in ids)
    lift = next(i for i, ids in enumerate(order) if "left_arm_can_lift_post_grip" in ids)
    assert grip < lift


def test_recorded_actions_reference_a_sidecar_and_keep_the_taught_density():
    """Inlining would force a decimation the retiming could not undo."""
    cat = _catalogue()
    recorded = [
        action for action in cat.actions.values()
        if str(action.metadata.get("recording", "")).startswith("recordings/")
    ]
    assert len(recorded) == 7
    for action in recorded:
        # resolve_recordings materialised the waypoints at load.
        assert len(action.metadata["waypoints"]) >= 200
        # The sidecar states the side it was taught on, so the planner checks it
        # instead of taking the catalogue's word for it.
        assert action.metadata["recording_joint_names"][0].startswith(
            ("left_arm_", "right_arm_")
        )


def test_no_recorded_action_carries_both_playback_and_a_legacy_scale():
    """`playback` is the single timing authority; the pair is refused, never
    multiplied. Catching it here keeps the refusal out of the demo."""
    cat = _catalogue()
    planner = _planner()
    for node in cat.get_activity_plan(ACTIVITY).nodes.values():
        action = cat.get_action_detail(node.action_id)
        if "playback" not in action.metadata:
            continue
        assert "velocity_scaling" not in action.metadata
        assert "acceleration_scaling" not in action.metadata
        spec, explicit = planner.playback_for(action)
        # The demo's declared timing: every recording replays in taught time at
        # tempo 1.0. Pinned because it is a choice about how the demo runs, so
        # changing it shows up here rather than only on hardware.
        assert explicit and spec.mode == "tempo_scale"
        assert spec.smoothing_window_sec == 0.5
        assert spec.speed_scale == 1.0


def test_anchor_moves_are_planned_not_replayed():
    """A `to_pose` action must reach move_group, which is what collision-checks
    it; a replay is executed without planning."""
    cat = _catalogue()
    planner = _planner()
    for action_id in SYNC_GROUPS[2] | SYNC_GROUPS[5]:
        action = cat.get_action_detail(action_id)
        if action.actiontype_id != ACTIONTYPE_TRAJECTORY:
            continue
        assert isinstance(planner.plan(action), MoveGroupPlan)


@pytest.mark.skipif(
    not (CTRL_CONFIG_DIR / "omnihand_skills.yaml").is_file(),
    reason="agx_arm_ctrl config not reachable from this tree",
)
def test_every_hand_action_names_a_skill_that_maps_to_a_taught_gesture():
    """The three-hop mapping the hand path actually walks: the action names a
    skill, the skill names a preset, the preset is a measured pose."""
    cat = _catalogue()
    skills = yaml.safe_load(
        (CTRL_CONFIG_DIR / "omnihand_skills.yaml").read_text(encoding="utf-8")
    )["omnihand_skills"]
    gestures = yaml.safe_load(
        (CTRL_CONFIG_DIR / "omnihand_pro_gestures.yaml").read_text(encoding="utf-8")
    )["omnihand_gestures"]

    hand_nodes = [
        cat.get_action_detail(node.action_id)
        for node in cat.get_activity_plan(ACTIVITY).nodes.values()
        if cat.get_action_detail(node.action_id).actiontype_id != ACTIONTYPE_TRAJECTORY
    ]
    assert {action.action_id for action in hand_nodes} == HAND_ACTIONS
    for action in hand_nodes:
        skill_name = action.metadata["skill_name"]
        assert skill_name in skills, f"{action.action_id}: unknown skill '{skill_name}'"
        preset = skills[skill_name].get("target_preset")
        assert preset in gestures, f"{skill_name}: unknown preset '{preset}'"
