"""Tests for the YAML activity/catalogue loader, including the shipped config."""

from pathlib import Path

import pytest

from agx_arm_coordination.graph_loader import ActivityCatalogue


CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def test_shipped_catalogue_loads_and_has_demo_actions():
    cat = ActivityCatalogue.from_config_dir(CONFIG_DIR)
    actions = cat.actions
    for action_id in (
        "left_hand_grasp_glass", "right_hand_grasp_bottle",
        "both_arms_home_to_pregrasp", "both_arms_pour_profile_v1",
    ):
        assert action_id in actions


@pytest.mark.parametrize("activity_id", [
    "hefeweizen_pour_v1",
    "hands_open_close_release_v1",
    "hands_open_release_v1",
    "both_arms_pregrasp_grasp_retract_v1",
    "both_arms_lift_pour_return_v1",
])
def test_shipped_activities_validate_clean(activity_id):
    cat = ActivityCatalogue.from_config_dir(CONFIG_DIR)
    problems = cat.validate_activity(activity_id)
    assert problems == [], f"{activity_id}: {problems}"


def test_hefeweizen_graph_shape():
    cat = ActivityCatalogue.from_config_dir(CONFIG_DIR)
    graph = cat.get_activity_plan("hefeweizen_pour_v1")
    # 18 nodes per the activity graph doc.
    assert len(graph.nodes) == 18
    # hand open/grasp/release pairs share sync flags.
    assert graph.nodes[20].sync_flag == graph.nodes[21].sync_flag == 1
    assert graph.nodes[40].sync_flag == graph.nodes[41].sync_flag == 2
    assert graph.nodes[120].sync_flag == graph.nodes[121].sync_flag == 3


def test_unknown_activity_reports_problem():
    cat = ActivityCatalogue.from_config_dir(CONFIG_DIR)
    assert cat.validate_activity("does_not_exist")


def test_tmp_roundtrip(tmp_path):
    (tmp_path / "catalogue.yaml").write_text(
        "actions:\n"
        "  h_open: { actiontype_id: Gripper, robot_id: left_hand }\n",
        encoding="utf-8",
    )
    activities = tmp_path / "activities"
    activities.mkdir()
    (activities / "tiny.yaml").write_text(
        "activity: tiny\n"
        "nodes:\n"
        "  - { action_no: 1, action_id: h_open }\n"
        "edges: []\n",
        encoding="utf-8",
    )
    cat = ActivityCatalogue.from_config_dir(tmp_path)
    assert cat.validate_activity("tiny") == []
    assert cat.get_action_detail("h_open").robot_id == "left_hand"
    assert cat.available_activities() == ["tiny"]
