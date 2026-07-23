from agx_arm_coordination.arm_executor import ArmConfig

from agx_arm_mit_demos.teach_manager import _build_transition_targets, _transition_robot_ids


def _config() -> ArmConfig:
    return ArmConfig.from_dict({
        "arm_executor": {
            "groups": {
                "both_arms": {
                    "planning_group": "both_arms",
                    "joint_names": ["l1", "l2", "r1", "r2"],
                },
                "left_arm": {
                    "planning_group": "left_arm",
                    "joint_names": ["l1", "l2"],
                },
                "right_arm": {
                    "planning_group": "right_arm",
                    "joint_names": ["r1", "r2"],
                },
            },
            "poses": {
                "Idle_L": [0.0, 0.1],
                "Idle_R": [0.2, 0.3],
                "Pre_Grip_L": [1.0, 1.1],
                "Pre_Grip_R": [1.2, 1.3],
                "Place_R": [2.0, 2.1],
            },
        }
    })


def _config_explicit() -> ArmConfig:
    # Resource stored explicitly via robot_id (the new form): both_arms is ONE
    # 4-DoF entry, single sides carry robot_id, and names are free of _L/_R.
    return ArmConfig.from_dict({
        "arm_executor": {
            "groups": {
                "both_arms": {
                    "planning_group": "both_arms",
                    "joint_names": ["l1", "l2", "r1", "r2"],
                },
                "left_arm": {"planning_group": "left_arm", "joint_names": ["l1", "l2"]},
                "right_arm": {"planning_group": "right_arm", "joint_names": ["r1", "r2"]},
            },
            "poses": {
                "handoff": {"robot_id": "both_arms", "q": [0.0, 0.1, 0.2, 0.3]},
                "tee_lifted": {"robot_id": "right_arm", "q": [2.0, 2.1]},
                "park": {"robot_id": "left_arm", "q": [3.0, 3.1]},
            },
        }
    })


def test_transition_robot_ids_default_to_right_arm_for_un_namespaced_session():
    assert _transition_robot_ids([""]) == ("right_arm",)


def test_build_transition_targets_uses_explicit_both_arms_entry():
    targets = _build_transition_targets(_config_explicit(), ["left_arm", "right_arm"])
    by_label = {t.label: t for t in targets}
    # One explicit 14-DoF-style both_arms entry, named freely (no _L/_R pairing).
    assert "both_arms:handoff" in by_label
    both = by_label["both_arms:handoff"]
    assert both.pose_names == ("handoff",)
    assert both.target_positions == (0.0, 0.1, 0.2, 0.3)
    # Single sides resolve by stored robot_id, not by name suffix.
    assert by_label["tee_lifted"].robot_id == "right_arm"
    assert by_label["park"].robot_id == "left_arm"


def test_build_transition_targets_single_side_uses_explicit_robot_id():
    # An un-suffixed right_arm pose is picked up for a right-only session.
    targets = _build_transition_targets(_config_explicit(), [""])
    labels = [t.label for t in targets]
    assert labels == ["tee_lifted"]
    assert targets[0].robot_id == "right_arm"


def test_update_pose_writes_explicit_robot_id_and_round_trips(tmp_path):
    from agx_arm_mit_demos.capture_anchor_pose import update_pose_in_config

    cfg_path = tmp_path / "arm_config.yaml"
    cfg_path.write_text(
        "arm_executor:\n  poses:\n    seed_R: [0.0, 0.0]\n", encoding="utf-8"
    )
    update_pose_in_config(cfg_path, "handoff", [0.1, 0.2, 0.3, 0.4], 3, robot_id="both_arms")
    assert "handoff: {robot_id: both_arms, q: [0.100, 0.200, 0.300, 0.400]}" in cfg_path.read_text()

    cfg = ArmConfig.from_file(cfg_path)
    assert cfg.poses["handoff"] == (0.1, 0.2, 0.3, 0.4)
    assert cfg.pose_robot_id("handoff") == "both_arms"

    # Without robot_id it stays a legacy bare list (side from the _R suffix).
    update_pose_in_config(cfg_path, "legacy_R", [5.0, 6.0], 3)
    cfg2 = ArmConfig.from_file(cfg_path)
    assert cfg2.pose_robot_id("legacy_R") == "right_arm"


def test_build_transition_targets_for_single_right_arm_filters_right_targets_only():
    targets = _build_transition_targets(_config(), [""])

    assert [target.label for target in targets] == ["Idle_R", "Place_R", "Pre_Grip_R"]
    assert all(target.robot_id == "right_arm" for target in targets)


def test_build_transition_targets_for_duo_session_includes_both_and_per_arm_targets():
    targets = _build_transition_targets(_config(), ["left_arm", "right_arm"])

    assert [target.label for target in targets] == [
        "both_arms:Idle",
        "both_arms:Pre_Grip",
        "Idle_L",
        "Pre_Grip_L",
        "Idle_R",
        "Place_R",
        "Pre_Grip_R",
    ]
    both_target = targets[0]
    assert both_target.pose_names == ("Idle_L", "Idle_R")
    assert both_target.target_positions == (0.0, 0.1, 0.2, 0.3)

def test_discover_mit_namespaces_finds_complete_stacks_only():
    from agx_arm_mit_demos.teach_manager import _discover_mit_namespaces

    class FakeNode:
        def get_service_names_and_types(self):
            required = [
                "set_normal_mode",
                "mit_controller/enable",
                "mit_controller/freedrive",
                "mit_controller/hold_current",
            ]
            services = []
            # complete namespaced duo stacks (right listed first: sorting must fix it)
            for ns in ("/right_arm", "/left_arm"):
                services.extend((f"{ns}/{name}", ["t"]) for name in required)
            # incomplete stack must NOT count
            services.append(("/broken_arm/set_normal_mode", ["t"]))
            # unrelated service noise
            services.append(("/rosout/get_parameters", ["t"]))
            return services

    assert _discover_mit_namespaces(FakeNode()) == ["left_arm", "right_arm"]


def test_discover_mit_namespaces_reports_unnamespaced_stack_as_empty_string():
    from agx_arm_mit_demos.teach_manager import _discover_mit_namespaces

    class FakeNode:
        def get_service_names_and_types(self):
            return [
                ("/set_normal_mode", ["t"]),
                ("/mit_controller/enable", ["t"]),
                ("/mit_controller/freedrive", ["t"]),
                ("/mit_controller/hold_current", ["t"]),
            ]

    assert _discover_mit_namespaces(FakeNode()) == [""]
