from __future__ import annotations

import yaml

from agx_arm_ctrl.execution_profiles import resolve_execution_profile


def test_resolve_right_hand_profile_sets_duo_model_and_omnihand():
    resolved = resolve_execution_profile(
        "right_hand",
        duo_model_path="/tmp/duo_system.urdf.xacro",
    )

    assert resolved["custom_model"] == "/tmp/duo_system.urdf.xacro"
    assert resolved["moveit_profile"] == "right_arm"
    assert resolved["effector_type"] == "omnihand"
    assert resolved["omnihand_type"] == "right"
    assert resolved["launch_omnihand_bridge"] == "true"
    assert resolved["input_joint_prefix"] == "right_arm_"
    assert resolved["feedback_joint_prefix"] == "right_arm_"
    assert resolved["tcp_parent_frame"] == "right_arm_nero_tool0"
    assert (
        resolved["custom_model_xacro_args"]
        == "use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true"
    )


def test_resolve_single_side_profiles_inherit_registry_can_port():
    # A left profile must resolve its own side bus; before this fill the
    # launch-default right bus silently connected the WRONG arm.
    for profile_name, expected_port in (
        ("left_arm", "can_nero_left"),
        ("left_hand", "can_nero_left"),
        ("right_arm", "can_nero_right"),
        ("right_hand", "can_nero_right"),
    ):
        resolved = resolve_execution_profile(
            profile_name, duo_model_path="/tmp/duo_system.urdf.xacro"
        )
        assert resolved["can_port"] == expected_port, profile_name


def test_resolve_duo_hand_profile_carries_per_instance_hands():
    resolved = resolve_execution_profile(
        "duo_hand",
        duo_model_path="/tmp/duo_system.urdf.xacro",
    )

    assert resolved["moveit_profile"] == "both_arms"
    assert resolved["effector_type"] == "omnihand"
    assert resolved["launch_omnihand_bridge"] == "true"
    assert "use_left_hand:=true" in resolved["custom_model_xacro_args"]
    assert "use_right_hand:=true" in resolved["custom_model_xacro_args"]
    # multi-side profile: no single top-level can_port; each instance has its own
    assert "can_port" not in resolved

    arm_instances = yaml.safe_load(resolved["arm_instances"])
    assert [
        (i["omnihand_type"], i["effector_type"], i["can_port"], i["joint_prefix"])
        for i in arm_instances
    ] == [
        ("left", "omnihand", "can_nero_left", "left_arm_"),
        ("right", "omnihand", "can_nero_right", "right_arm_"),
    ]


def test_resolve_duo_arm_profile_brings_up_per_arm_drivers():
    # The moveit_mit duo_arm slice owns the per-arm agx_arm_ctrl driver bring-up
    # (launch_driver:true + can_port); without it nothing publishes
    # /<side>_arm/feedback/joint_states and move_group never sees the arms.
    resolved = resolve_execution_profile(
        "duo_arm",
        duo_model_path="/tmp/duo_system.urdf.xacro",
    )

    assert resolved["custom_model"] == "/tmp/duo_system.urdf.xacro"
    assert resolved["moveit_profile"] == "both_arms"
    assert resolved["effector_type"] == "none"

    arm_instances = yaml.safe_load(resolved["arm_instances"])
    assert arm_instances == [
        {
            "name": "left_arm",
            "namespace": "left_arm",
            "joint_prefix": "left_arm_",
            "feedback_joint_prefix": "left_arm_",
            "can_port": "can_nero_left",
            "launch_driver": True,
        },
        {
            "name": "right_arm",
            "namespace": "right_arm",
            "joint_prefix": "right_arm_",
            "feedback_joint_prefix": "right_arm_",
            "can_port": "can_nero_right",
            "launch_driver": True,
        },
    ]


def test_resolve_multi_arm_profile_can_be_rejected_for_single_arm_surfaces():
    try:
        resolve_execution_profile(
            "duo_arm",
            duo_model_path="/tmp/duo_system.urdf.xacro",
            allow_multi_arm=False,
        )
    except ValueError as exc:
        assert "multi-arm" in str(exc)
    else:
        raise AssertionError("Expected multi-arm profile rejection")


def test_resolve_unknown_profile_lists_choices():
    try:
        resolve_execution_profile("unknown_profile", duo_model_path="/tmp/duo_system.urdf.xacro")
    except ValueError as exc:
        assert "Available profiles" in str(exc)
        assert "right_hand" in str(exc)
    else:
        raise AssertionError("Expected unknown profile rejection")
