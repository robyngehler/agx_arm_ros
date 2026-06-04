from __future__ import annotations

import pytest

from agx_arm_ctrl.duo_runtime_contract import validate_duo_both_arms_contract


def test_validate_duo_both_arms_contract_accepts_arm_only_mit_flow():
    validate_duo_both_arms_contract(
        "both_arms",
        "none",
        [
            {"namespace": "left_arm", "joint_prefix": "left_arm_"},
            {"namespace": "right_arm", "joint_prefix": "right_arm_"},
        ],
        follow="true",
        use_mit_controller=True,
    )


def test_validate_duo_both_arms_contract_rejects_global_hand_effector():
    with pytest.raises(ValueError, match="hand-aware dual-arm variants"):
        validate_duo_both_arms_contract(
            "both_arms",
            "omnihand",
            [
                {"namespace": "left_arm", "joint_prefix": "left_arm_"},
                {"namespace": "right_arm", "joint_prefix": "right_arm_"},
            ],
            follow="true",
            use_mit_controller=True,
        )


def test_validate_duo_both_arms_contract_rejects_instance_effector_override():
    with pytest.raises(ValueError, match="arm-only"):
        validate_duo_both_arms_contract(
            "both_arms",
            "none",
            [
                {"namespace": "left_arm", "joint_prefix": "left_arm_", "effector_type": "omnihand"},
                {"namespace": "right_arm", "joint_prefix": "right_arm_"},
            ],
            follow="true",
            use_mit_controller=True,
        )


def test_validate_duo_both_arms_contract_requires_follow_for_mit_mode():
    with pytest.raises(ValueError, match="follow:=true"):
        validate_duo_both_arms_contract(
            "both_arms",
            "none",
            [
                {"namespace": "left_arm", "joint_prefix": "left_arm_"},
                {"namespace": "right_arm", "joint_prefix": "right_arm_"},
            ],
            follow="false",
            use_mit_controller=True,
        )