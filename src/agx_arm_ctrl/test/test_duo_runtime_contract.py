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


def test_validate_duo_both_arms_contract_accepts_omnihand_effectors():
    # duo_hand profile shape: global omnihand + per-instance omnihand sides.
    validate_duo_both_arms_contract(
        "both_arms",
        "omnihand",
        [
            {
                "namespace": "left_arm",
                "joint_prefix": "left_arm_",
                "effector_type": "omnihand",
                "omnihand_type": "left",
            },
            {
                "namespace": "right_arm",
                "joint_prefix": "right_arm_",
                "effector_type": "omnihand",
                "omnihand_type": "right",
            },
        ],
        follow="true",
        use_mit_controller=True,
    )


def test_validate_duo_both_arms_contract_rejects_unsupported_global_effector():
    with pytest.raises(ValueError, match="hand-aware dual-arm variants"):
        validate_duo_both_arms_contract(
            "both_arms",
            "agx_gripper",
            [
                {"namespace": "left_arm", "joint_prefix": "left_arm_"},
                {"namespace": "right_arm", "joint_prefix": "right_arm_"},
            ],
            follow="true",
            use_mit_controller=True,
        )


def test_validate_duo_both_arms_contract_rejects_unsupported_instance_effector():
    with pytest.raises(ValueError, match="only supports"):
        validate_duo_both_arms_contract(
            "both_arms",
            "none",
            [
                {"namespace": "left_arm", "joint_prefix": "left_arm_", "effector_type": "revo2"},
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
