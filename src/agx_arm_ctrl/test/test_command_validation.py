"""L1 tests for what a MIT command must satisfy to reach the arm.

The driver used to check array lengths and non-emptiness and then forward
whatever was in the arrays. Each test below is a command that reached the vendor
SDK before this guard existed.
"""

import math

import pytest

from agx_arm_ctrl.command_validation import (
    NERO_DEFAULT_MIT_LIMITS,
    NERO_V111_MIT_LIMITS,
    mit_limits_for_tier,
    positions_outside_joint_limits,
    validate_mit_command,
)


def _command(
    joint_index=(1, 2, 3),
    p_des=(0.0, 0.0, 0.0),
    v_des=(0.0, 0.0, 0.0),
    kp=(10.0, 10.0, 10.0),
    kd=(0.8, 0.8, 0.8),
    torque=(0.0, 0.0, 0.0),
    joint_count=7,
    limits=NERO_DEFAULT_MIT_LIMITS,
):
    return validate_mit_command(
        joint_index, p_des, v_des, kp, kd, torque,
        joint_count=joint_count, limits=limits,
    )


def test_a_well_formed_command_is_admitted():
    assert _command() is None


def test_a_full_seven_joint_command_is_admitted():
    assert _command(
        joint_index=tuple(range(1, 8)),
        p_des=(0.1,) * 7,
        v_des=(0.0,) * 7,
        kp=(30.0,) * 7,
        kd=(1.0,) * 7,
        torque=(0.5,) * 7,
    ) is None


def test_inconsistent_array_lengths_are_refused():
    rejection = _command(joint_index=(1, 2), p_des=(0.0, 0.0, 0.0))
    assert rejection is not None
    assert rejection.reason == "length_mismatch"


def test_an_empty_command_is_refused():
    rejection = _command(
        joint_index=(), p_des=(), v_des=(), kp=(), kd=(), torque=()
    )
    assert rejection is not None
    assert rejection.reason == "empty"


@pytest.mark.parametrize("bad_index", [0, -1, 8, 255])
def test_a_joint_the_arm_does_not_have_is_refused(bad_index):
    rejection = _command(joint_index=(1, 2, bad_index))
    assert rejection is not None
    assert rejection.reason == "unknown_joint"
    assert str(bad_index) in rejection.detail


def test_the_same_joint_commanded_twice_is_refused():
    """Last-one-wins would make the effect depend on array order."""
    rejection = _command(joint_index=(1, 2, 2))
    assert rejection is not None
    assert rejection.reason == "duplicate_joint"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_value_is_refused(bad):
    assert _command(p_des=(0.0, bad, 0.0)).reason == "non_finite"
    assert _command(v_des=(0.0, bad, 0.0)).reason == "non_finite"
    assert _command(kp=(10.0, bad, 10.0)).reason == "non_finite"
    assert _command(kd=(0.8, bad, 0.8)).reason == "non_finite"
    assert _command(torque=(0.0, bad, 0.0)).reason == "non_finite"


def test_a_position_the_protocol_cannot_encode_is_refused():
    assert _command(p_des=(0.0, 13.0, 0.0)).reason == "out_of_range"
    assert _command(p_des=(0.0, -13.0, 0.0)).reason == "out_of_range"
    assert _command(p_des=(0.0, 12.5, 0.0)) is None


def test_a_velocity_the_protocol_cannot_encode_is_refused():
    assert _command(v_des=(0.0, 46.0, 0.0)).reason == "out_of_range"
    assert _command(v_des=(0.0, 45.0, 0.0)) is None


def test_gains_outside_the_protocol_range_are_refused():
    assert _command(kp=(10.0, 501.0, 10.0)).reason == "out_of_range"
    assert _command(kp=(10.0, -0.1, 10.0)).reason == "out_of_range"
    assert _command(kd=(0.8, 5.1, 0.8)).reason == "out_of_range"
    assert _command(kd=(0.8, -5.1, 0.8)).reason == "out_of_range"


def test_the_default_tier_bounds_torque_per_joint():
    """Firmware <= 1.10: joints 1-2 take 24 Nm, 3-4 take 16, 5-7 take 8."""
    assert _command(joint_index=(1,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(23.0,)) is None
    assert _command(joint_index=(3,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(23.0,)).reason == "out_of_range"
    assert _command(joint_index=(3,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(15.0,)) is None
    assert _command(joint_index=(6,), p_des=(0.0,), v_des=(0.0,),
                    kp=(10.0,), kd=(0.8,), torque=(15.0,)).reason == "out_of_range"


def test_the_rejection_detail_names_the_joint_and_the_bound():
    rejection = _command(kp=(10.0, 900.0, 10.0))
    assert "kp" in rejection.detail
    assert "joint 2" in rejection.detail
    assert "500" in rejection.detail


# --- joint limits are flagged, not refused -----------------------------------

JOINT_NAMES = [f"joint{n}" for n in range(1, 8)]
JOINT_LIMITS = {name: [-1.5, 1.5] for name in JOINT_NAMES}


def test_a_position_past_the_configured_limit_is_flagged():
    outside = positions_outside_joint_limits(
        (1, 2, 3), (0.0, 2.0, 0.0), JOINT_NAMES, JOINT_LIMITS
    )
    assert len(outside) == 1
    assert "joint2" in outside[0]


def test_a_position_inside_the_configured_limit_is_not_flagged():
    assert positions_outside_joint_limits(
        (1, 2, 3), (0.0, 1.4, -1.4), JOINT_NAMES, JOINT_LIMITS
    ) == []


def test_flagging_is_silent_when_no_limits_are_configured():
    assert positions_outside_joint_limits(
        (1, 2), (99.0, -99.0), JOINT_NAMES, {}
    ) == []


def test_flagging_never_raises_on_input_the_validator_would_have_refused():
    """It runs after validation, but must not be a second failure mode."""
    assert positions_outside_joint_limits(
        (1, 99), (0.0, 0.0), JOINT_NAMES, JOINT_LIMITS
    ) == []
    assert positions_outside_joint_limits(
        (1,), (math.nan,), JOINT_NAMES, JOINT_LIMITS
    ) == []


# --- the driver actually refuses ---------------------------------------------

from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode  # noqa: E402
from agx_arm_msgs.msg import MoveMITMsg  # noqa: E402


class _FakeLogger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _CountingArm:
    """Records every MIT frame that made it to the vendor SDK."""

    def __init__(self):
        self.sent = []

    def set_motion_mode(self, _mode):
        pass

    def set_auto_set_motion_mode_enabled(self, _enabled):
        pass

    def move_mit(self, **kwargs):
        self.sent.append(kwargs)


def _mit_node(arm):
    node = AgxArmRosNode.__new__(AgxArmRosNode)
    node.agx_arm = arm
    node.arm_joint_count = 7
    node.arm_joint_names = list(JOINT_NAMES)
    node.arm_joint_limits = dict(JOINT_LIMITS)
    node.mit_limits = NERO_DEFAULT_MIT_LIMITS
    node._command_rejections = {}
    node._estop_latched = False
    # These exercise the *payload* contract. Admission — commander,
    # generations, sequence — has its own tests against the authority.
    node.require_command_stamp = False
    node._last_rejection_log_monotonic = {}
    node._rejection_log_period_s = 0.0
    node._current_motion_mode = "mit"
    node.is_mit_mode = True
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._check_can_control = lambda: True
    return node


def _msg(joint_index, p_des, **overrides):
    msg = MoveMITMsg()
    msg.joint_index = list(joint_index)
    msg.p_des = list(p_des)
    count = len(joint_index)
    msg.v_des = list(overrides.get("v_des", [0.0] * count))
    msg.kp = list(overrides.get("kp", [10.0] * count))
    msg.kd = list(overrides.get("kd", [0.8] * count))
    msg.torque = list(overrides.get("torque", [0.0] * count))
    return msg


def test_a_valid_command_reaches_the_sdk():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2, 3), (0.1, 0.2, 0.3)))
    assert len(arm.sent) == 3


def test_a_nan_setpoint_never_reaches_the_sdk():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2, 3), (0.1, math.nan, 0.3)))
    assert arm.sent == []
    assert node.logger.errors


def test_the_whole_message_is_refused_not_the_bad_joint():
    """Six good joints and one bad one would leave a pose nobody commanded."""
    arm = _CountingArm()
    node = _mit_node(arm)
    positions = [0.1] * 7
    positions[6] = 99.0
    node._move_mit_callback(_msg(tuple(range(1, 8)), positions))
    assert arm.sent == []


def test_rejections_are_counted_per_reason():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 1), (0.1, 0.2)))
    node._move_mit_callback(_msg((1, 1), (0.1, 0.2)))
    node._move_mit_callback(_msg((1, 2), (math.inf, 0.2)))

    assert node._command_rejections[("move_mit", "duplicate_joint")] == 2
    assert node._command_rejections[("move_mit", "non_finite")] == 1
    assert arm.sent == []


def test_a_position_past_a_joint_limit_is_warned_but_still_sent():
    arm = _CountingArm()
    node = _mit_node(arm)
    node._move_mit_callback(_msg((1, 2), (0.0, 2.0)))

    assert len(arm.sent) == 2, "a joint-limit excursion must not freeze the loop"
    assert node.logger.warns
    assert "joint2" in node.logger.warns[-1]


# --- the firmware tier decides the torque bound ------------------------------
#
# Found on hardware 2026-08-12: the two arms of this unit do not run the same
# firmware (right 1.06 -> default tier, left 1.11 -> v111). The tiers bound
# feed-forward torque differently, so one table for both is wrong in both
# directions.

def _torque(joint, value, limits):
    return validate_mit_command(
        (joint,), (0.0,), (0.0,), (10.0,), (0.8,), (value,),
        joint_count=7, limits=limits,
    )


def test_the_v111_tier_bounds_every_joint_at_the_same_torque():
    for joint in range(1, 8):
        assert _torque(joint, 15.9, NERO_V111_MIT_LIMITS) is None
        assert _torque(joint, 16.1, NERO_V111_MIT_LIMITS).reason == "out_of_range"


def test_the_default_table_would_refuse_valid_v111_commands():
    """The dangerous direction: a legitimate command frozen out mid-stream."""
    assert _torque(6, 12.0, NERO_V111_MIT_LIMITS) is None
    assert _torque(6, 12.0, NERO_DEFAULT_MIT_LIMITS).reason == "out_of_range"


def test_the_v111_table_would_admit_commands_the_default_tier_refuses():
    assert _torque(1, 20.0, NERO_DEFAULT_MIT_LIMITS) is None
    assert _torque(1, 20.0, NERO_V111_MIT_LIMITS).reason == "out_of_range"


def test_the_tier_lookup_covers_every_tier_the_sdk_ships():
    assert mit_limits_for_tier("default") is NERO_DEFAULT_MIT_LIMITS
    assert mit_limits_for_tier("v111") is NERO_V111_MIT_LIMITS
    # 1.12 inherits 1.11's move_mit, so it inherits its bounds.
    assert mit_limits_for_tier("v112") is NERO_V111_MIT_LIMITS


def test_an_unknown_tier_falls_back_to_the_driver_the_sdk_would_build():
    assert mit_limits_for_tier("v999") is NERO_DEFAULT_MIT_LIMITS
    assert mit_limits_for_tier(None) is NERO_DEFAULT_MIT_LIMITS
