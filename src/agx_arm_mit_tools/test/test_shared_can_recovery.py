"""Pure-helper tests for the shared_can_recovery service node.

These lock the recovery ORDERING contract (shared-CAN step-and-settle plan
section 2.2) without a ROS graph: the hand must be stopped before the arm
emergency stop (which may force a link reset), and normal mode is only
forced/verified after the stop. The node builds its clients and callbacks from
exactly this sequence, so guarding the sequence guards the runtime order.
"""

from agx_arm_mit_tools.shared_can_recovery import (
    feedback_topic,
    recover_service_name,
    recovery_service_sequence,
)


def _steps(namespace):
    return [step for step, _path, _srv in recovery_service_sequence(namespace)]


def test_hand_stop_precedes_emergency_stop_precedes_normal_mode():
    steps = _steps("right_arm")
    assert steps.index("hand_stop") < steps.index("emergency_stop")
    assert steps.index("emergency_stop") < steps.index("set_normal_mode")
    # cancel/hold come first; hand is re-checked last, after the reset.
    assert steps[0] == "cancel_trajectory"
    assert steps[-1] == "hand_recheck"


def test_sequence_paths_target_the_right_namespaced_services():
    by_step = {step: (path, srv) for step, path, srv in recovery_service_sequence("right_arm")}
    assert by_step["cancel_trajectory"] == ("/right_arm/mit_controller/cancel_trajectory", "empty")
    assert by_step["hold_current"] == ("/right_arm/mit_controller/hold_current", "empty")
    assert by_step["hand_stop"] == ("/right_arm/control/omnihand/stop", "trigger")
    assert by_step["emergency_stop"] == ("/right_arm/emergency_stop", "trigger")
    assert by_step["set_normal_mode"] == ("/right_arm/set_normal_mode", "trigger")
    assert by_step["hand_recheck"] == ("/right_arm/control/omnihand/stop", "trigger")


def test_recover_service_name_and_feedback_topic():
    assert recover_service_name("right_arm") == "recover_right_arm"
    assert feedback_topic("right_arm") == "/right_arm/feedback/joint_states"
    assert feedback_topic("") == "/feedback/joint_states"


def test_unprefixed_namespace_stays_at_root():
    by_step = {step: path for step, path, _srv in recovery_service_sequence("")}
    assert by_step["emergency_stop"] == "/emergency_stop"
    assert by_step["cancel_trajectory"] == "/mit_controller/cancel_trajectory"
