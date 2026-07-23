from agx_arm_mit_tools.duo_soft_estop import (
    arm_service_path,
    hold_service_name,
    mit_service_path,
    recover_service_name,
)


def test_mit_service_path_includes_mit_controller_namespace():
    assert mit_service_path("left_arm", "hold_current") == "/left_arm/mit_controller/hold_current"
    assert mit_service_path("duo/left_arm", "cancel_trajectory") == "/duo/left_arm/mit_controller/cancel_trajectory"


def test_hold_service_name_normalizes_nested_namespace():
    assert hold_service_name("left_arm") == "hold_left_arm"
    assert hold_service_name("duo/right_arm") == "hold_duo_right_arm"


def test_arm_service_path_is_namespace_root_sibling_of_mit_controller():
    # The arm driver services live at the arm-namespace root, not under
    # mit_controller — this is what the duo e-stop escalation and the recovery
    # helper call.
    assert arm_service_path("right_arm", "emergency_stop") == "/right_arm/emergency_stop"
    assert arm_service_path("right_arm", "control/omnihand/stop") == "/right_arm/control/omnihand/stop"
    assert arm_service_path("", "emergency_stop") == "/emergency_stop"


def test_recover_service_name_normalizes_nested_namespace():
    assert recover_service_name("left_arm") == "recover_left_arm"
    assert recover_service_name("duo/right_arm") == "recover_duo_right_arm"
