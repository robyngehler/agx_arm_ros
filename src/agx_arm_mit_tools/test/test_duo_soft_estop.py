from agx_arm_mit_tools.duo_soft_estop import hold_service_name, mit_service_path


def test_mit_service_path_includes_mit_controller_namespace():
    assert mit_service_path("left_arm", "hold_current") == "/left_arm/mit_controller/hold_current"
    assert mit_service_path("duo/left_arm", "cancel_trajectory") == "/duo/left_arm/mit_controller/cancel_trajectory"


def test_hold_service_name_normalizes_nested_namespace():
    assert hold_service_name("left_arm") == "hold_left_arm"
    assert hold_service_name("duo/right_arm") == "hold_duo_right_arm"