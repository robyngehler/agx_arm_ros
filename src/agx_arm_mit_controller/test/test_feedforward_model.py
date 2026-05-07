from agx_arm_mit_controller.feedforward_model import CalibrationModel


def test_calibration_model_applies_scale_and_bias():
    model = CalibrationModel(
        joint_names=["joint1", "joint2"],
        scale=[2.0, 0.5],
        bias=[1.0, -1.0],
    )

    assert model.apply([3.0, 8.0]) == [7.0, 3.0]