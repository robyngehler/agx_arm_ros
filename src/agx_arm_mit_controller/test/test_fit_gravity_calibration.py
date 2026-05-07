from agx_arm_mit_controller.fit_gravity_calibration import _fit_scale_and_bias


def test_fit_scale_and_bias_matches_linear_relation():
    scale, bias = _fit_scale_and_bias([1.0, 2.0, 3.0], [3.0, 5.0, 7.0])

    assert abs(scale - 2.0) < 1e-9
    assert abs(bias - 1.0) < 1e-9