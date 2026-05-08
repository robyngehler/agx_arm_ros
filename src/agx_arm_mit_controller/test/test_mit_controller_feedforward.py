from agx_arm_mit_controller.mit_controller_node import scale_gravity_feedforward


def test_scale_gravity_feedforward_applies_command_sign_and_scale():
    assert scale_gravity_feedforward([1.5, -2.0, 0.25], 0.5, -1.0) == [-0.75, 1.0, -0.125]


def test_scale_gravity_feedforward_preserves_sign_when_requested():
    assert scale_gravity_feedforward([1.5, -2.0, 0.25], 0.5, 1.0) == [0.75, -1.0, 0.125]