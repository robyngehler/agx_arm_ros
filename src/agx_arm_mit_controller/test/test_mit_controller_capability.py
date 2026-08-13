"""The controller fits its envelope to what its own arm can encode.

The two arms permanently run different protocol tiers and cannot be flashed
(constraint C8), so one shared configuration is not necessarily encodable on
both. Unchecked, that difference shows up as the hardware boundary refusing
commands mid-stream — and a refused MIT command leaves the firmware holding its
previous setpoint, so under a dual-arm activity one arm keeps moving while the
other freezes.
"""

import rclpy

from agx_arm_msgs.msg import AgxDeviceCapability

from agx_arm_mit_controller.mit_controller_node import NeroMitControllerNode


def _capability(
    *,
    device_id="arm_right",
    tier="default",
    torque=(24.0, 24.0, 16.0, 16.0, 8.0, 8.0, 8.0),
    velocity=45.0,
):
    msg = AgxDeviceCapability()
    msg.device_id = device_id
    msg.protocol_tier = tier
    msg.firmware_version = "1.06"
    msg.joint_count = 7
    msg.max_torque = list(torque)
    msg.max_position = 12.5
    msg.max_velocity = velocity
    msg.max_kp = 500.0
    msg.max_kd = 5.0
    return msg


def _node():
    rclpy.init()
    node = NeroMitControllerNode()
    node.require_device_authority = False
    return node


def _close(node):
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_a_configuration_that_fits_is_left_alone():
    node = _node()
    try:
        node.torque_limit = [8.0] * 7
        node._capability_callback(_capability())
        assert node.torque_limit == [8.0] * 7
    finally:
        _close(node)


def test_a_configuration_above_the_tier_bound_is_reduced_to_it():
    """The v111 arm bounds every joint at 16 N*m, the default tier does not."""
    node = _node()
    try:
        node.torque_limit = [20.0] * 7
        node._capability_callback(_capability(tier="v111", torque=(16.0,) * 7))
        assert node.torque_limit == [16.0] * 7
    finally:
        _close(node)


def test_the_reduction_is_per_joint_on_the_default_tier():
    node = _node()
    try:
        node.torque_limit = [20.0] * 7
        node._capability_callback(_capability())
        # 24/24/16/16/8/8/8: only the joints whose bound is below 20 move.
        assert node.torque_limit == [20.0, 20.0, 16.0, 16.0, 8.0, 8.0, 8.0]
    finally:
        _close(node)


def test_the_same_configuration_lands_differently_on_the_two_arms():
    """The concrete asymmetry C8 is about, pinned so it cannot be forgotten."""
    shared_config = [20.0] * 7

    right = _node()
    try:
        right.torque_limit = list(shared_config)
        right._capability_callback(_capability(device_id="arm_right"))
        right_limits = list(right.torque_limit)
    finally:
        _close(right)

    left = _node()
    try:
        left.torque_limit = list(shared_config)
        left._capability_callback(
            _capability(device_id="arm_left", tier="v111", torque=(16.0,) * 7)
        )
        left_limits = list(left.torque_limit)
    finally:
        _close(left)

    assert right_limits != left_limits
    assert right_limits[0] == 20.0 and left_limits[0] == 16.0
    assert right_limits[6] == 8.0 and left_limits[6] == 16.0


def test_a_velocity_above_the_device_bound_is_reduced():
    node = _node()
    try:
        node.velocity_limit = [50.0] * 7
        node._capability_callback(_capability(velocity=45.0))
        assert node.velocity_limit == [45.0] * 7
    finally:
        _close(node)


def test_capability_for_another_device_is_ignored():
    node = _node()
    try:
        node.expected_device_id = "arm_right"
        node.torque_limit = [20.0] * 7
        node._capability_callback(_capability(device_id="arm_left", torque=(1.0,) * 7))
        assert node.torque_limit == [20.0] * 7
        assert node.device_capability is None
    finally:
        _close(node)


def test_the_clamp_never_raises_a_configured_limit():
    """Reducing a ceiling is safe; raising one would grant unasked authority."""
    node = _node()
    try:
        node.torque_limit = [2.0] * 7
        node._capability_callback(_capability())
        assert node.torque_limit == [2.0] * 7
    finally:
        _close(node)
