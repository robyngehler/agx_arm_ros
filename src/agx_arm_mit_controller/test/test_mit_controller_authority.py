"""The MIT controller stands down on the device's published authority.

Until now the only thing this controller could act on was
`feedback/hand_window_active`, which says the shared bus is busy and nothing
about faults, emergency stops, recovery, or the device changing hands. Each of
those must stop it streaming, and only one of them was visible.

The tests build a real node (it needs rclpy) and drive the callback directly.
"""

import rclpy

from agx_arm_msgs.msg import AgxDeviceAuthority

from agx_arm_mit_controller.mit_controller_node import (
    ExecutionState,
    NeroMitControllerNode,
)


def _authority(
    *,
    state=AgxDeviceAuthority.STATE_READY,
    accepts_motion=True,
    device_epoch=1,
    unit_safety_epoch=0,
    reason="test",
):
    msg = AgxDeviceAuthority()
    msg.device_id = "arm_right"
    msg.state = state
    msg.accepts_motion = accepts_motion
    msg.device_epoch = device_epoch
    msg.unit_safety_epoch = unit_safety_epoch
    msg.reason = reason
    return msg


def _node():
    rclpy.init()
    node = NeroMitControllerNode()
    node._set_enabled(True)
    return node


def _close(node):
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_without_any_authority_the_legacy_gates_still_decide():
    """A driver that publishes no authority must not freeze the arm."""
    node = _node()
    try:
        assert node.device_authority is None
        assert node._authority_blocks_motion() is False
    finally:
        _close(node)


def test_losing_motion_stands_the_controller_down():
    node = _node()
    try:
        node._authority_callback(_authority())
        node.active_trajectory = object()
        node.hold_reference = object()

        node._authority_callback(
            _authority(
                state=AgxDeviceAuthority.STATE_STOPPED,
                accepts_motion=False,
                device_epoch=2,
                unit_safety_epoch=1,
                reason="unit stop: emergency stop requested",
            )
        )

        assert node.active_trajectory is None
        assert node.hold_reference is None
        node._control_loop()
        assert node.execution_state == ExecutionState.NOT_AUTHORISED
    finally:
        _close(node)


def test_an_epoch_bump_aborts_even_while_motion_is_still_accepted():
    """A new epoch means what was in flight was issued against a state that is
    gone — a recovery, a rearm, or another commander taking over.
    """
    node = _node()
    try:
        node._authority_callback(_authority(device_epoch=1))
        node.active_trajectory = object()
        node.hold_reference = object()

        node._authority_callback(_authority(device_epoch=2))

        assert node.active_trajectory is None
        assert node.hold_reference is None
        # Still authorised, so it is not stood down — only the stale work went.
        assert node._authority_blocks_motion() is False
    finally:
        _close(node)


def test_a_unit_safety_epoch_bump_aborts_too():
    node = _node()
    try:
        node._authority_callback(_authority(unit_safety_epoch=0))
        node.active_trajectory = object()

        node._authority_callback(_authority(unit_safety_epoch=1))

        assert node.active_trajectory is None
    finally:
        _close(node)


def test_an_unchanged_authority_does_not_abort_anything():
    """It is latched and republished; a repeat must not drop live work."""
    node = _node()
    try:
        node._authority_callback(_authority())
        trajectory = object()
        node.active_trajectory = trajectory

        node._authority_callback(_authority())

        assert node.active_trajectory is trajectory
    finally:
        _close(node)


def test_the_first_authority_does_not_abort_work_in_flight():
    """Arriving late is not a transition."""
    node = _node()
    try:
        trajectory = object()
        node.active_trajectory = trajectory

        node._authority_callback(_authority())

        assert node.active_trajectory is trajectory
    finally:
        _close(node)


def test_regaining_authority_recaptures_rather_than_resuming_a_stale_target():
    node = _node()
    try:
        node._authority_callback(_authority(device_epoch=1))
        node._authority_callback(
            _authority(
                state=AgxDeviceAuthority.STATE_STANDBY,
                accepts_motion=False,
                device_epoch=2,
            )
        )
        node.hold_reference = object()

        node._authority_callback(_authority(device_epoch=3))

        assert node.hold_reference is None
        assert node._authority_blocks_motion() is False
    finally:
        _close(node)


def test_every_non_ready_state_blocks_motion():
    node = _node()
    try:
        for state in (
            AgxDeviceAuthority.STATE_OFFLINE,
            AgxDeviceAuthority.STATE_STANDBY,
            AgxDeviceAuthority.STATE_RECOVERING,
            AgxDeviceAuthority.STATE_FAULTED,
            AgxDeviceAuthority.STATE_STOPPED,
        ):
            node._authority_callback(
                _authority(state=state, accepts_motion=False, device_epoch=state + 10)
            )
            assert node._authority_blocks_motion() is True
    finally:
        _close(node)
