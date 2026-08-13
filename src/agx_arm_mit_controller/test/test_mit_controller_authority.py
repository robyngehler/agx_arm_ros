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
    motion_ready=True,
    device_epoch=1,
    unit_safety_epoch=0,
    reason="test",
    accepts_motion_device=None,
    owner_id=None,
):
    msg = AgxDeviceAuthority()
    msg.device_id = "arm_right"
    # Readiness is not permission: a snapshot naming no owner means this
    # controller may not command, so the default names it as the holder.
    msg.owner_id = _COMMANDER if owner_id is None else owner_id
    msg.state = state
    msg.motion_ready = motion_ready
    msg.device_epoch = device_epoch
    msg.unit_safety_epoch = unit_safety_epoch
    msg.reason = reason
    return msg


_COMMANDER = "mit_controller"


def _node(require_authority=False):
    rclpy.init()
    node = NeroMitControllerNode()
    node.commander_id = _COMMANDER
    # Most tests here drive the authority directly, so the requirement is off
    # unless the test is about the requirement itself.
    node.require_device_authority = require_authority
    node._set_enabled(True)
    return node


def _close(node):
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_a_missing_authority_is_a_block_when_one_is_required():
    """Fail-closed by default.

    A namespace typo, a QoS mismatch and an old driver are indistinguishable
    from here, and only one of them is a configuration anybody chose. Treating
    absence as permission makes a wiring error look supported.
    """
    node = _node(require_authority=True)
    try:
        assert node.device_authority is None
        assert node._authority_blocks_motion() is True
    finally:
        _close(node)


def test_the_development_profile_can_still_run_without_one():
    """Explicitly chosen, and named as such — not the default."""
    node = _node(require_authority=False)
    try:
        assert node.device_authority is None
        assert node._authority_blocks_motion() is False
    finally:
        _close(node)


def test_authority_for_another_device_is_ignored_not_obeyed():
    """Two arms publish two authorities.

    Being gated by the wrong one is worse than being gated by none: it would
    report ready while the device this controller commands is stopped.
    """
    node = _node(require_authority=True)
    try:
        node.expected_device_id = "arm_right"
        node._authority_callback(_authority())          # arm_right, ready
        assert node.device_authority is not None

        foreign = _authority(accepts_motion_device="arm_left")
        foreign.device_id = "arm_left"
        foreign.motion_ready = False
        foreign.state = AgxDeviceAuthority.STATE_STOPPED
        node._authority_callback(foreign)

        assert node.foreign_authority_messages == 1
        assert node.device_authority.device_id == "arm_right"
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
                motion_ready=False,
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
                motion_ready=False,
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
                _authority(state=state, motion_ready=False, device_epoch=state + 10)
            )
            assert node._authority_blocks_motion() is True
    finally:
        _close(node)


# --- the action goal must terminate too --------------------------------------

def test_losing_authority_latches_an_abort_for_the_active_goal():
    """Stopping the stream is not enough: the FJT goal owns its own buffer.

    Without this the goal stayed active until some unrelated condition timed it
    out, and could still report on a run that had lost permission to command.
    """
    node = _node()
    try:
        node._authority_callback(_authority())
        node.active_goal_handle = object()

        node._authority_callback(
            _authority(
                state=AgxDeviceAuthority.STATE_STOPPED,
                motion_ready=False,
                device_epoch=2,
                unit_safety_epoch=1,
                reason="unit stop: emergency stop requested",
            )
        )

        reason = node._take_authority_abort()
        assert reason is not None
        assert "device authority changed" in reason
        assert "emergency stop" in reason
        # Consumed once: the goal terminates on it exactly once.
        assert node._take_authority_abort() is None
    finally:
        _close(node)


def test_no_abort_is_latched_when_no_goal_is_running():
    node = _node()
    try:
        node._authority_callback(_authority())
        node.active_goal_handle = None

        node._authority_callback(_authority(motion_ready=False, device_epoch=2))

        assert node._take_authority_abort() is None
    finally:
        _close(node)


def test_an_epoch_bump_alone_aborts_a_running_goal():
    node = _node()
    try:
        node._authority_callback(_authority(device_epoch=1))
        node.active_goal_handle = object()

        node._authority_callback(_authority(device_epoch=2))

        assert node._take_authority_abort() is not None
    finally:
        _close(node)
