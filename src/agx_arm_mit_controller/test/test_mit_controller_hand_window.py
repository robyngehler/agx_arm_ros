"""The MIT controller stands down while the driver has a hand window open.

During a hand window the driver silences the arm feedback push and holds the
arm in a firmware MOVE-J. This controller must NOT read that expected silence
as a dead bus and stream a dead-man flood into the (gated) driver — measured on
hardware to starve the co-located OmniHand's CANFD access under teach load.
"""

import rclpy
from std_msgs.msg import Bool

from agx_arm_mit_controller.mit_controller_node import (
    ExecutionState,
    NeroMitControllerNode,
)


def _bool(value: bool) -> Bool:
    msg = Bool()
    msg.data = value
    return msg


def test_open_window_stands_the_controller_down_and_drops_the_trajectory():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        # Legacy gate under test: it only applies where no device authority
        # is published, which is the development profile.
        node.require_device_authority = False
        node._set_enabled(True)
        node.active_trajectory = object()
        node.hold_reference = object()

        node._hand_window_callback(_bool(True))

        assert node.hand_window_active is True
        # Active trajectory dropped so a feedback comeback cannot sample a
        # far-ahead point and snap the arm.
        assert node.active_trajectory is None
        assert node.hold_reference is None

        node._control_loop()
        assert node.execution_state == ExecutionState.HAND_WINDOW
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_stand_down_takes_precedence_over_the_stale_feedback_dead_man():
    # No feedback has ever arrived (stale), which would normally trigger the
    # dead-man. With a window open the controller must stand down instead.
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        # Legacy gate under test: it only applies where no device authority
        # is published, which is the development profile.
        node.require_device_authority = False
        node._set_enabled(True)
        node._hand_window_callback(_bool(True))

        node._control_loop()

        assert node.execution_state == ExecutionState.HAND_WINDOW
        assert node._stale_since_monotonic is None  # dead-man never armed
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_closing_the_window_recaptures_the_hold_reference():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        # Legacy gate under test: it only applies where no device authority
        # is published, which is the development profile.
        node.require_device_authority = False
        node._set_enabled(True)
        node._hand_window_callback(_bool(True))
        node.hold_reference = object()  # pretend something set it

        node._hand_window_callback(_bool(False))

        assert node.hand_window_active is False
        # Cleared so the next control loop recaptures the arm's actual pose,
        # never a stale target.
        assert node.hold_reference is None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
