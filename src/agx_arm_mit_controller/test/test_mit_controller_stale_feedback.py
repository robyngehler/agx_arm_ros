import time

import rclpy
from sensor_msgs.msg import JointState

from agx_arm_mit_controller.mit_controller_node import (
    ExecutionState,
    NeroMitControllerNode,
)


def _joint_state(positions: list[float]) -> JointState:
    msg = JointState()
    msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    msg.position = positions
    msg.velocity = [0.0] * len(positions)
    return msg


def test_stale_feedback_drops_active_trajectory_and_recaptures_hold():
    """A stale feedback outage must not leave the trajectory clock running.

    Regression guard for the shared-CAN resume snap (plan section 1.4): if the
    active trajectory stays armed while feedback is stale, its monotonic start
    clock keeps advancing through the outage and a feedback comeback samples a
    far-ahead point, snapping the arm under MIT gains. The control loop must
    drop the trajectory on the first stale tick and re-capture the current pose
    as the hold reference when feedback returns.
    """
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        node._feedback_callback(_joint_state([0.0] * 7))
        node._set_enabled(True)

        # Arm a trajectory whose start clock is already well in the past.
        node.active_trajectory = object()
        node.trajectory_start_monotonic = time.monotonic() - 100.0
        node.hold_reference = None

        # Force stale feedback and run one control tick.
        node.last_feedback_monotonic = time.monotonic() - 1000.0
        assert not node._has_fresh_feedback()
        node._control_loop()

        # Snap prevention: the stale trajectory is dropped, not resumed.
        assert node.active_trajectory is None
        assert node.hold_reference is None
        assert node.execution_state == ExecutionState.STALE_FEEDBACK

        # Feedback returns at a NEW pose; the loop must hold the current pose,
        # never a far-ahead sample of the abandoned trajectory clock.
        node._feedback_callback(_joint_state([0.3] * 7))
        assert node._has_fresh_feedback()
        node._control_loop()

        assert node.active_trajectory is None
        assert node.hold_reference is not None
        assert node.hold_reference.positions == (0.3,) * 7
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
