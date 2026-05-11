import rclpy
from sensor_msgs.msg import JointState

from agx_arm_msgs.msg import AgxArmStatus
from agx_arm_mit_controller.mit_controller_node import (
    CTRL_MODE_LINKAGE_TEACHING_INPUT_MODE,
    NeroMitControllerNode,
)


def _joint_state(positions: list[float]) -> JointState:
    msg = JointState()
    msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    msg.position = positions
    msg.velocity = [0.0] * len(positions)
    return msg


def _arm_status(ctrl_mode: int) -> AgxArmStatus:
    msg = AgxArmStatus()
    msg.ctrl_mode = ctrl_mode
    return msg


def test_leader_feedback_retargets_hold_reference():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        node._feedback_callback(_joint_state([0.0] * 7))
        node._set_enabled(True)

        node._arm_status_callback(_arm_status(CTRL_MODE_LINKAGE_TEACHING_INPUT_MODE))
        node._leader_feedback_callback(_joint_state([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))

        assert node.feedback_positions["joint1"] == 0.1
        assert node.hold_reference is not None
        assert node.hold_reference.positions == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_entering_leader_mode_cancels_active_trajectory():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        node.active_trajectory = object()

        node._arm_status_callback(_arm_status(CTRL_MODE_LINKAGE_TEACHING_INPUT_MODE))

        assert node.active_trajectory is None
        assert node.leader_mode_active is True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()