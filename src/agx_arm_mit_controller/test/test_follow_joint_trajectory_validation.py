from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from sensor_msgs.msg import JointState

from agx_arm_mit_controller.mit_controller_node import NeroMitControllerNode


def _joint_state(positions: list[float]) -> JointState:
    msg = JointState()
    msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    msg.position = positions
    msg.velocity = [0.0] * len(positions)
    return msg


def _trajectory(joint_names: list[str], points: list[tuple[float, list[float]]]) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = joint_names
    for time_from_start_s, positions in points:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.effort = [0.0] * len(positions)
        point.time_from_start.sec = int(time_from_start_s)
        point.time_from_start.nanosec = int((time_from_start_s - int(time_from_start_s)) * 1e9)
        msg.points.append(point)
    return msg


def test_validate_trajectory_goal_rejects_start_state_violation():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        node._feedback_callback(_joint_state([0.0] * 7))

        buffer, error_code, detail = node._validate_trajectory_goal(
            _trajectory(
                ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
                [(1.0, [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            )
        )

        assert buffer is None
        assert error_code != 0
        assert "Start state mismatch" in detail
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_validate_trajectory_goal_accepts_reordered_joints_when_enabled():
    rclpy.init()
    node = NeroMitControllerNode()
    try:
        node.allow_joint_reordering = True
        node._feedback_callback(_joint_state([0.0] * 7))

        buffer, error_code, detail = node._validate_trajectory_goal(
            _trajectory(
                ["joint2", "joint1", "joint3", "joint4", "joint5", "joint6", "joint7"],
                [(1.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            )
        )

        assert error_code == 0
        assert detail == ""
        assert buffer is not None
        assert buffer.joint_names == tuple(node.joint_names)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()