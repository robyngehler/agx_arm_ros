from sensor_msgs.msg import JointState

from agx_arm_mit_controller.joint_state_trajectory_bridge import (
    build_single_point_trajectory,
    select_target_positions,
)


def test_select_target_positions_updates_only_controlled_joints():
    msg = JointState()
    msg.name = ["joint2", "gripper_joint1"]
    msg.position = [1.25, 0.4]

    target = select_target_positions(
        ["joint1", "joint2", "joint3"],
        [0.1, 0.2, 0.3],
        msg,
    )

    assert target == [0.1, 1.25, 0.3]


def test_build_single_point_trajectory_sets_duration_and_zeroes():
    trajectory = build_single_point_trajectory(
        ["joint1", "joint2"],
        [0.5, -0.25],
        0.75,
    )

    assert trajectory.joint_names == ["joint1", "joint2"]
    assert len(trajectory.points) == 1
    point = trajectory.points[0]
    assert list(point.positions) == [0.5, -0.25]
    assert list(point.velocities) == [0.0, 0.0]
    assert list(point.effort) == [0.0, 0.0]
    assert point.time_from_start.sec == 0
    assert point.time_from_start.nanosec == 750000000