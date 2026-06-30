from __future__ import annotations

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_mit_tools.both_arms_trajectory_bridge import _split_indices, split_trajectory


def _combined_trajectory() -> JointTrajectory:
    names = [f"left_arm_joint{i}" for i in range(1, 8)] + [
        f"right_arm_joint{i}" for i in range(1, 8)
    ]
    traj = JointTrajectory()
    traj.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [float(i) for i in range(14)]
    point.velocities = [float(i) * 0.1 for i in range(14)]
    traj.points = [point]
    return traj


def test_split_indices_route_by_prefix():
    names = ["left_arm_joint1", "right_arm_joint1", "left_arm_joint2"]
    assert _split_indices(names, "left_arm_") == [0, 2]
    assert _split_indices(names, "right_arm_") == [1]


def test_split_trajectory_partitions_names_and_values():
    traj = _combined_trajectory()
    left_idx = _split_indices(traj.joint_names, "left_arm_")
    right_idx = _split_indices(traj.joint_names, "right_arm_")

    left = split_trajectory(traj, left_idx, [traj.joint_names[i] for i in left_idx])
    right = split_trajectory(traj, right_idx, [traj.joint_names[i] for i in right_idx])

    assert left.joint_names == [f"left_arm_joint{i}" for i in range(1, 8)]
    assert right.joint_names == [f"right_arm_joint{i}" for i in range(1, 8)]
    # left gets the first 7 positions, right the last 7 (this trajectory's layout).
    assert list(left.points[0].positions) == [float(i) for i in range(7)]
    assert list(right.points[0].positions) == [float(i) for i in range(7, 14)]
    assert list(right.points[0].velocities) == [float(i) * 0.1 for i in range(7, 14)]


def test_split_trajectory_handles_interleaved_order():
    names = ["right_arm_joint1", "left_arm_joint1", "right_arm_joint2", "left_arm_joint2"]
    traj = JointTrajectory()
    traj.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [10.0, 20.0, 11.0, 21.0]
    traj.points = [point]

    left_idx = _split_indices(names, "left_arm_")
    left = split_trajectory(traj, left_idx, [names[i] for i in left_idx])
    assert left.joint_names == ["left_arm_joint1", "left_arm_joint2"]
    assert list(left.points[0].positions) == [20.0, 21.0]
