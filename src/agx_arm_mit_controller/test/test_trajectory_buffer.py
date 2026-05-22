from types import SimpleNamespace

from agx_arm_mit_controller.trajectory_buffer import JointTrajectoryBuffer


def make_point(time_sec, positions, velocities=None, effort=None):
    return SimpleNamespace(
        time_from_start=SimpleNamespace(sec=time_sec, nanosec=0),
        positions=positions,
        velocities=velocities or [],
        effort=effort or [],
    )


def test_joint_trajectory_buffer_interpolates_midpoint():
    msg = SimpleNamespace(
        joint_names=["joint1", "joint2"],
        points=[
            make_point(1, [0.0, 1.0], [0.0, 0.0], [0.0, 0.0]),
            make_point(3, [2.0, 3.0], [1.0, 1.0], [0.5, 0.5]),
        ],
    )

    buffer = JointTrajectoryBuffer.from_ros_message(["joint1", "joint2"], msg)
    sampled = buffer.sample(2.0)

    assert sampled.positions == (1.0, 2.0)
    assert sampled.velocities == (0.5, 0.5)
    assert sampled.efforts == (0.25, 0.25)


def test_joint_trajectory_buffer_rejects_name_mismatch():
    msg = SimpleNamespace(
        joint_names=["joint1", "jointX"],
        points=[make_point(1, [0.0, 1.0])],
    )

    try:
        JointTrajectoryBuffer.from_ros_message(["joint1", "joint2"], msg)
    except ValueError as exc:
        assert "joint_names mismatch" in str(exc)
    else:
        raise AssertionError("Expected ValueError for joint_names mismatch")


def test_joint_trajectory_buffer_reorders_joint_names_when_enabled():
    msg = SimpleNamespace(
        joint_names=["joint2", "joint1"],
        points=[make_point(1, [1.0, 2.0], [0.1, 0.2], [0.3, 0.4])],
    )

    buffer = JointTrajectoryBuffer.from_ros_message(
        ["joint1", "joint2"],
        msg,
        allow_joint_reordering=True,
    )

    assert buffer.initial_point.positions == (2.0, 1.0)
    assert buffer.initial_point.velocities == (0.2, 0.1)
    assert buffer.initial_point.efforts == (0.4, 0.3)
