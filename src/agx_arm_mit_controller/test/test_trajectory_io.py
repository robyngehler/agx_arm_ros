from agx_arm_mit_controller.trajectory_io import (
    RecordedTrajectory,
    RecordedTrajectoryPoint,
    load_recorded_trajectory,
    sanitize_trajectory_name,
    save_recorded_trajectory,
    trim_trailing_stationary_points,
)


def test_trim_trailing_stationary_points_removes_idle_tail():
    points = [
        RecordedTrajectoryPoint(0.0, [0.0], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.1, [0.2], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.2, [0.2], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.3, [0.2], [0.0], [0.0]),
    ]

    trimmed, last_motion_index = trim_trailing_stationary_points(points, movement_threshold_rad=0.01)

    assert last_motion_index == 1
    assert len(trimmed) == 2


def test_recorded_trajectory_round_trip(tmp_path):
    path = tmp_path / "demo.json"
    trajectory = RecordedTrajectory(
        name="demo",
        robot="nero",
        joint_names=["joint1"],
        sample_rate_hz=50.0,
        recorded_at="2026-05-07T00:00:00+00:00",
        points=[RecordedTrajectoryPoint(0.0, [0.1], [0.0], [0.0])],
        metadata={"kind": "test"},
    )

    save_recorded_trajectory(trajectory, path)
    loaded = load_recorded_trajectory(path)

    assert loaded == trajectory


def test_sanitize_trajectory_name_rewrites_spaces():
    assert sanitize_trajectory_name("  demo trajectory 01 ") == "demo_trajectory_01"