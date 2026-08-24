import pytest

from agx_arm_mit_controller.trajectory_io import (
    RecordedTrajectory,
    RecordedTrajectoryPoint,
    load_recorded_trajectory,
    recorded_to_joint_trajectory,
    sanitize_trajectory_name,
    save_recorded_trajectory,
    smooth_recorded_trajectory,
    smoothing_window_samples,
    trim_trailing_stationary_points,
)


def _staircase_trajectory() -> RecordedTrajectory:
    # Stale-sample staircase: every second sample repeats the previous position,
    # so raw finite-difference velocities chatter between 0 and twice the true
    # value (the recorded-playback judder pattern).
    points = []
    position = 0.0
    for index in range(11):
        if index > 0 and index % 2 == 0:
            position += 0.04
        velocity = 0.0 if index % 2 == 1 or index == 0 else 2.0
        points.append(
            RecordedTrajectoryPoint(index * 0.02, [position], [velocity], [0.0])
        )
    return RecordedTrajectory(
        name="staircase",
        robot="nero",
        joint_names=["joint1"],
        sample_rate_hz=50.0,
        recorded_at="2026-07-03T00:00:00+00:00",
        points=points,
        metadata={},
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


def test_recorded_to_joint_trajectory_zeroes_efforts_by_default():
    trajectory = RecordedTrajectory(
        name="demo",
        robot="nero",
        joint_names=["joint1", "joint2"],
        sample_rate_hz=50.0,
        recorded_at="2026-05-07T00:00:00+00:00",
        points=[RecordedTrajectoryPoint(0.0, [0.1, 0.2], [0.0, 0.0], [1.2, -0.4])],
        metadata={"kind": "test"},
    )

    msg = recorded_to_joint_trajectory(trajectory)

    assert list(msg.points[0].effort) == [0.0, 0.0]


def test_recorded_to_joint_trajectory_scales_time_and_prepends_lead_in():
    trajectory = RecordedTrajectory(
        name="demo",
        robot="nero",
        joint_names=["joint1", "joint2"],
        sample_rate_hz=50.0,
        recorded_at="2026-05-07T00:00:00+00:00",
        points=[
            RecordedTrajectoryPoint(0.2, [0.1, 0.2], [0.4, 0.6], [0.0, 0.0]),
            RecordedTrajectoryPoint(0.4, [0.3, 0.5], [0.8, 1.2], [0.0, 0.0]),
        ],
        metadata={"kind": "test"},
    )

    msg = recorded_to_joint_trajectory(
        trajectory,
        time_scale=2.0,
        current_positions=[-0.2, -0.1],
        lead_in_sec=1.5,
    )

    assert len(msg.points) == 3
    assert list(msg.points[0].positions) == [-0.2, -0.1]
    assert msg.points[0].time_from_start.sec == 0
    assert msg.points[1].time_from_start.sec == 1
    assert msg.points[1].time_from_start.nanosec == 500_000_000
    assert msg.points[2].time_from_start.sec == 1
    assert msg.points[2].time_from_start.nanosec == 900_000_000
    assert list(msg.points[2].velocities) == [0.4, 0.6]


def test_smooth_recorded_trajectory_flattens_velocity_chatter():
    trajectory = _staircase_trajectory()

    smoothed = smooth_recorded_trajectory(trajectory, smoothing_window=5)

    def interior_jumps(points):
        # skip the first/last transition: v is pinned to 0 at the endpoints, so
        # the boundary jump is real start/stop signal, not chatter
        return [
            abs(b.velocities[0] - a.velocities[0])
            for a, b in zip(points[1:-1], points[2:-1])
        ]

    raw_jumps = interior_jumps(trajectory.points)
    smooth_jumps = interior_jumps(smoothed.points)
    assert max(raw_jumps) == 2.0
    assert max(smooth_jumps) < 0.2 * max(raw_jumps)
    # endpoints stay exact and at rest, timing untouched
    assert smoothed.points[0].positions == trajectory.points[0].positions
    assert smoothed.points[-1].positions == trajectory.points[-1].positions
    assert smoothed.points[0].velocities == [0.0]
    assert smoothed.points[-1].velocities == [0.0]
    assert [p.time_from_start for p in smoothed.points] == [
        p.time_from_start for p in trajectory.points
    ]
    # interior positions stay close to the recorded path (no distortion)
    assert all(
        abs(s.positions[0] - r.positions[0]) < 0.03
        for s, r in zip(smoothed.points, trajectory.points)
    )
    assert smoothed.metadata["playback_smoothing_window"] == 5


def _rated(rate_hz: float, count: int) -> RecordedTrajectory:
    step = 1.0 / rate_hz
    return RecordedTrajectory(
        name="rated", robot="nero", joint_names=["joint1"], sample_rate_hz=rate_hz,
        recorded_at="2026-08-22T00:00:00+00:00",
        points=[
            RecordedTrajectoryPoint(index * step, [index * 0.001], [0.0], [0.0])
            for index in range(count)
        ],
        metadata={},
    )


def test_smoothing_window_is_the_same_duration_at_any_recording_rate():
    """The filter must not change when the recording rate does.

    Expressed in samples, raising teach recordings from 50 Hz to 100 Hz halved
    the smoothing without anyone touching the setting.
    """
    assert smoothing_window_samples(_rated(50.0, 100), 0.30) == 15
    assert smoothing_window_samples(_rated(100.0, 200), 0.30) == 30
    assert smoothing_window_samples(_rated(200.0, 400), 0.30) == 60


def test_smoothing_window_falls_back_to_the_recorded_timestamps():
    """A recording that declares no rate still gets the right window."""
    trajectory = _rated(100.0, 200)
    undeclared = RecordedTrajectory(
        name=trajectory.name, robot=trajectory.robot,
        joint_names=list(trajectory.joint_names), sample_rate_hz=0.0,
        recorded_at=trajectory.recorded_at, points=trajectory.points, metadata={},
    )
    assert smoothing_window_samples(undeclared, 0.30) == 30


def test_smoothing_window_is_zero_when_no_rate_can_be_established():
    empty = RecordedTrajectory(
        name="empty", robot="nero", joint_names=["joint1"], sample_rate_hz=0.0,
        recorded_at="2026-08-22T00:00:00+00:00", points=[], metadata={},
    )
    assert smoothing_window_samples(empty, 0.30) == 0
    assert smoothing_window_samples(_rated(100.0, 200), 0.0) == 0


def test_smooth_recorded_trajectory_window_leq_one_is_identity():
    trajectory = _staircase_trajectory()

    assert smooth_recorded_trajectory(trajectory, smoothing_window=1) is trajectory
    assert smooth_recorded_trajectory(trajectory, smoothing_window=0) is trajectory

def test_deduplicate_drops_repeats_and_keeps_the_taught_path():
    """A repeat is what the clock stored when the arm had nothing new. Removing
    it must not move a single surviving sample."""
    from agx_arm_mit_controller.trajectory_io import deduplicate_recorded_trajectory

    points = [
        RecordedTrajectoryPoint(0.00, [0.0], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.01, [0.0], [0.0], [0.0]),   # repeat
        RecordedTrajectoryPoint(0.02, [0.1], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.03, [0.1], [0.0], [0.0]),   # repeat
        RecordedTrajectoryPoint(0.04, [0.2], [0.0], [0.0]),
    ]
    trajectory = RecordedTrajectory(
        name="dup", robot="nero", joint_names=["joint1"], sample_rate_hz=100.0,
        recorded_at="now", points=points, metadata={},
    )
    cleaned, removed = deduplicate_recorded_trajectory(trajectory)

    assert removed == 2
    assert [p.time_from_start for p in cleaned.points] == [0.0, 0.02, 0.04]
    assert [p.positions for p in cleaned.points] == [[0.0], [0.1], [0.2]]
    # The stored rate becomes what survived, since that is what a reader believes.
    assert cleaned.sample_rate_hz == pytest.approx(50.0)
    assert cleaned.metadata["deduplicated"]["removed_repeats"] == 2
    assert cleaned.metadata["deduplicated"]["original_sample_rate_hz"] == 100.0


def test_deduplicate_keeps_the_final_pose_even_when_it_repeats():
    """Dropping a trailing repeat would leave a replay short of the taught end."""
    from agx_arm_mit_controller.trajectory_io import deduplicate_recorded_trajectory

    points = [
        RecordedTrajectoryPoint(0.00, [0.0], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.01, [0.5], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.02, [0.5], [0.0], [0.0]),
        RecordedTrajectoryPoint(0.03, [0.5], [0.0], [0.0]),
    ]
    trajectory = RecordedTrajectory(
        name="tail", robot="nero", joint_names=["joint1"], sample_rate_hz=100.0,
        recorded_at="now", points=points, metadata={},
    )
    cleaned, removed = deduplicate_recorded_trajectory(trajectory)
    # Two interior repeats drop, the last one comes back as the end point, so the
    # net removal is one and the recording keeps the duration it was taught with.
    assert removed == 1
    assert cleaned.points[-1].time_from_start == pytest.approx(0.03)
    assert cleaned.points[-1].positions == [0.5]
    assert cleaned.duration == pytest.approx(trajectory.duration)


def test_deduplicate_recomputes_velocities_over_the_new_spacing():
    from agx_arm_mit_controller.trajectory_io import deduplicate_recorded_trajectory

    points = [
        RecordedTrajectoryPoint(0.0, [0.0], [99.0], [0.0]),
        RecordedTrajectoryPoint(0.1, [0.0], [99.0], [0.0]),
        RecordedTrajectoryPoint(0.2, [1.0], [99.0], [0.0]),
        RecordedTrajectoryPoint(0.4, [3.0], [99.0], [0.0]),
    ]
    trajectory = RecordedTrajectory(
        name="v", robot="nero", joint_names=["joint1"], sample_rate_hz=10.0,
        recorded_at="now", points=points, metadata={},
    )
    cleaned, _ = deduplicate_recorded_trajectory(trajectory)
    # Middle sample: central difference over 0.0 -> 0.4, i.e. (3.0 - 0.0) / 0.4.
    assert cleaned.points[1].velocities[0] == pytest.approx(7.5)
    assert cleaned.points[0].velocities == [0.0]
    assert cleaned.points[-1].velocities == [0.0]


def test_a_recording_without_repeats_is_returned_untouched():
    from agx_arm_mit_controller.trajectory_io import deduplicate_recorded_trajectory

    points = [RecordedTrajectoryPoint(i * 0.01, [i * 0.1], [0.0], [0.0]) for i in range(5)]
    trajectory = RecordedTrajectory(
        name="clean", robot="nero", joint_names=["joint1"], sample_rate_hz=100.0,
        recorded_at="now", points=points, metadata={},
    )
    cleaned, removed = deduplicate_recorded_trajectory(trajectory)
    assert removed == 0
    assert cleaned is trajectory
