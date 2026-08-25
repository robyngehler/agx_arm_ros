"""Unit tests for the feedforward velocities a recorded replay is dispatched with.

The MIT controller reads a missing velocity as a commanded zero, so the kd term
brakes against the motion the position term is asking for.
"""

import pytest

from agx_arm_coordination.arm_executor import TrajectoryPoint
from agx_arm_coordination.coordinator_node import _waypoint_velocities


def _points(times, positions):
    return [
        TrajectoryPoint(positions=tuple(p), time_from_start_sec=t)
        for t, p in zip(times, positions)
    ]


def test_velocities_follow_the_polyline_the_controller_walks():
    points = _points([0.0, 1.0, 2.0, 3.0], [[0.0], [0.1], [0.3], [0.6]])
    velocities = _waypoint_velocities(points)

    assert len(velocities) == len(points)
    # Endpoints at rest, interior a central difference over the waypoint times.
    assert velocities[0] == (0.0,)
    assert velocities[-1] == (0.0,)
    assert velocities[1][0] == pytest.approx((0.3 - 0.0) / 2.0)
    assert velocities[2][0] == pytest.approx((0.6 - 0.1) / 2.0)


def test_a_multi_joint_waypoint_gets_one_velocity_per_joint():
    points = _points([0.0, 0.5, 1.0], [[0.0, 1.0], [0.2, 0.8], [0.4, 0.4]])
    velocities = _waypoint_velocities(points)
    assert [len(v) for v in velocities] == [2, 2, 2]
    assert velocities[1][0] == pytest.approx(0.4)
    assert velocities[1][1] == pytest.approx(-0.6)


def test_a_zero_span_yields_rest_rather_than_a_division_by_zero():
    # The central difference spans the neighbours, so it takes three equal times
    # to collapse one — a waypoint block that repeats a timestamp.
    points = _points([0.0, 1.0, 1.0, 1.0, 2.0], [[0.0], [0.1], [0.2], [0.3], [0.4]])
    velocities = _waypoint_velocities(points)
    assert all(len(v) == 1 for v in velocities)
    assert velocities[2] == (0.0,)


def test_a_trajectory_too_short_to_difference_is_all_rest():
    for count in (0, 1, 2):
        points = _points([float(i) for i in range(count)], [[0.0]] * count)
        velocities = _waypoint_velocities(points)
        assert len(velocities) == count
        assert all(v == (0.0,) for v in velocities)


def test_a_dwell_is_commanded_at_rest():
    """A held pose must not carry a feedforward: the arm is meant to stay."""
    points = _points([0.0, 1.0, 2.0, 3.0], [[0.5], [0.5], [0.5], [0.5]])
    assert all(v == (0.0,) for v in _waypoint_velocities(points))
