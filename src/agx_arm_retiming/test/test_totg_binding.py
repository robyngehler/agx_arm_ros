"""The TOTG binding is the only path from this Python stack to MoveIt's
time-optimal parameterization, so these pin the properties the callers rely on:
the limits are actually respected, the path is honoured, and a failure is
reported rather than substituted."""

import math

import pytest

from agx_arm_retiming import retime_path

# A short two-joint path with a corner, so the parameterization has to slow down
# somewhere rather than running one straight segment at the velocity limit.
PATH = [
    [0.0, 0.0],
    [0.5, 0.0],
    [1.0, 0.5],
    [1.0, 1.0],
]
V_MAX = [1.0, 1.0]
A_MAX = [2.0, 2.0]


def peak(rows):
    return max(max(abs(value) for value in row) for row in rows)


def test_retimes_and_reports_valid():
    result = retime_path(PATH, V_MAX, A_MAX, resample_dt=0.01)
    assert result["valid"] is True
    assert result["duration"] > 0.0
    assert len(result["times"]) == len(result["positions"])
    assert len(result["times"]) == len(result["velocities"])
    assert len(result["times"]) == len(result["accelerations"])


def test_respects_velocity_and_acceleration_limits():
    result = retime_path(PATH, V_MAX, A_MAX, resample_dt=0.002)
    # A small tolerance: the profile is sampled, not evaluated at its extrema.
    assert peak(result["velocities"]) <= max(V_MAX) * 1.02
    assert peak(result["accelerations"]) <= max(A_MAX) * 1.05


def test_endpoints_are_exact_and_at_rest():
    result = retime_path(PATH, V_MAX, A_MAX, resample_dt=0.01)
    assert result["positions"][0] == pytest.approx(PATH[0], abs=1e-9)
    assert result["positions"][-1] == pytest.approx(PATH[-1], abs=1e-6)
    assert result["times"][-1] == pytest.approx(result["duration"], abs=1e-12)
    assert peak([result["velocities"][0]]) == pytest.approx(0.0, abs=1e-6)
    assert peak([result["velocities"][-1]]) == pytest.approx(0.0, abs=1e-6)


def test_raising_the_limits_shortens_the_motion():
    slow = retime_path(PATH, V_MAX, A_MAX, resample_dt=0.01)
    fast = retime_path(PATH, [2.0, 2.0], [4.0, 4.0], resample_dt=0.01)
    assert fast["duration"] < slow["duration"]


def test_zero_deviation_keeps_the_path_through_its_waypoints():
    """max_deviation=0 is what makes a critical replay safe to re-time: the
    geometry is the taught one, only the timing is new."""
    result = retime_path(PATH, V_MAX, A_MAX, max_deviation=0.0, resample_dt=0.001)
    for waypoint in PATH:
        closest = min(
            math.dist(waypoint, position) for position in result["positions"]
        )
        assert closest < 5e-3


def test_blending_shortens_the_path_and_stays_within_its_tolerance():
    """Blending is a geometry trade, not a speed guarantee.

    It corners through an arc instead of stopping, which always shortens the
    path but does not always shorten the duration: on short segments the arc's
    curvature limit can cost more than the stop it avoids. Measured here, a 0.1
    blend made this path 2.042 s against 2.021 s exact. Assert the geometry,
    which holds, not the duration, which depends on the path.
    """
    deviation = 0.1
    exact = retime_path(PATH, V_MAX, A_MAX, max_deviation=0.0, resample_dt=0.01)
    blended = retime_path(PATH, V_MAX, A_MAX, max_deviation=deviation, resample_dt=0.001)
    assert blended["path_length"] < exact["path_length"]
    for waypoint in PATH:
        closest = min(math.dist(waypoint, p) for p in blended["positions"])
        assert closest <= deviation * 1.05


@pytest.mark.parametrize(
    "kwargs",
    [
        {"waypoints": [[0.0, 0.0]]},
        {"max_velocity": [0.0, 1.0]},
        {"max_velocity": [float("nan"), 1.0]},
        {"max_velocity": [1.0]},
        {"resample_dt": 0.0},
        {"max_deviation": -1.0},
    ],
)
def test_rejects_unusable_input(kwargs):
    call = {
        "waypoints": PATH,
        "max_velocity": V_MAX,
        "max_acceleration": A_MAX,
    }
    call.update(kwargs)
    with pytest.raises(ValueError):
        retime_path(**call)


def test_rejects_a_waypoint_of_the_wrong_width():
    with pytest.raises(ValueError):
        retime_path([[0.0, 0.0], [1.0]], V_MAX, A_MAX)
