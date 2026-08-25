"""The mode layer decides whether a taught motion keeps its timing or is
re-timed, so these pin the property each mode is chosen for: that the
timing-preserving pair keeps the taught duration and stays near the taught path,
and that every mode emits a uniform grid the controller can interpolate without
stepping."""

import math

import numpy as np
import pytest

from agx_arm_retiming import (
    AS_RECORDED,
    DEFAULT_RESAMPLE_DT,
    MAXIMIZE_SPEED,
    NERO_MAX_VELOCITY,
    RECONSTRUCTION_WINDOW_SEC,
    SMOOTH,
    SPEED_SCALE,
    TEMPO_SCALE,
    RetimingError,
    default_acceleration,
    retime,
)

JOINTS = 7
RATE = 100.0
DURATION = 4.0


def taught_motion(noise=0.0, seed=1):
    """A slow, smooth 7-joint motion at the recording rate, optionally with the
    sample noise a real capture carries."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, DURATION, 1.0 / RATE)
    q = np.column_stack([
        0.6 * np.sin(2 * np.pi * (0.25 + 0.05 * j) * t / DURATION * 2) + 0.1 * j
        for j in range(JOINTS)
    ])
    if noise:
        q = q + rng.normal(0.0, noise, q.shape)
    return t.tolist(), q.tolist()


def peak_ratio(rows, limits):
    return float(np.max(np.abs(np.asarray(rows)) / np.asarray(limits)))


def uneven_motion(seed=3):
    """A taught motion whose samples land on an uneven grid, as a real capture
    does: the feedback cadence jitters and a still arm produces no update."""
    rng = np.random.default_rng(seed)
    t, q = taught_motion(noise=2e-4, seed=seed)
    keep = [0] + [i for i in range(1, len(t) - 1) if rng.random() > 0.35] + [len(t) - 1]
    return [t[i] for i in keep], [q[i] for i in keep]


def commanded_acceleration(result, control_dt=0.005):
    """Peak acceleration of the position stream a linearly interpolating
    controller walks, which is what the arm reproduces as judder."""
    times = np.asarray(result.times)
    grid = np.arange(0.0, times[-1], control_dt)
    walked = np.column_stack([
        np.interp(grid, times, np.asarray(result.positions)[:, j])
        for j in range(np.asarray(result.positions).shape[1])
    ])
    return float(np.max(np.abs(np.diff(walked, n=2, axis=0)))) / control_dt ** 2


def test_as_recorded_keeps_the_duration_and_stays_on_the_taught_path():
    t, q = taught_motion()
    result = retime(t, q, AS_RECORDED)
    assert result.duration == pytest.approx(t[-1], rel=1e-9)
    assert result.speed_achieved == pytest.approx(1.0)
    # Filtered, so not bit-exact, but a smooth taught motion is barely moved.
    assert 0.0 < result.path_deviation < 5e-3


def test_every_mode_emits_a_uniform_grid():
    """The controller interpolates linearly between trajectory points, so an
    uneven knot is a step in commanded velocity regardless of the mode. Only the
    final step may be short, where the grid does not divide the duration."""
    t, q = uneven_motion()
    for mode, kwargs in (
        (AS_RECORDED, {}), (SMOOTH, {}),
        (SPEED_SCALE, {"speed_scale": 1.0}), (MAXIMIZE_SPEED, {}),
    ):
        gaps = np.diff(retime(t, q, mode, **kwargs).times)[:-1]
        assert gaps.max() - gaps.min() < 1e-9, f"{mode} emitted an uneven grid"


def test_an_uneven_recording_does_not_reach_the_commanded_motion():
    """An uneven grid replayed as knots makes the commanded velocity a
    staircase. Resampling removes it; without that step the timing-preserving
    modes are an order of magnitude rougher than the same motion sampled
    evenly."""
    even = retime(*taught_motion(noise=2e-4, seed=3), AS_RECORDED)
    uneven = retime(*uneven_motion(seed=3), AS_RECORDED)
    assert commanded_acceleration(uneven) < 3.0 * commanded_acceleration(even)


def test_as_recorded_filters_at_the_floor_and_smooth_may_widen_it():
    t, q = uneven_motion()
    floor = retime(t, q, AS_RECORDED)
    # A window under the floor cannot make a replay rougher than the floor.
    assert retime(t, q, SMOOTH, smoothing_window_sec=0.0).path_deviation == pytest.approx(
        floor.path_deviation, rel=1e-9
    )
    assert retime(t, q, AS_RECORDED, smoothing_window_sec=0.5).path_deviation == pytest.approx(
        floor.path_deviation, rel=1e-9
    )
    wider = retime(t, q, SMOOTH, smoothing_window_sec=4 * RECONSTRUCTION_WINDOW_SEC)
    assert wider.path_deviation > floor.path_deviation
    assert commanded_acceleration(wider) < commanded_acceleration(floor)


def test_the_output_grid_follows_the_requested_resample_dt():
    t, q = taught_motion()
    for dt in (DEFAULT_RESAMPLE_DT, 0.01):
        result = retime(t, q, AS_RECORDED, resample_dt=dt)
        assert np.diff(result.times).mean() == pytest.approx(dt, rel=0.02)


def test_as_recorded_supplies_derivatives_the_recording_did_not():
    """The whole point: the coordinator path sends zero velocities today."""
    t, q = taught_motion()
    result = retime(t, q, AS_RECORDED)
    assert any(abs(v) > 1e-3 for row in result.velocities for v in row)
    assert any(abs(a) > 1e-3 for row in result.accelerations for a in row)
    # Starts and ends at rest.
    assert max(abs(v) for v in result.velocities[0]) == pytest.approx(0.0, abs=1e-9)
    assert max(abs(v) for v in result.velocities[-1]) == pytest.approx(0.0, abs=1e-9)


def test_smooth_trades_path_for_quieter_derivatives():
    t, q = taught_motion(noise=2e-3)
    rough = retime(t, q, AS_RECORDED)
    smooth = retime(t, q, SMOOTH)
    assert smooth.path_deviation > 0.0
    assert smooth.duration == pytest.approx(rough.duration, rel=1e-9)
    limits = default_acceleration(NERO_MAX_VELOCITY)
    assert peak_ratio(smooth.accelerations, limits) < peak_ratio(rough.accelerations, limits)


def test_maximize_speed_is_faster_and_discards_the_taught_timing():
    t, q = taught_motion()
    result = retime(t, q, MAXIMIZE_SPEED)
    assert result.duration < result.recorded_duration
    assert result.speed_achieved > 1.0
    assert any("taught timing discarded" in note for note in result.notes)


def test_retimed_output_respects_the_limits_it_was_given():
    t, q = taught_motion()
    result = retime(t, q, MAXIMIZE_SPEED)
    assert result.velocity_utilisation <= 1.0
    assert result.acceleration_utilisation <= 1.0


def test_speed_scale_hits_the_requested_multiple_of_the_recording():
    """Asked for a multiple the path can reach, it lands near it on the slow
    side. Duration is not monotone in the limit scale, so 'near' is a few
    percent rather than the search tolerance."""
    t, q = taught_motion()
    ceiling = retime(t, q, MAXIMIZE_SPEED).speed_achieved
    requested = 0.95 * ceiling
    result = retime(t, q, SPEED_SCALE, speed_scale=requested)
    assert result.speed_achieved == pytest.approx(requested, rel=0.05)
    assert result.duration == pytest.approx(result.recorded_duration / requested, rel=0.05)


def test_speed_scale_below_one_slows_the_motion_down():
    t, q = taught_motion()
    result = retime(t, q, SPEED_SCALE, speed_scale=0.5)
    assert result.duration > result.recorded_duration


def test_an_unreachable_speed_returns_the_fastest_plan_and_says_so():
    t, q = taught_motion()
    result = retime(t, q, SPEED_SCALE, speed_scale=500.0)
    assert result.speed_achieved < 500.0
    assert any("reached" in note for note in result.notes)
    assert result.velocity_utilisation <= 1.0


def test_a_recording_that_already_exceeds_the_limits_is_reported_not_hidden():
    t, q = taught_motion()
    slow_limits = [0.01] * JOINTS
    result = retime(t, q, AS_RECORDED, max_velocity=slow_limits,
                    max_acceleration=default_acceleration(slow_limits))
    assert result.velocity_utilisation > 1.0
    assert any("the recording, not the plan" in note for note in result.notes)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"mode": "warp_speed"}, "unknown mode"),
        ({"mode": SPEED_SCALE, "speed_scale": 0.0}, "speed_scale"),
        ({"max_velocity": [1.0] * (JOINTS - 1)}, "max_velocity has"),
        ({"max_velocity": [0.0] * JOINTS}, "must be finite"),
    ],
)
def test_rejects_unusable_requests(kwargs, message):
    t, q = taught_motion()
    call = {"mode": AS_RECORDED}
    call.update(kwargs)
    if "max_velocity" in call and "max_acceleration" not in call:
        call["max_acceleration"] = default_acceleration(call["max_velocity"])
    with pytest.raises(RetimingError, match=message):
        retime(t, q, **call)


def test_rejects_non_monotonic_recorded_time():
    t, q = taught_motion()
    t[5], t[6] = t[6], t[5]
    with pytest.raises(RetimingError, match="strictly increasing"):
        retime(t, q, AS_RECORDED)


def test_manufacturer_limits_are_the_documented_ones():
    """180 deg/s on J1-J3 and 225 deg/s on J4-J7, per the datasheet."""
    degrees = [v / (math.pi / 180.0) for v in NERO_MAX_VELOCITY]
    assert degrees == pytest.approx([180, 180, 180, 225, 225, 225, 225])


def test_speed_scale_never_returns_faster_than_requested():
    """On a taught motion an unrequested speed-up is the dangerous direction, so
    the search resolves to the slow side of the target."""
    t, q = taught_motion()
    for requested in (1.2, 1.5, 2.0):
        result = retime(t, q, SPEED_SCALE, speed_scale=requested)
        assert result.speed_achieved <= requested * 1.01, (
            f"asked for {requested}x, got {result.speed_achieved}x"
        )


def test_planned_path_never_leaves_the_recorded_range():
    """A quintic smoothing spline overshot joint 4 by 2.48 rad on a real
    recording and commanded a pose 142 degrees outside anything taught. Every
    smoothing step is now an average, and the result is checked besides."""
    t, q = taught_motion(noise=5e-3, seed=7)
    recorded = np.asarray(q)
    for mode, kwargs in ((MAXIMIZE_SPEED, {}), (SPEED_SCALE, {"speed_scale": 1.5})):
        planned = np.asarray(retime(t, q, mode, **kwargs).positions)
        assert planned.min(axis=0).min() >= recorded.min(axis=0).min() - 0.05
        assert planned.max(axis=0).max() <= recorded.max(axis=0).max() + 0.05


def test_an_excursion_past_the_recorded_range_is_refused():
    from agx_arm_retiming.planning import _assert_within_recorded_range

    recorded = np.array([[0.0], [1.0], [0.5]])
    _assert_within_recorded_range(np.array([[0.2], [0.9]]), recorded, 0.02)
    with pytest.raises(RetimingError, match="leaves the recorded range on joint 1"):
        _assert_within_recorded_range(np.array([[0.2], [1.6]]), recorded, 0.02)


# --- tempo_scale and the geometric path -----------------------------------


def test_tempo_scale_stretches_the_clock_and_keeps_the_shape():
    """The mode exists because a path parameterization cannot preserve local
    tempo: dwells and reversals are not part of its objective."""
    t, q = taught_motion()
    base = retime(t, q, SMOOTH)
    slow = retime(t, q, TEMPO_SCALE, speed_scale=0.5)

    assert slow.duration == pytest.approx(2.0 * base.duration, rel=1e-6)
    assert slow.speed_achieved == pytest.approx(0.5)
    # Same path, walked at half the pace.
    assert np.allclose(
        np.asarray(slow.positions)[:: 2][: len(base.positions)],
        np.asarray(base.positions)[: len(np.asarray(slow.positions)[:: 2])],
        atol=2e-3,
    )


def test_tempo_scale_lowers_commanded_velocity_by_the_factor():
    """This is the lever for a take taught faster than the arm can command."""
    t, q = taught_motion()
    fast = retime(t, q, TEMPO_SCALE, speed_scale=1.0)
    slow = retime(t, q, TEMPO_SCALE, speed_scale=0.5)
    assert slow.velocity_utilisation < 0.6 * fast.velocity_utilisation


def test_tempo_scale_preserves_the_taught_speed_profile():
    """A dwell stays a dwell relative to the motion around it — the property
    TOTG discards, measured as the correlation of the normalised speed."""
    t, q = taught_motion()

    def profile(result, samples=200):
        speed = np.linalg.norm(np.asarray(result.velocities), axis=1)
        resampled = np.interp(
            np.linspace(0.0, 1.0, samples),
            np.linspace(0.0, 1.0, len(speed)),
            speed,
        )
        return resampled / max(resampled.max(), 1e-9)

    taught = profile(retime(t, q, SMOOTH))
    assert np.corrcoef(taught, profile(retime(t, q, TEMPO_SCALE, speed_scale=0.5)))[0, 1] > 0.95


def test_tempo_scale_rejects_an_unusable_factor():
    t, q = taught_motion()
    with pytest.raises(RetimingError, match="speed_scale"):
        retime(t, q, TEMPO_SCALE, speed_scale=0.0)


def test_the_geometric_path_is_robust_to_the_sampling_pattern():
    """The same motion sampled evenly and unevenly must produce approximately
    the same geometric path — the smoothing is a time-domain filter only after
    the uniform resample, and the waypoints are chosen by chord error, so
    neither depends on where the samples happened to land."""
    from agx_arm_retiming.planning import _geometric_path

    t, q = taught_motion(noise=2e-4, seed=5)
    even, _ = _geometric_path(
        np.asarray(t), np.asarray(q),
        smoothing_window_sec=0.10, waypoints=40, resample_dt=DEFAULT_RESAMPLE_DT,
    )
    ut, uq = uneven_motion(seed=5)
    uneven, _ = _geometric_path(
        np.asarray(ut), np.asarray(uq),
        smoothing_window_sec=0.10, waypoints=40, resample_dt=DEFAULT_RESAMPLE_DT,
    )
    # Compare as paths: the farthest either polyline strays from the other.
    def gap(a, b):
        worst = 0.0
        for point in a:
            worst = max(worst, float(np.min(np.linalg.norm(b - point, axis=1))))
        return worst

    assert max(gap(even, uneven), gap(uneven, even)) < 0.05


def test_chord_waypoints_land_where_the_path_bends():
    from agx_arm_retiming.planning import _chord_waypoints

    # A path that is straight for most of its length with one sharp corner.
    straight = np.linspace(0.0, 1.0, 100)
    path = np.column_stack([straight, np.abs(straight - 0.5)])
    chosen = _chord_waypoints(path, 6)

    assert chosen[0] == 0 and chosen[-1] == len(path) - 1
    # The corner is at index 49/50; a waypoint must land on it.
    assert min(abs(index - 49) for index in chosen) <= 2


def test_path_deviation_covers_the_sparse_path_too():
    """The chord error was previously invisible: only the smoothing stage was
    reported, while the waypoints cut whatever their chords cut."""
    t, q = taught_motion(noise=1e-3, seed=11)
    coarse = retime(t, q, MAXIMIZE_SPEED, waypoints=6)
    fine = retime(t, q, MAXIMIZE_SPEED, waypoints=80)
    assert coarse.path_deviation > fine.path_deviation


def test_the_scale_search_survives_a_non_monotone_duration():
    """`_run_totg` corrects its limits against the peak it samples, and that
    correction moves with the blend radii, so a higher scale can produce a
    longer trajectory. A bisection reports its own dead end as the hardware's."""
    t, q = taught_motion()
    ceiling = retime(t, q, MAXIMIZE_SPEED).speed_achieved
    for fraction in (0.75, 0.85, 0.95):
        result = retime(t, q, SPEED_SCALE, speed_scale=fraction * ceiling)
        # Never faster than asked, and never so far short that it implies a
        # ceiling well below the one maximize_speed measured.
        assert result.speed_achieved <= fraction * ceiling * 1.01
        assert result.speed_achieved > 0.85 * fraction * ceiling
