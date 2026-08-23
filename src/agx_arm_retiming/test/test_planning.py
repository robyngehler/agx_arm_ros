"""The mode layer decides whether a taught motion keeps its timing or is
re-timed, so these pin the property each mode is chosen for -- above all that
``as_recorded`` does not move the path, which is what makes it safe for a replay
that has to thread into something."""

import math

import numpy as np
import pytest

from agx_arm_retiming import (
    AS_RECORDED,
    MAXIMIZE_SPEED,
    NERO_MAX_VELOCITY,
    SMOOTH,
    SPEED_SCALE,
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


def test_as_recorded_keeps_the_path_and_the_duration():
    t, q = taught_motion()
    result = retime(t, q, AS_RECORDED)
    assert result.path_deviation == pytest.approx(0.0, abs=1e-9)
    assert result.duration == pytest.approx(t[-1], rel=1e-9)
    assert result.speed_achieved == pytest.approx(1.0)


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
    t, q = taught_motion()
    result = retime(t, q, SPEED_SCALE, speed_scale=1.5)
    assert result.speed_achieved == pytest.approx(1.5, rel=0.05)
    assert result.duration == pytest.approx(result.recorded_duration / 1.5, rel=0.05)


def test_speed_scale_below_one_slows_the_motion_down():
    t, q = taught_motion()
    result = retime(t, q, SPEED_SCALE, speed_scale=0.5)
    assert result.duration > result.recorded_duration


def test_an_unreachable_speed_returns_the_fastest_plan_and_says_so():
    t, q = taught_motion()
    result = retime(t, q, SPEED_SCALE, speed_scale=500.0)
    assert result.speed_achieved < 500.0
    assert any("fastest feasible" in note for note in result.notes)
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
