"""Turn a recording into an executable trajectory, under one of four modes.

The modes split by what the underlying tool can express:

* ``as_recorded`` and ``smooth`` keep the taught duration and the taught pace.
  Both resample the recording onto a uniform grid, filter it there, and take
  derivatives there; they differ only in filter width.
* ``speed_scale`` and ``maximize_speed`` hand the geometric path to MoveIt's
  time-optimal parameterization, which computes its own timing and so discards
  anything purely temporal in the recording — a dwell is zero path progress.

Nothing here fits a curve. Every smoothing step is a moving average or a bin
mean, so a planned pose is a convex combination of recorded ones and cannot
leave the range the recording covers.

**Both timing-preserving modes resample and filter, ``as_recorded`` included.**
A recording carries an uneven time grid, and the controller interpolates linearly
between trajectory points, so an uneven knot is a step in commanded velocity: 27-43
rad/s² of commanded acceleration against 6 on the same recording resampled.
``RECONSTRUCTION_WINDOW_SEC`` is the filter floor below which sample noise
dominates. ``as_recorded`` therefore means the taught path and pace at the
smallest filter that executes, not an unprocessed sample dump
(``docs/sprint_refactor/reference/teach_replay_timebase.md``).

The parameterization needs a smoothed, sparse path: fed a dense recording it
produces either curvature spikes at every sample or blend radii so small the
result is slower than the recording
(``docs/sprint_refactor/reference/trajectory_retiming.md``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from agx_arm_retiming._totg import retime_path

AS_RECORDED = "as_recorded"
SMOOTH = "smooth"
TEMPO_SCALE = "tempo_scale"
SPEED_SCALE = "speed_scale"
MAXIMIZE_SPEED = "maximize_speed"
MODES = (AS_RECORDED, SMOOTH, TEMPO_SCALE, SPEED_SCALE, MAXIMIZE_SPEED)
#: Modes that keep the taught timing structure and never reach the parameterization.
TIMING_PRESERVING = (AS_RECORDED, SMOOTH, TEMPO_SCALE)

_DEG = math.pi / 180.0

# Manufacturer maximum joint speeds: J1-J3 180 deg/s, J4-J7 225 deg/s. The
# planner config and the MIT controller's clamp declare the same figures, so a
# plan made here is executable rather than silently clamped.
NERO_MAX_VELOCITY = (
    180 * _DEG, 180 * _DEG, 180 * _DEG,
    225 * _DEG, 225 * _DEG, 225 * _DEG, 225 * _DEG,
)

# No acceleration is specified for these joints. 2.5/s is a deliberately
# conservative stand-in, not a measurement — it says a joint may reach full
# speed in 0.4 s.
ACCELERATION_PER_VELOCITY = 2.5

# Waypoints handed to the parameterization. Chosen by chord error, so they land
# on corners and reversals and extra ones refine rather than add noise: across
# four recordings the worst chord error falls 0.076 rad at 40 to 0.024 at 80,
# for 0.2% more duration. A count above ~80 keeps buying fidelity but the
# planning cost grows.
DEFAULT_WAYPOINTS = 80
DEFAULT_BLEND_TOLERANCE = 0.01
# Moving-average width in seconds, used by the timing-preserving modes and by
# the geometric path handed to the parameterization.
DEFAULT_SMOOTHING_WINDOW_SEC = 0.10
DEFAULT_RESAMPLE_DT = 0.005
# Smallest filter width the timing-preserving modes apply, `as_recorded`
# included. Commanded acceleration on two 7-joint recordings: 27-43 rad/s²
# unfiltered, 6 here, 4 at the 0.10 s default. Costs 0.01-0.05 rad of path
# deviation.
RECONSTRUCTION_WINDOW_SEC = 0.06
# How far past the recorded per-joint range a plan may still land, on top of
# whatever the smoothing moved it. Corner blending cuts inside the path, so the
# margin only has to absorb rounding.
PATH_EXCURSION_TOLERANCE = 0.02
# TOTG bounds the profile along its path segments, but curvature jumps
# discontinuously where a straight segment meets a blend arc, so the sampled
# acceleration overshoots there — measured at 1.3x with a fixed derating. The
# limits handed to it are corrected against the sampled peak instead of by a
# guessed factor, which is what lets this promise a bound on its output.
LIMIT_CORRECTION_ROUNDS = 6
# Bounds on the scale search that hits a requested duration. Duration is not
# monotone in the scale, so the range is scanned before it is refined; a
# bisection alone reported 1.13x on a path that reaches 1.38x.
SCALE_SCAN_POINTS = 10
SCALE_REFINE_STEPS = 5
SCALE_SEARCH_FLOOR = 0.02
# Shortfall above which the cheap descending scan is abandoned for the full grid.
SCALE_SCAN_FALLBACK = 0.05
SCALE_SEARCH_TOLERANCE = 0.01


class RetimingError(ValueError):
    """A trajectory could not be produced under the requested mode."""


@dataclass
class RetimedTrajectory:
    mode: str
    times: list[float]
    positions: list[list[float]]
    velocities: list[list[float]]
    accelerations: list[list[float]]
    duration: float
    recorded_duration: float
    #: Largest deviation of the planned path from the recording, in rad.
    #: Non-zero in every mode: all of them filter the recording before they emit
    #: it. For the parameterized modes it also carries the chord error of the
    #: sparse path, so it bounds the whole geometric deviation rather than only
    #: the smoothing stage.
    path_deviation: float
    #: Achieved speed relative to the recording (2.0 = twice as fast).
    speed_achieved: float
    #: Peak |value| / limit over all joints; > 1.0 means a limit was exceeded.
    velocity_utilisation: float
    acceleration_utilisation: float
    notes: list[str] = field(default_factory=list)


def default_acceleration(max_velocity: Sequence[float]) -> list[float]:
    return [ACCELERATION_PER_VELOCITY * float(v) for v in max_velocity]


def _validate(times, positions, max_velocity, max_acceleration):
    t = np.asarray(times, dtype=float)
    q = np.asarray(positions, dtype=float)
    if t.ndim != 1 or q.ndim != 2:
        raise RetimingError("times must be 1-D and positions 2-D")
    if len(t) != len(q):
        raise RetimingError(f"{len(t)} times against {len(q)} position rows")
    if len(t) < 4:
        raise RetimingError("need at least four samples to fit a trajectory")
    if not np.all(np.diff(t) > 0.0):
        raise RetimingError("recorded times must be strictly increasing")
    if not np.all(np.isfinite(q)):
        raise RetimingError("recorded positions contain non-finite values")
    joints = q.shape[1]
    if len(max_velocity) != joints:
        raise RetimingError(
            f"max_velocity has {len(max_velocity)} entries, recording has {joints} joints"
        )
    if len(max_acceleration) != joints:
        raise RetimingError(
            f"max_acceleration has {len(max_acceleration)} entries, recording has {joints} joints"
        )
    for name, limits in (("max_velocity", max_velocity), ("max_acceleration", max_acceleration)):
        for index, value in enumerate(limits):
            if not (float(value) > 0.0) or not math.isfinite(float(value)):
                raise RetimingError(f"{name}[{index}] must be finite and > 0")
    return t - t[0], q


def _utilisation(values, limits):
    if len(values) == 0:
        return 0.0
    return float(np.max(np.abs(np.asarray(values)) / np.asarray(limits)))


def _uniform_resample(t, q, dt):
    """Linearly interpolate a recording onto a uniform grid of step ``dt``.

    The grid spans the recorded duration exactly, so the taught pace is
    preserved. Linear interpolation keeps every output pose a convex combination
    of two recorded ones, and the recorded endpoints land on the grid ends.

    A uniform grid is what makes the filter below a time-domain filter and the
    differences below true derivatives; on the recorded grid both are index
    operations over unequal intervals.
    """
    duration = float(t[-1])
    if duration <= 0.0 or not (dt > 0.0) or not math.isfinite(dt):
        return np.asarray(t, dtype=float), q.copy()
    # At least four points, so a central difference over the grid has something
    # to work with however coarse the step asked for was.
    steps = max(3, int(math.ceil(duration / dt - 1e-9)))
    grid = np.linspace(0.0, duration, steps + 1)
    resampled = np.stack(
        [np.interp(grid, t, q[:, joint]) for joint in range(q.shape[1])], axis=1
    )
    return grid, resampled


def _moving_average(q, t, window_sec):
    """Zero-phase centred moving average, width given in seconds.

    Not a fitted spline: a fit chooses one smoothness for the whole signal and
    reproduces whatever it did not smooth as derivative noise.

    The ends are extended by odd reflection through the endpoint
    (``q[-k] = 2*q[0] - q[k]``), which holds the window at full width and keeps
    the first and last samples exactly where they were. A window that shrinks at
    the edges instead leaves the outermost samples unfiltered, at 109 rad/s² of
    commanded acceleration against 5.8 over the rest of the same replay.

    ``t`` must be uniformly spaced: the width is converted to samples through the
    median spacing, which is the requested width only when every interval equals
    it.
    """
    if window_sec <= 0.0:
        return q.copy(), 0.0
    spacing = float(np.median(np.diff(t)))
    half = int(round(0.5 * window_sec / spacing)) if spacing > 0.0 else 0
    count = len(q)
    if half < 1 or count < 2:
        return q.copy(), 0.0
    half = min(half, count - 1)

    reflection = np.arange(half, 0, -1)
    padded = np.concatenate([
        2.0 * q[0] - q[reflection],
        q,
        2.0 * q[-1] - q[count - 1 - reflection[::-1]],
    ])
    window = 2 * half + 1
    cumulative = np.cumsum(np.vstack([np.zeros((1, q.shape[1])), padded]), axis=0)
    smoothed = (cumulative[window:] - cumulative[:-window]) / window
    return smoothed, float(np.max(np.abs(smoothed - q)))


def _finite_difference_states(t, q):
    """Central differences over ``t``. Endpoints are held at rest.

    Accurate to second order on a uniform grid and first order otherwise, so
    callers resample before differencing.
    """
    count = len(q)
    velocities = np.zeros_like(q)
    accelerations = np.zeros_like(q)
    for index in range(1, count - 1):
        before = t[index] - t[index - 1]
        after = t[index + 1] - t[index]
        if before <= 0.0 or after <= 0.0:
            continue
        velocities[index] = (q[index + 1] - q[index - 1]) / (before + after)
        # Taken from the positions, not from the velocities above: those are
        # pinned to zero at the ends, and differencing across that pin reports an
        # acceleration the motion does not contain.
        accelerations[index] = 2.0 * (
            (q[index + 1] - q[index]) / after - (q[index] - q[index - 1]) / before
        ) / (before + after)
    return velocities, accelerations


def _deviation_at_recorded_times(grid, planned, t, q):
    """Largest gap between the planned path and the recording, at its own times."""
    interpolated = np.stack(
        [np.interp(t, grid, planned[:, joint]) for joint in range(planned.shape[1])],
        axis=1,
    )
    return float(np.max(np.abs(interpolated - q)))


def _chord_waypoints(path, count):
    """Indices of ``count`` samples placed where the path bends.

    Douglas-Peucker refinement: start from the two endpoints and repeatedly
    split the segment whose farthest sample deviates most from its chord. The
    result bounds the geometric error of the sparse path directly, and spends
    waypoints on corners and reversals rather than on however long the arm
    happened to spend somewhere.

    Selecting by time or by sample index instead spends them on dwells, which a
    geometric path has no notion of, and measured 20-28% slower at full limits.
    """
    total = len(path)
    count = max(2, min(int(count), total))
    if count >= total:
        return np.arange(total)

    def worst(lo, hi):
        """Index between lo and hi farthest from the chord, and that distance."""
        if hi - lo < 2:
            return -1, 0.0
        anchor = path[lo]
        chord = path[hi] - anchor
        offsets = path[lo + 1: hi] - anchor
        length = float(np.dot(chord, chord))
        if length <= 0.0:
            distances = np.linalg.norm(offsets, axis=1)
        else:
            projected = np.outer(offsets @ chord / length, chord)
            distances = np.linalg.norm(offsets - projected, axis=1)
        index = int(np.argmax(distances))
        return lo + 1 + index, float(distances[index])

    chosen = [0, total - 1]
    segments = [(lo, hi, *worst(lo, hi)) for lo, hi in ((0, total - 1),)]
    while len(chosen) < count:
        segments.sort(key=lambda segment: segment[3])
        lo, hi, split, error = segments.pop()
        if split < 0 or error <= 0.0:
            break
        chosen.append(split)
        segments.extend([
            (lo, split, *worst(lo, split)),
            (split, hi, *worst(split, hi)),
        ])
    return np.array(sorted(chosen))


def _geometric_path(t, q, *, smoothing_window_sec, waypoints, resample_dt):
    """Resample, smooth in the time domain, then pick a sparse geometric path.

    The smoothing is a time-domain filter only on a uniform grid, so the
    resample comes first. Every step is an average or a linear interpolation of
    recorded samples, so each waypoint stays a convex combination of them and
    the path cannot leave the range the recording covers — the guarantee
    `_assert_within_recorded_range` checks.
    """
    grid, resampled = _uniform_resample(t, q, resample_dt)
    smoothed, _ = _moving_average(resampled, grid, smoothing_window_sec)
    path = smoothed[_chord_waypoints(smoothed, waypoints)]
    # Both stages move the path away from the recording, and only the first was
    # ever reported: the sparse path cuts whatever its chords cut. Their sum
    # bounds the total, which is what `_assert_within_recorded_range` is handed.
    deviation = _deviation_at_recorded_times(grid, smoothed, t, q) + _chord_error(path, smoothed)
    return path, deviation


def _chord_error(sparse, dense):
    """Farthest a dense sample lies from the sparse polyline through it."""
    worst = np.zeros(len(dense))
    worst[:] = np.inf
    for start, end in zip(sparse[:-1], sparse[1:]):
        segment = end - start
        offsets = dense - start
        length = float(segment @ segment)
        if length <= 0.0:
            distances = np.linalg.norm(offsets, axis=1)
        else:
            ratio = np.clip(offsets @ segment / length, 0.0, 1.0)
            distances = np.linalg.norm(offsets - np.outer(ratio, segment), axis=1)
        worst = np.minimum(worst, distances)
    return float(worst.max())


def _assert_within_recorded_range(planned, recorded, tolerance):
    """Refuse a plan that commands a pose outside the taught envelope.

    The parameterization only re-times a path, so its output belongs inside the
    recording's per-joint range. Anything else is a defect in how the path was
    built, and the arm is the wrong place to discover it.
    """
    lower = recorded.min(axis=0) - tolerance
    upper = recorded.max(axis=0) + tolerance
    below = lower - planned.min(axis=0)
    above = planned.max(axis=0) - upper
    excursion = np.maximum(below, above)
    worst = int(np.argmax(excursion))
    if excursion[worst] > 0.0:
        raise RetimingError(
            f"planned path leaves the recorded range on joint {worst + 1} by "
            f"{excursion[worst]:.4f} rad (recorded "
            f"[{recorded[:, worst].min():+.4f}, {recorded[:, worst].max():+.4f}], "
            f"planned [{planned[:, worst].min():+.4f}, {planned[:, worst].max():+.4f}]); "
            "refusing to command it"
        )


def _run_totg(path, max_velocity, max_acceleration, scale, blend_tolerance, resample_dt):
    """Parameterize at ``scale`` of the limits, returning output that honours them.

    The limits are corrected against the peak actually sampled rather than by a
    fixed derating, because the overshoot at blend junctions depends on the blend
    radius and so on the path.
    """
    target_v = [scale * float(v) for v in max_velocity]
    target_a = [scale * float(a) for a in max_acceleration]
    rows = [list(row) for row in path]
    correction = 1.0
    result = None

    for _ in range(LIMIT_CORRECTION_ROUNDS):
        candidate = retime_path(
            rows,
            [correction * v for v in target_v],
            [correction * a for a in target_a],
            max_deviation=blend_tolerance,
            resample_dt=resample_dt,
        )
        if not candidate["valid"]:
            return result
        result = candidate
        worst = max(
            _utilisation(candidate["velocities"], target_v),
            _utilisation(candidate["accelerations"], target_a),
        )
        if worst <= 1.0:
            return result
        # A small margin, so a converging step does not land exactly on the edge
        # and oscillate across it.
        correction /= worst * 1.02
    return result


def _search_scale_for_duration(path, max_velocity, max_acceleration, target,
                               blend_tolerance, resample_dt):
    """Find the limit scale whose parameterization lands closest to ``target``.

    **Duration is not monotone in the scale.** `_run_totg` corrects the limits
    against the peak it actually samples, and that correction depends on the
    blend radii, which change with the scale — measured non-monotone over
    0.49-0.57 (3.54 s, then 4.09 s at higher limits). A bisection therefore
    converges onto whichever local branch it started on and reports its own
    dead end as the hardware's: it returned 1.13x on a path that reaches 1.38x.

    So: scan the scale range, then refine either side of the best sample. Only
    slow-side candidates are ever returned — on a taught motion an unrequested
    speed-up is the dangerous direction.
    """
    fastest = _run_totg(path, max_velocity, max_acceleration, 1.0,
                        blend_tolerance, resample_dt)
    if fastest is None:
        return None, None
    if fastest["duration"] >= target:
        return fastest, 1.0

    def best_slow_side(pool):
        slow_side = [entry for entry in pool if entry[1]["duration"] >= target]
        return min(slow_side, key=lambda entry: entry[1]["duration"]) if slow_side else None

    def close_enough(entry):
        return entry is not None and entry[1]["duration"] - target <= SCALE_SEARCH_TOLERANCE * target

    # Descending, because a requested tempo usually sits just under the ceiling
    # and the answer is then the first grid point tried. The scan stops at the
    # first slow-side hit; refinement runs between it and the faster neighbour
    # above, which is already evaluated and brackets the crossing.
    evaluated: list[tuple[float, dict]] = [(1.0, fastest)]
    grid = list(np.geomspace(SCALE_SEARCH_FLOOR, 1.0, SCALE_SCAN_POINTS)[:-1])[::-1]
    low, high = SCALE_SEARCH_FLOOR, 1.0
    best = None
    for scale in grid:
        candidate = _run_totg(path, max_velocity, max_acceleration, float(scale),
                              blend_tolerance, resample_dt)
        if candidate is None:
            high = float(scale)
            continue
        evaluated.append((float(scale), candidate))
        if candidate["duration"] >= target:
            best = best_slow_side(evaluated)
            low = float(scale)
            break
        high = float(scale)

    if best is None:
        return fastest, 1.0
    if close_enough(best):
        return best[1], best[0]

    # The early exit assumes the crossing is where the descent found it, which
    # only holds while duration falls with the scale. When the bracket it
    # produced is far off the target, the crossing is somewhere the descent
    # stepped over: evaluate the rest of the grid before refining.
    if best[1]["duration"] - target > SCALE_SCAN_FALLBACK * target:
        seen = {round(scale, 9) for scale, _ in evaluated}
        for scale in grid:
            if round(float(scale), 9) in seen:
                continue
            candidate = _run_totg(path, max_velocity, max_acceleration, float(scale),
                                  blend_tolerance, resample_dt)
            if candidate is not None:
                evaluated.append((float(scale), candidate))
        best = best_slow_side(evaluated) or best
        if close_enough(best):
            return best[1], best[0]
        scales = sorted(scale for scale, _ in evaluated)
        index = scales.index(best[0])
        low = scales[index - 1] if index > 0 else SCALE_SEARCH_FLOOR
        high = scales[index + 1] if index + 1 < len(scales) else 1.0
    for _ in range(SCALE_REFINE_STEPS):
        if best[1]["duration"] - target <= SCALE_SEARCH_TOLERANCE * target:
            break
        probe = 0.5 * (low + high)
        candidate = _run_totg(path, max_velocity, max_acceleration, probe,
                              blend_tolerance, resample_dt)
        if candidate is None:
            high = probe
            continue
        evaluated.append((probe, candidate))
        if candidate["duration"] >= target:
            low = probe
        else:
            high = probe
        best = best_slow_side(evaluated) or best
    return best[1], best[0]



def retime(
    times: Sequence[float],
    positions: Sequence[Sequence[float]],
    mode: str = AS_RECORDED,
    *,
    max_velocity: Sequence[float] | None = None,
    max_acceleration: Sequence[float] | None = None,
    speed_scale: float = 1.0,
    smoothing_window_sec: float = DEFAULT_SMOOTHING_WINDOW_SEC,
    waypoints: int = DEFAULT_WAYPOINTS,
    blend_tolerance: float = DEFAULT_BLEND_TOLERANCE,
    resample_dt: float = DEFAULT_RESAMPLE_DT,
) -> RetimedTrajectory:
    """Plan an executable trajectory from a recording.

    ``max_velocity`` defaults to the manufacturer's joint speeds for one Nero
    arm; a duo recording must pass its own 14-entry limits.
    """
    if mode not in MODES:
        raise RetimingError(f"unknown mode '{mode}'; expected one of {', '.join(MODES)}")
    if max_velocity is None:
        max_velocity = list(NERO_MAX_VELOCITY)
    if max_acceleration is None:
        max_acceleration = default_acceleration(max_velocity)
    if mode in (SPEED_SCALE, TEMPO_SCALE) and not (
        speed_scale > 0.0 and math.isfinite(speed_scale)
    ):
        raise RetimingError("speed_scale must be finite and > 0")

    t, q = _validate(times, positions, max_velocity, max_acceleration)
    recorded_duration = float(t[-1])
    notes: list[str] = []

    if mode in TIMING_PRESERVING:
        # The floor applies to all three: the recorded grid is uneven, and the
        # controller's linear interpolation turns an uneven knot into a step in
        # commanded velocity.
        window = (
            RECONSTRUCTION_WINDOW_SEC
            if mode == AS_RECORDED
            else max(RECONSTRUCTION_WINDOW_SEC, smoothing_window_sec)
        )
        # Everything happens in the *output* time base, so the window and the
        # grid mean the same thing whatever the tempo. Scaling the axis is the
        # whole of the operation: relative dwells, reversals and the phase
        # between two arms are ratios, and a ratio survives it.
        tempo = speed_scale if mode == TEMPO_SCALE else 1.0
        scaled = t / tempo
        tt, resampled = _uniform_resample(scaled, q, resample_dt)
        pos, _ = _moving_average(resampled, tt, window)
        deviation = _deviation_at_recorded_times(tt, pos, scaled, q)
        vel, acc = _finite_difference_states(tt, pos)
        recorded_spacing = float(np.median(np.diff(t)))
        notes.append(
            f"taught timing kept; resampled from {1.0 / recorded_spacing:.0f} Hz median "
            f"to {1.0 / resample_dt:.0f} Hz uniform, {window:.2f}s moving-average window"
        )
        if mode == TEMPO_SCALE:
            notes.append(
                f"taught timing scaled by {tempo:.2f}x, not re-planned; the window "
                "is in replay seconds, so a faster tempo filters more of the path"
            )
        if mode == AS_RECORDED and smoothing_window_sec > RECONSTRUCTION_WINDOW_SEC:
            notes.append(
                f"as_recorded ignores the {smoothing_window_sec:.2f}s window; "
                f"use 'smooth' to widen it"
            )
        velocity_use = _utilisation(vel, max_velocity)
        acceleration_use = _utilisation(acc, max_acceleration)
        if velocity_use > 1.0:
            notes.append(
                f"commanded velocity reaches {velocity_use:.2f}x the limit; "
                + (
                    f"a tempo below {tempo / velocity_use:.2f}x would bring it under"
                    if mode == TEMPO_SCALE
                    else "the recording, not the plan, is what exceeds it"
                )
            )
        return RetimedTrajectory(
            mode=mode,
            times=[float(v) for v in tt],
            positions=pos.tolist(),
            velocities=vel.tolist(),
            accelerations=acc.tolist(),
            duration=recorded_duration / tempo,
            recorded_duration=recorded_duration,
            path_deviation=deviation,
            speed_achieved=tempo,
            velocity_utilisation=velocity_use,
            acceleration_utilisation=acceleration_use,
            notes=notes,
        )

    path, deviation = _geometric_path(
        t, q,
        smoothing_window_sec=smoothing_window_sec,
        waypoints=waypoints,
        resample_dt=resample_dt,
    )
    if mode == MAXIMIZE_SPEED:
        result = _run_totg(path, max_velocity, max_acceleration, 1.0,
                           blend_tolerance, resample_dt)
        scale = 1.0
    else:
        target = recorded_duration / speed_scale
        result, scale = _search_scale_for_duration(
            path, max_velocity, max_acceleration, target, blend_tolerance, resample_dt
        )
        if result is not None:
            achieved = recorded_duration / result["duration"]
            if achieved < speed_scale * 0.99:
                notes.append(
                    f"requested {speed_scale:.2f}x, reached {achieved:.2f}x; the search "
                    "returns only the slow side, and duration is not monotone in the "
                    "limit scale, so this is the best it found rather than a hardware "
                    "ceiling — 'maximize_speed' is what measures that"
                )
    if result is None:
        raise RetimingError(
            "time parameterization failed on this path — it is usually a path with "
            "near-duplicate waypoints; raise smoothing or lower the waypoint count"
        )

    _assert_within_recorded_range(
        np.asarray(result["positions"], dtype=float), q, deviation + PATH_EXCURSION_TOLERANCE
    )
    notes.append(f"taught timing discarded; limits used at {scale:.3f} of maximum")
    return RetimedTrajectory(
        mode=mode,
        times=list(result["times"]),
        positions=[list(row) for row in result["positions"]],
        velocities=[list(row) for row in result["velocities"]],
        accelerations=[list(row) for row in result["accelerations"]],
        duration=float(result["duration"]),
        recorded_duration=recorded_duration,
        path_deviation=deviation,
        speed_achieved=recorded_duration / float(result["duration"]),
        velocity_utilisation=_utilisation(result["velocities"], max_velocity),
        acceleration_utilisation=_utilisation(result["accelerations"], max_acceleration),
        notes=notes,
    )


__all__ = [
    "AS_RECORDED",
    "TEMPO_SCALE",
    "TIMING_PRESERVING",
    "MAXIMIZE_SPEED",
    "MODES",
    "DEFAULT_RESAMPLE_DT",
    "DEFAULT_SMOOTHING_WINDOW_SEC",
    "NERO_MAX_VELOCITY",
    "RECONSTRUCTION_WINDOW_SEC",
    "SMOOTH",
    "SPEED_SCALE",
    "RetimedTrajectory",
    "RetimingError",
    "default_acceleration",
    "retime",
]
