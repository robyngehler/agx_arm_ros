"""Turn a recording into an executable trajectory, under one of four modes.

The modes split by what the underlying tool can express, not by preference:

* ``as_recorded`` and ``smooth`` keep the taught timing and fit nothing. The
  recorded samples are replayed at their recorded times, with derivatives taken
  as central differences and, for ``smooth``, a moving-average window over the
  positions first. A motion whose dynamics were taught deliberately — a dwell, a
  slow pour — therefore survives, and so does the path.
* ``speed_scale`` and ``maximize_speed`` hand the geometric path to MoveIt's
  time-optimal parameterization, which computes its own timing and therefore
  discards anything purely temporal in the recording.

Path fidelity belongs to the first pair, and it is why they are blunt: a fitted
spline has to choose a smoothness for the whole signal and reproduces whatever it
did not smooth as derivative noise. The second pair fits before it re-times
because the parameterization needs it: fed a dense recording directly,
it produces either curvature spikes at every sample or blend radii so small the
result is slower than the recording
(``docs/sprint_refactor/reference/trajectory_retiming.md``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from scipy.interpolate import UnivariateSpline

from agx_arm_retiming._totg import retime_path

AS_RECORDED = "as_recorded"
SMOOTH = "smooth"
SPEED_SCALE = "speed_scale"
MAXIMIZE_SPEED = "maximize_speed"
MODES = (AS_RECORDED, SMOOTH, SPEED_SCALE, MAXIMIZE_SPEED)

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

# Defaults tuned on one 14-joint duo recording; see the reference note for the
# sweep and its limits as evidence.
DEFAULT_WAYPOINTS = 40
DEFAULT_BLEND_TOLERANCE = 0.01
DEFAULT_SMOOTHING = 2e-5
# The blunt modes' window, in seconds and independent of the fit above: one is a
# spline smoothness feeding the parameterization, the other is a filter width an
# operator turns. Tying them would make a replay's path depend on a number
# chosen for a different stage.
DEFAULT_SMOOTHING_WINDOW_SEC = 0.10
DEFAULT_RESAMPLE_DT = 0.005
# TOTG bounds the profile along its path segments, but curvature jumps
# discontinuously where a straight segment meets a blend arc, so the sampled
# acceleration overshoots there — measured at 1.3x with a fixed derating. The
# limits handed to it are corrected against the sampled peak instead of by a
# guessed factor, which is what lets this promise a bound on its output.
LIMIT_CORRECTION_ROUNDS = 6


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
    #: Largest deviation of the planned path from the recording, in rad. Zero by
    #: construction for ``as_recorded``.
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


def _moving_average(q, t, window_sec):
    """Zero-phase centred moving average over the recorded samples.

    Deliberately not a fitted spline. A fit has to choose a smoothness for the
    whole signal and then reproduces whatever it did not smooth as derivative
    noise; a window is blunt, local, and its effect is one number an operator can
    turn. The half-window shrinks symmetrically at the edges, so the first and
    last samples stay exactly where they were recorded.
    """
    if window_sec <= 0.0:
        return q.copy(), 0.0
    spacing = float(np.median(np.diff(t)))
    half = int(round(0.5 * window_sec / spacing)) if spacing > 0.0 else 0
    if half < 1:
        return q.copy(), 0.0

    count = len(q)
    smoothed = np.empty_like(q)
    for index in range(count):
        reach = min(half, index, count - 1 - index)
        smoothed[index] = (
            q[index - reach: index + reach + 1].mean(axis=0) if reach else q[index]
        )
    return smoothed, float(np.max(np.abs(smoothed - q)))


def _finite_difference_states(t, q):
    """Central differences over the recorded, possibly uneven spacing.

    Sound only because repeats are removed before a recording is stored: a
    difference taken across a repeated sample alternates between zero and twice
    the true value. Endpoints are held at rest, where the arm was.
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
        # Straight from the positions, not from the velocities above: those are
        # pinned to zero at the ends, and differencing across that pin reports an
        # acceleration spike the motion does not contain.
        accelerations[index] = 2.0 * (
            (q[index + 1] - q[index]) / after - (q[index] - q[index - 1]) / before
        ) / (before + after)
    return velocities, accelerations


def _geometric_path(t, q, *, smoothing, waypoints, degree):
    """Smooth, then resample to a sparse waypoint set for the parameterization."""
    joints = q.shape[1]
    splines = [
        UnivariateSpline(t, q[:, j], s=smoothing * len(t), k=degree) for j in range(joints)
    ]
    tt = np.linspace(0.0, float(t[-1]), waypoints)
    path = np.column_stack([s(tt) for s in splines])
    deviation = float(np.max(np.abs(np.column_stack([s(t) for s in splines]) - q)))
    return path, deviation


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

    Duration falls monotonically as the limits rise, so a bisection on the scale
    converges. Where even full limits cannot reach the target the fastest
    feasible result is returned and the caller is told by how much it fell short.
    """
    fastest = _run_totg(path, max_velocity, max_acceleration, 1.0,
                        blend_tolerance, resample_dt)
    if fastest is None:
        return None, None
    if fastest["duration"] >= target:
        return fastest, 1.0

    # Never return something faster than was asked for: on a taught motion an
    # unrequested speed-up is the dangerous direction. Among everything
    # evaluated, take the candidate closest to the target from the slow side,
    # and fall back to the fastest feasible plan only if nothing reaches it.
    low, high = 1e-3, 1.0
    evaluated: list[tuple[float, dict]] = []
    for _ in range(24):
        mid = 0.5 * (low + high)
        candidate = _run_totg(path, max_velocity, max_acceleration, mid,
                              blend_tolerance, resample_dt)
        if candidate is None:
            low = mid
            continue
        evaluated.append((mid, candidate))
        if abs(candidate["duration"] - target) < 1e-3:
            return candidate, mid
        if candidate["duration"] > target:
            low = mid
        else:
            high = mid

    at_or_slower = [(s, c) for s, c in evaluated if c["duration"] >= target]
    if at_or_slower:
        scale, candidate = min(at_or_slower, key=lambda item: item[1]["duration"])
        return candidate, scale
    return fastest, 1.0


def retime(
    times: Sequence[float],
    positions: Sequence[Sequence[float]],
    mode: str = AS_RECORDED,
    *,
    max_velocity: Sequence[float] | None = None,
    max_acceleration: Sequence[float] | None = None,
    speed_scale: float = 1.0,
    smoothing: float = DEFAULT_SMOOTHING,
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
    if mode == SPEED_SCALE and not (speed_scale > 0.0 and math.isfinite(speed_scale)):
        raise RetimingError("speed_scale must be finite and > 0")

    t, q = _validate(times, positions, max_velocity, max_acceleration)
    recorded_duration = float(t[-1])
    notes: list[str] = []

    if mode in (AS_RECORDED, SMOOTH):
        window = 0.0 if mode == AS_RECORDED else smoothing_window_sec
        pos, deviation = _moving_average(q, t, window)
        vel, acc = _finite_difference_states(t, pos)
        tt = t
        if mode == AS_RECORDED:
            notes.append("every recorded sample replayed unchanged, at its recorded time")
        else:
            notes.append(f"{window:.2f}s moving-average window, no fit")
        velocity_use = _utilisation(vel, max_velocity)
        acceleration_use = _utilisation(acc, max_acceleration)
        if velocity_use > 1.0:
            notes.append(
                f"recorded motion already demands {velocity_use:.2f}x the velocity "
                "limit; the recording, not the plan, is what exceeds it"
            )
        return RetimedTrajectory(
            mode=mode,
            times=[float(v) for v in tt],
            positions=pos.tolist(),
            velocities=vel.tolist(),
            accelerations=acc.tolist(),
            duration=recorded_duration,
            recorded_duration=recorded_duration,
            path_deviation=deviation,
            speed_achieved=1.0,
            velocity_utilisation=velocity_use,
            acceleration_utilisation=acceleration_use,
            notes=notes,
        )

    path, deviation = _geometric_path(
        t, q, smoothing=max(smoothing, 1e-9), waypoints=waypoints, degree=5
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
                    f"requested {speed_scale:.2f}x but the limits allow {achieved:.2f}x; "
                    "returned the fastest feasible plan"
                )
    if result is None:
        raise RetimingError(
            "time parameterization failed on this path — it is usually a path with "
            "near-duplicate waypoints; raise smoothing or lower the waypoint count"
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
    "MAXIMIZE_SPEED",
    "MODES",
    "DEFAULT_SMOOTHING_WINDOW_SEC",
    "NERO_MAX_VELOCITY",
    "SMOOTH",
    "SPEED_SCALE",
    "RetimedTrajectory",
    "RetimingError",
    "default_acceleration",
    "retime",
]
