# Replaying a taught motion: what each tool can and cannot express

date: 2026-08-24
platform: Jetson AGX Orin, both arms, MoveIt 2.5.9
method: MoveIt's time-optimal path parameterization (TOTG, Kunz & Stilman) via a
pybind11 module in `agx_arm_retiming`, plus scipy splines; measured against real
14-joint duo and 7-joint single-arm teach recordings

## Why this exists

A recorded replay was dispatched with positions and times only, so the MIT
feedforward velocity was zero and the motion was pure position chasing. Fixing
that raised a question the codebase had never answered: whether a replay should
keep the timing it was taught with, or traverse the same path on a new one. The
answer is that both are needed, and that no single tool provides them.

> **Superseded in part, 2026-08-24.** The claim below that `as_recorded` keeps
> the path *exactly* no longer holds, and the "0.30 s window brought every
> recording under the limit" figures were measured through a filter applied in
> sample space on an uneven grid. Every mode now resamples onto a uniform grid
> before it filters or differentiates, and `as_recorded` carries a 0.06 s filter
> floor. See `teach_replay_timebase.md`. Everything about TOTG itself stands.

## The split is forced by the tools, not chosen

| mode | tool | keeps |
| --- | --- | --- |
| `as_recorded` | uniform resample, 0.06 s window, central differences | timing exactly, path to ~0.05 rad |
| `smooth` | same, operator-chosen window ≥ 0.06 s | timing exactly |
| `speed_scale` | TOTG, limit scale searched for the target duration | path geometry |
| `maximize_speed` | TOTG at full limits | path geometry |

No mode replays raw samples. The controller interpolates linearly between
trajectory points, so a knot at an uneven timestamp is a step in commanded
velocity whatever the mode intends.

**A path parameterization computes the timing; the taught timing is exactly what
it discards.** Anything purely temporal in a recording — a dwell, a deliberately
slow pour — does not survive TOTG, because a dwell is zero path progress and the
geometric path has no notion of it. That is a property of the method, not a
setting, and it is why the timing-preserving modes do not go through it at all.

**The timing-preserving pair fits nothing, deliberately.** A fitted spline has to
choose one smoothness for the whole signal and reproduces whatever it did not
smooth as derivative noise; measured on the same recording it was roughly twice
as rough as central differences over the raw samples (acceleration 148x the limit
against 74x). A moving-average window is blunt and local, and its width is one
number an operator turns rather than a fit parameter.

The reverse of path fidelity still holds: replaying the recording faithfully
replays its noise. That is why `as_recorded` filters at all — see the floor in
`teach_replay_timebase.md`.

## What the binding actually needs

TOTG's `Path` and `Trajectory` are **model-free** — plain joint vectors and
explicit per-joint limits, no RobotModel, no URDF, no planning group — so the
binding is a pure function and needs no node in the graph. `getPosition`,
`getVelocity` and `getAcceleration` are analytic at any instant, which is also
the resampling step: output density costs message size and never changes the
motion.

`max_deviation = 0` keeps the path exactly through its waypoints. That is the
geometric guarantee a critical replay needs, built into the tool.

Humble ships no `moveit_py`, and every runtime package here is `ament_python`,
which is the whole reason `agx_arm_retiming` is an `ament_cmake` package.

## TOTG will not take a dense recording

Fed the raw capture directly, it fails in both directions:

| input | result |
| --- | --- |
| 1737 raw waypoints, `max_deviation=0` | 5.6 s, but sampled acceleration to 4434 rad/s² against a limit of 5 |
| 1737 raw waypoints, `max_deviation>0` | 24–53 s — *slower than the 13 s recording* |
| smoothed to 60 waypoints, `max_deviation=0` | 3.34 s, velocity and acceleration exactly at their limits |

At zero deviation every sample is a corner and curvature is a delta there; with
blending, the radii in the noise become so small that the curvature limit brakes
everything. **The smoothing stage is a prerequisite for the retiming stage, not
an alternative to it.** TOTG is built for planner output — a handful of
waypoints — not for hundreds of noisy samples.

Waypoint count matters more than expected: 40 beat 80 and 160 consistently.

## Two properties worth knowing before tuning

**TOTG is bang-bang in acceleration by construction.** Across an entire
parameter sweep the peak jerk was identical at 3927 = `2·a_max/Δt` — the
acceleration switches instantly between ±a_max at every switching point. It
cannot bound jerk at all; that is what Ruckig (C++ only here) or a quintic
spline would be for. Do not tune parameters hoping to reduce it.

**Its limits bound the profile along path segments, not at blend junctions.**
Curvature jumps where a straight segment meets a blend arc, and the sampled
acceleration overshoots there — 1.3x with a fixed derating factor. The limits
handed to it are therefore corrected against the peak actually sampled, in a
bounded loop, which is what lets the result promise a bound on its own output.

**Acceleration binds, velocity does not.** Across all measurements velocity
stayed at 0.5–0.8 of its limit while acceleration saturated. With no
manufacturer acceleration figure for these joints, `a_max = 2.5 · v_max` is
carried as an explicitly conservative stand-in — so that stand-in, not the
hardware, is what currently sets replay speed.

## Measured on real recordings

Fresh duo capture, 26.45 s taught:

| mode | duration | speed | v/limit | a/limit | path deviation |
| --- | --- | --- | --- | --- | --- |
| `as_recorded` | 26.45 s | 1.00 | 1.61 | 74 | 0.0000 |
| `smooth` 0.10 s | 26.45 s | 1.00 | 0.77 | 8.3 | 0.0488 |
| `smooth` 0.25 s | 26.45 s | 1.00 | 0.42 | 3.5 | 0.0903 |
| `speed_scale 1.5` | 18.16 s | 1.46 | 0.30 | 0.31 | 0.0368 |
| `maximize_speed` | 10.45 s | 2.53 | 0.58 | 0.97 | 0.0368 |

`speed_scale` resolves to the **slow side** of the requested multiple: on a
taught motion an unrequested speed-up is the dangerous direction. Requesting
more than the limits allow returns the fastest feasible plan and says by how
much it fell short.

For the timing-preserving pair, **velocity utilisation is the number that
matters**: velocity is commanded and the controller clamps it, while acceleration
is never commanded — the MIT frame carries position, velocity, kp, kd and torque.
The acceleration figure describes how rough the recording is, not how badly the
plan will execute.

The velocity-utilisation figures previously recorded here (`as_recorded`
0.30-1.75, `smooth` 0.10 s 0.23-1.24) were taken before the uniform resample and
are superseded. On the two recordings re-measured 2026-08-24, `as_recorded` moved
from 1.75 to 0.84 on the right arm — the excess was the sample-to-sample noise the
floor now removes, not the taught motion. What still holds is the direction: a
wider window lowers utilisation and raises path deviation.

## Open

- the acceleration limit is a stand-in (`2.5 · v_max`), and it is the binding
  constraint for the TOTG modes. The velocity limits are resolved: the manual,
  `joint_limits.yaml` and the MIT controller now all declare 3.14 rad/s on J1-J3
  and 3.93 on J4-J7
- the TOTG defaults (`smoothing 2e-5`, `40 waypoints`, `blend 0.01`) are tuned on
  a small number of recordings and have not been swept against a clean capture
- the moving-average window is independent of them on purpose: one feeds the
  parameterization, the other is the filter width an operator turns
