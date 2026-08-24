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

## The split is forced by the tools, not chosen

| mode | tool | keeps |
| --- | --- | --- |
| `as_recorded` | interpolating spline over recorded `(t, q)` | path exactly, timing exactly |
| `smooth` | approximating spline, declared deviation | timing exactly |
| `speed_scale` | TOTG, limit scale searched for the target duration | path geometry |
| `maximize_speed` | TOTG at full limits | path geometry |

**A path parameterization computes the timing; the taught timing is exactly what
it discards.** Anything purely temporal in a recording — a dwell, a deliberately
slow pour — does not survive TOTG, because a dwell is zero path progress and the
geometric path has no notion of it. That is a property of the method, not a
setting, and it is why the timing-preserving modes use a spline instead.

The reverse also holds: an interpolating spline reproduces the recording
faithfully *including its noise*, so `as_recorded` is only as executable as the
capture is clean.

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
| `as_recorded` | 26.45 s | 1.00 | 2.61 | 148 | 0.0000 |
| `smooth` | 26.45 s | 1.00 | 0.93 | 9.04 | 0.0368 |
| `speed_scale 1.5` | 18.16 s | 1.46 | 0.30 | 0.31 | 0.0368 |
| `maximize_speed` | 10.45 s | 2.53 | 0.58 | 0.97 | 0.0368 |

`speed_scale` resolves to the **slow side** of the requested multiple: on a
taught motion an unrequested speed-up is the dangerous direction. Requesting
more than the limits allow returns the fastest feasible plan and says by how
much it fell short.

`as_recorded` exceeded the limits on every recording measured so far, even ones
captured at 95 real updates per second. The residual sample noise is enough for
an interpolating spline to derive unusable accelerations from it. A light
approximating fit that barely moves the path is the missing piece; smoothing
tuned for the TOTG path is too aggressive for the timing-preserving one.

## Open

- one acceleration limit is a stand-in, and it is the binding constraint
- `joint_limits.yaml` declares 5.0 rad/s where the manufacturer says 3.14 (J1-J3)
  and 3.93 (J4-J7), and the MIT controller clamps at 2.0; three numbers for one
  quantity, and TOTG output above the controller's clamp is unexecutable
- the defaults (`smoothing 2e-5`, `40 waypoints`, `blend 0.01`) are tuned on a
  small number of recordings and have not been swept against a clean capture
