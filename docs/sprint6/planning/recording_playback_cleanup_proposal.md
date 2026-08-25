# Recording / Playback Pipeline Cleanup Proposal

> **Status 2026-08-25.** Worked through in the recommended order. Detail and
> measurements: `docs/sprint_refactor/reference/teach_replay_timebase.md` and
> `trajectory_retiming.md`.
>
> | item | state |
> | --- | --- |
> | P0.1 pre-motion trim + pre-roll | **done** — `--pre-roll-sec` 0.25 s, one cut instant for all arms |
> | P0.2 source-level freshness | **investigated, not implemented** — the per-group timestamps already exist in the vendor driver and are discarded during assembly; see below |
> | P0 `_geometric_path` order | **done** — resample, smooth, then select |
> | P0 geometric waypoint selection | **done** — chord error (Douglas-Peucker), default 80 |
> | P1.A `tempo_scale` | **done** — scales the clock, never calls TOTG |
> | P1.B TOTG for the geometric modes | unchanged, now explicitly labelled in the mode table |
> | P1 no geometric inflation | held — nothing inflates the path; density is the lever |
> | P1 Ruckig evaluation | **deferred by the proposal's own gate** — `tempo_scale` first |
> | P2 acceleration / jerk limits | unchanged — still an explicit stand-in, still the binding constraint |
> | Validation 1, 2, 4, 5 | **covered by package tests** |
> | Validation 3 (hardware capture) | **done 2026-08-25** — duo teach+replay over all five modes |
> | *(not in the proposal)* activity/catalogue path | **partly closed** — velocities and chord-error waypoints now shared; retiming modes and taught density are not |
>
> **P0.2 finding.** `get_joint_angles()` in
> `pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py` reassembles the
> 7-joint vector from four separately cached CAN frames (`joint_12`, `joint_34`,
> `joint_56`, `joint_7`), each carrying its own `timestamp`, then collapses them
> to one: `self._joint_angles.timestamp = joint_angles.timestamp`, the last group
> present in the if-chain. The per-group freshness the proposal asks for already
> exists and is thrown away at the point a consumer could use it. Publishing it
> alongside `feedback/joint_states` would make `reconstruct_stalled_joints()` a
> read instead of an inference. Arm-bridge change, needs hardware validation.
>
> **One correction to the proposal.** P0 says to resample before smoothing the
> geometric path, which is right, and warns against uniformly-spaced waypoints,
> which is also right — but the first attempt did exactly that and measured
> 20-28% slower at full limits. Chord error is what resolves it: waypoints where
> the path bends, not where the clock ran.

## Goal

Improve recorded-trajectory playback quality without introducing custom trajectory-optimization algorithms where established libraries already exist.

The main goals are:

- remove artefacts caused by stale / uneven arm feedback,
- preserve natural taught motion around reversals when requested,
- keep time-optimal retiming as a separate, explicit mode,
- avoid invented acceleration / jerk limits until hardware testing provides useful values.

---

## P0 — Recording start and feedback quality

### 1. Remove pre-motion dead time

Do not persist the complete stationary interval between arming the recorder and the first actual movement.

Preferred implementation:

- keep capture armed immediately,
- maintain a short rolling pre-roll buffer,
- detect the first motion using the existing movement threshold,
- commit only a small pre-roll plus all samples from the detected motion onward,
- re-base `time_from_start` to the retained first sample.

A short pre-roll is preferable to starting exactly at the first threshold crossing because it avoids clipping the physical beginning of the motion.

Apply the same rule consistently to single-arm and duo recordings.

### 2. Keep improving feedback freshness at the source

The current callback-driven recorder and stalled-joint reconstruction are good recovery mechanisms, but post-processing should not be the primary source of truth for whether a joint value was fresh.

Investigate adding per-frame / per-joint freshness information in `agx_ctrl_node` or the lower driver layer, e.g.:

- generation / sequence counter,
- source timestamp,
- freshness mask,
- or publication only after a coherent set of position CAN frames has advanced.

Keep `reconstruct_stalled_joints()` as a bounded fallback, not as the long-term substitute for source freshness.

Use new hardware captures from both arms to validate this. The older `Wave_Left_V02` and `Wave_Right_V01` recordings should remain regression fixtures because they contain the failure pattern.

---

## P0 — Fix geometric-path preprocessing

`_moving_average()` explicitly assumes uniformly spaced time samples, while `_geometric_path()` currently calls it directly on the irregular recorded grid.

Fix the processing order:

1. reconstruct / resample the recording to a uniform grid,
2. apply time-domain smoothing on that uniform grid,
3. extract the sparse geometric path afterwards.

Do **not** simply choose the sparse path as uniformly spaced time samples, because this wastes geometric waypoints on dwells.

Instead, use established geometric criteria for waypoint selection, e.g.:

- joint-space arc length,
- maximum interpolation / chord error,
- local curvature,
- mandatory preservation of extrema / reversal regions.

Add a regression test with deliberately irregular 10/20/30 ms input spacing and verify that the cleaned geometric path is approximately invariant to the sampling pattern.

---

## P1 — Separate playback semantics clearly

The current modes should represent two fundamentally different operations.

### A. Timing-preserving playback: add `tempo_scale`

Purpose:

> Replay the taught motion faster or slower while preserving its local timing structure.

Implementation should stay simple:

1. use the cleaned, uniformly reconstructed trajectory,
2. scale timestamps by the requested factor:
   `t_new = t_recorded / speed_factor`,
3. scale or recompute velocity / acceleration consistently,
4. resample to the controller output rate,
5. execute through the existing `FollowJointTrajectory` path.

This preserves:

- relative slow-down before reversals,
- natural reversal timing,
- pauses / dwells,
- relative timing between both arms.

This mode should **not** call TOTG.

This is standard time scaling, not a new trajectory optimizer.

### B. Geometric retiming: keep TOTG for `maximize_speed` / geometric `speed_scale`

Use MoveIt 2 `TimeOptimalTrajectoryGeneration` when the intended behavior is:

> Keep approximately the same geometric path and find a new feasible timing from velocity / acceleration limits.

Make the UI / metadata explicit that taught timing is discarded.

Prefer using MoveIt's existing trajectory-processing implementation directly rather than maintaining a behaviorally diverging custom TOTG implementation where practical.

MoveIt TOTG is appropriate for this mode, but it cannot preserve the operator's original local tempo because that timing is not part of the optimization objective.

---

## P1 — Do not geometrically “inflate” slow regions

Do not add artificial positions or distort the joint-space path merely to make a reversal take longer.

If slow / reversal regions need more representation, increase **parameter / waypoint density**, not geometric excursion.

For timing, prefer established mechanisms:

### First choice: preserve taught timing with `tempo_scale`

For the current use case this is likely sufficient and should be implemented before introducing another optimizer.

### Second choice: evaluate Ruckig features before writing custom logic

Ruckig already provides relevant concepts:

- minimum trajectory duration,
- per-section minimum durations,
- per-section velocity / acceleration / jerk limits,
- offline tracking of a target state signal,
- trajectory speed control.

These are conceptually close to “temporally inflate this region” or “follow this taught tempo profile while respecting limits”.

Important limitation:

- MoveIt's built-in `RuckigSmoothing` is mainly a jerk-limited smoothing stage for an already time-parameterized trajectory.
- Ruckig's more advanced multi-waypoint, tracking, per-section and speed-control functionality is largely Ruckig Pro / cloud functionality.

Therefore:

1. first implement and test `tempo_scale`,
2. only if local limit-aware temporal stretching is still required, evaluate whether Ruckig Pro is acceptable for the project,
3. do not implement a custom soft-constrained tempo optimizer before that evaluation.

TOPP-RA may also be evaluated as an established open-source path-parameterization library, but it should not be treated as an automatic solution for preserving the taught local tempo; its main strength is constrained path retiming.

---

## P2 — Acceleration and jerk limits

Do not spend significant effort inventing Nero acceleration or jerk limits.

There are currently no trustworthy manufacturer values available.

For now:

- treat acceleration / jerk assumptions as experimental configuration, not hardware truth,
- avoid safety claims based on guessed limits,
- prioritize hardware playback tests,
- record commanded and measured position / velocity and, where useful, acceleration proxies,
- derive conservative working values only after repeatable measurements.

Ruckig jerk smoothing should remain optional until meaningful jerk limits are available.

---

## Validation

Add focused regression / hardware tests for:

1. **Pre-motion trim**
   - arm recorder,
   - wait several seconds,
   - move,
   - saved trajectory starts close to actual motion onset and contains only the configured pre-roll.

2. **Sampling robustness**
   - identical synthetic motion with regular and irregular sample spacing,
   - reconstructed trajectory should remain close in position and derivatives.

3. **Right-arm hardware capture**
   - measure callback/frame cadence,
   - gap histogram,
   - complete-vector stalls,
   - per-joint reconstruction count,
   - compare before / after source-freshness changes.

4. **Reversal behavior**
   - record a smooth wave / sinusoidal reversal,
   - compare:
     - `smooth`,
     - `tempo_scale`,
     - TOTG geometric retiming.
   - `tempo_scale` must retain the taught local slow-down / reversal shape after global time scaling.

5. **Duo synchronization**
   - verify that trimming / pre-roll and tempo scaling preserve the relative phase between left and right recordings.

---

## Recommended implementation order

1. Pre-motion rolling buffer + trim / rebase.
2. Fix `_geometric_path()` to smooth only after uniform resampling.
3. Add geometric/adaptive waypoint extraction.
4. Add timing-preserving `tempo_scale`.
5. Re-run real right-arm and duo capture/playback tests.
6. Improve source-level feedback freshness if hardware evidence still shows partial-frame stalls.
7. Evaluate Ruckig Pro only if `tempo_scale` is insufficient for local limit-aware stretching.
8. Keep acceleration / jerk calibration as lower-priority experimental work.

## External components to prefer

- **MoveIt 2 TOTG** — geometric path retiming under velocity / acceleration limits.
- **MoveIt 2 RuckigSmoothing** — optional jerk-limited post-processing once meaningful jerk limits exist.
- **Ruckig advanced trajectory generation / tracking** — evaluate before implementing custom local temporal stretching.
- **TOPP-RA** — established open-source constrained path-retiming alternative if MoveIt TOTG becomes insufficient.

The design rule should be: use custom code for recording cleanup, representation and mode semantics; use established trajectory libraries for optimization.
