# Playback Mode Integration Cleanup Proposal

> **Status 2026-08-25.** Worked through. Detail and measurements:
> `docs/sprint_refactor/reference/teach_replay_timebase.md` and
> `trajectory_retiming.md`.
>
> | item | state |
> | --- | --- |
> | P0 legacy scaling vs playback | **done** — refused together, legacy alone still works and is deprecated |
> | P0 sidecar joint identity | **done** — ordered names, exact or under one shared prefix; `--joint-prefix` added so a single-arm sidecar can state its side |
> | P0 sidecar structural validation | **done** — finite/strictly increasing times, finite positions, uniform row width, width against declared joints |
> | P1 catalogue-level playback validation | **done** — every static block parsed at catalogue load |
> | P1 numeric parameter validation | **done** — one rule for all four; `smoothing_window_sec >= 0`, the rest `> 0`, no NaN/inf, no silent default |
> | P1 `tempo_scale` smoothing semantics | **done** — reconstructed in taught time, axis scaled last; normalised path invariant to 2e-4 rad against 6.6e-3 before |
> | P1 velocity-limit guard | **done** — `tempo_scale` only, names the joint and the largest admissible tempo |
> | P1 cancel-aware preplanning | **done** — checked between actions; one library call stays non-interruptible, as the proposal allows |
> | P2 configuration scope | **done** — documented as an Action-level default with a run-wide override; no node-level override added |
> | P2 retiming separation | **unchanged** — the five modes keep their existing split |
> | Hardware validation | **not run** — the ten-case matrix below is still outstanding |
>
> Two things the proposal did not anticipate, both found while implementing it:
> a real single-arm sidecar stores joint names *unprefixed*, so an exact match
> would have refused every one of them (see P0 above); and the retiming cache was
> keyed on the playback spec but not on the legacy time scale, so two plans of one
> action under different scaling collided.

## Goal

Finalize the integration of recorded-trajectory playback modes into the Coordinator / Activity workflow.

The current architecture is largely sound:

- recorded actions are preplanned before activity execution,
- playback modes are selected through action metadata / runtime override,
- timing-preserving and geometric retiming modes are separated,
- recorded trajectories are stored as sidecars rather than duplicated in the catalogue,
- retimed velocities are propagated into the execution path.

This proposal focuses only on the remaining contract, validation, and playback-quality issues before broader hardware validation.

## P0 — Resolve legacy scaling vs. playback mode semantics

Recorded actions currently still apply legacy `velocity_scaling` / `acceleration_scaling` before playback retiming. This can unintentionally multiply with `tempo_scale` or other playback settings.

For recorded actions with an explicit `playback` block:

- reject legacy `velocity_scaling != 1.0`,
- reject legacy `acceleration_scaling != 1.0`,
- make `playback` the single authority over recorded-trajectory timing.

For legacy recorded actions without a `playback` block:

- keep the existing behavior temporarily for backward compatibility,
- document it as deprecated,
- migrate catalogue entries over time.

Add tests proving that explicit playback configuration cannot silently combine with legacy scaling.

## P0 — Validate recording sidecar joint identity

Recording sidecars contain `joint_names`, but matching only the number of joints is insufficient.

When loading a recorded trajectory:

- read and retain `joint_names`,
- compare them against the expected target group,
- require an exact ordered match unless an explicit remapping mechanism exists.

Also validate:

- finite timestamps,
- strictly increasing timestamps,
- finite positions,
- equal row width for all samples,
- number of positions equals number of declared joints.

Reject invalid sidecars before activity execution begins.

## P1 — Make playback validation truly catalogue-level

Static playback configuration should be validated when the catalogue is loaded, not only during activity prewarming.

During catalogue construction:

- parse every recorded action's playback specification,
- reject unsupported modes,
- reject missing required mode parameters,
- reject non-finite or invalid numeric values.

Runtime overrides remain validated when the activity goal is received.

## P1 — Validate playback numeric parameters consistently

Ensure all numeric playback options use the same finite / positive validation rules.

In particular:

- `speed_scale > 0`,
- `resample_dt > 0`,
- `smoothing_window_sec >= 0` or `> 0` according to the chosen contract,
- no `NaN`,
- no `inf`.

Do not silently fall back to defaults for malformed explicit values.

## P1 — Correct `tempo_scale` smoothing semantics

`tempo_scale` should preserve the locally taught timing profile while scaling the complete motion globally.

Preferred processing order:

1. clean / reconstruct the recorded trajectory,
2. uniformly resample in taught time,
3. smooth in taught time,
4. obtain the clean reference trajectory,
5. apply global time scaling: `t_new = t_taught / speed_scale`,
6. resample to the playback/controller rate.

This should make normalized trajectory shape approximately invariant across different `tempo_scale` values.

Do not make a fixed replay-time smoothing window stronger merely because the trajectory is replayed faster.

### Regression test

Replay the same synthetic / recorded wave with several `tempo_scale` values and compare `q(normalized_time)`.

The normalized position profiles should remain approximately identical.

## P1 — Velocity-limit guard for `tempo_scale`

Known joint velocity limits are available and should be used for a simple fail-closed guard.

Before executing `tempo_scale`:

- compute the resulting maximum joint velocity,
- reject the request if a known velocity limit would be exceeded,
- report the limiting joint and, if practical, the maximum admissible global tempo factor.

Do not introduce hard acceleration or jerk rejection yet. There are currently no trustworthy manufacturer acceleration or jerk limits; these should remain experimental parameters until hardware measurements justify stronger constraints.

## P1 — Keep preplanning cancel-aware

Recorded actions are correctly preplanned before physical execution, but a computationally expensive geometric retiming step may take several seconds.

At minimum:

- check activity cancellation / stop state between preplanned actions,
- abort remaining prewarming when cancellation is requested.

Do not implement a custom interruptible TOTG algorithm merely for this purpose. A single in-progress library call may remain non-interruptible for now; document this limitation if necessary.

## P2 — Clarify playback configuration scope

Current playback metadata is an **Action-level default**, not strictly an Activity-node-level property.

Document this correctly.

Current intended hierarchy:

1. runtime activity-wide override, if provided,
2. action-level playback metadata,
3. default playback mode.

Do not introduce node-specific playback overrides unless a real use case requires the same Action to use different playback behavior inside different Activities.

## P2 — Preserve current retiming separation

Keep the playback modes conceptually separate:

### `as_recorded`
Use recorded timing with minimal processing.

### `smooth`
Preserve recorded timing but use the cleaned / uniformly reconstructed trajectory.

### `tempo_scale`
Preserve taught local timing structure and globally scale time.

### `speed_scale`
Use geometric retiming toward a requested duration / speed target.

### `maximize_speed`
Use geometric retiming as fast as permitted by configured constraints.

`speed_scale` / `maximize_speed` may continue to use MoveIt TOTG. They are intentionally different from `tempo_scale`: taught local timing is discarded during geometric retiming.

## Hardware validation

After the cleanup above, prioritize hardware testing instead of further speculative tuning.

Test at least:

1. single-arm recorded action with `smooth`,
2. same action with `tempo_scale < 1`,
3. same action with `tempo_scale = 1`,
4. moderate `tempo_scale > 1`,
5. geometric `speed_scale`,
6. duo recorded action with synchronized arms,
7. recorded actions separated by normal grip / payload-changing actions,
8. repeated execution of the same Activity to verify cache reuse,
9. cancel / Ctrl+C during normal playback,
10. cancel during expensive preplanning.

Collect:

- commanded vs. measured joint position,
- measured update / feedback rate per arm,
- playback duration,
- tracking error,
- velocity utilization,
- preplanning duration,
- cancellation latency.

For reversal-heavy motions such as the wave recordings, specifically compare whether `tempo_scale` preserves the natural deceleration and reversal shape.

## Acceptance criteria

The integration can be considered complete when:

- explicit playback settings cannot accidentally combine with legacy scaling,
- sidecar joint identity is validated,
- malformed static playback configuration fails during catalogue loading,
- `tempo_scale` preserves normalized taught timing behavior,
- known velocity limits cannot be exceeded silently,
- preplanning can be cancelled between actions,
- current Coordinator / Activity tests remain green,
- at least one single-arm and one duo hardware Activity complete successfully under the new playback workflow.

## Non-goals

Do not currently:

- invent manufacturer acceleration or jerk limits,
- implement a custom trajectory optimizer,
- replace MoveIt TOTG,
- add a trajectory database,
- geometrically distort slow / reversal regions,
- add Activity-node playback overrides without a demonstrated use case.

The remaining work should be cleanup and integration hardening, not another retiming redesign.
