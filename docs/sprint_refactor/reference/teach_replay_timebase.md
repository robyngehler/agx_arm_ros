# The time grid a replay is built on

date: 2026-08-24
platform: Jetson AGX Orin, both arms
method: offline, against two real 7-joint teach recordings
(`Wave_Right_V01`, `Wave_Left_V02`). The retiming output is fed through
`JointTrajectoryBuffer.sample()` on the MIT controller's 200 Hz grid, and the
metric is the acceleration of the position stream the controller actually walks.

**Confirmed on hardware 2026-08-25** — both arms, a duo teach-and-replay session
across all five modes. See "What the hardware run showed" at the end.

## The metric

The MIT frame carries position, velocity, kp, kd and torque — acceleration is
never commanded. What the arm reproduces as judder is therefore the **second
difference of the commanded position stream**, sampled at the control rate, plus
the disagreement between `v_des` and the slope of that stream.

`JointTrajectoryBuffer.sample()` interpolates **linearly** between trajectory
points. The commanded velocity is therefore piecewise constant, with one step per
trajectory knot. Everything below follows from that one property.

## A recording has no rate, it has a grid

A teach recording is not evenly spaced. Two independent causes:

- the arm's feedback cadence jitters, and it is not configurable
  (`feedback_rate_budget.md`)
- until 2026-08-24 the recorder sampled a feedback cache on a fixed clock. At
  100 Hz against ~100 updates/s the two beat, so 27-39% of cycles stored the
  previous sample again. De-duplicating those repeats removed rows but left the
  survivors on the times they were taken

Measured on the two recordings, both captured at 100 Hz nominal:

| | stored | removed as repeats | dt histogram |
| --- | --- | --- | --- |
| `Wave_Right_V01` | 1192 of 1939 | 39% | 10-12 ms: 715, 20-30 ms: 336, 30-50 ms: 127, >50 ms: 9 (max 1048) |
| `Wave_Left_V02` | 1061 of 1447 | 27% | 10-12 ms: 733, 20-30 ms: 284, 30-50 ms: 38 (max 253) |

The grid is bimodal at 10/20 ms. A filter and a difference taken in **sample
index space** over that grid are neither a time-domain filter nor a derivative:
positions are made comparable by index, then divided by unequal intervals.

## What that cost

Commanded acceleration through the controller's sampler, before and after the
2026-08-24 change:

| mode | p95 (rad/s²) | max | \|v_des − dp/dt\| rms | accel sign flips /s/joint |
| --- | --- | --- | --- | --- |
| `as_recorded` before | 27.3 / 42.9 | 1177 / 383 | 0.096 / 0.087 | 42 / 56 |
| `as_recorded` after | **5.8 / 5.6** | 109 / 27 | 0.010 / 0.006 | 24 / 37 |
| `smooth` before | 12.5 / 22.7 | 248 / 143 | 0.047 / 0.045 | 58 / 67 |
| `smooth` after | **3.9 / 3.7** | 68 / 16 | 0.007 / 0.004 | 27 / 39 |
| `speed_scale` (TOTG) | 1.4 / 1.7 | 1.5 / 1.7 | 0.001 / 0.002 | unchanged |

(right arm / left arm.)

50-odd sign changes per second per joint is a ~25 Hz excitation of the command
stream, inside the arm's structural band.

**The TOTG modes were never affected.** `_run_totg` emits a uniform 5 ms
resample and its velocities are the true derivative of the path it emits. That
is why re-timed replay was smooth while a timing-preserving replay was not.

## Three things it was not

**Not the recording rate against the control rate.** Resampling `as_recorded`
from 60-72 Hz onto the controller's 200 Hz grid changes the numbers by nothing at
all — 27.27 → 27.27. Linear interpolation of a piecewise-linear path onto a finer
grid retraces the same path. Coupling replay to `sample_rate_hz` would have fixed
nothing.

**Not the filter width.** Widening `smooth` from 0.10 s to 0.30 s on the recorded
grid moves p95 from 12.5 to 11.6. More smoothing in sample space cannot repair a
division by unequal dt.

**Not a regression from adding TOTG.** The pre-TOTG path replayed on a uniform
10 ms grid, where a sample-count window *is* a time-domain filter. Reconstructing
that path (zero-order hold back to 100 Hz, 15-sample window) measures 5.1 / 6.2
rad/s² — the working point the current `as_recorded` floor reproduces. The old
design also had no `as_recorded`: it always smoothed. Raw replay on the old
uniform grid measures 74.7 / 100.5 and was never available.

## The fix

**Every mode resamples onto a uniform grid before it filters or differentiates**,
and emits on that grid.

1. linear interpolation onto `resample_dt` (default 0.005 s, the control period)
   over the recorded duration exactly — the taught pace is preserved and every
   output pose stays a convex combination of two recorded ones
2. moving average on that grid, which is now a filter with a fixed cutoff
3. central differences on that grid, so `v_des` matches the slope of the emitted
   path

`as_recorded` keeps its name and gets a **floor** of `RECONSTRUCTION_WINDOW_SEC`
= 0.06 s. It means the taught path and pace at the smallest filter that executes,
not an unprocessed sample dump. The floor is where the old design's working point
sits; below it the recording's own noise dominates:

| window (s) | p95 accel | max path deviation (rad) |
| --- | --- | --- |
| 0 (resample only) | 27.3 / 42.9 | 0 |
| 0.04 | 8.3 / 8.3 | 0.034 / 0.008 |
| **0.06 (floor)** | **6.0 / 5.7** | 0.046 / 0.009 |
| 0.10 (`smooth` default) | 4.0 / 3.8 | 0.058 / 0.010 |
| 0.30 | 1.8 / 1.8 | 0.078 / 0.023 |

`path_deviation` is now non-zero in every mode and is reported per replay.

### The window is reflected, not shrunk

A centred window that shrinks at the edges preserves the endpoints but leaves the
outermost samples unfiltered, which put the peak commanded acceleration there —
109 rad/s² against 5.8 over the rest of the replay. Odd reflection through the
endpoint (`q[-k] = 2*q[0] - q[k]`) holds the window at full width **and**
reproduces the endpoint exactly, because the reflected pairs sum to `2*q[0]`.

### `_geometric_path` resamples too, and picks by chord error

**Superseded 2026-08-25.** This section previously said the geometric path stays
on the recorded grid, because resampling it and then binning by equal samples
spent waypoints on dwells and measured 20-28% slower at full limits — `Wave_Left_V02`
went 6.28 s → 8.02 s.

That was the wrong conclusion from a correct measurement: the problem was the
*selection*, not the resample. `_moving_average` is a time-domain filter only on
a uniform grid, so it belongs after the resample there as much as anywhere. The
sparse path is now chosen by **chord error** (Douglas-Peucker refinement), which
places waypoints where the path bends rather than where the arm spent time, and
bounds the geometric error of the sparse path directly. Detail and the
fidelity/duration trade in `trajectory_retiming.md`.

## Recording is now callback-driven

The recorder stores one sample per **changed** feedback read, timed by the
publisher's frame stamp — the arm's cadence is the recording's cadence. There is
no `--sample-rate` argument, because there is no rate to choose: a sampler above
the arm's rate stores repeats, and one below it discards updates.

- `sample_rate_hz` in a new recording is the rate the arm delivered
- a duo capture records each arm from its own callbacks and re-bases both onto one
  time axis afterwards. `merge_arm_recordings` resamples to a common grid at the
  faster arm's rate
- an arm that publishes no header stamp falls back to arrival time; mixed clocks
  across arms are refused rather than silently skewed
- there is no de-duplication pass after the fact. Capture never stores a repeat,
  and a duo merge output is a uniform grid that removing rows could only make
  uneven

### An advancing stamp is not advancing data

**Measured 2026-08-24 on `Wave_Both_V1`.** The stamp comes from the last CAN
frame to touch the driver's cache (`rx_can_frame.timestamp`), and one complete
joint update is four position frames. So the stamp advances while the positions
may not, and a capture keyed on the stamp stores the stall as a sample — which
asserts the arm was at that pose at that instant and forces the eventual
catch-up into a single step.

On the right arm, at t=3.404 and t=3.413, **six of seven joints stepped together
at 3-7x their own typical sample**, reaching 4.37 rad/s on joint4 against its
3.93 rad/s limit. The left arm had no such sample anywhere in the recording. Six
joints accelerating fivefold for exactly two samples and back is not a motion a
hand can make.

| | left arm | right arm |
| --- | --- | --- |
| position steps over 6x that joint's p95 | 0 | 2 |
| worst implied joint speed | 1.94 rad/s | 4.37 rad/s (over limit) |
| samples with ≥4 joints stepping together | 0 | 2 |
| max commanded accel after `smooth` | 12.4 rad/s² | 39.3 rad/s² |

Capture therefore refuses a read whose positions equal the previous stored one.
The stall becomes a gap, and the playback resample interpolates the catch-up
across it. On the measured stall shape — a joint at 0.22 rad/s frozen for three
reads, then catching up:

| | max commanded accel | v/limit |
| --- | --- | --- |
| stall stored as samples | 6.28 rad/s² | 0.086 |
| stalled reads dropped | **0.00** | 0.070 |
| the motion actually performed | 0.00 | 0.070 |

Dropping reconstructs the true constant-velocity motion exactly. A genuinely
still arm is unaffected: its gap interpolates flat between two equal poses.

### A stall is per joint, not per read

Refusing an unchanged read only catches a stall of the **whole** vector. The
driver assembles that vector from four position frames covering joint pairs, so
one pair can hold while another updates — the read is genuinely new for the rest
of the arm and cannot be dropped. Measured 2026-08-24 on the next two duo takes,
after whole-vector stalls were already being refused (929 and 525 of them):

```
[right_arm] joint6 implies 11.76 rad/s at t=5.06s, over its 3.93 rad/s limit
```

Each arm's samples therefore get a per-joint pass before the recording is built:
a joint that holds a bit-identical value and then steps has that step spread
linearly back over the hold. A moving joint covers tens of encoder quanta per
sample, so two consecutive bit-identical values are the cache, not the joint.

Holds longer than `MAX_STALL_HOLD_SEC` (0.1 s) are left alone — that is a joint
standing still, and ramping a step back across it would turn a sharp start of
motion into a slow one. The cap is a duration, not a sample count, so a change of
recording rate does not retune it.

On a synthesised stall of the observed shape — one joint frozen for four reads
while its neighbours update, then catching up:

| | implied joint speed | max commanded accel | error vs the true motion |
| --- | --- | --- | --- |
| stall stored | 1.30 rad/s | 9.86 rad/s² | 0.014 rad |
| reconstructed | 0.26 rad/s | **0.00** | **0.000 rad** |
| the motion actually made | 0.26 rad/s | 0.00 | — |

Note that this stall was **under** the joint's velocity limit and still cost
9.86 rad/s². Gating the reconstruction on the velocity limit was tried and
rejected for that reason: duration separates a stall from a dwell, speed does
not.

Each capture reports its stalled-read count, its per-joint spread count, and the
worst implied joint speed — the stored file cannot be read back for this, because
a stall and a still arm look identical in it.

## Not every over-limit sample is a stall

**Measured 2026-08-24.** In freedrive the arm is back-driven by hand, and a hand
can move a joint faster than the joint will accept as a setpoint: the 3.93 rad/s
figure is a commanded-velocity limit, not a mechanical bound on back-driving. So
a sample implying more than the limit is *not* evidence of a stalled cache.

The two shapes separate cleanly:

| shape | reading |
| --- | --- |
| an isolated step whose neighbours are near zero | the cache stalled and caught up |
| a run of consecutive large steps | the arm really was moved that fast |

Across the two takes, 11 of 14 over-limit samples were runs — a bell-shaped ramp
over six samples reaching 8.3 rad/s on `right_arm_joint6`. Only 3 were isolated.
The recording is honest; the taught pace is simply faster than the arm can
reproduce. Reconstructing the stalls leaves those runs untouched, by design, and
leaves the commanded acceleration where it was:

| | left arm | right arm |
| --- | --- | --- |
| `smooth` 0.10 s, p95 / max commanded accel | 2.2 / 9.7 | 3.2 / **72.9** |
| `smooth` 0.10 s, velocity utilisation | 0.20 | **0.81** |
| `smooth` 0.30 s, max commanded accel | 5.1 | 27.1 |
| `speed_scale` 1.0, max commanded accel | 1.7 | 1.7 |

**`speed_scale` is the answer to a take that was taught too fast**, because TOTG
re-times against the limits by construction. A wider `smooth` window helps but
cannot fully remove content the recording genuinely contains. The capture warning
names the joint and the speed and says the replay will have to slow down or
smooth it away; it no longer asserts which of the two causes it was, because
speed alone cannot tell.

This does not make the grid uniform: the arm still jitters, and a dropped frame
is still a gap. The playback resample is what the replay depends on.

## Recording starts at the motion, not at the keypress

Arming the recorder and starting to move are seconds apart, and that interval was
persisted: every replay opened with a dead hold. Capture now drops it and keeps a
`--pre-roll-sec` window (0.25 s) before the first threshold crossing, because a
cut at the crossing itself clips the physical start of the motion.

**One cut instant for every arm**, taken from the earliest onset across them. Per
arm it would shift the arms against each other; the pair's relative phase is what
a duo replay is for. An arm that never crossed the threshold is not trimmed to a
stub — it still has to start where it was standing, and the retiming needs four
samples.

## Where per-joint freshness could come from

`reconstruct_stalled_joints()` is a recovery mechanism, not a source of truth.
The information it reconstructs **already exists one layer down**:
`get_joint_angles()` in the vendor driver
(`pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py`) reassembles the
7-joint vector from four separately cached frames — `joint_12`, `joint_34`,
`joint_56`, `joint_7` — each carrying its own `timestamp` from the CAN frame that
filled it. Assembly then collapses them:

```python
self._joint_angles.timestamp = joint_angles.timestamp   # the last group present
```

So the composite stamp is one group's, not the newest and not per joint, and the
per-group freshness is discarded at exactly the point where a consumer could use
it. Publishing it — a four-entry freshness vector or per-group stamps alongside
`feedback/joint_states` — would turn the reconstruction from an inference into a
read. That is an arm-bridge change needing hardware validation, and it is the
recommended next step rather than more post-processing.

## Moving to a replay's start pose

A replay begins at the pose it was taught from. A gap up to
`PLAYBACK_MAX_START_OFFSET` (0.35 rad) is bridged by a lead-in — a straight
joint-space move. Beyond it, `m` in playback mode plans and runs a
collision-checked MoveIt move to the recording's first waypoint, on `both_arms`
for a duo recording and on that side's group for a single arm. It is a separate
keypress because it is motion through free space that was never taught.

A duo refusal takes its maximum over fourteen joints, so it now names the arm and
joint that is furthest out rather than only the distance.

## Existing recordings

No migration needed. The fix is at playback, so recordings taken under the paced
sampler replay at the numbers in the table above. `scripts/clean_recording.py`
removes repeats and therefore makes a grid less even; it is no longer part of the
recording path.

## Open

- the reconstruction floor is set from two recordings. It is a filter width, so a
  motion with real content above ~8 Hz would be cut by it; nothing taught by hand
  has come close
- the acceleration limit is still `2.5 · v_max`, an admitted stand-in, and it is
  what sets every TOTG duration in this note
- per-joint freshness at the source is not implemented; the reconstruction stays
  an inference

## `tempo_scale`: replay slower without re-planning

A take taught faster than the arm can command has two possible answers, and until
2026-08-25 only one existed. `speed_scale` re-times through TOTG, which brings
the motion under the limits but discards the taught timing — a dwell is zero path
progress, so the parameterization has no notion of it.

`tempo_scale` scales the time axis and nothing else. The recording is
reconstructed in **taught** time — resampled onto a uniform grid, then filtered —
and only then is the axis divided by the tempo. Relative dwells, reversal timing
and the phase between two arms are all ratios, and a ratio survives a scaled
clock. The grid step is taken in *replay* seconds, so the controller is handed
`resample_dt` knots however the clock was scaled.

Measured on the duo take whose right arm was taught too fast (30.16 s):

| mode | duration | v/limit (right) | max commanded accel | deviation |
| --- | --- | --- | --- | --- |
| `smooth` 0.10 s | 30.16 s | 0.40 | 33.1 rad/s² | 0.023 |
| `tempo_scale` 0.8 | 37.70 s | 0.32 | 20.6 | 0.024 |
| `tempo_scale` 0.6 | 50.27 s | 0.24 | 11.9 | 0.023 |
| `tempo_scale` 0.5 | 60.32 s | 0.20 | 8.5 | 0.023 |
| `speed_scale` 1.0 | 30.49 s | 0.12 | 1.2 | 0.048 |

Correlation of the right arm's normalised speed profile with the taught one:
**`tempo_scale` gives r = 1.000, `speed_scale` gives r = 0.242.** That difference
is the whole reason the mode exists.

Three consequences worth knowing:

- **the path is the same path at every tempo.** Commanded acceleration falls as
  tempo² exactly (33.1 → 20.6 → 11.9 → 8.5 against 21.2 / 11.9 / 8.3 predicted),
  and the deviation column does not move. Filtering in *replay* seconds instead —
  the behaviour until 2026-08-25 — made the window a function of the tempo, so a
  faster replay smoothed more of the taught path and a slower one less: the
  normalised path moved by up to 0.126 rad at 3x and 0.027 rad at 0.5x on
  `Push_Up_V01`, against 0.002 rad now. A tempo dialled at replay time must not
  change what was taught.
- **a tempo that leaves the joint speed limits is refused**, naming the joint and
  the largest tempo that would pass. It is the one timing-preserving mode that
  carries a speed request, so there is something to refuse; `as_recorded` and
  `smooth` reproduce whatever was taught and report an over-limit recording
  instead, because a hand back-driving a joint can exceed a setpoint limit.
  Acceleration is reported and never refused — `2.5 · v_max` is a stand-in and
  cannot carry a rejection.
- it costs 0.1-0.2 s to plan against 1-10 s for `speed_scale`, because there is
  no search — it is a resample.

## The limit scale search is not a bisection any more

`speed_scale` searches for the limit scale whose parameterization lands on a
requested duration. That search assumed duration falls monotonically as the
limits rise. **It does not.** `_run_totg` corrects the limits against the peak it
samples, that correction moves with the blend radii, and the blend radii move
with the scale. Traced on one path:

```
scale 0.3767 -> 3.969 s
scale 0.4909 -> 3.540 s
scale 0.5674 -> 4.093 s   <- higher limits, longer trajectory
scale 0.6599 -> 2.495 s
```

A bisection converges onto whichever local branch it started on and then reports
its own dead end as the hardware's: it returned 1.13x on a path that
`maximize_speed` runs at 1.38x. The search now scans the scale range descending,
refines around the best slow-side sample, and rescans the full grid when the
bracket it found is more than 5% off target. It still returns only slow-side
candidates — on a taught motion an unrequested speed-up is the dangerous
direction.

Seeding the descent from a power-law estimate was tried and reverted: a wrong
estimate left the descent with no slow-side candidate at all, and it fell back to
full limits — *faster than requested*. The invariant is worth more than the 40%
of planning time it saved.

The note it emits no longer says "the limits allow X". It says what was reached
and that `maximize_speed` is what measures a ceiling.

## Related: the resample step changes the answer

`_run_totg`'s limit correction measures its peaks on the **resampled** output, so
a coarser step aliases them away and under-corrects: the same path at scale 0.5
returns 18.94 s at `resample_dt` 0.005 and 13.11 s at 0.05. A cheap coarse probe
during the search is therefore not available — the duration it would report is
not the duration the plan has.

## What the hardware run showed

2026-08-25, both arms, one duo teach-and-replay session over all five modes. The
recording is `Wave_Both_V01-01` (1067 samples, 14.82 s, 71.9 Hz merged).

**The offline pipeline reproduces the run exactly.** Re-planning the saved
recording with the parameters the session used returns the same duration, path
deviation and utilisation figures the node logged, to the digit, in all five
cases. Everything measured in this note therefore describes what the arms
actually received.

**Per-joint stalls are frequent, and not only on the right arm.** Of the reads
that survived whole-vector rejection, the reconstruction spread a stall on:

| | whole-vector reads dropped | stored samples | per-joint stalls spread | per joint |
| --- | --- | --- | --- | --- |
| left arm | 905 | 1229 | 669 | 137, 85, 96, 76, 94, 97, 84 |
| right arm | 596 | 1081 | 401 | 88, 75, 37, 51, 61, 47, 42 |

Roughly 7-11% of stored samples carry a stall on any given joint, and the left
arm carries *more* of them than the right despite replaying cleanly all along —
the per-joint stall is a property of how the vector is assembled, not of one
arm's health. That is the strongest argument yet for publishing per-group
freshness at the source rather than inferring it.

**The reconstruction is measurable on a real capture.** The capture warned that
`right_arm` joint4 reached 6.89 rad/s on the raw samples; the saved recording's
worst implied speed is 5.72 rad/s. Resampling upward preserves segment slopes, so
that 17% is the reconstruction, not the merge.

**`tempo_scale` behaves as designed.** At 2.0x it returned exactly 7.41 s against
14.82 s, and at 1.0x it is bit-identical to `smooth` at the same window — which
is the correctness check worth having, since 1.0x *is* `smooth`.

The window-in-replay-seconds effect is visible in the numbers: doubling the tempo
raised velocity utilisation 0.35 → 0.50, a factor of 1.43 rather than 2, because
the 0.30 s window covers twice as much taught content at the faster tempo.

**One transparency gap the run exposed.** The smoothing window persists across
mode switches, and `speed_scale` uses it for the geometric path while its prompt
never mentions it: a window of 0.30 set for a `tempo_scale` replay silently
shaped the next `speed_scale` plan (15.68 s and 0.097 rad deviation against
15.41 s and 0.069 rad at the 0.10 default). The parameterized modes' note now
states the window and waypoint count that produced their path.

## The coordinator path did not inherit any of this

Everything above describes the **teach manager's** replay. The coordinator — the
path that runs an assembled activity such as the tea demo — reaches the same
controller through a different chain, and until 2026-08-25 it carried none of it:

```
recording (clean)  ->  recorded_to_waypoints  ->  catalogue YAML
                    ->  arm_executor  ->  coordinator_node  ->  ExecuteTrajectory
```

Two losses, both measured on `Wave_Left_V02` at the tea demo's own density
(1061 samples down to 73 waypoints, as `left_arm_pour_tea` does at 721 -> 73):

| path | points | p95 cmd accel | max | \|v_des − dp/dt\| |
| --- | --- | --- | --- | --- |
| teach manager `smooth` | 2950 | 3.70 | **16.3** | 0.004 |
| coordinator, before | 73 | 0.00 | **98.4** | **0.224** |
| coordinator, after | 73 | 0.00 | 81.9 | 0.040 |

**The p95 of 0.00 is not smoothness.** With 73 knots over 14.7 s almost every
control tick falls *inside* a linear segment, where the second difference is
exactly zero; all of the acceleration is concentrated at the knots. A sparse
waypoint list is a rougher command stream than a dense one, not a gentler one.

**`v_des` was zero everywhere.** `_build_execute_trajectory_goal` emitted
positions and times only, and the trajectory buffer reads a missing velocity as a
commanded zero — so `kd·(0 − q̇)` braked against the motion the position term was
asking for. This is the same defect that started this whole line of work, fixed
in the teach path months earlier and still live in the production path. It now
supplies central differences over the waypoint times.

**Catalogue waypoints were selected by even index**, which is selection by the
clock: it spends the budget on dwells. `recorded_to_waypoints` now uses the same
chord-error selection as the geometric path, which costs nothing in storage and
measured 1.1-4.2x less chord error at the same count across five recordings.

### Both gaps closed 2026-08-25

**The modes.** A recorded action carries an optional `playback` block, declared
where `payload_update` is declared — a property of this step in this activity,
not of the recording:

```yaml
metadata:
  source: recorded
  recording: recordings/tea_pour_left.json
  playback: { mode: tempo_scale, speed_scale: 0.6 }
```

Default is `smooth` at a 0.3 s window. An unusable request is refused at load,
never silently defaulted: a replay that ran under a mode the activity did not ask
for is worse than one that refused to start. A run-level override travels in
`PerformActivity.metadata_json` — `{"playback": {...}}` applies to every recorded
action in that run and leaves the catalogue alone.

Planning happens **before the first action moves**, not at dispatch: `speed_scale`
searches the limit scale and measured 11 s on a 30 s duo recording, which would be
a stall in the middle of a sequence. The result is cached per action and spec.

**The density.** An action may reference a recording instead of inlining
waypoints. The reference is resolved when the catalogue is read, so a missing or
malformed file stops the coordinator coming up rather than failing an activity
that is already running.

The sidecar carries only joint names, times and positions — the retiming
recomputes velocities, playback zeroes efforts, the flange pose is diagnostic:

| | size | parse | density |
| --- | --- | --- | --- |
| full teach recording | 2320 KB | 30 ms | 2236 pts |
| lean sidecar | **277 KB** (12%) | **8.5 ms** | 2236 pts |
| inlined catalogue waypoints | 27 KB | — | 73 pts (10:1 loss) |

**Not a database.** There is one row per action, fetched by name — an index buys
nothing. What the data does need is to be diffable and versioned with the
catalogue that references it, because a demo is reproducible only if its
trajectories are in git alongside the graph that plays them. A binary store would
trade that for a lookup nobody performs.

Inline `waypoints` still work; declaring both is refused.

### What the catalogue proves before an activity runs (2026-08-25)

The refusals below all happen before the first action moves — at catalogue load
where the input allows it, at preplanning otherwise.

- **the sidecar has to be replayable**: finite and strictly increasing times,
  finite positions, one row width, as many positions as declared joint names.
- **the recording has to belong to the group it will be commanded on.** Names
  equal to the group's match; names the group's own carry a single shared prefix
  over (`joint1` against `left_arm_joint1`) match too, because a single-arm teach
  recording stores joint names unprefixed. That spelling names the joints and
  their order but *not* the side, which is then the catalogue's claim — emit the
  sidecar with `--joint-prefix left_arm_` to make it the recording's claim and
  have it checked. A duo recording is already prefixed by the merge.
- **every static `playback` block is parsed at catalogue load.** Preplanning
  would catch it too, but only once someone ran an activity that used the action.
  A run-level override is checked when the goal arrives.
- **`playback` is the single authority over a recorded action's timing.**
  `velocity_scaling` / `acceleration_scaling` predate it and stretch the taught
  times before the mode sees them, so `tempo_scale: 0.5` under
  `velocity_scaling: 0.5` would run at a quarter speed with neither number saying
  so. The pair is refused. On an action with no `playback` block the legacy knobs
  still apply, deprecated: catalogue entries migrate over time.
- **preplanning is cancellable between actions.** One retiming is a single
  library call that runs to completion, so an action is the finest granularity
  available; under `speed_scale` that call is seconds, not the whole graph.

### What is still not shared

- **the stall reconstruction and the pre-motion trim** apply at record time, so an
  activity assembled from a recording taken after 2026-08-25 gets them for free —
  a catalogue block pasted before then still carries whatever the recorder
  produced at the time
