# The time grid a replay is built on

date: 2026-08-24
platform: Jetson AGX Orin, both arms
method: offline, against two real 7-joint teach recordings
(`Wave_Right_V01`, `Wave_Left_V02`). The retiming output is fed through
`JointTrajectoryBuffer.sample()` on the MIT controller's 200 Hz grid, and the
metric is the acceleration of the position stream the controller actually walks.
Not exercised on hardware.

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

### `_geometric_path` stays on the recorded grid

TOTG re-times the path, so grid unevenness cannot reach its output, and
equal-sample bins put more waypoints where the arm was moving. Binning a uniform
grid instead spends waypoints on dwells: `maximize_speed` measured 6.28 s → 8.02 s
on `Wave_Left_V02`. The uniform grid is for the timing-preserving modes only.

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

Each capture reports its stalled-read count and the worst implied joint speed,
and warns when that speed exceeds the joint's limit — the stored file cannot be
read back for this, because a stall and a still arm look identical in it.

This does not make the grid uniform: the arm still jitters, and a dropped frame
is still a gap. The playback resample is what the replay depends on.

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
- none of this is measured on hardware. The prediction is that `as_recorded` and
  `smooth` become executable at the same duration they were taught at
