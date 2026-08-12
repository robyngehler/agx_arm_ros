# Phase 1A — hardware validation (L3)

date: 2026-08-12
platform: Jetson AGX Orin, four-bus topology, both arms and both hands online
scope: the Phase 1A changes that could only be argued at L1 until hardware
returned — enable readback, firmware-tier resolution, MIT boundary validation,
the published device authority, and the emergency-stop epoch.

Nothing here was measured on a moving arm. Every accepted-command test ran as a
position hold at the pose the arm was already in; every rejected-command test
sends nothing by construction.

## What the arms actually are

| Arm | CAN interface | firmware | protocol tier | MIT torque bound per joint (N·m) |
| --- | --- | --- | --- | --- |
| right | `can_nero_right` | 1.06 | `NeroFW.DEFAULT` | 24, 24, 16, 16, 8, 8, 8 |
| left | `can_nero_left` | 1.11 | `NeroFW.V111` | 16 on every joint |

**The two arms do not run the same firmware.** This was recorded nowhere before
and is why the tier now appears in the startup log. The tiers are different
protocols, not different revisions of one: the 1.11 driver encodes MIT frames
with a 12-bit feed-forward torque field and no CRC, and overrides
`set_motion_mode`, `move_mit`, `get_flange_pose` and `get_motor_states`.
The arms were bought as different versions and cannot be flashed, so this is
permanent — binding constraint C8 in `planning/integration_plan.md`.

## Enable readback

The stricter check — the joint readback decides both `enable_flag` and the
return value — confirmed on the first attempt on both arms at startup, so it
introduces no spurious failures. A full disable/enable cycle on the right arm
returned success in both directions with the readback agreeing each time.

## MIT boundary validation

Rejections, right arm (default tier):

| command | outcome |
| --- | --- |
| `p_des` containing NaN | refused, `non_finite` |
| joint 2 listed twice | refused, `duplicate_joint` |
| joint index 8 | refused, `unknown_joint`, "outside 1..7" |
| 12 N·m on joint 6 | refused, `out_of_range`, against `[-8, 8]` |

Left arm (1.11 tier): 20 N·m on joint 6 refused against `[-16, 16]` — the same
joint, a different bound, which is the point.

Acceptance, right arm: the MIT controller was launched with the deployed config
(gravity compensation on, hold gains `kp [50,20,50,20,20,10,10]`), enabled, and
told to hold the current pose. Over the hold, the driver logged **zero
rejections and zero joint-limit warnings**. The validator does not refuse
legitimate controller traffic.

The joint-limit check never fired during hold. That is one data point toward
promoting it from a warning to a rejection; a trajectory run is the stronger
test and has not been made.

## Published device authority

Right arm, `feedback/authority`, latched. The full sequence in one session:

| event | state | device epoch | unit epoch | accepts motion |
| --- | --- | --- | --- | --- |
| startup, feedback ready | READY | 1 | 0 | yes |
| MIT hold streaming | READY | 1 | 0 | yes |
| emergency stop, verified | STOPPED | 2 | 1 | no |
| `clear_fault_lockout` | READY | 3 | 2 | yes |
| arm disabled | STANDBY | 4 | 2 | no |
| arm re-enabled | READY | 5 | 2 | yes |

The emergency stop raised the unit epoch *before* attempting the stop, and the
device left READY on its own epoch — so a command issued under epoch 1 is stale
for both reasons. The stop itself reported
`stop=verified — confirmed stopped (peak 0.007 rad/s (dt=24ms))` and did not
escalate, so the arm stayed energised.

## Two reporting gaps this session exposed

Both were introduced by the Phase 1A changes and fixed the same day:

- `clear_fault_lockout` answered "no fault lockout was active" to a call that
  had just released a unit safety stop. A verified emergency stop leaves a unit
  stop and no lockout, so the only latch it reported on was the one that was
  not set.
- the emergency stop reported "confirmed stopped" without mentioning that a unit
  stop stays latched. A caller would have found the arm refusing motion with
  nothing in the response explaining what held it.

## Runtime cost

Right arm driver, idle, no MIT load, metrics enabled:

```
publish_batch:     n=2001 mean=0.95ms min=0.40ms max=2.33ms
motor_state_reads: n=2001 mean=0.10ms min=0.04ms max=0.35ms
sdk calls: 16008 (1600/s) from 1 thread(s)
```

Against the 0E baseline (1.10 ms mean batch, per-joint reads ~10 % of it) this
is unchanged within run-to-run variation. The Phase 1A additions — validation
per command, one authority sync per publish cycle — did not move the hot path.
The serialized SDK worker is not in this path yet.

## MIT consuming the authority (added 2026-08-12, same session)

Right arm, driver plus MIT controller from the deployed launch, arm held at its
own pose throughout.

| step | messages on `control/move_mit` |
| --- | --- |
| MIT enabled, holding | 601 in 6 s (100.2/s) |
| after `emergency_stop` | **0 in 8 s** |
| after `clear_fault_lockout` | 563 in 6 s (93.8/s) |

The controller stands down completely on losing authority and resumes on its
own when authority returns — no operator step in between. The dip in the resume
rate is the gain ramp and the hold recapture.

The controller's own log carried the whole chain:

```
Device authority for 'arm_right' is now the gate (state=0, accepts_motion=False)
Device authority changed (state=2, device_epoch=1, unit_safety_epoch=0, accepts_motion=True): rearmed: publish loop
Device authority changed (state=5, device_epoch=2, unit_safety_epoch=1, accepts_motion=False): unit stop: emergency stop requested
Device authority changed (state=2, device_epoch=3, unit_safety_epoch=2, accepts_motion=True): rearmed: fault lockout cleared
```

### How this was measured, and how it was nearly mismeasured

`ros2 topic hz` is not a trustworthy witness for "did this node stop
publishing?". Its output is block-buffered when redirected, so the last seconds
are lost when the process is killed, and a shell marker appended to the same
file has no defined position relative to those buffered flushes. During this
session that produced two confident-looking zeros that were artifacts — one of
them briefly supported a wrong conclusion, that `hold_current` stops the
stream, which the deterministic count then disproved at 100.2/s.

Every number in the table above comes from `scripts/count_topic_messages.py`,
which subscribes, counts over a fixed window, and prints once at the end. Use it
for any "is it still publishing?" question; do not use `ros2 topic hz`.

## Fail-closed authority (added 2026-08-12)

`require_device_authority` now defaults to true, so a missing authority is a
refusal rather than a pass. The risk of that change is that a wiring mistake in
the deployed launch would stop the arms; verified on hardware that it does not:
the standard launch comes up, the controller reports
`Device authority for 'arm_right' is now the gate`, the identity check accepts
it, and MIT settles at **100.0 commands/s**.

The launch derives `expected_device_id` from the same `can_port` the driver
uses, so with two arms publishing, a controller cannot be gated by the other
arm's authority — which would report ready while the device it commands is
stopped.
