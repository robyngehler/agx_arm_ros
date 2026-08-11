# Phase 0 Baseline — measured

status: PARTIAL — no-motion scenarios measured directly; MIT load from captured pcaps
date: 2026-08-11
platform: Jetson AGX Orin, 12 cores, ROS Humble
level: **L3** (real hardware, real CAN) for everything below

The before-half of the refactor's before/after. Captured with the counters from
`agx_arm_ctrl/runtime_metrics.py` and `scripts/measure_can_baseline.sh`.

**Scope limit:** hardware access was granted for communication only, with no
commanded motion, so every scenario measured *directly* here ran with
`auto_enable:=false` — connected, streaming feedback, joints never energised.
Scenario 4 is the exception and did not come from this session: it is read from
pcaps captured separately while the MIT controllers were driving both arms. The
hand-action and parallel scenarios are still missing and need a motion slot.

Hardware state during capture: both arms reachable and pushing feedback after
the Jetson 40-pin header was configured; `hand_right` healthy; `hand_left` on a
faulty cable (see `errors_and_fixes.md`).

## Scenario 1 — idle, no ROS nodes running

Nothing of ours running. This is the floor the software starts from.

| Interface | RX/s | TX/s | drops/s |
| --- | ---: | ---: | ---: |
| `can_nero_right` | 2175 | 0 | 0 |
| `can_nero_left` | 2136 | 0 | 0 |
| `hand_left` | 1119 | 0 | 0 |
| `hand_right` | 0 | 0 | 0 |

**The host drains ~5430 frames/s before a single line of our code runs.** The
arms push autonomously at ~2150 f/s each once activated, which matches the
~2150 f/s figure assumed in `critical_cpu_paths.md`. The remaining 1119 f/s is
the faulty left hand emitting frames nobody consumes.

## Scenario 2 — one arm driver, no motion

`agx_arm_ctrl_single` on `can_nero_right`, `auto_enable:=false`, `pub_rate=200`.

| Measure | Value |
| --- | --- |
| process CPU | **71.6 % of one core** (≈6 % of the machine) |
| threads | 26 |
| publish loop rate | 198 Hz (configured 200) |
| `publish_batch` | mean **1.10 ms**, min 0.32 ms, max **2.73 ms** |
| `motor_state_reads` | mean 0.11 ms, min 0.04 ms, max 0.50 ms |
| SDK calls | **1587/s**, all from `Thread-4 (_publish_thread)` |
| — `get_motor_states` | 1388/s |
| — `get_joint_angles` | 198/s |
| CAN delta vs idle | none — RX unchanged, TX still 0 |

### What this changes in the plan

**The per-joint SDK reads are not the dominant cost.**
`critical_cpu_paths.md` names hot path 1 — `get_motor_states` once per joint per
cycle — as "the dominant single-node load". Measured, those reads are **0.11 ms
of a 1.10 ms batch: about 10 %**. The other 90 % is the rest of the batch —
pose, arm status, effector status, leader publication, and message construction
and serialisation. Phase 1E should target the batch as a whole, and the "batch
the per-joint reads" idea alone would recover roughly a tenth of it.

**The callback budget gate is already violated at idle.** Proposal §16.8 asks
that no non-control callback regularly exceed 20 % of its period. At 200 Hz the
period is 5 ms, and the batch averages 1.10 ms (22 %), peaking at 2.73 ms
(55 %) — with one arm, no motion, no MIT controller and no hand.

**`pub_rate` is a republish rate, not a bus lever** — now measured rather than
asserted: adding the driver changed neither RX nor TX on the bus.

**The SDK call volume estimate was accurate.** `critical_cpu_paths.md` estimated
~3.2k blocking SDK calls/s for two arms; one arm measures 1587/s, so two arms
land at ~3.2k.

**One thread today, which is the Phase 1 comparison point.** All 1587 calls/s
came from the publish thread because nothing else was running. The same counter
under a full stack is what shows whether the SDK really has one caller — that
measurement is still owed.

## Scenario 3 — silent bus (degenerate, recorded for contrast)

Same node before the 40-pin header was configured, so the bus carried nothing:

| Measure | Value |
| --- | --- |
| publish loop rate | **3.5 Hz** (configured 200) |
| `publish_batch` | mean 0.03 ms, max 0.09 ms |

The loop degrades by a factor of ~57 when feedback is absent, and the time goes
to the readiness and feedback-timeout paths rather than to publishing. Not a
fault — but it means "the loop runs at `pub_rate`" is false in two different
directions, and any before/after has to name the regime it measured.

## Scenario 4 — MIT active on both arms (from captured pcaps)

Captured by hand into `evidence/can_nero_{left,right}.pcap` with the MIT
controllers running, and analysed with `scripts/analyze_can_pcap.py`. Both
buses carry MIT command frames, so both arms were under control.

| Measure | Per arm bus |
| --- | ---: |
| total | 2849 f/s |
| RX (feedback) | 2148 f/s |
| TX (MIT commands) | 700 f/s |
| MIT command rate | **100 Hz per joint** × 7 joints |

The RX rate is identical to the idle figure: the arm's feedback push does not
change under load, and MIT adds ~700 f/s of command traffic on top. Across both
buses that is ~5700 f/s of arm traffic, plus `hand_left`'s 1119 f/s of garbage,
so a loaded system drains roughly **6800 frames/s**.

## Velocity on the wire — settled: the firmware does not report it

The MIT captures answer this decisively, and no further hardware run is needed.

While the joints were being driven, the reported velocity field
(`0x251`–`0x257`, bytes 0-1, int16, 0.001 rad/s) stayed at **0**, or reached
**±1** on three joints. Velocity derived from the position field in the very
same frames reached 542, 1403, 471, 497, 1396, 33 and 1014 units/s on the right
arm, and up to 3215 on the left.

| Joint (right arm) | Reported peak | Derived peak | Position span |
| ---: | ---: | ---: | ---: |
| 1 | 0 | 542.6 | 180 |
| 2 | 1 | 1402.9 | 486 |
| 3 | 0 | 471.0 | 66 |
| 4 | 0 | 497.3 | 147 |
| 5 | 1 | 1396.3 | 489 |
| 6 | 0 | 32.9 | 1 |
| 7 | 1 | 1013.8 | 471 |

Three consequences:

1. **The vendor's `velocity = 0.0` override hides nothing.** The wire data is
   already zero, so removing it would surface a field that reads plausible and
   is wrong — worse than an obvious zero. The `# TODO: remove this after the bug
   is fixed` comment refers to a firmware bug still present in this capture.
2. **Deriving velocity from timestamped positions is not a workaround, it is the
   only available source.** That closes the C3 checklist item without patching
   the vendor driver: there is no protocol value to compare against.
3. **Anything downstream that consumed that field was reading zeros too** — most
   importantly the MIT controller's goal-velocity tolerance checks, which have
   therefore never actually constrained velocity. Phase 1 must treat velocity
   tolerances as unavailable until they are fed from the derived source.

## Velocity on the wire — original capture (stationary, inconclusive)

Raw frames from a stationary right arm, CAN IDs `0x251`–`0x253`
(`ArmMsgFeedbackHighSpd`, bytes 0-1 = velocity, int16, 0.001 rad/s):

```text
can_nero_right  251  [8]  00 00 FF FA FF FF F9 C4
can_nero_right  252  [8]  00 00 00 47 00 00 05 BA
can_nero_right  253  [8]  00 00 00 4C FF FF FB E5
```

Every velocity field read `0x0000` across the sample. On a stationary arm that
is equally consistent with real velocity and with firmware that always sends
zero, so this capture settles nothing on its own. It is kept as the reference
for what a stationary arm looks like, and as the state of the question before
the MIT pcaps answered it above.

## Still owed for a complete Phase 0 baseline

These need a motion slot and are not captured:

- dual-arm hold; one MIT arm (two MIT arms captured, see scenario 4)
- one hand action
- same-side arm and hand in parallel (only possible under C1)
- both sides arm-plus-hand in parallel
- bus-fault and recovery

Two notes for whoever runs them:

- `hand_left` contributes a constant ~1119 f/s of drain that belongs to a broken
  cable, not to the code under measurement. Either subtract it as a fixed offset
  or capture with that hand's bridge stopped, and say which was done.
- Report CPU both as percentage of one core and of the machine. 71.6 % of a core
  is 6 % of this Jetson, and the two numbers support very different conclusions.
