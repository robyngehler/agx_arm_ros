# SDK call latency budget for the serialized worker

date: 2026-08-12, extended 2026-08-13 with a real bus fault
platform: Jetson AGX Orin, right arm (`can_nero_right`, firmware 1.06, default
protocol tier), driver plus MIT controller holding at the measured pose
method: `RuntimeMetrics` + `MeasuredSdk`, which wraps the vendor SDK object and
times every call by name — chosen over instrumenting call sites so that a call
nobody thought to measure is still measured, since that is the one that would
set the worst case.

## Why this exists

Phase 1A moves every SDK call for a device onto one serialized worker thread.
The worker's safety lane overtakes *queued* work, but nothing preempts work
already executing, so an emergency stop is only as prompt as the longest thing
it can queue behind. That number must be measured before the routing lands, or
the refactor trades a race for head-of-line blocking on the stop path.

## Measured, under MIT streaming at 100 Hz

Two ten-second windows, ~5000 SDK calls/s.

| call | n | mean | max |
| --- | --- | --- | --- |
| `get_motor_states` | 14000 | 0.01 ms | 0.18 ms |
| `get_arm_status` | 2000 | 0.01 ms | 0.18 ms |
| `is_ok` | 4000 | 0.00 ms | 0.09 ms |
| `has_comm_error` | 2000 | 0.01 ms | 0.18 ms |
| `get_send_error_count` | 2000 | 0.00 ms | 0.07 ms |
| `get_leader_joint_angles` | 2000 | 0.01 ms | 0.23 ms |
| `get_joint_angles` | 4000 | 0.05 ms | 0.52 ms |
| `get_flange_pose` | 2000 | 0.05 ms | 0.26 ms |
| `get_flange2tcp_pose` | 2000 | 0.02 ms | 0.18 ms |
| `set_motion_mode` | 1 | 0.36 ms | 0.36 ms |
| `set_speed_percent` | 1 | 0.34 ms | 0.34 ms |
| `set_tcp_offset` | 1 | 0.06 ms | 0.06 ms |
| `get_joint_enable_status` | 3 | 0.06 ms | 0.08 ms |
| `enable` | 48 | 0.48 ms | 1.15 ms |
| `disable` | 22 | 0.25 ms | 0.60 ms |
| **`move_mit`** | 1708 | 0.23 ms | **3.32 ms** |
| **`get_firmware`** | 1 | **112.16 ms** | **112.16 ms** |

## Classification

**Bounded, sub-millisecond.** Every read. They are served from the parser's
cache of already-received frames, not from the bus, which is why they cost
almost nothing and why the Phase 0 conclusion — that the per-joint reads were
not the expensive part — holds.

**Bounded, low milliseconds.** `move_mit` at 3.32 ms worst case, an order of
magnitude above its own mean because it is a real CAN transmit. It is also the
call the hot path makes most often, so it sets the practical floor for safety
latency during streaming.

**Long, single call.** `get_firmware` at 112 ms. Startup and recovery only,
never on the hot path, but it is 30× the next worst thing and would dominate any
stop queued behind it.

**Measured 2026-08-13 under a real bus fault** (`ip link set can_nero_right
down`, 15 s, then up):

| call | n | mean | max |
| --- | --- | --- | --- |
| `connect` | 4 | 8.8 ms | 10.1 ms |
| **`disconnect`** | 3 | **666 ms** | **1000.2 ms** |
| `get_firmware` (under fault) | 1 | 175 ms | 175 ms |
| `enable` | 5 | 0.4 ms | 1.05 ms |

`disconnect` is the finding. **It is a single SDK call that blocks for a
second**, which is the one thing the earlier conclusion said did not exist — the
claim that individual calls are almost all sub-millisecond holds for every call
on the hot path and fails exactly here, on the recovery path.

## The findings that matter

The review framed the risk as "the safety lane cannot preempt a vendor SDK call
already blocking on the worker thread". Measuring produced two hazards, not one,
and the first measurement found only the second of them.

**On the hot path, no single call is the hazard.** Every call the running system
makes is sub-millisecond except `move_mit` at 3.32 ms. What threatens the stop
path there is the driver's *composite* operations, which would naturally be
submitted as one worker task:

| operation | what it is | bound |
| --- | --- | --- |
| `_enable_arm` | `while not enable(): sleep(0.01)` plus a readback poll | `enable_timeout`, default **5 s** |
| `_wait_for_firmware` | polls `get_firmware` until it answers | `enable_timeout`, **5 s** |
| `_wait_motion_done` | polls `get_arm_status` | **5 s** |
| `_recover_bus` | disconnect, link reset, connect, re-arm, wait | multi-second, unbounded |

`enable` was measured as 48 calls of at most 1.15 ms each — the blocking is in
the loop, not in the call.

**Off the hot path, one call is.** The 2026-08-13 fault run found `disconnect`
blocking for a full second in a single call. The first measurement missed it
because it never provoked a fault, and stating "individual calls are almost all
sub-millisecond" as a general property was therefore an overreach: it is a
property of the calls a *healthy* system makes.

## The rules this produces

**1. The worker's unit of work is one SDK call, never a retry loop.** Composite
operations stay on their calling thread and submit each iteration separately, so
the safety lane interleaves between iterations rather than waiting for the whole
operation. A loop submitted as a single task converts a 1 ms call into a 5 s
block on the stop path.

**2. One call per task is necessary but not sufficient.** `disconnect` is one
call and blocks for a second. Routing recovery through the same worker as the
safety lane would put a 1 s head-of-line block directly in front of an emergency
stop, which is thirty times the budget below and is not fixable by splitting
tasks more finely.

**Therefore recovery does not share the worker with the safety lane.** The
justification is not convenience: during recovery the link is being torn down,
so an emergency stop could not reach the hardware through it anyway. What
protects the arm in that window is the damped MIT zero the driver already sends
*before* the teardown, and the fault lockout it latches afterwards. That
sequence, not the queue, is the safety story during recovery — and the budget
below says so explicitly rather than implying a guarantee it cannot keep.

## Budget

- **Emergency stop reaches the SDK within 20 ms** while normal work is running.
  Derived from the worst hot-path call (`move_mit`, 3.32 ms) plus margin for one
  in-flight call and the queue hand-off. This is the number the L3 stress test
  has to demonstrate.
- **Declared exception: recovery.** `get_firmware` (175 ms) and `disconnect`
  (1 s) are excluded, because they only run while the link is being torn down
  and re-established. A stop in that window is covered by the damped MIT zero
  the driver sends before the teardown and by the fault lockout after it — not
  by the queue. The exception is stated here so nobody reads the 20 ms as
  unconditional.
- Queue wait and SDK execution are already timed separately (`sdk_queue_wait`
  versus `sdk.<call>`), so a budget violation says which of the two caused it.

## What the fault run also showed, and what fixing it changed

The first fault run (2026-08-13, recovery inline) blocked the publish loop for
**13.1 seconds** — `publish-loop overrun: 13078 ms gap`. Nothing published
state, nothing drained the CAN RX socket, and nothing accounted for time.

Recovery was moved onto its own thread and the fault was provoked again the same
day, same interface, same 15 s down:

| | inline recovery | recovery off the loop |
| --- | --- | --- |
| publish-loop overruns | 1, **13078 ms** | **none** |
| authority publications across 75 s spanning the fault | — | **2384 (31.8/s)** |
| recovery duration | 13.1 s, 3 attempts | 13.1 s, 3 attempts |
| `disconnect` attributed to | the publish thread | `recovery-arm_right` |

The recovery still takes 13.1 s: three attempts at a `disconnect` measured
991-998 ms each is the vendor and the hardware, not the scheduling. What changed
is that it no longer costs the acquisition path — the loop kept publishing at
31.8/s straight through, and the metrics now attribute the blocking calls to the
recovery thread by name, which is the ownership split made visible.

The honest-reporting fix from 2026-08-12 held under a real fault:
`CAN bus recovery verified on attempt 3: feedback advancing, joints enabled:
confirmed by readback` — feedback and the enable readback named separately,
rather than the bare "recovery succeeded" the 0E run produced.

## Routing the acquisition batch (2026-08-13)

The batch is now acquired as **one** worker task rather than eight separate
ones. That was a correction to this document's own rule: "one SDK call per task"
was too crude a statement of the real principle, which is *bounded* work. Seven
motor-state reads plus pose and status measure 0.10-0.20 ms in total — less than
one `move_mit` at 3.32 ms — and nothing about them retries or waits on the bus.
A retry loop bounded only by a 5 s timeout is the thing that must never be one
task; a fixed batch of bounded reads is not.

Measured with the batch routed, MIT streaming at 100 Hz:

| | |
| --- | --- |
| `sdk.acquire_feedback_snapshot` | mean 0.43 ms, max 1.03 ms |
| `sdk_queue_wait` | mean 0.47 ms, **max 5.05 ms** |
| `publish_batch` | mean 0.84 ms, max 2.18 ms |
| acquisition cycles | 2000 in 10 s (200/s) |
| MIT command rate | 100.2/s, unchanged |

The queue hand-off costs 0.47 ms on average against a 5 ms period, and its
worst case reaches the whole period. That is affordable at one hand-off per
cycle and would not have been at eight — which is the measurement that decided
the design.

### The exit gate, measured twice (2026-08-13)

**First attempt**: `from 2 thread(s)`. The batch had moved, but the loop's own
health checks had not — `is_ok`, `has_comm_error`, `get_send_error_count`,
`get_joint_angles` ran on the publish thread *before* acquisition, because they
decide whether to acquire at all.

**After acquiring first and deciding second**, with the health values carried in
the snapshot they judge:

```
sdk calls: 49975 (4995/s) from 1 thread(s): sdk-arm_right
publish_batch:                 mean 0.92 ms, max 2.95 ms
sdk.acquire_feedback_snapshot: mean 0.51 ms, max 1.36 ms
sdk_queue_wait:                mean 0.35 ms, max 3.71 ms
```

The **acquisition path** now has exactly one owner. That is the half of the gate
this work closes.

**The command path does not.** A window with MIT streaming reports:

```
sdk calls: 50980 (5096/s) from 2 thread(s): MainThread, sdk-arm_right
    move_mit[MainThread]: 7000 (700/s)
    set_auto_set_motion_mode_enabled[MainThread]: 33/s
    set_motion_mode[MainThread]: 1
```

`_move_mit_callback` still reaches the SDK directly from the subscription
thread, which is the largest single writer in the system at 700 calls/s. So
"exactly one SDK owner at any instant" holds for reads and **not** for commands,
and Phase 1A does not close until the command path is routed too — the first
item in the review's own list.

### Routing the command path (2026-08-13, not yet measured)

The MIT setpoint is now written by the session owner: `_move_mit_callback`
validates on its own thread and submits `send_mit_setpoint` — the mode bracket
plus the per-joint frames — as **one** task, keyed `move_mit` so a superseded
setpoint is dropped while queued rather than delivered late.

One task rather than nine follows the same rule the acquisition batch settled:
*bounded*, not literally one call. Nine sub-millisecond calls, no retry, nothing
waiting on the bus. The measurement that decided it is already in this document
— a queue hand-off costs 0.35-0.47 ms mean, which is affordable once per cycle
and was explicitly recorded as *not* affordable eight times over. Nine times per
MIT message at 100 Hz would be worse than the race it removes. The setpoint is
also one unit: splitting it would interleave the joints of two messages and run
other work inside the auto-mode-ctrl bracket.

The ingress gate stopped reading the SDK in the same change. `_check_can_control`
ran `_check_arm_ready` — a blocking worker round trip — once per command at the
control rate, and read `get_arm_status` straight off the session for the
teach-mode check, which was the second owner the counter reported. Both now
decide on the publish loop's latest acquisition, refusing when none is younger
than `feedback_timeout`. That refusal is new behaviour and is not a stop: the
firmware holds its last setpoint, so it prevents new motion only.

### What the hardware said about that (2026-08-13, right arm, MIT hold)

The gate passed and the timing did not.

```
sdk calls: 34047 (3403/s) from 1 thread(s): sdk-arm_right
    move_mit[sdk-arm_right]: 6999 (700/s)
```

The 700 calls/s that were on the subscription thread are on the worker, and the
MIT rate held at 100.1/s with no rejections. But:

| | idle | under MIT hold |
| --- | --- | --- |
| acquisition cycles | 193/s | **93-96/s** against a 200 Hz target |
| `feedback/joint_states` | — | **84.7/s** |
| `sdk_queue_wait` mean | 0.34 ms | **2.93 ms** (max 21.2 ms) |
| `sdk.send_mit_setpoint` | — | mean **6.35 ms**, max **21.38 ms** |

**The stop budget was blown by the routing that was supposed to protect it.**
`send_mit_setpoint` as a single task is up to 21.4 ms of work nothing preempts —
more than the whole 20 ms budget, inside one queue entry.

The prediction above was wrong on a specific point: this document argued the
setpoint was "nine sub-millisecond calls". Measured, `move_mit` is 0.86 ms mean
and up to 19 ms — seven of them are ~6 ms. The acquisition batch is nine *cached
reads* costing 0.56 ms in total; a setpoint is seven *CAN transmits*. Treating
the two as the same kind of bounded work is what produced the wrong call.

### The rework, and what it measured (2026-08-13)

Three changes, all forced by the numbers above:

1. **The setpoint is a cycle, not a task.** One queue entry for the epoch check
   and the supersede, executed one frame at a time with the safety lane drained
   between frames. Seven independent submissions would have been the other
   error: two setpoints interleaving leaves the arm holding half of each.
2. **Four lanes** — `SAFETY`, `CONTROL`, `ACQUISITION`, `DIAGNOSTIC` — in strict
   priority. With one lane for everything that is not a stop, a 200 Hz read
   queued behind every setpoint on equal terms with status polling.
3. **The acquisition cadence is its own number.** It followed `pub_rate`, which
   is a ROS publication rate and was never a statement about how fresh a
   hardware reading must be. It is now `acquisition_rate_hz`, default 100 Hz,
   justified by the consumers: the MIT controller runs at 100 Hz and the
   recovery watchdog decides on `feedback_timeout`, which is seconds.

Measured on both arms, MIT hold, metrics on:

| | right (1.06, default tier) | left (1.11, `NeroFW.V111`) |
| --- | --- | --- |
| SDK threads | **1** (`sdk-arm_right`) | **1** (`sdk-arm_left`) |
| `move_mit` | 700/s, mean 0.61 ms, **max 10.48 ms** | 700/s, mean 0.47 ms, **max 5.54 ms** |
| `sdk.send_mit_setpoint` | 100/s, mean 4.70 ms, max 13.26 ms | 100/s, mean 3.70 ms, max 10.75 ms |
| `sdk_queue_wait` | mean 1.68 ms, max 29.46 ms | mean 1.97 ms, max 17.78 ms |
| acquisition cycles | 98/s against a 100 Hz target | 97.5/s |
| `feedback/joint_states` | 98.8/s | 97.4/s |
| command rejections | 0 | 0 |
| CAN errors / drops / bus-off | 0 | 0 |

The behaviour is the same on both protocol tiers, which is what the left arm was
run to establish.

**The non-preemptible unit is now one `move_mit`** — max 10.48 ms right,
5.54 ms left — instead of the whole 21.4 ms setpoint.

**The 20 ms budget is still not claimed as restored.** The tail moved but did not
disappear, and more importantly stop latency has never been measured at all;
what is measured here is the longest thing a stop can queue behind. The L3
stop-latency stress test is the evidence, and it has not been run.

### One number left unexplained, now explained

The earlier 2000-cycles-versus-141-delivered gap was not a QoS depth question.
The loop was not making its configured rate: at a 200 Hz target under MIT load it
achieved 93-96/s and delivered 84.7/s. At a justified 100 Hz it acquires ~98/s
and delivers 97-99/s. Production, not delivery, was the difference.

### Two measurement defects found while measuring

Both inflated counts, neither changed behaviour, both are fixed:

- **Step names collided with SDK call names.** A cycle step named `move_mit`
  was counted by the worker *and* by `MeasuredSdk`, reporting 1400 frames/s for
  700. Cycle steps are no longer instrumented: the wrapper already counts every
  real call, and the step is a preemption boundary, not a measurement unit.
- **The acquisition batch counted its own reads on top of the wrapper**, which
  is pre-existing and predates the routing work. It reported 14 motor-state
  reads per cycle for seven joints.

**Earlier totals in this document are inflated by roughly 2× on the read
counts** — the "4995/s" and "5096/s" figures include the double counting. The
per-call *durations* are unaffected; only the counts and the totals derived from
them are wrong. Corrected totals under MIT hold are ~2560/s per arm.

A third caution, not a defect: `count_topic_messages.py` on `/control/move_mit`
read 75.9/s in one window while the driver processed 1001 setpoints in the same
10 s. That is subscriber-side loss in the counting script, not a control-rate
drop — the driver-side task count is the authoritative number for what reached
the arm.

## The stop-latency stress test (2026-08-13, both arms) — the budget is met

Until this run the safety lane had **no users**. The emergency stop made its SDK
calls straight from the service thread, so it was not ahead of the setpoint
stream, it was beside it — and the 20 ms budget was a statement about nothing.
The stop ladder now runs on the safety lane: the damped zero as one cycle, and
the pose read, the hold and the electronic stop as single calls.

Recovery deliberately does **not** go through the lane. It has already quiesced
the worker and taken the session, so a submission would wait for a handover that
does not complete until recovery ends; recovery calls the SDK directly because
at that moment it is the owner.

Method: driver plus MIT controller holding at the current pose, MIT streaming at
100 Hz, eight stop cycles three seconds apart. Each cycle is stop → clear the
fault lockout → re-enable and re-hold, which is also the enable churn the
checklist asks for. `sdk_queue_wait.safety` is the budget number: how long the
stop waited behind work already executing. The service round trip is **not** that
number — it includes up to 0.5 s of feedback verification, which is the stop
proving itself, not the stop being delayed.

| | right (1.06) | left (1.11) |
| --- | --- | --- |
| **`sdk_queue_wait.safety` worst case** | **0.94 ms** | **0.55 ms** |
| `damped_stop_mit` cycle, 8 frames | 0.75-2.88 ms | 0.56-1.23 ms |
| stops verified in feedback | **8 / 8** | **8 / 8** |
| escalations to electronic stop | 0 | 0 |
| SDK threads, steady state | 1 | 1 |
| CAN errors / drops / bus-off | 0 | 0 |

**An emergency stop reaches the SDK within 1 ms while a 100 Hz MIT stream is
running, against a 20 ms budget.** Worst observed from submission to the last
damped frame on the wire is under 4 ms. The margin comes from the lane, not from
luck: the stop overtakes queued setpoints by priority rather than by the queue
happening to be short.

The queue wait is now recorded per lane. One aggregate could not answer this —
it averaged the safety lane together with diagnostic reads that are *supposed*
to wait behind the control stream.

### The exit gate, read under the full stack

Every steady-state window reports `from 1 thread(s)`. The one window that
reports two covers construction: `connect`, `get_firmware`, `enable`,
`get_joint_enable_status`, `set_speed_percent` and `set_tcp_offset` run in
`__init__` on `MainThread` before the node serves anything. Those are the only
calls ever attributed to it — there are no steady-state calls off the worker.

### What the deeper TX queue changed

Both arm buses were raised from the kernel default of 10 to `txqueuelen 1000`
between runs. Same scenario, right arm:

| | `txqueuelen 10` | `txqueuelen 1000` |
| --- | --- | --- |
| `sdk.move_mit` mean | 0.61 ms | **0.38 ms** |
| `sdk.move_mit` max | 10.48 ms | **5.35 ms** |
| `sdk.send_mit_setpoint` mean | 4.70 ms | **3.01 ms** |
| `sdk.send_mit_setpoint` max | 13.26 ms | 10.77 ms |

The hypothesis below holds in part: the deeper queue roughly halves both the mean
per-frame cost and the tail. It does not remove them — a residual of ~5 ms worst
case survives, which is arbitration against the arm's own feedback push and no
queue depth fixes that. For the 200-250 Hz target (C2) that residual is the
thing to attack, and batching frames per SDK call is the candidate.

**`activate_duo_can.sh` still sets no `txqueuelen`.** These runs used a manual
`ip link set`, so the supported bring-up does not yet produce the configuration
these numbers were taken under. That is a bring-up defect, recorded in the
checklist.

### Why one `move_mit` costs what it does — a hypothesis, partly confirmed

`move_mit` packs one frame and calls `comm.send()`; `_send_msgs` is a plain
Python loop over the same, so the SDK has **no batched socket write** to reach
for. The likely cost is not CPU but TX backpressure: `can_nero_right` and
`can_nero_left` run at `txqueuelen 10`, the kernel default, while a setpoint is
a burst of seven frames at 100 Hz onto a bus already carrying ~2800 frames/s of
feedback push. The repo's older `activate_native_can.sh` sets 1000 and documents
exactly this — "an arm command burst (7 MIT frames per control cycle) can
overrun it" — but the supported four-bus bring-up, `activate_duo_can.sh`, sets
no `txqueuelen` at all.

The experiment is one line (`ip link set <iface> txqueuelen 1000`, then re-read
`sdk.move_mit`) and has **not been run**. If it is the cause, it matters well
beyond this: the C2 target of 200-250 Hz doubles the burst rate, and batching
frames per SDK call would then be the deeper fix.

### One number left unexplained

The loop ran 2000 acquisition cycles in 10 s, while a subscriber counted 141.4
`feedback/joint_states` per second. The publisher's depth is 1 and the
subscriber's is 50, so delivery rather than production is the likely difference
— but `ros2 topic info --verbose` reports the depth as UNKNOWN here, so this is
untested and recorded as open rather than explained away.

## Accepted limit

The recovery window is not a defect to be closed by better scheduling. A vendor
call that blocks for a second blocks for a second, and a stack cannot command an
arm through a link it is tearing down. The mitigation inside the software — the
damped zero before the teardown, the lockout after — is what there is, and it is
a mitigation rather than a guarantee.

This is recorded as the concrete, measured requirement for the independent
hardware watchdog in `docs/open_questions.md`: authority over the devices during
a window of roughly ten seconds in which this stack is provably unable to
command them, recurring on every transport fault.

## Still open

Superseded 2026-08-13: the L3 stress test has been run and the budget is met
(see the stress-test section above), and "enforce the one-call rule" is no
longer the right item — the rule itself was wrong. The unit of work is bounded
work, and for a command that is several transmits but one instruction it is a
*cycle*, enforced by `submit_cycle` rather than by a call-count check.

What remains:

- **the RX-socket drain under a fault.** The loop now survives recovery without
  overruns, but whether it drains the socket fast enough while a fault is in
  progress is a separate measurement, and it belongs to Phase 1B.
- **the residual per-frame transmit cost.** ~5 ms worst case survives a
  1000-frame TX queue and is arbitration against the arm's own feedback push.
  It is not a safety problem at 100 Hz; it is the thing that decides whether the
  200-250 Hz target (C2) is reachable without batching frames per SDK call.
- **`activate_duo_can.sh` sets no TX queue depth**, so the supported bring-up
  does not produce the configuration the numbers above were taken under.
- **the service handlers and the quarantined legacy motion paths** still hold
  the session directly. Off the hot path, and the legacy paths are off by
  default, but a development profile that enables them puts a second writer on
  the session — so the one-owner property is conditional on the profile, not
  unconditional.
