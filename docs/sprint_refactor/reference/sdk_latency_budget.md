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

### The exit gate is not met yet

`sdk calls: 52000 (5199/s) from 2 thread(s)`. The batch moved to the worker
(28000 calls), but the loop's own health checks did not:

```
is_ok[_publish_thread]: 400/s
has_comm_error[_publish_thread]: 200/s
get_send_error_count[_publish_thread]: 200/s
get_joint_angles[_publish_thread]: 200/s
```

Those sit in `_check_arm_ready`, `_check_arm_connected`, `_surface_silent_tx_loss`
and `_should_recover_bus`, which run *before* the acquisition and decide whether
to recover at all. Folding them into the snapshot means acquiring first and
deciding second — a loop restructure, not a call move. Until it lands, "exactly
one SDK owner at any instant" is **not** satisfied and Phase 1A does not close.

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

- the L3 stress test itself: stop latency under concurrent MIT streaming,
  recovery, and enable churn;
- enforce the one-call rule in the worker rather than only documenting it;
- the RX-socket drain itself: the loop now survives recovery, but whether it
  drains fast enough under a fault is a separate Phase 1B measurement.
