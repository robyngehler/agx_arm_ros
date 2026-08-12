# SDK call latency budget for the serialized worker

date: 2026-08-12
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

**Not measured.** `connect` and `disconnect` on the recovery path. The Phase 0
fault test showed multi-second recovery, so they must be treated as unbounded
until measured.

## The finding that matters

The review framed the risk as "the safety lane cannot preempt a vendor SDK call
already blocking on the worker thread". Measured, that framing understates one
thing and overstates another: **individual SDK calls are almost all
sub-millisecond**, so a single call is not the hazard. The hazard is the
driver's *composite* operations, which would naturally be submitted as one
worker task:

| operation | what it is | bound |
| --- | --- | --- |
| `_enable_arm` | `while not enable(): sleep(0.01)` plus a readback poll | `enable_timeout`, default **5 s** |
| `_wait_for_firmware` | polls `get_firmware` until it answers | `enable_timeout`, **5 s** |
| `_wait_motion_done` | polls `get_arm_status` | **5 s** |
| `_recover_bus` | disconnect, link reset, connect, re-arm, wait | multi-second, unbounded |

`enable` was measured as 48 calls of at most 1.15 ms each — the blocking is in
the loop, not in the call.

## The rule this produces

**The worker's unit of work is one SDK call, never a retry loop.** Composite
operations stay on their calling thread and submit each iteration separately, so
the safety lane interleaves between iterations rather than waiting for the whole
operation. A loop submitted as a single task converts a 1 ms call into a 5 s
block on the stop path.

## Budget

- **Emergency stop reaches the SDK within 20 ms** while normal work is running.
  Derived from the worst hot-path call (`move_mit`, 3.32 ms) plus margin for one
  in-flight call and the queue hand-off. This is the number the L3 stress test
  has to demonstrate.
- `get_firmware` and the recovery calls are excluded from that budget because
  they are startup and recovery paths; a stop during recovery is covered by the
  driver's own damped stop before the link is torn down, not by the queue.
- Queue wait and SDK execution are already timed separately (`sdk_queue_wait`
  versus `sdk.<call>`), so a budget violation says which of the two caused it.

## Still open

- measure `connect` and `disconnect` under a real bus fault;
- the L3 stress test itself: stop latency under concurrent MIT streaming,
  recovery, and enable churn;
- enforce the one-call rule in the worker rather than only documenting it.
