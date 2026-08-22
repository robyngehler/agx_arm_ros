# Arm feedback rate budget: what the wire actually delivers

date: 2026-08-22
platform: Jetson AGX Orin, both arms holding at the idle pose, MIT streaming at
198 Hz, full stack up (drivers, MIT controllers, hand bridges, MoveIt, RViz)
method: `candump` on the raw SocketCAN interface, cross-checked against
`/sys/class/net/*/statistics` — chosen because it sits *below* pyAgxArm, so the
SDK cannot be the reason a frame is missing from it. Topic-level cadence from
the frame timestamp carried in `feedback/joint_states`, which is the arm's frame
instant and not the moment the driver read it.

## Why this exists

Acquisition, publication and teach recording were all configured to 200 Hz on
the assumption that the arm supplies data that fast. It does not. Everything
above the wire was manufacturing duplicates, and no instrumentation in the stack
could see it: the driver's acquisition loop reported ~180 Hz because it *ran*
that fast, while the frames it read had not changed.

This note records the measured ceiling so a future rate choice starts from it.

## One state update is eleven CAN frames

A frame count is not an update count. `_acquire_feedback_snapshot` reads two
groups:

| group | IDs | frames | content |
| --- | --- | --- | --- |
| joint positions (`get_joint_angles`) | `0x2A5`, `0x2A6`, `0x2A7`, `0x2A9` | 4 | two joints per 8-byte frame, seven joints |
| motor states (`get_motor_states` 1..7) | `0x251`–`0x257` | 7 | position, velocity, current, torque per joint |

The rest of the traffic is not read per cycle: `0x2A1` arm status, `0x2A2`–
`0x2A4` end pose (xy / zrx / ryrz), `0x2A8`, and `0x261`–`0x267` driver states
(the vendor's "LowSpd", ~38/s).

Left arm, 5 s, adding up to the kernel's count:

```
0x2A1 239 + (2A5,2A6,2A7,2A9) 153x4 = 612 + (251-257) 153x7 = 1071
0x2A8 152 + (2A2,2A3,2A4)      60x3 = 180 + (261-267)  38x7 =  266
                                                       total = 2520/s
```

So ~2520 frames/s is ~150 complete state updates/s, not 2520.

## Measured, 15 s per window

| | left (`can_nero_left`, FW 1.11) | right (`can_nero_right`, FW 1.06) |
| --- | --- | --- |
| positions `0x2A5`/`0x2A6`/`0x2A7`/`0x2A9` | 140.7 uniform | 101.6 / 101.4 / 101.2 / 100.7 |
| motor states `0x251`..`0x257` | 136.2 uniform | 101.0 / 100.4 / 100.4 / 99.9 / 97.7 / 83.3 / **63.4** |
| total RX (kernel) | 2556/s | 2520/s |
| total TX (kernel) | 1582/s | 1589/s |

`candump` saw 2551/s and 2517/s against those kernel counts, so it is a faithful
witness. It does **not** show the TX loopback — a TX rate has to be read from
`/sys/class/net/<iface>/statistics/tx_packets`, and reading a capture as "no
commands on the bus" is a mistake this measurement made once.

TX is the MIT stream: 198 Hz x 8 frames. `get_motor_states` is a cache read
(`getattr(self._parser, ...)`), so acquisition costs CPU and no bandwidth.

## The bus is not the limit, and 2 kHz is not reachable

Every received frame is classic CAN with 8 data bytes; zero CAN FD frames were
observed, though both interfaces are FD-capable (`dbitrate 5000000`). At the
1 Mbit/s arbitration rate a frame costs ~125 bit with stuffing and IFS, so the
bus carries ~8000 frames/s.

Current load is RX 2520 + TX 1582 = 4102 frames/s, about 51%. The headroom is
therefore roughly 2x, not the 20x that 2 kHz would need: eleven frames at
2000 Hz is 22,000 frames/s, ~2.75x the whole bus.

The interface is clean and is not the loss: 304 drops in 18.9M packets
(0.0016%), `missed 0`, `errors 0`, `bus-errors 0`, `arbit-lost 0`, never
`error-warn`.

**There is no rate knob.** The Nero transmit set has no feedback-period message.
`0x477` byte 2 toggles only the `0x48X` end-V/acc report; `0x151` byte 6 is a
boolean CAN-push enable, which `nero_can_push.set_can_push` already sets to
ENABLE at startup.

Where a 2 kHz expectation belongs: MIT closes the loop **in the joint**. The MIT
frame carries position, velocity, kp, kd and torque, and the joint controller
evaluates them locally at its own internal rate. The CAN feedback rate bounds
monitoring, recording and any Jetson-side outer loop — not the servo loop.

## The right arm degrades along the joint chain

The right arm's motor-state rate falls monotonically with the joint index. The
left arm's does not, at a higher rate, under the same bus load. Stopping the MIT
stream on the right arm only (the left kept streaming on its own bus as a
within-run control) separates our traffic from the arm:

| motor state | right, MIT on | right, MIT off | left, MIT on |
| --- | --- | --- | --- |
| `0x251` (J1) | 101.0 | 105.1 | 136.2 |
| `0x254` | 99.9 | 104.3 | 136.2 |
| `0x255` | 97.7 | 102.0 | 136.2 |
| `0x256` | 83.3 | 91.5 | 136.2 |
| `0x257` (J7) | **63.4** | **79.2** | **136.2** |

Removing 1353 frames/s of our own traffic recovers J7 by 25% and narrows the
J7-vs-J1 deficit from 37% to 25%. **Both causes are real**: bus contention from
the MIT stream accounts for about a third, and the remainder is internal to the
right arm — the left arm carries the same TX load on its own bus and shows no
falloff at all. Joint positions (`0x2A5`–`0x2A9`) stay uniform on both arms; only
the motor-state group degrades.

Consequence: velocity and effort in `feedback/joint_states` for the right arm's
distal joints are less than half as fresh as the left arm's. Whether that
reaches the control law is unresolved — in MIT mode the joint damps locally, so
the Jetson-side velocity may not enter the command at all.

Cause is unattributed. Right is FW 1.06 and left is FW 1.11, which fits the
per-tier rule, but a firmware difference and a difference in that individual arm
are not separated by this measurement.

Method note: the test is reversible and does not move the arm. The MIT control
loop returns without publishing when disabled, and the firmware holds the last
setpoint. TX went 1353 -> 0 -> 1347/s and the pose was unchanged to within
7e-5 rad.

## The chain above the wire

Right arm, same session:

| stage | rate |
| --- | --- |
| CAN wire, joint positions | 101/s |
| distinct frames on `feedback/joint_states` | 77.6/s |
| messages published on `feedback/joint_states` | 129.4/s |
| acquisition loop iterations | 173.7–186.7/s |
| `control/move_mit` | 198.4/s |
| teach recording | 200/s nominal |

A 200 Hz recording made in this session carried **33.4% identical consecutive
samples**. The wire-to-topic loss (101 -> 77.6 distinct) is software and is not
yet attributed; two candidates are the publication loop sampling at 200 Hz and
emitting at most one snapshot per tick (its receive gaps quantize at ~5 ms and
~10 ms, the signature of that), and the silent `return` paths in
`_publish_joint_states` that discard a snapshot after `published_at` has already
been advanced.

## What follows from this

- the honest source rate is ~100 Hz for the right arm and ~137 Hz for the left;
  a duo recording is bounded by the slower one
- a rate configured above the source does not add information, and costs file
  size, dispatch time and CPU
- the driver's stall threshold (`max(2 / acquisition_rate_hz, 0.2)` = 200 ms) is
  structurally blind to a shortfall of this size; the periodic achieved-rate
  report added in `0ed2f1e` is what makes it visible
