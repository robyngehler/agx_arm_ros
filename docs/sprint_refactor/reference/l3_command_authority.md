# L3: hand command authority on hardware (2026-08-17)

Evidence for the Phase 4D command stamp, taken on the right hand against the
vendor SDK backend on `hand_right`, plus the four-bus census that accompanied
it. Level L3. Script: `scripts/l3_hand_command_authority.py`.

## What this settles

Before the stamp, the bridge built a command's identity from its own current
epoch and its own counter, then checked that identity against the state it came
from. The stale-epoch and out-of-order checks compared each value with itself
and passed unconditionally — the code ran and could refuse nothing. That was
demonstrable at L1 only; this is the same claim on a real device.

**The run does not move the hand.** Every command carries the hand's own
measured pose as its target, so admission is exercised end to end while the
commanded position is the one the hand is already in.

| Claim | Result | Evidence |
| --- | --- | --- |
| A correctly stamped command is admitted | PASS | no refusal for owner + current epochs + fresh sequence |
| A foreign owner is refused | PASS | `not_owner: 'reactive:someone_else' is not the commander` |
| An out-of-order sequence is refused | PASS | `stale_sequence: sequence 1 <= 1` |
| An unknown unit-safety generation is refused | PASS | `unknown_unit_epoch: unit epoch 5 != 0` |
| A stale device epoch is refused after a handover | PASS | epoch 2 -> 4, `stale_device_epoch: device epoch 2 != 4` |
| The current epoch is still admitted afterwards | PASS | no refusal — the gate is selective, not shut |

Each refusal carried its own structured reason, which is what distinguishes this
from a gate that simply rejects everything.

## Four-bus census taken in the same session

Both hand bridges up on the SDK backend, arms not running:

| Interface | Frames/s | RX errors | RX dropped | RX missed |
| --- | --- | --- | --- | --- |
| `hand_right` | 50 | 0 | 0 | 0 |
| `hand_left` | 48 | 0 | 0 | 0 |
| `can_nero_right` | 0 | 0 | 0 | 0 |
| `can_nero_left` | 0 | 0 | 0 | 0 |

Hand-bridge CPU, steady state, both hands: **30.2 % of one core for the pair**
(15.3 % / 14.8 %), consistent with the 25.3 % recorded after the vendor spin-loop
patch and against 319 % before it. Exactly one `sdk-<device>` worker thread per
bridge, which is how the single-owner invariant is read.

## Findings

- **`hand_left` answers again.** The Phase 0E item "capture both sides
  arm-and-hand in parallel" was blocked on a `hand_left` cable fault. On this
  date the interface is up, the device answers, and its bus carries 48 frames/s
  with zero errors and zero drops. The blocker is not reproducible; the
  remaining half of that capture needs arm motion, not a cable.
- **The owner-liveness revocation works, and it bites test harnesses.** The
  first version of this script declared itself `reactive:l3_authority` while its
  node was named `l3_hand_command_authority`. The bridge revoked the claim
  mid-run, and the next command was refused as "no commander" — correct
  behaviour, but it masked the check under test. An owner id's node half has to
  be the real node name.
- **Refusal logging is throttled by identical detail string** (5 s). Counting
  refusals in a test therefore needs distinct reasons, or it will under-count.
- The acquisition thread is named at the Python level but not at the OS level,
  so it does not appear in `/proc/<pid>/task/*/comm` alongside `sdk-<device>`.
  Observability gap only; per-thread SDK attribution still works because the
  worker is the thread that matters. Not fixed here.

## The arms did not answer (2026-08-17)

Arm motion was authorised for this session, limited to joints 1, 3 and 5 from
the current pose. It could not be attempted: **neither arm answers on CAN.**

| Interface | RX pkts since boot | TX pkts | Bus state |
| --- | --- | --- | --- |
| `can_nero_right` | 0 | 0 | ERROR-ACTIVE |
| `can_nero_left` | 0 | 0 | ERROR-ACTIVE |
| `hand_right` | 6421 | 6421 | ERROR-ACTIVE |
| `hand_left` | 1588 | 1588 | ERROR-ACTIVE |

Both drivers ran the full startup ladder, including the known wake-up path — the
firmware push only starts after an incoming command, so the driver sends
`set_normal_mode` once and retries. Both then reported:

> Failed to get firmware version, also after re-asserting the feedback push. The
> arm is not answering on CAN: check power, E-stop and wiring for this side.

Nothing was received from either arm since the interfaces came up, while both
hands ran on the same host in the same session — so the CAN stack, the adapters
and the ROS graph are healthy.

**The driver's "check power, E-stop and wiring" is not the whole story, and
probing further changed the answer.** The arm's feedback push only starts after
it receives a command, and that command never reaches the wire:

- Both ROS services that could send it — `enable_agx_arm` and `set_normal_mode`,
  the documented wake sequence used by `agx_arm_mit_tools/test_position_hold.py`
  — refuse with **"Agx_arm is not connected"**. That guard is
  `_check_arm_connected()`, which reads `is_ok()`, which is false because no
  feedback has arrived. The wake command is gated behind the feedback it exists
  to start.
- Bypassing every ROS guard and driving the vendor SDK directly does not help.
  `connect()` succeeds and the socket is genuinely bound to the right interface
  (`can_nero_right` appears in `/proc/net/can/rcvlist_all`), `set_normal_mode()`
  returns without raising, and `get_firmware()` returns `None` — while
  **TX packets stay 0 on every CAN interface on the host**, with TX errors 0 and
  TX dropped 0. Nothing was handed to the controller at all.

For contrast, `hand_right` shows TX 6421 in the same session, so the host can
transmit. Error counters do not discriminate here: `berr-counter tx` is 0 on the
hand bus too, because successful transmissions accumulate no errors.

So the deadlock is real and sits below our code: the SDK appears to gate its
sends on a link-health flag it can only satisfy from received feedback. Whether
an arm is *also* unpowered cannot be determined from this host, because no frame
is put on the wire either way. Both need checking, and the software half is a
vendor-fork question under C3 rather than something the driver can fix above it.

## Not covered by this run

Concurrent arm-and-hand motion per side and across sides, and stop/rearm during
active motion, need arms that answer. The reactive-versus-trajectory handover on
a physical grasp additionally needs an object placed in the hand. All remain
open in the Phase 2B/2C acceptance list.
