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

## The arms did not answer (2026-08-17, RESOLVED same day)

**Resolved:** the Jetson 40-pin header pinmux had been discarded by a kernel
update. Once it was reconfigured with `sudo /opt/nvidia/jetson-io/jetson-io.py`
both arms came up immediately — left firmware 1.11 (`NeroFW.V111`), right 1.06
(`NeroFW.DEFAULT`) — and every blocked hardware case below was then run. The
diagnostic reasoning is kept because it is what the next occurrence looks like,
and because two of its readings were wrong; see `docs/errors_and_fixes.md`.

Arm motion was authorised for this session, limited to joints 1, 3 and 5 from
the current pose. It could not be attempted at the time: **neither arm answered
on CAN.**

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

## Why the arms are silent (revised 2026-08-17, after the bootstrap fix)

This section was rewritten twice on the same day as the evidence improved. Both
earlier readings are superseded and are stated here so the reasoning is
followable:

1. *"Check power, E-stop and wiring"* — the driver's own message, correct in
   direction but unproven at the time.
2. *"Nothing our stack sends reaches the wire; the SDK appears to gate its sends
   on a link-health flag."* **Superseded 2026-08-17.** The gating half was a
   hypothesis and it is wrong: the SDK does attempt the send.

### What was actually established

Two things were genuinely broken and one was misread.

**Broken, and fixed (software).** The wake command was gated behind the feedback
it exists to start. `enable_agx_arm` and `set_normal_mode` — the documented
sequence in `agx_arm_mit_tools/test_position_hold.py` — both refused with
*"Agx_arm is not connected"*, from `_check_arm_connected()` reading `is_ok()`,
which is an FPS window over *received* frames and is therefore false for every
arm that is connected but silent. Fixed: a transport session now gates a
bootstrap command, and feedback health gates only what depends on feedback.

**Broken, and fixed (reporting).** With the gate relaxed, `set_normal_mode`
answered `success=True, "Switched to normal mode"` on an arm that answers
nothing — the SDK not raising was being read as the arm having complied. It now
distinguishes *sent* from *confirmed*. In the same run the silent-TX-loss
warning asserted *"while feedback is live"* unconditionally, next to dropped
sends on an arm with no feedback at all; it now states the feedback side from
the snapshot it already holds.

**Misread.** The claim that nothing was handed to the controller. Once the
bootstrap ladder actually ran, the driver reported:

> silent TX loss: 2 send(s) dropped (total 2) and NO feedback is arriving either
> (last: **Transmit buffer full**); arm commands may not be reaching the firmware.

`Transmit buffer full` is ENOBUFS from the socket write. Our stack does hand
frames to the socket; the kernel cannot place them on the wire. Measured on
`can_nero_right` immediately afterwards:

```text
RX: packets 0   errors 0  dropped 0
TX: packets 0   errors 0  dropped 0   carrier 0
state ERROR-ACTIVE (berr-counter tx 0 rx 0)   <ONE-SHOT,FD>
```

Zero completed transmissions, zero errors, and a send path that blocks on a full
buffer. On CAN a frame counts as transmitted only once another node
acknowledges it; with nothing else powered on that bus the controller never
completes the transmit, the queue fills, and `write()` returns ENOBUFS. The
error counters stay at zero because ONE-SHOT aborts the frame instead of
retrying it into error-passive — which is why they are useless as a
discriminator here, and why TX *packets* is the number to read.

**So the remaining fault was below the driver and below the SDK.** It was the
40-pin header: the pins were no longer muxed to the CAN controller, so a frame
was queued and never completed, the socket buffer filled, and `write()` returned
ENOBUFS. Arm power and wiring were never the cause — the third reading was as
wrong as the first two, in the other direction. What survives is the method:
read TX *packets*, not error counters, and treat a healthy hand bus as no
evidence at all, since the hands are on USB-CAN FD adapters that never touch the
header.

Both hands transmitted normally on the same host in the same session
(`hand_right` TX 6421, `hand_left` TX 1588), so the CAN stack, the adapters and
the ROS graph are healthy.

## Arm bootstrap cases that could be completed

Two of the six silent-arm cases need an arm that does *not* answer, so they ran:

- **Case E — feedback cannot be restored.** Both arms. The full ladder now
  executes — push bootstrap, enable request, push re-assert, linkage
  re-assert — and startup then exits with the four facts separated:
  `Transport session: present; feedback-push bootstrap: sent; feedback: none;
  enable: unverified`. No READY, no command admission.
- **Case D — explicit enable service before feedback is alive.** Right arm. The
  command is now *attempted* rather than refused as "not connected", and the
  service reports `Failed to send enable to Agx_arm` — the SDK's `enable()`
  never returning truthy — instead of claiming success. `set_normal_mode`
  reports the mode as sent but unconfirmed.

Cases A, B, C and F ran once the header was fixed; all six are complete.
`scripts/l3_arm_silent_bootstrap.py` proves the push-only primitive on hardware:
`can_nero_right` went 2162.7 RX/s -> 0.0 -> 2167.7 with no mode switch and no
motion command, and a deliberately muted arm then reached READY through startup
alone, without the `set_normal_mode` escalation firing.

## Not covered by this run

Concurrent arm-and-hand motion per side and across sides, and stop/rearm during
active motion, need arms that answer. The reactive-versus-trajectory handover on
a physical grasp additionally needs an object placed in the hand. All remain
open in the Phase 2B/2C acceptance list.

**Re-run 2026-08-17 with legacy ingress off** — the evidence above was taken
while both primitives still published a stamped and a bare copy of every motion.
The repeat passed 6/6 on the shipped path, and the bare surfaces do not exist as
topics on a default bridge while both stamped surfaces carry exactly one
subscriber.

Parallel operation ran the same day: left arm + left hand, right arm + right
hand, and both sides concurrently, with zero errors and zero drops on every bus.
That run used the **quarantined development MOVE-J ingress**; the production
stamped arm path under the same load is the one case still open.
