# L3: production MIT e-stop under parallel hand load (2026-08-17)

The final Refactor Runtime RC gate. Both arms held by streamed MIT torque, both
hands under stamped command load, emergency stop on both. Level L3.

Stack: `start_nero_mit_controller.launch.py` per side (arm driver + MIT
controller), both OmniHand bridges on the vendor SDK backend, all four devices
on their own bus.

## What counts as evidence, and what does not

**Pose drift is not the criterion.** Both arms stood near a gravity-neutral
configuration, where a de-energized arm stays put as well as a held one. The
first version of this run recorded drift of 5·10⁻⁵ rad and read it as proof; it
is not, and the reading was withdrawn.

What discriminates is the firmware's own **move-mode readback** together with
`ctrl_mode`. A de-energized arm cannot report MOVE-J under CAN control.

| | before stop | after stop |
| --- | --- | --- |
| left arm (fw 1.11) | `move_mode=0x06` MIT | `move_mode=0x01` MOVE-J, `ctrl_mode=0x01` |
| right arm (fw 1.06) | `move_mode=0x04` MIT | `move_mode=0x01` MOVE-J, `ctrl_mode=0x01` |

The two MIT codes differ because the tiers differ — 0x06 on v111, 0x04 on the
default tier — which is why the hold verifier takes the code set from the active
driver rather than hardcoding it. Both tiers appear in one run here.

Joint torque was recorded too and is **inconclusive in this pose**: the right
arm's summed |torque| fell to 0.059 N·m after the stop, which is what a
gravity-neutral pose costs to hold. It is reported for completeness, not as
evidence.

## Gate criteria

- **arm remains stiff at the current pose** — positive MOVE-J readback under
  `ctrl_mode=0x01` on both arms
- **firmware leaves MIT, positively confirmed** — 0x06→0x01 and 0x04→0x01; no
  driver logged "NOT in a firmware hold"
- **no `disable()` issued** — zero occurrences in either driver log
- **stale queued MIT work cannot execute after the hold** — arm TX fell to
  0 frames/s immediately after the stop, from ~700 frames/s while MIT streamed
- **`MOVE-J(current_q)` is the terminal hold** — the mode readback above
- **hand-side parallel traffic stays healthy** — `hand_left` 104 TX/s,
  `hand_right` 98 TX/s through and after the stop; zero errors, zero drops on
  all four buses

Both stops reported `stop=verified` with joints settled (peak 0.000 and
0.026 rad/s).

**Refactor Runtime RC is closed.**

## The left-arm stall does not reproduce (proposal §3)

The stall recorded on 2026-08-17 during the first parallel run — bus stall,
three failed recovery attempts, arm offline — was re-run on the current branch
under the same load case: both sides, 20 cycles, quarantined MOVE-J ingress
enabled, both hands loaded.

```text
stall detections        0
recovery attempts       0
silent TX loss events   0
new rx drops            0   (can_nero_left stays at its pre-existing 303)
can_nero_left           2168.1 RX/s   5.0 TX/s   0 err  0 drop
can_nero_right          2132.5 RX/s   5.0 TX/s   0 err  0 drop
hand_left                 85.3 RX/s  85.0 TX/s   0 err  0 drop
hand_right                84.3 RX/s  83.9 TX/s   0 err  0 drop
```

**Classified as pre-fix/confounded evidence.** The original observation predates
routing the quarantined `control/move_j` ingress through the `SdkWorker`; until
then that path called the vendor SDK directly from a subscription callback and
raced the worker, which is a sufficient explanation for an acquisition stall and
is no longer present.

No socket-buffer or recovery-architecture change is made on the strength of the
old reading. If the stall returns, it should be investigated from a fresh
capture rather than from that one.
