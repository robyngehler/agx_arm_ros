# The emergency stop ladder ends at the MOVE-J hold (2026-08-20)

The canonical statement of what the Jetson side may command when it stops an
arm, and why the ladder stops where it does.

## The rule

**No safety path issues `electronic_emergency_stop()`.** The escalation on this
side ends at the firmware `MOVE-J(current_q)` hold, re-asserted up to
`ESTOP_HOLD_ATTEMPTS` times. Beyond it there is transport repair and then the
external CAN watchdog — no stronger *motion* command exists here.

## Why the vendor call is not a stop we want

`electronic_emergency_stop()` sends `ArmMsgMotionCtrl(1)` — CAN `0x150`, byte 0
= `0x01`. Both Nero tiers use the same frame; there is no v111 override. The
vendor driver calls it "a damped emergency stop … applying damping to all
joints", and the Nero API note is explicit about the consequence:

> If the arm joints are in a raised position when executed, the arm will
> **slowly descend with constant damping**.

Damping is not stiffness. That is the same terminal state as a kp=0 damped MIT
command, which this repository already rejected as a stop: it brakes a moving
arm and then lets gravity have it. The intended stopped state is the opposite —
motors enabled, firmware position controller active, current pose held, motion
authority revoked.

## The ladder as it now stands

Per attempt, all on the `SAFETY` lane:

1. damped MIT zero, **only** as a braking transient when a MIT stream is live —
   needs no feedback, has no stiffness, never terminal
2. capture the current pose from trustworthy live feedback
3. `MOVE-J(current_q)`, re-asserted until the firmware's move-mode readback
   positively confirms it left MIT
4. verify in feedback that joint velocities settled

An unverified result **re-asserts the same hold** at the pose the arm is at now,
rather than escalating to a different command. After the attempts are spent the
driver requests a bus-recovery link reset — transport repair, which re-attempts
the hold on its way in — and reports the stop as unverified.

Where no trustworthy pose exists, **nothing is commanded and nothing is
claimed.** A pose synthesised from stale feedback is a wrong hold rather than a
missing one, and a damped descent is worse than either.

## What the escalation used to do, and why it was wrong

Until 2026-08-20 the stop had two rungs above the hold, both the vendor call:

- **no trustworthy pose** → `electronic_emergency_stop()`. Commented as "hard
  stop is the only safe option", but it commands a descent. It also disagreed
  with `_hold_before_teardown`, which answers the identical condition by
  claiming no hold and leaving the regime to the watchdog. The two paths read
  the pose differently as well — the stop demanded `js.hz > 0` where
  `_capture_hold_pose` also accepts advancing frames, so it could drop an arm
  for which a hold was available.
- **stop not verified** → `electronic_emergency_stop()`. `verified` is False
  both when the arm is measurably moving *and* when the measurement produced no
  evidence at all. So a firmware hold that had been positively confirmed, plus a
  feedback hiccup inside the 0.5 s verification window, cancelled the hold and
  commanded the descent. **The escalation undid the thing it was escalating
  from.**

Neither call site was covered by a test. The fake existed in
`test_safety_hold_semantics.py`; nothing asserted the call was or was not made.

## Consequence for the firmware state

`0x150` byte 0 = `0x02` (`reset()`) is what leaves the firmware e-stop state,
and it only takes effect after an e-stop while the arm is enabled. Nothing on
the Nero path ever sent it: the sole `reset()` call sits in the Piper-only
teach-mode exit. `clear_fault_lockout` cleared the ROS-side latch and the
authority while sending nothing to the arm, so an operator could be told
"emergency stop latch cleared" and still find the firmware refusing motion.

Removing the vendor call from the safety ladder closes that asymmetry: no path
on this side puts the firmware into a state that needs a reset we never send.

## Where the boundary actually is

The external CAN watchdog owns the regime this side cannot cover. When the
Jetson's CAN signal stops it takes the bus and disconnects this side, and it is
free to command a descent-type stop — at that point nothing here is holding the
arm anyway. That layer is unbuilt and its shape is undecided
(`docs/open_questions.md`, "Independent hardware emergency stop"). Until it
exists the physical e-stop remains the only guaranteed stop, and the Nero
firmware still has no MIT command watchdog.

The single remaining `electronic_emergency_stop()` call in `agx_arm_ctrl` is the
vendor teach-mode-exit recipe in `_exit_teach_mode_callback` — Piper-only, at the
home pose, paired with the `reset()` that leaves the state again. It is a mode
transition, not a rung of this ladder.

## Validation

L1/L2: `src/agx_arm_ctrl/test/test_safety_hold_semantics.py` pins that no safety
path sends the vendor call, that an unverified stop re-asserts the hold, that a
verified stop holds once, and that a stop without a trustworthy pose commands
nothing. Not exercised on hardware — the last hardware record of the stop path
is `l3_production_estop.md` (2026-08-17), whose gate criteria are unchanged by
this because they were already met by the hold.
