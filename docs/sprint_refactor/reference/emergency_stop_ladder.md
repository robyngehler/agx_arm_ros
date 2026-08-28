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

1. capture the current pose from trustworthy live feedback
2. `MOVE-J(current_q)`, re-asserted until the firmware's move-mode readback
   positively confirms it left MIT
3. verify in feedback that joint velocities settled

An unverified result **re-asserts the same hold** at the pose the arm is at now,
rather than escalating to a different command. After the attempts are spent the
driver requests a bus-recovery link reset — transport repair, which re-attempts
the hold on its way in — and reports the stop as unverified.

Where no trustworthy pose exists, the hold cannot be built: a pose synthesised
from stale feedback is a wrong hold rather than a missing one. The rung below it
is a **mode frame, not a setpoint** — `set_normal_mode` needs neither pose nor
feedback, ends the MIT setpoint the firmware would otherwise keep executing, and
hands the arm to its own position controller, which holds it where it is. It is
unverifiable by construction, so it is attempted and reported, never claimed.

## There is no kp=0 rung, at any height (2026-08-28)

A kp=0 MIT command carries no stiffness. It ends a moving setpoint, which is why
it was attractive — it needs no feedback — but what it leaves behind is an arm
with nothing holding it up. **It is not a weaker hold. It is a sag**, and it was
removed from this driver and this controller entirely rather than left available:

| Where it was | What it does now |
|---|---|
| `emergency_stop`, "braking transient" before MOVE-J | straight to `MOVE-J(current_q)`; `set_normal_mode` where no pose exists |
| `_hold_before_teardown`, pre-recovery quiesce | same |
| `prepare_hand_window`, before the mode change | the `set_normal_mode` frame already ends MIT; the damped zero only added a window with no stiffness in it |
| MIT controller, stale-feedback dead-man | requests the driver's `hold_current_pose`; publishes no MIT command |
| MIT controller, shutdown | stiff gravity-compensated hold at the measured pose |

`_submit_damped_stop_mit`, `_send_damped_stop_mit`, `_send_damped_stop_joint` and
`_publish_damped_stop_command` are **deleted**, and tests assert they are not
reachable. An escalation step that exists gets called.

The one surviving kp=0 command is **freedrive**, which is kp=0 *with* the gravity
feedforward — the model carries the arm's weight, which is what makes it
back-drivable rather than limp. Its service refuses to enter without a gravity
model, and that gate is what keeps the prohibition true.

## The ladder, end to end

1. **MIT hold** at the measured pose — the controller's own, while it has feedback
2. **`MOVE-J(current_q)`** — the driver's, reading the pose from the SDK rather
   than from a ROS subscription, so a starved executor does not cost the rung.
   Reachable on its own as `hold_current_pose` (Trigger), which latches no fault
3. **`set_normal_mode`** — the mode frame, where not even the driver has a pose
4. **the external CAN watchdog** — where the bus is genuinely gone. It also
   commands `MOVE-J` at the current pose

Every rung holds the current pose. The last attempt is always that hold,
independent of whether this Jetson is healthy — CPU saturation is one of the ways
the rungs above it fail.

## Shutdown is on the ladder too (2026-08-26)

The firmware executes the last MIT command it received indefinitely, so the
setpoint a process leaves behind is what the arm does from then on. Ordinary exit
is therefore a rung, not an absence of one, and it holds the current pose like
every other rung:

1. **MIT controller**, on shutdown: a stiff gravity-compensated hold at the
   measured pose. Where feedback cannot place the arm it commands *nothing* and
   says so — the pose hold belongs to the rung below, not to a weaker MIT
   command.
2. **arm driver**, on shutdown: `MOVE-J(current_q)`, the same
   `_command_firmware_hold` the stop ladder ends at, so the arm leaves MIT
   entirely. Nothing is latched: an ordinary exit must not cost the next
   bring-up a lockout to clear.
3. **external CAN watchdog** beyond that, which also commands `MOVE-J` to the
   current pose.

The two processes receive `SIGINT` together under `ros2 launch` and either may
win the race. Both terminal states are a hold, so either order is safe.

Until 2026-08-26 the MIT controller's shutdown published
`_publish_damped_stop_command(float("inf"))` five times instead. `inf` selects
`torque_scale = 0.0` past the dead-man's ramp, so the arm's final setpoint was
`kp = 0`, `torque = 0`, damping only — no stiffness and no gravity feedforward.
A raised, loaded arm sags on that command, and the firmware holds it forever.
The rest of the codebase already stated the rule the shutdown path broke: the
same file's `_command_firmware_hold` warns that a dropped mode frame "leaves the
firmware in MIT executing the kp=0 damped stop, which has no stiffness — the arm
sags instead of holding."

**A kp=0 damped command is a braking transient, on every path, never a terminal
state.** It is the dead-man for a live stream that has lost its feedback, where
the ramp to zero torque exists because a feedforward frozen for a pose the arm
has left can actively drive it. It is not an answer to "this process is going
away".

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
(`docs/open_questions.md`, "Independent hardware emergency stop").

**There is no mechanical emergency stop on this unit.** The arm is either
powered or it is not, so the only guaranteed stop is removing arm power — and
that *drops* the arm, because a de-energized Nero has no brakes. That is the
reason the hold matters and the reason the independent layer is urgent rather
than optional: today the fallback below the hold is a fall. The Nero firmware
still has no MIT command watchdog either.

The single remaining `electronic_emergency_stop()` call in `agx_arm_ctrl` is the
vendor teach-mode-exit recipe in `_exit_teach_mode_callback` — Piper-only, at the
home pose, paired with the `reset()` that leaves the state again. It is a mode
transition, not a rung of this ladder.

## Validation

L1/L2: `src/agx_arm_ctrl/test/test_safety_hold_semantics.py` pins that no safety
path sends the vendor call, that an unverified stop re-asserts the hold, that a
verified stop holds once, and that a stop without a trustworthy pose commands
nothing. The last hardware record of the stop path is `l3_production_estop.md`
(2026-08-17), whose gate criteria are unchanged by this because they were
already met by the hold.

### L3 on the wire

"No safety path issues the vendor stop" is a claim about frames, so it is
settled on frames rather than on logs. `scripts/l3_estop_pcap_run.py` captures
both arm buses, runs `tea_pour_left_v1`, and fires
`/{left,right}_arm/emergency_stop` while a **recorded trajectory** is replaying:

```bash
python3 scripts/l3_estop_pcap_run.py                      # stop during node 160
python3 scripts/l3_estop_pcap_run.py --trigger-action-no 110 --trigger-delay 8
```

Node 160 (`left_arm_teapot_handle_release`) is the default because the teapot is
already down and released, so the arm is moving and empty. Node 110
(`left_arm_pour_tea`) is the harder case: payload at height, which is exactly
where a damped descent would have shown.

`scripts/analyze_can_pcap.py --stop-at <ts>` reads the capture back. Pass
criteria, in order of what they decide:

| Criterion | Frame |
| --- | --- |
| **no electronic stop, anywhere** | `0x150` byte 0 = `0x01` must not appear |
| the firmware is put in MOVE-J | `0x151` byte 1 = `0x01` after the stop |
| the hold carries a pose | `0x155`/`0x156`/`0x157`/`0x170` after that mode frame |
| the control stream does not outlive the hold | no `0x15A`–`0x160` after it |

The first row is the one this change is about. The rest were already true before
it and are carried so a regression in either direction is visible.

Everything except the first row is measured **against the hold, not against the
stop instant.** A stop legitimately puts MIT frames on the wire after it: the
kp=0 damped zero is one frame per joint, and a control cycle already in flight
finishes too. Judging those as a failed stop was the first version's mistake.

The run latches both arms and the unit; it prints the `clear_fault_lockout` and
`unit_safety/rearm` calls needed before anything else runs.

### Result, 2026-08-20 — both runs clean

Two live runs on the four-bus stack, hands on the mock backend so no object was
involved. Both stopped mid-replay of a **recorded trajectory**, both arms
answered `stop=verified`.

| run | trigger node | hold latency, left | hold latency, right | electronic stop |
| --- | --- | --- | --- | --- |
| 1 | 160 `left_arm_teapot_handle_release` | 10.6 ms | 8.1 ms | **none** |
| 2 | 110 `left_arm_pour_tea` | 12.9 ms | 7.6 ms | **none** |

All four captures verdict **clean**. Settle peaks 0.034 / 0.048 rad/s (run 1)
and 0.021 / 0.000 rad/s (run 2). The idle right arm is pinned at its current
pose by the same ladder — it receives the MOVE-J and its payload without ever
having been in MIT.

The frame-level shape of run 1's left arm, which is the interesting one because
a control cycle was in flight when the stop landed:

```text
  -39.9 .. -0.1 ms   MIT stream, 7 frames per cycle at 100 Hz
   +0.4 ..  +7.0 ms  the in-flight control cycle finishes, then the damped zero
   +8.1 ..  +9.3 ms  the residual cycle's own MIT mode frames (move=0x06, v111)
  +10.6 ms           MODE ctrl=0x01 move=0x01 mit=0x00   <- MOVE-J, MIT off
  +11.1 .. +14.0 ms  JOINT-CTRL 0x155/0x156/0x157/0x170  <- the held pose
```

That interleaving is the documented bound, not a defect: the safety lane
preempts the queue, not the call in flight. It also shows why
`_assert_firmware_hold` re-asserts rather than sending one MOVE-J — the residual
cycle put the firmware *back* into MIT 1.5 ms before the hold landed, and only
the readback-confirmed re-assertion makes the final state deterministic.

Evidence: `docs/sprint6/evidence/test_run_estop{,_2}/`.

**What this run does not cover.** Both stops verified on the first attempt, so
the retry ladder and the `no_hold_commanded` outcome are still L1/L2 only. That
needs a deliberately provoked unverified stop.
