# `tea_pour_left_v1` — first hardware runs

Date: **2026-08-17**, 16:06 – 18:09 local
Level: **L3** for the live runs (both arms, both hands, four CAN buses, real motion)
Branch: `ROS2_Duo_System_V02_refactor`
Commit: **`31c0350`** for every successful run — see "How the commit was established"
Topology: `dedicated_per_device` — arms `can_nero_left` / `can_nero_right`, hands `hand_left` / `hand_right`
Execution profile: `duo_hand_external_bridge` (the tea launch owns the hand bridges)
Source: recovered from `~/.ros/log/`; the per-file list is at the bottom

---

## The session, in order

Eight attempts across six coordinator processes. Three were dry runs; five moved the arm.

| # | Time | Mode | Outcome |
| --- | --- | --- | --- |
| 1 | 16:06:55 | dry | **aborted** after 12.6 s — `payload update failed: payload_attach[left]: service unavailable` |
| 2 | 16:09:05 | dry | complete, 17 dispatches, 15.6 s |
| 3 | 16:13:42 | dry | complete, 17 dispatches, 15.5 s |
| 4 | **16:41:23** | **live** | **aborted** after 5.9 s at the first arm move — `child failed: MoveIt error_code=-4` (`CONTROL_FAILED`) |
| — | 16:45:10 | — | commit `31c0350`, *"stop the controller aborting the goal its own claim enabled"* |
| 5 | **16:49:56** | **live** | **complete**, 90.0 s — the first successful end-to-end run |
| 6 | **16:57:14** | **live** | **cancelled** 73.3 s in, at action 150 of 170 — two nodes from the end |
| 7 | **17:02:40** | **live** | **complete**, 92.7 s |
| 8 | **17:12:52** | **live** | **complete**, 93.8 s |

Runs 5 and 6 share one stack (coordinator pid 53259); runs 7 and 8 share the next one (pid 54445).
Each stack was ended by `Ctrl+C` on the launches.

## What the session proves

- **Three complete end-to-end runs on hardware**, across two independent bring-ups, on the
  post-refactor contracts and on the existing taught data.
- **Repeatable, not a lucky pass.** 90.0 s, 92.7 s and 93.8 s; within one stack no action differs by
  more than 0.9 s between runs.
- **A run can be started again in the same stack** without a restart or an operator step.
- **Both payload transitions fire on hardware** in every completed run: attach after the grip action,
  detach after the release, each applied in ~0.51 s and each naming the gravity model it switched to.
- **The hand ran under the claim-based authority contract throughout** — 19 claim/release pairs on
  `hand_left` across the two live stacks against 19 skill-controller `perform` calls, so **not one
  hand action skipped its claim**, all by `reactive:omnihand_skill_controller`.
- **The two arms came up on different protocol tiers and each fitted its own envelope**, as C8
  requires: left firmware 1.11 → `NeroFW.V111`, bound `[16]*7`; right 1.06 → `NeroFW.DEFAULT`,
  bound `[24,24,16,16,8,8,8]`.
- **A mid-activity cancellation was exercised** (run 6) and behaved correctly, with the caveats in
  its own section below.

## What the session does **not** prove

- **Not a cancel of a *moving* arm.** Run 6's cancel landed between children, with nothing in
  flight — see below. The stop ladder that pins a moving arm is still unexercised.
- **Not coordinator-crash containment.** Graceful cancellation and abrupt process death are
  different failure modes.
- **Nothing about the Hefeweizen demo**, which is dual-arm and tactile-gated.
- **Nothing about the generic `duo_hand` profile.** Every run used
  `duo_hand_external_bridge`.
- **No CPU or CAN counters were captured.** They are not retrospectively reconstructable and none
  are claimed here.
- **The payload mass was applied, not validated.** 1.0 kg at `[0.15, 0.0, 0.0]` is still the
  unmeasured estimate.

---

## Run 4 — the live abort, and the fix that followed it

The first attempt with real motion failed 0.33 s into the first arm move. The left MIT controller's
log has the whole mechanism in four lines:

```text
16:41:29.078  MIT controller enabled
16:41:29.089  Accepted FollowJointTrajectory goal with 34 points and 3.248s duration
16:41:29.090  'left_arm/mit_controller' does not hold this device (held by nobody); not commanding
16:41:29.096  Device authority changed (state=2, device_epoch=2, unit_safety_epoch=0,
              motion_ready=True): claimed by left_arm/mit_controller; aborted the active trajectory
```

**The controller's own claim bumped the device epoch, its authority callback saw an epoch change,
and it aborted the trajectory it had just accepted.** MoveIt reported `CONTROL_FAILED`, and the
coordinator aborted the activity.

That is precisely the defect `31c0350` fixed, committed four minutes later at 16:45:10. The first
successful run came eight minutes after that. In every run since, the same three-line sequence
appears and ends with `Took command of 'arm_left'` instead of an abort — the benign one-cycle
remainder recorded under "Anomalies" below.

This is also the independent confirmation of which commit the successful runs used: the failure that
`31c0350` describes happens before it and never after it.

## Run 1 — what the dry runs were worth

The first dry attempt aborted with `payload update failed: payload_attach[left]: service
unavailable`, seven actions in. A dry run cannot move an arm, but it *can* find that a service the
activity depends on is not being started — which is what it did, before any hardware was at risk.
The two dry runs after it completed all 17 nodes.

---

## Activity timing

The three completed runs, per action, in seconds. `--` rows are the payload service transitions,
which the coordinator applies after a child succeeds and before the node counts as completed.

| # | Action | Run 5 | Run 7 | Run 8 |
| --- | --- | ---: | ---: | ---: |
| 10 | `left_hand_rest_fist` | 5.57 | 5.57 | 5.57 |
| 20 | `left_arm_to_teapot_grip_idle` | 4.10 | 2.27 | 3.16 |
| 30 | `left_hand_pre_grip_handle` | 2.20 | 2.20 | 2.20 |
| 40 | `left_arm_to_teapot_pre_grip` | 2.45 | 2.45 | 2.48 |
| 50 | `left_arm_teapot_handle_entry` | 5.97 | 5.71 | 5.86 |
| 60 | `left_arm_to_teapot_grip` | 2.68 | 2.70 | 2.66 |
| 70 | `left_hand_grip_handle` | 0.92 | 5.11 | 5.11 |
| -- | **payload attach** | 0.51 | 0.51 | 0.51 |
| 80 | `left_arm_to_teapot_post_grip` | 2.99 | 2.99 | 3.05 |
| 90 | `left_arm_to_pour_init` | 4.28 | 4.30 | 4.32 |
| 100 | `left_arm_to_pour_idle` | 4.52 | 5.02 | 5.05 |
| 110 | **`left_arm_pour_tea`** | **21.33** | **21.24** | **21.32** |
| 120 | `left_arm_to_pour_init` | 4.92 | 4.93 | 4.95 |
| 130 | `left_arm_to_teapot_pre_place` | 4.31 | 4.38 | 4.32 |
| 140 | `left_arm_to_teapot_place` | 1.78 | 1.77 | 1.83 |
| 150 | `left_hand_release_handle` | 0.98 | 1.03 | 0.98 |
| -- | **payload detach** | 0.51 | 0.51 | 0.52 |
| 160 | **`left_arm_teapot_handle_release`** | **14.39** | **14.41** | **14.34** |
| 170 | `left_hand_rest_fist` | 5.57 | 5.56 | 5.56 |
| | **total** | **90.0** | **92.7** | **93.8** |

**Two actions dominate, and both are taught replays whose duration is the teach data, not the
runtime.** `left_arm_pour_tea` is a 73-point trajectory the MIT controller accepted with a declared
19.364 s duration in every run — identical to three decimal places, because it is the same recorded
segment. `left_arm_teapot_handle_release` is 50 points at 13.177 s. Together they are 38 % of the
activity. A faster demo means re-timing those two recordings, not optimising the runtime.

Every arm motion went through `FollowJointTrajectory` on the MIT controller — 16 goals per complete
run, none rejected, aborted or replanned after run 4.

**One cell in run 5 is worth not glossing over.** Its `left_hand_grip_handle` took **0.92 s** against
5.11 s in both later runs — the only action anywhere in the table that differs by more than a factor
of two. It is a hand pose whose duration is dominated by delivery verification, and run 5 is the
first activity after that stack came up, so the fingers started from a different configuration and
converged sooner. Recorded as observed; no cause is claimed from one occurrence.

The other cross-stack differences are ordinary: `left_arm_to_teapot_grip_idle` spans 2.27–4.10 s
across the three runs, which is MoveIt planning time on the first move of an activity.

---

## Run 6 — the mid-activity cancellation

This is the run the operator remembers as "aborted with `Ctrl+C` shortly before the end", and the
logs bear it out with more precision than the recollection could.

```text
16:58:26.0   -> dispatch left_hand_release_handle ([150])       15th of 17 nodes
16:58:26.0   [left] perform left_hand_release_handle
16:58:26.0   hand_left transport claimed by 'reactive:omnihand_skill_controller'
16:58:27.5   hand_left transport released by 'reactive:omnihand_skill_controller'
             ... nothing further ...
16:58:42.5   stop requested (interrupt (Ctrl+C)); no activity running
```

The activity was cancelled **two nodes from the end**, in the ~1 s window between the hand child
finishing on the hardware and the coordinator recording it complete. The payload detach never ran
and action 160 was never dispatched.

**How we know it was a cancel and not a failure:** the abort path logs
`ERROR: aborting '<activity>': <message>` and there is no such line. The cancel path logs nothing at
all, which is the finding below.

### Three things this establishes

- **Cancellation works, and terminates the activity cleanly.** The coordinator accepted the cancel,
  unwound, and 15 s later reported no activity running. Both launches then shut down cleanly.
- **It does not exercise the stop ladder.** Nothing was in flight at the moment of the cancel: the
  hand goal had already succeeded and no arm goal was open — the left MIT controller's last accepted
  goal was action 140, and it logged nothing after. So `_cancel_children` had no moving child to
  stop, and "cancel a moving arm and pin it" remains untested on hardware.
- **The activity ended with the loaded payload model still active.** The detach is applied after the
  release child completes; the cancel arrived first. This is the first hardware occurrence of the
  behaviour `tea_demo.md` already documents as deliberate — an activity that ends while nominally
  carrying keeps the loaded model, which over-compensates gravity rather than under-compensating it.
  Worth knowing when a cancelled run is followed by another in the same stack: the attach is
  idempotent, so the next run recovers, but the arm holds a 1 kg model in the meantime.

---

## Anomalies observed

None failed a completed run. All are known open items, now with reproducible triggers.

### 1. The coordinator logs nothing when an activity is cancelled

**The most useful finding of the session, and it is an observability defect rather than a motion
one.** `_abort()` logs an ERROR naming the activity and the reason. The cancel branch —
`goal_handle.is_cancel_requested or self.stop_requested` — emits a `failed` event on `~/events`,
sets the action result, and calls `goal_handle.canceled()`, but makes no logger call.

The consequence is exactly what happened here: a cancelled run leaves a log that stops mid-dispatch
with no terminal line, indistinguishable at a glance from a coordinator that hung. It took reading
the abort path's source to establish which of the two had occurred. Anyone reconstructing a session
afterwards — which is the situation this whole record exists for — has no line to find.

Not fixed here; this record is documentation only.

### 2. A closing hand gesture always exhausts the delivery-verification retry bound

```text
OmniHand hand_joint_target command not verified within 8 attempts (tolerance 0.100 rad);
giving up — fingers may be in contact or the bus is congested
```

Nine occurrences across the two live stacks (four and five), on the left hand. **Every one follows a closing
gesture** — `grip_handle` on the teapot handle, or `rest_fist` where the fingers close on each other
— so the commanded joint positions are physically unreachable and the readback never converges.

The action succeeded anyway and the activity completed, so the demo is unaffected. But the message
is the honest statement of a real gap: **the bridge cannot distinguish "the fingers are in contact"
from "the bus is congested"**, and says so. That is the open Phase-4D item — distinguish
`commanded`, `delivery_verified` and `contact_confirmed` — with a trigger that reproduces on demand.

It is also the measured case for the open 2C item *bound and record the SDK round trips per
commanded setpoint*: nine commands each spent 8 sends plus their verification readbacks.

### 3. One acquisition-loop overrun, at the instant an activity started

`acquisition-loop overrun: 340 ms gap (> 200 ms; count=1, peak=340 ms)` on the left arm, 0.8 s after
the first dispatch of run 7 — MoveIt planning and the first hand action starting together. `count=1`
across the whole stack, no recovery triggered. The message names the right reading itself: local
starvation, not a dead bus, which is the Level-0 recovery classification working.

### 4. A one-cycle ownership race at the first claim

Within 6 ms: `arm_left claimed by 'left_arm/mit_controller'`, then `does not hold this device (held
by nobody); not commanding`, then `Took command of 'arm_left'`. One control cycle declines to
command while the claim is in flight. This is the benign remainder of the run-4 defect — the goal is
no longer aborted, and what is left is a skipped cycle at enable time. Once per stack.

### 5. Teardown ordering at shutdown

`publish batch failed: Failed to publish: publisher's context is invalid` on the right arm at SIGINT,
and `move_group` exiting with `-11` in the first live stack. Both at process exit, on components that
were not moving.

---

## The final interrupt at 18:09 — an idle exit, not a cancel

Separate from run 6, and worth keeping separate. The last stack was interrupted **55 minutes after
run 8 finished**, with nothing running:

```text
17:14:25.916  activity 'tea_pour_left_v1' complete       (run 8)
18:09:28.680  stop requested (interrupt (Ctrl+C)); no activity running
18:09:28.925  omnihand_skill_controller-2 finished cleanly
18:09:28.932  omnihand_skill_controller-4 finished cleanly
18:09:28.956  coordinator-5              finished cleanly
18:09:28.988  omnihand_bridge-1          finished cleanly
18:09:29.339  omnihand_bridge-3          finished cleanly
```

All five processes gone within 0.66 s, the coordinator in 0.28 s. That is hardware evidence for the
Phase-0B finding *"make SIGINT with no activity in flight exit rather than spin"*: an idle
coordinator now exits on the first interrupt.

---

## Runtime composition

```bash
# components stack
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit execution_profile:=duo_hand_external_bridge \
  payload_mass_kg:=1.0

# hand bridges, skill controllers and the coordinator
ros2 launch agx_arm_coordination start_tea_demo.launch.py
```

`payload_mass_kg:=1.0` is **deduced, not transcribed**, and it is the one argument that can be: the
launch default is `0.0`, which preloads no second gravity model, and both MIT controllers logged
`Payload gravity model ready: 1.0 kg at [0.15, 0.0, 0.0]`. Left at the default, the attach on action
70 would have been refused and the activity aborted there — which is exactly what happened in the
first dry run for the neighbouring reason, a service that was not up.

Confirmed from the node logs:

| Fact | Evidence |
| --- | --- |
| Bus topology `dedicated_per_device` | coordinator: "same-side arm and hand may overlap, arm handoff off" |
| Each hand on its own adapter | "OmniHand SocketCAN interface: hand_left / hand_right (from `duo_motion_registry.yaml` omnihand.sides)" |
| Both hands `o12_pro`, vendor SDK backend | `backend_type=vendor_sdk_o12_pro`, 12 joints |
| MIT at the required rate | "MIT controller ready … at 100.0 Hz" on both sides |
| Arm acquisition decoupled from publication | `pub_rate: 200`, `acquisition_rate_hz: 100.0` |
| TX-loss observability present | "SDK send-error counters available" on both arms |
| Gravity articulates the hand | 19 payload joints driven from live feedback, per side |
| Real motion | `arm_dry_run=False` in the coordinator's startup line |
| Legacy hand ingress **off** | `allow_legacy_hand_command_ingress` defaults false and neither launch sets it |

The hand commands travelled the stamped `HandJointTarget` surface — the bridge's own warnings name
`hand_joint_target` as the command being verified.

---

## How the commit was established

`31c0350` is a retrospective determination from three independent lines of evidence, not a
substitution of current `HEAD`:

1. **The reflog.** `31c0350` was committed at 16:45:10 on 2026-08-17 and remained `HEAD` until
   12:52:36 on 2026-08-18. Every successful run falls inside that window, and no other commit exists
   between them.
2. **The installed code.** `install/` was populated by copy, not symlink;
   `mit_controller_node.py` was installed at 16:44 — the build that preceded the commit. The
   installed `mit_controller_node.py`, `omnihand_bridge_node.py` and `coordinator_node.py` are still
   **byte-identical** to their sources on the current branch, and every commit since has been
   documentation-only.
3. **The behaviour changed at the commit.** Run 4, before it, failed with the exact defect
   `31c0350`'s message describes. No run after it does.

## Evidence not recoverable

- **CPU load and CAN counters during any run.** Nothing sampled them. Current `ip -s link`, `top` or
  `pidstat` output describes today and cannot be presented as this session's.
- **The exact launch command lines.** The launch logs record the *resolved* profile and arm
  instances, not the operator's keystrokes, so the invocations above are the resolved composition
  plus one deduced argument.
- **Per-goal MoveIt planning times.** `move_group`'s log for these sessions was not retained at INFO
  level in the recovered directories; what survives is the MIT controller's acceptance record, which
  is why goal counts and declared durations are reported and planning times are not.
- **The precise instant of run 6's cancellation.** It is bounded to the ~1 s window between the hand
  child completing (16:58:27.5) and the next loop pass, because the cancel path leaves no log line.
- **Anything about the objects.** Whether the teapot was empty or filled, and what was poured, is
  not in any log.

## Source files

```text
~/.ros/log/python3_44366_1786975602218.log   coordinator, dry run 1 (payload service unavailable)
~/.ros/log/python3_45080_1786975733152.log   coordinator, dry run 2
~/.ros/log/python3_46812_1786976015552.log   coordinator, dry run 3
~/.ros/log/python3_50357_1786976902357.log   coordinator, live abort (run 4)
~/.ros/log/python3_50101_*.log               left MIT controller, the run-4 abort mechanism
~/.ros/log/python3_53259_1786978188436.log   coordinator, runs 5 and 6
~/.ros/log/python3_53002_*.log               left MIT controller, runs 5 and 6
~/.ros/log/python3_53251_*.log               left hand bridge, runs 5 and 6
~/.ros/log/python3_54445_1786978866080.log   coordinator, runs 7 and 8
~/.ros/log/python3_54162_*.log               left MIT controller, runs 7 and 8
~/.ros/log/python3_54437_*.log               left hand bridge, runs 7 and 8
~/.ros/log/2026-08-17-16-49-{33,47}-*/launch.log   stack for runs 5 and 6
~/.ros/log/2026-08-17-17-0{0,1}-*/launch.log       stack for runs 7 and 8
```

Not committed: they are runtime output under `~/.ros/`, which repository policy does not treat as
canonical source. The findings above are the promoted artefact.
