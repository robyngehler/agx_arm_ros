# `tea_pour_left_v1` — first successful end-to-end hardware run

Date: **2026-08-17**
Session: 17:00:27 – 18:09:29 local (69 min); the two activity runs occupy 17:02:40 – 17:14:26
Level: **L3** (both arms, both hands, four CAN buses, real motion — `arm_dry_run=False`)
Branch: `ROS2_Duo_System_V02_refactor`
Commit: **`31c0350`** — see "How the commit was established"
Topology: `dedicated_per_device` — arms `can_nero_left` / `can_nero_right`, hands `hand_left` / `hand_right`
Execution profile: `duo_hand_external_bridge` (the tea launch owns the hand bridges)
Source: recovered from `~/.ros/log/`, launch directories `…-17-00-24-…-54155` and `…-17-01-05-…-54436`

---

## What this run proves

The first complete end-to-end execution of the coordinated demo on hardware, **twice**, on the
post-refactor contracts. It is the acceptance fact for Sprint 6's Step 5.

- The full 17-node activity completed: `activity 'tea_pour_left_v1' complete`, twice.
- The second run started and completed **in the same stack**, ~8 minutes after the first, with no
  restart and no operator intervention between them.
- The two runs are within **1.1 s of each other end to end** (92.7 s and 93.8 s), and no single
  action differs by more than 0.9 s. The demo is repeatable, not a lucky pass.
- Both **payload transitions fired on hardware**, in both runs: attach after the grip action,
  detach after the release, each applied in ~0.51 s and each naming the gravity model it switched to.
- The hand ran under the **claim-based authority contract** throughout: eight claim/release pairs on
  `hand_left`, none skipped, all by `reactive:omnihand_skill_controller`.
- The two arms came up on **different protocol tiers** and each fitted its own envelope, exactly as
  C8 requires: left firmware 1.11 → `NeroFW.V111`, torque bound `[16]*7`; right firmware 1.06 →
  `NeroFW.DEFAULT`, bound `[24,24,16,16,8,8,8]`.
- The unit-safety writer was the single generation allocator, with its incarnation identity live.

## What this run does **not** prove

- **It is not a `Ctrl+C`-during-motion test.** See "The Ctrl+C event" below — the interrupt arrived
  55 minutes after the second run finished, with nothing executing.
- **It is not coordinator-crash containment.** A graceful SIGINT and an abrupt process death are
  different failure modes; the second remains open.
- **It says nothing about the Hefeweizen demo**, which is dual-arm and tactile-gated. Do not
  generalise.
- **It says nothing about the generic `duo_hand` profile.** This run used
  `duo_hand_external_bridge`, because `start_tea_demo.launch.py` owns the hand bridges.
- **No CPU or CAN counters were captured during the run.** They are not retrospectively
  reconstructable and none are claimed here.
- **The payload mass was not validated, only applied.** The 1.0 kg at `[0.15, 0.0, 0.0]` is still the
  unmeasured estimate; this run shows the transition mechanism works, not that the number is right.

---

## Runtime composition

Two launches, started a minute apart:

```bash
# 17:00:24 — components stack (pid 54155)
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit execution_profile:=duo_hand_external_bridge \
  payload_mass_kg:=1.0

# 17:01:05 — tea demo stack (pid 54436)
ros2 launch agx_arm_coordination start_tea_demo.launch.py
```

`payload_mass_kg:=1.0` is **deduced, not transcribed**, and it is the one argument that can be:
the launch default is `0.0`, which preloads no second gravity model, and both MIT controllers logged
`Payload gravity model ready: 1.0 kg at [0.15, 0.0, 0.0]`. Had it been left at the default, the
`payload_update: attach` on action 70 would have been refused and the activity aborted there.

The components launch resolved `execution_profile: duo_hand_external_bridge` and started ten
processes: the unit-safety writer, two arm drivers, two MIT controllers, the joint-state merger, the
duo soft e-stop, the shared-CAN recovery node, `robot_state_publisher` and `move_group`. The tea
launch added two OmniHand bridges, two skill controllers and the coordinator.

Confirmed from the node logs:

| Fact | Evidence |
| --- | --- |
| Bus topology `dedicated_per_device` | coordinator: "same-side arm and hand may overlap, arm handoff off" |
| Each hand on its own adapter | "OmniHand SocketCAN interface: hand_left / hand_right (from `duo_motion_registry.yaml` omnihand.sides)" |
| Both hands `o12_pro`, vendor SDK backend | `backend_type=vendor_sdk_o12_pro`, 12 joints |
| MIT at the required rate | "MIT controller ready … at 100.0 Hz" on both sides |
| Arm acquisition decoupled from publication | `pub_rate: 200`, `acquisition_rate_hz: 100.0` |
| TX-loss observability present | "SDK send-error counters available" on both arms |
| Payload models preloaded per side | "Payload gravity model ready: 1.0 kg at [0.15, 0.0, 0.0] in frame `<side>_arm_nero_tool0`" |
| Gravity articulates the hand | 19 payload joints driven from live feedback, per side |
| Legacy hand ingress **off** | `allow_legacy_hand_command_ingress` defaults false and neither launch sets it; the bridge's startup line names the topic it *would* subscribe, not one it did |

The hand commands travelled the stamped `HandJointTarget` surface — the bridge's own warnings name
`hand_joint_target` as the command being verified.

---

## Activity timing

Both runs, per action, in seconds. `--` rows are the payload service transitions, which the
coordinator applies after the child succeeds and before the node counts as completed.

| # | Action | Run 1 | Run 2 |
| --- | --- | ---: | ---: |
| 10 | `left_hand_rest_fist` | 5.57 | 5.57 |
| 20 | `left_arm_to_teapot_grip_idle` | 2.27 | 3.16 |
| 30 | `left_hand_pre_grip_handle` | 2.20 | 2.20 |
| 40 | `left_arm_to_teapot_pre_grip` | 2.45 | 2.48 |
| 50 | `left_arm_teapot_handle_entry` | 5.71 | 5.86 |
| 60 | `left_arm_to_teapot_grip` | 2.70 | 2.66 |
| 70 | `left_hand_grip_handle` | 5.11 | 5.11 |
| -- | **payload attach** | 0.51 | 0.51 |
| 80 | `left_arm_to_teapot_post_grip` | 2.99 | 3.05 |
| 90 | `left_arm_to_pour_init` | 4.30 | 4.32 |
| 100 | `left_arm_to_pour_idle` | 5.02 | 5.05 |
| 110 | **`left_arm_pour_tea`** | **21.24** | **21.32** |
| 120 | `left_arm_to_pour_init` | 4.93 | 4.95 |
| 130 | `left_arm_to_teapot_pre_place` | 4.38 | 4.32 |
| 140 | `left_arm_to_teapot_place` | 1.77 | 1.83 |
| 150 | `left_hand_release_handle` | 1.03 | 0.98 |
| -- | **payload detach** | 0.51 | 0.52 |
| 160 | **`left_arm_teapot_handle_release`** | **14.41** | **14.34** |
| 170 | `left_hand_rest_fist` | 5.56 | 5.56 |
| | **total** | **92.67** | **93.79** |

Run 1: 17:02:40 → 17:04:12. Run 2: 17:12:52 → 17:14:25.

**Two actions dominate, and both are taught replays whose duration is the teach data, not the
runtime.** `left_arm_pour_tea` is a 73-point trajectory the MIT controller accepted with a declared
19.364 s duration in both runs — identical to three decimal places, because it is the same recorded
segment. `left_arm_teapot_handle_release` is 50 points at 13.177 s. Together they are 38 % of the
activity. Anyone wanting a faster demo should re-time those two recordings rather than look at the
runtime.

Every arm motion went through `FollowJointTrajectory` on the MIT controller — 16 goals per run,
accepted with point counts from 5 to 73. No goal was rejected, aborted, or replanned.

---

## Anomalies observed

The session was not anomaly-free. None of these failed the activity, and all four are already-known
open items rather than new defects — which is the reason to record them.

### 1. Hand delivery verification gave up five times

```text
OmniHand hand_joint_target command not verified within 8 attempts (tolerance 0.100 rad);
giving up — fingers may be in contact or the bus is congested
```

Left hand, at 17:03:10, 17:04:15, 17:13:00, 17:13:23 and 17:14:29. **Every one follows a closing
gesture** — `grip_handle` or `rest_fist` — where the fingers are physically blocked by the teapot
handle or by each other and therefore cannot reach the commanded joint positions.

The action succeeded anyway: the coordinator advanced and the activity completed. So the demo is
unaffected, but the message is the honest statement of a real gap — **the bridge cannot distinguish
"the fingers are in contact" from "the bus is congested"**, and says so. That is exactly the open
Phase-4D item *distinguish `commanded`, `delivery_verified` and `contact_confirmed` completion*, and
it now has a reproducible hardware case: a `pose` motion into a physical stop hits the 8-attempt
retry bound every time.

It is also the concrete instance of the open 2C item *bound and record the SDK round trips per
commanded setpoint*: five commands in this session each spent 8 sends plus their verification
readbacks.

### 2. One acquisition-loop overrun, at the moment the first activity started

```text
acquisition-loop overrun: 340 ms gap (> 200 ms; count=1, peak=340 ms)
```

Left arm, 17:02:40.842 — 0.8 s after the first activity was dispatched, i.e. as MoveIt planning and
the first hand action started together. `count=1` for the whole 69-minute session, and the message
itself names the right reading: local starvation, not a dead bus. No recovery was triggered, no
frames were lost that anything noticed.

### 3. A one-cycle ownership race at the first claim

```text
17:02:45.873  arm_left claimed by 'left_arm/mit_controller' at device generation 2
17:02:45.873  'left_arm/mit_controller' does not hold this device (held by nobody); not commanding
17:02:45.879  Took command of 'arm_left' at device generation 2
```

6 ms between the claim landing and the controller observing that it holds the device, during which
one control cycle declined to command. This is the **benign remainder** of the defect fixed in
`31c0350` ("stop the controller aborting the goal its own claim enabled") — the goal is no longer
aborted, and what is left is a single skipped cycle at enable time. It occurred once, on the first
enable of the session.

### 4. Teardown ordering on the right arm

```text
publish batch failed: Failed to publish: publisher's context is invalid
```

Right arm, at SIGINT. The publish loop outlived its rclpy context by one cycle during shutdown.
Cosmetic, at process exit, on the arm that was not moving.

---

## The Ctrl+C event — what it actually tested

**The review's summary of this session says a `Ctrl+C` test "terminated the activity correctly".
The logs do not support that reading and it should not be recorded that way.**

```text
17:14:25.916  activity 'tea_pour_left_v1' complete       (second run)
18:09:28.680  stop requested (interrupt (Ctrl+C)); no activity running
```

The interrupt arrived **55 minutes after the last activity finished**, with nothing in flight. The
coordinator said so itself. No child was cancelled, no arm was pinned, no hand was stopped, because
there was nothing to cancel.

What it *does* prove is worth having, and closes a different open item. All five tea-stack processes
exited within **0.66 s** of SIGINT, each reported by launch as "process has finished cleanly",
including the coordinator at 0.28 s:

```text
18:09:28.679  user interrupted with ctrl-c (SIGINT)
18:09:28.925  omnihand_skill_controller-2 finished cleanly
18:09:28.932  omnihand_skill_controller-4 finished cleanly
18:09:28.956  coordinator-5              finished cleanly
18:09:28.988  omnihand_bridge-1          finished cleanly
18:09:29.339  omnihand_bridge-3          finished cleanly
```

That is hardware evidence for the Phase-0B finding *"make SIGINT with no activity in flight exit
rather than spin"* — an idle coordinator now exits on the first interrupt instead of spinning until
a second one. It is **not** evidence for the stop ladder, which unwinds a *running* activity, and it
is not evidence for crash containment.

The components stack was interrupted one second earlier and its Python nodes exited via
`KeyboardInterrupt` tracebacks — untidy in the log, but they are the ordinary rclpy unwind, not
failures.

---

## How the commit was established

`31c0350` is a retrospective determination from two independent lines of evidence, not a
substitution of current `HEAD`:

1. **The reflog.** `31c0350` was committed at 16:45:10 on 2026-08-17 and remained `HEAD` until
   12:52:36 on 2026-08-18. The demo stack launched at 17:00:24 on 2026-08-17, inside that window,
   and no other commit exists between them.
2. **The installed code.** `install/` was populated by copy, not symlink;
   `mit_controller_node.py` was installed at 16:44 — one minute before `31c0350` was committed,
   which is the build that preceded the commit. The installed `mit_controller_node.py`,
   `omnihand_bridge_node.py` and `coordinator_node.py` are still **byte-identical** to their sources
   on the current branch, and every commit since has been documentation-only.

---

## Evidence not recoverable

- **CPU load and CAN counters during the run.** Nothing sampled them. Current `ip -s link`, `top` or
  `pidstat` output describes today and cannot be presented as this session's.
- **The exact launch arguments.** The launch log records the *resolved* profile
  (`duo_hand_external_bridge`) and the resolved arm instances, not the operator's command line, so
  the invocations reconstructed above are the resolved composition rather than transcribed keystrokes.
- **Per-goal MoveIt planning times and result codes.** `move_group`'s own log for this session was
  not retained at INFO level in the recovered directory; what survives is the MIT controller's
  acceptance record, which is why goal counts and durations are reported and planning times are not.
- **Anything about the objects.** Whether the teapot was empty or filled, and what was poured, is
  not in any log.

## Source files

```text
~/.ros/log/2026-08-17-17-00-24-000000-ubuntu-54155/launch.log   components stack
~/.ros/log/2026-08-17-17-01-05-328460-ubuntu-54436/launch.log   tea demo stack
~/.ros/log/python3_54445_1786978866080.log                      coordinator
~/.ros/log/python3_54162_1786978827780.log                      left MIT controller
~/.ros/log/python3_54160_1786978827982.log                      left arm driver
~/.ros/log/python3_54164_1786978827977.log                      right arm driver
~/.ros/log/python3_54437_1786978865942.log                      left hand bridge
~/.ros/log/python3_54439_1786978866027.log                      left skill controller
~/.ros/log/python3_54158_1786978827467.log                      unit safety writer
```

Not committed: they are runtime output under `~/.ros/`, which repository policy does not treat as
canonical source. The findings above are the promoted artefact.
