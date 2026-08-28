# Tea Pour Demo (`tea_pour_duo_v2`) — Runbook

> **`tea_pour_left_v1` cannot run any more (2026-08-27).** Its eight anchor poses
> (`Can_Grip_Idle_L`, `Can_Pre_Grip_L`, `Can_Grip_L`, `Can_Post_Grip_L`,
> `Can_Fill_Init_L`, `Can_Fill_Idle`, `Pre_Place_Can_L`, `Place_Can_L`) were
> re-captured out of `arm_config.yaml` under `Tee-Can_*` names, so every anchor in
> it now names a pose that does not exist. The current activity is
> **`tea_pour_duo_v2`** — see § The v2 activity below. `tea_pour_left_v1` is kept
> for its taught waypoints, not because it is runnable; re-anchor it or delete it.
>
> Everything in this page from § Emergency stop onwards describes mechanisms that
> are unchanged between the two — payload attach/detach, how a replay reaches its
> start, the playback modes, the CPU budget. The v1 step tables are historical.

> **Superseded in part — V02 refactor.** Each device now has its own CAN bus
> (arms `can_nero_left`/`can_nero_right` native, hands `hand_left`/`hand_right`
> on USB-CAN FD adapters), declared once as `bus_topology` in the registry. Every
> hand bridge resolves its interface from its **own** registry entry and fails
> closed without one; the derivation from the arm's `can_port` is gone. Same-side
> arm and hand motion runs in parallel, the handshake defaults **off**, and the
> shared-bus hand window is a selectable degraded mode.
>
> **The hand-window and handshake passages below therefore describe the
> `shared_per_side` degraded topology, not normal operation.** A hand command
> also now carries the authority it was issued under, and an unclaimed hand
> executes nothing. This page has not yet been rewritten around either change.
> See `docs/sprint_refactor/planning/integration_plan.md` (C1, C7) and
> `docs/sprint_refactor/planning/decision_record.md` §5 and §7.

The end-to-end hardware demo: the **left** arm plus the **left** OmniHand pick up a tea can, carry it
to the pour station, pour, set it down and withdraw, while the **right** arm supports.

v1 was left-only, with the right side merely live. v2 is a duo activity: the right arm is staged
first, its two longest motions were taught on both arms at once, and three of its steps overlap an
arm move with a hand shape. The right hand is out of service and is commanded nowhere.

Built on the sprint 6 coordinator.

> **Naming.** The anchor poses and hand gestures were captured under `Can_*` names before the object
> was settled. The object is a **teapot**; `Can_*` is a capture-time misnomer that is kept verbatim
> wherever it names stored data so nothing has to be re-measured. The public `action_id`s say teapot.

## Emergency stop — read this first

`Ctrl+C` on either the coordinator or the `run_activity` client now **stops the robot** instead of
just killing the process. That is a deliberate change: previously, exiting either one left the MoveIt
trajectory and the hand goal executing with nobody left to cancel them.

| You press | What happens |
|---|---|
| `Ctrl+C` on `run_activity` | cancels the activity goal, waits for the coordinator to confirm it unwound, then exits |
| `Ctrl+C` on the coordinator | cancels children → pins the moving arm (`cancel_trajectory` + `hold_current`) → exits. On the degraded `shared_per_side` topology it also closes any hand window it opened; on the normal topology there is none to close |
| `Ctrl+C` with **no activity running** | the coordinator exits on the first interrupt. Verified on hardware 2026-08-17: all five tea-stack processes gone within 0.66 s |
| `Ctrl+C` a second time | escalates to `emergency_stop` on both sides, then exits immediately |
| the coordinator crashes | **not covered by any of the above.** The MIT controller streams a damped stop on its own shutdown and the driver puts the arm in a firmware MOVE-J hold before a recovery disconnect, but a hard crash of the coordinator alone leaves the MoveIt goal running |

**This unit has no mechanical emergency stop.** The arm is either powered or it is not, and the only
guaranteed stop is removing arm power — which drops the arm, because a de-energized Nero has no
brakes. The Nero firmware also has no MIT command watchdog, so silence is not a safe state either.
See `teach_and_run.md` § Emergency stop / runaway.

## What runs where

| Layer | Command |
|---|---|
| CAN buses | `sudo bash ./scripts/activate_duo_can.sh` |
| arms + MoveIt + unit safety | `start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=duo_hand_external_bridge` |
| hands + coordinator | `start_tea_demo.launch.py` |
| the activity | `ros2 run agx_arm_coordination run_activity --activity tea_pour_duo_v2` |

> **The unit safety writer must be running.** `agx_arm_ctrl unit_safety` allocates the unit's safety
> generation, and the coordinator is fail-closed: with no generation established it rejects every
> activity (`unit safety state is not established (none has ever arrived)`). The arm bring-up starts
> it (`start_unit_safety`, default true). Exactly one per unit — the coordination launch defaults its
> own copy to **off** so the two cannot both write.

> **Use `duo_hand_external_bridge`, not `duo_hand`.** `start_tea_demo.launch.py`
> starts both hand bridges itself, under `/left_hand` and `/right_hand`. The
> `duo_hand` profile sets `launch_omnihand_bridge: true`, and a profile value
> **overrides** the launch argument — so `launch_omnihand_bridge:=false` cannot
> turn it off. Pairing the two therefore puts two bridges, and two vendor SDK
> sessions, on one physical hand.
>
> `duo_hand_external_bridge` is identical except that the arm slice does not own
> the bridges. The hand stays in the description, so gravity keeps its 1.06 kg at
> the flange and MoveIt keeps its collision geometry — which `duo_arm` would both
> drop. Each arm driver is pointed at `/<side>_hand/feedback/omnihand/joint_states`
> so the hand joints still reach the combined `feedback/joint_states`, which is
> where move_group reads the full robot from.
>
> **Order matters with this profile.** Until the hands come up, move_group repeats
> `The complete state of the robot is not yet known. Missing left_index_abad_joint, …`
> — it has the arms but not the 24 hand joints, and it will not plan. The warning
> stops once step 3 runs. If it persists after the hands are up, the arm driver
> and the bridge disagree about the topic; check
> `ros2 param get /left_arm/agx_arm_ctrl_single_node omnihand_joint_states_topic`
> against `ros2 topic list | grep omnihand/joint_states`.

## The v2 activity (`tea_pour_duo_v2`)

Source of the sequence: `docs/sprint6/target/README.md`. Graph:
`agx_arm_coordination/config/activities/tea_pour_duo_v2.yaml`; actions:
`config/catalogue.d/tea_pour_duo_v2.yaml`.

> **The right hand is out of service (2026-08-27)** and this activity commands
> it nowhere — not even to `zero`. It is assumed to already sit flat and to stay
> there. The right **arm** is unchanged, so its support replay and its half of
> the two duo takes now run with a *flat* right hand where they were taught with
> a shaped one (`can_prep`). Check that before the first live run.
>
> The activity also no longer ends on the two-handed heart. It returns to
> `Functional_Init_Both_V01` with the left hand closing to `fist`, and stops
> there. `both_arms_to_heart_top`, `left_hand_heart` and `left_hand_zero` stay
> defined in the catalogue, referenced by nothing.

20 nodes, dispatched in 17 steps. `‖` marks a step the scheduler admits as one
batch — an arm goal and a hand goal running at the same time.

| # | Step | Group | Taught / planned |
|---|---|---|---|
| 1 | `both_arms_to_functional_init` | both_arms | anchor |
| 2 | `both_arms_to_can_prep_grip` | both_arms | anchor |
| 3 | `right_arm_can_prep_4grip` | right_arm | replay, 9.49 s taught |
| 4 | `both_arms_to_can_grip_idle` ‖ `left_hand_can_pre_grip` | both_arms + left_hand | anchor |
| 5 | `both_arms_to_can_pre_grip` | both_arms | anchor |
| 6 | `left_arm_can_grip_move` | left_arm | replay, 7.83 s taught → 6.12 s |
| 7 | `both_arms_to_can_pre_grip_adjust` | both_arms | anchor, 0.05 scaling |
| 8 | `left_hand_can_grip` ‖ `both_arms_to_can_adjust_while_grip` | left_hand + both_arms | anchor, 0.05 scaling; **payload attach** |
| 9 | `left_arm_can_lift_post_grip` | left_arm | replay, 4.65 s taught → 2.83 s |
| 10 | `both_arms_can_goto_pour_init` | both_arms | replay, 10.91 s |
| 11 | `both_arms_can_pour` | both_arms | replay, 25.35 s |
| 12 | `both_arms_to_can_post_grip` | both_arms | anchor |
| 13 | `both_arms_to_can_place` | both_arms | anchor, 0.05 scaling |
| 14 | `left_hand_can_release` | left_hand | **payload detach** |
| 15 | `left_arm_can_release_motion` | left_arm | replay, 8.68 s |
| 16 | `left_arm_can_post_place_adjust` | left_arm | replay, 8.85 s → 8.69 s |
| 17 | `both_arms_to_functional_init` ‖ `left_hand_fist` | both_arms + left_hand | anchor; last step |

Three replays come out **shorter** than taught. `smooth` reconstructs the motion,
and a take that opened with the arm standing still — 2.00 s on the lift, 1.88 s
on the grip move — loses that dead lead-in. The motion itself is not sped up.

### The overlap is the activity, not an optimisation of it

Steps 4, 8 and 17 are `sync_flag` groups (step 2 was one until the right
hand's `can_prep` came out; the closing heart was another). They are only
legal because
every device owns its own bus (`bus_topology: dedicated_per_device`): a hand
action then holds no resource token the arms hold, the scheduler admits the group
whole, and the coordinator dispatches two independent goals — a `both_arms`
action is not one of the two per-arm shapes it would try to merge.

Under `shared_per_side` the activity is **refused at validation**, naming all
three pairs, rather than running serialized. That is deliberate: a run that
quietly dropped the overlap would be a different activity wearing this one's
name. Switching the registry back to the degraded topology therefore means
re-writing this graph, not just accepting a slower run.

### Every anchor is a `both_arms` pose

The taught vectors in `arm_config.yaml` are 14-DoF, so an anchor move always
states where *both* arms are. The arm that is not the subject of a step holds its
measured pose rather than wherever it happened to stop.

One consequence to be aware of on hardware: after the pour, the right arm is
3.755 rad from `Tee-Can_Post_Grip_L`'s right half on j5, and step 13 then moves
it a further 1.915 rad away. The right arm returns to the support pose and
immediately leaves it. That is what the flow says; if the swing is unwanted, the
fix is a right-arm anchor between steps 11 and 13, not a change here.

### Recordings are referenced, not inlined

`config/recordings/*.json` holds lean sidecars — joint names, times, positions —
that a `recording:` key points at. Full taught density survives (219–1407
samples per take, 4.1 MB of teach files as 426 KB of sidecars), which is what the
retiming needs and what decimating into the catalogue would destroy. The sidecars
carry side-prefixed joint names, so the planner checks the side against the group
instead of taking the catalogue's word for it.

**A sidecar is a copy, so re-teaching a take on hardware does not update it.**
The activity keeps replaying the older motion and nothing says so — that cost one
run, where a re-taught `Prep_Tee-Can_4Grip_Right` (633 samples, 9.49 s) was still
being replayed as the 454-sample, 7.37 s take it replaced. After any re-teach:

```bash
scripts/refresh_demo_recordings.py --check   # what is out of date (exit 1 if any)
scripts/refresh_demo_recordings.py           # rewrite them
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_coordination
```

It reads the `recording:` references straight out of the catalogue and takes the
side prefix from each action's `robot_id`, so nothing has to be listed twice. For
a single take by hand:

```bash
ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue \
    ~/agx_arm_trajectories/teach/Tee-Can_Pour_V01.json \
    --action-id both_arms_can_pour \
    --emit-recording src/agx_arm_coordination/config/recordings/Tee-Can_Pour_V01.json
```

Add `--joint-prefix left_arm_` (or `right_arm_`) for a single-arm take; a duo take
already carries prefixed names.

### Replanning after a MoveIt failure

`left_arm_link2` against `body_base_link` was **not** an intermittent failure,
and this is worth keeping straight because the symptom reads like one.
Surface-to-surface distance, measured 2026-08-27 against the same URDF move_group
loads: 0.01 mm at `Tee-Can_Adjust-While-Grip_L` — touching — against 3.95 mm at
`Tee-Can_Post_Grip_L` and 5.33 mm at `Tee-Can_Pre_Grip_Adjust_L`, both of which
plan. `Unable to sample any valid states for goal tree` says every state in the
±0.01 rad goal box collides, which is a deterministic refusal, not a flaky one.
No joint nudge helps: only j1 and j2 move link2, and ±0.10 rad on either stays
between 1.1 and 4.0 mm.

link2 rotates about the mount, so it sweeps along the torso hull by construction:
over the **whole** j1 x j2 range it never gets further than 12.9 mm (left) /
13.3 mm (right) from the body, median 7.2 / 8.0 mm, and 81% of that space is
under 10 mm. A collision check on a pair that can never be more than 13 mm apart
yields refusals, not safety, so `<side>_arm_link2` vs `body_base_link` is now
disabled in the SRDF alongside `base_link` and `link1`, which were already out.
The pose it refused is one an operator back-drove the real arm into by hand.

It is **not** the payload update, although it lands in the same instant: the grip
that attaches the payload and this anchor move are one sync group, dispatched
together, and this is the tightest pose in the activity. `~/payload_attached`
swaps a reference between two gravity models preloaded at startup under the
control loop's lock — it publishes nothing, touches no planning scene, and
generates no motion.

Genuinely intermittent planning failures do happen here, which is what the retry
below is for.

`num_planning_attempts` does not help there. Its attempts share one goal sampler:
in the failure that prompted this, the first attempt spent 0.55 s and the next
two returned in 0.48 ms and 0.23 ms with `Insufficient states in sampleable goal
region`. Only a **new goal** rebuilds the sampler.

The coordinator therefore re-dispatches a failed arm goal up to
`plan_retry_attempts` times (default 2, so three tries in all), and logs each
one. Retried: `FAILURE`, `PLANNING_FAILED`, `INVALID_MOTION_PLAN`,
`MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE`, `TIMED_OUT`,
`START_STATE_IN_COLLISION`, `GOAL_IN_COLLISION`. **Not** retried: `CONTROL_FAILED`
and `PREEMPTED`, because the motion started and the arm is somewhere this cannot
reason about, and the `INVALID_*` configuration codes, which are deterministic. A
taught replay is retried only while its planned approach is what failed.

```bash
ros2 run agx_arm_coordination coordinator --ros-args -p plan_retry_attempts:=4
```

Set it to 0 to fail on the first refusal.

### 1. Dry run first (no hardware)

```bash
ros2 launch agx_arm_coordination start_tea_demo.launch.py \
  arm_dry_run:=true start_unit_safety:=true
ros2 run agx_arm_coordination run_activity --activity tea_pour_duo_v2
```

Mock hands, no arm goals sent. Confirms the graph validates, every action resolves, and the
scheduler serializes the chain. It does **not** exercise planning, so it cannot tell you whether the
anchors are reachable.

`start_unit_safety:=true` because the live bring-up gets the unit safety writer from the arm launch,
and there is no arm launch here. Without a writer the coordinator refuses every activity with
`unit safety state is not established`. Exactly one writer per unit, so do **not** pass it when the
arm slice is up.

The payload transitions are skipped in a dry run (`dry_run: skipped payload attach ...`): the payload
service lives on the MIT controller, which is an arm surface.

### 2. Live bring-up

```bash
sudo bash ./scripts/activate_duo_can.sh

# Arms + MoveIt. use_rviz:=false is a CPU decision, not cosmetic (see below).
# payload_mass_kg arms the teapot gravity model; without it the arm carries the
# teapot on the unloaded model and the coordinator's attach request is refused.
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit execution_profile:=duo_hand_external_bridge follow:=true \
  planning_pipelines:=ompl use_rviz:=false \
  payload_mass_kg:=1.0

# Hands + coordinator
ros2 launch agx_arm_coordination start_tea_demo.launch.py backend_type:=sdk

# Run it
ros2 run agx_arm_coordination run_activity --activity tea_pour_duo_v2
```

**v1 measured 2026-08-17: 93 s**, twice, within 1.1 s of each other — 8 planned anchor moves, 3
taught replays (4.6 s + 19.4 s + 13.2 s at the 0.75 time-stretch) and 5 hand actions. v2 has not
been run on hardware: 8 anchor moves, 7 replays (73.4 s of taught motion) and 4 hand actions, three
of which cost nothing extra because they overlap an arm move. On the normal
`dedicated_per_device` topology **no hand window is opened**: the hand claims its own device, the arm
keeps its own bus, and the handshake is off. Per-action timings for all three completed runs, and the
anomalies they recorded, are in `../../sprint6/evidence/tea_pour_left_v1_2026-08-17.md`.

Two taught replays are 38 % of the runtime (`left_arm_pour_tea` 21.3 s,
`left_arm_teapot_handle_release` 14.3 s). Both durations come from the recordings themselves, so a
faster demo means re-teaching those two segments.

## CPU budget

The Jetson, not the robot, is the binding constraint: when the host stalls, the kernel CAN RX socket
overflows and hand response frames are dropped. That failure mode is **host CPU starvation, not bus
contention**, so it survived the move to four buses — and parallel operation makes it more likely,
not less.

The conservative rates below are kept, but their reason has changed. They are **headroom**, not
shared-bus mitigation: each hand now has its own adapter, so a hand poll no longer competes with arm
feedback for a bus. Distinguish the three things a "rate" can mean here:

- **ROS publication cadence** — since the refactor, publication is driven by *new readbacks*, and
  `hand_pub_rate` is a ceiling that can throttle it and cannot make it faster;
- **real CAN/SDK request cadence** — `hand_joint_read_rate` is the one that still costs bus traffic
  and vendor round trips;
- **rendering and planning-scene load** — `use_rviz`, which is pure host CPU.

> **Raised 2026-08-22.** The active hand now runs at 50 Hz on both knobs, on the
> decision that 50 Hz is the floor for record and playback. That reverses the
> throttling this table was written to justify, and the cost it names is real:
> `hand_joint_read_rate` is a vendor round trip per sample, so 10 Hz to 50 Hz is
> five times the SDK load on that hand. Measure the bridge CPU before assuming
> it is free. The **idle** side stays throttled — it is never addressed.

| Knob | Default here | vs. Hefeweizen | What lowering it used to buy |
|---|---|---|---|
| `hand_pub_rate` (left) | 50 Hz | 50 Hz | ROS republish only; nothing closes a loop on it |
| `hand_joint_read_rate` (left) | 50 Hz | 20 Hz | the one rate that is still real CAN traffic and a vendor round trip |
| `idle_hand_pub_rate` (right) | 5 Hz | 50 Hz | the right hand is kept alive but never addressed |
| `idle_hand_joint_read_rate` (right) | 2 Hz | 20 Hz | pure background load |
| `use_rviz` on the arm bring-up | `false` | `true` | rviz rendering plus its planning-scene monitor is the largest non-node consumer |

**Deliberately not lowered:** the arm driver's `pub_rate` (200 Hz) and the MIT control rate. Those are
load bearing for control and gravity compensation, and the control rate is a hard requirement
(constraint C2) rather than a tuning knob.

*Superseded 2026-08-14:* an earlier version of this section named the arm driver's 200 Hz per-joint
SDK reads as the standing CPU target. They were measured at ~10 % of their own publish batch. The
real consumer was a vendor busy-wait inside the hand SDK, since patched, which took the whole stack
from 814 % of a core to 400 % idle.

`poll_period_sec` (coordinator scheduler tick, 0.05 s) bounds how fast a `Ctrl+C` is noticed. Do not
raise it far to save CPU — it is a safety latency, and at 20 Hz it costs nothing.

## How the chain is built

The activity is a strictly linear DAG — no `sync_flag`, no parallelism. The **edges** are what
serialize it: on the normal `dedicated_per_device` topology the left arm and left hand hold
independent resource tokens and *could* be scheduled together, so the ordering here is a property of
this graph rather than of the wiring. *Superseded 2026-08-11: this used to read "they share one
physical CAN bus, so `graph_model.ROBOT_UNITS` serializes them anyway", which was true of the shared
side bus and is not true of the deployed topology.*

| # | Action | Kind | Backing data |
|---|---|---|---|
| 10 | `left_hand_rest_fist` | hand pose | gesture `fist_vendor_demo` |
| 20 | `left_arm_to_teapot_grip_idle` | planned | anchor `Can_Grip_Idle_L` |
| 30 | `left_hand_pre_grip_handle` | hand pose | gesture `can_pre_grip` |
| 40 | `left_arm_to_teapot_pre_grip` | planned | anchor `Can_Pre_Grip_L` |
| 50 | `left_arm_teapot_handle_entry` | **replay** | recording `Grip_Can_L` (18 waypoints) |
| 60 | `left_arm_to_teapot_grip` | planned | anchor `Can_Grip_L` — seats the hand in the handle |
| 70 | `left_hand_grip_handle` | hand pose | gesture `can_grip_V01` |
| 80 | `left_arm_to_teapot_post_grip` | planned | anchor `Can_Post_Grip_L` |
| 90 | `left_arm_to_pour_init` | planned | anchor `Can_Fill_Init_L` |
| 100 | `left_arm_to_pour_idle` | planned | anchor `Can_Fill_Idle` |
| 110 | `left_arm_pour_tea` | **replay** | recording `CanFill02` (73 waypoints) |
| 120 | `left_arm_to_pour_init` | planned | anchor `Can_Fill_Init_L` (same action as 90) |
| 130 | `left_arm_to_teapot_pre_place` | planned | anchor `Pre_Place_Can_L` |
| 140 | `left_arm_to_teapot_place` | planned | anchor `Place_Can_L` |
| 150 | `left_hand_release_handle` | hand pose | gesture `can_pre_grip` |
| 160 | `left_arm_teapot_handle_release` | **replay** | recording `Can_Release_L_V02` (50 waypoints) |
| 170 | `left_hand_rest_fist` | hand pose | gesture `fist_vendor_demo` (same action as 10) |

### Hand: `pose`, not `close_until_contact`

The four hand skills use the new `pose` motion — a bounded ramp to a taught preset, no tactile
gating. The presets were captured on the real teapot, so the shape *is* the grasp.

The tradeoff is explicit: `pose` is deterministic and repeatable, but blind. It will close on empty
air just as happily if the handle is not where the anchor says it is. Tactile-gated grasping stays
available through the `grasp_*_until_contact` skills once a contact threshold is calibrated — the
0.35 placeholder in `catalogue.yaml` is orders of magnitude below the raw normal-force magnitudes the
Pro actually reports, so it would trigger on noise today.

### Payload: the arm is told when it is carrying the teapot

Action 70 (`left_hand_grip_handle`) carries `payload_update: attach` and action 150
(`left_hand_release_handle`) carries `payload_update: detach`. On success the coordinator calls
`/left_arm/mit_controller/payload_attached` **before** marking the node completed, so the lift at
action 80 cannot start under the unloaded gravity model. A failed transition aborts the activity —
lifting with the wrong model is worse than stopping.

The flag is on the **action**, never on the hand preset: actions 30 and 150 both run the
`can_pre_grip` shape, and only one of them means the teapot is gone.

If you bring the arms up without `payload_mass_kg:=1.0`, the controller has no loaded model, refuses
the attach, and the activity fails at action 70 with `no payload gravity model is configured`. That
is deliberate — it fails loudly rather than pouring under the wrong gravity compensation.

The mass (1.0 kg) and the 0.15 m lever are **estimates**, not measurements; the axis is derived from
the URDF. See `../../sprint6/reference/payload_gravity_model.md`.

### Arm: how a replay reaches its start

A recorded action is dispatched as **two** MoveIt goals, not one:

1. a planned, collision-aware `MoveGroup` move to the trajectory's own first waypoint
   (`recorded_approach_scaling`, 0.10 by default);
2. the `ExecuteTrajectory` replay.

That is what makes the preceding anchor a *staging* pose rather than a required start state. A taught
trajectory is recorded from wherever the arm happened to be, and `ExecuteTrajectory` only executes —
it does not plan, and MoveIt rejects a replay whose first point deviates from the current state by
more than `trajectory_execution.allowed_start_tolerance`. Resolving the offset with a planned move
keeps taught data untouched and works from wherever the arm actually is.

Measured offsets between each anchor and its recording's first waypoint:

| Replay | Anchor | Offset |
|---|---|---|
| `left_arm_teapot_handle_entry` | `Can_Pre_Grip_L` | 0.014 rad |
| `left_arm_pour_tea` | `Can_Fill_Idle` | **0.131 rad** (j3) |
| `left_arm_teapot_handle_release` | `Place_Can_L` | 0.010 rad |

The 0.131 rad is because `CanFill02` was taught starting where `CanFill01` ended. `CanFill01` is the
discarded first attempt and is deliberately not in the chain.

Note that **all three** exceed MoveIt's 0.01 rad default, which is why
`_moveit_config_builder._apply_trajectory_execution_tuning` now raises `allowed_start_tolerance` to
0.05 and `allowed_execution_duration_scaling` to 2.0. Override per launch with
`trajectory_start_tolerance:=…` / `trajectory_duration_scaling:=…`.

> The replay itself is **not** collision checked. It is only as safe as the taught motion was.

### `velocity_scaling` means two different things

For an **anchor** action it is a planner limit (0.10 here; 0.05 for node 60, which threads the hand
into the handle). For a **recorded** action it is a pure time stretch —
`arm_executor._recorded_time_scale` is `1/min(vel, acc)` — so the replays use 0.75, i.e. 1.33× slower
than taught. Setting a replay to 0.10 would stretch the 14.5 s pour to 145 s.

## Where the pieces live

| Thing | File |
|---|---|
| activity graph | `src/agx_arm_coordination/config/activities/tea_pour_left_v1.yaml` |
| actions + taught waypoints | `src/agx_arm_coordination/config/catalogue.d/tea_pour_left_v1.yaml` |
| anchor poses | `src/agx_arm_coordination/config/arm_config.yaml` |
| hand skill → preset | `src/agx_arm_ctrl/config/omnihand_skills.yaml` |
| hand presets | `src/agx_arm_ctrl/config/omnihand_pro_gestures.yaml` |
| source recordings | `~/agx_arm_trajectories/teach/*.json` |
| bring-up | `src/agx_arm_coordination/launch/start_tea_demo.launch.py` |

Catalogue actions may live in `config/catalogue.yaml` or in any `config/catalogue.d/*.yaml` fragment;
`graph_loader` merges them into one flat `action_id` namespace and rejects a fragment that redefines
an existing action. The tea demo uses a fragment because its ~140 taught waypoints would drown the
shared catalogue.

## What a taught replay keeps, and what it loses here (2026-08-25)

A catalogue inlines a **decimation** of the recording — `left_arm_pour_tea` is 73
waypoints out of 721 recorded samples. That is a storage decision, and it has
consequences the teach loop does not have:

- **the sparse list is a rougher command stream, not a gentler one.** With 73
  knots over 14.5 s almost every control tick sits inside a linear segment, so
  all the acceleration lands on the knots — 98 rad/s² of commanded acceleration
  against 16 for the same recording replayed through the teach loop.
- **an inlined block is still a decimation.** Reference the recording instead
  (below) to keep the full taught density.
- **velocities are now dispatched** (they were not before 2026-08-25, and the MIT
  controller reads a missing velocity as a commanded zero — the kd term braked
  against the position command). Regenerating a waypoint block also now places
  the waypoints by chord error rather than by even sample index, so corners and
  reversals survive a small budget.

Regenerate a block with `agx_arm_recorded_to_catalogue` after re-teaching; a
block pasted before 2026-08-25 still carries evenly-spaced waypoints. Detail:
`docs/sprint_refactor/reference/teach_replay_timebase.md`.

### Choosing the playback mode, and keeping the full density

Both limitations above are addressable per action. A recorded action may name how
it is replayed, and may reference its recording instead of inlining a decimation:

```yaml
metadata:
  source: recorded
  recording: recordings/tea_pour_left.json   # relative to the config dir
  playback: { mode: tempo_scale, speed_scale: 0.6 }
```

`mode` is one of `as_recorded`, `smooth` (the default, 0.3 s window),
`tempo_scale`, `speed_scale`, `maximize_speed`. The first three keep the taught
timing; `tempo_scale` also scales the clock, which is what a pour taught too
briskly needs. An unusable request is refused when the catalogue loads, and a
`tempo_scale` that would drive a joint past its speed limit is refused too,
naming the joint and the largest tempo that would pass.

**`playback` is the only timing authority on a recorded action.** The older
`velocity_scaling` / `acceleration_scaling` stretch the taught times before the
mode sees them, so the two would multiply — `tempo_scale: 0.5` under
`velocity_scaling: 0.5` is a quarter speed and neither number says so. Declaring
both is refused. An action with no `playback` block keeps the old behaviour;
those entries are deprecated and migrate over time.

Write the sidecar with:

```bash
ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue \
  ~/agx_arm_trajectories/teach/CanFill02.json \
  --action-id left_arm_pour_tea \
  --joint-prefix left_arm_ \
  --emit-recording src/agx_arm_coordination/config/recordings/tea_pour_left.json
```

It writes a lean file (times and positions only — 12% of the recording, full
density) and prints a `recording:` line to paste in place of the waypoint block.

`--joint-prefix` matters for a single-arm recording: the teach loop stores joint
names unprefixed (`joint1..7`), which is the right shape for *either* arm, so the
file does not say which one it was taught on. Prefixed, it does, and the planner
checks it against the group before commanding — a left-arm recording replayed on
the right arm is refused instead of mirrored. A duo merge prefixes both sides
already. Without it the ordering is still checked; only the side is taken on the
catalogue's word.

To try a different tempo without touching the catalogue, override for one run:

```bash
ros2 run agx_arm_coordination agx_arm_run_activity \
  --activity tea_pour_duo_v2 \
  --metadata-json '{"playback": {"mode": "tempo_scale", "speed_scale": 0.6}}'
```

Every recorded action is planned before the first one moves, so a recording that
cannot be replayed under the requested mode fails at the start rather than three
actions in. That preplanning is cancellable between actions — Ctrl+C during a
`speed_scale` prewarm abandons the rest of the graph instead of running it out.

## Re-teaching a segment

```bash
# 1. record with the teach manager (see teach_and_run.md), then
ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue \
    ~/agx_arm_trajectories/teach/CanFill03.json \
    --action-id left_arm_pour_tea --max-points 73
# 2. paste the emitted waypoints: block over the old one in
#    config/catalogue.d/tea_pour_left_v1.yaml
# 3. rebuild: bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_coordination
```

`--max-points` is chosen for roughly one waypoint every 0.2 s. You do not need to make the recording
start on an anchor — the coordinator's approach phase handles that.

## Validation status

**Validated on hardware 2026-08-17** (L3, commit `31c0350`): the composition documented above ran the
full activity end to end **three times** — at 16:49, 17:02 and 17:12, across two bring-ups. 90.0 s,
92.7 s and 93.8 s; within a stack no action differed by more than 0.9 s. 16 `FollowJointTrajectory`
goals per run, none rejected or replanned; both payload transitions in every completed run; 19 hand
claim/release pairs against 19 skill performs, so none skipped. A fourth run was cancelled two nodes
from the end and terminated cleanly. Full record, including the five non-fatal anomalies and the
first live attempt that aborted before the fix that enabled the rest:
`../../sprint6/evidence/tea_pour_left_v1_2026-08-17.md`.

What that run does **not** establish: nothing sampled CPU or CAN counters, so this runbook's rate
budget remains reasoned rather than measured *for this demo*; the payload mass is still an estimate;
and the interrupt tested at the end of the session landed on an idle stack, so the stop ladder
mid-motion is unexercised.

## Known gaps

- Anchor `Can_Grip_L` sits ~0.22 rad (j5) / 0.20 rad (j7) past the end of the handle-entry replay.
  That offset is intentional (it seats the hand in the handle) but has not been re-verified since the
  recording was taught.
- The hand grasp is open loop (see `pose` above). A closing gesture therefore exhausts the bridge's
  8-attempt delivery verification every time — five occurrences in the 2026-08-17 session, all
  benign, because the fingers are physically blocked and the bridge cannot yet tell contact from
  congestion.
- **The stop ladder is unexercised on hardware.** The 2026-08-17 `Ctrl+C` landed with no activity
  running, so it proved the idle exit and nothing about cancelling a moving arm.
- Coordinator crash recovery is not covered — only clean interrupts are.
- The teapot payload mass and lever are unmeasured estimates, and the payload state does not survive
  a MIT controller restart (deliberate for the MVP — re-establish it through the service before
  resuming motion). An activity that aborts while still gripping keeps the loaded model, which is the
  safer approximation.

## References

- `teach_and_run.md`: teach loop, gravity, emergency stop, and the degraded shared-bus mode
- `launches.md`: the full bring-up matrix
- `../../sprint_refactor/reference/critical_cpu_paths.md`: the CPU hot paths this runbook budgets against
- `../../sprint6/planning/hefeweizen_activity_graph.md`: the coordinator's graph/catalogue contract
- `../../sprint6/reference/payload_gravity_model.md`: the carried-payload gravity model and its flange-axis evidence
