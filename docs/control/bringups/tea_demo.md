# Tea Pour Demo (`tea_pour_left_v1`) — Runbook

> **Superseded in part — V02 refactor.** Each device now has its own CAN bus
> (arms `can_nero_left`/`can_nero_right` native, hands `hand_left`/`hand_right`
> on USB-CAN FD adapters), so
> same-side arm and hand motion may run in parallel and the shared-bus hand
> window is a selectable degraded mode, not normal operation. This page still
> describes the **current code**, which resolves the hand interface from the arm
> bus; it is rewritten in phase 2A. See
> `docs/sprint_refactor/planning/integration_plan.md` (constraint C1).

The first end-to-end hardware demo: the **left** arm plus the **left** OmniHand pick up a teapot,
carry it to the pour station, pour, set it down and withdraw. The right arm and right hand are
brought up and stay live, but the activity never addresses them — they hold wherever they are.

Built on the sprint 6 coordinator. Both arms and both hands run, only the left side moves.

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
| `Ctrl+C` on the coordinator | cancels children → reopens any hand window → pins the moving arm (`cancel_trajectory` + `hold_current`) → exits |
| `Ctrl+C` a second time | escalates to `emergency_stop` on both sides, then exits immediately |
| the coordinator crashes | **not covered by any of the above.** The MIT controller streams a damped stop on its own shutdown and the driver puts the arm in a firmware MOVE-J hold before a recovery disconnect, but a hard crash of the coordinator alone leaves the MoveIt goal running |

The physical e-stop remains the only guaranteed stop: the Nero firmware has no MIT command watchdog,
so silence is not a safe state. See `teach_and_run.md` § Emergency stop / runaway.

## What runs where

| Layer | Command |
|---|---|
| CAN buses | `bash ./scripts/activate_native_can.sh` |
| arms + MoveIt | `start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=duo_hand` |
| hands + coordinator | `start_tea_demo.launch.py` |
| the activity | `ros2 run agx_arm_coordination run_activity --activity tea_pour_left_v1` |

### 1. Dry run first (no hardware)

```bash
ros2 launch agx_arm_coordination start_tea_demo.launch.py arm_dry_run:=true
ros2 run agx_arm_coordination run_activity --activity tea_pour_left_v1
```

Mock hands, no arm goals sent. Confirms the graph validates, every action resolves, and the
scheduler serializes the chain. It does **not** exercise planning, so it cannot tell you whether the
anchors are reachable.

### 2. Live bring-up

```bash
bash ./scripts/activate_native_can.sh

# Arms + MoveIt. use_rviz:=false is a CPU decision, not cosmetic (see below).
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit execution_profile:=duo_hand follow:=true \
  planning_pipelines:=ompl omnihand_backend_type:=sdk use_rviz:=false

# Hands + coordinator
ros2 launch agx_arm_coordination start_tea_demo.launch.py backend_type:=sdk

# Run it
ros2 run agx_arm_coordination run_activity --activity tea_pour_left_v1
```

Expect roughly two minutes: 8 planned anchor moves, 3 taught replays (4.6 s + 19.4 s + 13.2 s at the
0.75 time-stretch) and 5 hand actions, each of which opens and closes a shared-bus hand window.

## CPU budget

The Jetson, not the robot, is the binding constraint. When the host stalls, nothing drains the CAN RX
socket and the kernel drops hand response frames — see
`../../sprint_refactor/reference/critical_cpu_paths.md`. Every rate below is bus reliability, not
just headroom.

| Knob | Default here | vs. Hefeweizen | Why it is safe to lower |
|---|---|---|---|
| `hand_pub_rate` (left) | 20 Hz | 50 Hz | ROS republish only; nothing closes a loop on it |
| `hand_joint_read_rate` (left) | 10 Hz | 20 Hz | each poll is real CAN traffic on the shared bus (hot path 4) |
| `idle_hand_pub_rate` (right) | 5 Hz | 50 Hz | the right hand is kept alive but never addressed |
| `idle_hand_joint_read_rate` (right) | 2 Hz | 20 Hz | pure background load |
| `use_rviz` on the arm bring-up | `false` | `true` | rviz rendering plus its planning-scene monitor is hot path 5 |

**Deliberately not lowered:** the arm driver's `pub_rate` (200 Hz) and the MIT control rate. Those are
load bearing for control and gravity compensation, and they belong to the arm bring-up. Hot path 1
(the 200 Hz per-joint SDK reads) is a refactor target, not something to trim here.

`poll_period_sec` (coordinator scheduler tick, 0.05 s) bounds how fast a `Ctrl+C` is noticed. Do not
raise it far to save CPU — it is a safety latency, and at 20 Hz it costs nothing.

## How the chain is built

The activity is a strictly linear DAG — no `sync_flag`, no parallelism. The left arm and left hand
share one physical CAN bus, so `graph_model.ROBOT_UNITS` serializes them anyway; the edges make the
order explicit and leave no room to interleave a hand action into an arm move.

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

## Known gaps

- **Not validated on hardware.** Every claim above about planning, reachability and grasp quality is
  untested; only the graph, catalogue, planning-layer wiring and stop logic are covered by unit tests.
- Anchor `Can_Grip_L` sits ~0.22 rad (j5) / 0.20 rad (j7) past the end of the handle-entry replay.
  That offset is intentional (it seats the hand in the handle) but has not been re-verified since the
  recording was taught.
- The hand grasp is open loop (see `pose` above).
- Coordinator crash recovery is not covered — only clean interrupts are.

## References

- `teach_and_run.md`: teach loop, gravity, shared-bus details, emergency stop
- `launches.md`: the full bring-up matrix
- `../../sprint_refactor/reference/critical_cpu_paths.md`: the CPU hot paths this runbook budgets against
- `../../sprint6/planning/hefeweizen_activity_graph.md`: the coordinator's graph/catalogue contract
