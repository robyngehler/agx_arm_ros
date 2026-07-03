# Teach + Run (hardware-first)

How to teach and run coordinated arm(+hand) motions — single arm, either side, or both arms
simultaneously — by **reusing** the MIT demo/tools instead of building new motion code. Examples use
the right side (`can_nero_right`); the left side and dual-arm mirror the same commands (see
[bringup.md](bringup.md) for the argument matrix).

## Tool reuse map (no new motion stack)

| Need | Tool | Package |
|---|---|---|
| freedrive + record a motion | `agx_arm_record_leader_trajectory` | `agx_arm_mit_demos` |
| play a recorded motion | `agx_arm_execute_saved_trajectory` | `agx_arm_mit_demos` |
| save a joint config as a named anchor pose | `agx_arm_capture_anchor_pose` | `agx_arm_mit_demos` |
| arm execution (plan + execute) | MoveIt `move_group` (per-arm MIT controllers via `moveit_mit`) | `agx_arm_moveit` |
| hand grasp/open/release as a skill | `omnihand_skill_controller` | `agx_arm_ctrl` |
| order arm + hand actions (DAG) | coordinator | `agx_arm_coordination` |
| recorded JSON → catalogue `waypoints` | `agx_arm_recorded_to_catalogue` | `agx_arm_mit_demos` |
| **all of the above from one keyboard UI** | `agx_arm_teach_manager` | `agx_arm_mit_demos` |

## Primary path: the teach manager

`agx_arm_teach_manager` is **the** entry point for teaching — freedrive, trajectory recording,
anchor-pose capture, playback, and waypoint conversion — from one keyboard UI (modelled on the
wakeword motion manager). Use it instead of the raw CLIs: on its own, `agx_arm_record_leader_trajectory`
/ `agx_arm_capture_anchor_pose` require you to already have freedrive enabled and the mode switching
right; the manager does that for you and switches modes **safely** (MIT off during freedrive; on
playback it enables MIT and captures the current pose so the arm never snaps to a stale target).

### 1. Bring up the arm (dependency — must run first)

The teach manager is a wrapper; it does **not** start the arm. Bring up the right arm's MIT
controller + driver first (it provides normal mode, `enable_agx_arm`, `mit_controller/enable` +
`mit_controller/freedrive` + `hold_current`, and the `feedback/*` topics). Un-namespaced, so the
manager's default service/topic names match:

```bash
bash ./scripts/activate_native_can.sh        # can_nero_right up (see Step A for the hand)
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero_right \
  gravity_arm_side:=right \
  effector_type:=omnihand
# gravity_arm_side:=right bakes the real body mount (tilt) into the gravity URDF;
# effector_type:=omnihand folds the ~1 kg OmniHand into the gravity model (articulated by default).
```

> **`effector_type:=omnihand` is required when the OmniHand is mounted.** `gravity_arm_side` bakes the
> body mount (tilt) into the gravity URDF, but the end-effector load is only folded in when
> `effector_type:=omnihand` — that flag makes the gravity slice keep the hand (`use_right_hand:=true`;
> verified: the gravity URDF grows from 4.58 kg to 5.64 kg, +1.04 kg of hand). Without it,
> `gravity_arm_side` alone builds a hand-less model and the arm sags/drifts under the real hand. It is
> otherwise harmless on this bring-up: on the driver it only merges `feedback/omnihand/joint_states`
> into the combined `feedback/joint_states` (no extra CAN traffic), and the teach loop reads
> `joint1..7` by name regardless.

> **`gravity_hand_payload:=articulated` (default) tracks the live finger pose.** The hand joints stay
> movable in the gravity URDF and the MIT controller feeds the hand joint states from the combined
> `feedback/joint_states` into pinocchio by name (URDF mimic couplings for the underactuated distal
> joints are re-applied), so the payload COM follows fist/open instead of assuming the frozen zero
> pose. With no hand feedback the result is identical to the legacy rigid payload;
> `gravity_hand_payload:=static` restores the frozen behavior explicitly.

> **A stale gravity calibration is no longer auto-applied on custom gravity URDFs.** The auto-discovered
> `config/nero_gravity_calibration.json` was least-squares-fitted on an upright, hand-less log; its
> per-joint scale (×1.21/×0.72/×0.78 on joints 2–4) and bias would distort the now-correct duo+hand
> model by whole newton-meters — far more than the finger-pose effect. When a custom gravity URDF is in
> play (any `gravity_arm_side`/`custom_model` bring-up) the MIT controller now skips the auto-load and
> logs a warning; pass `calibration_file:=...` explicitly once a calibration recorded **with** the hand
> and mount exists (record freedrive residuals via `compare_gravity --urdf-path <generated urdf>`,
> then fit with `fit_gravity_calibration`).

> **`gravity_arm_side:=right|left` is required for a body-mounted arm.** The arm base is tilted on the
> torso (`body_to_right_arm_mount` in `duo_body.xacro` is a 90° pitch), so a gravity model built from the
> standalone *upright* Nero URDF puts gravity on the wrong axis — it commands ~0 on joint1/joint3 while
> those actually carry the load, and the arm drifts sideways. `gravity_arm_side` derives the gravity URDF
> from `duo_system.urdf.xacro` for that side, so the world→base mount is **baked in** (ground truth from
> the description) and pinocchio computes gravity correctly — no hand-typed angle. Leave it empty only for
> a standalone arm physically mounted upright. (`gravity_mounting_rpy` remains as a manual override for a
> one-off rig; keep it `[0,0,0]` when `gravity_arm_side` is set, or the mount is applied twice.)

If the services are not up yet, the manager waits and prints exactly this command (it never
free-runs without the arm).

> **Do not set `input_joint_prefix` for the standalone teach loop.** The driver publishes
> `feedback/joint_states` with the raw nero joint names `joint1..joint7` (unprefixed), and MIT
> playback replays the *recorded* joint names straight back to the controller. If `input_joint_prefix`
> is `right_arm_`, the controller rejects any trajectory whose names do not start with `right_arm_`, so
> there is no single `--source-joints` value that makes both recording (needs `joint1..7` to match the
> feedback) **and** playback (would need `right_arm_joint1..7`) work. Keep the whole teach loop
> unprefixed: launch **without** `input_joint_prefix` and pass `--source-joints joint1,…,joint7`. The
> `input_joint_prefix:=right_arm_` argument belongs to the dual-arm **MoveIt** slice, where MoveIt/the
> coordinator send `right_arm_`-prefixed goals — not to this bring-up. (The reverse direction works
> since 2026-07-03: a **prefixed** MoveIt bring-up also accepts trajectories with the raw
> `joint1..7` names, so teach-manager playback over the debug topic runs against the same
> `moveit_mit` slice used for transitions — pass `enable_debug_joint_trajectory_topic:=true`.)

> **Freedrive = software leader mode (mounting-pose aware).** The teach manager no longer uses the
> firmware drag mode (`set_leader_mode`): that mode compensates gravity in firmware with **no** hook
> for the mounting pose or end-effector, so it feels wrong on a tilted body mount. Instead `idle`/
> `record` call `mit_controller/freedrive`, which drives a zero-force MIT command (kp=0, `freedrive_kd`
> damping, gravity feedforward) using the pinocchio gravity model. Set `gravity_mounting_rpy` to the
> arm base orientation in world `[roll, pitch, yaw]` (XYZ extrinsic, rad) so compensation is correct;
> `[0,0,0]` is an upright table mount. Because the arm stays in **normal** mode, `feedback/joint_states`
> stays live — recordings are sourced from `--source-joints` on that topic (same order as anchor
> capture), not from `feedback/leader_joint_angles`. Tune `freedrive_kd` on hardware: raise if the arm
> drifts/oscillates, lower if it feels sticky.

> **Shared arm+hand CAN bus — keep the hand alive.** On the right side the arm and the OmniHand share one
> physical bus (`can_nero_right`; there are only two mttcan channels, one per arm). The arm firmware pushes
> feedback autonomously and the MIT controller adds 7 command frames per control cycle; the OmniHand's CANFD
> IDs are high (low arbitration priority), so under arm load with `one-shot on` the hand loses arbitration,
> its frames are dropped, and the bridge goes silent (`请求超时`) the moment the MIT controller starts — while
> idle-hold is fine. To keep the hand alive on the shared bus:
> - **first tests: keep `one-shot on` (the validated arm ENOBUFS mitigation) and just deepen the TX ring** —
>   `TX_QUEUE_LEN=1000 sudo bash ./scripts/activate_native_can.sh right` — so an arm command burst fits the
>   queue (avoids ENOBUFS `[105]`) without changing retransmission behaviour.
> - keep `control_rate_hz` at its 50 Hz default (do **not** drop it — a lower rate risks hold instability);
>   handle any residual softness with the stiffer/more-damped MIT hold gains (`kp`/`kd`) instead.
> - only if the hand still starves under arm load, fall back to `ONE_SHOT=off` (lets hand frames retry) — but
>   then watch the arm for ENOBUFS, since retransmission buildup returns.
> - **`pub_rate` is not the bus lever:** it only sets the ROS republish rate of already-cached feedback (the
>   arm firmware push rate is fixed), so lowering it does not reduce CAN traffic — it only makes teach
>   recording sample staler feedback. Leave it at its default.
> - **the bridge's `joint_read_rate` IS a bus lever** (unlike the arm's `pub_rate`): every hand joint
>   readback is a real CANFD request/response. It now defaults to 20 Hz, decoupled from the 50 Hz ROS
>   republish of the cached state — 30 fewer request frames per second on the shared bus, still plenty
>   for teach recordings and command verification.
> - **hand commands are now verified and re-sent by the bridge** (`command_retry_*` parameters,
>   default 8 attempts every 0.3 s, 0.10 rad tolerance — eventual delivery matters more than the
>   ~2.4 s worst case): a dropped gesture command ("every 3rd
>   `omnihand_exerciser` call gets through") is retried automatically until the joint readback confirms
>   it; `feedback/omnihand/status.status_text` shows `command_retry i/n pending` while in flight, and a
>   give-up (fingers in contact, or bus dead) is logged once. `control/omnihand/stop` clears any
>   pending retry.

### 2. Run the teach manager

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager --arm-config src/agx_arm_coordination/config/arm_config.yaml --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
```

For first hardware playback tests, add a slower speed scale and a lead-in from the current hold pose:

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7 \
  --playback-speed-scale 0.25 \
  --playback-lead-in-sec 2.0 \
  --publish-repetitions 1
```

- `--playback-speed-scale`: direct teach-manager replay speed; `0.25` = quarter-speed, `1.0` = recorded speed.
- `--playback-lead-in-sec`: inserts a linear blend from the current hold pose to the first recorded waypoint, so the start is not limited to the raw recorded first sample.
- `--publish-repetitions 1`: recommended for cautious first tests; repeated debug publishes restart the same debug trajectory.

> `--source-joints` are the joint names **as they appear on `feedback/joint_states`** — on this
> un-namespaced, unprefixed bring-up that is `joint1..joint7` (the tool prints the names it sees if one
> is missing). Recordings and anchor captures are stored in this order; align the captured vector with
> the group's `joint_names` when you convert to catalogue waypoints for the dual-arm slice.

Keys: `i` idle/freedrive · `r` record (`n` to record) · `p` playback (`f` to play, `c` cancel) ·
`t` transitions (`f` to plan selected anchor target, `f` again to execute the cached MoveIt plan, `c`
to clear it) · `a` capture current pose → named anchor in `arm_config.yaml` · `w` convert the selected
recording → catalogue `waypoints` · `[`/`]` select recording or anchor target · `s`/`h`/`q`.

Internally the manager **reuses** the same building blocks as the CLIs below (the leader recorder,
saved-trajectory executor, `capture_anchor_pose`'s averaging, and `recorded_to_catalogue`) — no
duplicated motion code. The individual CLIs remain for scripted/one-shot use.

### Current playback scaling behavior

- Teach-manager `f` playback is **direct MIT replay**, not MoveIt: it republishes the recorded trajectory to the controller's debug `~/joint_trajectory` input.
- Anchor-pose transitions are **not** played inside the teach manager. Those are tested through the MoveIt/coordinator path, where `velocity_scaling` and `acceleration_scaling` already slow the `MoveGroup` request.
- Direct recorded playback now has two explicit runtime levers: `--playback-speed-scale` stretches all recorded timestamps, and `--playback-lead-in-sec` inserts a current-pose entry segment before the first recorded waypoint.
- The controller still validates the start state. Without a lead-in, the first recorded pose must be within `start_state_tolerance` of the current pose or the replay is rejected.

### MoveIt-backed anchor transition tests from the teach manager

- Start the normal MoveIt+MIT slice first, for example `start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm` or `execution_profile:=duo_arm`.
- Use the `execution_profile` as the mounted-slice ground truth. That preset resolves the Duo model,
  arm/hand composition, prefixes/frames, and the downstream gravity slice. If a different mounted
  assembly is needed, change `src/agx_arm_ctrl/config/execution_profiles.yaml` instead of rebuilding the
  same selection through one-off top-level launch arguments.
- Enter `t` transitions mode in the teach manager. The manager reads `arm_config.yaml`, derives the
  matching anchor targets for the current session (`right_arm`, `left_arm`, and when available
  `both_arms`), and keeps the arm controllers in MIT hold rather than freedrive.
- Use `[` and `]` to select the target anchor pose.
- First `f` sends a `MoveGroup` request with `plan_only=true` and caches the resulting trajectory.
- Second `f` sends that cached `RobotTrajectory` through `ExecuteTrajectory`.

Conservative defaults are exposed directly on the teach manager CLI:

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7 \
  --transition-velocity-scaling 0.10 \
  --transition-acceleration-scaling 0.10
```

This is the clean path to test anchor-pose actions before building activities out of them.

## The execution seam (coordinator → MoveIt slice)

The coordinator does **not** own an arm-execution path: it dispatches arm motion through the
**MoveIt multi-arm slice**, and MoveIt fans a `both_arms` plan out to the per-arm controllers
natively (its `MoveItSimpleControllerManager` splits by joint membership).

- **Bring-up = one slice:** `start_agx_arm_components.launch.py execution_profile:=duo_arm
  mode:=moveit_mit` brings up **both per-arm MIT controllers** (namespaced
  `/<side>_arm/arm_controller/follow_joint_trajectory`) **and `move_group`** together (see
  `start_agx_arm_moveit`, `use_mit_controller:=true`). There is no separate both_arms controller.
- **Coordinator dispatch:** `arm_executor` builds a `MoveGroupPlan` (anchor `to_pose`) or a
  `RecordedTrajectoryPlan` (recorded `waypoints`); the coordinator sends `moveit_msgs/MoveGroup`
  (collision-aware plan + execute, `/move_action`) or `moveit_msgs/ExecuteTrajectory`
  (`/execute_trajectory`). The planning group + joint names come from the motion registry;
  `arm_config.yaml` only carries the group list, the two MoveIt action names, poses, and defaults.
- For recorded `waypoints`, the coordinator now stretches `time_from_start_sec` by the more conservative
  of `velocity_scaling` and `acceleration_scaling`. Example: `0.10` means roughly 10x slower timing than
  the taught timestamps.
- The hand is driven by `omnihand_skill_controller` (semantic skill → vendor preset → SDK),
  **not** by MoveIt. MoveIt only models the arm; the O12 hand description was migrated so the
  model is clean, but the demo does not MoveIt-plan the fingers.

> `entry_pose` on recorded catalogue actions is currently descriptive routing metadata, not an enforced
> runtime precondition by itself. The safe path is therefore: test the recorded motion directly in the
> teach manager first, then run it in an activity only after the preceding anchor transition is taught and
> validated.

> Use `arm_dry_run:=true` on the coordinator until anchor poses + recorded waypoints are taught;
> the joint vectors below are still placeholders. If `move_group` runs under a namespace, override
> `move_group_action` / `execute_trajectory_action` in `arm_config.yaml`.

## Dual-arm teach + duo-vs-parallel routing

Both arms can be taught **simultaneously** to capture time-dependent, synchronized trajectories.
The teach manager is arm-count-aware: pass one namespace per arm. Each arm is its own namespaced MIT
stack that internally uses the **unprefixed** joints `joint1..7` — the namespace, not a joint prefix,
separates the arms (never bring two arms up un-namespaced: both would publish `joint1..7` on the same
`feedback/joint_states` and collide).

```bash
# one MIT bring-up per arm, each namespaced (own CAN bus, own gravity mounting pose)
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  namespace:=left_arm  can_port:=can_nero_left  gravity_mounting_rpy:="[...]"
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  namespace:=right_arm can_port:=can_nero_right gravity_mounting_rpy:="[...]"

# one teach manager driving both; record captures both arms on ONE clock
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml \
  --arms left_arm right_arm \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
```

**Resource choice at save time (not a hardcoded rule).** With two arms, `record` (`n`) and anchor
(`a`) ask which resource to store the result as: `both_arms` (merged **14-dim**, left then right),
or a single side (**7-dim**). If you pick `both_arms` while one arm stood still, that arm is stored
as a constant hold — which is a valid "hold while the other works" duo action.

**Storage stays arm-agnostic; the coordinator decides how to run it.** Two representations are both
legal (no `PerformAction.action` change — `both_arms` is already a first-class `robot_id`/resource and
pose dimension is generic):
- **Duo action** (`robot_id: both_arms`, 14-dim): one MoveIt `both_arms` goal. `MoveItSimpleControllerManager`
  splits by joint membership to each side's `FollowJointTrajectory`; the `joint_state_merger` re-prefixes
  each side's feedback onto `feedback/prefixed_joint_states`. **One trajectory, one time
  parameterization = genuine sync.**
- **Two per-arm actions** (`left_arm` + `right_arm`) with the same `sync_flag`: the scheduler releases
  them as a barrier group (start together).

Convert a taught duo motion to catalogue waypoints by merging the two per-arm recordings:

```bash
ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue LEFT.json \
  --merge-with RIGHT.json --action-id both_arms_pour_profile_v1
# emits 14-dim waypoints in registry order (left_arm_* then right_arm_*); the block echoes the
# joint order — cross-check it against group_joint_names('both_arms') before pasting.
```

### The parallelism constraint (important)

Two separate `ExecuteTrajectory` goals to the **same** `move_group` **serialize** — MoveIt executes one
at a time. So "two parallel actions" over MoveIt is *not* actually simultaneous. Genuine simultaneous
dual-arm motion needs one of:
1. **A merged `both_arms` trajectory** (one goal) — the sync path. The coordinator does this
   **automatically**: when two per-arm Trajectory actions (`left_arm` + `right_arm`) share a
   `sync_flag`, it merges their plans into one `both_arms` goal at dispatch (`merge_arm_plans` in
   `arm_executor`) — recorded pairs onto a shared timeline via `ExecuteTrajectory`, anchor pairs into
   one collision-aware `MoveGroup` goal. Not-taught / mixed / non-arm groups fall back to independent
   dispatch unchanged.
2. **Direct per-arm `FollowJointTrajectory`** (bypassing move_group) — two different controllers run
   concurrently, replay-only (no live planning). This is what the teach manager's own `f` playback does
   (it needs `enable_debug_joint_trajectory_topic:=true` on the MIT bring-up).

### Single-arm action while the other arm is present

Route a 7-dim action to its **own** group (`left_arm`/`right_arm`), not `both_arms`: MoveIt plans only
that side; the other arm gets no trajectory and its MIT controller **holds** its pose. Because the
merged feedback carries both arms, the moving arm's plan is collision-aware against the static one.
Making the idle arm actively **make space / dodge** is a planning-time decision (a `both_arms` goal
that constrains only the working arm and leaves the other free) — coordinator policy, not a replay.

## Step A — bring up the side (example: right)

```bash
# clean system-python build (see ../project/python_environment_workflow.md); the ~/.local cmake shim
# must not shadow /usr/bin/cmake for ament_cmake packages.
bash ./scripts/colcon_build_system_python.sh --packages-select \
  agx_arm_msgs agx_arm_ctrl agx_arm_coordination agx_arm_mit_controller agx_arm_mit_demos
source install/setup.bash

# native CAN side bus (right)
bash ./scripts/activate_native_can.sh        # brings up can_nero_right

# OmniHand bridge (right, real SDK) — self-locates the vendor pkg, opens can_nero_right
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
  backend_type:=sdk omnihand_type:=right hand_model:=o12_pro
```

## Step B — teach the anchor poses (freedrive + capture)

> **Recommended: use the teach manager** (§Primary path) — press `i` for freedrive, hand-move the
> arm, press `a` and name the pose. The manager handles freedrive + the safe mode switch for you.
> The raw CLI below is the scripted/one-shot alternative and needs freedrive already active.

The `arm_config.yaml` anchor poses (`Idle_R`, `Pre_Grip_R`, `grasp_R`, …) ship as all-zero
placeholders. Capture the real right-arm vectors (scripted alternative):

```bash
# put the right arm in leader/freedrive (the recorder enters leader mode; or use the
# arm's freedrive service directly), hand-move it to the target, then in another shell:
ros2 run agx_arm_mit_demos agx_arm_capture_anchor_pose \
  --pose-name Pre_Grip_R \
  --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7 \
  --config src/agx_arm_coordination/config/arm_config.yaml
```

- `--source-joints` is the joint-name order **on the live feedback topic** (the tool prints the
  names it sees if a name is missing). The captured vector is stored in that order, so align it
  with the group's `joint_names` (the `_R` half of `both_arms`).
- Default source topic is `feedback/joint_states` (override with `--source-topic`).
- Repeat for every `*_R` anchor. Rebuild `agx_arm_coordination` (or symlink-install) afterwards.
- Record the measured vectors in `../development/sprint6/planning/hefeweizen_validation_log.md`.

## Step C — teach the functional trajectories (cap opener, pour)

Use the leader recorder for the multi-waypoint motions:

```bash
ros2 run agx_arm_mit_demos agx_arm_record_leader_trajectory --name pour_profile_right
# -> ~/agx_arm_trajectories/pour_profile_right.json (RecordedTrajectory)
```

Then turn its sampled points into the matching catalogue action's `waypoints:`
(`positions` + `time_from_start_sec`) in `agx_arm_coordination/config/catalogue.yaml`. The
`arm_executor` replays `waypoints` through MoveIt's `ExecuteTrajectory`. Use the converter (or the
teach manager's `w` key) instead of transcribing by hand:

```bash
ros2 run agx_arm_mit_demos agx_arm_recorded_to_catalogue \
  ~/agx_arm_trajectories/pour_profile_right.json \
  --action-id both_arms_pour_profile_v1 --max-points 8
```

It downsamples to a few timed waypoints and writes a paste-ready `waypoints:` block (sidecar
`<action_id>.waypoints.yaml` + stdout). It deliberately does **not** rewrite `catalogue.yaml`
in place (flow-style metadata + comments would be clobbered); paste the block under the action's
`metadata` and confirm the echoed recorded joint order matches the group's `joint_names`.

## Step D — dry-run, then live

```bash
# scheduling/routing only, no arm motion (hand open/release still safe to exercise)
ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py arm_dry_run:=true

# live once anchors/waypoints are taught: bring up the MoveIt slice (per-arm controllers +
# move_group), then run the coordinator without arm_dry_run
#   ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
#     execution_profile:=duo_arm mode:=moveit_mit omnihand_backend_type:=sdk
#   ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py   # arm_dry_run defaults false
```

## Gravity-compensation tuning on hardware (freedrive is the clean test)

Freedrive runs at `kp=0`, so the gravity feedforward is the **only** thing holding the arm — it
exposes any sign/scale error that the position gain masks in the trajectory path. The controller
publishes the commanded gravity torque so you can compare it against the measured motor effort at the
same pose, and the gravity/gain knobs are **live-tunable** (no relaunch):

```bash
# in freedrive (teach manager 'i'), move the arm so joint2/joint4 are horizontal (max load) and watch:
ros2 topic echo /mit_controller/gravity_feedforward   # commanded gravity torque (effort[])
ros2 topic echo /feedback/joint_states                # effort[] = measured motor torque
```

- **Opposite signs** (commanded vs measured) → flip the sign:
  `ros2 param set /mit_controller gravity_feedforward_sign 1.0`
- **Same sign, too small** (arm still sags) → raise the scale:
  `ros2 param set /mit_controller gravity_scale 1.3` (step up until it floats).
- **Commanded ≈ measured but arm still won't hold** → gravity is right; the issue is damping/stiffness,
  tune `freedrive_kd` (lower = easier to move) or the hold `kp`/`kd`.
- **Vibration in normal hold** is usually a wrong-signed or mis-scaled gravity feedforward fighting
  `kp`; fix gravity first (above), then retune `kp`/`kd`.
- **Sideways drift / arm won't hold laterally = wrong gravity axis, not scale/sign.** A body-mounted arm
  is tilted, so launch with `gravity_arm_side:=right|left` (bakes the mount into the gravity URDF). Symptom
  of a missing/incorrect mount: `gravity_feedforward` shows ~0 on joint1/joint3 while the arm clearly needs
  torque there. This needs a relaunch (URDF is built at start-up).

Persist the values that work into `nero_mit_controller_defaults.yaml`.

## Calibration still owed on hardware (see ../development/sprint6/planning/hefeweizen_validation_log.md)

- replace the `zero` / `fist_vendor_demo` presets with measured O12 `open` / glass / bottle
  grasp poses; tactile `contact_threshold` / `stable_samples` per object (mock tactile is all
  zeros, so grasps only confirm on the SDK backend).
- the anchor poses and recorded waypoints above.
- pour angle / duration for a low-risk first demo.
