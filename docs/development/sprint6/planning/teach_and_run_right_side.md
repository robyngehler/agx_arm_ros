# Teach + Run — Right Side (Hefeweizen, hardware-first)

How to make the Hefeweizen task executable on the **right side** (the side currently
wired: `can_nero_right` up, hand + right arm via `nero_right_arm`) by **reusing** the
existing MIT demo/tools instead of building new motion code. The architecture stays
dual-arm; the left side mirrors this once it is connected.

## Tool reuse map (no new motion stack)

| Need | Reused tool | Package |
|---|---|---|
| freedrive / leader mode + record a motion | `agx_arm_record_leader_trajectory` | `agx_arm_mit_demos` |
| play a recorded motion | `agx_arm_execute_saved_trajectory` | `agx_arm_mit_demos` |
| **save a single joint config as a named anchor pose** | `agx_arm_capture_anchor_pose` *(new, this sprint)* | `agx_arm_mit_demos` |
| arm execution via FollowJointTrajectory | MIT controller's built-in FJT action (`action_name` param) | `agx_arm_mit_controller` |
| hand grasp/open/release as a skill | `omnihand_skill_controller` | `agx_arm_ctrl` |
| order arm + hand actions (DAG) | coordinator | `agx_arm_coordination` |
| recorded JSON → catalogue `waypoints` | `agx_arm_recorded_to_catalogue` *(new)* | `agx_arm_mit_demos` |
| **all of the above from one keyboard UI** | `agx_arm_teach_manager` *(new)* | `agx_arm_mit_demos` |

## Central teach tool (one process for the whole loop)

Instead of juggling the separate CLIs, `agx_arm_teach_manager` runs the full teach loop —
freedrive, record, capture anchor pose, playback, and waypoint conversion — from one keyboard UI
(modelled on the wakeword motion manager):

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
  --source-joints right_arm_joint1,right_arm_joint2,right_arm_joint3,right_arm_joint4,right_arm_joint5,right_arm_joint6,right_arm_joint7 \
  --arm-config src/agx_arm_coordination/config/arm_config.yaml
```

Keys: `i` idle/freedrive · `r` record (`n` to record) · `p` playback (`f` to play, `c` cancel) ·
`a` capture current pose → named anchor in `arm_config.yaml` · `w` convert the selected recording →
catalogue `waypoints` · `[`/`]` select recording · `s`/`h`/`q`. The individual CLIs below still
work for scripted/one-shot use; the manager just wires them together with the MIT/leader mode
switching.

## The execution seam (config → real providers)

- The MIT controller node exposes a `control_msgs/FollowJointTrajectory` action server named by
  the `action_name` parameter (default `arm_controller/follow_joint_trajectory`). For the Duo, each
  per-arm MIT controller runs under its own `namespace` (`left_arm` / `right_arm`), so the servers
  are `/<side>_arm/arm_controller/follow_joint_trajectory`. The combined
  `both_arms_controller/follow_joint_trajectory` is provided by the fan-out bridge (see the
  execution-seam note below); `start_both_arms_execution.launch.py` brings up both per-arm
  controllers and the bridge together.
- The coordinator's `arm_executor` reads `agx_arm_coordination/config/arm_config.yaml`:
  groups → `action_server` + `joint_names`, and named `poses`. It is already structured
  dual-arm (`both_arms`, `right_arm`, `left_arm`). **No arm_executor code change is needed** —
  right-side bring-up is a launch/config + teach exercise.
- The hand is driven by `omnihand_skill_controller` (semantic skill → vendor preset → SDK),
  **not** by MoveIt. MoveIt only models the arm; the O12 hand description was migrated so the
  model is clean, but the demo does not MoveIt-plan the fingers.

> **Execution seam (wired).** Planning and execution are separate surfaces. MoveIt's `both_arms`
> profile stays *planning-only* (`start_moveit` skips fake ros2_control; the `duo_arm` execution
> profile carries planning-only, driver-off arm instances). Execution is owned by
> `agx_arm_mit_controller/launch/start_both_arms_execution.launch.py`, which brings up one per-arm
> MIT controller per side (namespaced `/<side>_arm/arm_controller/follow_joint_trajectory`) plus the
> `both_arms` fan-out FJT bridge (`agx_arm_mit_tools`) that owns
> `both_arms_controller/follow_joint_trajectory` and splits the 14-joint goal by prefix to the two
> per-arm servers. `arm_config.yaml` already points the three groups at those real providers:
>
> ```bash
> ros2 launch agx_arm_mit_controller start_both_arms_execution.launch.py \
>   left_effector_type:=omnihand right_effector_type:=omnihand omnihand_backend_type:=sdk
> ```
>
> Use `arm_dry_run:=true` on the coordinator until anchor poses + recorded waypoints are taught;
> the joint vectors below are still placeholders.

## Step A — bring up the right side

```bash
# clean system-python build (see python_environment_workflow.md); the ~/.local cmake shim
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

The `arm_config.yaml` anchor poses (`Idle_R`, `Pre_Grip_R`, `grasp_R`, …) ship as all-zero
placeholders. Capture the real right-arm vectors:

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
- Record the measured vectors in `hefeweizen_validation_log.md`.

## Step C — teach the functional trajectories (cap opener, pour)

Use the leader recorder for the multi-waypoint motions:

```bash
ros2 run agx_arm_mit_demos agx_arm_record_leader_trajectory --name pour_profile_right
# -> ~/agx_arm_trajectories/pour_profile_right.json (RecordedTrajectory)
```

Then turn its sampled points into the matching catalogue action's `waypoints:`
(`positions` + `time_from_start_sec`) in `agx_arm_coordination/config/catalogue.yaml`. The
`arm_executor` replays `waypoints` as a FollowJointTrajectory goal. Use the converter (or the
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

# live once anchors/waypoints are taught: bring up the arm execution seam, then drop arm_dry_run
#   ros2 launch agx_arm_mit_controller start_both_arms_execution.launch.py omnihand_backend_type:=sdk
#   ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py   # arm_dry_run defaults false
```

## Calibration still owed on hardware (see hefeweizen_validation_log.md)

- replace the `zero` / `fist_vendor_demo` presets with measured O12 `open` / glass / bottle
  grasp poses; tactile `contact_threshold` / `stable_samples` per object (mock tactile is all
  zeros, so grasps only confirm on the SDK backend).
- the anchor poses and recorded waypoints above.
- pour angle / duration for a low-risk first demo.
