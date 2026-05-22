# Nero MIT Gravity Hold, Trajectory Workflow, And Wakeword Demo Tooling

Status: targeted Sprint 2 workflow note for the current Nero MIT controller path and the adjacent wakeword demo tooling.

This note documents an adjacent workflow that is not a direct Physical AI roadmap gate by itself, but it is a useful integration slice for later TTS-driven and interaction-driven demos.

Use `docs/development/nero_physical_ai_roadmap.md` and `docs/development/nero_physical_ai_progress.md` for cross-sprint coordination.

## Goal

This is the final workflow that produced smooth gravity-assisted MIT hold on Nero and the current wakeword-triggered teach-and-playback demo flow.

Use it in this order:

1. launch the MIT stack with a controller YAML,
2. validate static hold with `agx_arm_test_position_hold`,
3. only then record and replay trajectories,
4. then add wakeword-triggered playback on top of the validated controller path.

The important design choice is that the controller owns all feedforward during playback. Leader-mode recordings do not replay hand-applied torques.

## Components

- `start_nero_mit_controller.launch.py`: launches `agx_arm_ctrl` plus the MIT controller and loads a parameter YAML through `params_file`.
- `agx_arm_test_position_hold`: captures the current pose, enables MIT hold, and reports drift.
- `agx_arm_record_leader_trajectory`: records pose targets from `feedback/leader_joint_angles`.
- `agx_arm_execute_saved_trajectory`: replays only positions and velocities from a saved JSON.
- `agx_arm_wakeword_motion_manager`: keeps a long-lived `idle` / `record` / `playback` state machine alive and exposes a trigger service for wakeword playback.
- `wakeword-benchmark/scripts/trigger_service_oww.py`: external wakeword listener that can call the motion-manager trigger service directly.

## Final Working Setup

These points are what actually mattered for the final stable behavior:

1. Validate gravity with a static hold test before using trajectories.
2. Do not replay torques recorded during Leader Mode.
3. Record trajectory positions from `feedback/leader_joint_angles`, because that topic reliably reflects manual motion in Leader Mode.
4. Switch back to Normal Mode before enabling MIT for playback.
5. Load controller gains and gravity settings from a startup YAML via `params_file`.
6. Leave `gravity_urdf_path` and `calibration_file` empty in the default Nero profiles to auto-discover the canonical URDF and `config/nero_gravity_calibration.json`.
7. Use `gravity_feedforward_sign: -1.0` in the MIT command path.
8. Keep the wakeword detector outside `agx_arm_ros` and let the ROS-side motion semantics stay inside the MIT demo package instead of the controller runtime package.

## Prerequisites

- ROS 2 Humble is sourced.
- The workspace is built.
- The Nero arm is available on the expected CAN port.
- The separate `wakeword-benchmark` workspace folder exists when you want to run the external listener.

Typical shell setup:

```bash
cd ~/workspace/agx_arm_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Launch

The default launch path already uses the packaged controller YAML:

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero
```

The default profile is `src/agx_arm_mit_controller/config/nero_mit_controller_defaults.yaml` and now enables gravity compensation by default.

Important detail:

- inside this colcon workspace, the launch now prefers the source YAML under `src/agx_arm_mit_controller/config/...` when that file exists,
- outside a source workspace, it falls back to the packaged copy under `install/agx_arm_mit_controller/share/...`,
- you can always bypass that packaged default by launching with an explicit `params_file:=...` override.

Key gravity fields in the default profile:

```text
gravity_compensation_enabled = true
gravity_backend = pinocchio
gravity_scale = 1.0
gravity_feedforward_sign = -1.0
gravity_urdf_path = ""          # auto-discover
calibration_file = ""           # auto-discover
```

## Basic Position Hold Test

Run this before any trajectory replay:

```bash
ros2 run agx_arm_mit_tools agx_arm_test_position_hold -- --duration 8.0
```

What it does:

1. queries the running MIT controller for gravity-related parameters,
2. switches the robot to Normal Mode,
3. waits for you to place the arm at the pose to test,
4. enables MIT and calls `mit_controller/hold_current`,
5. prints current and peak joint drift during the hold window.

Interpretation:

- If the arm sags or pushes the wrong way here, fix MIT hold or gravity compensation first.
- If hold is smooth here, trajectory problems are downstream of the gravity setup.

## Leader Recording

Record a trajectory only after hold is working:

```bash
ros2 run agx_arm_mit_demos agx_arm_record_leader_trajectory -- --output-dir ~/agx_arm_trajectories --auto-enable
```

What happens:

1. the recorder enables the arm if requested,
2. switches to Normal Mode,
3. switches to Leader Mode,
4. waits for `feedback/leader_joint_angles`,
5. records positions until the arm stays still for `--hold-timeout`.

Important note:

- The saved JSON contains trajectory points, but playback does not use recorded efforts as torque feedforward.

## Trajectory Replay

Replay with the MIT controller still running:

```bash
ros2 run agx_arm_mit_demos agx_arm_execute_saved_trajectory -- ~/agx_arm_trajectories/pick_demo.json
```

What happens:

1. the executor loads the JSON trajectory,
2. forces the robot back to Normal Mode,
3. waits for fresh `feedback/joint_states`,
4. enables MIT,
5. publishes the trajectory several times.

Replay behavior:

- position and velocity targets come from the JSON,
- effort feedforward from the JSON is discarded,
- gravity feedforward comes from the running MIT controller configuration.

## Wakeword-Triggered Teach And Playback

Use this when you want to teach 5 to 6 wakeword motions once and then trigger them from an external listener:

```bash
ros2 run agx_arm_mit_demos agx_arm_wakeword_motion_manager -- --auto-enable-arm --start-mode idle
```

Startup behavior:

- the manager does not launch the driver or the MIT controller itself,
- start the control stack first and then start the manager,
- with `--start-mode idle` or `--start-mode record` it waits for `set_normal_mode` and `set_leader_mode`,
- with `--start-mode playback` it also waits for `mit_controller/enable` and `mit_controller/hold_current`,
- `--startup-timeout 0` is the default and waits indefinitely; set a positive value only if you want an explicit timeout.

The manager keeps one process alive with three states:

1. `idle`: switches the robot into Leader Mode and keeps MIT disabled.
2. `record`: stays in Leader Mode for teaching, can record new samples, delete old samples, and temporarily switch to MIT `hold_current`.
3. `playback`: switches back to Normal Mode, enables MIT, captures a compliant hold target, and waits for trigger requests.

The default trigger service is:

```bash
ros2 service call /agx_arm_motion_manager/trigger_motion std_srvs/srv/Trigger "{}"
```

That service now reports whether the trigger request was accepted. The actual playback is executed immediately afterward from the manager loop, not directly inside the service callback.

Useful keys while the manager is running:

- `i`: idle / Leader Mode
- `r`: record mode
- `p`: playback mode
- `n`: record a new sample immediately in record mode
- `x`: delete the selected sample in record mode
- `[` and `]`: move across saved samples
- `1` through `9`: jump to a sample slot directly
- `m`: toggle deterministic versus random sample selection
- `f`: fire the selected or random sample immediately in playback mode
- `g`: refresh MIT `hold_current`
- `c`: cancel the active MIT trajectory in playback mode

The wakeword listener lives outside this repo and can call that service directly:

```bash
python3 ../wakeword-benchmark/scripts/trigger_service_oww.py \
  --model ../wakeword-benchmark/models/openwakeword/de_170526/mille_mani.tflite \
  --framework tflite \
  --threshold 0.6 \
  --consecutive-hits 2 \
  --cooldown 3.0 \
  --ros-trigger-service /agx_arm_motion_manager/trigger_motion \
  --device 24
```

Find the device number with:

```bash
cd ~/workspace/wakeword-benchmark
.venv-oww/bin/python - <<'PY'
import sounddevice as sd
print(sd.query_devices())
PY
```

If you use the external listener without `--ros-trigger-service`, its `--action` fallback now executes without a shell by default. Only enable `--action-shell` when shell features are explicitly needed.

That keeps wakeword detection outside the ROS package while moving the teach/playback application layer into `agx_arm_mit_demos` and keeping the runtime controller itself narrow.

## Parameter Profiles

Useful packaged profiles:

- `src/agx_arm_mit_controller/config/nero_mit_controller_defaults.yaml`
- `src/agx_arm_mit_controller/config/mit_playback_soft.yaml`
- `src/agx_arm_mit_controller/config/mit_playback_soft_gravity_template.yaml`

Use a different profile by restarting the launch with `params_file:=...`.

Example:

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero \
  params_file:=$PWD/src/agx_arm_mit_controller/config/mit_playback_soft.yaml
```

The gravity template works with the standard repo layout as-is because URDF and calibration paths are auto-discovered when left empty. Only set explicit paths if you want to override the defaults.

## Recovery And Inspection

If you need to recover the robot mode manually:

```bash
ros2 service call /set_normal_mode std_srvs/srv/Trigger "{}"
```

Enable MIT manually:

```bash
ros2 service call /mit_controller/enable std_srvs/srv/SetBool "{data: true}"
```

Capture the current pose as the MIT hold target:

```bash
ros2 service call /mit_controller/hold_current std_srvs/srv/Empty "{}"
```

Inspect the reference state published by the controller:

```bash
ros2 topic echo /mit_controller/reference_joint_states
```

Inspect loaded MIT parameters:

```bash
ros2 param get /mit_controller kp
ros2 param get /mit_controller kd
ros2 param get /mit_controller gravity_compensation_enabled
ros2 param get /mit_controller gravity_feedforward_sign
ros2 param get /mit_controller calibration_file
```

## Current Limitations

- MIT gains are still startup-time parameters; there is no live retuning path yet.
- The hold and replay workflow depends on fresh `feedback/joint_states` from `agx_arm_ctrl`.
- The static hold test is the authoritative gravity check; trajectory behavior still depends on the chosen gains and limits.
- The wakeword listener is still an external demo utility and has not yet been promoted into a stable repo-local runtime surface.