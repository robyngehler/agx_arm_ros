# agx_arm_mit_controller

ROS2 application package for Nero MIT joint-space control.

This package stays ROS-centric:

- it subscribes to arm feedback from `agx_arm_ctrl`,
- it runs a timer-driven MIT control loop in its own node,
- it publishes `agx_arm_msgs/MoveMITMsg` back to `agx_arm_ctrl`,
- it accepts standard `trajectory_msgs/JointTrajectory` commands.
- it includes an interactive leader-mode recorder and a saved-trajectory executor.
- it includes an interactive wakeword motion manager for stateful teach-and-trigger workflows.
- it includes a dedicated MIT position-hold test tool.
- it supports optional gravity feed-forward through a common-framework adapter and a simple calibration file.

## Interfaces

- subscribe: `feedback/joint_states`
- publish: `control/move_mit`
- subscribe: `~/joint_trajectory`
- publish: `~/reference_joint_states`
- service: `~/enable` (`std_srvs/SetBool`)
- service: `~/hold_current` (`std_srvs/Empty`)
- service: `~/cancel_trajectory` (`std_srvs/Empty`)

## Recorder Workflow

`agx_arm_record_leader_trajectory` is an interactive ROS tool that:

- enables the arm through `agx_arm_ctrl`,
- switches Nero into normal mode and then leader mode,
- waits for you to press Enter before recording,
- records `feedback/leader_joint_angles` until the arm stays still for a configurable hold time,
- trims the trailing stationary section,
- asks for a trajectory name,
- saves a JSON file with trajectory points, MDH flange poses, and URDF inertial metadata.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_record_leader_trajectory -- --output-dir ~/agx_arm_trajectories
```

## Saved Trajectory Execution

`agx_arm_execute_saved_trajectory` loads a saved JSON recording and publishes it to the MIT controller as a `trajectory_msgs/JointTrajectory`.

Trajectory efforts from the JSON are always ignored during playback. MIT feedforward comes from the running controller configuration, especially gravity compensation, not from hand-applied torques recorded in Leader Mode.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_execute_saved_trajectory -- ~/agx_arm_trajectories/demo.json
```

## Wakeword Motion Manager

`agx_arm_wakeword_motion_manager` keeps a long-lived `idle` / `record` / `playback` state machine alive for teach-and-trigger workflows.

Ongoing work: the wakeword-oriented application layer is still under active development and should be treated as evolving workflow tooling rather than a frozen repo contract.

It:

- uses Leader Mode as the idle state,
- records multiple wakeword variants into a trajectory library,
- can delete or reselect saved variants with keyboard shortcuts,
- switches into MIT hold for compliant playback,
- exposes `~/trigger_motion` as a `std_srvs/Trigger` service for external wakeword listeners.

The trigger service acknowledges accepted requests immediately and the manager then executes the queued playback from its main loop. That avoids re-entering ROS service calls from inside the trigger callback itself.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_wakeword_motion_manager -- --auto-enable-arm --start-mode idle
```

Important startup detail:

- the motion manager does not launch `agx_arm_ctrl` or the MIT controller for you,
- start the control stack first, for example with `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py ...`,
- in `idle` and `record` it only needs the Nero mode services such as `set_normal_mode` and `set_leader_mode`,
- in `playback` it additionally needs `mit_controller/enable` and `mit_controller/hold_current`,
- by default it now waits for the services needed by the selected `--start-mode`; use `--startup-timeout` if you want it to fail fast instead.

Then point the wakeword listener at the manager service:

```bash
python3 ../wakeword-benchmark/scripts/trigger_service_oww.py \
  --model ../wakeword-benchmark/models/openwakeword/de_170526/mille_mani.tflite \
  --framework tflite \
  --ros-trigger-service /agx_arm_motion_manager/trigger_motion
```

The external listener runs outside this repo. Its `--action` fallback now executes without a shell by default; only use `--action-shell` when shell features are genuinely required.

Keyboard summary:

- `i` idle leader mode
- `r` record mode
- `p` playback mode
- `n` record a new sample in record mode
- `x` delete the selected sample in record mode
- `f` fire the selected or random sample in playback mode
- `m` toggle deterministic versus random playback selection
- `g` refresh MIT `hold_current`

For a full end-to-end guide covering launch, recording, playback, and MIT gain experiments, see:

- `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md`

## Basic Position Hold Test

`agx_arm_test_position_hold` isolates MIT hold behavior from trajectory playback.

It:

- switches the robot to Normal Mode,
- enables MIT,
- calls `mit_controller/hold_current`,
- holds the captured pose for a fixed duration,
- prints the active gravity-related controller parameters,
- prints current and peak joint drift in radians.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_test_position_hold -- --duration 8.0
```

Use this first when gravity compensation looks wrong. If the arm cannot hold a static pose here, the problem is in MIT hold or gravity feedforward, not in trajectory recording or playback.

## Validated Gravity Setup

The working gravity-enabled setup depends on these points:

- MIT hold is validated first with `agx_arm_test_position_hold` before any trajectory playback.
- Recorded trajectory efforts are not replayed.
- The recorder uses `feedback/leader_joint_angles` because that stream reflects manual Leader Mode motion reliably.
- Playback switches the robot back to Normal Mode before enabling MIT.
- The launch file loads a controller YAML through `params_file`.
- The default controller profile enables gravity compensation and auto-discovers the canonical Nero URDF plus `config/nero_gravity_calibration.json` when those paths are left empty.
- The MIT command path applies `gravity_feedforward_sign=-1.0`, which is the sign that produced smooth hold behavior on hardware.

## Model Validation And Calibration

`agx_arm_validate_urdf_mdh` compares pyAgxArm MDH FK against the canonical Nero URDF FK backend when available.

`agx_arm_compare_gravity` computes `tau_g(q)` with the configured gravity backend, logs measured torques against model torques, and writes CSV for calibration.

`agx_arm_fit_gravity_calibration` fits a simple per-joint scale-and-bias model from logged CSV data.

Recommended gravity-calibration workflow:

1. Put the robot in a static pose.
2. Run a short capture:

```bash
ros2 run agx_arm_mit_controller agx_arm_compare_gravity -- --can-port can_nero --duration 2.0 --rate 2.0 --csv-path logs/nero_gravity_dataset.csv
```

3. Move the robot by hand or with a normal motion command to a clearly different pose.
4. Run another short capture and append it to the same CSV:

```bash
ros2 run agx_arm_mit_controller agx_arm_compare_gravity -- --can-port can_nero --duration 2.0 --rate 2.0 --csv-path logs/nero_gravity_dataset.csv --append
```

5. Repeat for several distinct static poses, then fit one calibration JSON from the combined dataset:

```bash
ros2 run agx_arm_mit_controller agx_arm_fit_gravity_calibration -- logs/nero_gravity_dataset.csv --output config/nero_gravity_calibration.json
```

You can also fit across several separate CSV files in one command:

```bash
ros2 run agx_arm_mit_controller agx_arm_fit_gravity_calibration -- logs/pose_a.csv logs/pose_b.csv logs/pose_c.csv --output config/nero_gravity_calibration.json
```

`agx_arm_fit_gravity_calibration` does not move the robot and does not need live CAN access. It only reads CSV files.

If Pinocchio is not installed yet, the compare and FK validation tools explain that clearly instead of failing mysteriously.

## Behavior

- When enabled, the node captures the current pose and starts a soft MIT hold.
- A received `JointTrajectory` is replayed with linear interpolation.
- Per-joint gains and torque limits come from ROS parameters.
- An optional gravity model plus calibration file can inject `tau_ff` automatically.
- If feedback becomes stale, command publishing pauses until feedback recovers.
- When a trajectory finishes, the final waypoint becomes the new hold target.

## Launch

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can0
```

## Typical command topic

```bash
ros2 topic pub /mit_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
  points: [
    {
      positions: [0.0, 0.2, 0.0, 0.3, 0.0, 0.0, 0.0],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

Start with conservative gains and validate on hardware carefully.