# agx_arm_mit_controller

ROS2 application package for Nero MIT joint-space control.

This package stays ROS-centric:

- it subscribes to arm feedback from `agx_arm_ctrl`,
- it runs a timer-driven MIT control loop in its own node,
- it publishes `agx_arm_msgs/MoveMITMsg` back to `agx_arm_ctrl`,
- it accepts standard `trajectory_msgs/JointTrajectory` commands.
- it includes an interactive leader-mode recorder and a saved-trajectory executor.
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
- saves a JSON file with trajectory points, effort observations, MDH flange poses, and URDF inertial metadata.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_record_leader_trajectory -- --output-dir ~/agx_arm_trajectories
```

## Saved Trajectory Execution

`agx_arm_execute_saved_trajectory` loads a saved JSON recording and publishes it to the MIT controller as a `trajectory_msgs/JointTrajectory`.

Example:

```bash
ros2 run agx_arm_mit_controller agx_arm_execute_saved_trajectory -- ~/agx_arm_trajectories/demo.json
```

## Model Validation And Calibration

`agx_arm_validate_urdf_mdh` compares pyAgxArm MDH FK against the canonical Nero URDF FK backend when available.

`agx_arm_compare_gravity` computes `tau_g(q)` with the configured gravity backend, logs measured torques against model torques, and writes CSV for calibration.

`agx_arm_fit_gravity_calibration` fits a simple per-joint scale-and-bias model from logged CSV data.

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