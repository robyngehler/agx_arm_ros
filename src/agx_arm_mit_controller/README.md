# agx_arm_mit_controller

ROS2 runtime MIT controller package for Nero.

This package owns the production MIT execution surface:

- it subscribes to arm feedback from `agx_arm_ctrl`,
- it runs the timer-driven MIT control loop,
- it exposes the production `FollowJointTrajectory` action for MoveIt,
- it publishes `agx_arm_msgs/MoveMITMsg` back to `agx_arm_ctrl`,
- it keeps the shared trajectory and gravity/feedforward libraries used by the adjacent demo and tool packages.

Interactive demo apps now live in `agx_arm_mit_demos`. Debug bridges, hold checks, and calibration helpers now live in `agx_arm_mit_tools`.

## Runtime Interfaces

- subscribe: `feedback/joint_states`
- subscribe: `feedback/leader_joint_angles`
- subscribe: `feedback/arm_status`
- publish: `control/move_mit`
- publish: `~/reference_joint_states`
- publish: `~/execution_state`
- action: `arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`)
- optional debug subscribe: `~/joint_trajectory` when `enable_debug_joint_trajectory_topic:=true`
- service: `~/enable` (`std_srvs/SetBool`)
- service: `~/hold_current` (`std_srvs/Empty`)
- service: `~/cancel_trajectory` (`std_srvs/Empty`)

## Native MoveIt And RViz Flow

The production control path is now:

```text
MoveIt / RViz
  -> arm_controller/follow_joint_trajectory
  -> mit_controller
  -> control/move_mit
  -> agx_arm_ctrl_single_node
```

The legacy standalone `FollowJointTrajectory` bridge is no longer part of the normal runtime surface. `agx_arm_moveit start_moveit.launch.py use_mit_controller:=true` expects `mit_controller` to already provide `arm_controller/follow_joint_trajectory`. `demo.launch.py` remains as a compatibility alias.

For RViz soft-target debugging, keep the MIT controller's topic input disabled by default and only enable it in explicit debug launches such as `agx_arm_ctrl start_agx_arm_components.launch.py mode:=debug_soft_target` or `start_single_agx_arm_rviz.launch.py use_mit_controller:=true control:=true`. Enable MIT explicitly before moving RViz sliders; the debug bridge no longer auto-arms the controller.

For the current Duo custom-model and prefixed-joint slice, use `start_single_agx_arm_rviz.launch.py` with `custom_model`, `custom_model_xacro_args`, `input_joint_prefix:=right_arm_`, and `tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'`. The launch now auto-derives the prefixed follow-side adapter and `right_arm_nero_tool0` TCP parent for the common single-arm Duo path; override `feedback_joint_prefix`, `follow_joint_states_topic`, or `tcp_parent_frame` only when that default path is not sufficient. `start_agx_arm_components.launch.py mode:=debug_soft_target` now forwards the same Duo custom-model hooks through the common wrapper. The MIT launch also auto-derives a matching gravity URDF from `custom_model` plus `custom_model_xacro_args` when gravity compensation is enabled, so the mounted base orientation is no longer forced back onto the canonical standalone Nero URDF. For Duo OmniHand profiles, that gravity slice now keeps the active hand as a fixed-pose payload by freezing the hand joints at their zero pose inside the derived URDF; the controller still owns only the seven Nero arm joints, and dynamic hand-pose compensation remains future work.

For prefixed MoveIt execution on the same staged Duo model, `start_agx_arm_moveit.launch.py`, `start_single_agx_arm_moveit.launch.py`, and `start_nero_mit_controller.launch.py` accept `input_joint_prefix`. `start_nero_mit_controller.launch.py` now also accepts `custom_model`, `custom_model_xacro_args`, and an optional explicit `gravity_urdf_path`; when `custom_model` is set, the launch resolves a per-instance gravity URDF slice before the controller starts. `start_agx_arm_components.launch.py mode:=moveit_mit` and the new canonical `start_agx_arm_moveit.launch.py` also expose `moveit_profile:=right_arm|left_arm|both_arms`; `both_arms` is driven through two declared `arm_instances`, one MIT controller per arm, and a merged prefixed feedback path back into MoveIt/RViz. Keep each MIT controller on canonical `joint1` through `joint7` inside its namespace, require a consistent configured prefix on incoming MoveIt trajectories, and strip that prefix at the action boundary.

## Controller Behavior

- fresh `feedback/joint_states` is required before enabling or accepting a trajectory goal
- MoveIt goals are validated for joint names, monotonic timestamps, start-state tolerance, and goal tolerances
- the controller rejects new MoveIt goals while another goal is active
- stale feedback pauses command publishing and aborts active action goals
- leader mode aborts active action goals and pauses MIT publishing
- arm-status faults abort active action goals and block new ones until cleared
- when a trajectory finishes successfully, the final waypoint becomes the hold target when `hold_final_point:=true`

## Launch

Standalone controller bringup:

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero_right \
  gravity_arm_side:=right \
  arm_type:=nero \
  effector_type:=agx_gripper \
  tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

This launch now also exposes:

- `launch_driver`: whether to start `agx_arm_ctrl` together with the MIT controller
- `enable_debug_joint_trajectory_topic`: guard the debug `~/joint_trajectory` input
- `control_rate_hz`, `params_file`, and `log_level`

Recommended common bringup from `agx_arm_ctrl`:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=right_arm \
  can_port:=can_nero_right \
  arm_type:=nero \
  effector_type:=agx_gripper
```

Debug soft-target mode:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=debug_soft_target \
  execution_profile:=right_arm \
  can_port:=can_nero_right \
  arm_type:=nero
```

For the current bringup matrix and the teach workflow, prefer `docs/control/bringups/launches.md` and `docs/control/bringups/teach_and_run.md` over duplicating launch combinations in package-local notes.

## Demo Package

Use `agx_arm_mit_demos` for the app-layer workflows:

- `agx_arm_record_leader_trajectory`
- `agx_arm_execute_saved_trajectory`
- `agx_arm_teach_manager`
- `agx_arm_wakeword_motion_manager`

Examples:

```bash
ros2 run agx_arm_mit_demos agx_arm_record_leader_trajectory -- --output-dir ~/agx_arm_trajectories
ros2 run agx_arm_mit_demos agx_arm_execute_saved_trajectory -- ~/agx_arm_trajectories/demo.json
ros2 run agx_arm_mit_demos agx_arm_teach_manager --arm-config src/agx_arm_coordination/config/arm_config.yaml --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
ros2 run agx_arm_mit_demos agx_arm_wakeword_motion_manager -- --auto-enable-arm --start-mode idle
```

For the current record, replay, anchor-capture, and teach-manager walkthrough, see `docs/control/bringups/teach_and_run.md`. Keep `docs/sprint2/evidence/mit_runtime_history.md` only as the historical Sprint 2 workflow note.

## Tools Package

Use `agx_arm_mit_tools` for the adjacent helper surface:

- `agx_arm_mit_joint_state_bridge`
- `agx_arm_test_position_hold`
- `agx_arm_validate_urdf_mdh`
- `agx_arm_compare_gravity`
- `agx_arm_fit_gravity_calibration`

Examples:

```bash
ros2 run agx_arm_mit_tools agx_arm_test_position_hold -- --duration 8.0
ros2 run agx_arm_mit_tools agx_arm_compare_gravity -- --can-port can_nero_right --duration 2.0 --rate 2.0 --csv-path logs/nero_gravity_dataset.csv
ros2 run agx_arm_mit_tools agx_arm_fit_gravity_calibration -- logs/nero_gravity_dataset.csv --output config/nero_gravity_calibration.json
```

## Gravity Notes

The currently validated gravity-enabled setup depends on these points:

- validate MIT hold first with `agx_arm_test_position_hold`
- recorded trajectory efforts are not replayed
- playback switches the robot back to Normal Mode before enabling MIT
- the controller YAML loaded through `params_file` remains the source of gain and gravity settings
- the default controller profile enables gravity compensation and auto-discovers the canonical Nero URDF plus `config/nero_gravity_calibration.json` when those paths are left empty
- the MIT command path uses `gravity_feedforward_sign=-1.0`

Start with conservative gains and validate on hardware carefully.