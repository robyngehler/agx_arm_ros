# agx_arm_mit_demos

ROS2 demo and workflow entry points for the Nero MIT controller stack.

This package owns interactive demo flows such as:

- the teach manager keyboard UI
- leader-mode recording
- saved-trajectory playback
- anchor-pose capture
- recorded-trajectory to catalogue conversion
- wakeword-triggered teach-and-playback workflows

The runtime MIT controller stays in `agx_arm_mit_controller`. This package depends on its shared libraries, but owns these demo implementations and entry points directly so package discovery and source ownership stay aligned.

## Current primary workflow

For the current hardware-first teach flow, use `agx_arm_teach_manager` as the primary entry point after bringing the arm up through `agx_arm_mit_controller/start_nero_mit_controller.launch.py`.

```bash
ros2 run agx_arm_mit_demos agx_arm_teach_manager \
	--arm-config src/agx_arm_coordination/config/arm_config.yaml \
	--source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
```

This package also owns:

- `agx_arm_record_leader_trajectory`
- `agx_arm_execute_saved_trajectory`
- `agx_arm_capture_anchor_pose`
- `agx_arm_recorded_to_catalogue`
- `agx_arm_wakeword_motion_manager`

Use `docs/control/teach_and_run.md` for the canonical teach, capture, replay, and coordination-facing conversion flow instead of reconstructing that workflow from the individual CLIs.