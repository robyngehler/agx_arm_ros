# Control Script Reference

This page is a reference map for repo-owned control and bringup scripts.

It does not replace `docs/control/bringups/launches.md` or `docs/control/bringups/teach_and_run.md`.

## Primary operational entrypoints

- `scripts/activate_native_can.sh`: native Jetson CAN bringup for the current side-bus baseline
- `scripts/colcon_build_system_python.sh`: workspace build wrapper
- `scripts/run_in_ros_conda.sh -- <command>`: Conda-backed runtime wrapper
- `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py ...`: canonical combined runtime entrypoint
- `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py ...`: MIT teach and replay baseline

## When to use this page

Use this page only when you need a quick role summary for scripts that are already documented in the
operational control docs.

## Script role summary

| Script or surface | Role |
| --- | --- |
| `scripts/activate_native_can.sh` | brings up the validated native CAN FD side buses and applies the required side-bus settings |
| `scripts/prepare_can_interfaces.py` | role-based CAN or CAN FD preparation for USB or fallback adapter setups |
| `scripts/colcon_build_system_python.sh` | keeps builds on system Python and filters stale or conflicting local environment state |
| `scripts/run_in_ros_conda.sh` | runs a ROS command inside the repo-owned Conda runtime after sourcing ROS and local overlays |
| `scripts/setup_agx_arm_runtime_env.sh` | creates or updates the repo-owned Conda runtime environment |

For usage details and stable command lines, go back to `../../control/bringups/launches.md` or
`../../control/environment.md`.