# Control Script Reference

This page is a reference map for repo-owned control and bringup scripts.

It does not replace `docs/control/bringups/launches.md` or `docs/control/bringups/teach_and_run.md`.

## Primary operational entrypoints

- `scripts/activate_stack.sh`: the operator entry point — activates the buses, verifies they carry
  clean traffic, and reloads the CAN driver when they do not
- `scripts/activate_duo_can.sh`: the bring-up itself, matched by physical slot; called by the above
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
| `scripts/activate_duo_can.sh` | brings up all four Duo buses by physical slot and applies bitrates, one-shot, txqueuelen, rmem and the arm-side TDC offset |
| `scripts/activate_stack.sh` | wraps the above: samples each bus after bring-up (RX advancing on the arms, controller ERROR-ACTIVE, error counters flat) and runs a bounded `rmmod`/`modprobe`/reactivate cycle when a bus does not pass. `--show` and `--verify-only` change nothing; `--recover` forces the reload chain |
| `scripts/prepare_can_interfaces.py` | role-based CAN or CAN FD preparation for USB or fallback adapter setups |
| `scripts/colcon_build_system_python.sh` | keeps builds on system Python and filters stale or conflicting local environment state |
| `scripts/run_in_ros_conda.sh` | runs a ROS command inside the repo-owned Conda runtime after sourcing ROS and local overlays |
| `scripts/setup_agx_arm_runtime_env.sh` | creates or updates the repo-owned Conda runtime environment |

For usage details and stable command lines, go back to `../../control/bringups/launches.md` or
`../../control/environment.md`.