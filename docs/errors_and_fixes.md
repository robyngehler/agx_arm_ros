# Global Errors And Fixes

Cross-cutting issues that already produced confusion or wasted debugging time.

## Native CAN naming versus legacy public naming

Problem: docs and examples mixed native Jetson side-bus names with USB role names.

Current fix:

- native Duo bringup uses `scripts/activate_native_can.sh`
- native side buses are `can_nero_right` and `can_nero_left`
- old public runtime names such as `can0` and `can_nero` are deprecated
- the USB `nero` role now also targets `can_nero_right` by default to stay aligned with the current single-arm baseline

See `control/bringup.md` and `CAN_USER_EN.md`.

## pyAgxArm source drift

Problem: docs described `vendor/pyAgxArm` as the runtime pin while the runtime environment script still preferred the sibling checkout.

Current fix:

- `scripts/setup_agx_arm_runtime_env.sh` now installs `vendor/pyAgxArm` first
- it falls back to `../pyAgxArm` only when the vendored checkout is unavailable

See `project/python_environment_workflow.md` and `project/control_layer_and_dependencies.md`.

## Build Python versus runtime Python

Problem: mixing Conda and ROS build shells hides ROS dependencies and creates false failures.

Current fix:

- use `scripts/colcon_build_system_python.sh` for `colcon build` and `colcon test`
- use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands
- append to `PYTHONPATH`; do not replace it

See `project/python_environment_workflow.md`.

## Implicit wrapper defaults

Problem: wrapper examples that omit `execution_profile` fall back to `manual`, which is not the current recommended operational path.

Current fix:

- package README examples now set explicit profiles such as `right_arm`, `right_hand`, or `duo_arm`
- operational launch matrices stay in `control/bringup.md`

See `control/bringup.md`, `src/agx_arm_moveit/README_EN.md`, and `src/agx_arm_mit_controller/README.md`.