# Environment

status: ACTIVE_BASELINE
last_updated: 2026-07-19

## Purpose

This is the operational source of truth for Python environments, ROS overlays, build wrappers, and
test wrappers.

Launch combinations now live in `bringups/launches.md`.

## Environment classes

### Jetson or other `aarch64` ROS plus hardware environment

Use this class when you need live CAN bringup, real arm or OmniHand launches, or vendor SDK probes
against connected devices.

- `sudo` may be required for CAN bringup scripts
- live hardware validation belongs here

### x86 or editor-only environment without ROS or hardware

Use this class for documentation work, code changes, file-level checks, and offline validation.

- do not claim live hardware validation from this environment
- do not treat x86 timing behavior as a substitute for CAN or real-device validation

## Golden rules

- use `scripts/agx_arm_install_deps.sh` for system and ROS dependencies
- use `scripts/colcon_build_system_python.sh` for workspace builds
- use `scripts/setup_agx_arm_runtime_env.sh` to create or update the Conda runtime environment
- use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands
- do not mix manual `conda activate` with `source install/setup.bash` in the same shell flow
- treat `vendor/OmniHand-Pro-2025` as upstream input, not as a normal workspace package for the
  default repo-wide `colcon build`

## System build path

Install dependencies:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash ./agx_arm_install_deps.sh
```

Build with system Python through the repo wrapper:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl
```

Run package tests from a system-Python ROS shell, not from Conda:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl
colcon test --packages-select agx_arm_ctrl
```

Notes:

- if you use a separate TRAC-IK overlay, set `AGX_ARM_TRAC_IK_OVERLAY=/path/to/install/setup.bash`
  before running the wrapper
- after `rm -rf build/ install/ log/`, the wrapper filters stale local prefix entries under this
  workspace's `install/` path before `colcon` starts
- the wrapper skips the vendor OmniHand SDK by default during repo-wide builds unless you select the
  vendor package explicitly

Full clean rebuild:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
rm -rf build/ install/ log/
bash ./scripts/colcon_build_system_python.sh
```

## Optional Conda runtime path

Create or update the runtime environment:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/setup_agx_arm_runtime_env.sh
```

The environment script installs `vendor/pyAgxArm` first and falls back to `../pyAgxArm` only when
the vendored copy is unavailable.

Run ROS commands through the runtime wrapper instead of activating Conda manually:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_hand
```

The runtime wrapper sources, in order:

1. `/opt/ros/$ROS_DISTRO/setup.bash`
2. the optional `AGX_ARM_TRAC_IK_OVERLAY`
3. the local workspace `install/setup.bash` when it exists

and only then executes the requested command through `conda run`.

## Testing guidance

- prefer `scripts/colcon_build_system_python.sh --packages-select <pkg>` for package builds
- prefer `colcon test --packages-select <pkg>` from a system-Python ROS shell for ROS package tests
- prefer `/usr/bin/python3 -m pytest ...` when the test targets source-only Python helpers in the ROS
  build path
- use `scripts/run_in_ros_conda.sh -- python3 -m pytest ...` when the test depends on Conda-managed
  runtime libraries such as `pinocchio`
- call out explicitly when hardware validation could not be run because the active environment is
  editor-only or lacks live devices

## Common failure patterns

If ROS imports such as `xacro` or `yaml` disappear after switching into Conda, the usual causes are:

1. `PYTHONPATH` was replaced instead of appended by sourced overlays
2. the Conda environment Python version does not match the ROS distro
3. the command was run in a manually activated Conda shell instead of through `scripts/run_in_ros_conda.sh`

If a repo-wide build unexpectedly tries to compile `omni_hand_pro_2025`, use the repo wrapper instead
of raw `colcon build`.

## Related operational docs

- `bringups/launches.md`: current launch matrix
- `bringups/teach_and_run.md`: teach, replay, and coordination-facing workflow guidance