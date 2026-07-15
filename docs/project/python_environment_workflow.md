# Python Environment Workflow

status: ACTIVE_DUO_BASELINE
last_updated: 2026-07-15

## Purpose

This repo supports two explicit Python paths:

1. system Python for `colcon build`, `colcon test`, and generated ROS overlays
2. an optional Conda environment for runtime tooling, launch debugging, and Python-side development dependencies such as `python-can`, `PyYAML`, `scipy`, `pinocchio`, and `pyAgxArm`

Do not mix these paths in the same shell.

## System Build Path

Install the system and ROS dependencies first:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash ./agx_arm_install_deps.sh
```

Build the workspace with the repo wrapper so Conda and Miniforge interpreters are filtered out before `colcon build` starts:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl
```

You can pass any normal `colcon build` arguments through this wrapper.

If you rely on a separate TRAC-IK overlay, set `AGX_ARM_TRAC_IK_OVERLAY` to that overlay's `setup.bash` before running the build wrapper.

### Clean rebuild (`rm -rf build/ install/ log/`)

A full clean rebuild is just the wrapper without `--packages-select`:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
rm -rf build/ install/ log/
bash ./scripts/colcon_build_system_python.sh
```

Two things the wrapper handles for you here (both bit a real clean rebuild on 2026-07-15):

1. **Stale prefix paths.** After `rm -rf install/`, a shell that previously sourced
   `install/setup.bash` still carries the deleted paths in `AMENT_PREFIX_PATH` /
   `CMAKE_PREFIX_PATH` / `COLCON_PREFIX_PATH`, and colcon prints a
   `prefix_path ... doesn't exist` warning per package. The wrapper now filters
   non-existent entries **under this workspace's `install/`** out of those variables
   before building (external overlays are untouched). The warnings were cosmetic —
   a fresh shell avoids them too — but the filter removes the noise and any
   stale-overlay ambiguity.
2. **The OmniHand Pro vendor SDK is not a workspace package.** Without
   `--packages-select`, colcon discovers the vendor CMake project
   `omni_hand_pro_2025` and tries to build it. Its CMake requires the pip
   `build` module, which typically lives in the user site
   (`~/.local/lib/python3.10`) — exactly what this wrapper hides via
   `PYTHONNOUSERSITE=1`, so the package fails (`No module named build`) and
   aborts dependents. Per repo policy the SDK is **upstream input**: build it
   with its own `vendor/OmniHand-Pro-2025/build.sh`, and the bridge consumes
   `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg` via runtime auto-discovery
   (the workspace `rm -rf build/` does not touch it). The wrapper therefore
   skips `omni_hand_pro_2025` by default; naming it explicitly in the wrapper
   arguments disables the skip.

## Optional Conda Runtime Path

Create or update the runtime environment from the repo-owned environment file:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/setup_agx_arm_runtime_env.sh
```

By default this creates or updates `agx-arm-runtime`. The script installs the vendored `vendor/pyAgxArm` checkout first and only falls back to a sibling `../pyAgxArm` checkout when the vendored copy is unavailable.

Run ROS commands inside that environment with the repo wrapper instead of activating Conda manually:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_hand
```

The runtime wrapper sources:

1. `/opt/ros/$ROS_DISTRO/setup.bash`
2. the optional `AGX_ARM_TRAC_IK_OVERLAY`
3. the local workspace `install/setup.bash` when it exists

and only then executes the requested command through `conda run`.

## Testing Guidance

- prefer `scripts/colcon_build_system_python.sh --packages-select <pkg>` for package builds
- prefer `/usr/bin/python3 -m pytest ...` when the test is about source-only Python helpers in the ROS build path
- use `scripts/run_in_ros_conda.sh -- python3 -m pytest ...` when the test depends on Conda-managed runtime libraries such as `pinocchio`

## Common Failure Pattern

If ROS imports such as `xacro` or `yaml` disappear after activating Conda, the problem is usually one of these:

1. `PYTHONPATH` was replaced instead of appended by sourced overlays
2. the Conda environment was created with a Python version that does not match the ROS distro
3. the command was run in an unsourced Conda shell instead of through `scripts/run_in_ros_conda.sh`