# AgileX Robotic Arm ROS2 Driver

[中文](./README.md)

## Repository role

This repository is the current Duo or Nero ROS2 workspace for runtime arm control, MIT execution,
MoveIt planning, OmniHand integration, coordination flows, and the supporting documentation.

The root README is intentionally short. Detailed runtime usage, environment rules, and architecture
now live under `docs/`.

## Documentation entrypoints

- `docs/README.md`: global documentation hub
- `docs/target/README.md`: active documentation target, migration phases, and control-session findings
- `docs/control/environment.md`: system Python, Conda, ROS overlay, build and test wrappers, and platform split
- `docs/control/bringups/launches.md`: canonical launch matrix
- `docs/control/teach_and_run.md`: teach, record, replay, and coordination-facing motion workflow
- `docs/project/architecture.md`: stable component relationships and Mermaid diagrams
- `docs/project/repository_structure.md`: package boundaries, documentation split, and stable ownership
- `docs/checklist.md`: global migration and integration status
- `docs/errors_and_fixes.md`: cross-cutting issues, current mitigations, and validated fixes
- `docs/open_questions.md`: global Human-Agent exchange surface for unresolved design choices

## Core packages

- `src/agx_arm_ctrl`: runtime arm bridge, launch surfaces, and current OmniHand integration point
- `src/agx_arm_mit_controller`: MIT execution, gravity-aware control, and `FollowJointTrajectory`
- `src/agx_arm_moveit`: MoveIt planning baseline and compatibility simulation path
- `src/agx_arm_coordination`: dual-arm and dual-hand task orchestration through the Activity-DAG coordinator
- `src/agx_arm_sim/agx_arm_description`: canonical long-term description assets
- `src/duo_body_description`: current Duo staging description package
- `src/agx_arm_msgs`: repo-owned ROS messages
- `vendor/OmniHand-Pro-2025`: upstream SDK input, not the public ROS contract

## Fastest correct path

1. Clone the repo with submodules: `git clone -b ros2 --recurse-submodules ...`
2. Install system and ROS dependencies: `bash ./scripts/agx_arm_install_deps.sh`
3. Build with the system-Python wrapper: `bash ./scripts/colcon_build_system_python.sh`
4. Create the optional runtime environment when needed: `bash ./scripts/setup_agx_arm_runtime_env.sh`
5. Run ROS commands through the runtime wrapper:
   `bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm`

## Environment split

- use system Python for `colcon build` and `colcon test`
- use Conda only for the optional runtime and Python-side development dependencies, entered through the repo wrapper
- do not manually mix `conda activate` with `source install/setup.bash` in one shell flow
- live CAN, arm, and OmniHand validation belongs on Jetson or another `aarch64` ROS plus hardware environment
- x86 or editor-only environments are for documentation, code changes, and offline validation only

## Runtime notes

- the authoritative bringup and script matrix is `docs/control/bringups/launches.md`
- teach, replay, and shared-bus operating guidance lives in `docs/control/teach_and_run.md`
- the shared arm-plus-hand CAN path still has known hazards; see `docs/errors_and_fixes.md`