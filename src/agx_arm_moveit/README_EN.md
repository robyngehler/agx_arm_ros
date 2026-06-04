# agx_arm_moveit

[中文](./README.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> Note: the active MoveIt surface in this workspace is now intentionally Nero-only. `arm_type` is restricted to `nero`.

## Overview

`agx_arm_moveit` is the MoveIt2 configuration package used by the current Nero workspace.

Current support:

- Arm type: `nero`
- End effectors: `none`, `agx_gripper`, `revo2`, `omnihand`
- Planning groups: `nero_arm` plus end-effector groups on the built-in single-arm model, and `right_arm` or `left_arm` plus the matching hand end-effector groups on the staged Duo custom-model path; `both_arms` remains the arm-only dual-arm slice
- Kinematics plugin: TRAC-IK (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`)

## 1. Installation

### 1.1 Install MoveIt 2

```bash
sudo apt install ros-$ROS_DISTRO-moveit*
```

### 1.2 Install extra dependencies

```bash
sudo apt-get install -y \
    ros-$ROS_DISTRO-control* \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-* \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-trajectory-msgs
```

If `ros-$ROS_DISTRO-trac-ik-kinematics-plugin` is available in your apt metadata, install it as well:

```bash
sudo apt-get install -y ros-$ROS_DISTRO-trac-ik-kinematics-plugin
```

On ROS 2 Humble / Jetson, `ros-$ROS_DISTRO-trac-ik-kinematics-plugin` may be absent from the configured apt metadata. In that case, build TRAC-IK in a separate overlay and source it before this workspace. A reproducible reference is documented in [TRAC-IK Humble / Jetson repro](../../docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md).

If your locale is not English, set:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 2. Usage

### 2.1 Simulation demo

```bash
cd ~/agx_arm_ws
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/workspace/trac_ik_ws/install/setup.bash ]; then source ~/workspace/trac_ik_ws/install/setup.bash; fi
source install/setup.bash
```

If you installed TRAC-IK from a distro package instead of a source overlay, the conditional line simply does nothing.

Canonical package-local MoveIt bringup:

No end effector:

```bash
ros2 launch agx_arm_moveit start_moveit.launch.py arm_type:=nero
```

With AgileX gripper:

```bash
ros2 launch agx_arm_moveit start_moveit.launch.py arm_type:=nero effector_type:=agx_gripper
```

With Revo2 hand:

```bash
ros2 launch agx_arm_moveit start_moveit.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left
```

With OmniHand:

```bash
ros2 launch agx_arm_moveit start_moveit.launch.py arm_type:=nero effector_type:=omnihand omnihand_type:=left
```

Load the repo-owned simple obstacle baseline:

```bash
ros2 launch agx_arm_moveit start_moveit.launch.py arm_type:=nero load_simple_obstacles:=true
```

This calls `scripts/apply_simple_obstacles.py` and seeds the planning scene from `config/simple_obstacles.json`. Use `simple_obstacles_config:=/abs/path/to/file.json` if you want to replace that baseline.

### 2.2 Control the real arm

Recommended common bringup for native MIT execution:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  load_simple_obstacles:=true
```

Canonical combined MoveIt wrapper:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_moveit.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  load_simple_obstacles:=true
```

Compatibility wrapper with the older combined launch name:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  load_simple_obstacles:=true
```

This combined bringup now defaults to `use_mit_controller:=true`, so MoveIt sends `arm_controller/follow_joint_trajectory` goals directly to the integrated action server exposed by `mit_controller`. The fake `ros2_control` executor is skipped and the MIT controller owns trajectory sampling, tolerance checks, and `/control/move_mit` publishing. Set `use_mit_controller:=false` only if you intentionally want the legacy fake-controller path.

Revo2 example:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=revo2 \
  revo2_type:=left
```

The shared config-based wrappers now resolve the first per-arm OmniHand bringup path through `execution_profile:=left_hand|right_hand`. That profile family keeps the underlying MoveIt planning group at `left_arm` or `right_arm`, adds the matching OmniHand end-effector group on the staged Duo custom model, and forwards the bridge-friendly arm, frame, and feedback defaults in one place:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_moveit.launch.py \
  execution_profile:=right_hand \
  can_port:=can_right \
  follow:=true \
  use_rviz:=false
```

This contract is intentionally per-arm. `moveit_profile:=both_arms` and `execution_profile:=duo_arm` still stay arm-only while the shared dual-hand collision and controller-ownership semantics remain open.

Recommended split-launch flow if you want to keep the MIT soft-trajectory path while bringing components up separately:

```bash
# Terminal 1
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  publish_gripper_joint:=false

# Terminal 2
ros2 launch agx_arm_moveit start_moveit.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=true \
  use_mit_controller:=true \
  load_simple_obstacles:=true
```

When `use_mit_controller:=true`, `start_moveit.launch.py` no longer starts the legacy bridge path. It expects the MIT controller to already provide `arm_controller/follow_joint_trajectory`. `demo.launch.py` is now only a compatibility alias pointing at the same implementation.

The first Duo per-arm profile path is now available through `moveit_profile:=right_arm|left_arm`, which derives the prefixed joint names and the staged body-mounted arm chain frames automatically for the current custom-model slice:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  use_rviz:=false \
  follow:=true \
  moveit_profile:=right_arm \
  custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true' \
  planning_pipelines:=ompl
```

This right-arm profile auto-derives the `right_arm_` joint prefix, the `right_arm` planning group, and the `right_arm_base_link` to `right_arm_nero_tool0` arm chain. `left_arm` mirrors the same contract. The lower-level `input_joint_prefix`, `arm_base_frame`, and `arm_tip_frame` overrides still remain available when the staged defaults are not sufficient.

`both_arms` is now available on the canonical `agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit` path when you provide two managed `arm_instances`, and it remains available directly on `agx_arm_moveit start_moveit.launch.py`:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  use_rviz:=false \
  follow:=true \
  moveit_profile:=both_arms \
  custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
  custom_model_xacro_args:='use_left_arm:=true use_left_hand:=false use_right_arm:=true use_right_hand:=false' \
  arm_instances:='[{name: left_arm, namespace: left_arm, can_port: can_left, joint_prefix: left_arm_, feedback_joint_prefix: left_arm_, launch_driver: false}, {name: right_arm, namespace: right_arm, can_port: can_right, joint_prefix: right_arm_, feedback_joint_prefix: right_arm_, launch_driver: false}]' \
  planning_pipelines:=ompl
```

That profile emits `left_arm`, `right_arm`, and the composed `both_arms` group while loading separate IK solvers for the left and right chains. The combined wrapper now also starts one MIT controller per declared arm instance, keeps each arm runtime in its own namespace, and merges the prefixed feedback path back into MoveIt/RViz. `start_moveit.launch.py` is the canonical package-local entrypoint; `demo.launch.py` remains only as the compatibility alias.

The shared dual-arm MIT wrappers now also start `agx_arm_duo_soft_estop`. It exposes `/emergency_stop` for the current central soft-stop path and per-arm `hold_<namespace>` services that fan `cancel_trajectory` plus `hold_current` into each MIT namespace. Future one-arm selective fixation can build on those per-arm hooks without changing the current central stop contract.

### 2.3 Launch parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `arm_type` | `nero` | Arm model | `nero` |
| `moveit_profile` | `nero_arm` | MoveIt planning profile; `right_arm` and `left_arm` auto-derive the Duo custom-model prefix, group, and arm-chain frames, while `both_arms` emits the first composed dual-arm planning group | `nero_arm`, `right_arm`, `left_arm`, `both_arms` |
| `robot_name` | `agx_arm` | Robot name used by the generated SRDF; override this when the custom model uses a different robot name. The combined wrappers auto-resolve `duo_nero_system` when `custom_model` is set and `robot_name` is left empty | Any valid robot name |
| `custom_model` | empty string | Optional custom model path; when set, MoveIt models that xacro/URDF directly instead of the built-in single-arm description | Any readable xacro/URDF |
| `custom_model_xacro_args` | empty string | Extra xacro argument string forwarded when `custom_model` is set | Any valid xacro arg string |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2`, `omnihand` |
| `revo2_type` | `left` | Revo2 hand side | `left`, `right` |
| `omnihand_type` | `left` | OmniHand side | `left`, `right` |
| `namespace` | empty string | Namespace for the MoveIt/control instance | Any valid ROS namespace |
| `input_joint_prefix` | empty string | Joint prefix used by prefixed custom models for the controlled arm joints; `right_arm` and `left_arm` derive this automatically | e.g. `right_arm_` |
| `feedback_joint_prefix` | empty string | Prefix re-applied on merged follow-side feedback; mainly used by the common multi-arm wrappers | e.g. `left_arm_` |
| `arm_instances` | empty string | Optional YAML list describing managed arm runtime instances; required by the combined wrapper for `moveit_profile:=both_arms` | YAML list of arm-instance mappings |
| `arm_base_frame` | empty string | Optional arm base frame override for custom body-mounted models; `right_arm` and `left_arm` derive this automatically | e.g. `right_arm_base_link` |
| `arm_tip_frame` | empty string | Optional arm tip frame override for custom body-mounted models; `right_arm` and `left_arm` derive this automatically | e.g. `right_arm_nero_tool0` |
| `follow` | `false` | `true` subscribes to `/feedback/joint_states` and is recommended for real-arm / MIT flows; `false` subscribes to `/control/joint_states` | `true`, `false` |
| `follow_joint_states_topic` | `feedback/joint_states` | JointState topic consumed when `follow:=true`; point this at an adapted topic for prefixed multi-arm models | Any valid topic |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP offset [x, y, z, rx, ry, rz] in meters/radians | - |
| `use_mit_controller` | `false` | When `true`, skip fake `ros2_control`, load `moveit_controllers_mit.yaml`, and expect `mit_controller` to provide `arm_controller/follow_joint_trajectory` | `true`, `false` |
| `use_rviz` | `true` | Launch RViz | `true`, `false` |
| `db` | `false` | Launch MoveIt warehouse database | `true`, `false` |
| `planning_pipelines` | empty string | Optional comma-separated planning pipeline whitelist forwarded to `move_group.launch.py`; empty uses the package defaults | e.g. `ompl`, `ompl,chomp` |
| `load_simple_obstacles` | `false` | Load the repo-owned baseline obstacle set into the planning scene | `true`, `false` |
| `simple_obstacles_config` | `config/simple_obstacles.json` | Path to the planning-scene obstacle JSON file | Any readable JSON path |

### 2.4 Current constraints

- The active launch surface no longer exposes Piper-family options.
- `namespace` still works for multi-instance isolation, but each instance is expected to use the Nero asset tree.
- `publish_gripper_joint` is handled automatically in the combined bringup path to avoid invalid-joint warnings.
- `start_moveit.launch.py` is now the canonical package-local MoveIt entrypoint. `demo.launch.py` remains as a compatibility alias.
- `start_agx_arm_moveit.launch.py` is now the canonical combined MoveIt wrapper. `start_single_agx_arm_moveit.launch.py` remains as a compatibility alias, while `start_single_agx_arm.launch.py` still accurately names the one-driver-per-arm launch surface.
- `moveit_profile:=right_arm`, `moveit_profile:=left_arm`, and `moveit_profile:=both_arms` are the first landed Duo profiles. On the shared `agx_arm_ctrl` wrappers, `execution_profile:=left_hand|right_hand` now resolves the first hand-aware per-arm config path, while `both_arms` and `execution_profile:=duo_arm` intentionally remain arm-only.
- `start_agx_arm_components.launch.py` provides the new common agx_arm_ctrl bringup surface with `manual_vendor`, `debug_soft_target`, and `moveit_mit` modes.
- The current MoveIt baseline expects TRAC-IK. If the distro package is unavailable on Humble / Jetson, use the documented source-build overlay and source `/opt/ros/$ROS_DISTRO/setup.bash`, `~/workspace/trac_ik_ws/install/setup.bash`, then this workspace's `install/setup.bash`.
- `nero_tool0` now comes from the canonical Nero description package, while `tcp_link` remains the TCP and interactive planning target frame.
- `config/simple_obstacles.json` is only a conservative baseline for early planning checks. Adjust it to match the real fixture and workspace before executing on hardware.
- `share/agx_arm_moveit/scripts/plan_pose_smoke_test.py` provides the current repo-owned representative near-home OMPL pose-planning check for Sprint 3.
- Simulation-only MoveIt validation across `none`, `agx_gripper`, `revo2`, and `omnihand` profiles is a valid hardening path before real-arm collision-checked execution.
- The staged Duo gravity slice now keeps an active OmniHand as a fixed-pose payload by freezing its joints at zero pose in the derived URDF. The MIT controller still owns only the seven Nero arm joints; dynamic hand-pose compensation remains future work.
- Per-arm OmniHand now covers the staged Duo MoveIt, RViz, generated SRDF, config-based `agx_arm_ctrl` wrappers, and the MIT gravity payload slice. Live dual-hand execution and a hand-aware `both_arms` contract remain open.
- A 2026-05-21 validation pass confirmed six-profile startup readiness with the external TRAC-IK overlay and a successful live `/compute_ik` call on `nero_arm`.
- A 2026-05-28 validation pass confirmed that `plan_pose_smoke_test.py` can obtain a representative `ompl` pose plan for `nero_arm`, but the Humble/aarch64 `move_group` shutdown crash still reproduces even when the launch is reduced to `planning_pipelines:=ompl`.

### 2.5 RViz operations

![piper_moveit](./assets/pictures/piper_moveit.png)

- Drag the interactive marker at the arm tip to define a target pose.
- Use the MotionPlanning panel to switch between `nero_arm`, `gripper`, and `hand` on the built-in single-arm model, or `right_arm`, `left_arm`, and `both_arms` on the staged Duo custom-model path. The wrapper-level `left_hand` and `right_hand` execution profiles reuse the matching per-arm planning group while adding the staged OmniHand end-effector group.
- Pick preset states such as `home`, `gripper_open`, `hand_half_close`, or `hand_close` from Goal State.

## 3. Troubleshooting

### 3.1 `double` vs `string` parameter parsing errors

This is typically caused by locale settings. Either configure the shell permanently:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

Or prefix a single command:

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```
