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
- Planning groups: `nero_arm`, `gripper`, `hand`
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

No end effector:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```

With AgileX gripper:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

With Revo2 hand:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left
```

With OmniHand:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=omnihand omnihand_type:=left
```

Load the repo-owned simple obstacle baseline:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero load_simple_obstacles:=true
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

OmniHand is not wired into the real `agx_arm_ctrl` hardware bringup path yet. At this stage it is available only through the simulation and visualization surfaces in `agx_arm_moveit` and `display_control.launch.py`.

Recommended split-launch flow if you want to keep the MIT soft-trajectory path while bringing components up separately:

```bash
# Terminal 1
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  publish_gripper_joint:=false

# Terminal 2
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=true \
  use_mit_controller:=true \
  load_simple_obstacles:=true
```

When `use_mit_controller:=true`, `demo.launch.py` no longer starts the legacy bridge path. It expects the MIT controller to already provide `arm_controller/follow_joint_trajectory`.

### 2.3 Launch parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `arm_type` | `nero` | Arm model | `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2`, `omnihand` |
| `revo2_type` | `left` | Revo2 hand side | `left`, `right` |
| `omnihand_type` | `left` | OmniHand side | `left`, `right` |
| `namespace` | empty string | Namespace for the MoveIt/control instance | Any valid ROS namespace |
| `follow` | `false` | `true` subscribes to `/feedback/joint_states` and is recommended for real-arm / MIT flows; `false` subscribes to `/control/joint_states` | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP offset [x, y, z, rx, ry, rz] in meters/radians | - |
| `use_mit_controller` | `false` | When `true`, skip fake `ros2_control`, load `moveit_controllers_mit.yaml`, and expect `mit_controller` to provide `arm_controller/follow_joint_trajectory` | `true`, `false` |
| `use_rviz` | `true` | Launch RViz | `true`, `false` |
| `db` | `false` | Launch MoveIt warehouse database | `true`, `false` |
| `load_simple_obstacles` | `false` | Load the repo-owned baseline obstacle set into the planning scene | `true`, `false` |
| `simple_obstacles_config` | `config/simple_obstacles.json` | Path to the planning-scene obstacle JSON file | Any readable JSON path |

### 2.4 Current constraints

- The active launch surface no longer exposes Piper-family options.
- `namespace` still works for multi-instance isolation, but each instance is expected to use the Nero asset tree.
- `publish_gripper_joint` is handled automatically in the combined bringup path to avoid invalid-joint warnings.
- `start_single_agx_arm_moveit.launch.py` and `start_single_agx_arm_rviz.launch.py` now default to the MIT soft-trajectory route. `demo.launch.py` still keeps `use_mit_controller:=false` as the simulation-first default.
- `start_agx_arm_components.launch.py` provides the new common agx_arm_ctrl bringup surface with `manual_vendor`, `debug_soft_target`, and `moveit_mit` modes.
- The current MoveIt baseline expects TRAC-IK. If the distro package is unavailable on Humble / Jetson, use the documented source-build overlay and source `/opt/ros/$ROS_DISTRO/setup.bash`, `~/workspace/trac_ik_ws/install/setup.bash`, then this workspace's `install/setup.bash`.
- `nero_tool0` now comes from the canonical Nero description package, while `tcp_link` remains the TCP and interactive planning target frame.
- `config/simple_obstacles.json` is only a conservative baseline for early planning checks. Adjust it to match the real fixture and workspace before executing on hardware.
- Simulation-only MoveIt validation across `none`, `agx_gripper`, `revo2`, and `omnihand` profiles is a valid hardening path before real-arm collision-checked execution.
- OmniHand currently covers only the MoveIt simulation, RViz, SRDF, and fake `ros2_control` path. Real hardware bringup is still open.
- A 2026-05-21 validation pass confirmed six-profile startup readiness with the external TRAC-IK overlay and a successful live `/compute_ik` call on `nero_arm`; the remaining known runtime issue is a Humble/aarch64 `move_group` shutdown crash after SIGINT.

### 2.5 RViz operations

![piper_moveit](./assets/pictures/piper_moveit.png)

- Drag the interactive marker at the arm tip to define a target pose.
- Use the MotionPlanning panel to switch between `nero_arm`, `gripper`, and `hand`.
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
