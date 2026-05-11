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
- End effectors: `none`, `agx_gripper`, `revo2`
- Planning groups: `arm`, `gripper`, `hand`
- Kinematics plugin: KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`)

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

If your locale is not English, set:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 2. Usage

### 2.1 Simulation demo

```bash
cd ~/agx_arm_ws
source install/setup.bash
```

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

### 2.2 Control the real arm

One-click launch for control, MoveIt, and RViz:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=agx_gripper
```

Revo2 example:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=revo2 \
  revo2_type:=left
```

Recommended split-launch flow with feedback tracking:

```bash
# Terminal 1
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=agx_gripper

# Terminal 2
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=true
```

### 2.3 Launch parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `arm_type` | `nero` | Arm model | `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `revo2_type` | `left` | Revo2 hand side | `left`, `right` |
| `namespace` | empty string | Namespace for the MoveIt/control instance | Any valid ROS namespace |
| `follow` | `false` | `true` subscribes to `/feedback/joint_states`; `false` subscribes to `/control/joint_states` | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP offset [x, y, z, rx, ry, rz] in meters/radians | - |
| `use_rviz` | `true` | Launch RViz | `true`, `false` |
| `db` | `false` | Launch MoveIt warehouse database | `true`, `false` |

### 2.4 Current constraints

- The active launch surface no longer exposes Piper-family options.
- `namespace` still works for multi-instance isolation, but each instance is expected to use the Nero asset tree.
- `publish_gripper_joint` is handled automatically in the combined bringup path to avoid invalid-joint warnings.
- The current workspace still uses KDL rather than TRAC-IK.

### 2.5 RViz operations

![piper_moveit](./assets/pictures/piper_moveit.png)

- Drag the interactive marker at the arm tip to define a target pose.
- Use the MotionPlanning panel to switch between `arm`, `gripper`, and `hand`.
- Pick preset states such as `home`, `gripper_open`, or `hand_close` from Goal State.

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
