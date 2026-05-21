# agx_arm_moveit

[English](./README_EN.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> 注：当前工作区的活动 MoveIt 配置已收敛到 Nero 资产树，`arm_type` 仅保留 `nero`。

## 概述

`agx_arm_moveit` 是当前工作区的 Nero 专用 MoveIt2 配置包。

当前支持范围：

- 臂型：`nero`
- 末端执行器：`none`、`agx_gripper`、`revo2`、`omnihand`
- 规划组：`nero_arm`、`gripper`、`hand`
- 运动学插件：TRAC-IK（`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`）

## 1 安装

### 1.1 安装 MoveIt2

```bash
sudo apt install ros-$ROS_DISTRO-moveit*
```

### 1.2 安装额外依赖

```bash
sudo apt-get install -y \
    ros-$ROS_DISTRO-control* \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-* \
    ros-$ROS_DISTRO-gripper-controllers \
  ros-$ROS_DISTRO-trajectory-msgs
```

若 apt 元数据里存在 `ros-$ROS_DISTRO-trac-ik-kinematics-plugin`，请额外安装：

```bash
sudo apt-get install -y ros-$ROS_DISTRO-trac-ik-kinematics-plugin
```

若 Humble / Jetson 主机没有该 apt 包，请参考英文复现实录 `../../docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md` 中的独立 overlay 构建方法。

若系统区域设置不是英文，启动前请设置：

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 2 使用方法

### 2.1 仿真演示

```bash
cd ~/agx_arm_ws
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/workspace/trac_ik_ws/install/setup.bash ]; then source ~/workspace/trac_ik_ws/install/setup.bash; fi
source install/setup.bash
```

若使用发行版提供的 TRAC-IK apt 包，上述条件判断会直接跳过 overlay source。

无末端执行器：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```

带 AgileX 夹爪：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

带 Revo2 灵巧手：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left
```

带 OmniHand 灵巧手：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=omnihand omnihand_type:=left
```

### 2.2 控制真实机械臂

一键启动控制节点、MoveIt 和 RViz：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=agx_gripper
```

带 Revo2 的一键启动示例：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=revo2 \
  revo2_type:=left
```

当前 OmniHand 还没有接入 `agx_arm_ctrl` 的真实硬件启动路径。现阶段仅支持 `agx_arm_moveit` / `display_control.launch.py` 下的仿真与可视化集成，不应把 `effector_type:=omnihand` 误解为真机后端已经打通。

分步启动时，推荐使用 `follow:=true` 让 MoveIt 跟随真实反馈：

```bash
# 终端 1
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=agx_gripper

# 终端 2
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=true
```

### 2.3 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `arm_type` | `nero` | 机械臂型号 | `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2`, `omnihand` |
| `revo2_type` | `left` | Revo2 灵巧手类型 | `left`, `right` |
| `omnihand_type` | `left` | OmniHand 左右手类型 | `left`, `right` |
| `namespace` | 空字符串 | 当前 MoveIt/控制实例命名空间 | 任意合法 ROS 命名空间 |
| `follow` | `false` | `true` 时订阅 `/feedback/joint_states`，`false` 时订阅 `/control/joint_states` | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP 偏移 [x, y, z, rx, ry, rz]（米/弧度） | - |
| `use_rviz` | `true` | 是否启动 RViz | `true`, `false` |
| `db` | `false` | 是否启动 MoveIt warehouse 数据库 | `true`, `false` |

### 2.4 当前约束

- 当前活动配置只覆盖 Nero，不再暴露 Piper 系列启动选项。
- `namespace` 仍可用于多实例隔离，但多个实例都应基于 Nero 资产树。
- `publish_gripper_joint` 会在一键启动路径中自动处理，以避免 MoveIt 中出现无效关节告警。
- 当前 MoveIt 基线要求 TRAC-IK；若 Humble / Jetson 主机没有可用的 apt 包，请参考英文复现实录 `../../docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md` 中的独立 overlay 构建方法。
- `nero_tool0` 现在由 Nero 规范描述包直接提供，`tcp_link` 继续作为 TCP 与交互式规划目标参考帧。
- 对 `none`、`agx_gripper`、`revo2`、`omnihand` 各配置做纯仿真 MoveIt 集成验证，是进入真机碰撞检查执行前的有效路径。
- OmniHand 当前只覆盖 MoveIt 仿真、RViz、SRDF 和 fake `ros2_control` 路径，尚未接入真实硬件控制启动链路。

### 2.5 RViz 操作

![piper_moveit](./assets/pictures/piper_moveit.png)

- 拖动机械臂末端的交互标记来设定目标位姿。
- 在左侧 MotionPlanning 面板中切换 `nero_arm`、`gripper`、`hand` 规划组。
- 在 Goal State 中选择 `home`、`gripper_open`、`hand_half_close`、`hand_close` 等预设状态。

## 3 常见问题

### 3.1 启动时报 `double`/`string` 匹配错误

通常是区域设置导致的小数解析问题。执行：

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

或者在单次启动前显式设置：

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```
