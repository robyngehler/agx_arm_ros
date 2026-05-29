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

加载仓库内置的简易障碍物基线：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero load_simple_obstacles:=true
```

该选项会调用 `scripts/apply_simple_obstacles.py`，将 `config/simple_obstacles.json` 中的基础障碍物集合注入 MoveIt 规划场景。若需替换障碍物集合，可传入 `simple_obstacles_config:=/abs/path/to/file.json`。

### 2.2 控制真实机械臂

推荐使用新的公共组件启动面来走原生 MIT 执行路径：

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  load_simple_obstacles:=true
```

兼容旧的一键启动名称：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  load_simple_obstacles:=true
```

该一键启动路径默认 `use_mit_controller:=true`，因此 MoveIt 的 `arm_controller/follow_joint_trajectory` 会直接连接到 `mit_controller` 暴露的集成 action server，由 MIT 控制器负责轨迹采样、容差检查和 `/control/move_mit` 发布，而不是使用 fake `ros2_control` 或独立桥接节点。若需退回旧路径，可显式设置 `use_mit_controller:=false`。

带 Revo2 的一键启动示例：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=revo2 \
  revo2_type:=left
```

当前 OmniHand 还没有接入 `agx_arm_ctrl` 的真实硬件启动路径。现阶段仅支持 `agx_arm_moveit` / `display_control.launch.py` 下的仿真与可视化集成，不应把 `effector_type:=omnihand` 误解为真机后端已经打通。

若采用分步启动并保留 MIT 软轨迹路径，推荐使用以下方式让 MoveIt 跟随真实反馈并通过 MIT 执行轨迹：

```bash
# 终端 1
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
  can_port:=can_nero \
  arm_type:=nero \
  effector_type:=agx_gripper \
  publish_gripper_joint:=false

# 终端 2
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=true \
  use_mit_controller:=true \
  load_simple_obstacles:=true
```

当 `use_mit_controller:=true` 时，`demo.launch.py` 不再启动旧的桥接执行路径，而是要求 MIT 控制器已经提供 `arm_controller/follow_joint_trajectory`。

### 2.3 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `arm_type` | `nero` | 机械臂型号 | `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2`, `omnihand` |
| `revo2_type` | `left` | Revo2 灵巧手类型 | `left`, `right` |
| `omnihand_type` | `left` | OmniHand 左右手类型 | `left`, `right` |
| `namespace` | 空字符串 | 当前 MoveIt/控制实例命名空间 | 任意合法 ROS 命名空间 |
| `follow` | `false` | `true` 时订阅 `/feedback/joint_states`，推荐用于真机 / MIT 路径；`false` 时订阅 `/control/joint_states` | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP 偏移 [x, y, z, rx, ry, rz]（米/弧度） | - |
| `use_mit_controller` | `false` | `true` 时跳过 fake `ros2_control`，加载 `moveit_controllers_mit.yaml`，并要求 `mit_controller` 提供 `arm_controller/follow_joint_trajectory` | `true`, `false` |
| `use_rviz` | `true` | 是否启动 RViz | `true`, `false` |
| `db` | `false` | 是否启动 MoveIt warehouse 数据库 | `true`, `false` |
| `planning_pipelines` | 空字符串 | 可选的逗号分隔规划流水线白名单，会透传到 `move_group.launch.py`；留空时使用包内默认值 | 例如 `ompl`、`ompl,chomp` |
| `load_simple_obstacles` | `false` | 是否加载仓库内置的基础障碍物集合 | `true`, `false` |
| `simple_obstacles_config` | `config/simple_obstacles.json` | 规划场景障碍物 JSON 配置文件路径 | 任意可读 JSON 路径 |

### 2.4 当前约束

- 当前活动配置只覆盖 Nero，不再暴露 Piper 系列启动选项。
- `namespace` 仍可用于多实例隔离，但多个实例都应基于 Nero 资产树。
- `publish_gripper_joint` 会在一键启动路径中自动处理，以避免 MoveIt 中出现无效关节告警。
- `start_single_agx_arm_moveit.launch.py` 与 `start_single_agx_arm_rviz.launch.py` 默认都会走 MIT 软轨迹路径；`demo.launch.py` 仍保留 `use_mit_controller:=false` 作为纯仿真默认值。
- `start_agx_arm_components.launch.py` 提供新的公共 agx_arm_ctrl 组件启动面，包含 `manual_vendor`、`debug_soft_target`、`moveit_mit` 三种模式。
- 当前 MoveIt 基线要求 TRAC-IK；若 Humble / Jetson 主机没有可用的 apt 包，请参考英文复现实录 `../../docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md` 中的独立 overlay 构建方法。
- `nero_tool0` 现在由 Nero 规范描述包直接提供，`tcp_link` 继续作为 TCP 与交互式规划目标参考帧。
- `config/simple_obstacles.json` 只提供早期规划验证的保守基线；进入真机执行前仍应根据现场工装与工作空间自行调整。
- `share/agx_arm_moveit/scripts/plan_pose_smoke_test.py` 提供当前 Sprint 3 使用的仓库内近 home 位姿 OMPL 规划烟雾测试。
- 对 `none`、`agx_gripper`、`revo2`、`omnihand` 各配置做纯仿真 MoveIt 集成验证，是进入真机碰撞检查执行前的有效路径。
- OmniHand 当前只覆盖 MoveIt 仿真、RViz、SRDF 和 fake `ros2_control` 路径，尚未接入真实硬件控制启动链路。
- 2026-05-28 的验证表明，`plan_pose_smoke_test.py` 已能在 `nero_arm` 上成功拿到代表性的 `ompl` 位姿规划结果，但 Humble/aarch64 主机上的 `move_group` 退出崩溃即使在 `planning_pipelines:=ompl` 的精简路径下仍会复现。

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
