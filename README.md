# AgileX 机械臂 ROS2 驱动

[English](./README_EN.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 概述

当前工作区以 Nero 为主线，提供对应的 ROS2 控制、描述、MoveIt2 与 MIT 软控制接口。

|说明 |文档|
|---|---|
|SDK|[pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)|
|官方CAN模块的使用|[can_user](./docs/CAN_USER.md)|
|TCP偏移设置|[tcp_offset](./docs/tcp_offset/TCP_OFFSET.md)|
|URDF|[agx_arm_description](./src/agx_arm_sim/agx_arm_description/README.md)|
|Moveit| [Moveit](./src/agx_arm_moveit/README.md) |
|Nero MIT 软控制|[agx_arm_mit_controller](./src/agx_arm_mit_controller/README.md)|
|Q&A|[Q&A](./docs/Q&A.md)|

---

## 快速开始

### 1. 安装 Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
```

根据你的 ROS 版本选择安装命令：

**Jazzy** 安装命令：

```bash
pip3 install . --break-system-packages
```

**Humble** 安装命令：

```bash
pip3 install .
```

### 2. 安装 ROS2 驱动

1. 创建工作空间

    ```bash
    mkdir -p ~/agx_arm_ws/src
    cd ~/agx_arm_ws/src
    ```

2. 克隆仓库

    ```bash
    git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
    ```

    ```bash
    cd agx_arm_ros/
    git submodule update --remote --recursive
    ```

    > 当前仓库仅保留 `vendor/Omnihand-2025-SDK` 作为子模块；Nero/Revo2 描述资产已经直接提交在仓库内。

### 3. 安装依赖

运行脚本一键安装所有依赖

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts/
chmod +x agx_arm_install_deps.sh
bash ./agx_arm_install_deps.sh
```

或者依次执行以下命令手动安装：

1. Python 依赖

    根据你的 ROS 版本选择安装命令：

    **Jazzy** 安装命令:

    ```bash
    pip3 install python-can scipy numpy --break-system-packages
    ```

    **Humble** 安装命令:

    ```bash
    pip3 install python-can scipy numpy
    ```

2. CAN 工具

    ```bash
    sudo apt update && sudo apt install can-utils ethtool
    ```

3. ROS2 依赖

    ```bash
    sudo apt install -y \
        ros-$ROS_DISTRO-ros2-control \
        ros-$ROS_DISTRO-ros2-controllers \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-topic-tools \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-xacro \
        python3-colcon-common-extensions
    ```

4. Moveit

    使用 MoveIt 前，需先配置相关依赖。 具体步骤请参考：[agx_arm_moveit](./src/agx_arm_moveit/README.md)
    
    或者依次执行以下命令进行配置：

    ```bash
    sudo apt install ros-$ROS_DISTRO-moveit*
    ```

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

    在 ROS 2 Humble / Jetson 环境下，该 TRAC-IK apt 包可能不存在。此时请参考 [TRAC-IK Humble / Jetson 复现实录](./docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md)，在独立 overlay 中源码构建并先于 `agx_arm_ros` source。

    若系统语言区域设置不为英文区域，须设置为英文区域

    ```bash
    echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
    source ~/.bashrc
    ```

### 4.编译并Source工作空间

检查是否在虚拟环境里，如果是，建议先退出虚拟环境。

```bash
which pip3
```

编译工作空间并加载环境配置：

```bash
cd ~/agx_arm_ws
colcon build
source install/setup.bash
```

---

## 使用说明

### 激活 CAN 模块

使用前需先激活 CAN 模块，详见：[CAN 配置指南](./docs/CAN_USER.md)

推荐优先使用仓库内的角色化准备脚本。该脚本会读取 [config/can_interface_roles.json](./config/can_interface_roles.json) 中的角色定义，并统一处理 Nero、末端执行器与 OmniHand 的 SocketCAN 命名、波特率和 CAN FD 参数：

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
python3 scripts/prepare_can_interfaces.py --list
python3 scripts/prepare_can_interfaces.py --roles nero --dry-run
python3 scripts/prepare_can_interfaces.py --roles nero
```

若需要显式绑定某个 USB 口或 Linux 接口，可额外传入 `--nero-can-interface 3-1.4:1.0` 或 `--nero-can-interface can0`。当同时准备机械臂和 OmniHand 时，可使用 `--roles nero,omnihand`。兼容旧流程的 `can_activate.sh` / `can_muti_activate.sh` 仍保留在 [CAN 配置指南](./docs/CAN_USER.md) 中。

### 启动驱动

您可以通过 launch 文件或直接运行节点来启动驱动。

> **重要提示：启动前必读**
> 以下启动命令中的参数**必须**根据您的实际硬件配置进行替换：
> - **`can_port`**：机械臂连接的 CAN 端口，示例值 `can0`；若按本文推荐流程使用 `prepare_can_interfaces.py`，Nero 默认角色名为 `can_nero`。
> - **`arm_type`**：机械臂的型号，当前工作区示例值 `nero`。
> - **`effector_type`**：末端执行器类型，示例值 `none` 或 `agx_gripper`。
> - **`tcp_offset`**：工具中心（TCP）相对法兰盘中心的偏移量，示例值：[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
>   - 注意 ：`tcp_offset` 所有值均需为浮点数；关于 TCP 偏移实际配置示例，请参考 [TCP 设置详解](./docs/tcp_offset/TCP_OFFSET.md)。
>
> 所有参数的完整说明、默认值及可选值，请参阅下方的 **[启动参数](#启动参数)** 。


**使用 launch 文件启动：**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=nero effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**直接运行节点启动:**

```bash
ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args -p can_port:=can0 -p arm_type:=nero -p effector_type:=none -p tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**可视化调试启动:**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

> **注意：**
> - `start_single_agx_arm_rviz.launch.py` 默认 `use_mit_controller:=true`，因此 RViz 控制路径会优先走 MIT 软轨迹，而不是直接写 `/control/joint_states`。
> - 当 `control:=true` 且 `use_mit_controller:=true` 时，RViz 关节滑条会发布到 `mit_controller/soft_target_joint_states`，再由 `agx_arm_mit_joint_state_bridge` 转成发往 `mit_controller` 的调试短段 `trajectory_msgs/JointTrajectory`。
> - `mit_joint_target_duration_s` 用于调节每个滑条目标的软轨迹时长；若要退回原始 `/control/joint_states` 路径，可显式设置 `use_mit_controller:=false`。
> - `follow:=true` 时，模型会订阅真实反馈并驱动显示；若只做状态跟随展示，推荐保持 `control:=false`。
> - MIT 调试 topic 仅在 `control:=true` 时开启，且滑条桥接节点不再自动使能 MIT；在拖动滑条前请先显式使能 `mit_controller`。

**MoveIt 一键启动（臂控 + MoveIt + RViz）：**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can_nero \
    arm_type:=nero \
    effector_type:=agx_gripper \
    load_simple_obstacles:=true
```

该启动方式默认 `use_mit_controller:=true`：会先启动 `start_nero_mit_controller.launch.py`，再让 MoveIt 通过 `arm_controller/follow_joint_trajectory` 直接调用 `mit_controller` 上的集成 MIT action server。`load_simple_obstacles:=true` 会通过 `src/agx_arm_moveit/scripts/apply_simple_obstacles.py` 将 [simple_obstacles.json](./src/agx_arm_moveit/config/simple_obstacles.json) 中的基础障碍物注入规划场景；如需替换障碍物集合，可传入 `simple_obstacles_config:=/abs/path/to/file.json`。若要退回旧的直接执行路径，可显式设置 `use_mit_controller:=false`。

MIT 侧控制频率与参数文件可继续通过 `mit_control_rate_hz` 和 `mit_params_file` 调整。

**Nero MIT 软轨迹控制（ROS 应用节点 + 臂控节点）：**

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
    can_port:=can_nero \
    arm_type:=nero \
    effector_type:=agx_gripper \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

该启动方式会复用 `agx_arm_ctrl` 作为硬件适配层，并额外启动一个面向应用的 MIT 控制节点。它现在除了 `can_port` 外，也直接暴露 `arm_type`、`effector_type`、`omnihand_type`、`launch_omnihand_bridge`、`omnihand_backend_type`、`auto_enable`、`fast_mode`、`speed_percent`、`pub_rate`、`enable_timeout`、`tcp_offset`、`gripper_default_effort`、`publish_gripper_joint` 等底层运行参数，同时额外提供 `control_rate_hz`、`params_file`、`log_level`，以及用于 RViz 软目标调试的显式 `enable_debug_joint_trajectory_topic` 开关。

该节点订阅 `feedback/joint_states`，对外提供 `arm_controller/follow_joint_trajectory` 给 MoveIt 使用，并持续发布 `control/move_mit`。仅当显式开启调试输入时，它才接收调试用的 `~/joint_trajectory` topic，便于后续扩展软轨迹回放、重力补偿和碰撞监测。

> 若需要同时启动机械臂控制节点、MoveIt2 和 RViz，请使用上面的 `start_single_agx_arm_moveit.launch.py`；它会自动接入关节反馈，并默认复用 MIT 软轨迹执行路径。详见 [Moveit](./src/agx_arm_moveit/README.md)。

### 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `can_port` | `can0` | CAN 端口 | - |
| `arm_type` | `nero` | 机械臂型号 | `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2` |
| `namespace` | 空字符串 | 机械臂实例命名空间 | 任意合法 ROS 命名空间 |
| `auto_enable` | `true` | 启动时自动使能 | `true`, `false` |
| `fast_mode` | `false` | 启用快速模式（如果启用，`/control/joint_states` 内部将改用无平滑无插值的 `move_js` 关节控制接口控制机械臂） | `true`, `false` |
| `speed_percent` | `100` | 运动速度 (%) | `0-100` |
| `pub_rate` | `200` | 状态发布频率 (Hz) | - |
| `enable_timeout` | `5.0` | 使能超时 (秒) | - |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | 工具中心(TCP)相对法兰盘中心的偏移 [x, y, z, rx, ry, rz] | - |
| `gripper_default_effort` | `1.0` | 夹爪默认力（单位：N） | `>=0.0` |
| `publish_gripper_joint` | `true` | 是否在 `/feedback/joint_states` 中发布 `gripper` 关节（夹爪开口宽度）。与 MoveIt 联用时设为 `false`，因 URDF 中仅有 `gripper_joint1`/`gripper_joint2` | `true`, `false` |
| `log_level` | `info` | 日志级别 | `debug`, `info`, `warn`, `error`, `fatal` |

### URDF 模型可视化

#### 独立查看模型

在 RViz 中加载 URDF 模型并通过 GUI 滑条手动调试关节，可以不启动机械臂节点：

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=nero
```

**支持以下三种方式指定模型：**

1. **预设型号名称（通过 `arm_type` 指定）**（推荐）：直接使用内置型号名，自动匹配对应 URDF 文件

    ```bash
    ros2 launch agx_arm_description display.launch.py arm_type:=nero
    ```

2. **相对路径（通过 `custom_model` 指定）**：相对于 `agx_arm_urdf/` 目录的路径，适用于自定义模型  

    ```bash
    ros2 launch agx_arm_description display_control.launch.py custom_model:=nero/urdf/nero_description.urdf
    ```

3. **绝对路径（通过 `custom_model` 指定）**：直接指定 URDF 文件的绝对路径，适用于任意位置的模型文件

    ```bash
    ros2 launch agx_arm_description display_control.launch.py custom_model:=~/agx_arm_ws/src/agx_arm_ros/src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf
    ```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_type` | `nero` | 机械臂型号，预设值：`nero` |
| `custom_model` | 空字符串 | 可选自定义模型路径；相对路径时相对于 `agx_arm_urdf/` 目录，绝对路径可指向任意 URDF/xacro 文件。若设置该参数，则 `arm_type` 和 `effector_type` 将被忽略 |
| `effector_type` | `none` | 末端执行器类型，预设值：`none`, `agx_gripper`, `revo2` |
| `revo2_type` | `left` | Revo2 灵巧手类型，预设值：`left`, `right` |
| `pub_rate` | `200` | 状态发布频率 (Hz) |
| `gui` | `true` | 是否启用 joint_state_publisher_gui 关节滑条控制界面 |
| `rvizconfig` | 内置配置 | 自定义 RViz 配置文件的绝对路径 |
| `follow` | `false` | 是否跟随真实机械臂状态（订阅 `/feedback/joint_states`，并在 robot_state_publisher 中重映射 `/joint_states` 为 `feedback/joint_states`） |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP 偏移 [x, y, z, rx, ry, rz]（米/弧度）。非零时自动发布 `tcp_link` 坐标系 |
| `control` | `true` | 是否通过 joint_state_publisher（或 GUI 版本）发布控制话题：`true` 时发布到 `control_topic`；`false` 时仅用于跟随或显示，不发布控制话题。与 `follow:=true` 配合时，常用组合为 `follow:=true, control:=false`（只跟随真实机械臂，不从 RViz 发出控制） |
| `control_topic` | `/control/joint_states` | RViz 关节滑条输出（`joint_state_publisher_gui`）发布到的目标话题 |

#### 典型应用组合示例（follow / control）

下面给出几种**常见使用场景**下，`follow` 与 `control` 的推荐组合及对应示例指令：

- **场景 1：纯模型调试（无真机，仅看 URDF、用滑条拖动）**  
  - 是否需要真机：否  
  - 推荐配置：`follow:=false, control:=true`  
  - 示例：  
    ```bash
    ros2 launch agx_arm_description display_control.launch.py arm_type:=nero follow:=false control:=true
    ```

> 说明：若希望把 RViz 滑动条发布的关节目标重定向给阻抗控制器（例如 `agx_arm_impedance` 的关节阻抗 `control_type:=joint_impedance`），可在启动 `display.launch.py` 时设置 `control:=true` 且 `control_topic:=/impedance/target_joint`。

- **场景 2：真机 + 仅控制不跟随**（从 RViz 发控制，但 RViz 不显示真实反馈）  
  - 是否需要真机：是  
  - 推荐配置：`follow:=false, control:=true`  
  - 示例：  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=false control:=true
    ```

- **场景 3：真机 + 仅跟随不控制**（常见：只看状态，不希望 RViz 干扰控制）  
  - 是否需要真机：是  
  - 推荐配置：`follow:=true, control:=false`  
  - 示例：  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=true control:=false
    ```

- **场景 4：真机 + 控制 + 跟随**（从 RViz 发控制，并在 RViz 跟随真实反馈）  
  - 是否需要真机：是  
  - 推荐配置：`follow:=true, control:=true`
  - 示例：  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=true control:=true
    ```

> **提示：** 一般情况下，建议**控制通道保持唯一**，即只保留一个组件负责发布 `/control/*` 话题（如 `agx_arm_ctrl` 节点、MoveIt、或 RViz 中的 joint_state_publisher 三者选其一），以避免多源控制导致冲突。

---

## 控制示例

额外启动一个终端,运行以下指令:

```bash
cd ~/agx_arm_ws
source install/setup.bash
cd src/agx_arm_ros
```

### Nero 机械臂

1. 关节运动

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/nero/test_move_j.yaml)" -1
    ```

2. 点到点运动

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_p.yaml)" -1
    ```

3. 直线运动

    ```bash
    ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_l.yaml)" -1
    ```

4. 圆弧运动（起点 → 中间点 → 终点）

    ```bash
    ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
      "$(cat test/nero/test_move_c.yaml)" -1
    ```

### Gripper 夹爪

1. 夹爪控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/gripper/test_gripper_joint_states.yaml)" -1
    ```

> **注意：** 以上夹爪控制指令，需在 launch 文件或参数中设置 `effector_type=agx_gripper`。

### Hand 灵巧手

1. 灵巧手 — 位置模式（所有手指移动到 10）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_position.yaml)" -1
    ```

2. 灵巧手 — 速度模式（所有手指速度 50）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_speed.yaml)" -1
    ```

3. 灵巧手 — 电流模式（所有手指电流 50）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_current.yaml)" -1
    ```

4. 灵巧手 — 位置-时间控制（所有手指移动到 50，时间 1 秒）

    ```bash
    ros2 topic pub /control/hand_position_time agx_arm_msgs/msg/HandPositionTimeCmd \
      "$(cat test/hand/test_hand_position_time.yaml)" -1
    ```

5. 灵巧手控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/hand/test_hand_joint_states.yaml)" -1
    ```

> **注意：** 以上灵巧手控制指令，需在 launch 文件或参数中设置 `effector_type=revo2`。

### 服务调用

1. 使能机械臂

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
    ```

2. 失能机械臂

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
    ```

3. 回零位

    ```bash
    ros2 service call /move_home std_srvs/srv/Empty
    ```

4. 急停（保持当前位置）

    ```bash
    ros2 service call /emergency_stop std_srvs/srv/Empty
    ```

### 状态订阅

1. 关节状态

    ```bash
    ros2 topic echo /feedback/joint_states
    ```

2. TCP 位姿

    ```bash
    ros2 topic echo /feedback/tcp_pose
    ```

3. 机械臂状态

    ```bash
    ros2 topic echo /feedback/arm_status
    ```

4. 主导臂关节角度(主导臂模式下使用)

    ```bash
    ros2 topic echo /feedback/leader_joint_angles
    ```

5. 夹爪状态

    ```bash
    ros2 topic echo /feedback/gripper_status
    ```

6. 灵巧手状态

    ```bash
    ros2 topic echo /feedback/hand_status
    ```

---

## ROS2 接口

### 反馈话题

| 话题 | 消息类型 | 说明 | 适用条件 |
|------|----------|------|----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | 关节状态 | 始终可用 |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 位姿 | 始终可用 |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | 机械臂状态 | 始终可用 |
| `/feedback/leader_joint_angles` | `sensor_msgs/JointState` | 主导臂关节角度 | 主导臂模式 |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | 夹爪状态 | 配置 AgxGripper |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | 灵巧手状态 | 配置 Revo2 |

#### `/feedback/joint_states` 详细说明

该话题包含机械臂和末端执行器的组合关节状态：

**机械臂关节** (`joint1` ~ `joint*`)

| 字段 | 说明 |
|------|------|
| `position` | 关节角度 (rad) |
| `velocity` | 关节速度 (rad/s) |
| `effort` | 关节力矩 (Nm) |

**夹爪关节** （需配置 `effector_type=agx_gripper`）

默认发布 `gripper`、`gripper_joint1`、`gripper_joint2` 三个关节；若 `publish_gripper_joint` 设为 `false`，仅发布 `gripper_joint1` 和 `gripper_joint2`（URDF 中的关节名，适配 MoveIt）。

| 关节名 | `position` 说明 | `velocity` | `effort` |
|--------|-----------------|------------|----------|
| `gripper` | 夹爪开口宽度 (m)，范围 [0, 0.1] | 0.0 | 力 (N) |
| `gripper_joint1` | 单侧夹片位移 = 宽度 × 0.5 (m) | 0.0 | 力 (N) |
| `gripper_joint2` | 单侧夹片位移 = 宽度 × -0.5 (m) | 0.0 | 力 (N) |

**灵巧手关节**（需配置 `effector_type=revo2`）

左手关节命名：

- `left_thumb_metacarpal_joint`
- `left_thumb_proximal_joint`
- `left_index_proximal_joint`
- `left_middle_proximal_joint`
- `left_ring_proximal_joint`
- `left_pinky_proximal_joint`

右手关节命名：

- `right_thumb_metacarpal_joint`
- `right_thumb_proximal_joint`
- `right_index_proximal_joint`
- `right_middle_proximal_joint`
- `right_ring_proximal_joint`
- `right_pinky_proximal_joint`

| 字段 | 说明 |
|------|------|
| `position` | 手指关节角度 (rad) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

#### `/feedback/arm_status` 详细说明

消息类型：`agx_arm_msgs/AgxArmStatus`

**消息字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `ctrl_mode` | `uint8` | 控制模式，见下表 |
| `arm_status` | `uint8` | 机械臂状态，见下表 |
| `mode_feedback` | `uint8` | 模式反馈，见下表 |
| `teach_status` | `uint8` | 示教状态，见下表 |
| `motion_status` | `uint8` | 运动状态：0=已到达目标位置，1=未到达目标位置 |
| `trajectory_num` | `uint8` | 当前轨迹点序号（0~255，离线轨迹模式下反馈） |
| `err_status` | `int64` | 错误状态码 |
| `joint_1_angle_limit` ~ `joint_7_angle_limit` | `bool` | 关节1~7角度超限（true=异常，false=正常） |
| `communication_status_joint_1` ~ `communication_status_joint_7` | `bool` | 关节1~7通信状态（true=异常，false=正常） |

**控制模式 (`ctrl_mode`)：**

| 值 | 说明 |
|----|------|
| 0 | 待机 |
| 1 | CAN指令控制 |
| 2 | 示教模式 |
| 3 | 以太网控制 |
| 4 | WiFi控制 |
| 5 | 遥控模式 |
| 6 | 联动示教输入 |
| 7 | 离线轨迹模式 |
| 8 | TCP控制 |

**机械臂状态 (`arm_status`)：**

| 值 | 说明 |
|----|------|
| 0 | 正常 |
| 1 | 急停 |
| 2 | 无解 |
| 3 | 奇异点 |
| 4 | 目标角度超限 |
| 5 | 关节通信异常 |
| 6 | 关节刹车未释放 |
| 7 | 发生碰撞 |
| 8 | 示教拖动超速 |
| 9 | 关节状态异常 |
| 10 | 其他异常 |
| 11 | 示教记录中 |
| 12 | 示教执行中 |
| 13 | 示教暂停 |
| 14 | 主控NTC过温 |
| 15 | 释放电阻NTC过温 |

**模式反馈 (`mode_feedback`)：**

| 值 | 说明 |
|----|------|
| 0 | MOVE P |
| 1 | MOVE J |
| 2 | MOVE L |
| 3 | MOVE C |
| 4 | MOVE MIT |
| 5 | MOVE CPV |

**示教状态 (`teach_status`)：**

| 值 | 说明 |
|----|------|
| 0 | 关闭 |
| 1 | 开始示教记录（进入拖动示教） |
| 2 | 结束示教记录（退出拖动示教） |
| 3 | 执行示教轨迹 |
| 4 | 暂停执行 |
| 5 | 继续执行 |
| 6 | 终止执行 |
| 7 | 移动至轨迹起点 |

#### `/feedback/gripper_status` 详细说明

消息类型：`agx_arm_msgs/GripperStatus`

**消息字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 消息头 |
| `width` | `float64` | 当前夹爪开口宽度（单位：米） |
| `force` | `float64` | 当前夹持力（单位：牛顿） |
| `voltage_too_low` | `bool` | 电压过低（true=异常，false=正常） |
| `motor_overheating` | `bool` | 电机过热（true=异常，false=正常） |
| `driver_overcurrent` | `bool` | 驱动器过流（true=异常，false=正常） |
| `driver_overheating` | `bool` | 驱动器过热（true=异常，false=正常） |
| `sensor_status` | `bool` | 传感器状态（true=异常，false=正常） |
| `driver_error_status` | `bool` | 驱动器错误状态（true=异常，false=正常） |
| `driver_enable_status` | `bool` | 驱动器使能状态（true=已使能，false=未使能） |
| `homing_status` | `bool` | 回零/归零状态（true=已完成，false=未完成） |

#### `/feedback/hand_status` 详细说明

消息类型：`agx_arm_msgs/HandStatus`

**消息字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 消息头 |
| `left_or_right` | `uint8` | 手部类型标识：1=左手，2=右手 |

**手指位置字段（范围：[0, 100]，0=完全张开，100=完全弯曲）：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `thumb_tip_pos` | `uint8` | 拇指指尖位置 |
| `thumb_base_pos` | `uint8` | 拇指指根位置 |
| `index_finger_pos` | `uint8` | 食指位置 |
| `middle_finger_pos` | `uint8` | 中指位置 |
| `ring_finger_pos` | `uint8` | 无名指位置 |
| `pinky_finger_pos` | `uint8` | 小指位置 |

**手指电机状态字段（0=空闲，1=运行中，2=堵转/卡死）：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `thumb_tip_status` | `uint8` | 拇指指尖电机状态 |
| `thumb_base_status` | `uint8` | 拇指指根电机状态 |
| `index_finger_status` | `uint8` | 食指电机状态 |
| `middle_finger_status` | `uint8` | 中指电机状态 |
| `ring_finger_status` | `uint8` | 无名指电机状态 |
| `pinky_finger_status` | `uint8` | 小指电机状态 |

### 控制话题

| 话题                            | 消息类型                               | 说明           | 适用条件          |
| ----------------------------- | ---------------------------------- | ------------ | ------------- |
| `/control/joint_states`       | `sensor_msgs/JointState`           | 关节控制（含末端执行器） | 始终可用          |
| `/control/move_j`             | `sensor_msgs/JointState`           | 关节控制运动       | 始终可用          |
| `/control/move_p`             | `geometry_msgs/PoseStamped`        | 点到点运动        | 始终可用          |
| `/control/move_l`             | `geometry_msgs/PoseStamped`        | 直线运动         | 始终可用      |
| `/control/move_c`             | `geometry_msgs/PoseArray`          | 圆弧运动         | 始终可用      |
| `/control/move_js`            | `sensor_msgs/JointState`           | MIT 模式关节运动   | 始终可用      |
| `/control/move_mit`           | `agx_arm_msgs/MoveMITMsg`          | MIT 力矩控制     | 始终可用      |
| `/control/hand`               | `agx_arm_msgs/HandCmd`             | 灵巧手控制        | 配置 Revo2      |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | 灵巧手位置时间控制    | 配置 Revo2      |

#### `/control/joint_states` 详细说明

该话题使用 `sensor_msgs/JointState` 消息类型，支持同时控制机械臂关节和末端执行器（夹爪/灵巧手）。只需发送要控制的关节即可，未包含的关节不受影响。

**消息字段说明：**

| 字段 | 说明 |
|------|------|
| `name` | 关节名称列表 |
| `position` | 对应关节的目标位置 |
| `velocity` | 未使用（可留空） |
| `effort` | 用于夹爪力控制（仅对 `gripper` 关节有效） |

**通过 `/control/joint_states` 控制夹爪**（需配置 `effector_type=agx_gripper`）

在 `name` 中包含 `gripper`，通过 `position` 设置目标宽度，通过 `effort` 设置夹持力。

| 关节名 | position（宽度） | effort（力） |
|--------|-----------------|-------------|
| `gripper` | 目标宽度 (m)，范围: [0.0, 0.1] | 目标力 (N)，范围: [0.5, 3.0]，默认: 1.0 |

> **注意：** 当 `effort` 为 0 或未指定时，使用默认力 1.0N。

示例：控制夹爪宽度 0.05m、力 1.5N
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [gripper], position: [0.05], velocity: [], effort: [1.5]}" -1
```

**通过 `/control/joint_states` 控制灵巧手**（需配置 `effector_type=revo2`）

在 `name` 中包含灵巧手关节名，通过 `position` 设置目标位置（position 模式, 单位: rad）。仅需发送要控制的关节，未包含的关节将保持当前位置。

示例：仅控制左手食指到位置 0.5rad
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [left_index_proximal_joint], position: [0.5], velocity: [], effort: []}" -1
```

| 关节名 | 说明 | position 范围 |
|--------|------|--------------|
| `left_thumb_metacarpal_joint` / `right_thumb_metacarpal_joint` | 大拇指指根 | [0, 1.57] |
| `left_thumb_proximal_joint` / `right_thumb_proximal_joint` | 大拇指指尖 | [0, 1.03] |
| `left_index_proximal_joint` / `right_index_proximal_joint` | 食指 | [0, 1.41] |
| `left_middle_proximal_joint` / `right_middle_proximal_joint` | 中指 | [0, 1.41] |
| `left_ring_proximal_joint` / `right_ring_proximal_joint` | 无名指 | [0, 1.41] |
| `left_pinky_proximal_joint` / `right_pinky_proximal_joint` | 小指 | [0, 1.41] |

#### `/control/move_mit` 详细说明

消息类型：`agx_arm_msgs/MoveMITMsg`

**消息字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `joint_index` | `int32[]` | 要控制的关节索引数组 |
| `p_des` | `float64[]` | 期望关节位置数组（单位：弧度） |
| `v_des` | `float64[]` | 期望关节速度数组（单位：弧度/秒） |
| `kp` | `float64[]` | 位置增益数组 |
| `kd` | `float64[]` | 速度增益数组 |
| `torque` | `float64[]` | 期望关节力矩数组（单位：牛·米，N·m） |

> **注意：** 所有数组字段长度需与 `joint_index` 一致，支持同时控制多个关节。

#### `/control/hand` 详细说明

消息类型：`agx_arm_msgs/HandCmd`

**消息字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `mode` | `string` | 控制模式：`position`（位置）/ `speed`（速度）/ `current`（电流） |

**各手指目标值字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `thumb_tip` | `int8` | 拇指指尖目标值 |
| `thumb_base` | `int8` | 拇指指根目标值 |
| `index_finger` | `int8` | 食指目标值 |
| `middle_finger` | `int8` | 中指目标值 |
| `ring_finger` | `int8` | 无名指目标值 |
| `pinky_finger` | `int8` | 小指目标值 |

**不同模式下的数值范围：**

| 模式 | 数值范围 | 说明 |
|------|---------|------|
| `position` | [0, 100] | 0=完全张开，100=完全弯曲 |
| `speed` | [-100, 100] | 负值=张开方向，正值=弯曲方向 |
| `current` | [-100, 100] | 负值=张开方向，正值=弯曲方向 |

#### `/control/hand_position_time` 详细说明

消息类型：`agx_arm_msgs/HandPositionTimeCmd`

**消息字段说明：**

**各手指目标位置字段（范围：[0, 100]，0=完全张开，100=完全弯曲）：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `thumb_tip_pos` | `int8` | 拇指指尖位置 |
| `thumb_base_pos` | `int8` | 拇指指根位置 |
| `index_finger_pos` | `int8` | 食指位置 |
| `middle_finger_pos` | `int8` | 中指位置 |
| `ring_finger_pos` | `int8` | 无名指位置 |
| `pinky_finger_pos` | `int8` | 小指位置 |

**各手指到达时间字段（单位：10毫秒，范围：[0, 255]，例如：200 = 2秒）：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `thumb_tip_time` | `uint8` | 拇指指尖到达时间 |
| `thumb_base_time` | `uint8` | 拇指指根到达时间 |
| `index_finger_time` | `uint8` | 食指到达时间 |
| `middle_finger_time` | `uint8` | 中指到达时间 |
| `ring_finger_time` | `uint8` | 无名指到达时间 |
| `pinky_finger_time` | `uint8` | 小指到达时间 |

### 服务

| 服务 | 类型 | 说明 | 适用条件 |
|------|------|------|----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | 使能/失能机械臂 | 始终可用 |
| `/move_home` | `std_srvs/Empty` | 回零位 | 始终可用 |
| `/emergency_stop` | `std_srvs/Empty` | 急停（保持当前位置） | 始终可用 |

---

## 参数限制

### 夹爪 (Gripper)

| 参数 | 范围 | 默认值 | 说明 |
|------|------|--------|------|
| width (宽度) | [0.0, 0.1] m | - | 目标开合宽度 |
| force (力) | [0.5, 3.0] N | 1.0 | 夹持力度 |

> ⚠️ 超出范围的值将被拒绝（不执行），节点输出警告日志。例如：发送 force=5.0 时，该指令不会执行，并输出 `force must be in range [0.5, 3.0], current value: 5.0` 警告。

### 灵巧手 (Revo2)

| 参数 | 范围 | 说明 |
|------|------|------|
| position (位置) | [0, 100] | 手指目标位置，0 为完全张开，100 为完全握紧 |
| speed (速度) | [-100, 100] | 手指运动速度 |
| current (电流) | [-100, 100] | 手指驱动电流 |
| time (时间) | [0, 255] | 到达目标位置的时间（单位: 10ms，例如 100 = 1 秒） |

> ⚠️ 超出范围的值将被拒绝（不执行），节点输出警告日志。例如：发送 position=120 时，该指令不会执行，并输出 `position must be in range [0, 100], current value: 120` 警告。

---

## 注意事项

### CAN 通信

- 使用前**必须先激活 CAN 模块**
- 波特率：**1000000 bps**
- 若出现 `SendCanMessage failed` 错误，请检查 CAN 连接

### ⚠️ 安全警告

- **保持安全距离**：机械臂运动时，请勿进入其工作空间，以免造成伤害
- **奇异点风险**：靠近运动学奇异点时，关节可能发生突然大幅运动
- **MIT 模式危险**：高速响应的 MIT 模式极具危险性，请谨慎使用
