# agx_arm_description

当前工作区的 `agx_arm_description` 已收敛为 Nero 专用描述包。`agx_arm_urdf` 不再依赖独立 git submodule，而是作为仓库内固定资源树直接随包安装。

## 当前范围

- 机械臂型号：`nero`
- 末端执行器：`none`、`gripper`、`revo2_left`、`revo2_right`、`omnihand_left`、`omnihand_right`
- 控制兼容入口：`display_control.launch.py`（参数名与 `agx_arm_ctrl` / `agx_arm_moveit` 保持一致）
- 附加资源：相机支架、RealSense D435 挂载逻辑、已确认的 USD 资产

当前 OmniHand 在该包中的支持范围仍以描述、仿真与可视化集成为主；真机 ROS bridge、CAN bringup 与运行时控制归 `src/agx_arm_ctrl` 所有，当前可执行路径以 `docs/control/bringups/launches.md` 为准。

## 包结构

```text
agx_arm_description/
├── agx_arm_urdf/
│   ├── LICENSE
│   ├── README.md
│   ├── README_EN.md
│   ├── nero/
│   ├── omnihand/
│   └── revo2/
├── launch/
│   ├── display.launch.py
│   └── display_control.launch.py
├── urdf/
│   ├── agx_arm_description.urdf.xacro
│   └── USD/
├── meshes/
├── rviz/
├── config/
└── scripts/
```

## 依赖

```bash
sudo apt install \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro
```

## 编译

```bash
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to agx_arm_description
source install/setup.bash
```

## 启动方式

### 1. 纯显示入口 `display.launch.py`

该入口更适合直接可视化 Xacro 组合，参数名使用 `end_effector`。

默认模型：

```bash
ros2 launch agx_arm_description display.launch.py
```

Nero + 夹爪：

```bash
ros2 launch agx_arm_description display.launch.py \
    arm_type:=nero \
    end_effector:=gripper
```

Nero + 左手 Revo2：

```bash
ros2 launch agx_arm_description display.launch.py \
    arm_type:=nero \
    end_effector:=revo2_left
```

Nero + 左手 OmniHand：

```bash
ros2 launch agx_arm_description display.launch.py \
    arm_type:=nero \
    end_effector:=omnihand_left
```

Nero + 夹爪 + 相机支架 + D435：

```bash
ros2 launch agx_arm_description display.launch.py \
    arm_type:=nero \
    end_effector:=gripper \
    with_camera_stand:=true \
    with_camera:=true
```

### 2. 控制兼容入口 `display_control.launch.py`

该入口与 `agx_arm_ctrl` / `agx_arm_moveit` 的参数名对齐，使用 `effector_type` 与 `revo2_type`。

其中 `display.launch.py` 继续保留面向直接 Xacro 组合的 `end_effector` 参数；`display_control.launch.py` 则是与控制和 MoveIt 对齐的仓库级入口。OmniHand 在两者都可显示，但仿真和后续桥接工作优先以 `display_control.launch.py` / `agx_arm_moveit` 为准。

```bash
ros2 launch agx_arm_description display_control.launch.py \
    arm_type:=nero \
    effector_type:=agx_gripper
```

```bash
ros2 launch agx_arm_description display_control.launch.py \
    arm_type:=nero \
    effector_type:=revo2 \
    revo2_type:=right
```

```bash
ros2 launch agx_arm_description display_control.launch.py \
    arm_type:=nero \
    effector_type:=omnihand \
    omnihand_type:=left
```

## 参数说明

### `display.launch.py`

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `arm_type` | `nero` | 机械臂型号 | `nero` |
| `end_effector` | `none` | 末端执行器类型 | `none`, `gripper`, `revo2_left`, `revo2_right`, `omnihand_left`, `omnihand_right` |
| `with_camera_stand` | `false` | 是否加载相机支架 | `true`, `false` |
| `with_camera` | `false` | 是否加载 D435 | `true`, `false` |
| `use_gui` | `true` | 是否启动关节滑条 GUI | `true`, `false` |
| `rviz_config` | 内置配置 | RViz 配置文件路径 | 任意 `.rviz` 路径 |

### `display_control.launch.py`

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `arm_type` | `nero` | 机械臂型号 | `nero` |
| `custom_model` | 空字符串 | 可选自定义模型路径；设置后优先于内置 arm/effector 组合 | 绝对路径或 `agx_arm_urdf/` 下的相对路径 |
| `custom_model_xacro_args` | 空字符串 | 当 `custom_model` 指向 Xacro 时追加的额外参数字符串 | 任意合法 Xacro 参数串 |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2`, `omnihand` |
| `revo2_type` | `left` | Revo2 左右手 | `left`, `right` |
| `omnihand_type` | `left` | OmniHand 左右手 | `left`, `right` |
| `follow` | `false` | 是否跟随真实反馈 | `true`, `false` |
| `follow_joint_states_topic` | `feedback/joint_states` | 当 `follow:=true` 时消费的 JointState 话题；多臂前缀模型可指向适配后的反馈话题 | 任意合法 topic |
| `control` | `true` | 是否发布控制话题 | `true`, `false` |
| `control_topic` | `control/joint_states` | 关节滑条输出目标话题 | 任意合法 topic |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP 偏移 [x, y, z, rx, ry, rz] | - |
| `tcp_parent_frame` | 空字符串 | 自定义 TCP 静态变换的父坐标系；自定义模型若法兰名不是内置默认值，需要显式设置 | 任意合法 frame 名 |

## 解析 Xacro

在 ROS2 环境中可以直接解析当前 Nero 模型：

```bash
xacro urdf/agx_arm_description.urdf.xacro arm_type:=nero > nero.urdf
```

或者使用 `ros2 run xacro xacro`：

```bash
ros2 run xacro xacro \
    urdf/agx_arm_description.urdf.xacro \
    arm_type:=nero \
    end_effector:=gripper \
    with_camera_stand:=true \
    with_camera:=true \
    -o nero_full.urdf
```

## 自定义模型路径

`display_control.launch.py` 仍支持 `custom_model`。相对路径会在 `agx_arm_urdf/` 下解析，例如：

```bash
ros2 launch agx_arm_description display_control.launch.py \
    custom_model:=nero/urdf/nero_with_gripper_description.xacro
```

如果 `custom_model` 本身是带参数的 Xacro，还可以额外传入 `custom_model_xacro_args`，例如：

```bash
ros2 launch agx_arm_description display_control.launch.py \
    custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
    custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true' \
    follow:=true \
    follow_joint_states_topic:=feedback/prefixed_joint_states \
    tcp_parent_frame:=right_arm_nero_tool0 \
    tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

## USD 资产说明

已确认的 USD 资产位于 `urdf/USD/nero_gripper_d435/`。Sprint 1 当前只将该路径视为已验证资产，其它 Isaac 资产仍按部分可用处理。

## License

MIT License
