# TCP 偏移设置

[English](./TCP_OFFSET_EN.md)

当前工作区的活动描述与 MoveIt 配置都以 Nero 为准，因此本文只保留 Nero 的 TCP 偏移说明。

## 1. `tcp_offset` 参数定义

`tcp_offset` 的 6 个数值依次对应 `[x, y, z, rx, ry, rz]`：

| 维度 | 单位 | 说明 |
|------|------|------|
| `x/y/z` | 米 (m) | 工具中心相对法兰盘中心的空间位置偏移 |
| `rx/ry/rz` | 弧度 (rad) | 工具中心相对法兰盘中心的姿态偏移 |

## 2. 在 RViz 中查看 Nero 法兰盘

```bash
cd ~/agx_arm_ws
source install/setup.bash

ros2 launch agx_arm_description display_control.launch.py arm_type:=nero
```

若需要带末端执行器：

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper
```

或：

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=revo2 \
	revo2_type:=left
```

Nero 的末端法兰是 `link7`。在 RViz 中启用 TF 或展开 `RobotModel -> Links` 即可查看 `link7` 与 `tcp_link`。

## 3. 预览 TCP 偏移

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper \
	tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'
```

上例会在 `link7` 前方 0.12 m 处发布 `tcp_link`。

## 4. 在 MoveIt 中使用 TCP 偏移

```bash
ros2 launch agx_arm_moveit demo.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper \
	tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'
```

设置后，规划目标和交互标记会对齐到 `tcp_link`。