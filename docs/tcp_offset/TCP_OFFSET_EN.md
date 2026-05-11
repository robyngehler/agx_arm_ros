# TCP Offset Configuration

[中文](./TCP_OFFSET.md)

The active description and MoveIt surfaces in this workspace are now Nero-only, so this note documents TCP offset usage for Nero only.

## 1. `tcp_offset` definition

The six values of `tcp_offset` are `[x, y, z, rx, ry, rz]`:

| Dimension | Unit | Description |
|-----------|------|-------------|
| `x/y/z` | meter (m) | Position offset from the flange center |
| `rx/ry/rz` | radian (rad) | Orientation offset from the flange center |

## 2. Inspect the Nero flange in RViz

```bash
cd ~/agx_arm_ws
source install/setup.bash

ros2 launch agx_arm_description display_control.launch.py arm_type:=nero
```

With an end effector:

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper
```

Or:

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=revo2 \
	revo2_type:=left
```

Nero uses `link7` as the flange link. In RViz, enable TF or expand `RobotModel -> Links` to inspect `link7` and `tcp_link`.

## 3. Preview TCP offset

```bash
ros2 launch agx_arm_description display_control.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper \
	tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'
```

This publishes `tcp_link` 0.12 m in front of `link7`.

## 4. Use TCP offset in MoveIt

```bash
ros2 launch agx_arm_moveit demo.launch.py \
	arm_type:=nero \
	effector_type:=agx_gripper \
	tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'
```

With this set, the planning target and interactive marker align with `tcp_link`.