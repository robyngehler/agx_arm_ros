# AgileX 机械臂本地 URDF 资产

[English](./README_EN.md)

该目录现在是 `agx_arm_description` 包内随仓库提交的固定资产树，不再依赖独立的 `agx_arm_urdf` git submodule。

## 当前保留内容

| 目录 | 说明 |
|------|------|
| `nero/` | Nero 本体、夹爪、Revo2 组合所需的 URDF/Xacro 与 mesh 资源 |
| `omnihand/` | 当前仓库归一化后的 OmniHand 描述、URDF 与 mesh 资源 |
| `revo2/` | Revo2 手模型与 mesh 资源 |
| `README.md` / `README_EN.md` | 当前资产树说明 |
| `LICENSE` | 上游许可证保留 |

## 目录结构

```text
agx_arm_urdf/
├── LICENSE
├── README.md
├── README_EN.md
├── nero/
│   ├── meshes/
│   └── urdf/
├── omnihand/
│   ├── meshes/
│   ├── meshes_pro/
│   └── urdf/
└── revo2/
    ├── meshes/
    └── urdf/
```

## 典型文件

- `nero/urdf/nero_description.urdf`
- `nero/urdf/nero_with_gripper_description.xacro`
- `nero/urdf/nero_with_left_revo2_description.xacro`
- `nero/urdf/nero_with_right_revo2_description.xacro`
- `omnihand/urdf/`
- `revo2/urdf/revo2_left_hand.urdf`
- `revo2/urdf/revo2_right_hand.urdf`

## 使用方式

推荐直接通过主仓中的 `agx_arm_description` 包使用这些资源：

```bash
ros2 launch agx_arm_description display_control.launch.py arm_type:=nero
```

或：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left
```

如果需要在其它工作区复用，请整体复制 `agx_arm_urdf/` 目录，并确保它被安装到同名的 `agx_arm_description` ROS 包共享目录下。

## 许可证

本目录保留上游 [MIT License](./LICENSE)。
