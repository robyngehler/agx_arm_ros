# agx_arm_sim

该目录现在主要承担仿真与描述相关资源的宿主作用，其中活动内容已经收敛到 Nero 主线。

## 当前有效组成

```text
agx_arm_sim/
├── agx_arm_description/    # 当前活动的 Nero/Revo2 描述包与 USD 相关资源
├── realsense2_description/ # RealSense D435 相关描述资源
└── Moveit2/                # 历史参考目录，不再作为当前工作区的活动 MoveIt 包
```

## 当前职责

- `agx_arm_description/` 是当前工作区中唯一活动的描述包。
- `agx_arm_description/agx_arm_urdf/` 已经直接随仓库提交，只保留 `nero/`、`revo2/` 以及许可证/README。
- 当前活动的 MoveIt 包不在这里，而是在工作区顶层的 `src/agx_arm_moveit/`。
- 已确认的 Isaac/USD 资产位于 `agx_arm_description/urdf/USD/nero_gripper_d435/`。

## 使用建议

若你要在当前工作区做可视化或仿真联调，优先使用：

```bash
ros2 launch agx_arm_description display_control.launch.py arm_type:=nero
```

若你要做当前活动的 MoveIt 规划，请使用：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```

## 说明

- 本目录下的 `Moveit2/` 子目录保留为历史资料，不代表当前推荐或默认的规划入口。
- 当前仓库不再依赖 `agx_arm_urdf` 独立子模块；唯一保留的子模块是顶层 `vendor/OmniHand-Pro-2025`。

## License

MIT License
