# Moveit2 历史参考目录

本目录保留了较早期按型号拆分的 MoveIt2 配置资料，仅作为历史参考，不再代表当前工作区的活动规划入口。

## 当前状态

- 当前活动 MoveIt 包是工作区顶层的 `src/agx_arm_moveit/`。
- `src/agx_arm_moveit/` 已统一为参数化配置，并且当前活动表面只保留 `arm_type:=nero`。
- 本目录中旧的按型号拆分配置不应再作为新开发的默认起点。

## 如果你要运行当前 MoveIt

请使用：

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero
```

真实机械臂一键启动：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
   can_port:=can_nero_right \
   arm_type:=nero \
   effector_type:=agx_gripper
```

旧的公开运行时名称如 `can0`、`can_nero` 仅可视为历史痕迹；当前对外运行时接口请统一使用 `can_nero_right` 或 `can_nero_left`。

## 为什么保留本目录

- 作为旧布局与旧文档的参考材料。
- 作为 Isaac/MoveIt 早期实验路径的存档。
- 便于 Sprint 1 文档说明仓库从“按型号拆分”迁移到了“统一 Nero 主线参数化包”。

#### 步骤 4：完成联合仿真

启动完成后，Isaac Sim 中的机械臂关节状态会自动同步到 MoveIt2 中，你可以在 RViz2 中完成运动规划的交互：

- 拖动交互标记，指定机械臂的目标位姿
- MoveIt2 会自动完成碰撞检测与轨迹规划
- 规划完成的轨迹会自动发送到 Isaac Sim 中，控制仿真中的机械臂完成运动，实现高保真的联合仿真测试

![](./img/nero_isaac.png)

## License

MIT License
