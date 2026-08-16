# AgileX 机械臂 ROS2 驱动

[English](./README_EN.md)

## 仓库定位

该仓库是当前 Duo/Nero 基线的 ROS2 工作区，包含机械臂运行时、MIT 软控制、MoveIt、
OmniHand 集成、协调执行以及相关文档。

根 README 只保留项目入口与导航；具体运行方法、环境规则和架构说明统一进入 `docs/`。

## 文档入口

- `docs/README.md`：全局文档导航
- `docs/target/README.md`：当前仓库目标与文档归属规则
- `docs/control/environment.md`：系统 Python、Conda、ROS overlay、build/test wrapper 与平台边界
- `docs/control/bringups/launches.md`：当前规范启动矩阵
- `docs/control/bringups/teach_and_run.md`：teach、record、replay 与协调执行工作流
- `docs/project/architecture.md`：组件关系与 Mermaid 架构图
- `docs/project/repository_structure.md`：包边界、文档分层与稳定职责
- `docs/checklist.md`：全局迁移与集成状态
- `docs/errors_and_fixes.md`：跨组件问题、当前规避策略与已验证修复
- `docs/open_questions.md`：Human 与 Agent 的全局设计问题交换面

## 核心包

- `src/agx_arm_ctrl`：机械臂运行时 bridge、launch surface、当前 OmniHand 集成点
- `src/agx_arm_mit_controller`：MIT 执行、重力补偿与 `FollowJointTrajectory`
- `src/agx_arm_moveit`：MoveIt 规划基线与兼容仿真控制路径
- `src/agx_arm_coordination`：双臂/双手任务协调与 Activity-DAG 执行
- `src/agx_arm_sim/agx_arm_description`：长期 canonical 描述资产
- `src/duo_body_description`：当前 Duo 系统 staging 描述包
- `src/agx_arm_msgs`：仓库自有消息
- `vendor/OmniHand-Pro-2025`：上游 SDK 输入，不是公共 ROS 合约

## 最短正确路径

1. 克隆仓库并同步 submodule。`ros2` 是稳定公开分支，下面的示例克隆的就是它：
   `git clone -b ros2 --recurse-submodules ...`
   当前 V02 重构在 `ROS2_Duo_System_V02_refactor` 上进行；按分支相关的重构文档操作时请显式检出该分支。
2. 安装系统与 ROS 依赖：`bash ./scripts/agx_arm_install_deps.sh`
3. 使用系统 Python wrapper 编译：`bash ./scripts/colcon_build_system_python.sh`
4. 如需 Conda 运行环境：`bash ./scripts/setup_agx_arm_runtime_env.sh`
5. 使用运行 wrapper 执行 ROS 命令：
   `bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_arm`

## 环境边界

- `colcon build` 与 `colcon test` 统一走系统 Python
- Conda 只用于可选运行时与 Python 侧开发依赖，并通过仓库 wrapper 进入
- 不要在同一 shell 中手工混用 `conda activate` 与 `source install/setup.bash`
- 真机 CAN、机械臂和 OmniHand 测试只适用于 Jetson 或其他 `aarch64` ROS 硬件环境
- x86 或纯编辑器环境仅适合文档、代码与离线验证

## 运行提示

- 实际 bringup 与脚本矩阵以 `docs/control/bringups/launches.md` 为准
- teach/replay 工作流以 `docs/control/bringups/teach_and_run.md` 为准
- 正常拓扑（`dedicated_per_device`）是每个设备独占一条 CAN 总线：机械臂用原生
  `can_nero_left` / `can_nero_right`，手用 USB-CAN FD 适配器上的 `hand_left` / `hand_right`。
  同侧机械臂与手可以并行运动
- 共享 arm-plus-hand CAN（`shared_per_side`）只是可选的降级兼容模式，不是正常架构；
  其已知风险见 `docs/errors_and_fixes.md`