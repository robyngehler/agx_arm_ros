# Repo Interaction Diagrams

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-17

## Purpose

This document provides the stable visual reference for how the current repo behaves during Sprint 2.

It focuses on the currently active ROS2 runtime and launch surfaces:

- `src/agx_arm_ctrl`
- `src/agx_arm_moveit`
- `src/agx_arm_sim/agx_arm_description`
- `src/agx_arm_msgs`

Use this document together with:

- `docs/project/repository_structure.md`
- `docs/project/ros2_development_practices.md`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`

## 1. ROS2 Nodes And Interfaces

This diagram shows the main repo-owned ROS graph for the current arm plus MoveIt plus optional OmniHand bridge path.

```mermaid
flowchart LR
    CLI["RViz panel and CLI clients"]

    subgraph MoveIt["MoveIt and visualization"]
        MG["move_group"]
        CM["ros2_control_node<br/>controller_manager"]
        AC["arm_controller"]
        HC["hand controller<br/>optional profile"]
        RSP["robot_state_publisher"]
    end

    subgraph Runtime["Repo-owned runtime nodes"]
        AGX["agx_arm_ctrl_single_node"]
        OH["omnihand_bridge_node<br/>optional"]
    end

    CJS["control/joint_states<br/>sensor_msgs/JointState"]
    FJS["feedback/joint_states<br/>sensor_msgs/JointState"]
    OJS["feedback/omnihand/joint_states<br/>sensor_msgs/JointState"]
    OSTAT["feedback/omnihand/status<br/>agx_arm_msgs/OmniHandStatus"]
    OTACT["feedback/omnihand/tactile_raw<br/>agx_arm_msgs/OmniHandTactileRaw"]
    OTRAJ["control/omnihand/joint_trajectory<br/>trajectory_msgs/JointTrajectory"]

    CLI -- "plan and execute" --> MG
    MG -- "FollowJointTrajectory action" --> AC
    CM --> AC
    CM --> HC
    CM -- "joint_states remapped" --> CJS

    CJS --> AGX
    CJS --> OH
    OTRAJ --> OH

    CLI -- "enable_agx_arm, move_home,<br/>set_normal_mode, set_leader_mode,<br/>emergency_stop" --> AGX
    CLI -- "control/omnihand/stop" --> OH

    AGX -- "publish" --> FJS
    AGX -- "feedback/tcp_pose and feedback/arm_status" --> CLI

    OH -- "publish" --> OJS
    OH -- "publish" --> OSTAT
    OH -- "publish" --> OTACT

    OJS --> AGX
    FJS -. "follow=true" .-> RSP
    CJS -. "follow=false" .-> RSP
```

Notes:

- `agx_arm_ctrl_single_node` publishes the merged `feedback/joint_states` stream.
- When `effector_type:=omnihand`, `agx_arm_ctrl_single_node` subscribes to `feedback/omnihand/joint_states` and folds those joints into the combined feedback stream.
- The OmniHand bridge currently accepts both the shared `control/joint_states` path and the compatibility `control/omnihand/joint_trajectory` path.
- `robot_state_publisher` follows real feedback when `follow:=true`; otherwise it follows the mock-controller side `control/joint_states` stream.

## 2. Launch And Runtime Event Flow

These two diagrams separate launch-time branching from runtime topic flow.

The first diagram answers which launch files and nodes are started.

```mermaid
flowchart TD
    START["ros2 launch agx_arm_ctrl<br/>start_single_agx_arm_moveit.launch.py"] --> ARGS["resolve launch arguments"]
    ARGS --> CTRL["include start_single_agx_arm.launch.py"]
    ARGS --> DEMO["include agx_arm_moveit/demo.launch.py"]

    CTRL --> ARMNODE["start agx_arm_ctrl_single_node"]
    CTRL --> BRIDGEGATE{"effector_type is omnihand<br/>and launch_omnihand_bridge is true?"}
    BRIDGEGATE -- yes --> BRIDGENODE["start omnihand_bridge_node"]
    BRIDGEGATE -- no --> NOBRIDGE["skip OmniHand bridge"]

    DEMO --> BUILDER["build_moveit_config(context)"]
    BUILDER --> RSP["include rsp.launch.py"]
    BUILDER --> MOVEGROUP["include move_group.launch.py"]
    BUILDER --> RVIZ["optionally include moveit_rviz.launch.py"]
    BUILDER --> TMPCTRL["generate temporary ros2_controllers YAML"]

    TMPCTRL --> ROS2CTRL["start ros2_control_node"]
    ROS2CTRL --> SPAWN["spawn joint_state_broadcaster<br/>and arm or hand controllers"]
    RVIZ --> PANEL["RViz MotionPlanning panel"]
    PANEL --> MOVEGROUP
    MOVEGROUP --> EXEC["plan and execute"]
    EXEC --> SPAWN
```

The second diagram answers how the runtime data paths behave after startup.

```mermaid
flowchart LR
    SPAWN["joint_state_broadcaster and arm or hand controllers"] --> CJS["control/joint_states"]
    CJS --> ARMNODE["agx_arm_ctrl_single_node"]
    CJS -. optional parallel input when bridge exists .-> BRIDGENODE["omnihand_bridge_node"]
    BRIDGENODE --> OJS["feedback/omnihand/joint_states"]
    OJS --> ARMNODE
    ARMNODE --> FJS["feedback/joint_states"]
    FJS --> RSPTRUE["robot_state_publisher<br/>follow=true"]
    CJS --> RSPFALSE["robot_state_publisher<br/>follow=false"]
```

Notes:

- The MoveIt side still uses a mock `ros2_control` system for planning and controller execution.
- `build_moveit_config(context)` does not start `omnihand_bridge_node`; bridge startup happens only inside `start_single_agx_arm.launch.py` when the launch condition is true.
- The main runtime path is `control/joint_states -> agx_arm_ctrl_single_node -> feedback/joint_states`.
- The OmniHand bridge consumes `control/joint_states` only as an optional parallel branch and never as a required hop for the arm controller path.
- When `effector_type:=omnihand`, `agx_arm_ctrl_single_node` also subscribes to `feedback/omnihand/joint_states` and merges those joints into the combined feedback stream.
- `follow:=true` makes the published robot model follow real feedback; `follow:=false` keeps visualization on the MoveIt/mock-controller side.

## 3. File Interaction Under Launches

This diagram shows how launch files pull together URDF, Xacro, SRDF, and description assets.

```mermaid
flowchart TD
    ARGS["arm_type, effector_type,<br/>revo2_type, omnihand_type,<br/>tcp_offset"]

    subgraph MoveItPath["MoveIt launch path"]
        M0["start_single_agx_arm_moveit.launch.py"]
        M1["agx_arm_moveit/demo.launch.py"]
        M2["_moveit_config_builder.py"]
        M3["config/agx_arm.urdf.xacro"]
        M4["config/agx_arm.srdf.xacro"]
        M5["config/agx_arm.ros2_control.xacro"]
        M6["agx_arm_description/agx_arm_urdf selected variant"]
        M7["config/initial_positions.yaml"]
        M8["robot_description"]
        M9["robot_description_semantic"]

        M0 --> M1 --> M2
        M2 --> M3
        M2 --> M4
        M3 --> M6
        M3 --> M5 --> M7
        M3 --> M8
        M4 --> M9
    end

    subgraph RvizPath["RViz compatibility path"]
        R0["start_single_agx_arm_rviz.launch.py"]
        R1["agx_arm_description/display_control.launch.py"]
        R2["resolve_model_path()"]
        R3["selected builtin or custom model"]
        R4["xacro selected model"]
        R5["robot_description"]

        R0 --> R1 --> R2 --> R3 --> R4 --> R5
    end

    ARGS --> M2
    ARGS --> R2
```

Notes:

- The MoveIt path builds its own `robot_description` and `robot_description_semantic` through `agx_arm_moveit/config/*.xacro`.
- The RViz compatibility path resolves directly into the canonical `agx_arm_description` asset tree.
- `tcp_offset` becomes a fixed `tcp_link` joint in the MoveIt URDF path and becomes an optional `static_transform_publisher` in the RViz compatibility path when the offset is nonzero.

## 4. Config And Definition Dataflow

This diagram shows which configuration files and definitions feed the current runtime.

```mermaid
flowchart LR
    MOVEITARGS["MoveIt launch args<br/>arm_type, effector_type,<br/>revo2_type, omnihand_type,<br/>tcp_offset, follow"] --> BUILDER["build_moveit_config()"]

    BUILDER --> URDF["config/agx_arm.urdf.xacro"]
    BUILDER --> SRDF["config/agx_arm.srdf.xacro"]
    BUILDER --> KIN["config/kinematics.yaml"]
    BUILDER --> LIMITS["config/joint_limits.yaml"]
    BUILDER --> SENSORS["config/sensors_3d.yaml"]
    BUILDER --> EXEC["config/moveit_controllers_profile.yaml"]
    BUILDER --> MOVEITNODES["move_group, RViz, robot_state_publisher"]

    URDF --> DESCPKG["agx_arm_description selected model variant"]
    URDF --> ROS2X["config/agx_arm.ros2_control.xacro"]
    ROS2X --> INIT["config/initial_positions.yaml"]

    MOVEITARGS --> TMPCTRL["_build_ros2_controllers_file(...)"]
    TMPCTRL --> TMPYAML["temporary ros2_controllers file"]
    TMPYAML --> ROS2CTRL["ros2_control_node"]

    ARMARGS["Arm runtime launch args<br/>can_port, pub_rate, auto_enable,<br/>speed_percent, effector_type,<br/>tcp_offset, gripper_default_effort"] --> ARMNODE["agx_arm_ctrl_single_node"]

    HANDARGS["OmniHand bridge launch args<br/>omnihand_type, backend_type,<br/>pub_rate, joint_states_command_topic,<br/>tactile_sample_count"] --> HANDNODE["omnihand_bridge_node"]
    MSGDEFS["src/agx_arm_msgs/msg/<br/>OmniHandStatus and OmniHandTactileRaw"] --> HANDNODE
```

Notes:

- The `moveit_controllers_profile.yaml` label stands for the profile selected by `effector_type`, for example `moveit_controllers_none.yaml` or `moveit_controllers_omnihand_left.yaml`.
- The temporary `ros2_controllers` file is generated at launch time from the selected arm and hand profile.
- Runtime node parameters are still sourced directly from launch arguments rather than from a single shared runtime YAML.