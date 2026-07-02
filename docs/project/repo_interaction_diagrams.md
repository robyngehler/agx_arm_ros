# Repo Interaction Diagrams

status: ACTIVE_DUO_BASELINE
last_updated: 2026-07-02

## Purpose

This document provides the stable visual reference for how the current repo behaves in the Duo baseline.

It focuses on the currently active ROS2 runtime and launch surfaces:

- `src/agx_arm_ctrl`
- `src/agx_arm_mit_controller`
- `src/agx_arm_moveit`
- `src/agx_arm_sim/agx_arm_description`
- `src/agx_arm_msgs`

`src/duo_body_description` is intentionally not a primary actor in these diagrams yet. It is the current description-only Sprint 3 and Sprint 4 staging package for Duo body system bringup, while the runtime ownership shown here remains in the existing `agx_arm_*` packages.

Use this document together with:

- `docs/project/repository_structure.md`
- `.claude/rules/ros2-development.md`
- `docs/assets/omnihand/omnihand_ros_integration_options.md`
- `docs/assets/omnihand/omnihand_wrapper_integration_plan.md`

## 1. ROS2 Nodes And Interfaces

This diagram shows the main repo-owned ROS graph for the current arm plus MoveIt plus optional OmniHand bridge path.

```mermaid
flowchart LR
    CLI["RViz panel and CLI clients"]

    subgraph MoveIt["MoveIt and visualization"]
        MG["move_group"]
        CM["ros2_control_node<br/>controller_manager<br/>legacy fake-controller path"]
        AC["arm_controller<br/>legacy fake controller"]
        DBG["agx_arm_mit_joint_state_bridge<br/>debug soft-target only"]
        HC["hand controller<br/>optional profile"]
        RSP["robot_state_publisher"]
    end

    subgraph Runtime["Repo-owned runtime nodes"]
        MIT["mit_controller<br/>integrated FJT action server"]
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
    MG -- "FollowJointTrajectory action<br/>use_mit_controller=true" --> MIT
    MG -. "FollowJointTrajectory action<br/>use_mit_controller=false" .-> AC
    CM --> AC
    CM --> HC
    CM -- "joint_states remapped" --> CJS
    CLI -- "debug sliders only" --> DBG
    DBG -- "debug JointTrajectory" --> MIT

    CJS --> AGX
    CJS --> OH
    OTRAJ --> OH
    FJS --> MIT
    MIT -- "control/move_mit" --> AGX

    CLI -- "enable_agx_arm, move_home,<br/>set_normal_mode, set_leader_mode,<br/>emergency_stop" --> AGX
    CLI -- "mit_controller/enable,<br/>hold_current, cancel_trajectory" --> MIT
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
- `mit_controller` is the default execution target for MoveIt when `use_mit_controller:=true`.
- The `ros2_control_node` plus `controller_manager` branch is only the legacy fake-controller execution path when `use_mit_controller:=false`.
- `agx_arm_mit_joint_state_bridge` is a debug-only RViz helper and should not be treated as the production execution path.
- When `effector_type:=omnihand`, `agx_arm_ctrl_single_node` subscribes to `feedback/omnihand/joint_states` and folds those joints into the combined feedback stream.
- The OmniHand bridge currently accepts both the shared `control/joint_states` path and the compatibility `control/omnihand/joint_trajectory` path.
- `robot_state_publisher` follows real feedback when `follow:=true`; otherwise it follows the mock-controller side `control/joint_states` stream.

## 2. Launch And Runtime Event Flow

These two diagrams separate launch-time branching from runtime topic flow.

The first diagram answers which launch files and nodes are started.

```mermaid
flowchart TD
    START["ros2 launch agx_arm_ctrl<br/>start_single_agx_arm_moveit.launch.py"] --> ARGS["resolve launch arguments"]
    ARGS --> CTRL["include start_single_agx_arm.launch.py<br/>or start_nero_mit_controller.launch.py"]
    ARGS --> DEMO["include agx_arm_moveit/demo.launch.py"]

    CTRL --> ARMNODE["start agx_arm_ctrl_single_node"]
    CTRL --> BRIDGEGATE{"effector_type is omnihand<br/>and launch_omnihand_bridge is true?"}
    BRIDGEGATE -- yes --> BRIDGENODE["start omnihand_bridge_node"]
    BRIDGEGATE -- no --> NOBRIDGE["skip OmniHand bridge"]

    DEMO --> BUILDER["build_moveit_config(context)"]
    BUILDER --> RSP["include rsp.launch.py"]
    BUILDER --> MOVEGROUP["include move_group.launch.py"]
    BUILDER --> RVIZ["optionally include moveit_rviz.launch.py"]
    BUILDER --> MITMODE{"use_mit_controller?"}

    MITMODE -- true --> MITEXEC["use integrated mit_controller action server"]
    MITMODE -- false --> TMPCTRL["generate temporary ros2_controllers YAML"]

    TMPCTRL --> ROS2CTRL["start ros2_control_node"]
    ROS2CTRL --> SPAWN["spawn joint_state_broadcaster<br/>and arm or hand controllers"]
    RVIZ --> PANEL["RViz MotionPlanning panel"]
    PANEL --> MOVEGROUP
    MOVEGROUP --> EXEC["plan and execute"]
    EXEC -- "use_mit_controller=true" --> MITEXEC
    EXEC -- "use_mit_controller=false" --> SPAWN
```

The second diagram answers how the runtime data paths behave after startup.

```mermaid
flowchart LR
    MOVEIT["move_group"] -- "FollowJointTrajectory<br/>use_mit_controller=true" --> MIT["mit_controller"]
    SPAWN["joint_state_broadcaster and arm or hand controllers<br/>legacy fake-controller path"] --> CJS["control/joint_states"]
    CJS --> ARMNODE["agx_arm_ctrl_single_node"]
    CJS -. optional parallel input when bridge exists .-> BRIDGENODE["omnihand_bridge_node"]
    DBG["agx_arm_mit_joint_state_bridge<br/>debug_soft_target only"] --> DJT["mit_controller/joint_trajectory<br/>debug only"]
    DJT --> MIT
    MIT --> MMIT["control/move_mit"]
    MMIT --> ARMNODE
    BRIDGENODE --> OJS["feedback/omnihand/joint_states"]
    OJS --> ARMNODE
    ARMNODE --> FJS["feedback/joint_states"]
    FJS --> MIT
    FJS --> RSPTRUE["robot_state_publisher<br/>follow=true"]
    CJS --> RSPFALSE["robot_state_publisher<br/>follow=false"]
```

Notes:

- The MoveIt side uses the integrated `mit_controller` action server by default and only falls back to the mock `ros2_control` system when `use_mit_controller:=false`.
- `build_moveit_config(context)` does not start `omnihand_bridge_node`; bridge startup happens only inside `start_single_agx_arm.launch.py` when the launch condition is true.
- The main runtime execution path in MIT mode is `move_group -> mit_controller -> control/move_mit -> agx_arm_ctrl_single_node -> feedback/joint_states`.
- The `mit_controller/joint_trajectory` debug topic is only for explicit RViz soft-target debugging.
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
- `tcp_offset` becomes a fixed `tcp_link` joint off `nero_tool0` in the MoveIt URDF path.
- In the built-in Nero RViz compatibility path, `display_control.launch.py` now always publishes the `nero_tool0` to `tcp_link` transform, including the zero-offset case.
- If `custom_model` is used in RViz, the custom asset owns its own TCP/flange frames and `display_control.launch.py` does not inject a Nero-specific `tcp_link` transform.

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

    MOVEITARGS --> MITMODE{"use_mit_controller"}
    MITMODE -- false --> TMPCTRL["_build_ros2_controllers_file(...)"]
    TMPCTRL --> TMPYAML["temporary ros2_controllers file"]
    TMPYAML --> ROS2CTRL["ros2_control_node"]
    MITMODE -- true --> MITNODE["mit_controller"]

    MITARGS["MIT launch args<br/>control_rate_hz, params_file,<br/>enable_debug_joint_trajectory_topic,<br/>launch_driver"] --> MITNODE

    ARMARGS["Arm runtime launch args<br/>can_port, pub_rate, auto_enable,<br/>speed_percent, effector_type,<br/>tcp_offset, gripper_default_effort"] --> ARMNODE["agx_arm_ctrl_single_node"]

    HANDARGS["OmniHand bridge launch args<br/>omnihand_type, backend_type,<br/>pub_rate, joint_states_command_topic,<br/>tactile_sample_count"] --> HANDNODE["omnihand_bridge_node"]
    MSGDEFS["src/agx_arm_msgs/msg/<br/>OmniHandStatus and OmniHandTactileRaw"] --> HANDNODE
```

Notes:

- The `moveit_controllers_profile.yaml` label stands for the selected controller profile, including `moveit_controllers_mit.yaml` when `use_mit_controller:=true`.
- The temporary `ros2_controllers` file is generated only for the legacy fake-controller branch when `use_mit_controller:=false`.
- MIT runtime parameters are sourced from launch arguments plus the MIT params YAML, and the debug `joint_trajectory` input stays opt-in.
- Runtime node parameters are still sourced directly from launch arguments rather than from a single shared runtime YAML.