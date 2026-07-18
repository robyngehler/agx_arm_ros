# Architecture

status: ACTIVE_MIGRATION_BASELINE
last_updated: 2026-07-18

This document is the stable architecture reference for the current Duo baseline. It focuses on how
the repo-owned runtime surfaces interact, how launch files compose them, and where the public ROS
contract lives.

Use this together with:

- `repository_structure.md` for package ownership and documentation boundaries
- `components/README.md` for the stable component index
- `../control/bringups/launches.md` for runnable entrypoints

## Architecture summary

- `src/agx_arm_ctrl` owns the runtime arm bridge, launch surfaces, and current OmniHand bridge
  integration point
- `src/agx_arm_mit_controller` owns integrated `FollowJointTrajectory`, MIT command generation, and
  gravity-aware execution
- `src/agx_arm_moveit` owns the planning baseline and fake-controller compatibility path
- `src/agx_arm_sim/agx_arm_description` remains the canonical long-term description package
- `src/duo_body_description` remains the current Duo staging surface for body-mounted system assembly
- `src/agx_arm_msgs` owns repo-specific messages such as OmniHand diagnostics and tactile payloads

## 1. ROS2 nodes and interfaces

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

Key contract points:

- `feedback/joint_states` is the canonical combined follow-mode state
- hand-only diagnostics stay under `feedback/omnihand/*`
- the MoveIt default path is `move_group -> mit_controller -> control/move_mit -> agx_arm_ctrl_single_node`
- the `ros2_control` branch is a compatibility path, not the preferred production execution route

## 2. Launch and runtime flow

```mermaid
flowchart TD
    START["ros2 launch agx_arm_ctrl<br/>start_agx_arm_components.launch.py<br/>mode:=moveit_mit execution_profile:=right_arm"] --> ARGS["resolve launch arguments"]
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

## 3. File interaction under launches

```mermaid
flowchart TD
    ARGS["arm_type, effector_type,<br/>revo2_type, omnihand_type,<br/>tcp_offset"]

    subgraph MoveItPath["MoveIt launch path"]
        M0["start_agx_arm_components.launch.py\nmode:=moveit_mit execution_profile:=right_arm"]
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

## 4. Config and definition dataflow

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