# Architecture

status: ACTIVE_BASELINE
last_updated: 2026-08-18

This document is the stable architecture reference for the current Duo baseline. It focuses on how
the repo-owned runtime surfaces interact, how launch files compose them, and where the public ROS
contract lives.

**It does not describe the control-integrity layer.** Who may command a device,
which generation a command was issued under, who owns a device's vendor SDK
session, and what happens on an e-stop or a recovery are in
`control_integrity_architecture.md`. Read that one before changing anything on a
command path; this one is about composition and wiring.

Use this together with:

- `repository_structure.md` for package ownership and documentation boundaries
- `control_integrity_architecture.md` for authority, epochs, SDK ownership, recovery, and e-stop
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
    OAT["control/omnihand/authorized_trajectory<br/>agx_arm_msgs/AuthorizedJointTrajectory"]
    OJT["control/omnihand/joint_target<br/>agx_arm_msgs/HandJointTarget"]
    AUTH["feedback/authority<br/>agx_arm_msgs/AgxDeviceAuthority"]
    UNS["/unit_safety<br/>agx_arm_msgs/AgxUnitSafety"]

    CLI -- "plan and execute" --> MG
    MG -- "FollowJointTrajectory action<br/>use_mit_controller=true" --> MIT
    MG -. "FollowJointTrajectory action<br/>use_mit_controller=false" .-> AC
    CM --> AC
    CM --> HC
    CM -- "joint_states remapped" --> CJS
    CLI -- "debug sliders only" --> DBG
    DBG -- "debug JointTrajectory" --> MIT

    CJS -. "legacy, off by default" .-> AGX
    CJS -. "legacy, allow_legacy_hand_command_ingress" .-> OH
    OAT --> OH
    OJT --> OH
    FJS --> MIT
    MIT -- "control/move_mit<br/>stamped MoveMITMsg" --> AGX

    AGX -- "publish, latched" --> AUTH
    OH -- "publish, latched" --> AUTH
    AUTH --> MIT
    UNS --> AGX
    UNS --> OH

    CLI -- "enable_agx_arm, move_home,<br/>set_normal_mode, set_leader_mode,<br/>emergency_stop, claim_device" --> AGX
    CLI -- "mit_controller/enable,<br/>hold_current, cancel_trajectory" --> MIT
    CLI -- "control/omnihand/stop,<br/>control/omnihand/claim_device" --> OH

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
- **every production command carries the authority it was issued under.** The
  arm's `MoveMITMsg` and the hand's `AuthorizedJointTrajectory` /
  `HandJointTarget` all embed `owner_id`, `device_epoch`, `unit_safety_epoch`
  and `sequence`; the receiving device admits on the stamp the command *arrived
  with* and never substitutes a field from its own state
- **the bare surfaces are legacy and off.** Shared `control/joint_states` and
  `control/omnihand/joint_trajectory` reach a device only under
  `allow_legacy_hand_command_ingress` / `allow_legacy_motion_ingress`, both
  default false. They self-stamp, so they cannot refuse a stale or reordered
  command — never describe them as authority-safe
- **four devices, four buses, four authorities.** Arms on `can_nero_left` /
  `can_nero_right` (native `mttcan`), hands on `hand_left` / `hand_right`
  (USB-CAN FD adapters). Same-side arm and hand motion runs in parallel;
  `shared_per_side` is a selectable degraded topology declared once as
  `bus_topology` in the registry

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

    ARMARGS["Arm runtime launch args<br/>can_port, pub_rate, auto_enable,<br/>speed_percent, effector_type,<br/>tcp_offset, gripper_default_effort,<br/>allow_legacy_gripper_command_ingress"] --> ARMNODE["agx_arm_ctrl_single_node"]

    HANDARGS["OmniHand bridge launch args<br/>omnihand_type, backend_type,<br/>pub_rate, joint_states_command_topic,<br/>tactile_sample_count"] --> HANDNODE["omnihand_bridge_node"]
    MSGDEFS["src/agx_arm_msgs/msg/<br/>OmniHandStatus and OmniHandTactileRaw"] --> HANDNODE
```