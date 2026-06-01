import type { WorkspaceData } from "./types";

export const mockData: WorkspaceData = {
  root: "/workspace/agx_arm_ros",
  scannedAt: new Date().toISOString(),
  packages: [
    { name: "agx_arm_ctrl", path: "src/agx_arm_ctrl", deps: ["agx_arm_msgs", "sensor_msgs", "std_srvs"], buildType: "ament_python" },
    { name: "agx_arm_msgs", path: "src/agx_arm_msgs", deps: [], buildType: "ament_cmake" },
    { name: "agx_arm_description", path: "src/agx_arm_description", deps: [], buildType: "ament_cmake" },
    { name: "agx_arm_moveit", path: "src/agx_arm_moveit", deps: ["agx_arm_ctrl", "agx_arm_description"], buildType: "ament_cmake" },
    { name: "agx_arm_mit_controller", path: "src/agx_arm_mit_controller", deps: ["agx_arm_msgs"], buildType: "ament_python" },
  ],
  nodes: [
    {
      id: "agx_arm_ctrl/agx_arm_ctrl_single_node",
      nodeName: "agx_arm_ctrl_single_node",
      package: "agx_arm_ctrl",
      filePath: "src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py",
      topics: [
        { topic: "control/joint_states", msgType: "sensor_msgs/JointState", direction: "pub" },
        { topic: "feedback/joint_states", msgType: "sensor_msgs/JointState", direction: "pub" },
        { topic: "agx_arm_status", msgType: "agx_arm_msgs/AgxArmStatus", direction: "pub" },
        { topic: "gripper_status", msgType: "agx_arm_msgs/GripperStatus", direction: "pub" },
        { topic: "hand_status", msgType: "agx_arm_msgs/HandStatus", direction: "pub" },
        { topic: "hand_cmd", msgType: "agx_arm_msgs/HandCmd", direction: "sub" },
        { topic: "move_mit", msgType: "agx_arm_msgs/MoveMITMsg", direction: "sub" },
        { topic: "target_pose", msgType: "geometry_msgs/PoseStamped", direction: "sub" },
      ],
      services: [
        { service: "enable_arm", srvType: "std_srvs/SetBool", role: "server" },
        { service: "reset_arm", srvType: "std_srvs/Empty", role: "server" },
        { service: "trigger_gravity_comp", srvType: "std_srvs/Trigger", role: "server" },
      ],
      actions: [],
      parameters: [
        { name: "can_port", type: "string", default: "can0", description: "CAN port" },
        { name: "arm_type", type: "string", default: "nero", description: "Arm model" },
        { name: "effector_type", type: "string", default: "none", description: "End effector" },
        { name: "auto_enable", type: "bool", default: "true", description: "Auto-enable on start" },
        { name: "fast_mode", type: "bool", default: "false", description: "Fast publish mode" },
        { name: "namespace", type: "string", default: "", description: "ROS namespace" },
      ],
      lifecycleNode: false,
    },
    {
      id: "agx_arm_ctrl/omnihand_bridge_node",
      nodeName: "omnihand_bridge_node",
      package: "agx_arm_ctrl",
      filePath: "src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py",
      topics: [
        { topic: "omnihand_status", msgType: "agx_arm_msgs/OmniHandStatus", direction: "pub" },
        { topic: "omnihand_tactile_raw", msgType: "agx_arm_msgs/OmniHandTactileRaw", direction: "pub" },
        { topic: "hand_cmd", msgType: "agx_arm_msgs/HandCmd", direction: "sub" },
        { topic: "hand_position_time_cmd", msgType: "agx_arm_msgs/HandPositionTimeCmd", direction: "sub" },
      ],
      services: [],
      actions: [],
      parameters: [
        { name: "omnihand_type", type: "string", default: "left", description: "left or right" },
        { name: "backend_type", type: "string", default: "mock", description: "OmniHand backend" },
        { name: "canfd_port", type: "string", default: "can1", description: "CANFD port" },
      ],
      lifecycleNode: false,
    },
    {
      id: "agx_arm_mit_controller/mit_controller_node",
      nodeName: "mit_controller_node",
      package: "agx_arm_mit_controller",
      filePath: "src/agx_arm_mit_controller/agx_arm_mit_controller/mit_controller_node.py",
      topics: [
        { topic: "move_mit", msgType: "agx_arm_msgs/MoveMITMsg", direction: "pub" },
        { topic: "control/joint_states", msgType: "sensor_msgs/JointState", direction: "sub" },
      ],
      services: [
        { service: "start_gravity_calibration", srvType: "std_srvs/Trigger", role: "server" },
      ],
      actions: [],
      parameters: [
        { name: "kp", type: "double", default: "1.0" },
        { name: "kd", type: "double", default: "0.0" },
      ],
      lifecycleNode: false,
    },
    {
      id: "agx_arm_moveit/move_group",
      nodeName: "move_group",
      package: "agx_arm_moveit",
      filePath: "src/agx_arm_moveit/",
      topics: [
        { topic: "feedback/joint_states", msgType: "sensor_msgs/JointState", direction: "sub" },
        { topic: "target_pose", msgType: "geometry_msgs/PoseStamped", direction: "pub" },
        { topic: "display_planned_path", msgType: "moveit_msgs/DisplayTrajectory", direction: "pub" },
      ],
      services: [],
      actions: [
        { action: "move_group", actionType: "moveit_msgs/MoveGroup", role: "server" },
      ],
      parameters: [
        { name: "robot_description", type: "string", default: "", description: "URDF" },
        { name: "robot_description_semantic", type: "string", default: "", description: "SRDF" },
      ],
      lifecycleNode: false,
    },
  ],
  launches: [
    {
      id: "agx_arm_ctrl/launch/start_single_agx_arm",
      filePath: "src/agx_arm_ctrl/launch/start_single_agx_arm.launch.py",
      package: "agx_arm_ctrl",
      args: [
        { name: "log_level", default: "info", description: "Logging level (debug, info, warn, error, fatal)" },
        { name: "namespace", default: "", description: "ROS namespace for this arm instance (e.g. arm1)" },
        { name: "can_port", default: "can0", description: "CAN port to be used by the AGX Arm node" },
        { name: "arm_type", default: "nero", description: "Robotic arm type", choices: ["nero"] },
        { name: "effector_type", default: "none", description: "End effector type", choices: ["none", "agx_gripper", "revo2", "omnihand"] },
        { name: "omnihand_type", default: "left", description: "OmniHand type", choices: ["left", "right"] },
        { name: "launch_omnihand_bridge", default: "false", description: "Launch the OmniHand bridge", choices: ["true", "false"] },
        { name: "omnihand_backend_type", default: "mock", description: "Backend type for OmniHand bridge" },
        { name: "auto_enable", default: "true", description: "Automatically enable the AGX Arm", choices: ["true", "false"] },
        { name: "fast_mode", default: "false", description: "Enable fast mode", choices: ["true", "false"] },
      ],
      nodes: [
        { package: "agx_arm_ctrl", executable: "agx_arm_ctrl_single_node", name: "agx_arm_ctrl_single_node" },
      ],
      includes: [
        { file: "agx_arm_ctrl/launch/start_omnihand_bridge.launch.py", condition: "launch_omnihand_bridge == 'true'" },
        { file: "agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py" },
      ],
    },
    {
      id: "agx_arm_ctrl/launch/start_omnihand_bridge",
      filePath: "src/agx_arm_ctrl/launch/start_omnihand_bridge.launch.py",
      package: "agx_arm_ctrl",
      args: [
        { name: "omnihand_type", default: "left", choices: ["left", "right"] },
        { name: "omnihand_backend_type", default: "mock" },
        { name: "canfd_port", default: "can1" },
      ],
      nodes: [
        { package: "agx_arm_ctrl", executable: "omnihand_bridge_node", name: "omnihand_bridge_node" },
      ],
      includes: [],
    },
    {
      id: "agx_arm_ctrl/launch/start_single_agx_arm_moveit",
      filePath: "src/agx_arm_ctrl/launch/start_single_agx_arm_moveit.launch.py",
      package: "agx_arm_ctrl",
      args: [
        { name: "namespace", default: "" },
        { name: "can_port", default: "can0" },
        { name: "arm_type", default: "nero" },
      ],
      nodes: [
        { package: "moveit_ros_move_group", executable: "move_group", name: "move_group" },
        { package: "rviz2", executable: "rviz2", name: "rviz2" },
      ],
      includes: [
        { file: "agx_arm_ctrl/launch/start_single_agx_arm.launch.py" },
      ],
    },
    {
      id: "agx_arm_ctrl/launch/start_single_agx_arm_rviz",
      filePath: "src/agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py",
      package: "agx_arm_ctrl",
      args: [
        { name: "namespace", default: "" },
      ],
      nodes: [
        { package: "robot_state_publisher", executable: "robot_state_publisher", name: "robot_state_publisher" },
        { package: "rviz2", executable: "rviz2", name: "rviz2" },
      ],
      includes: [],
    },
  ],
  messages: [
    { name: "AgxArmStatus", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/AgxArmStatus.msg", fields: [{ name: "joint_positions", type: "float64[]" }, { name: "joint_velocities", type: "float64[]" }, { name: "joint_efforts", type: "float64[]" }, { name: "is_enabled", type: "bool" }] },
    { name: "GripperStatus", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/GripperStatus.msg", fields: [{ name: "position", type: "float64" }, { name: "is_moving", type: "bool" }] },
    { name: "HandCmd", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/HandCmd.msg", fields: [{ name: "finger_positions", type: "float64[]" }] },
    { name: "HandPositionTimeCmd", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/HandPositionTimeCmd.msg", fields: [{ name: "finger_positions", type: "float64[]" }, { name: "time_ms", type: "uint32" }] },
    { name: "HandStatus", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/HandStatus.msg", fields: [{ name: "finger_positions", type: "float64[]" }, { name: "finger_efforts", type: "float64[]" }] },
    { name: "MoveMITMsg", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/MoveMITMsg.msg", fields: [{ name: "joint_ids", type: "uint8[]" }, { name: "positions", type: "float64[]" }, { name: "velocities", type: "float64[]" }, { name: "kp", type: "float64[]" }, { name: "kd", type: "float64[]" }] },
    { name: "OmniHandStatus", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/OmniHandStatus.msg", fields: [{ name: "finger_positions", type: "float64[]" }, { name: "finger_efforts", type: "float64[]" }, { name: "is_connected", type: "bool" }] },
    { name: "OmniHandTactileRaw", package: "agx_arm_msgs", kind: "msg", filePath: "src/agx_arm_msgs/msg/OmniHandTactileRaw.msg", fields: [{ name: "data", type: "uint8[]" }, { name: "finger_id", type: "uint8" }] },
  ],
  entryPoints: [
    { name: "agx_arm_ctrl_single_node", module: "agx_arm_ctrl.agx_arm_ctrl_single_node:main", package: "agx_arm_ctrl" },
    { name: "omnihand_bridge_node", module: "agx_arm_ctrl.omnihand_bridge_node:main", package: "agx_arm_ctrl" },
    { name: "mit_controller_node", module: "agx_arm_mit_controller.mit_controller_node:main", package: "agx_arm_mit_controller" },
  ],
};
