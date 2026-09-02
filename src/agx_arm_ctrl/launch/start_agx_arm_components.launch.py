from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

from agx_arm_ctrl.execution_profiles import resolve_execution_profile

# The hand-bus default is DERIVED from the one declared topology, never typed in
# here. It used to default to "shared" while the hardware had four buses, so a
# full bring-up quiesced the arm for every hand motion that did not need it.
try:
    from agx_arm_ctrl.motion_registry import handshake_required as _handshake_required

    _DEFAULT_HAND_BUS = "shared" if _handshake_required() else "dedicated"
except Exception:  # registry unreadable: take the degraded, safe reading
    _DEFAULT_HAND_BUS = "shared"


os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _resolved_moveit_profile(context) -> str:
    profile_values = resolve_execution_profile(
        LaunchConfiguration("execution_profile").perform(context).strip()
    )
    return profile_values.get("moveit_profile") or LaunchConfiguration("moveit_profile").perform(context).strip()


def _validate_mode_contract(context):
    mode = LaunchConfiguration("mode").perform(context).strip()
    moveit_profile = _resolved_moveit_profile(context)

    if mode == "manual_vendor" and moveit_profile == "both_arms":
        raise ValueError(
            "mode 'manual_vendor' remains a one-driver-per-arm surface; "
            "use mode:=moveit_mit or mode:=debug_soft_target for the current Duo both_arms bringup"
        )

    return []


def _default_mit_params_file() -> str:
    package_share_dir = Path(get_package_share_directory("agx_arm_mit_controller")).resolve()
    installed_params_file = package_share_dir / "config" / "nero_mit_controller_defaults.yaml"

    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return str(installed_params_file)

    source_params_file = workspace_root / "src" / "agx_arm_mit_controller" / "config" / "nero_mit_controller_defaults.yaml"
    if source_params_file.is_file():
        return str(source_params_file)
    return str(installed_params_file)


def generate_launch_description():
    default_mit_params_file = _default_mit_params_file()

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal).",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="moveit_mit",
        choices=["manual_vendor", "debug_soft_target", "moveit_mit"],
        description="Common agx_arm_ctrl bringup mode.",
    )
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="",
        description="CAN port used by the wrapped AGX Arm node. Empty resolves the side bus from the execution profile (registry arm.sides.*.can_port), falling back to can_nero_right. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.",
    )
    execution_profile_arg = DeclareLaunchArgument(
        "execution_profile",
        default_value="manual",
        choices=["manual", "standalone", "left_arm", "left_hand", "left_gripper", "right_arm", "right_hand", "right_gripper", "duo_arm", "duo_gripper", "duo_hand", "duo_hand_external_bridge"],
        description=(
            "Repo-owned execution preset for the wrapped debug and MoveIt launches. "
            "This is the normal source of truth for Duo slice selection: the preset resolves the mounted "
            "model, arm/hand composition, prefixes/frames, and the custom_model/custom_model_xacro_args "
            "that the downstream MIT launch uses to derive the gravity URDF. Prefer updating the preset "
            "configuration over reconstructing the same slice through ad hoc per-command overrides."
        ),
    )
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="nero",
        choices=["nero"],
        description="Robotic arm type.",
    )
    moveit_profile_arg = DeclareLaunchArgument(
        "moveit_profile",
        default_value="nero_arm",
        choices=["nero_arm", "right_arm", "left_arm", "both_arms"],
        description="MoveIt planning profile. Use both_arms with arm_instances for staged Duo multi-arm bringup.",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Optional SRDF robot name override for custom models. Empty defaults to duo_nero_system when custom_model is set, otherwise agx_arm.",
    )
    custom_model_arg = DeclareLaunchArgument(
        "custom_model",
        default_value="",
        description="Optional custom model path forwarded to the debug and MoveIt wrapper launches.",
    )
    custom_model_xacro_args_arg = DeclareLaunchArgument(
        "custom_model_xacro_args",
        default_value="",
        description="Optional extra xacro args appended when custom_model is set.",
    )
    arm_instances_arg = DeclareLaunchArgument(
        "arm_instances",
        default_value="",
        description="Optional YAML list describing managed arm runtime instances for the MoveIt bringup wrapper.",
    )
    effector_type_arg = DeclareLaunchArgument(
        "effector_type",
        default_value="none",
        choices=["none", "agx_gripper", "revo2", "omnihand"],
        description="End effector type.",
    )
    revo2_type_arg = DeclareLaunchArgument(
        "revo2_type",
        default_value="left",
        choices=["left", "right"],
        description="Revo2 end effector type.",
    )
    omnihand_type_arg = DeclareLaunchArgument(
        "omnihand_type",
        default_value="left",
        choices=["left", "right"],
        description="OmniHand type.",
    )
    # The unit's one safety-generation writer. Nothing may command until it has
    # spoken: the coordinator is fail-closed on an unestablished generation.
    # Exactly one per unit — turn this off only when another launch starts it.
    start_unit_safety_arg = DeclareLaunchArgument(
        "start_unit_safety",
        default_value="true",
        choices=["true", "false"],
        description=(
            "Start the unit safety generation writer (agx_arm_ctrl unit_safety). "
            "Exactly one must run per unit; the coordinator refuses new activities "
            "while no generation is established. Set false only if another launch "
            "already starts it."
        ),
    )
    launch_omnihand_bridge_arg = DeclareLaunchArgument(
        "launch_omnihand_bridge",
        default_value="false",
        choices=["true", "false"],
        description="Launch the repo-owned OmniHand bridge when effector_type is omnihand.",
    )

    hand_bus_arg = DeclareLaunchArgument(
        "hand_bus",
        default_value=_DEFAULT_HAND_BUS,
        choices=["shared", "dedicated"],
        description=(
            "OmniHand CAN topology. 'shared': the hand shares the arm side bus, "
            "so hand execution goes through the arm<->hand window handshake "
            "(quiesce arm, silence its feedback push, command hand, resume). "
            "'dedicated': the hand has its own bus (e.g. a second USB-CAN "
            "adapter) — the handshake is turned off and arm MIT runs in parallel "
            "with the hand. Only select 'dedicated' with a real second bus."
        ),
    )
    omnihand_backend_type_arg = DeclareLaunchArgument(
        "omnihand_backend_type",
        default_value="mock",
        description="Backend type for the repo-owned OmniHand bridge.",
    )
    omnihand_device_id_arg = DeclareLaunchArgument(
        "omnihand_device_id",
        default_value="1",
        description="Vendor SDK device_id used when omnihand_backend_type is sdk.",
    )
    omnihand_canfd_id_arg = DeclareLaunchArgument(
        "omnihand_canfd_id",
        default_value="0",
        description="Vendor SDK canfd_id used when omnihand_backend_type is sdk.",
    )
    omnihand_sdk_cfg_path_arg = DeclareLaunchArgument(
        "omnihand_sdk_cfg_path",
        default_value="",
        description="Optional vendor SDK config path used when omnihand_backend_type is sdk.",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        choices=["true", "false"],
        description="Automatically enable the AGX Arm node.",
    )
    fast_mode_arg = DeclareLaunchArgument(
        "fast_mode",
        default_value="false",
        choices=["true", "false"],
        description="Enable fast mode for the AGX Arm node.",
    )
    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="100",
        description="Movement speed as a percentage of maximum speed.",
    )
    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="200",
        description="Publishing rate for the AGX Arm node.",
    )
    runtime_metrics_enabled_arg = DeclareLaunchArgument(
        "runtime_metrics_enabled",
        default_value="false",
        description=(
            "Log per-thread SDK call counts and durations in the arm drivers and "
            "the hand bridges. Off by default because it costs CPU on the Jetson. "
            "It is how both the one-owner claim and the hand bridge's SDK share "
            "are read, so it has to be reachable from the supported bring-up."
        ),
    )
    enable_timeout_arg = DeclareLaunchArgument(
        "enable_timeout",
        default_value="5.0",
        description="Timeout in seconds for arm enable/disable operations.",
    )
    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description="TCP offset in x, y, z, roll, pitch, yaw in meters/radians.",
    )
    input_joint_prefix_arg = DeclareLaunchArgument(
        "input_joint_prefix",
        default_value="",
        description="Optional prefix stripped from prefixed custom-model joint names before controller-side processing.",
    )
    feedback_joint_prefix_arg = DeclareLaunchArgument(
        "feedback_joint_prefix",
        default_value="",
        description="Optional prefix added onto follow-side feedback/joint_states for prefixed custom models. Empty falls back to input_joint_prefix.",
    )
    follow_joint_states_topic_arg = DeclareLaunchArgument(
        "follow_joint_states_topic",
        default_value="feedback/joint_states",
        description="JointState topic consumed when follow:=true. Override for prefixed custom-model feedback adaptation.",
    )
    arm_base_frame_arg = DeclareLaunchArgument(
        "arm_base_frame",
        default_value="",
        description="Optional arm base frame used by MoveIt for custom body-mounted models.",
    )
    arm_tip_frame_arg = DeclareLaunchArgument(
        "arm_tip_frame",
        default_value="",
        description="Optional arm tip frame used by MoveIt for custom body-mounted models.",
    )
    tcp_parent_frame_arg = DeclareLaunchArgument(
        "tcp_parent_frame",
        default_value="",
        description="Optional parent frame for the tcp_offset static transform in debug RViz mode.",
    )
    gripper_default_effort_arg = DeclareLaunchArgument(
        "gripper_default_effort",
        default_value="1.0",
        description="Default effort for gripper commands (>= 0.0).",
    )
    publish_gripper_joint_arg = DeclareLaunchArgument(
        "publish_gripper_joint",
        default_value="true",
        choices=["true", "false"],
        description="Publish the synthetic gripper opening joint in feedback/joint_states.",
    )
    follow_arg = DeclareLaunchArgument(
        "follow",
        default_value="true",
        choices=["true", "false"],
        description="Follow real arm state for visualization and MoveIt.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz as part of the wrapped MoveIt bringup.",
    )
    db_arg = DeclareLaunchArgument(
        "db",
        default_value="false",
        choices=["true", "false"],
        description="Launch the MoveIt warehouse database in the wrapped MoveIt bringup.",
    )
    mit_control_rate_arg = DeclareLaunchArgument(
        "mit_control_rate_hz",
        default_value="200.0",
        description="MIT controller update rate when MIT mode is enabled.",
    )
    mit_params_file_arg = DeclareLaunchArgument(
        "mit_params_file",
        default_value=default_mit_params_file,
        description="Optional MIT controller params file override.",
    )
    # A carried object the arm can pick up (moveit_mit mode only). >0 preloads a
    # second gravity model per arm; which arm is carrying is decided at runtime
    # by the coordinator through the per-side ~/payload_attached service.
    payload_mass_kg_arg = DeclareLaunchArgument(
        "payload_mass_kg",
        default_value="0.0",
        description="Mass in kg of a payload the arm can pick up. 0 = no payload model; an attach request is then refused.",
    )
    payload_com_xyz_arg = DeclareLaunchArgument(
        "payload_com_xyz",
        default_value="[0.15, 0.0, 0.0]",
        description="Payload centre of mass in the flange frame [x, y, z] in m. The hand reaches along the flange's +x, so a tool-axis offset goes in x.",
    )
    payload_cylinder_radius_m_arg = DeclareLaunchArgument(
        "payload_cylinder_radius_m",
        default_value="0.06",
        description="Cylinder radius for the payload inertia tensor (gravity ignores it; carried for completeness).",
    )
    payload_cylinder_height_m_arg = DeclareLaunchArgument(
        "payload_cylinder_height_m",
        default_value="0.15",
        description="Cylinder height for the payload inertia tensor.",
    )
    payload_parent_link_arg = DeclareLaunchArgument(
        "payload_parent_link",
        default_value="",
        description="Link the payload is fixed to. Empty resolves the arm's '*nero_tool0' flange link from the gravity URDF.",
    )
    mit_joint_target_duration_arg = DeclareLaunchArgument(
        "mit_joint_target_duration_s",
        default_value="0.75",
        description="Duration in seconds for RViz joint-slider soft MIT targets.",
    )
    load_simple_obstacles_arg = DeclareLaunchArgument(
        "load_simple_obstacles",
        default_value="false",
        choices=["true", "false"],
        description="Seed the MoveIt planning scene with the repo-owned simple obstacle set.",
    )
    planning_pipelines_arg = DeclareLaunchArgument(
        "planning_pipelines",
        default_value="",
        description="Optional comma-separated planning pipeline whitelist forwarded into move_group.",
    )
    simple_obstacles_config_arg = DeclareLaunchArgument(
        "simple_obstacles_config",
        default_value=os.path.join(
            get_package_share_directory("agx_arm_moveit"),
            "config",
            "simple_obstacles.json",
        ),
        description="Path to the JSON file with simple planning-scene obstacles.",
    )

    mode_is_moveit = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'moveit_mit'"])
    mode_is_debug = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'debug_soft_target'"])
    debug_is_multi_arm = PythonExpression([
        "('", LaunchConfiguration("mode"),
        "' == 'debug_soft_target') and (('", LaunchConfiguration("execution_profile"),
        "' == 'duo_arm') or ('", LaunchConfiguration("moveit_profile"),
        "' == 'both_arms'))",
    ])
    debug_is_single_arm = PythonExpression([
        "('", LaunchConfiguration("mode"),
        "' == 'debug_soft_target') and not (('", LaunchConfiguration("execution_profile"),
        "' == 'duo_arm') or ('", LaunchConfiguration("moveit_profile"),
        "' == 'both_arms'))",
    ])

    manual_vendor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm.launch.py",
            )
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "namespace": LaunchConfiguration("namespace"),
            "can_port": LaunchConfiguration("can_port"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "runtime_metrics_enabled": LaunchConfiguration("runtime_metrics_enabled"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "arm_type": LaunchConfiguration("arm_type"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "effector_type": LaunchConfiguration("effector_type"),
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id"),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id"),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
            "publish_gripper_joint": PythonExpression([
                "'false' if '", LaunchConfiguration("mode"), "' == 'moveit_mit' else '",
                LaunchConfiguration("publish_gripper_joint"), "'",
            ]),
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual_vendor'"])),
    )

    debug_soft_target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "namespace": LaunchConfiguration("namespace"),
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": LaunchConfiguration("arm_type"),
            "execution_profile": LaunchConfiguration("execution_profile"),
            "custom_model": LaunchConfiguration("custom_model"),
            "custom_model_xacro_args": LaunchConfiguration("custom_model_xacro_args"),
            "effector_type": LaunchConfiguration("effector_type"),
            "revo2_type": LaunchConfiguration("revo2_type"),
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id"),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id"),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "runtime_metrics_enabled": LaunchConfiguration("runtime_metrics_enabled"),
            "follow": LaunchConfiguration("follow"),
            "control": "true",
            "use_mit_controller": "true",
            "mit_control_rate_hz": LaunchConfiguration("mit_control_rate_hz"),
            "mit_params_file": LaunchConfiguration("mit_params_file"),
            "mit_joint_target_duration_s": LaunchConfiguration("mit_joint_target_duration_s"),
            "input_joint_prefix": LaunchConfiguration("input_joint_prefix"),
            "feedback_joint_prefix": LaunchConfiguration("feedback_joint_prefix"),
            "follow_joint_states_topic": LaunchConfiguration("follow_joint_states_topic"),
            "tcp_parent_frame": LaunchConfiguration("tcp_parent_frame"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
        }.items(),
        condition=IfCondition(debug_is_single_arm),
    )

    debug_soft_target_multi_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_multi_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "namespace": LaunchConfiguration("namespace"),
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": LaunchConfiguration("arm_type"),
            "execution_profile": LaunchConfiguration("execution_profile"),
            "moveit_profile": LaunchConfiguration("moveit_profile"),
            "custom_model": LaunchConfiguration("custom_model"),
            "custom_model_xacro_args": LaunchConfiguration("custom_model_xacro_args"),
            "arm_instances": LaunchConfiguration("arm_instances"),
            "effector_type": LaunchConfiguration("effector_type"),
            "revo2_type": LaunchConfiguration("revo2_type"),
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "hand_bus": LaunchConfiguration("hand_bus"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id"),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id"),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "runtime_metrics_enabled": LaunchConfiguration("runtime_metrics_enabled"),
            "follow": LaunchConfiguration("follow"),
            "use_mit_controller": "true",
            "mit_control_rate_hz": LaunchConfiguration("mit_control_rate_hz"),
            "mit_params_file": LaunchConfiguration("mit_params_file"),
            "mit_joint_target_duration_s": LaunchConfiguration("mit_joint_target_duration_s"),
            "input_joint_prefix": LaunchConfiguration("input_joint_prefix"),
            "feedback_joint_prefix": LaunchConfiguration("feedback_joint_prefix"),
            "follow_joint_states_topic": LaunchConfiguration("follow_joint_states_topic"),
            "tcp_parent_frame": LaunchConfiguration("tcp_parent_frame"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
        }.items(),
        condition=IfCondition(debug_is_multi_arm),
    )

    moveit_mit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_agx_arm_moveit.launch.py",
            )
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "namespace": LaunchConfiguration("namespace"),
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": LaunchConfiguration("arm_type"),
            "execution_profile": LaunchConfiguration("execution_profile"),
            "moveit_profile": LaunchConfiguration("moveit_profile"),
            "robot_name": LaunchConfiguration("robot_name"),
            "custom_model": LaunchConfiguration("custom_model"),
            "custom_model_xacro_args": LaunchConfiguration("custom_model_xacro_args"),
            "arm_instances": LaunchConfiguration("arm_instances"),
            "effector_type": LaunchConfiguration("effector_type"),
            "revo2_type": LaunchConfiguration("revo2_type"),
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "hand_bus": LaunchConfiguration("hand_bus"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id"),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id"),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "runtime_metrics_enabled": LaunchConfiguration("runtime_metrics_enabled"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "input_joint_prefix": LaunchConfiguration("input_joint_prefix"),
            "feedback_joint_prefix": LaunchConfiguration("feedback_joint_prefix"),
            "follow_joint_states_topic": LaunchConfiguration("follow_joint_states_topic"),
            "arm_base_frame": LaunchConfiguration("arm_base_frame"),
            "arm_tip_frame": LaunchConfiguration("arm_tip_frame"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
            "follow": LaunchConfiguration("follow"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "db": LaunchConfiguration("db"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "use_mit_controller": "true",
            "mit_control_rate_hz": LaunchConfiguration("mit_control_rate_hz"),
            "mit_params_file": LaunchConfiguration("mit_params_file"),
            "payload_mass_kg": LaunchConfiguration("payload_mass_kg"),
            "payload_com_xyz": LaunchConfiguration("payload_com_xyz"),
            "payload_cylinder_radius_m": LaunchConfiguration("payload_cylinder_radius_m"),
            "payload_cylinder_height_m": LaunchConfiguration("payload_cylinder_height_m"),
            "payload_parent_link": LaunchConfiguration("payload_parent_link"),
            "planning_pipelines": LaunchConfiguration("planning_pipelines"),
            "load_simple_obstacles": LaunchConfiguration("load_simple_obstacles"),
            "simple_obstacles_config": LaunchConfiguration("simple_obstacles_config"),
        }.items(),
        condition=IfCondition(mode_is_moveit),
    )

    # No namespace: the writer publishes the relative topic `unit_safety` and
    # every observer subscribes to absolute `/unit_safety`, so it has to sit at
    # the root to be heard at all.
    unit_safety_node = Node(
        package="agx_arm_ctrl",
        executable="unit_safety",
        name="unit_safety",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("start_unit_safety")),
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        mode_arg,
        can_port_arg,
        execution_profile_arg,
        arm_type_arg,
        moveit_profile_arg,
        robot_name_arg,
        custom_model_arg,
        custom_model_xacro_args_arg,
        arm_instances_arg,
        effector_type_arg,
        revo2_type_arg,
        omnihand_type_arg,
        start_unit_safety_arg,
        launch_omnihand_bridge_arg,
        hand_bus_arg,
        omnihand_backend_type_arg,
        omnihand_device_id_arg,
        omnihand_canfd_id_arg,
        omnihand_sdk_cfg_path_arg,
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        runtime_metrics_enabled_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        input_joint_prefix_arg,
        feedback_joint_prefix_arg,
        follow_joint_states_topic_arg,
        arm_base_frame_arg,
        arm_tip_frame_arg,
        tcp_parent_frame_arg,
        gripper_default_effort_arg,
        publish_gripper_joint_arg,
        follow_arg,
        use_rviz_arg,
        db_arg,
        mit_control_rate_arg,
        mit_params_file_arg,
        payload_mass_kg_arg,
        payload_com_xyz_arg,
        payload_cylinder_radius_m_arg,
        payload_cylinder_height_m_arg,
        payload_parent_link_arg,
        mit_joint_target_duration_arg,
        load_simple_obstacles_arg,
        planning_pipelines_arg,
        simple_obstacles_config_arg,
        OpaqueFunction(function=_validate_mode_contract),
        unit_safety_node,
        manual_vendor_launch,
        debug_soft_target_launch,
        debug_soft_target_multi_arm_launch,
        moveit_mit_launch,
    ])