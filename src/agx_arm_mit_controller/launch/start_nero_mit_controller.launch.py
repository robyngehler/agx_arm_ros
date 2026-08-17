import ast
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agx_arm_mit_controller.gravity_launch_utils import resolve_gravity_urdf_path


os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _default_params_file() -> str:
    package_share_dir = Path(get_package_share_directory("agx_arm_mit_controller")).resolve()
    installed_params_file = package_share_dir / "config" / "nero_mit_controller_defaults.yaml"

    # In a colcon workspace, prefer the source YAML so gain tweaks do not require a rebuild.
    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return str(installed_params_file)

    source_params_file = workspace_root / "src" / "agx_arm_mit_controller" / "config" / "nero_mit_controller_defaults.yaml"
    if source_params_file.is_file():
        return str(source_params_file)
    return str(installed_params_file)


def _duo_system_xacro_path() -> str:
    """Locate duo_system.urdf.xacro (source tree first, share dir fallback).

    Like the params file, prefer the source tree in a colcon workspace: a stale
    installed copy silently builds the gravity model against outdated assets
    (e.g. the legacy O10 hand instead of the O12 Pro — ~0.6 kg mass error).
    """
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "duo_body_description" / "urdf" / "duo_system.urdf.xacro"
        if candidate.is_file():
            return str(candidate)
    try:
        share = Path(get_package_share_directory("duo_body_description"))
        candidate = share / "urdf" / "duo_system.urdf.xacro"
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass
    return ""


def _expected_device_id(can_port: str) -> str:
    """Name the arm this controller is allowed to be gated by.

    Mirrors `derive_device_id` in `agx_arm_ctrl.agx_arm_ctrl_single_node`, which
    is the source of truth. It is copied rather than imported on purpose:
    importing that module pulls the vendor SDK and scipy into the launch
    process, which is a heavy and needless coupling for one string rule. Keep
    the two in step — they read the same `can_port`.
    """
    port = str(can_port or "").strip().lower()
    for side in ("left", "right"):
        if port.endswith(side):
            return f"arm_{side}"
    return f"arm_{port}" if port else "arm_unknown"


def _build_controller_node(context):
    # gravity_arm_side bakes the arm's real body mount into the gravity URDF
    # (ground truth from the description), so gravity compensation is correct for
    # the tilted body-mounted arm without a hand-typed gravity_mounting_rpy.
    gravity_arm_side = LaunchConfiguration("gravity_arm_side").perform(context).strip()
    expected_device_id = _expected_device_id(
        LaunchConfiguration("can_port").perform(context)
    )
    custom_model = LaunchConfiguration("custom_model").perform(context).strip()
    if gravity_arm_side in ("left", "right") and not custom_model:
        custom_model = _duo_system_xacro_path()

    resolved_gravity_urdf_path = resolve_gravity_urdf_path(
        custom_model=custom_model,
        custom_model_xacro_args=LaunchConfiguration("custom_model_xacro_args").perform(context),
        input_joint_prefix=LaunchConfiguration("input_joint_prefix").perform(context),
        effector_type=LaunchConfiguration("effector_type").perform(context),
        explicit_gravity_urdf_path=LaunchConfiguration("gravity_urdf_path").perform(context),
        duo_side=gravity_arm_side,
        hand_payload_mode=LaunchConfiguration("gravity_hand_payload").perform(context).strip(),
    )

    mounting_rpy_str = LaunchConfiguration("gravity_mounting_rpy").perform(context)
    try:
        gravity_mounting_rpy = [float(value) for value in ast.literal_eval(mounting_rpy_str)]
        if len(gravity_mounting_rpy) != 3:
            raise ValueError
    except Exception:
        gravity_mounting_rpy = [0.0, 0.0, 0.0]

    payload_com_str = LaunchConfiguration("payload_com_xyz").perform(context)
    try:
        payload_com_xyz = [float(value) for value in ast.literal_eval(payload_com_str)]
        if len(payload_com_xyz) != 3:
            raise ValueError
    except Exception:
        payload_com_xyz = [0.15, 0.0, 0.0]

    payload_mass_kg = float(LaunchConfiguration("payload_mass_kg").perform(context))
    payload_cylinder_radius_m = float(
        LaunchConfiguration("payload_cylinder_radius_m").perform(context)
    )
    payload_cylinder_height_m = float(
        LaunchConfiguration("payload_cylinder_height_m").perform(context)
    )

    return [
        LogInfo(msg=["MIT controller params_file: ", LaunchConfiguration("params_file")]),
        LogInfo(
            msg=(
                f"MIT controller gravity_urdf_path: {resolved_gravity_urdf_path}"
                if resolved_gravity_urdf_path
                else "MIT controller gravity_urdf_path: <auto>"
            )
        ),
        Node(
            package="agx_arm_mit_controller",
            executable="agx_arm_mit_controller",
            name="mit_controller",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                    "input_joint_prefix": LaunchConfiguration("input_joint_prefix"),
                    "gravity_urdf_path": resolved_gravity_urdf_path,
                    "gravity_mounting_rpy": gravity_mounting_rpy,
                    "payload_mass_kg": payload_mass_kg,
                    "payload_com_xyz": payload_com_xyz,
                    "payload_cylinder_radius_m": payload_cylinder_radius_m,
                    "payload_cylinder_height_m": payload_cylinder_height_m,
                    "payload_parent_link": LaunchConfiguration("payload_parent_link"),
                    "enable_debug_joint_trajectory_topic": LaunchConfiguration(
                        "enable_debug_joint_trajectory_topic"
                    ),
                    # Derived from the same CAN port the driver uses, so the
                    # controller is gated by *its own* arm's authority. With two
                    # arms publishing, being gated by the wrong one would report
                    # ready while the commanded device is stopped.
                    "expected_device_id": expected_device_id,
                },
            ],
        ),
    ]


def generate_launch_description():
    default_params_file = _default_params_file()

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.",
    )
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can_nero_right",
        description="CAN interface used by the underlying agx_arm_ctrl driver. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.",
    )
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="nero",
        description="Robotic arm type forwarded to agx_arm_ctrl.",
    )
    effector_type_arg = DeclareLaunchArgument(
        "effector_type",
        default_value="none",
        description="End-effector type forwarded to agx_arm_ctrl.",
    )
    omnihand_type_arg = DeclareLaunchArgument(
        "omnihand_type",
        default_value="left",
        description="OmniHand side forwarded to agx_arm_ctrl when applicable.",
    )
    omnihand_joint_states_topic_arg = DeclareLaunchArgument(
        "omnihand_joint_states_topic",
        default_value="feedback/omnihand/joint_states",
        description="Topic the arm driver reads OmniHand joint states from. Absolute when another launch owns the bridge.",
    )
    launch_omnihand_bridge_arg = DeclareLaunchArgument(
        "launch_omnihand_bridge",
        default_value="false",
        description="Launch the repo-owned OmniHand bridge when effector_type is omnihand.",
    )
    hand_bus_arg = DeclareLaunchArgument(
        "hand_bus",
        default_value="shared",
        choices=["shared", "dedicated"],
        description="shared: keep the arm<->hand window handshake; dedicated: "
        "turn it off for parallel arm+hand operation on a second bus.",
    )
    omnihand_backend_type_arg = DeclareLaunchArgument(
        "omnihand_backend_type",
        default_value="mock",
        description="Backend type for the repo-owned OmniHand bridge.",
    )
    omnihand_device_id_arg = DeclareLaunchArgument(
        "omnihand_device_id",
        default_value="1",
        description="Vendor SDK device_id forwarded to agx_arm_ctrl when omnihand_backend_type is sdk.",
    )
    omnihand_canfd_id_arg = DeclareLaunchArgument(
        "omnihand_canfd_id",
        default_value="0",
        description="Vendor SDK canfd_id forwarded to agx_arm_ctrl when omnihand_backend_type is sdk.",
    )
    omnihand_sdk_cfg_path_arg = DeclareLaunchArgument(
        "omnihand_sdk_cfg_path",
        default_value="",
        description="Optional vendor SDK config path forwarded to agx_arm_ctrl when omnihand_backend_type is sdk.",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        description="Automatically enable the AGX arm driver on startup.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the MIT controller node.",
    )
    fast_mode_arg = DeclareLaunchArgument(
        "fast_mode",
        default_value="false",
        description="Forward fast-mode control to agx_arm_ctrl.",
    )
    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="100",
        description="Motion speed percentage forwarded to agx_arm_ctrl.",
    )
    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="200",
        description="Feedback publish rate forwarded to agx_arm_ctrl.",
    )
    runtime_metrics_enabled_arg = DeclareLaunchArgument(
        "runtime_metrics_enabled",
        default_value="false",
        description=(
            "Forwarded to agx_arm_ctrl: log loop, callback and per-thread SDK "
            "call counters. This is the bring-up the 'one SDK owner per device' "
            "claim is measured on, so the switch has to reach it from here."
        ),
    )
    enable_timeout_arg = DeclareLaunchArgument(
        "enable_timeout",
        default_value="5.0",
        description="Enable timeout in seconds forwarded to agx_arm_ctrl.",
    )
    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description="TCP offset [x, y, z, rx, ry, rz] forwarded to agx_arm_ctrl.",
    )
    gripper_default_effort_arg = DeclareLaunchArgument(
        "gripper_default_effort",
        default_value="1.0",
        description="Default gripper effort forwarded to agx_arm_ctrl.",
    )
    publish_gripper_joint_arg = DeclareLaunchArgument(
        "publish_gripper_joint",
        default_value="true",
        description="Whether agx_arm_ctrl should publish the synthetic gripper joint.",
    )
    joint_states_command_topic_arg = DeclareLaunchArgument(
        "joint_states_command_topic",
        default_value="control/joint_states",
        description="JointState command topic consumed by the OmniHand bridge when the driver is launched.",
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="100.0",
        description="MIT controller update rate in hertz.",
    )
    custom_model_arg = DeclareLaunchArgument(
        "custom_model",
        default_value="",
        description="Optional custom model path used to derive a gravity URDF for mounted-arm slices.",
    )
    custom_model_xacro_args_arg = DeclareLaunchArgument(
        "custom_model_xacro_args",
        default_value="",
        description="Optional xacro argument string forwarded when custom_model is set.",
    )
    input_joint_prefix_arg = DeclareLaunchArgument(
        "input_joint_prefix",
        default_value="",
        description="Optional prefix stripped from incoming trajectory joint names before MIT validation.",
    )
    gravity_urdf_path_arg = DeclareLaunchArgument(
        "gravity_urdf_path",
        default_value="",
        description="Optional explicit gravity URDF path override. Empty auto-resolves from custom_model or the canonical Nero URDF.",
    )
    gravity_mounting_rpy_arg = DeclareLaunchArgument(
        "gravity_mounting_rpy",
        default_value="[0.0, 0.0, 0.0]",
        description="Manual arm base orientation in world [roll, pitch, yaw] (XYZ extrinsic, rad) for gravity/freedrive. [0,0,0] = upright. Prefer gravity_arm_side (URDF-derived); use this only as an override, and keep it [0,0,0] when gravity_arm_side is set.",
    )
    payload_mass_kg_arg = DeclareLaunchArgument(
        "payload_mass_kg",
        default_value="0.0",
        description="Mass of a carried object the arm can pick up, in kg. >0 preloads a second gravity model that ~/payload_attached switches to; 0 leaves only the unloaded model and refuses an attach request.",
    )
    payload_com_xyz_arg = DeclareLaunchArgument(
        "payload_com_xyz",
        default_value="[0.15, 0.0, 0.0]",
        description="Carried payload centre of mass in the payload_parent_link frame [x, y, z] in m. On the Duo the hand reaches along the flange's +x (fingertips at x~0.25 m), so a tool-axis offset goes in x, not z.",
    )
    payload_cylinder_radius_m_arg = DeclareLaunchArgument(
        "payload_cylinder_radius_m",
        default_value="0.06",
        description="Cylinder radius used for the payload inertia tensor. Gravity ignores the tensor; it is carried so the payload description stays physically complete.",
    )
    payload_cylinder_height_m_arg = DeclareLaunchArgument(
        "payload_cylinder_height_m",
        default_value="0.15",
        description="Cylinder height used for the payload inertia tensor (see payload_cylinder_radius_m).",
    )
    payload_parent_link_arg = DeclareLaunchArgument(
        "payload_parent_link",
        default_value="",
        description="Link the payload is fixed to. Empty resolves the arm's '*nero_tool0' flange link from the gravity URDF, narrowed by input_joint_prefix.",
    )
    gravity_hand_payload_arg = DeclareLaunchArgument(
        "gravity_hand_payload",
        default_value="articulated",
        description="How the OmniHand rides in the gravity model when effector_type is omnihand: 'articulated' keeps the finger joints movable and tracks live hand joint states (better compensation while grasping), 'static' freezes them at zero (legacy rigid payload). Identical when the hand publishes no feedback.",
    )
    gravity_arm_side_arg = DeclareLaunchArgument(
        "gravity_arm_side",
        default_value="",
        description="left|right to bake that arm's real body mount into the gravity URDF (ground truth from duo_system.urdf.xacro), so gravity compensation is correct for the tilted body-mounted arm. Empty = standalone upright arm.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="MIT controller parameter YAML file.",
    )
    launch_driver_arg = DeclareLaunchArgument(
        "launch_driver",
        default_value="true",
        description="Launch agx_arm_ctrl alongside the MIT controller.",
    )
    enable_debug_joint_trajectory_topic_arg = DeclareLaunchArgument(
        "enable_debug_joint_trajectory_topic",
        default_value="false",
        description="Enable the debug ~/joint_trajectory input on the MIT controller.",
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm.launch.py",
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": LaunchConfiguration("arm_type"),
            "effector_type": LaunchConfiguration("effector_type"),
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "hand_bus": LaunchConfiguration("hand_bus"),
            "omnihand_joint_states_topic": LaunchConfiguration("omnihand_joint_states_topic"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id"),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id"),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "log_level": LaunchConfiguration("log_level"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "runtime_metrics_enabled": LaunchConfiguration("runtime_metrics_enabled"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
            "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
            "joint_states_command_topic": LaunchConfiguration("joint_states_command_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    return LaunchDescription(
        [
            namespace_arg,
            can_port_arg,
            arm_type_arg,
            effector_type_arg,
            omnihand_type_arg,
            launch_omnihand_bridge_arg,
            omnihand_joint_states_topic_arg,
            hand_bus_arg,
            omnihand_backend_type_arg,
            omnihand_device_id_arg,
            omnihand_canfd_id_arg,
            omnihand_sdk_cfg_path_arg,
            auto_enable_arg,
            log_level_arg,
            fast_mode_arg,
            speed_percent_arg,
            pub_rate_arg,
            runtime_metrics_enabled_arg,
            enable_timeout_arg,
            tcp_offset_arg,
            gripper_default_effort_arg,
            publish_gripper_joint_arg,
            joint_states_command_topic_arg,
            control_rate_arg,
            custom_model_arg,
            custom_model_xacro_args_arg,
            input_joint_prefix_arg,
            gravity_urdf_path_arg,
            gravity_mounting_rpy_arg,
            payload_mass_kg_arg,
            payload_com_xyz_arg,
            payload_cylinder_radius_m_arg,
            payload_cylinder_height_m_arg,
            payload_parent_link_arg,
            gravity_hand_payload_arg,
            gravity_arm_side_arg,
            params_file_arg,
            launch_driver_arg,
            enable_debug_joint_trajectory_topic_arg,
            driver_launch,
            OpaqueFunction(function=_build_controller_node),
        ]
    )