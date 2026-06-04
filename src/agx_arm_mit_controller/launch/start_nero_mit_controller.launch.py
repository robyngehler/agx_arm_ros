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


def _build_controller_node(context):
    resolved_gravity_urdf_path = resolve_gravity_urdf_path(
        custom_model=LaunchConfiguration("custom_model").perform(context),
        custom_model_xacro_args=LaunchConfiguration("custom_model_xacro_args").perform(context),
        input_joint_prefix=LaunchConfiguration("input_joint_prefix").perform(context),
        effector_type=LaunchConfiguration("effector_type").perform(context),
        explicit_gravity_urdf_path=LaunchConfiguration("gravity_urdf_path").perform(context),
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
                    "enable_debug_joint_trajectory_topic": LaunchConfiguration(
                        "enable_debug_joint_trajectory_topic"
                    ),
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
        default_value="can_nero",
        description="CAN interface used by the underlying agx_arm_ctrl driver.",
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
    launch_omnihand_bridge_arg = DeclareLaunchArgument(
        "launch_omnihand_bridge",
        default_value="false",
        description="Launch the repo-owned OmniHand bridge when effector_type is omnihand.",
    )
    omnihand_backend_type_arg = DeclareLaunchArgument(
        "omnihand_backend_type",
        default_value="mock",
        description="Backend type for the repo-owned OmniHand bridge.",
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
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "log_level": LaunchConfiguration("log_level"),
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort"),
            "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
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
            omnihand_backend_type_arg,
            auto_enable_arg,
            log_level_arg,
            fast_mode_arg,
            speed_percent_arg,
            pub_rate_arg,
            enable_timeout_arg,
            tcp_offset_arg,
            gripper_default_effort_arg,
            publish_gripper_joint_arg,
            control_rate_arg,
            custom_model_arg,
            custom_model_xacro_args_arg,
            input_joint_prefix_arg,
            gravity_urdf_path_arg,
            params_file_arg,
            launch_driver_arg,
            enable_debug_joint_trajectory_topic_arg,
            driver_launch,
            OpaqueFunction(function=_build_controller_node),
        ]
    )