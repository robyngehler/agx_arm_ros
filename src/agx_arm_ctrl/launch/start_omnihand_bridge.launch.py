from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the OmniHand bridge node.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for this OmniHand bridge instance.",
    )
    omnihand_type_arg = DeclareLaunchArgument(
        "omnihand_type",
        default_value="left",
        description="OmniHand side (left or right).",
    )
    backend_type_arg = DeclareLaunchArgument(
        "backend_type",
        default_value="mock",
        description="Backend type. Supported values are mock and sdk.",
    )
    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="1",
        description="Vendor SDK device_id used when backend_type is sdk.",
    )
    canfd_id_arg = DeclareLaunchArgument(
        "canfd_id",
        default_value="0",
        description="Vendor SDK canfd_id used when backend_type is sdk when supported by the local SDK build.",
    )
    sdk_cfg_path_arg = DeclareLaunchArgument(
        "sdk_cfg_path",
        default_value="",
        description="Optional vendor SDK config path used when backend_type is sdk.",
    )
    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="50.0",
        description="Feedback publish rate in Hz.",
    )
    joint_states_command_topic_arg = DeclareLaunchArgument(
        "joint_states_command_topic",
        default_value="control/joint_states",
        description="Shared JointState command topic consumed by the OmniHand bridge.",
    )
    tactile_sample_count_arg = DeclareLaunchArgument(
        "tactile_sample_count",
        default_value="32",
        description="Mock tactile sample count.",
    )

    omnihand_bridge = Node(
        package="agx_arm_ctrl",
        executable="omnihand_bridge",
        name="omnihand_bridge_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "backend_type": LaunchConfiguration("backend_type"),
            "device_id": ParameterValue(LaunchConfiguration("device_id"), value_type=int),
            "canfd_id": ParameterValue(LaunchConfiguration("canfd_id"), value_type=int),
            "sdk_cfg_path": LaunchConfiguration("sdk_cfg_path"),
            "pub_rate": ParameterValue(LaunchConfiguration("pub_rate"), value_type=float),
            "joint_states_command_topic": LaunchConfiguration("joint_states_command_topic"),
            "tactile_sample_count": ParameterValue(LaunchConfiguration("tactile_sample_count"), value_type=int),
        }],
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        omnihand_type_arg,
        backend_type_arg,
        device_id_arg,
        canfd_id_arg,
        sdk_cfg_path_arg,
        pub_rate_arg,
        joint_states_command_topic_arg,
        tactile_sample_count_arg,
        omnihand_bridge,
    ])