from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        description="Backend type. Sprint 2 currently supports only mock.",
    )
    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="50.0",
        description="Feedback publish rate in Hz.",
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
            "pub_rate": LaunchConfiguration("pub_rate"),
            "tactile_sample_count": LaunchConfiguration("tactile_sample_count"),
        }],
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        omnihand_type_arg,
        backend_type_arg,
        pub_rate_arg,
        tactile_sample_count_arg,
        omnihand_bridge,
    ])