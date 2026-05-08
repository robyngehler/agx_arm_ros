import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("agx_arm_mit_controller"),
        "config",
        "nero_mit_controller_defaults.yaml",
    )

    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    can_port_arg = DeclareLaunchArgument("can_port", default_value="can_nero")
    log_level_arg = DeclareLaunchArgument("log_level", default_value="info")
    control_rate_arg = DeclareLaunchArgument("control_rate_hz", default_value="100.0")
    params_file_arg = DeclareLaunchArgument("params_file", default_value=default_params_file)

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
            "arm_type": "nero",
            "log_level": LaunchConfiguration("log_level"),
            "fast_mode": "true",
        }.items(),
    )

    controller_node = Node(
        package="agx_arm_mit_controller",
        executable="agx_arm_mit_controller",
        name="mit_controller",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            LaunchConfiguration("params_file"),
            {"control_rate_hz": LaunchConfiguration("control_rate_hz")},
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            can_port_arg,
            log_level_arg,
            control_rate_arg,
            params_file_arg,
            driver_launch,
            controller_node,
        ]
    )