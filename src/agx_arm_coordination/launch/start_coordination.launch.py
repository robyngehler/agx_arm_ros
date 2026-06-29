"""Bring up the activity-DAG coordinator alone.

Assumes the hand skill controllers (``/left_hand/perform``, ``/right_hand/perform``)
and the arm controllers already exist. Use ``arm_dry_run:=true`` to exercise
scheduling/routing without moving the arms (e.g. before anchor poses / recorded
trajectories are taught on hardware).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument("log_level", default_value="info")
    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Override the activity/catalogue config dir (empty => package share).",
    )
    arm_dry_run_arg = DeclareLaunchArgument(
        "arm_dry_run",
        default_value="false",
        description="If true, arm trajectories are logged but not sent (scheduling-only test).",
    )
    hand_action_template_arg = DeclareLaunchArgument(
        "hand_action_template",
        default_value="/{side}_hand/perform",
        description="PerformAction server name template for the hand skill controllers.",
    )

    parameters = {
        "arm_dry_run": ParameterValue(LaunchConfiguration("arm_dry_run"), value_type=bool),
        "hand_action_template": LaunchConfiguration("hand_action_template"),
    }
    # Only set config_dir when non-empty so the node default (package share) wins.
    config_dir = LaunchConfiguration("config_dir")

    coordinator = Node(
        package="agx_arm_coordination",
        executable="coordinator",
        name="agx_arm_coordinator",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[parameters, {"config_dir": config_dir}],
    )

    return LaunchDescription([
        log_level_arg,
        config_dir_arg,
        arm_dry_run_arg,
        hand_action_template_arg,
        coordinator,
    ])
