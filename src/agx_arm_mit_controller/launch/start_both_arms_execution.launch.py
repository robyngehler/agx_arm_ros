"""Bring up the ``both_arms`` execution seam for the Sprint-6 coordinator.

MoveIt's ``both_arms`` profile is planning-only and there is no single controller
that owns all 14 Duo arm joints. This launch is the runtime execution path the
coordinator's ``both_arms`` trajectories need:

1. a per-arm MIT controller for each side (``start_nero_mit_controller`` with its
   own ``namespace`` / ``can_port`` / ``input_joint_prefix``), each exposing
   ``/<side>_arm/arm_controller/follow_joint_trajectory``;
2. the ``both_arms`` fan-out FJT bridge (``agx_arm_mit_tools``) that owns
   ``both_arms_controller/follow_joint_trajectory`` and splits each combined goal
   to the two per-arm servers.

After this, all three ``agx_arm_coordination/config/arm_config.yaml`` arm groups
have real providers: ``both_arms`` -> the bridge, ``left_arm`` / ``right_arm`` ->
the namespaced per-arm controllers (point the per-arm group ``action_server`` at
``/<side>_arm/arm_controller/follow_joint_trajectory``).

Driver bring-up is owned HERE (per side, ``launch_driver``), not by the planning
``duo_arm`` execution profile — planning and execution stay separate surfaces.

Examples::

    # both arms live on the native side buses + the fan-out bridge
    ros2 launch agx_arm_mit_controller start_both_arms_execution.launch.py

    # controllers + bridge only (drivers already up elsewhere)
    ros2 launch agx_arm_mit_controller start_both_arms_execution.launch.py launch_driver:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arm_side(side: str, mit_launch_dir: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mit_launch_dir, "start_nero_mit_controller.launch.py")),
        launch_arguments={
            "namespace": f"{side}_arm",
            "can_port": LaunchConfiguration(f"{side}_can_port"),
            "input_joint_prefix": f"{side}_arm_",
            "effector_type": LaunchConfiguration(f"{side}_effector_type"),
            "omnihand_type": side,
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge"),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type"),
            "launch_driver": LaunchConfiguration("launch_driver"),
            "auto_enable": LaunchConfiguration("auto_enable"),
        }.items(),
    )


def generate_launch_description():
    mit_launch_dir = os.path.join(get_package_share_directory("agx_arm_mit_controller"), "launch")

    args = [
        DeclareLaunchArgument("left_can_port", default_value="can_nero_left"),
        DeclareLaunchArgument("right_can_port", default_value="can_nero_right"),
        DeclareLaunchArgument(
            "left_effector_type", default_value="omnihand",
            description="Effector on the left arm (omnihand | none | ...).",
        ),
        DeclareLaunchArgument("right_effector_type", default_value="omnihand"),
        DeclareLaunchArgument(
            "launch_omnihand_bridge", default_value="true",
            description="Launch the OmniHand bridge per side when the effector is omnihand.",
        ),
        DeclareLaunchArgument("omnihand_backend_type", default_value="sdk"),
        DeclareLaunchArgument(
            "launch_driver", default_value="true",
            description="Bring up each arm driver here (set false if drivers are already up).",
        ),
        DeclareLaunchArgument("auto_enable", default_value="false"),
        DeclareLaunchArgument(
            "both_arms_action_name",
            default_value="both_arms_controller/follow_joint_trajectory",
            description="Combined action server the coordinator's both_arms group targets.",
        ),
    ]

    both_arms_bridge = Node(
        package="agx_arm_mit_tools",
        executable="agx_arm_both_arms_trajectory_bridge",
        name="both_arms_follow_joint_trajectory",
        output="screen",
        parameters=[{
            "action_name": LaunchConfiguration("both_arms_action_name"),
            "left_action_name": "/left_arm/arm_controller/follow_joint_trajectory",
            "right_action_name": "/right_arm/arm_controller/follow_joint_trajectory",
            "left_joint_prefix": "left_arm_",
            "right_joint_prefix": "right_arm_",
        }],
    )

    return LaunchDescription(
        args
        + [_arm_side("left", mit_launch_dir), _arm_side("right", mit_launch_dir), both_arms_bridge]
    )
