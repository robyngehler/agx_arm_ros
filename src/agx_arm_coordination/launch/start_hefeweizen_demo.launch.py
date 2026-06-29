"""Full hand+coordination bring-up for the Hefeweizen demo.

Starts, per side, the OmniHand bridge and the skill controller (namespaced
``left_hand`` / ``right_hand`` so the skill action lands at ``/left_hand/perform``
and ``/right_hand/perform``), then the coordinator. The ARM controllers are NOT
started here — bring up the Duo arm/MoveIt stack separately, or pass
``arm_dry_run:=true`` to exercise the coordinator without arms.

Examples::

    # mock hands, arms dry-run: validate the coordinator end to end on a laptop
    ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py arm_dry_run:=true

    # real hands on the Jetson side buses
    ros2 launch agx_arm_coordination start_hefeweizen_demo.launch.py backend_type:=sdk
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _hand_side(side: str, ctrl_launch_dir: str):
    namespace = f"{side}_hand"
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_launch_dir, "start_omnihand_bridge.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "omnihand_type": side,
            "hand_model": LaunchConfiguration("hand_model"),
            "backend_type": LaunchConfiguration("backend_type"),
        }.items(),
    )
    skill = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_launch_dir, "start_omnihand_skill_controller.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "omnihand_type": side,
            "hand_model": LaunchConfiguration("hand_model"),
        }.items(),
    )
    return [bridge, skill]


def generate_launch_description():
    ctrl_launch_dir = os.path.join(get_package_share_directory("agx_arm_ctrl"), "launch")

    hand_model_arg = DeclareLaunchArgument("hand_model", default_value="o12_pro")
    backend_type_arg = DeclareLaunchArgument(
        "backend_type", default_value="mock",
        description="OmniHand backend: mock or sdk.",
    )
    arm_dry_run_arg = DeclareLaunchArgument(
        "arm_dry_run", default_value="false",
        description="If true, arm trajectories are logged but not sent.",
    )

    coordinator = Node(
        package="agx_arm_coordination",
        executable="coordinator",
        name="agx_arm_coordinator",
        output="screen",
        parameters=[{
            "arm_dry_run": ParameterValue(LaunchConfiguration("arm_dry_run"), value_type=bool),
        }],
    )

    nodes = [hand_model_arg, backend_type_arg, arm_dry_run_arg]
    nodes += _hand_side("left", ctrl_launch_dir)
    nodes += _hand_side("right", ctrl_launch_dir)
    nodes.append(coordinator)
    return LaunchDescription(nodes)
