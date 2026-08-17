"""Bring up one OmniHand skill controller (semantic grasp/release skills).

The controller sits on top of the OmniHand bridge: it commands the hand over the
shared ``control/joint_states`` topic and consumes ``feedback/omnihand/*``, so
the bridge must already be running on the same side / namespace. Compose this
with ``start_omnihand_bridge.launch.py`` under a per-side namespace, e.g.::

    ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
        namespace:=right_hand omnihand_type:=right backend_type:=sdk
    ros2 launch agx_arm_ctrl start_omnihand_skill_controller.launch.py \
        namespace:=right_hand omnihand_type:=right

The action server is then at ``/right_hand/perform`` (PerformAction).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the skill controller node.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace; share it with the OmniHand bridge for this side.",
    )
    omnihand_type_arg = DeclareLaunchArgument(
        "omnihand_type",
        default_value="right",
        description="OmniHand side (left or right); must match the bridge.",
    )
    hand_model_arg = DeclareLaunchArgument(
        "hand_model",
        default_value="o12_pro",
        description="OmniHand hardware model; must match the bridge's hand_model.",
    )
    skill_config_path_arg = DeclareLaunchArgument(
        "skill_config_path",
        default_value="",
        description="Optional skill catalogue YAML; empty auto-loads the package config.",
    )
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="perform",
        description="PerformAction server name (relative; namespaced per side).",
    )

    skill_controller = Node(
        package="agx_arm_ctrl",
        executable="omnihand_skill_controller",
        name="omnihand_skill_controller",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "hand_model": LaunchConfiguration("hand_model"),
            "skill_config_path": LaunchConfiguration("skill_config_path"),
            "action_name": LaunchConfiguration("action_name"),
        }],
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        omnihand_type_arg,
        hand_model_arg,
        skill_config_path_arg,
        action_name_arg,
        skill_controller,
    ])
