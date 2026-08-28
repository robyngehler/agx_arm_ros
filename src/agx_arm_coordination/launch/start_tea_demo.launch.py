"""Coordination bring-up for the tea-pour demo (activity ``tea_pour_duo_v2``).

Same shape as ``start_hefeweizen_demo.launch.py`` -- both OmniHand bridges, both
skill controllers, the coordinator.

Both hands run at the same rates. v1 lowered the right side to 5/2 Hz because it
was only kept alive; v2 shapes it three times and holds a support pose under the
can for the whole carry, so it is an active device.

``hand_pub_rate`` is the ROS feedback republish rate and nothing here closes a
loop on it -- the skill controller compares against its own commanded ramp.
``hand_joint_read_rate`` is real CAN traffic on the hand's own bus, and lowering
it is CPU headroom on the Jetson: when the host stalls, nothing drains the CAN RX
socket and the kernel drops response frames
(``docs/sprint_refactor/reference/critical_cpu_paths.md``).

Not settable here: the arm driver's ``pub_rate`` and the MIT control rate. Those
belong to the arm bring-up and are load bearing for control and gravity
compensation.

The ARM stack is NOT started here. Bring up the Duo arm/MoveIt slice first
(``mode:=moveit_mit``, and use ``use_rviz:=false`` for the demo -- rviz rendering
plus its planning-scene monitor is hot path 5), or pass ``arm_dry_run:=true`` to
exercise the coordinator with no arms at all.

Examples::

    # laptop / dry validation: mock hands, no arms. start_unit_safety:=true
    # because no arm bring-up is running to provide the writer, and the
    # coordinator refuses every activity until a generation is established.
    ros2 launch agx_arm_coordination start_tea_demo.launch.py \
        arm_dry_run:=true start_unit_safety:=true

    # on the Jetson, after the arm slice is up
    ros2 launch agx_arm_coordination start_tea_demo.launch.py backend_type:=sdk

    # then run it (Ctrl+C here cancels the activity and stops the arm)
    ros2 run agx_arm_coordination run_activity --activity tea_pour_duo_v2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIDES = ("left", "right")   # v2 drives both


def _hand_side(side: str, ctrl_launch_dir: str, pub_rate: str, joint_read_rate: str):
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
            "pub_rate": LaunchConfiguration(pub_rate),
            "joint_read_rate": LaunchConfiguration(joint_read_rate),
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

    args = [
        DeclareLaunchArgument("hand_model", default_value="o12_pro"),
        DeclareLaunchArgument(
            "backend_type", default_value="mock",
            description="OmniHand backend: mock or sdk.",
        ),
        DeclareLaunchArgument(
            "arm_dry_run", default_value="false",
            description="If true, arm goals are logged but never sent to MoveIt.",
        ),
        DeclareLaunchArgument(
            "hand_pub_rate", default_value="50.0",
            description="Hand feedback republish rate (Hz), both sides.",
        ),
        DeclareLaunchArgument(
            "hand_joint_read_rate", default_value="50.0",
            description="Hand SDK joint poll rate (Hz), both sides -- real CAN traffic.",
        ),
        DeclareLaunchArgument(
            "poll_period_sec", default_value="0.05",
            description="Coordinator scheduler tick (s). Bounds how quickly a "
                        "Ctrl+C is noticed, so do not raise it far.",
        ),
        # Off by default because the arm bring-up starts it, and exactly one
        # writer may run per unit. Turn it on for arm_dry_run, where there is no
        # arm bring-up and the coordinator would otherwise refuse every activity.
        DeclareLaunchArgument(
            "start_unit_safety", default_value="false",
            choices=["true", "false"],
            description="Start the unit safety generation writer here. Exactly "
                        "one must run per unit; the arm bring-up starts it, so "
                        "set true only for arm_dry_run.",
        ),
    ]

    coordinator = Node(
        package="agx_arm_coordination",
        executable="coordinator",
        name="agx_arm_coordinator",
        output="screen",
        parameters=[{
            "arm_dry_run": ParameterValue(LaunchConfiguration("arm_dry_run"), value_type=bool),
            "poll_period_sec": ParameterValue(
                LaunchConfiguration("poll_period_sec"), value_type=float
            ),
        }],
    )

    # Root namespace: the writer publishes relative `unit_safety` and every
    # observer subscribes to absolute `/unit_safety`.
    unit_safety = Node(
        package="agx_arm_ctrl",
        executable="unit_safety",
        name="unit_safety",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_unit_safety")),
    )

    nodes = list(args)
    nodes.append(unit_safety)
    for side in SIDES:
        nodes += _hand_side(
            side, ctrl_launch_dir, "hand_pub_rate", "hand_joint_read_rate"
        )
    nodes.append(coordinator)
    return LaunchDescription(nodes)
