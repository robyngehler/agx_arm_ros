"""Coordination bring-up for the tea-pour demo (activity ``tea_pour_left_v1``).

Same shape as ``start_hefeweizen_demo.launch.py`` -- both OmniHand bridges, both
skill controllers, the coordinator -- but with CPU-lean defaults, because the
Jetson is the binding constraint on this demo, not the robot. See
``docs/sprint_refactor/reference/critical_cpu_paths.md``: when the host stalls,
nothing drains the CAN RX socket and the kernel drops hand response frames. Every
Hz saved here is bus reliability, not just headroom.

What is lowered versus the Hefeweizen defaults, and why it is safe:

- ``hand_pub_rate`` 50 -> 20 Hz. The hand's ROS feedback republish rate. Nothing
  in this demo closes a loop on it -- the skill controller compares against its
  own commanded ramp -- and it is decoupled from the SDK poll rate.
- ``hand_joint_read_rate`` 20 -> 10 Hz. Each poll is a real CAN request/response
  on the shared arm+hand bus (hot path 4). Halving it halves that traffic; the
  skill controller's pose confirmation just needs a reading within its timeout.
- the right hand's rates are lowered further (``idle_hand_*``): the right side is
  brought up and stays live, but this activity never addresses it, so its polling
  is pure background load.

What is deliberately NOT lowered: the arm driver's ``pub_rate`` and the MIT
control rate. Those belong to the arm bring-up (started separately) and are load
bearing for control and gravity compensation.

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
    ros2 run agx_arm_coordination run_activity --activity tea_pour_left_v1
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

ACTIVE_SIDE = "left"   # the side this activity drives
IDLE_SIDE = "right"    # brought up and live, but never addressed by the activity


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
            description="Active hand feedback republish rate (Hz).",
        ),
        DeclareLaunchArgument(
            "hand_joint_read_rate", default_value="50.0",
            description="Active hand SDK joint poll rate (Hz) -- real CAN traffic.",
        ),
        DeclareLaunchArgument(
            "idle_hand_pub_rate", default_value="5.0",
            description="Idle-side hand republish rate (Hz); it is only kept alive.",
        ),
        DeclareLaunchArgument(
            "idle_hand_joint_read_rate", default_value="2.0",
            description="Idle-side hand SDK poll rate (Hz); pure background load.",
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
    nodes += _hand_side(ACTIVE_SIDE, ctrl_launch_dir, "hand_pub_rate", "hand_joint_read_rate")
    nodes += _hand_side(
        IDLE_SIDE, ctrl_launch_dir, "idle_hand_pub_rate", "idle_hand_joint_read_rate"
    )
    nodes.append(coordinator)
    return LaunchDescription(nodes)
