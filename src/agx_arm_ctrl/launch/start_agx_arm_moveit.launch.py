from __future__ import annotations

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MOVEIT_LAUNCH_DIR = Path(get_package_share_directory("agx_arm_moveit")) / "launch"
if str(MOVEIT_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(MOVEIT_LAUNCH_DIR))

from _multi_arm_runtime import (  # noqa: E402
    DEFAULT_PREFIXED_FEEDBACK_TOPIC,
    absolute_topic_for_namespace,
    join_relative_namespaces,
    normalize_relative_namespace,
    requires_prefixed_feedback,
    resolve_arm_instances,
)


def _bool_string(value: object, default: str) -> str:
    text = str(value).strip() if value is not None else ""
    return text if text else default


def _value_or_default(raw_value: object, default_value: str) -> str:
    text = str(raw_value).strip() if raw_value is not None else ""
    return text if text else default_value


def _resolved_robot_name(context) -> str:
    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    if robot_name:
        return robot_name
    custom_model = LaunchConfiguration("custom_model").perform(context).strip()
    return "duo_nero_system" if custom_model else "agx_arm"


def _default_mit_params_file() -> str:
    package_share_dir = Path(get_package_share_directory("agx_arm_mit_controller")).resolve()
    installed_params_file = package_share_dir / "config" / "nero_mit_controller_defaults.yaml"

    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return str(installed_params_file)

    source_params_file = workspace_root / "src" / "agx_arm_mit_controller" / "config" / "nero_mit_controller_defaults.yaml"
    if source_params_file.is_file():
        return str(source_params_file)
    return str(installed_params_file)


def _resolved_follow_joint_states_topic(context, instances: list[dict[str, str]]) -> str:
    follow = LaunchConfiguration("follow").perform(context).strip()
    explicit_topic = LaunchConfiguration("follow_joint_states_topic").perform(context).strip()
    if follow != "true":
        return explicit_topic or "feedback/joint_states"
    if explicit_topic and explicit_topic != "feedback/joint_states":
        return explicit_topic
    if requires_prefixed_feedback(instances):
        return DEFAULT_PREFIXED_FEEDBACK_TOPIC
    return "feedback/joint_states"


def _instance_runtime_launches(context):
    moveit_profile = LaunchConfiguration("moveit_profile").perform(context).strip()
    explicit_joint_prefix = LaunchConfiguration("input_joint_prefix").perform(context).strip()
    explicit_feedback_joint_prefix = LaunchConfiguration("feedback_joint_prefix").perform(context).strip()
    arm_instances_raw = LaunchConfiguration("arm_instances").perform(context)
    instances = resolve_arm_instances(
        moveit_profile,
        arm_instances_raw,
        explicit_joint_prefix,
        explicit_feedback_joint_prefix,
    )

    use_mit_controller = LaunchConfiguration("use_mit_controller").perform(context).strip() == "true"
    root_namespace = normalize_relative_namespace(LaunchConfiguration("namespace").perform(context))

    actions = []
    if len(instances) > 1:
        for instance in instances:
            if not instance["can_port"]:
                raise ValueError(
                    "Multi-arm runtime launch requires arm_instances entries to define can_port"
                )

    for instance in instances:
        instance_namespace = join_relative_namespaces(root_namespace, instance["namespace"])
        common_arguments = {
            "namespace": instance_namespace,
            "can_port": _value_or_default(instance["can_port"], LaunchConfiguration("can_port").perform(context)),
            "arm_type": _value_or_default(instance["arm_type"], LaunchConfiguration("arm_type").perform(context)),
            "effector_type": _value_or_default(instance["effector_type"], LaunchConfiguration("effector_type").perform(context)),
            "omnihand_type": _value_or_default(instance["omnihand_type"], LaunchConfiguration("omnihand_type").perform(context)),
            "revo2_type": _value_or_default(instance["revo2_type"], LaunchConfiguration("revo2_type").perform(context)),
            "launch_omnihand_bridge": LaunchConfiguration("launch_omnihand_bridge").perform(context),
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type").perform(context),
            "auto_enable": LaunchConfiguration("auto_enable").perform(context),
            "log_level": LaunchConfiguration("log_level").perform(context),
            "fast_mode": LaunchConfiguration("fast_mode").perform(context),
            "speed_percent": LaunchConfiguration("speed_percent").perform(context),
            "pub_rate": LaunchConfiguration("pub_rate").perform(context),
            "enable_timeout": LaunchConfiguration("enable_timeout").perform(context),
            "tcp_offset": _value_or_default(instance["tcp_offset"], LaunchConfiguration("tcp_offset").perform(context)),
            "gripper_default_effort": LaunchConfiguration("gripper_default_effort").perform(context),
            "publish_gripper_joint": "false",
        }

        if use_mit_controller:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(Path(get_package_share_directory("agx_arm_mit_controller")) / "launch" / "start_nero_mit_controller.launch.py")
                    ),
                    launch_arguments={
                        **common_arguments,
                        "control_rate_hz": LaunchConfiguration("mit_control_rate_hz").perform(context),
                        "input_joint_prefix": instance["joint_prefix"],
                        "params_file": LaunchConfiguration("mit_params_file").perform(context),
                        "launch_driver": _bool_string(instance["launch_driver"], "true"),
                    }.items(),
                )
            )
        else:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(Path(get_package_share_directory("agx_arm_ctrl")) / "launch" / "start_single_agx_arm.launch.py")
                    ),
                    launch_arguments=common_arguments.items(),
                )
            )

    if LaunchConfiguration("follow").perform(context).strip() == "true" and requires_prefixed_feedback(instances):
        actions.append(
            Node(
                package="agx_arm_mit_tools",
                executable="agx_arm_joint_state_merger",
                namespace=root_namespace,
                parameters=[
                    {
                        "source_topics": [
                            absolute_topic_for_namespace(
                                join_relative_namespaces(root_namespace, instance["namespace"]),
                                "feedback/joint_states",
                            )
                            for instance in instances
                        ],
                        "joint_prefixes": [instance["feedback_joint_prefix"] for instance in instances],
                        "output_topic": DEFAULT_PREFIXED_FEEDBACK_TOPIC,
                    }
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("agx_arm_moveit")) / "launch" / "start_moveit.launch.py")
            ),
            launch_arguments={
                "namespace": root_namespace,
                "arm_type": LaunchConfiguration("arm_type").perform(context),
                "moveit_profile": moveit_profile,
                "robot_name": _resolved_robot_name(context),
                "custom_model": LaunchConfiguration("custom_model").perform(context),
                "custom_model_xacro_args": LaunchConfiguration("custom_model_xacro_args").perform(context),
                "effector_type": LaunchConfiguration("effector_type").perform(context),
                "revo2_type": LaunchConfiguration("revo2_type").perform(context),
                "omnihand_type": LaunchConfiguration("omnihand_type").perform(context),
                "tcp_offset": LaunchConfiguration("tcp_offset").perform(context),
                "input_joint_prefix": explicit_joint_prefix,
                "feedback_joint_prefix": explicit_feedback_joint_prefix,
                "arm_base_frame": LaunchConfiguration("arm_base_frame").perform(context),
                "arm_tip_frame": LaunchConfiguration("arm_tip_frame").perform(context),
                "follow": LaunchConfiguration("follow").perform(context),
                "follow_joint_states_topic": _resolved_follow_joint_states_topic(context, instances),
                "use_mit_controller": LaunchConfiguration("use_mit_controller").perform(context),
                "use_rviz": LaunchConfiguration("use_rviz").perform(context),
                "db": LaunchConfiguration("db").perform(context),
                "planning_pipelines": LaunchConfiguration("planning_pipelines").perform(context),
                "load_simple_obstacles": LaunchConfiguration("load_simple_obstacles").perform(context),
                "simple_obstacles_config": LaunchConfiguration("simple_obstacles_config").perform(context),
                "arm_instances": LaunchConfiguration("arm_instances").perform(context),
            }.items(),
        )
    )

    if LaunchConfiguration("arm_instances").perform(context).strip():
        actions.append(
            LogInfo(
                msg=[
                    "Managed arm instances: ",
                    LaunchConfiguration("arm_instances"),
                ]
            )
        )

    return actions


def generate_launch_description():
    default_mit_params_file = _default_mit_params_file()
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("arm_type", default_value="nero", choices=["nero"]),
            DeclareLaunchArgument(
                "moveit_profile",
                default_value="nero_arm",
                choices=["nero_arm", "right_arm", "left_arm", "both_arms"],
            ),
            DeclareLaunchArgument("robot_name", default_value=""),
            DeclareLaunchArgument("custom_model", default_value=""),
            DeclareLaunchArgument("custom_model_xacro_args", default_value=""),
            DeclareLaunchArgument(
                "arm_instances",
                default_value="",
                description="Optional YAML list describing managed arm runtime instances. Empty derives defaults from moveit_profile.",
            ),
            DeclareLaunchArgument(
                "effector_type",
                default_value="none",
                choices=["none", "agx_gripper", "revo2", "omnihand"],
            ),
            DeclareLaunchArgument("revo2_type", default_value="left", choices=["left", "right"]),
            DeclareLaunchArgument("omnihand_type", default_value="left", choices=["left", "right"]),
            DeclareLaunchArgument("launch_omnihand_bridge", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("omnihand_backend_type", default_value="mock"),
            DeclareLaunchArgument("auto_enable", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("fast_mode", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("speed_percent", default_value="100"),
            DeclareLaunchArgument("pub_rate", default_value="200"),
            DeclareLaunchArgument("enable_timeout", default_value="5.0"),
            DeclareLaunchArgument("tcp_offset", default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("input_joint_prefix", default_value=""),
            DeclareLaunchArgument("feedback_joint_prefix", default_value=""),
            DeclareLaunchArgument("follow_joint_states_topic", default_value="feedback/joint_states"),
            DeclareLaunchArgument("arm_base_frame", default_value=""),
            DeclareLaunchArgument("arm_tip_frame", default_value=""),
            DeclareLaunchArgument("gripper_default_effort", default_value="1.0"),
            DeclareLaunchArgument("follow", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("use_mit_controller", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("db", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("mit_control_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("mit_params_file", default_value=default_mit_params_file),
            DeclareLaunchArgument(
                "planning_pipelines",
                default_value="",
                description="Optional comma-separated planning pipeline whitelist forwarded into move_group.",
            ),
            DeclareLaunchArgument("load_simple_obstacles", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "simple_obstacles_config",
                default_value=str(Path(get_package_share_directory("agx_arm_moveit")) / "config" / "simple_obstacles.json"),
            ),
            OpaqueFunction(function=lambda context: _instance_runtime_launches(context)),
        ]
    )
