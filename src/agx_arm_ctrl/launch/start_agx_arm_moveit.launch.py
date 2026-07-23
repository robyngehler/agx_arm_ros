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
from agx_arm_ctrl.execution_profiles import resolve_execution_profile  # noqa: E402
from agx_arm_ctrl.duo_runtime_contract import validate_duo_both_arms_contract  # noqa: E402


def _bool_string(value: object, default: str) -> str:
    text = str(value).strip() if value is not None else ""
    return text if text else default


def _value_or_default(raw_value: object, default_value: str) -> str:
    text = str(raw_value).strip() if raw_value is not None else ""
    return text if text else default_value


def _resolved_argument(context, profile_values: dict[str, str], name: str) -> str:
    if name in profile_values:
        return profile_values[name]
    return LaunchConfiguration(name).perform(context).strip()


def _resolved_robot_name(context, profile_values: dict[str, str]) -> str:
    robot_name = _resolved_argument(context, profile_values, "robot_name")
    if robot_name:
        return robot_name
    custom_model = _resolved_argument(context, profile_values, "custom_model")
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
    profile_values = resolve_execution_profile(
        LaunchConfiguration("execution_profile").perform(context).strip()
    )
    moveit_profile = _resolved_argument(context, profile_values, "moveit_profile")
    explicit_joint_prefix = _resolved_argument(context, profile_values, "input_joint_prefix")
    explicit_feedback_joint_prefix = _resolved_argument(context, profile_values, "feedback_joint_prefix")
    arm_instances_raw = _resolved_argument(context, profile_values, "arm_instances")
    custom_model = _resolved_argument(context, profile_values, "custom_model")
    custom_model_xacro_args = _resolved_argument(context, profile_values, "custom_model_xacro_args")
    effector_type = _resolved_argument(context, profile_values, "effector_type")
    revo2_type = _resolved_argument(context, profile_values, "revo2_type")
    omnihand_type = _resolved_argument(context, profile_values, "omnihand_type")
    launch_omnihand_bridge = _resolved_argument(context, profile_values, "launch_omnihand_bridge")
    tcp_offset = _resolved_argument(context, profile_values, "tcp_offset")
    arm_base_frame = _resolved_argument(context, profile_values, "arm_base_frame")
    arm_tip_frame = _resolved_argument(context, profile_values, "arm_tip_frame")
    instances = resolve_arm_instances(
        moveit_profile,
        arm_instances_raw,
        explicit_joint_prefix,
        explicit_feedback_joint_prefix,
    )
    # CAN bus resolution: an explicit (non-empty) can_port always wins, then the
    # profile's registry-derived side bus, then the legacy right-bus fallback.
    # Without the profile step a left-side profile silently drove the default
    # right bus and connected the wrong arm.
    can_port_cli = LaunchConfiguration("can_port").perform(context).strip()
    resolved_can_port = can_port_cli or profile_values.get("can_port", "") or "can_nero_right"
    use_mit_controller = LaunchConfiguration("use_mit_controller").perform(context).strip() == "true"
    validate_duo_both_arms_contract(
        moveit_profile,
        effector_type,
        instances,
        follow=LaunchConfiguration("follow").perform(context).strip(),
        use_mit_controller=use_mit_controller,
    )
    root_namespace = normalize_relative_namespace(LaunchConfiguration("namespace").perform(context))

    actions = []
    if len(instances) > 1:
        for instance in instances:
            if _bool_string(instance["launch_driver"], "true") == "true" and not instance["can_port"]:
                raise ValueError(
                    "Multi-arm runtime launch requires arm_instances entries to define can_port when launch_driver is true"
                )

    for instance in instances:
        instance_namespace = join_relative_namespaces(root_namespace, instance["namespace"])
        common_arguments = {
            "namespace": instance_namespace,
            "can_port": _value_or_default(instance["can_port"], resolved_can_port),
            "arm_type": _value_or_default(instance["arm_type"], LaunchConfiguration("arm_type").perform(context)),
            "effector_type": _value_or_default(instance["effector_type"], effector_type),
            "omnihand_type": _value_or_default(instance["omnihand_type"], omnihand_type),
            "revo2_type": _value_or_default(instance["revo2_type"], revo2_type),
            "launch_omnihand_bridge": launch_omnihand_bridge,
            "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type").perform(context),
            "omnihand_device_id": LaunchConfiguration("omnihand_device_id").perform(context),
            "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id").perform(context),
            "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path").perform(context),
            "auto_enable": LaunchConfiguration("auto_enable").perform(context),
            "log_level": LaunchConfiguration("log_level").perform(context),
            "fast_mode": LaunchConfiguration("fast_mode").perform(context),
            "speed_percent": LaunchConfiguration("speed_percent").perform(context),
            "pub_rate": LaunchConfiguration("pub_rate").perform(context),
            "enable_timeout": LaunchConfiguration("enable_timeout").perform(context),
            "tcp_offset": _value_or_default(instance["tcp_offset"], tcp_offset),
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
                        "custom_model": custom_model,
                        "custom_model_xacro_args": custom_model_xacro_args,
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

    if len(instances) > 1 and use_mit_controller:
        duo_arm_namespaces = [
            join_relative_namespaces(root_namespace, instance["namespace"])
            for instance in instances
        ]
        actions.append(
            Node(
                package="agx_arm_mit_tools",
                executable="agx_arm_duo_soft_estop",
                namespace=root_namespace,
                parameters=[{"arm_namespaces": duo_arm_namespaces}],
            )
        )
        # Hand-coordinated shared-CAN recovery service: the hard-escalation
        # target the duo e-stop delegates to (stop hand -> verified arm e-stop ->
        # bus recovery -> verify normal mode). Runs alongside so the escalation
        # is wired end to end; the arm driver's own emergency_stop is the
        # fallback if this node is absent.
        actions.append(
            Node(
                package="agx_arm_mit_tools",
                executable="agx_arm_shared_can_recovery",
                namespace=root_namespace,
                parameters=[{"arm_namespaces": duo_arm_namespaces}],
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
                "robot_name": _resolved_robot_name(context, profile_values),
                "custom_model": custom_model,
                "custom_model_xacro_args": custom_model_xacro_args,
                "effector_type": effector_type,
                "revo2_type": revo2_type,
                "omnihand_type": omnihand_type,
                "tcp_offset": tcp_offset,
                "input_joint_prefix": explicit_joint_prefix,
                "feedback_joint_prefix": explicit_feedback_joint_prefix,
                "arm_base_frame": arm_base_frame,
                "arm_tip_frame": arm_tip_frame,
                "follow": LaunchConfiguration("follow").perform(context),
                "follow_joint_states_topic": _resolved_follow_joint_states_topic(context, instances),
                "use_mit_controller": LaunchConfiguration("use_mit_controller").perform(context),
                "use_rviz": LaunchConfiguration("use_rviz").perform(context),
                "db": LaunchConfiguration("db").perform(context),
                "planning_pipelines": LaunchConfiguration("planning_pipelines").perform(context),
                "load_simple_obstacles": LaunchConfiguration("load_simple_obstacles").perform(context),
                "simple_obstacles_config": LaunchConfiguration("simple_obstacles_config").perform(context),
                "arm_instances": arm_instances_raw,
            }.items(),
        )
    )

    if arm_instances_raw:
        actions.append(
            LogInfo(
                msg=[
                    "Managed arm instances: ",
                    arm_instances_raw,
                ]
            )
        )

    if profile_values:
        actions.append(
            LogInfo(
                msg=[
                    "Resolved execution_profile: ",
                    LaunchConfiguration("execution_profile"),
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
            DeclareLaunchArgument(
                "execution_profile",
                default_value="manual",
                choices=["manual", "standalone", "left_arm", "left_hand", "right_arm", "right_hand", "duo_arm", "duo_hand"],
                description=(
                    "Repo-owned execution preset that resolves the mounted Duo model, arm/hand composition, "
                    "prefix/frame defaults, and supported hand wiring from one choice. In the normal wrapper "
                    "path this preset is also what determines the custom_model/custom_model_xacro_args passed "
                    "down to each MIT instance, which in turn controls how the gravity URDF is derived."
                ),
            ),
            DeclareLaunchArgument(
                "can_port",
                default_value="",
                description="CAN port used by the wrapped AGX Arm node. Empty resolves the side bus from the execution profile (registry arm.sides.*.can_port), falling back to can_nero_right. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.",
            ),
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
            DeclareLaunchArgument("omnihand_device_id", default_value="1"),
            DeclareLaunchArgument("omnihand_canfd_id", default_value="0"),
            DeclareLaunchArgument("omnihand_sdk_cfg_path", default_value=""),
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
            DeclareLaunchArgument("mit_control_rate_hz", default_value="50.0"),
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
