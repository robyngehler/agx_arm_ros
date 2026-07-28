from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from agx_arm_ctrl.duo_runtime_contract import validate_duo_both_arms_contract
from agx_arm_ctrl.execution_profiles import resolve_execution_profile


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


os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


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


def _resolved_argument(context, profile_values: dict[str, str], name: str) -> str:
    if name in profile_values:
        return profile_values[name]
    return LaunchConfiguration(name).perform(context).strip()


def _value_or_default(raw_value: object, default_value: str) -> str:
    text = str(raw_value).strip() if raw_value is not None else ""
    return text if text else default_value


def _bool_string(value: object, default: str) -> str:
    text = str(value).strip() if value is not None else ""
    return text if text else default


def _launch_actions(context):
    profile_values = resolve_execution_profile(
        LaunchConfiguration("execution_profile").perform(context).strip()
    )
    moveit_profile = _resolved_argument(context, profile_values, "moveit_profile")
    robot_name = _resolved_argument(context, profile_values, "robot_name") or "agx_arm"
    custom_model = _resolved_argument(context, profile_values, "custom_model")
    custom_model_xacro_args = _resolved_argument(context, profile_values, "custom_model_xacro_args")
    effector_type = _resolved_argument(context, profile_values, "effector_type")
    revo2_type = _resolved_argument(context, profile_values, "revo2_type")
    omnihand_type = _resolved_argument(context, profile_values, "omnihand_type")
    launch_omnihand_bridge = _resolved_argument(context, profile_values, "launch_omnihand_bridge")
    explicit_joint_prefix = _resolved_argument(context, profile_values, "input_joint_prefix")
    explicit_feedback_joint_prefix = _resolved_argument(context, profile_values, "feedback_joint_prefix")
    arm_instances_raw = _resolved_argument(context, profile_values, "arm_instances")
    follow = LaunchConfiguration("follow").perform(context).strip()
    use_mit_controller = LaunchConfiguration("use_mit_controller").perform(context).strip() == "true"

    if moveit_profile != "both_arms":
        raise ValueError(
            "start_multi_agx_arm_rviz.launch.py requires moveit_profile 'both_arms'; "
            "use start_single_agx_arm_rviz.launch.py for single-arm debug"
        )
    if not custom_model:
        raise ValueError(
            "start_multi_agx_arm_rviz.launch.py requires custom_model so the shared Duo description exists"
        )
    if not use_mit_controller:
        raise ValueError(
            "start_multi_agx_arm_rviz.launch.py currently requires use_mit_controller:=true"
        )

    instances = resolve_arm_instances(
        moveit_profile,
        arm_instances_raw,
        explicit_joint_prefix,
        explicit_feedback_joint_prefix,
    )
    validate_duo_both_arms_contract(
        moveit_profile,
        effector_type,
        instances,
        follow=follow,
        use_mit_controller=use_mit_controller,
    )

    root_namespace = normalize_relative_namespace(LaunchConfiguration("namespace").perform(context))
    shared_control_topic = absolute_topic_for_namespace(root_namespace, "control/duo_soft_target_joint_states")
    follow_joint_states_topic = LaunchConfiguration("follow_joint_states_topic").perform(context).strip()
    if not follow_joint_states_topic or follow_joint_states_topic == "feedback/joint_states":
        follow_joint_states_topic = DEFAULT_PREFIXED_FEEDBACK_TOPIC

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("agx_arm_description"),
                    "launch",
                    "display_control.launch.py",
                )
            ),
            launch_arguments={
                "namespace": root_namespace,
                "arm_type": LaunchConfiguration("arm_type").perform(context),
                "robot_name": robot_name,
                "moveit_profile": moveit_profile,
                "custom_model": custom_model,
                "custom_model_xacro_args": custom_model_xacro_args,
                "effector_type": effector_type,
                "revo2_type": revo2_type,
                "omnihand_type": omnihand_type,
                "pub_rate": LaunchConfiguration("pub_rate").perform(context),
                "follow": follow,
                "follow_joint_states_topic": follow_joint_states_topic,
                "input_joint_prefix": explicit_joint_prefix,
                "arm_base_frame": LaunchConfiguration("arm_base_frame").perform(context),
                "arm_tip_frame": LaunchConfiguration("arm_tip_frame").perform(context),
                "tcp_offset": LaunchConfiguration("tcp_offset").perform(context),
                "tcp_parent_frame": LaunchConfiguration("tcp_parent_frame").perform(context),
                "control": "true",
                "control_topic": shared_control_topic,
            }.items(),
        ),
        LogInfo(msg="Duo RViz debug uses one shared soft-target JointState topic and one MIT controller per arm namespace."),
    ]

    for instance in instances:
        instance_namespace = join_relative_namespaces(root_namespace, instance["namespace"])
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("agx_arm_mit_controller"),
                        "launch",
                        "start_nero_mit_controller.launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": instance_namespace,
                    "can_port": _value_or_default(instance["can_port"], LaunchConfiguration("can_port").perform(context)),
                    "arm_type": _value_or_default(instance["arm_type"], LaunchConfiguration("arm_type").perform(context)),
                    "custom_model": custom_model,
                    "custom_model_xacro_args": custom_model_xacro_args,
                    "effector_type": _value_or_default(instance["effector_type"], effector_type),
                    "omnihand_type": _value_or_default(instance["omnihand_type"], omnihand_type),
                    "launch_omnihand_bridge": launch_omnihand_bridge,
                    "hand_bus": LaunchConfiguration("hand_bus").perform(context),
                    "omnihand_backend_type": LaunchConfiguration("omnihand_backend_type").perform(context),
                    "omnihand_device_id": LaunchConfiguration("omnihand_device_id").perform(context),
                    "omnihand_canfd_id": LaunchConfiguration("omnihand_canfd_id").perform(context),
                    "omnihand_sdk_cfg_path": LaunchConfiguration("omnihand_sdk_cfg_path").perform(context),
                    "auto_enable": LaunchConfiguration("auto_enable").perform(context),
                    "fast_mode": LaunchConfiguration("fast_mode").perform(context),
                    "speed_percent": LaunchConfiguration("speed_percent").perform(context),
                    "pub_rate": LaunchConfiguration("pub_rate").perform(context),
                    "enable_timeout": LaunchConfiguration("enable_timeout").perform(context),
                    "tcp_offset": _value_or_default(instance["tcp_offset"], LaunchConfiguration("tcp_offset").perform(context)),
                    "gripper_default_effort": LaunchConfiguration("gripper_default_effort").perform(context),
                    "control_rate_hz": LaunchConfiguration("mit_control_rate_hz").perform(context),
                    "params_file": LaunchConfiguration("mit_params_file").perform(context),
                    "log_level": LaunchConfiguration("log_level").perform(context),
                    "input_joint_prefix": instance["joint_prefix"],
                    "launch_driver": _bool_string(instance["launch_driver"], "true"),
                    "enable_debug_joint_trajectory_topic": "true",
                }.items(),
            )
        )
        actions.append(
            Node(
                package="agx_arm_mit_tools",
                executable="agx_arm_mit_joint_state_bridge",
                namespace=instance_namespace,
                parameters=[
                    {
                        "input_topic": shared_control_topic,
                        "segment_duration_s": ParameterValue(
                            float(LaunchConfiguration("mit_joint_target_duration_s").perform(context)),
                            value_type=float,
                        ),
                        "input_joint_prefix": instance["joint_prefix"],
                        "auto_enable": False,
                    }
                ],
            )
        )

    if requires_prefixed_feedback(instances):
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
        Node(
            package="agx_arm_mit_tools",
            executable="agx_arm_duo_soft_estop",
            namespace=root_namespace,
            parameters=[
                {
                    "arm_namespaces": [
                        join_relative_namespaces(root_namespace, instance["namespace"])
                        for instance in instances
                    ],
                }
            ],
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
                description="Repo-owned execution preset for shared Duo RViz debug.",
            ),
            DeclareLaunchArgument(
                "can_port",
                default_value="can_nero_right",
                description="Fallback single-port argument for shared RViz debug. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.",
            ),
            DeclareLaunchArgument("arm_type", default_value="nero", choices=["nero"]),
            DeclareLaunchArgument(
                "moveit_profile",
                default_value="both_arms",
                choices=["nero_arm", "right_arm", "left_arm", "both_arms"],
                description="The shared Duo RViz debug surface currently requires both_arms.",
            ),
            DeclareLaunchArgument("custom_model", default_value=""),
            DeclareLaunchArgument("custom_model_xacro_args", default_value=""),
            DeclareLaunchArgument(
                "arm_instances",
                default_value="",
                description="Optional YAML list describing the two managed arm runtime instances.",
            ),
            DeclareLaunchArgument(
                "effector_type",
                default_value="none",
                choices=["none", "agx_gripper", "revo2", "omnihand"],
            ),
            DeclareLaunchArgument("revo2_type", default_value="left", choices=["left", "right"]),
            DeclareLaunchArgument("omnihand_type", default_value="left", choices=["left", "right"]),
            DeclareLaunchArgument("launch_omnihand_bridge", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "hand_bus", default_value="shared", choices=["shared", "dedicated"],
                description="shared: arm<->hand window handshake; dedicated: off (parallel).",
            ),
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
            DeclareLaunchArgument(
                "follow_joint_states_topic",
                default_value="feedback/prefixed_joint_states",
                description="JointState topic consumed by the shared Duo RViz description in follow mode.",
            ),
            DeclareLaunchArgument("arm_base_frame", default_value=""),
            DeclareLaunchArgument("arm_tip_frame", default_value=""),
            DeclareLaunchArgument("tcp_parent_frame", default_value=""),
            DeclareLaunchArgument("gripper_default_effort", default_value="1.0"),
            DeclareLaunchArgument("follow", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("use_mit_controller", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("mit_control_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("mit_params_file", default_value=default_mit_params_file),
            DeclareLaunchArgument("mit_joint_target_duration_s", default_value="0.75"),
            OpaqueFunction(function=_launch_actions),
        ]
    )