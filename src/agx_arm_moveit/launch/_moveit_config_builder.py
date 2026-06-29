import ast

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from _multi_arm_runtime import (
    controller_joint_names,
    controller_path,
    omnihand_controller_joint_names,
    omnihand_controller_path,
    resolve_arm_instances,
)

ALL_ARM_TYPES = ["nero"]
ALL_EFFECTOR_TYPES = ["none", "agx_gripper", "revo2", "omnihand"]
ALL_REVO2_TYPES = ["left", "right"]
ALL_OMNIHAND_TYPES = ["left", "right"]
ALL_MOVEIT_PROFILES = ["nero_arm", "right_arm", "left_arm", "both_arms"]
CANONICAL_ARM_JOINTS = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
]
DEFAULT_MOVEIT_GROUP = "nero_arm"
DUAL_ARM_MOVEIT_GROUP = "both_arms"
DUAL_ARM_JOINT_PREFIXES = ["left_arm_", "right_arm_"]
TRAC_IK_KINEMATICS = {
    "kinematics_solver": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
    "kinematics_solver_search_resolution": 0.005,
    "kinematics_solver_timeout": 0.01,
    "kinematics_solver_attempts": 5,
    "solve_type": "Distance",
}
MOVEIT_PROFILE_DEFAULTS = {
    "nero_arm": {
        "planning_group_name": "nero_arm",
        "input_joint_prefix": "",
        "arm_base_frame": "base_link",
        "arm_tip_frame": "tcp_link",
    },
    "right_arm": {
        "planning_group_name": "right_arm",
        "input_joint_prefix": "right_arm_",
        "arm_base_frame": "right_arm_base_link",
        "arm_tip_frame": "right_arm_nero_tool0",
    },
    "left_arm": {
        "planning_group_name": "left_arm",
        "input_joint_prefix": "left_arm_",
        "arm_base_frame": "left_arm_base_link",
        "arm_tip_frame": "left_arm_nero_tool0",
    },
    "both_arms": {
        "planning_group_name": "both_arms",
        "input_joint_prefix": "",
        "arm_base_frame": "",
        "arm_tip_frame": "",
    },
}


def _prefixed_arm_joint_names(joint_prefix: str) -> list[str]:
    if not joint_prefix:
        return list(CANONICAL_ARM_JOINTS)
    return [f"{joint_prefix}{joint_name}" for joint_name in CANONICAL_ARM_JOINTS]


def _resolved_arm_base_frame(custom_model: str, joint_prefix: str, explicit_frame: str) -> str:
    if explicit_frame:
        return explicit_frame
    if custom_model:
        return f"{joint_prefix}base_link" if joint_prefix else "base_link"
    return "base_link"


def _resolved_arm_tip_frame(custom_model: str, joint_prefix: str, explicit_frame: str) -> str:
    if explicit_frame:
        return explicit_frame
    if custom_model:
        return f"{joint_prefix}nero_tool0" if joint_prefix else "nero_tool0"
    return "tcp_link"


def _build_joint_limits(joint_names: list[str]) -> dict:
    return {
        "robot_description_planning": {
            "default_velocity_scaling_factor": 0.1,
            "default_acceleration_scaling_factor": 0.1,
            "joint_limits": {
                joint_name: {
                    "has_velocity_limits": True,
                    "max_velocity": 5.0,
                    "has_acceleration_limits": True,
                    "max_acceleration": 5.0,
                }
                for joint_name in joint_names
            }
        },
    }


def _build_kinematics(moveit_profile: str, group_name: str) -> dict:
    if moveit_profile == DUAL_ARM_MOVEIT_GROUP:
        return {
            "left_arm": dict(TRAC_IK_KINEMATICS),
            "right_arm": dict(TRAC_IK_KINEMATICS),
        }
    return {group_name: dict(TRAC_IK_KINEMATICS)}


def _profile_joint_prefixes(moveit_profile: str, explicit_joint_prefix: str) -> list[str]:
    if moveit_profile == DUAL_ARM_MOVEIT_GROUP:
        if explicit_joint_prefix:
            raise ValueError(
                "moveit_profile 'both_arms' does not accept input_joint_prefix; it always uses left_arm_ and right_arm_"
            )
        return list(DUAL_ARM_JOINT_PREFIXES)
    return [explicit_joint_prefix or MOVEIT_PROFILE_DEFAULTS[moveit_profile]["input_joint_prefix"]]


def _profile_arm_joint_names(moveit_profile: str, explicit_joint_prefix: str) -> list[str]:
    joint_names: list[str] = []
    for joint_prefix in _profile_joint_prefixes(moveit_profile, explicit_joint_prefix):
        joint_names.extend(_prefixed_arm_joint_names(joint_prefix))
    return joint_names


def _build_mit_trajectory_execution(arm_instances: list[dict[str, str]]) -> dict:
    controller_names: list[str] = []
    controllers: dict[str, dict] = {}
    for instance in arm_instances:
        name = controller_path(instance)
        controller_names.append(name)
        controllers[name] = {
            "type": "FollowJointTrajectory",
            "joints": controller_joint_names(instance["joint_prefix"]),
            "action_ns": "follow_joint_trajectory",
            "default": len(arm_instances) == 1,
        }

        # When the instance carries an OmniHand, register its FollowJointTrajectory
        # controller too, so MoveIt can actuate the 12 active hand joints (without
        # this, executing a hand-group plan fails with "Unable to identify any set
        # of controllers that can actuate the specified joints"). The action server
        # is provided by the hand's JointTrajectoryController / FJT bridge under the
        # same `<side>_omnihand_controller/follow_joint_trajectory` name.
        hand_controller = omnihand_controller_path(instance)
        if hand_controller:
            controller_names.append(hand_controller)
            controllers[hand_controller] = {
                "type": "FollowJointTrajectory",
                "joints": omnihand_controller_joint_names(instance["omnihand_type"].strip()),
                "action_ns": "follow_joint_trajectory",
                "default": False,
            }

    return {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": controller_names,
            **controllers,
        },
    }


def _resolve_profile_settings(
    moveit_profile: str,
    custom_model: str,
    explicit_joint_prefix: str,
    explicit_arm_base_frame: str,
    explicit_arm_tip_frame: str,
) -> dict[str, str]:
    defaults = MOVEIT_PROFILE_DEFAULTS.get(moveit_profile)
    if defaults is None:
        raise ValueError(f"Unsupported moveit_profile '{moveit_profile}'")

    if moveit_profile != DEFAULT_MOVEIT_GROUP and not custom_model:
        raise ValueError(
            f"moveit_profile '{moveit_profile}' requires custom_model so the prefixed Duo frames exist"
        )

    if moveit_profile == DUAL_ARM_MOVEIT_GROUP and (
        explicit_arm_base_frame or explicit_arm_tip_frame
    ):
        raise ValueError(
            "moveit_profile 'both_arms' does not accept single-arm base/tip overrides; use the staged Duo defaults"
        )

    input_joint_prefixes = _profile_joint_prefixes(moveit_profile, explicit_joint_prefix)
    input_joint_prefix = input_joint_prefixes[0] if len(input_joint_prefixes) == 1 else ""

    return {
        "planning_group_name": defaults["planning_group_name"],
        "input_joint_prefix": input_joint_prefix,
        "arm_base_frame": explicit_arm_base_frame or defaults["arm_base_frame"],
        "arm_tip_frame": explicit_arm_tip_frame or defaults["arm_tip_frame"],
        "include_dual_arm_groups": "true" if moveit_profile == DUAL_ARM_MOVEIT_GROUP else "false",
        "left_arm_base_frame": "left_arm_base_link",
        "left_arm_tip_frame": "left_arm_nero_tool0",
        "left_arm_joint_prefix": "left_arm_",
        "left_arm_link_prefix": "left_arm_",
        "right_arm_base_frame": "right_arm_base_link",
        "right_arm_tip_frame": "right_arm_nero_tool0",
        "right_arm_joint_prefix": "right_arm_",
        "right_arm_link_prefix": "right_arm_",
    }


def _resolve_include_end_effector_groups(custom_model: str, moveit_profile: str) -> str:
    if custom_model and moveit_profile == DUAL_ARM_MOVEIT_GROUP:
        return "false"
    return "true"


def _resolve_end_effector_parent_link(custom_model: str, arm_tip_frame: str) -> str:
    if custom_model:
        return arm_tip_frame
    return "tcp_link"


def _resolve_mount_body_link(custom_model: str) -> str:
    if custom_model:
        return "body_base_link"
    return ""


def declare_common_args():
    return [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.",
        ),
        DeclareLaunchArgument(
            "arm_type", default_value="nero",
            choices=ALL_ARM_TYPES, description="Arm type.",
        ),
        DeclareLaunchArgument(
            "moveit_profile",
            default_value=DEFAULT_MOVEIT_GROUP,
            choices=ALL_MOVEIT_PROFILES,
            description="MoveIt planning profile. Use right_arm or left_arm for prefixed Duo custom-model bringup.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="agx_arm",
            description="Robot name used in the generated SRDF. Override this for custom models whose URDF robot name differs.",
        ),
        DeclareLaunchArgument(
            "custom_model",
            default_value="",
            description="Optional custom model path. When set, MoveIt uses this xacro/URDF instead of the built-in arm model.",
        ),
        DeclareLaunchArgument(
            "custom_model_xacro_args",
            default_value="",
            description="Optional extra xacro args appended when custom_model is set.",
        ),
        DeclareLaunchArgument(
            "effector_type", default_value="none",
            choices=ALL_EFFECTOR_TYPES, description="Effector type.",
        ),
        DeclareLaunchArgument(
            "revo2_type", default_value="left",
            choices=ALL_REVO2_TYPES,
            description="Revo2 side (used when effector_type is revo2).",
        ),
        DeclareLaunchArgument(
            "omnihand_type", default_value="left",
            choices=ALL_OMNIHAND_TYPES,
            description="OmniHand side (used when effector_type is omnihand).",
        ),
        DeclareLaunchArgument(
            "tcp_offset",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
        ),
        DeclareLaunchArgument(
            "input_joint_prefix",
            default_value="",
            description="Optional prefix used by prefixed custom models for the controlled arm joints.",
        ),
        DeclareLaunchArgument(
            "feedback_joint_prefix",
            default_value="",
            description="Optional prefix added onto follow-side feedback/joint_states. This is mainly used by the multi-arm runtime wrappers.",
        ),
        DeclareLaunchArgument(
            "arm_instances",
            default_value="",
            description="Optional YAML list describing managed arm runtime instances. Empty derives defaults from moveit_profile.",
        ),
        DeclareLaunchArgument(
            "arm_base_frame",
            default_value="",
            description="Optional arm base frame used by the MoveIt arm chain when custom_model is set.",
        ),
        DeclareLaunchArgument(
            "arm_tip_frame",
            default_value="",
            description="Optional arm tip frame used by the MoveIt arm chain when custom_model is set.",
        ),
        DeclareLaunchArgument(
            "follow",
            default_value="false",
            choices=["true", "false"],
            description="Follow real arm state. "
            "true: move_group subscribes to feedback/joint_states; "
            "false: subscribes to control/joint_states (mock hardware).",
        ),
        DeclareLaunchArgument(
            "follow_joint_states_topic",
            default_value="feedback/joint_states",
            description="JointState topic consumed when follow:=true. Override for prefixed custom-model feedback adaptation.",
        ),
    ]


def _select_profile(
    effector_type: str, revo2_type: str, omnihand_type: str
) -> str:
    if effector_type == "agx_gripper":
        return "gripper"
    if effector_type == "revo2":
        return f"revo2_{revo2_type}"
    if effector_type == "omnihand":
        return f"omnihand_{omnihand_type}"
    return "none"


def build_moveit_config(context):
    arm_type = LaunchConfiguration("arm_type").perform(context)
    moveit_profile = LaunchConfiguration("moveit_profile").perform(context)
    custom_model = LaunchConfiguration("custom_model").perform(context)
    custom_model_xacro_args = LaunchConfiguration("custom_model_xacro_args").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    omnihand_type = LaunchConfiguration("omnihand_type").perform(context)
    explicit_joint_prefix = LaunchConfiguration("input_joint_prefix").perform(context)
    explicit_feedback_joint_prefix = LaunchConfiguration("feedback_joint_prefix").perform(context)
    arm_instances_raw = LaunchConfiguration("arm_instances").perform(context)
    explicit_arm_base_frame = LaunchConfiguration("arm_base_frame").perform(context)
    explicit_arm_tip_frame = LaunchConfiguration("arm_tip_frame").perform(context)
    profile_settings = _resolve_profile_settings(
        moveit_profile,
        custom_model,
        explicit_joint_prefix,
        explicit_arm_base_frame,
        explicit_arm_tip_frame,
    )
    input_joint_prefix = profile_settings["input_joint_prefix"]
    arm_instances = resolve_arm_instances(
        moveit_profile,
        arm_instances_raw,
        explicit_joint_prefix,
        explicit_feedback_joint_prefix,
    )
    arm_base_frame = _resolved_arm_base_frame(
        custom_model,
        input_joint_prefix,
        profile_settings["arm_base_frame"],
    )
    arm_tip_frame = _resolved_arm_tip_frame(
        custom_model,
        input_joint_prefix,
        profile_settings["arm_tip_frame"],
    )
    use_mit_controller = context.launch_configurations.get("use_mit_controller", "false") == "true"
    tcp_offset = ast.literal_eval(
        LaunchConfiguration("tcp_offset").perform(context)
    )
    controlled_joint_names = _profile_arm_joint_names(moveit_profile, explicit_joint_prefix)

    profile = _select_profile(effector_type, revo2_type, omnihand_type)
    trajectory_execution_config = (
        "config/moveit_controllers_mit.yaml"
        if use_mit_controller
        else f"config/moveit_controllers_{profile}.yaml"
    )
    urdf_mappings = {
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
        "omnihand_type": omnihand_type,
        "tcp_offset_xyz": f"{tcp_offset[0]} {tcp_offset[1]} {tcp_offset[2]}",
        "tcp_offset_rpy": f"{tcp_offset[3]} {tcp_offset[4]} {tcp_offset[5]}",
    }
    srdf_mappings = {
        "robot_name": LaunchConfiguration("robot_name").perform(context),
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
        "omnihand_type": omnihand_type,
        "planning_group_name": profile_settings["planning_group_name"],
        "arm_base_frame": arm_base_frame,
        "arm_tip_frame": arm_tip_frame,
        "end_effector_parent_link": _resolve_end_effector_parent_link(custom_model, arm_tip_frame),
        "mount_body_link": _resolve_mount_body_link(custom_model),
        "arm_joint_prefix": input_joint_prefix,
        "arm_link_prefix": input_joint_prefix,
        "include_end_effector_groups": _resolve_include_end_effector_groups(custom_model, moveit_profile),
        "include_dual_arm_groups": profile_settings["include_dual_arm_groups"],
        "left_arm_base_frame": profile_settings["left_arm_base_frame"],
        "left_arm_tip_frame": profile_settings["left_arm_tip_frame"],
        "left_mount_body_link": _resolve_mount_body_link(custom_model),
        "left_arm_joint_prefix": profile_settings["left_arm_joint_prefix"],
        "left_arm_link_prefix": profile_settings["left_arm_link_prefix"],
        "right_arm_base_frame": profile_settings["right_arm_base_frame"],
        "right_arm_tip_frame": profile_settings["right_arm_tip_frame"],
        "right_mount_body_link": _resolve_mount_body_link(custom_model),
        "right_arm_joint_prefix": profile_settings["right_arm_joint_prefix"],
        "right_arm_link_prefix": profile_settings["right_arm_link_prefix"],
    }

    moveit_config = (
        MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
        .robot_description(file_path="config/agx_arm.urdf.xacro", mappings=urdf_mappings)
        .robot_description_semantic(
            file_path="config/agx_arm.srdf.xacro", mappings=srdf_mappings
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .trajectory_execution(file_path=trajectory_execution_config)
        .to_moveit_configs()
    )

    if custom_model:
        moveit_config.robot_description = {
            "robot_description": ParameterValue(
                Command(["xacro ", custom_model, " ", custom_model_xacro_args]),
                value_type=str,
            )
        }

    if custom_model or moveit_profile != DEFAULT_MOVEIT_GROUP:
        moveit_config.robot_description_kinematics = _build_kinematics(
            moveit_profile,
            profile_settings["planning_group_name"]
        )

    # Single-arm OmniHand bring-up carries the effector at the top level, not on the
    # (lone) arm instance, so propagate it before building the controllers — otherwise
    # no hand controller is registered and MoveIt cannot actuate the finger joints.
    # Multi-arm instances declare their own omnihand_type via arm_instances.
    if effector_type == "omnihand" and omnihand_type and len(arm_instances) == 1:
        only = dict(arm_instances[0])
        if not only.get("omnihand_type"):
            only["omnihand_type"] = omnihand_type
            arm_instances = [only]

    if use_mit_controller:
        moveit_config.trajectory_execution = _build_mit_trajectory_execution(arm_instances)

    if arm_type == "nero":
        moveit_simple_controller_manager = moveit_config.trajectory_execution[
            "moveit_simple_controller_manager"
        ]
        if not use_mit_controller and "arm_controller" in moveit_simple_controller_manager:
            moveit_simple_controller_manager["arm_controller"]["joints"] = [
                *controlled_joint_names,
            ]

    if custom_model or input_joint_prefix or moveit_profile == DUAL_ARM_MOVEIT_GROUP:
        moveit_config.joint_limits = _build_joint_limits(controlled_joint_names)

    return moveit_config
