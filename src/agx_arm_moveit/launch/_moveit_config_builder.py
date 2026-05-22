import ast

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

ALL_ARM_TYPES = ["nero"]
ALL_EFFECTOR_TYPES = ["none", "agx_gripper", "revo2", "omnihand"]
ALL_REVO2_TYPES = ["left", "right"]
ALL_OMNIHAND_TYPES = ["left", "right"]


def declare_common_args():
    return [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this arm instance (e.g. arm1).",
        ),
        DeclareLaunchArgument(
            "arm_type", default_value="nero",
            choices=ALL_ARM_TYPES, description="Arm type.",
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
            "follow",
            default_value="false",
            choices=["true", "false"],
            description="Follow real arm state. "
            "true: move_group subscribes to feedback/joint_states; "
            "false: subscribes to control/joint_states (mock hardware).",
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
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    omnihand_type = LaunchConfiguration("omnihand_type").perform(context)
    use_mit_controller = context.launch_configurations.get("use_mit_controller", "false") == "true"
    tcp_offset = ast.literal_eval(
        LaunchConfiguration("tcp_offset").perform(context)
    )

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
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
        "omnihand_type": omnihand_type,
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

    if arm_type == "nero":
        moveit_config.trajectory_execution[
            "moveit_simple_controller_manager"
        ]["arm_controller"]["joints"] = [
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7",
        ]

    return moveit_config
