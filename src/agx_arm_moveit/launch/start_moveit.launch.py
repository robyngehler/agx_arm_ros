import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from _moveit_config_builder import (
    build_moveit_config,
    declare_common_args,
)
from _multi_arm_runtime import (
    ARM_SIDES,
    CANONICAL_ARM_JOINTS,
    MOTION_PROFILES,
    omnihand_controller_joint_names,
)


def _build_ros2_controllers_file(
    arm_type, effector_type, revo2_type, omnihand_type, namespace, moveit_profile, input_joint_prefix
):
    # Joint prefixes per side and the canonical joint set come from the registry
    # (motion_profiles.<profile>.sides + arm.sides.<side>.prefix + arm.canonical_joints),
    # not a second hardcoded copy. An explicit single-arm input_joint_prefix still overrides.
    profile_sides = MOTION_PROFILES.get(moveit_profile, {}).get("sides", [])
    if len(profile_sides) > 1:
        joint_prefixes = [str(ARM_SIDES.get(side, {}).get("prefix", f"{side}_arm_")) for side in profile_sides]
    elif input_joint_prefix:
        joint_prefixes = [input_joint_prefix]
    elif profile_sides:
        joint_prefixes = [str(ARM_SIDES.get(profile_sides[0], {}).get("prefix", ""))]
    else:
        joint_prefixes = [input_joint_prefix]

    arm_joints = []
    for joint_prefix in joint_prefixes:
        arm_joints.extend(f"{joint_prefix}{joint}" for joint in CANONICAL_ARM_JOINTS)

    cm_controllers = {
        "arm_controller": {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        },
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster",
        },
    }

    ns = namespace.strip("/")
    cm_node = f"/{ns}/controller_manager" if ns else "/controller_manager"

    config = {
        cm_node: {
            "ros__parameters": {"update_rate": 200, **cm_controllers},
        },
        (f"/{ns}/arm_controller" if ns else "/arm_controller"): {
            "ros__parameters": {
                "joints": arm_joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        },
    }

    if effector_type == "agx_gripper":
        cm_controllers["gripper_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/gripper_controller" if ns else "/gripper_controller")] = {
            "ros__parameters": {
                "joints": ["gripper_joint1", "gripper_joint2"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }
    elif effector_type == "revo2":
        side = revo2_type
        ctrl_name = f"{side}_hand_controller"
        cm_controllers[ctrl_name] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/{ctrl_name}" if ns else f"/{ctrl_name}")] = {
            "ros__parameters": {
                "joints": [
                    f"{side}_thumb_metacarpal_joint",
                    f"{side}_thumb_proximal_joint",
                    f"{side}_index_proximal_joint",
                    f"{side}_middle_proximal_joint",
                    f"{side}_ring_proximal_joint",
                    f"{side}_pinky_proximal_joint",
                ],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }
    elif effector_type == "omnihand":
        side = omnihand_type
        ctrl_name = f"{side}_omnihand_controller"
        cm_controllers[ctrl_name] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/{ctrl_name}" if ns else f"/{ctrl_name}")] = {
            "ros__parameters": {
                "joints": omnihand_controller_joint_names(side),
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="ros2_controllers_", delete=False
    )
    yaml.dump(config, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _build_namespaced_moveit_rviz_config(package_path, namespace):
    base_rviz = package_path / "config/moveit.rviz"
    content = base_rviz.read_text(encoding="utf-8")

    ns = namespace.strip("/")
    move_group_ns = f"/{ns}" if ns else ""

    content = content.replace(
        'Move Group Namespace: ""',
        f'Move Group Namespace: "{move_group_ns}"',
    )

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".rviz", prefix="moveit_", delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def _resolved_moveit_rviz_fixed_frame(custom_model: str) -> str:
    return "body_base_link" if custom_model else "base_link"


def _build_moveit_rviz_config(package_path, namespace, fixed_frame):
    base_rviz = package_path / "config/moveit.rviz"
    content = base_rviz.read_text(encoding="utf-8")

    ns = namespace.strip("/")
    move_group_ns = f"/{ns}" if ns else ""

    content = content.replace(
        'Move Group Namespace: ""',
        f'Move Group Namespace: "{move_group_ns}"',
    )

    if fixed_frame != "base_link":
        content = content.replace("Fixed Frame: base_link", f"Fixed Frame: {fixed_frame}")
        content = content.replace("Target Frame: base_link", f"Target Frame: {fixed_frame}")

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".rviz", prefix="moveit_", delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def _build_moveit(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)
    moveit_profile = LaunchConfiguration("moveit_profile").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    omnihand_type = LaunchConfiguration("omnihand_type").perform(context)
    input_joint_prefix = LaunchConfiguration("input_joint_prefix").perform(context)
    use_mit_controller = LaunchConfiguration("use_mit_controller")
    moveit_config = build_moveit_config(context)
    package_path = moveit_config.package_path
    use_fake_execution = (
        moveit_profile != "both_arms"
        and context.launch_configurations.get("use_mit_controller", "false") != "true"
    )
    launch_fake_execution = "true" if use_fake_execution else "false"
    dual_arm_planning_only = (
        "true"
        if moveit_profile == "both_arms"
        and not use_fake_execution
        and context.launch_configurations.get("use_mit_controller", "false") != "true"
        else "false"
    )

    actions = []

    virtual_joints_launch = package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch))
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/rsp.launch.py")
            )
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={
                "planning_pipelines": LaunchConfiguration("planning_pipelines"),
            }.items(),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/moveit_rviz.launch.py")
            ),
            launch_arguments={
                "rviz_config": _build_moveit_rviz_config(
                    package_path,
                    namespace,
                    _resolved_moveit_rviz_fixed_frame(
                        LaunchConfiguration("custom_model").perform(context)
                    ),
                ),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    ros2_controllers_yaml = _build_ros2_controllers_file(
        arm_type,
        effector_type,
        revo2_type,
        omnihand_type,
        namespace,
        moveit_profile,
        input_joint_prefix,
    )
    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_yaml,
            ],
            remappings=[("joint_states", "control/joint_states")],
            condition=IfCondition(launch_fake_execution),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/spawn_controllers.launch.py")
            ),
            condition=IfCondition(launch_fake_execution),
        )
    )

    actions.append(
        LogInfo(
            msg=(
                "MoveIt MIT mode expects mit_controller to already provide "
                "arm_controller/follow_joint_trajectory."
            ),
            condition=IfCondition(use_mit_controller),
        )
    )

    actions.append(
        LogInfo(
            msg=(
                "MoveIt both_arms profile currently starts as a planning-only surface; "
                "fake ros2_control is skipped until the staged Duo model owns a compatible execution path."
            ),
            condition=IfCondition(dual_arm_planning_only),
        )
    )

    actions.append(
        ExecuteProcess(
            cmd=[
                "python3",
                str(package_path / "scripts" / "apply_simple_obstacles.py"),
                "--config",
                LaunchConfiguration("simple_obstacles_config"),
                "--namespace",
                namespace,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("load_simple_obstacles")),
        )
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                SetRemap(src="/robot_description", dst="robot_description"),
                *actions,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *declare_common_args(),
            DeclareBooleanLaunchArg(
                "db",
                default_value=False,
                description="By default, we do not start a database (it can be large)",
            ),
            DeclareBooleanLaunchArg(
                "debug",
                default_value=False,
                description="By default, we are not in debug mode",
            ),
            DeclareBooleanLaunchArg(
                "use_rviz",
                default_value=True,
                description="Start RViz automatically with MoveIt",
            ),
            DeclareBooleanLaunchArg(
                "use_mit_controller",
                default_value=False,
                description="Use MIT controller trajectory execution instead of the fake ros2_control path",
            ),
            DeclareLaunchArgument(
                "planning_pipelines",
                default_value="",
                description="Optional comma-separated planning pipeline whitelist forwarded to move_group.",
            ),
            DeclareBooleanLaunchArg(
                "load_simple_obstacles",
                default_value=False,
                description="Load the repo-owned simple obstacle baseline into the planning scene after MoveIt starts",
            ),
            DeclareLaunchArgument(
                "simple_obstacles_config",
                default_value=str(Path(__file__).parent.parent / "config" / "simple_obstacles.json"),
                description="Path to the simple obstacle JSON file applied when load_simple_obstacles:=true.",
            ),
            OpaqueFunction(function=_build_moveit),
        ]
    )
