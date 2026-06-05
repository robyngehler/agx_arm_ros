import ast
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARM_TYPES = ('nero',)

MOVEIT_PROFILE_DEFAULTS = {
    'nero_arm': {
        'planning_group_name': 'nero_arm',
        'input_joint_prefix': '',
        'arm_base_frame': 'base_link',
        'arm_tip_frame': 'tcp_link',
    },
    'right_arm': {
        'planning_group_name': 'right_arm',
        'input_joint_prefix': 'right_arm_',
        'arm_base_frame': 'right_arm_base_link',
        'arm_tip_frame': 'right_arm_nero_tool0',
    },
    'left_arm': {
        'planning_group_name': 'left_arm',
        'input_joint_prefix': 'left_arm_',
        'arm_base_frame': 'left_arm_base_link',
        'arm_tip_frame': 'left_arm_nero_tool0',
    },
    'both_arms': {
        'planning_group_name': 'both_arms',
        'input_joint_prefix': '',
        'arm_base_frame': '',
        'arm_tip_frame': '',
    },
}

ROBOT_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_description.urdf'
    for arm_type in ARM_TYPES
}

ROBOT_WITH_GRIPPER_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_with_gripper_description.xacro'
    for arm_type in ARM_TYPES
}

ROBOT_WITH_REVO2_URDF_MAP = {
    arm_type: {
        side: f'{arm_type}/urdf/{arm_type}_with_{side}_revo2_description.xacro'
        for side in ('left', 'right')
    }
    for arm_type in ARM_TYPES
}

ROBOT_WITH_OMNIHAND_URDF_MAP = {
    arm_type: {
        side: f'{arm_type}/urdf/{arm_type}_with_{side}_omnihand_description.xacro'
        for side in ('left', 'right')
    }
    for arm_type in ARM_TYPES
}


def _resolve_custom_model_path(pkg_path, custom_model):
    custom_path = pkg_path / 'agx_arm_urdf' / custom_model
    if custom_path.exists() and custom_path.is_file():
        return str(custom_path)

    return str(Path(custom_model).expanduser())


def _resolve_builtin_model_path(
    arm_type, effector_type, revo2_type, omnihand_type, pkg_path
):
    if effector_type == 'agx_gripper':
        relative_path = ROBOT_WITH_GRIPPER_URDF_MAP[arm_type]
    elif effector_type == 'revo2':
        relative_path = ROBOT_WITH_REVO2_URDF_MAP[arm_type][revo2_type]
    elif effector_type == 'omnihand':
        relative_path = ROBOT_WITH_OMNIHAND_URDF_MAP[arm_type][omnihand_type]
    else:
        relative_path = ROBOT_URDF_MAP[arm_type]
    return str(pkg_path / 'agx_arm_urdf' / relative_path)


def _flange_link(_arm_type):
    return 'nero_tool0'


def _default_rviz_config_path(pkg_path):
    display_config = pkg_path / 'rviz' / 'display.rviz'
    if display_config.exists():
        return display_config
    return pkg_path / 'rviz' / 'default.rviz'


def _resolved_moveit_profile(moveit_profile: str) -> str:
    return moveit_profile if moveit_profile in MOVEIT_PROFILE_DEFAULTS else 'nero_arm'


def _resolved_joint_prefix(moveit_profile: str, input_joint_prefix: str) -> str:
    if input_joint_prefix:
        return input_joint_prefix
    return MOVEIT_PROFILE_DEFAULTS[_resolved_moveit_profile(moveit_profile)]['input_joint_prefix']


def _resolved_arm_base_frame(custom_model: str, moveit_profile: str, input_joint_prefix: str, explicit_frame: str) -> str:
    if explicit_frame:
        return explicit_frame
    if custom_model:
        return f'{_resolved_joint_prefix(moveit_profile, input_joint_prefix)}base_link'
    return MOVEIT_PROFILE_DEFAULTS[_resolved_moveit_profile(moveit_profile)]['arm_base_frame']


def _resolved_arm_tip_frame(custom_model: str, moveit_profile: str, input_joint_prefix: str, explicit_frame: str) -> str:
    if explicit_frame:
        return explicit_frame
    if custom_model:
        joint_prefix = _resolved_joint_prefix(moveit_profile, input_joint_prefix)
        return f'{joint_prefix}nero_tool0' if joint_prefix else 'nero_tool0'
    return MOVEIT_PROFILE_DEFAULTS[_resolved_moveit_profile(moveit_profile)]['arm_tip_frame']


def _resolve_include_end_effector_groups(custom_model: str, moveit_profile: str) -> str:
    if custom_model and _resolved_moveit_profile(moveit_profile) == 'both_arms':
        return 'false'
    return 'true'


def _robot_description_semantic_command(
    robot_name: str,
    arm_type: str,
    effector_type: str,
    revo2_type: str,
    omnihand_type: str,
    custom_model: str,
    moveit_profile: str,
    input_joint_prefix: str,
    arm_base_frame: str,
    arm_tip_frame: str,
):
    moveit_pkg_path = get_package_share_path('agx_arm_moveit')
    srdf_path = moveit_pkg_path / 'config' / 'agx_arm.srdf.xacro'
    resolved_profile = _resolved_moveit_profile(moveit_profile)
    planning_group_name = MOVEIT_PROFILE_DEFAULTS[resolved_profile]['planning_group_name']
    resolved_joint_prefix = _resolved_joint_prefix(moveit_profile, input_joint_prefix)
    resolved_arm_base_frame = _resolved_arm_base_frame(custom_model, moveit_profile, input_joint_prefix, arm_base_frame)
    resolved_arm_tip_frame = _resolved_arm_tip_frame(custom_model, moveit_profile, input_joint_prefix, arm_tip_frame)
    end_effector_parent_link = resolved_arm_tip_frame if custom_model else 'tcp_link'
    mount_body_link = 'body_base_link' if custom_model else ''

    return Command([
        'xacro ', str(srdf_path), ' ',
        f'robot_name:={robot_name}', ' ',
        f'arm_type:={arm_type}', ' ',
        f'effector_type:={effector_type}', ' ',
        f'revo2_type:={revo2_type}', ' ',
        f'omnihand_type:={omnihand_type}', ' ',
        f'planning_group_name:={planning_group_name}', ' ',
        f'arm_base_frame:={resolved_arm_base_frame}', ' ',
        f'arm_tip_frame:={resolved_arm_tip_frame}', ' ',
        f'end_effector_parent_link:={end_effector_parent_link}', ' ',
        f'mount_body_link:={mount_body_link}', ' ',
        f'arm_joint_prefix:={resolved_joint_prefix}', ' ',
        f'arm_link_prefix:={resolved_joint_prefix}', ' ',
        f'include_end_effector_groups:={_resolve_include_end_effector_groups(custom_model, moveit_profile)}', ' ',
        f'include_dual_arm_groups:={"true" if resolved_profile == "both_arms" else "false"}', ' ',
        'left_arm_base_frame:=left_arm_base_link ',
        'left_arm_tip_frame:=left_arm_nero_tool0 ',
        f'left_mount_body_link:={mount_body_link}', ' ',
        'left_arm_joint_prefix:=left_arm_ ',
        'left_arm_link_prefix:=left_arm_ ',
        'right_arm_base_frame:=right_arm_base_link ',
        'right_arm_tip_frame:=right_arm_nero_tool0 ',
        f'right_mount_body_link:={mount_body_link}', ' ',
        'right_arm_joint_prefix:=right_arm_ ',
        'right_arm_link_prefix:=right_arm_ ',
    ])


def _robot_description_semantic(*args, **kwargs):
    return ParameterValue(
        _robot_description_semantic_command(*args, **kwargs),
        value_type=str,
    )


def _resolved_rviz_fixed_frame(custom_model: str) -> str:
    return 'body_base_link' if custom_model else 'base_link'


def _build_rviz_config(base_config_path: str, fixed_frame: str) -> str:
    if fixed_frame == 'base_link':
        return base_config_path

    content = Path(base_config_path).read_text(encoding='utf-8')
    content = content.replace('Fixed Frame: base_link', f'Fixed Frame: {fixed_frame}')
    content = content.replace('Target Frame: base_link', f'Target Frame: {fixed_frame}')

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.rviz', prefix='display_control_', delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def _write_temp_text_file(content: str, suffix: str, prefix: str) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=suffix, prefix=prefix, delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def resolve_model_path(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    arm_type = LaunchConfiguration('arm_type').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    moveit_profile = LaunchConfiguration('moveit_profile').perform(context)
    effector_type = LaunchConfiguration('effector_type').perform(context)
    revo2_type = LaunchConfiguration('revo2_type').perform(context)
    omnihand_type = LaunchConfiguration('omnihand_type').perform(context)
    custom_model = LaunchConfiguration('custom_model').perform(context)
    custom_model_xacro_args = LaunchConfiguration('custom_model_xacro_args')
    input_joint_prefix = LaunchConfiguration('input_joint_prefix').perform(context)
    arm_base_frame = LaunchConfiguration('arm_base_frame').perform(context)
    arm_tip_frame = LaunchConfiguration('arm_tip_frame').perform(context)
    follow = LaunchConfiguration('follow').perform(context)
    follow_joint_states_topic = LaunchConfiguration('follow_joint_states_topic').perform(context)
    control = LaunchConfiguration('control').perform(context)
    control_topic = LaunchConfiguration('control_topic').perform(context)
    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    tcp_parent_frame = LaunchConfiguration('tcp_parent_frame').perform(context)
    tcp_offset = ast.literal_eval(
        LaunchConfiguration('tcp_offset').perform(context)
    )
    pkg_path = get_package_share_path('agx_arm_description')

    if custom_model:
        model_path = _resolve_custom_model_path(pkg_path, custom_model)
        robot_description = ParameterValue(
            Command(['xacro ', model_path, ' ', custom_model_xacro_args]),
            value_type=str,
        )
    else:
        model_path = _resolve_builtin_model_path(
            arm_type, effector_type, revo2_type, omnihand_type, pkg_path
        )
        robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    robot_description_semantic_command = _robot_description_semantic_command(
        robot_name,
        arm_type,
        effector_type,
        revo2_type,
        omnihand_type,
        custom_model,
        moveit_profile,
        input_joint_prefix,
        arm_base_frame,
        arm_tip_frame,
    )
    semantic_description_path = _write_temp_text_file(
        robot_description_semantic_command.perform(context),
        '.srdf',
        'display_control_semantic_',
    )
    semantic_publisher_cmd = [
        '/usr/bin/python3', str(pkg_path / 'scripts' / 'semantic_description_publisher.py'),
        '--file-path', semantic_description_path,
        '--topic', 'robot_description_semantic',
    ]
    if namespace:
        normalized_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        semantic_publisher_cmd.extend(['--ros-args', '-r', f'__ns:={normalized_namespace}'])

    state_joint_topic = (
        follow_joint_states_topic if follow == 'true' else str(control_topic)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', state_joint_topic)]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
        remappings=[('joint_states', str(control_topic))]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
        remappings=[('joint_states', str(control_topic))]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', _build_rviz_config(rviz_config, _resolved_rviz_fixed_frame(custom_model))],
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': _robot_description_semantic(
                robot_name,
                arm_type,
                effector_type,
                revo2_type,
                omnihand_type,
                custom_model,
                moveit_profile,
                input_joint_prefix,
                arm_base_frame,
                arm_tip_frame,
            ),
        }],
        remappings=[('/robot_description', 'robot_description')],
    )

    semantic_publisher_process = ExecuteProcess(
        cmd=semantic_publisher_cmd,
        output='screen',
    )

    nodes = [
        robot_state_publisher_node,
        semantic_publisher_process,
        rviz_node,
    ]

    if control == 'true':
        nodes = [
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            *nodes,
        ]

    if not custom_model or tcp_parent_frame:
        flange = tcp_parent_frame or _flange_link(arm_type)
        x, y, z, rx, ry, rz = tcp_offset
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=namespace,
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--roll', str(rx), '--pitch', str(ry), '--yaw', str(rz),
                    '--frame-id', flange, '--child-frame-id', 'tcp_link',
                ],
            )
        )

    return nodes


def generate_launch_description():
    pkg_path = get_package_share_path('agx_arm_description')
    default_rviz_config_path = _default_rviz_config_path(pkg_path)

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.'
    )
    arm_type_arg = DeclareLaunchArgument(
        name='arm_type',
        default_value='nero',
        choices=list(ROBOT_URDF_MAP.keys()),
        description='Robotic arm type. Only nero is supported in this workspace.'
    )
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='agx_arm',
        description='Robot name used in the generated SRDF for RViz displays that need semantic data.'
    )
    moveit_profile_arg = DeclareLaunchArgument(
        name='moveit_profile',
        default_value='nero_arm',
        choices=list(MOVEIT_PROFILE_DEFAULTS.keys()),
        description='MoveIt planning profile used to render the semantic robot model for RViz.'
    )
    custom_model_arg = DeclareLaunchArgument(
        name='custom_model',
        default_value='',
        description='Optional custom model path. Prefer absolute path, or relative path under agx_arm_urdf/. If set, arm_type and effector_type are ignored.'
    )
    custom_model_xacro_args_arg = DeclareLaunchArgument(
        name='custom_model_xacro_args',
        default_value='',
        description='Optional extra xacro args appended when custom_model is set.'
    )
    effector_type_arg = DeclareLaunchArgument(
        name='effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2', 'omnihand'],
        description='End effector type (e.g. agx_gripper, revo2, omnihand).'
    )
    revo2_type_arg = DeclareLaunchArgument(
       'revo2_type',
        default_value='left',
        choices=['left', 'right'],
        description='Revo2 end effector type (e.g. left, right).'
    )
    omnihand_type_arg = DeclareLaunchArgument(
       'omnihand_type',
        default_value='left',
        choices=['left', 'right'],
        description='OmniHand side (used when effector_type is omnihand).'
    )
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the joint state publisher.'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )
    follow_arg = DeclareLaunchArgument(
        name='follow',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable follow mode'
    )
    follow_joint_states_topic_arg = DeclareLaunchArgument(
        name='follow_joint_states_topic',
        default_value='feedback/joint_states',
        description='JointState topic consumed when follow:=true. Use a prefixed adapter topic for custom multi-arm models.',
    )
    input_joint_prefix_arg = DeclareLaunchArgument(
        name='input_joint_prefix',
        default_value='',
        description='Optional arm-joint prefix used by prefixed custom models when generating semantic RViz data.'
    )
    arm_base_frame_arg = DeclareLaunchArgument(
        name='arm_base_frame',
        default_value='',
        description='Optional arm base frame override for the generated RViz semantic model.'
    )
    arm_tip_frame_arg = DeclareLaunchArgument(
        name='arm_tip_frame',
        default_value='',
        description='Optional arm tip frame override for the generated RViz semantic model.'
    )
    control_arg = DeclareLaunchArgument(
        name='control',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable publishing control topics.'
    )
    control_topic_arg = DeclareLaunchArgument(
        name='control_topic',
        default_value='control/joint_states',
        description='Topic to publish joint slider targets.'
    )
    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset [x, y, z, rx, ry, rz] in meters/radians.'
    )
    tcp_parent_frame_arg = DeclareLaunchArgument(
        'tcp_parent_frame',
        default_value='',
        description='Optional parent frame for the tcp_offset static transform. Set this for custom models whose flange frame is not the built-in arm default.',
    )

    return LaunchDescription([
        namespace_arg,
        arm_type_arg,
        robot_name_arg,
        moveit_profile_arg,
        custom_model_arg,
        custom_model_xacro_args_arg,
        effector_type_arg,
        revo2_type_arg,
        omnihand_type_arg,
        pub_rate_arg,
        gui_arg,
        rviz_arg,
        follow_arg,
        follow_joint_states_topic_arg,
        input_joint_prefix_arg,
        arm_base_frame_arg,
        arm_tip_frame_arg,
        control_arg,
        control_topic_arg,
        tcp_offset_arg,
        tcp_parent_frame_arg,
        OpaqueFunction(function=resolve_model_path),
    ])