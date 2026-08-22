from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prefer_source_file(*relative_parts: str) -> Path:
    package_share_dir = get_package_share_path('duo_body_description').resolve()
    installed_file = package_share_dir.joinpath(*relative_parts)

    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return installed_file

    source_file = workspace_root / 'src' / 'duo_body_description' / Path(*relative_parts)
    if source_file.is_file():
        return source_file
    return installed_file


DEFAULT_RVIZ_CONFIG = _prefer_source_file('rviz', 'display_duo_system.rviz')


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    model_path = _prefer_source_file('urdf', 'duo_system.urdf.xacro')

    body_mesh_xyz = LaunchConfiguration('body_mesh_xyz').perform(context).strip()
    body_mesh_rpy = LaunchConfiguration('body_mesh_rpy').perform(context).strip()
    left_arm_base_xyz = LaunchConfiguration('left_arm_base_xyz').perform(context).strip()
    left_arm_base_rpy = LaunchConfiguration('left_arm_base_rpy').perform(context).strip()
    right_arm_base_xyz = LaunchConfiguration('right_arm_base_xyz').perform(context).strip()
    right_arm_base_rpy = LaunchConfiguration('right_arm_base_rpy').perform(context).strip()

    xacro_command = [
        'xacro ',
        str(model_path),
        ' use_left_arm:=', LaunchConfiguration('use_left_arm'),
        ' use_left_hand:=', LaunchConfiguration('use_left_hand'),
        ' use_right_arm:=', LaunchConfiguration('use_right_arm'),
        ' use_right_hand:=', LaunchConfiguration('use_right_hand'),
    ]

    if body_mesh_xyz:
        xacro_command.extend([' body_mesh_xyz:=\"', body_mesh_xyz, '\"'])
    if body_mesh_rpy:
        xacro_command.extend([' body_mesh_rpy:=\"', body_mesh_rpy, '\"'])
    if left_arm_base_xyz:
        xacro_command.extend([' left_arm_base_xyz:=\"', left_arm_base_xyz, '\"'])
    if left_arm_base_rpy:
        xacro_command.extend([' left_arm_base_rpy:=\"', left_arm_base_rpy, '\"'])
    if right_arm_base_xyz:
        xacro_command.extend([' right_arm_base_xyz:=\"', right_arm_base_xyz, '\"'])
    if right_arm_base_rpy:
        xacro_command.extend([' right_arm_base_rpy:=\"', right_arm_base_rpy, '\"'])

    robot_description = ParameterValue(
        Command(xacro_command),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_left_arm',
                default_value='false',
                choices=['true', 'false'],
                description='Instantiate the left Nero arm chain.',
            ),
            DeclareLaunchArgument(
                'use_left_hand',
                default_value='false',
                choices=['true', 'false'],
                description='Attach the left OmniHand to the left arm chain.',
            ),
            DeclareLaunchArgument(
                'use_right_arm',
                default_value='true',
                choices=['true', 'false'],
                description='Instantiate the right Nero arm chain.',
            ),
            DeclareLaunchArgument(
                'use_right_hand',
                default_value='true',
                choices=['true', 'false'],
                description='Attach the right OmniHand to the right arm chain.',
            ),
            DeclareLaunchArgument(
                'body_mesh_xyz',
                default_value='',
                description='Optional visual and collision mesh translation relative to body_base_link in meters. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'body_mesh_rpy',
                default_value='',
                description='Optional visual and collision mesh roll, pitch, yaw relative to body_base_link in radians. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'left_arm_base_xyz',
                default_value='',
                description='Optional extra fixed translation from left mount link to left arm base link in meters. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'left_arm_base_rpy',
                default_value='',
                description='Optional extra fixed roll, pitch, yaw from left mount link to left arm base link in radians. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'right_arm_base_xyz',
                default_value='',
                description='Optional extra fixed translation from right mount link to right arm base link in meters. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'right_arm_base_rpy',
                default_value='',
                description='Optional extra fixed roll, pitch, yaw from right mount link to right arm base link in radians. Empty uses the Xacro default.',
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                choices=['true', 'false'],
                description='Use joint_state_publisher_gui for interactive joint sliders.',
            ),
            DeclareLaunchArgument(
                'pub_rate',
                default_value='200',
                description='Joint state publisher rate in Hz.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with the Duo body display config.',
            ),
            DeclareLaunchArgument(
                'rvizconfig',
                default_value=str(DEFAULT_RVIZ_CONFIG),
                description='Absolute path to the RViz config file.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )