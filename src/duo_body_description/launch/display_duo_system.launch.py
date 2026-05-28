from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    model_path = (
        get_package_share_path('duo_body_description') / 'urdf' / 'duo_system.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                str(model_path),
                ' use_left_arm:=', LaunchConfiguration('use_left_arm'),
                ' use_left_hand:=', LaunchConfiguration('use_left_hand'),
                ' use_right_arm:=', LaunchConfiguration('use_right_arm'),
                ' use_right_hand:=', LaunchConfiguration('use_right_hand'),
            ]
        ),
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
                'gui',
                default_value='true',
                choices=['true', 'false'],
                description='Use joint_state_publisher_gui for interactive joint sliders.',
            ),
            DeclareLaunchArgument(
                'pub_rate',
                default_value='50',
                description='Joint state publisher rate in Hz.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz without a preloaded config.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )