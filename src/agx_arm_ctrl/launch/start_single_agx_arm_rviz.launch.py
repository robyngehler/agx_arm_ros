from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# arm_type → (description_package, launch_relative_path)
DESCRIPTION_LAUNCH_MAP = {
    'piper':   ('piper_description',   'launch/piper_with_gripper/display_xacro.launch.py'),
    'nero':    ('nero_description',    'launch/display_xacro.launch.py'),
    'piper_x': ('piper_x_description', 'launch/display_xacro.launch.py'),
    'piper_h': ('piper_h_description', 'launch/display_xacro.launch.py'),
    'piper_l': ('piper_l_description', 'launch/display_xacro.launch.py'),
}


def _launch_description(context):
    """Resolve description launch based on arm_type at runtime."""
    arm_type = LaunchConfiguration('arm_type').perform(context)

    if arm_type not in DESCRIPTION_LAUNCH_MAP:
        raise ValueError(
            f"Unknown arm_type '{arm_type}', "
            f"valid options: {list(DESCRIPTION_LAUNCH_MAP.keys())}"
        )

    pkg, launch_rel = DESCRIPTION_LAUNCH_MAP[arm_type]
    launch_path = os.path.join(get_package_share_directory(pkg), launch_rel)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={'gui': 'true'}.items(),
        )
    ]


def generate_launch_description():

    # arg
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the AGX Arm node.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        description='Type of robotic arm.',
        choices=['piper', 'nero', 'piper_x', 'piper_h', 'piper_l']
    )

    effector_type_arg = DeclareLaunchArgument(
        'effector_type',
        default_value='none',
        description='End effector type.',
        choices=['none', 'agx_gripper', 'revo2']
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='True',
        description='Automatically enable the AGX Arm node.'
    )

    installation_pos_arg = DeclareLaunchArgument(
        'installation_pos',
        default_value='horizontal',
        description='Installation position of the arm.',
        choices=['horizontal', 'left', 'right']
    )

    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Movement speed as a percentage of maximum speed.'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )

    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Timeout in seconds for arm enable/disable operations.'
    )

    payload_arg = DeclareLaunchArgument(
        'payload',
        default_value='empty',
        description='Payload type.',
        choices=['empty', 'half', 'full']
    )

    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset in x, y, z, roll, pitch, yaw in meters/radians.'
    )

    # description
    description_launch = OpaqueFunction(function=_launch_description)

    # node
    agx_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'installation_pos': LaunchConfiguration('installation_pos'),
            'effector_type': LaunchConfiguration('effector_type'),
            'payload': LaunchConfiguration('payload'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
        }],
        remappings=[
            # feedback topics
            ('/feedback/joint_states', '/feedback/joint_states'),
            ('/feedback/tcp_pose', '/feedback/tcp_pose'),
            ('/feedback/arm_status', '/feedback/arm_status'),
            ('/feedback/arm_ctrl_states', '/feedback/arm_ctrl_states'),
            ('/feedback/gripper_status', '/feedback/gripper_status'),
            ('/feedback/hand_status', '/feedback/hand_status'),

            # control topics
            ('/control/joint_states', '/joint_states'),
            ('/control/move_j', '/control/move_j'),
            ('/control/move_p', '/control/move_p'),
            ('/control/move_l', '/control/move_l'),
            ('/control/move_c', '/control/move_c'),
            ('/control/move_mit', '/control/move_mit'),
            ('/control/move_js', '/control/move_js'),
            ('/control/gripper', '/control/gripper'),
            ('/control/hand', '/control/hand'),
            ('/control/hand_position_time', '/control/hand_position_time'),

            # services
            ('/enable_agx_arm', '/enable_agx_arm'),
            ('/move_home', '/move_home'),
            ('/exit_teach_mode', '/exit_teach_mode'),
        ]
    )

    return LaunchDescription([
        # arguments
        log_level_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        auto_enable_arg,
        installation_pos_arg,
        speed_percent_arg,
        pub_rate_arg,
        enable_timeout_arg,
        payload_arg,
        tcp_offset_arg,
        # description
        description_launch,
        # node
        agx_arm_node,
    ])

