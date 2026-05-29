from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():

    # arg
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this arm instance (e.g. arm1).'
    )

    custom_model_arg = DeclareLaunchArgument(
        'custom_model',
        default_value='',
        description='Optional custom model path forwarded to display_control.launch.py.',
    )

    custom_model_xacro_args_arg = DeclareLaunchArgument(
        'custom_model_xacro_args',
        default_value='',
        description='Optional extra xacro args appended when custom_model is set.',
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the AGX Arm node.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='nero',
        choices=['nero'],
        description='Robotic arm type. Only nero is supported in this workspace.'
    )

    effector_type_arg = DeclareLaunchArgument(
        'effector_type',
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
        description='OmniHand type (e.g. left, right).'
    )

    launch_omnihand_bridge_arg = DeclareLaunchArgument(
        'launch_omnihand_bridge',
        default_value='false',
        choices=['true', 'false'],
        description='Launch the repo-owned OmniHand bridge when effector_type is omnihand.'
    )

    omnihand_backend_type_arg = DeclareLaunchArgument(
        'omnihand_backend_type',
        default_value='mock',
        description='Backend type for the repo-owned OmniHand bridge.'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically enable the AGX Arm node.'
    )

    fast_mode_arg = DeclareLaunchArgument(
        'fast_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable fast mode for the AGX Arm node.'
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

    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='true',
        choices=['true', 'false'],
        description='Follow real arm state.',
    )

    control_arg = DeclareLaunchArgument(
        'control',
        default_value='false',
        choices=['true', 'false'],
        description='Whether to publish control topics from the RViz-side joint state publisher.',
    )

    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset in x, y, z, roll, pitch, yaw in meters/radians.'
    )

    gripper_default_effort_arg = DeclareLaunchArgument(
        'gripper_default_effort',
        default_value='1.0',
        description='Default effort for gripper commands (>= 0.0).'
    )

    use_mit_controller_arg = DeclareLaunchArgument(
        'use_mit_controller',
        default_value='true',
        choices=['true', 'false'],
        description='Route RViz control-topic commands through the custom MIT controller.',
    )

    mit_control_rate_arg = DeclareLaunchArgument(
        'mit_control_rate_hz',
        default_value='100.0',
        description='MIT controller update rate when use_mit_controller is true.',
    )

    mit_params_file_arg = DeclareLaunchArgument(
        'mit_params_file',
        default_value='',
        description='Optional MIT controller params file override when use_mit_controller is true.',
    )

    mit_joint_target_duration_arg = DeclareLaunchArgument(
        'mit_joint_target_duration_s',
        default_value='0.75',
        description='Duration in seconds for RViz joint-slider soft MIT targets.',
    )

    input_joint_prefix_arg = DeclareLaunchArgument(
        'input_joint_prefix',
        default_value='',
        description='Optional prefix stripped from RViz-side joint names before forwarding MIT debug trajectories.',
    )

    # description: use the sim-backed compatibility launch from agx_arm_description
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_description'),
                'launch',
                'display_control.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'arm_type': LaunchConfiguration('arm_type'),
            'custom_model': LaunchConfiguration('custom_model'),
            'custom_model_xacro_args': LaunchConfiguration('custom_model_xacro_args'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'follow': LaunchConfiguration('follow'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'control': LaunchConfiguration('control'),
            'control_topic': PythonExpression([
                "'mit_controller/soft_target_joint_states' if '",
                LaunchConfiguration('use_mit_controller'),
                "' == 'true' else 'control/joint_states'",
            ]),
        }.items(),
    )

    # agx_arm
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('effector_type'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'launch_omnihand_bridge': LaunchConfiguration('launch_omnihand_bridge'),
            'omnihand_backend_type': LaunchConfiguration('omnihand_backend_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_mit_controller')),
    )

    mit_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_mit_controller'),
                'launch',
                'start_nero_mit_controller.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'launch_omnihand_bridge': LaunchConfiguration('launch_omnihand_bridge'),
            'omnihand_backend_type': LaunchConfiguration('omnihand_backend_type'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'control_rate_hz': LaunchConfiguration('mit_control_rate_hz'),
            'params_file': LaunchConfiguration('mit_params_file'),
            'log_level': LaunchConfiguration('log_level'),
            'enable_debug_joint_trajectory_topic': LaunchConfiguration('control'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mit_controller')),
    )

    mit_joint_state_bridge = Node(
        package='agx_arm_mit_tools',
        executable='agx_arm_mit_joint_state_bridge',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'segment_duration_s': LaunchConfiguration('mit_joint_target_duration_s'),
            'input_joint_prefix': LaunchConfiguration('input_joint_prefix'),
            'auto_enable': False,
        }],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_mit_controller'),
                "' == 'true' and '",
                LaunchConfiguration('control'),
                "' == 'true'",
            ])
        ),
    )

    return LaunchDescription([
        # arguments
        log_level_arg,
        namespace_arg,
        custom_model_arg,
        custom_model_xacro_args_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        revo2_type_arg,
        omnihand_type_arg,
        launch_omnihand_bridge_arg,
        omnihand_backend_type_arg,
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        gripper_default_effort_arg,
        follow_arg,
        control_arg,
        use_mit_controller_arg,
        mit_control_rate_arg,
        mit_params_file_arg,
        mit_joint_target_duration_arg,
        input_joint_prefix_arg,
        # description
        description_launch,
        # agx_arm
        agx_arm_launch,
        mit_arm_launch,
        mit_joint_state_bridge,
    ])
