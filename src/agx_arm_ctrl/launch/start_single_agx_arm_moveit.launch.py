from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():

    # ── arguments ────────────────────────────────────────────────────
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

    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='true',
        choices=['true', 'false'],
        description='Follow real arm state.',
    )

    use_mit_controller_arg = DeclareLaunchArgument(
        'use_mit_controller',
        default_value='true',
        choices=['true', 'false'],
        description='Route arm trajectory execution through the custom MIT controller.',
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

    load_simple_obstacles_arg = DeclareLaunchArgument(
        'load_simple_obstacles',
        default_value='false',
        choices=['true', 'false'],
        description='Seed the MoveIt planning scene with the repo-owned simple obstacle set.',
    )

    simple_obstacles_config_arg = DeclareLaunchArgument(
        'simple_obstacles_config',
        default_value=os.path.join(
            get_package_share_directory('agx_arm_moveit'),
            'config',
            'simple_obstacles.json',
        ),
        description='Path to the JSON file with simple planning-scene obstacles.',
    )

    # ── agx_arm_ctrl ─────────────────────────────────────────────────
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            )
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
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
            'publish_gripper_joint': 'false',
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
            'publish_gripper_joint': 'false',
            'control_rate_hz': LaunchConfiguration('mit_control_rate_hz'),
            'params_file': LaunchConfiguration('mit_params_file'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mit_controller')),
    )

    # ── agx_arm_moveit ───────────────────────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_moveit'),
                'launch',
                'demo.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'follow': LaunchConfiguration('follow'),
            'use_mit_controller': LaunchConfiguration('use_mit_controller'),
            'load_simple_obstacles': LaunchConfiguration('load_simple_obstacles'),
            'simple_obstacles_config': LaunchConfiguration('simple_obstacles_config'),
        }.items(),
    )

    return LaunchDescription([
        # arguments
        log_level_arg,
        namespace_arg,
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
        use_mit_controller_arg,
        mit_control_rate_arg,
        mit_params_file_arg,
        load_simple_obstacles_arg,
        simple_obstacles_config_arg,
        # launches
        agx_arm_launch,
        mit_arm_launch,
        moveit_launch,
    ])
