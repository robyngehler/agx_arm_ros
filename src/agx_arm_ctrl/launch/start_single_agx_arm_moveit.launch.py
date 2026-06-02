from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _default_mit_params_file() -> str:
    package_share_dir = Path(get_package_share_directory('agx_arm_mit_controller')).resolve()
    installed_params_file = package_share_dir / 'config' / 'nero_mit_controller_defaults.yaml'

    try:
        workspace_root = package_share_dir.parents[3]
    except IndexError:
        return str(installed_params_file)

    source_params_file = workspace_root / 'src' / 'agx_arm_mit_controller' / 'config' / 'nero_mit_controller_defaults.yaml'
    if source_params_file.is_file():
        return str(source_params_file)
    return str(installed_params_file)

def generate_launch_description():

    default_mit_params_file = _default_mit_params_file()

    # ── arguments ────────────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.'
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

    moveit_profile_arg = DeclareLaunchArgument(
        'moveit_profile',
        default_value='nero_arm',
        choices=['nero_arm', 'right_arm', 'left_arm'],
        description='MoveIt planning profile. Use right_arm or left_arm for prefixed Duo custom-model bringup.',
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Optional SRDF robot name override for custom models. Empty defaults to duo_nero_system when custom_model is set, otherwise agx_arm.',
    )

    custom_model_arg = DeclareLaunchArgument(
        'custom_model',
        default_value='',
        description='Optional custom model path forwarded to MoveIt for prefixed body-mounted bringup.',
    )

    custom_model_xacro_args_arg = DeclareLaunchArgument(
        'custom_model_xacro_args',
        default_value='',
        description='Optional extra xacro args appended when custom_model is set.',
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

    input_joint_prefix_arg = DeclareLaunchArgument(
        'input_joint_prefix',
        default_value='',
        description='Optional prefix stripped from MoveIt trajectory joint names before forwarding to the MIT controller.',
    )

    feedback_joint_prefix_arg = DeclareLaunchArgument(
        'feedback_joint_prefix',
        default_value='',
        description='Optional prefix added onto follow-side feedback/joint_states for prefixed custom models. Empty falls back to input_joint_prefix.',
    )

    follow_joint_states_topic_arg = DeclareLaunchArgument(
        'follow_joint_states_topic',
        default_value='feedback/joint_states',
        description='JointState topic consumed by MoveIt when follow:=true. Override for prefixed custom-model feedback adaptation.',
    )

    arm_base_frame_arg = DeclareLaunchArgument(
        'arm_base_frame',
        default_value='',
        description='Optional arm base frame used by MoveIt for custom body-mounted models.',
    )

    arm_tip_frame_arg = DeclareLaunchArgument(
        'arm_tip_frame',
        default_value='',
        description='Optional arm tip frame used by MoveIt for custom body-mounted models.',
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
        default_value=default_mit_params_file,
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

    resolved_input_joint_prefix = PythonExpression([
        "'",
        LaunchConfiguration('input_joint_prefix'),
        "' if '",
        LaunchConfiguration('input_joint_prefix'),
        "' != '' else ('right_arm_' if '",
        LaunchConfiguration('moveit_profile'),
        "' == 'right_arm' else ('left_arm_' if '",
        LaunchConfiguration('moveit_profile'),
        "' == 'left_arm' else ''))",
    ])

    resolved_feedback_joint_prefix = PythonExpression([
        "'",
        LaunchConfiguration('feedback_joint_prefix'),
        "' if '",
        LaunchConfiguration('feedback_joint_prefix'),
        "' != '' else '",
        resolved_input_joint_prefix,
        "'",
    ])

    resolved_follow_joint_states_topic = PythonExpression([
        "'",
        LaunchConfiguration('follow_joint_states_topic'),
        "' if '",
        LaunchConfiguration('follow_joint_states_topic'),
        "' != 'feedback/joint_states' else ('feedback/prefixed_joint_states' if ('",
        LaunchConfiguration('follow'),
        "' == 'true' and ('",
        resolved_feedback_joint_prefix,
        "' != '')) else 'feedback/joint_states')",
    ])

    resolved_robot_name = PythonExpression([
        "'",
        LaunchConfiguration('robot_name'),
        "' if '",
        LaunchConfiguration('robot_name'),
        "' != '' else ('duo_nero_system' if '",
        LaunchConfiguration('custom_model'),
        "' != '' else 'agx_arm')",
    ])

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
            'input_joint_prefix': resolved_input_joint_prefix,
            'params_file': LaunchConfiguration('mit_params_file'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mit_controller')),
    )

    feedback_joint_state_adapter = Node(
        package='agx_arm_mit_tools',
        executable='agx_arm_joint_state_name_adapter',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'input_topic': 'feedback/joint_states',
            'output_topic': 'feedback/prefixed_joint_states',
            'joint_prefix': resolved_feedback_joint_prefix,
            'mode': 'prepend',
        }],
        condition=IfCondition(
            PythonExpression([
                "('",
                LaunchConfiguration('follow'),
                "' == 'true') and ('",
                resolved_feedback_joint_prefix,
                "' != '') and (('",
                LaunchConfiguration('follow_joint_states_topic'),
                "' == 'feedback/joint_states') or ('",
                LaunchConfiguration('follow_joint_states_topic'),
                "' == 'feedback/prefixed_joint_states'))",
            ])
        ),
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
            'moveit_profile': LaunchConfiguration('moveit_profile'),
            'robot_name': resolved_robot_name,
            'custom_model': LaunchConfiguration('custom_model'),
            'custom_model_xacro_args': LaunchConfiguration('custom_model_xacro_args'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'input_joint_prefix': resolved_input_joint_prefix,
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'arm_tip_frame': LaunchConfiguration('arm_tip_frame'),
            'follow': LaunchConfiguration('follow'),
            'follow_joint_states_topic': resolved_follow_joint_states_topic,
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
        moveit_profile_arg,
        robot_name_arg,
        custom_model_arg,
        custom_model_xacro_args_arg,
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
        input_joint_prefix_arg,
        feedback_joint_prefix_arg,
        follow_joint_states_topic_arg,
        arm_base_frame_arg,
        arm_tip_frame_arg,
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
        feedback_joint_state_adapter,
        moveit_launch,
    ])
