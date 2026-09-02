from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from agx_arm_ctrl.execution_profiles import resolve_execution_profile

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


def _resolved_argument(context, profile_values: dict[str, str], name: str) -> str:
    if name in profile_values:
        return profile_values[name]
    return LaunchConfiguration(name).perform(context).strip()


def _resolved_follow_joint_states_topic(
    follow: str,
    explicit_topic: str,
    input_joint_prefix: str,
    feedback_joint_prefix: str,
) -> str:
    if explicit_topic and explicit_topic != 'feedback/joint_states':
        return explicit_topic
    if follow == 'true' and (feedback_joint_prefix or input_joint_prefix):
        return 'feedback/prefixed_joint_states'
    return 'feedback/joint_states'


def _resolved_tcp_parent_frame(explicit_parent_frame: str, custom_model: str, input_joint_prefix: str) -> str:
    if explicit_parent_frame:
        return explicit_parent_frame
    if custom_model and input_joint_prefix:
        return f'{input_joint_prefix}nero_tool0'
    return ''


def _launch_actions(context):
    profile_values = resolve_execution_profile(
        LaunchConfiguration('execution_profile').perform(context).strip(),
        allow_multi_arm=False,
    )

    namespace = LaunchConfiguration('namespace').perform(context).strip()
    log_level = LaunchConfiguration('log_level').perform(context).strip()
    custom_model = _resolved_argument(context, profile_values, 'custom_model')
    custom_model_xacro_args = _resolved_argument(context, profile_values, 'custom_model_xacro_args')
    robot_name = _resolved_argument(context, profile_values, 'robot_name') or 'agx_arm'
    moveit_profile = _resolved_argument(context, profile_values, 'moveit_profile') or 'nero_arm'
    # Explicit can_port wins, then the profile's registry-derived side bus,
    # then the legacy right-bus fallback (a left profile must not silently
    # drive the right bus).
    can_port = (
        LaunchConfiguration('can_port').perform(context).strip()
        or profile_values.get('can_port', '')
        or 'can_nero_right'
    )
    arm_type = LaunchConfiguration('arm_type').perform(context).strip()
    effector_type = _resolved_argument(context, profile_values, 'effector_type')
    revo2_type = _resolved_argument(context, profile_values, 'revo2_type')
    omnihand_type = _resolved_argument(context, profile_values, 'omnihand_type')
    launch_omnihand_bridge = _resolved_argument(context, profile_values, 'launch_omnihand_bridge')
    omnihand_backend_type = LaunchConfiguration('omnihand_backend_type').perform(context).strip()
    auto_enable = LaunchConfiguration('auto_enable').perform(context).strip()
    fast_mode = LaunchConfiguration('fast_mode').perform(context).strip()
    speed_percent = LaunchConfiguration('speed_percent').perform(context).strip()
    pub_rate = LaunchConfiguration('pub_rate').perform(context).strip()
    enable_timeout = LaunchConfiguration('enable_timeout').perform(context).strip()
    follow = LaunchConfiguration('follow').perform(context).strip()
    control = LaunchConfiguration('control').perform(context).strip()
    tcp_offset = LaunchConfiguration('tcp_offset').perform(context).strip()
    gripper_default_effort = LaunchConfiguration('gripper_default_effort').perform(context).strip()
    use_mit_controller = LaunchConfiguration('use_mit_controller').perform(context).strip() == 'true'
    mit_control_rate_hz = LaunchConfiguration('mit_control_rate_hz').perform(context).strip()
    mit_params_file = LaunchConfiguration('mit_params_file').perform(context).strip()
    mit_joint_target_duration_s = LaunchConfiguration('mit_joint_target_duration_s').perform(context).strip()
    input_joint_prefix = _resolved_argument(context, profile_values, 'input_joint_prefix')
    feedback_joint_prefix = _resolved_argument(context, profile_values, 'feedback_joint_prefix')
    follow_joint_states_topic = _resolved_follow_joint_states_topic(
        follow,
        LaunchConfiguration('follow_joint_states_topic').perform(context).strip(),
        input_joint_prefix,
        feedback_joint_prefix,
    )
    tcp_parent_frame = _resolved_tcp_parent_frame(
        _resolved_argument(context, profile_values, 'tcp_parent_frame'),
        custom_model,
        input_joint_prefix,
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('agx_arm_description'),
                    'launch',
                    'display_control.launch.py',
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'arm_type': arm_type,
                'robot_name': robot_name,
                'moveit_profile': moveit_profile,
                'custom_model': custom_model,
                'custom_model_xacro_args': custom_model_xacro_args,
                'effector_type': effector_type,
                'revo2_type': revo2_type,
                'omnihand_type': omnihand_type,
                'pub_rate': pub_rate,
                'follow': follow,
                'follow_joint_states_topic': follow_joint_states_topic,
                'input_joint_prefix': input_joint_prefix,
                'arm_base_frame': _resolved_argument(context, profile_values, 'arm_base_frame'),
                'arm_tip_frame': _resolved_argument(context, profile_values, 'arm_tip_frame'),
                'tcp_offset': tcp_offset,
                'tcp_parent_frame': tcp_parent_frame,
                'control': control,
                'control_topic': 'mit_controller/soft_target_joint_states' if use_mit_controller else 'control/joint_states',
            }.items(),
        )
    ]

    if use_mit_controller:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('agx_arm_mit_controller'),
                        'launch',
                        'start_nero_mit_controller.launch.py',
                    )
                ),
                launch_arguments={
                    'namespace': namespace,
                    'can_port': can_port,
                    'arm_type': arm_type,
                    'custom_model': custom_model,
                    'custom_model_xacro_args': custom_model_xacro_args,
                    'effector_type': effector_type,
                    'omnihand_type': omnihand_type,
                    'launch_omnihand_bridge': launch_omnihand_bridge,
                    'omnihand_backend_type': omnihand_backend_type,
                    'omnihand_device_id': LaunchConfiguration('omnihand_device_id').perform(context),
                    'omnihand_canfd_id': LaunchConfiguration('omnihand_canfd_id').perform(context),
                    'omnihand_sdk_cfg_path': LaunchConfiguration('omnihand_sdk_cfg_path').perform(context),
                    'auto_enable': auto_enable,
                    'fast_mode': fast_mode,
                    'speed_percent': speed_percent,
                    'pub_rate': pub_rate,
                    'enable_timeout': enable_timeout,
                    'tcp_offset': tcp_offset,
                    'gripper_default_effort': gripper_default_effort,
                    'control_rate_hz': mit_control_rate_hz,
                    'params_file': mit_params_file,
                    'log_level': log_level,
                    'input_joint_prefix': input_joint_prefix,
                    'joint_states_command_topic': 'mit_controller/soft_target_joint_states' if control == 'true' else 'control/joint_states',
                    'enable_debug_joint_trajectory_topic': control,
                }.items(),
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('agx_arm_ctrl'),
                        'launch',
                        'start_single_agx_arm.launch.py',
                    )
                ),
                launch_arguments={
                    'namespace': namespace,
                    'can_port': can_port,
                    'pub_rate': pub_rate,
                    'auto_enable': auto_enable,
                    'fast_mode': fast_mode,
                    'arm_type': arm_type,
                    'speed_percent': speed_percent,
                    'enable_timeout': enable_timeout,
                    'effector_type': effector_type,
                    'omnihand_type': omnihand_type,
                    'launch_omnihand_bridge': launch_omnihand_bridge,
                    'omnihand_backend_type': omnihand_backend_type,
                    'omnihand_device_id': LaunchConfiguration('omnihand_device_id').perform(context),
                    'omnihand_canfd_id': LaunchConfiguration('omnihand_canfd_id').perform(context),
                    'omnihand_sdk_cfg_path': LaunchConfiguration('omnihand_sdk_cfg_path').perform(context),
                    'joint_states_command_topic': 'control/joint_states',
                    'tcp_offset': tcp_offset,
                    'gripper_default_effort': gripper_default_effort,
                }.items(),
            )
        )

    if use_mit_controller and control == 'true':
        actions.append(
            Node(
                package='agx_arm_mit_tools',
                executable='agx_arm_mit_joint_state_bridge',
                namespace=namespace,
                parameters=[{
                    'segment_duration_s': ParameterValue(float(mit_joint_target_duration_s), value_type=float),
                    'input_joint_prefix': input_joint_prefix,
                    'auto_enable': False,
                }],
            )
        )

    if (
        follow == 'true'
        and (feedback_joint_prefix or input_joint_prefix)
        and follow_joint_states_topic in ('feedback/joint_states', 'feedback/prefixed_joint_states')
    ):
        actions.append(
            Node(
                package='agx_arm_mit_tools',
                executable='agx_arm_joint_state_name_adapter',
                namespace=namespace,
                parameters=[{
                    'input_topic': 'feedback/joint_states',
                    'output_topic': 'feedback/prefixed_joint_states',
                    'joint_prefix': feedback_joint_prefix or input_joint_prefix,
                    'mode': 'prepend',
                }],
            )
        )

    return actions


def generate_launch_description():

    default_mit_params_file = _default_mit_params_file()

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

    execution_profile_arg = DeclareLaunchArgument(
        'execution_profile',
        default_value='manual',
        choices=['manual', 'standalone', 'left_arm', 'left_hand', 'left_gripper', 'right_arm', 'right_hand', 'right_gripper', 'duo_arm', 'duo_gripper', 'duo_hand', 'duo_hand_external_bridge'],
        description='Repo-owned execution preset for single-arm debug and MIT bringup. Multi-arm profiles are rejected here.',
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
        default_value='',
        description='CAN port used by the AGX Arm node. Empty resolves the side bus from the execution profile (registry arm.sides.*.can_port), falling back to can_nero_right. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.'
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

    omnihand_device_id_arg = DeclareLaunchArgument(
        'omnihand_device_id',
        default_value='1',
        description='Vendor SDK device_id used when omnihand_backend_type is sdk.'
    )

    omnihand_canfd_id_arg = DeclareLaunchArgument(
        'omnihand_canfd_id',
        default_value='0',
        description='Vendor SDK canfd_id used when omnihand_backend_type is sdk.'
    )

    omnihand_sdk_cfg_path_arg = DeclareLaunchArgument(
        'omnihand_sdk_cfg_path',
        default_value='',
        description='Optional vendor SDK config path used when omnihand_backend_type is sdk.'
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
        description='Follow real arm state. Prefixed Duo custom models use the feedback-side adapter hooks when needed, while the MIT controller remains on canonical feedback/joint_states.',
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
        default_value='200.0',
        description='MIT controller update rate when use_mit_controller is true.',
    )

    mit_params_file_arg = DeclareLaunchArgument(
        'mit_params_file',
        default_value=default_mit_params_file,
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

    feedback_joint_prefix_arg = DeclareLaunchArgument(
        'feedback_joint_prefix',
        default_value='',
        description='Optional prefix added onto follow-side feedback/joint_states for custom prefixed models. Empty falls back to input_joint_prefix.',
    )

    follow_joint_states_topic_arg = DeclareLaunchArgument(
        'follow_joint_states_topic',
        default_value='feedback/joint_states',
        description='JointState topic consumed by RViz when follow:=true. Override for prefixed custom-model feedback adaptation.',
    )

    tcp_parent_frame_arg = DeclareLaunchArgument(
        'tcp_parent_frame',
        default_value='',
        description='Optional parent frame for the tcp_offset static transform. For the current prefixed Duo right-arm slice this is typically right_arm_nero_tool0.',
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        execution_profile_arg,
        custom_model_arg,
        custom_model_xacro_args_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        revo2_type_arg,
        omnihand_type_arg,
        launch_omnihand_bridge_arg,
        omnihand_backend_type_arg,
        omnihand_device_id_arg,
        omnihand_canfd_id_arg,
        omnihand_sdk_cfg_path_arg,
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
        feedback_joint_prefix_arg,
        follow_joint_states_topic_arg,
        tcp_parent_frame_arg,
        OpaqueFunction(function=_launch_actions),
    ])
