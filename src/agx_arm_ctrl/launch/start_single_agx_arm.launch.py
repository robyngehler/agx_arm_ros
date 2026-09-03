from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

# The hand-bus default is DERIVED from the one declared topology, never typed in
# here. It used to default to "shared" while the hardware had four buses, so a
# full bring-up quiesced the arm for every hand motion that did not need it.
try:
    from agx_arm_ctrl.motion_registry import handshake_required as _handshake_required

    _DEFAULT_HAND_BUS = "shared" if _handshake_required() else "dedicated"
except Exception:  # registry unreadable: take the degraded, safe reading
    _DEFAULT_HAND_BUS = "shared"


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
        description='ROS namespace for this robot instance. Leave empty for the default shared graph; use a namespace only to separate multiple robots.'
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can_nero_right',
        description='CAN port used by the AGX Arm node. Deprecated legacy names such as can0 or can_nero should not be used for the public runtime path.'
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

    # The driver folds these into the combined feedback/joint_states, which is
    # what move_group reads the full robot state from. The relative default only
    # resolves when the bridge shares this namespace; a bridge owned by another
    # launch has to be named absolutely or the hand joints never arrive.
    omnihand_joint_states_topic_arg = DeclareLaunchArgument(
        'omnihand_joint_states_topic',
        default_value='feedback/omnihand/joint_states',
        description=(
            'Topic the arm driver reads OmniHand joint states from. Relative '
            'resolves inside this arm namespace (bridge started here). Point it '
            'at an absolute topic such as /left_hand/feedback/omnihand/joint_states '
            'when another launch owns the bridge.'
        )
    )

    hand_bus_arg = DeclareLaunchArgument(
        'hand_bus',
        default_value=_DEFAULT_HAND_BUS,
        choices=['shared', 'dedicated'],
        description=(
            'Whether the OmniHand shares the arm side CAN bus (shared) or has '
            'its own line (dedicated). "shared" keeps the arm<->hand window '
            'handshake in the FollowJointTrajectory bridge; "dedicated" turns it '
            'off so arm MIT and the hand run in parallel. On a shared bus the '
            'hand needs the window, so only select "dedicated" with a real '
            'second bus (and point can_interface at it).'
        ),
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

    # The hand's cadence is its own. Forwarding the arm's pub_rate into the
    # bridge made the hand publish three messages 200 times a second while its
    # joints changed 20 times and its status and tactile once — 41.5 % of a core
    # against 4.5 %, measured on the mock backend with no CAN involved.
    hand_pub_rate_arg = DeclareLaunchArgument(
        'hand_pub_rate',
        default_value='50.0',
        description='Ceiling on OmniHand feedback publication in Hz. Publication is '
                    'driven by new readbacks, so this only throttles it further.'
    )

    hand_joint_read_rate_arg = DeclareLaunchArgument(
        'hand_joint_read_rate',
        default_value='50.0',
        description='OmniHand SDK joint readback rate in Hz. Each poll is a real CAN '
                    'request, and it is what actually sets the hand feedback rate.'
    )

    runtime_metrics_enabled_arg = DeclareLaunchArgument(
        'runtime_metrics_enabled',
        default_value='false',
        description=(
            'Log loop, callback and per-thread SDK call counters. Off by default '
            'because it costs CPU on the Jetson; it is how the "one SDK owner per '
            'device" claim is read on hardware, so it has to be reachable from the '
            'supported bring-up rather than only by running the node by hand.'
        )
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

    joint_states_command_topic_arg = DeclareLaunchArgument(
        'joint_states_command_topic',
        default_value='control/joint_states',
        description='JointState command topic consumed by the OmniHand bridge.'
    )

    publish_gripper_joint_arg = DeclareLaunchArgument(
        'publish_gripper_joint',
        default_value='true',
        choices=['true', 'false'],
        description='Publish "gripper" (opening width) joint in /feedback/joint_states. '
                    'Set false when used with MoveIt (URDF only has gripper_joint1/2).',
    )

    allow_legacy_gripper_command_ingress_arg = DeclareLaunchArgument(
        'allow_legacy_gripper_command_ingress',
        default_value='false',
        choices=['true', 'false'],
        description='Let bare gripper commands on control/joint_states move the '
                    'gripper. They carry no owner and no device generation, so a '
                    'stale one cannot be refused; development and debugging only. '
                    'Production commanders use control/gripper/authorized_trajectory.',
    )

    def _shutdown_on_driver_failure(event, context):
        """Take the bringup down when this arm's driver exits abnormally.

        Nothing above the driver notices its absence: MoveIt keeps planning, the
        MIT controller keeps reporting its loop rate, and RViz keeps rendering
        the URDF zero pose because no joint state ever arrives. A clean exit is
        the shutdown everyone is already in, so only a non-zero return code
        ends the launch.
        """
        if event.returncode in (0, None):
            return None
        return [Shutdown(reason=(
            f'{event.process_name} exited with code {event.returncode}; '
            'the stack has no arm feedback and nothing owns the arm'
        ))]

    # node
    agx_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        on_exit=_shutdown_on_driver_failure,
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'runtime_metrics_enabled': LaunchConfiguration('runtime_metrics_enabled'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('effector_type'),
            'omnihand_joint_states_topic': LaunchConfiguration('omnihand_joint_states_topic'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'publish_gripper_joint': LaunchConfiguration('publish_gripper_joint'),
            'allow_legacy_gripper_command_ingress': LaunchConfiguration(
                'allow_legacy_gripper_command_ingress'
            ),
        }],
        remappings=[
            # feedback topics
            ('feedback/joint_states', 'feedback/joint_states'),
            ('feedback/tcp_pose', 'feedback/tcp_pose'),
            ('feedback/arm_status', 'feedback/arm_status'),
            ('feedback/leader_joint_angles', 'feedback/leader_joint_angles'),
            ('feedback/gripper_status', 'feedback/gripper_status'),
            ('feedback/hand_status', 'feedback/hand_status'),

            # control topics
            ('control/joint_states', 'control/joint_states'),
            ('control/move_j', 'control/move_j'),
            ('control/move_p', 'control/move_p'),
            ('control/move_l', 'control/move_l'),
            ('control/move_c', 'control/move_c'),
            ('control/move_js', 'control/move_js'),
            ('control/move_mit', 'control/move_mit'),
            ('control/hand', 'control/hand'),
            ('control/hand_position_time', 'control/hand_position_time'),

            # services
            ('enable_agx_arm', 'enable_agx_arm'),
            ('set_normal_mode', 'set_normal_mode'),
            ('set_leader_mode', 'set_leader_mode'),
            ('move_home', 'move_home'),
            ('emergency_stop', 'emergency_stop'),
            ('exit_teach_mode', 'exit_teach_mode'),
        ],
    )

    omnihand_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_omnihand_bridge.launch.py',
            )
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
            'namespace': LaunchConfiguration('namespace'),
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'backend_type': LaunchConfiguration('omnihand_backend_type'),
            'device_id': LaunchConfiguration('omnihand_device_id'),
            'canfd_id': LaunchConfiguration('omnihand_canfd_id'),
            'sdk_cfg_path': LaunchConfiguration('omnihand_sdk_cfg_path'),
            'pub_rate': LaunchConfiguration('hand_pub_rate'),
            'joint_read_rate': LaunchConfiguration('hand_joint_read_rate'),
            'runtime_metrics_enabled': LaunchConfiguration('runtime_metrics_enabled'),
            'joint_states_command_topic': LaunchConfiguration('joint_states_command_topic'),
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('effector_type'), "' == 'omnihand' and '",
                LaunchConfiguration('launch_omnihand_bridge'), "' == 'true'",
            ])
        ),
    )

    omnihand_follow_joint_trajectory_node = Node(
        package='agx_arm_ctrl',
        executable='omnihand_follow_joint_trajectory',
        name='omnihand_follow_joint_trajectory',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'omnihand_type': LaunchConfiguration('omnihand_type'),
            'action_name': PythonExpression([
                "'", LaunchConfiguration('omnihand_type'), "_omnihand_controller/follow_joint_trajectory'",
            ]),
            # Window on for a shared bus, off for a dedicated hand bus (parallel).
            'handshake_enabled': ParameterValue(
                PythonExpression([
                    "'", LaunchConfiguration('hand_bus'), "' == 'shared'",
                ]),
                value_type=bool,
            ),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('effector_type'), "' == 'omnihand' and '",
                LaunchConfiguration('launch_omnihand_bridge'), "' == 'true'",
            ])
        ),
    )

    # The gripper's trajectory server. No handshake argument: the gripper rides
    # the arm's bus and its transmits are serialized onto the arm's worker, so
    # there is no window to open. It shares the driver's namespace, which is
    # where control/gripper/* and feedback/gripper_status live.
    gripper_follow_joint_trajectory_node = Node(
        package='agx_arm_ctrl',
        executable='gripper_follow_joint_trajectory',
        name='gripper_follow_joint_trajectory',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'action_name': 'gripper_controller/follow_joint_trajectory',
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('effector_type'), "' == 'agx_gripper'",
            ])
        ),
    )

    return LaunchDescription([
        # arguments
        log_level_arg,
        namespace_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        omnihand_type_arg,
        launch_omnihand_bridge_arg,
        omnihand_joint_states_topic_arg,
        hand_bus_arg,
        omnihand_backend_type_arg,
        omnihand_device_id_arg,
        omnihand_canfd_id_arg,
        omnihand_sdk_cfg_path_arg,
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        hand_pub_rate_arg,
        hand_joint_read_rate_arg,
        runtime_metrics_enabled_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        gripper_default_effort_arg,
        joint_states_command_topic_arg,
        publish_gripper_joint_arg,
        allow_legacy_gripper_command_ingress_arg,
        # node
        agx_arm_node,
        omnihand_bridge_launch,
        omnihand_follow_joint_trajectory_node,
        gripper_follow_joint_trajectory_node,
    ])
