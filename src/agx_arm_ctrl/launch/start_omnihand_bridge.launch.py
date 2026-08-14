from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from agx_arm_ctrl.motion_registry import arm_sides


def _resolve_namespace(context, *args, **kwargs):
    """Namespace for a standalone bridge: the side's Duo namespace by default.

    ``auto`` resolves to ``arm.sides.<side>.namespace`` from
    duo_motion_registry.yaml — the same namespace the Duo bringup and the
    exerciser use. Without this a standalone bridge listened on
    ``/control/omnihand/*`` while every repo-side tool addressed
    ``/<side>_arm/control/omnihand/*``, so commands went nowhere silently.
    Pass ``namespace:=''`` to force the bridge into the root namespace.
    """
    del args, kwargs
    requested = LaunchConfiguration("namespace").perform(context)
    if requested != "auto":
        return []
    side = LaunchConfiguration("omnihand_type").perform(context)
    try:
        resolved = str(arm_sides().get(side, {}).get("namespace", "")).strip("/")
    except Exception:
        resolved = ""
    context.launch_configurations["namespace"] = resolved
    return []


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the OmniHand bridge node.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="auto",
        description=(
            "ROS namespace for this OmniHand bridge instance. 'auto' takes the "
            "side namespace from duo_motion_registry.yaml (e.g. left_arm), "
            "matching the Duo bringup and omnihand_exerciser. Pass '' for root."
        ),
    )
    omnihand_type_arg = DeclareLaunchArgument(
        "omnihand_type",
        default_value="left",
        description="OmniHand side (left or right).",
    )
    hand_model_arg = DeclareLaunchArgument(
        "hand_model",
        default_value="o12_pro",
        description="OmniHand hardware model: o12_pro (OmniHand Pro 2025, 12 DoF) or o10 (mock only).",
    )
    backend_type_arg = DeclareLaunchArgument(
        "backend_type",
        default_value="mock",
        description="Backend type. Supported values are mock and sdk.",
    )
    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="1",
        description="Vendor SDK device_id used when backend_type is sdk.",
    )
    canfd_id_arg = DeclareLaunchArgument(
        "canfd_id",
        default_value="0",
        description="Vendor SDK canfd_id for the ZLG USB adapter path only; ignored by the native SocketCAN backend.",
    )
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="",
        description=(
            "Native SocketCAN interface for the hand (e.g. can_nero_right). Leave "
            "empty to resolve it from omnihand_type via the motion registry "
            "(duo_motion_registry.yaml arm.sides.<side>.can_port)."
        ),
    )
    sdk_cfg_path_arg = DeclareLaunchArgument(
        "sdk_cfg_path",
        default_value="",
        description="Optional vendor SDK config path used when backend_type is sdk.",
    )
    sdk_python_dir_arg = DeclareLaunchArgument(
        "sdk_python_dir",
        default_value="",
        description=(
            "Optional path to the built agibot_hand package. Leave empty to "
            "auto-locate the repo's vendor build; no PYTHONPATH/LD_LIBRARY_PATH "
            "export is needed (the compiled .so uses an $ORIGIN runpath)."
        ),
    )
    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="50.0",
        description="Feedback publish rate in Hz.",
    )
    runtime_metrics_enabled_arg = DeclareLaunchArgument(
        "runtime_metrics_enabled",
        default_value="false",
        description=(
            "Log per-SDK-call counts, rates and durations by name and thread. Off "
            "by default because it costs CPU on the Jetson; it is how the vendor "
            "SDK's share of the bridge's cost is read, so it has to be reachable "
            "from the supported bring-up rather than only by hand."
        ),
    )
    joint_read_rate_arg = DeclareLaunchArgument(
        "joint_read_rate",
        default_value="20.0",
        description=(
            "SDK joint readback poll rate in Hz (each poll is a real CAN request "
            "on the shared arm+hand bus). Decoupled from pub_rate, which "
            "republishes the cached state. <= 0 polls on every publish tick."
        ),
    )
    command_retry_enabled_arg = DeclareLaunchArgument(
        "command_retry_enabled",
        default_value="true",
        description=(
            "Re-send the latest hand target until the joint readback confirms it "
            "(the congested shared CAN bus silently drops hand frames under arm "
            "load in one-shot mode)."
        ),
    )
    command_retry_max_attempts_arg = DeclareLaunchArgument(
        "command_retry_max_attempts",
        default_value="8",
        description=(
            "Maximum command send attempts before giving up (eventual delivery "
            "matters more than latency on the congested shared bus)."
        ),
    )
    command_retry_period_s_arg = DeclareLaunchArgument(
        "command_retry_period_s",
        default_value="0.3",
        description="Seconds between verification checks / re-sends.",
    )
    command_verify_tolerance_rad_arg = DeclareLaunchArgument(
        "command_verify_tolerance_rad",
        default_value="0.10",
        description="Per-joint tolerance for treating a command as delivered.",
    )
    joint_states_command_topic_arg = DeclareLaunchArgument(
        "joint_states_command_topic",
        default_value="control/joint_states",
        description="Shared JointState command topic consumed by the OmniHand bridge.",
    )
    tactile_sample_count_arg = DeclareLaunchArgument(
        "tactile_sample_count",
        default_value="32",
        description="Mock tactile sample count.",
    )

    omnihand_bridge = Node(
        package="agx_arm_ctrl",
        executable="omnihand_bridge",
        name="omnihand_bridge_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "omnihand_type": LaunchConfiguration("omnihand_type"),
            "hand_model": LaunchConfiguration("hand_model"),
            "backend_type": LaunchConfiguration("backend_type"),
            "device_id": ParameterValue(LaunchConfiguration("device_id"), value_type=int),
            "canfd_id": ParameterValue(LaunchConfiguration("canfd_id"), value_type=int),
            "sdk_cfg_path": LaunchConfiguration("sdk_cfg_path"),
            "sdk_python_dir": LaunchConfiguration("sdk_python_dir"),
            "can_interface": LaunchConfiguration("can_interface"),
            "pub_rate": ParameterValue(LaunchConfiguration("pub_rate"), value_type=float),
            "joint_read_rate": ParameterValue(LaunchConfiguration("joint_read_rate"), value_type=float),
            "runtime_metrics_enabled": ParameterValue(
                LaunchConfiguration("runtime_metrics_enabled"), value_type=bool
            ),
            "command_retry_enabled": ParameterValue(
                LaunchConfiguration("command_retry_enabled"), value_type=bool
            ),
            "command_retry_max_attempts": ParameterValue(
                LaunchConfiguration("command_retry_max_attempts"), value_type=int
            ),
            "command_retry_period_s": ParameterValue(
                LaunchConfiguration("command_retry_period_s"), value_type=float
            ),
            "command_verify_tolerance_rad": ParameterValue(
                LaunchConfiguration("command_verify_tolerance_rad"), value_type=float
            ),
            "joint_states_command_topic": LaunchConfiguration("joint_states_command_topic"),
            "tactile_sample_count": ParameterValue(LaunchConfiguration("tactile_sample_count"), value_type=int),
        }],
    )

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        omnihand_type_arg,
        hand_model_arg,
        backend_type_arg,
        device_id_arg,
        canfd_id_arg,
        can_interface_arg,
        sdk_cfg_path_arg,
        sdk_python_dir_arg,
        pub_rate_arg,
        joint_read_rate_arg,
        runtime_metrics_enabled_arg,
        command_retry_enabled_arg,
        command_retry_max_attempts_arg,
        command_retry_period_s_arg,
        command_verify_tolerance_rad_arg,
        joint_states_command_topic_arg,
        tactile_sample_count_arg,
        # Must run after the arguments are declared and before the node is
        # created, so the resolved namespace is what the node actually gets.
        OpaqueFunction(function=_resolve_namespace),
        omnihand_bridge,
    ])