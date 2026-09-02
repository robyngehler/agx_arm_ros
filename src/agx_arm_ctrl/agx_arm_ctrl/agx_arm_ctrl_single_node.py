#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import errno
import rclpy
import math
import threading
import subprocess
from functools import partial
from typing import NamedTuple, Optional
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW, NeroFW
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool, Empty, Trigger
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from scipy.spatial.transform import Rotation as R

from agx_arm_ctrl.command_validation import (
    mit_limits_for_tier,
    positions_outside_joint_limits,
    validate_mit_command,
)
from agx_arm_ctrl.device_authority import (
    CommandStamp,
    DeviceAuthority,
    DeviceState,
    UnitSafety,
    UnitSafetySnapshot,
)
from agx_arm_msgs.srv import ClaimDevice, RequestUnitStop
from agx_arm_ctrl.runtime_metrics import MeasuredSdk, RuntimeMetrics, name_os_thread
from agx_arm_ctrl.sdk_worker import (
    CallNotExecuted, CallOutcome, CallOutcomeUnknown, Lane, SdkWorker,
)
from agx_arm_msgs.msg import (
    AgxArmStatus, AgxDeviceAuthority, AgxDeviceCapability, AgxUnitSafety,
    AuthorizedJointTrajectory,
    GripperStatus,
    HandStatus, HandCmd, HandPositionTimeCmd,
    MoveMITMsg
)
from agx_arm_ctrl.effector import AgxGripperWrapper, Revo2Wrapper
from agx_arm_ctrl.motion_registry import handshake_required
from agx_arm_ctrl import nero_can_push

GRIPPER_JOINT_NAME = "gripper"

REVO2_FINGER_CONFIG = [
    # (joint_name, attribute_name, max_angle)
    ("thumb_metacarpal_joint", "thumb_base", 1.57),
    ("thumb_proximal_joint", "thumb_tip", 1.03),
    ("index_proximal_joint", "index_finger", 1.41),
    ("middle_proximal_joint", "middle_finger", 1.41),
    ("ring_proximal_joint", "ring_finger", 1.41),
    ("pinky_proximal_joint", "pinky_finger", 1.41),
]

REVO2_LEFT_HAND_JOINT_NAMES = [f"left_{suffix}" for suffix, _, _ in REVO2_FINGER_CONFIG]
REVO2_RIGHT_HAND_JOINT_NAMES = [f"right_{suffix}" for suffix, _, _ in REVO2_FINGER_CONFIG]
REVO2_HAND_JOINT_NAMES = REVO2_LEFT_HAND_JOINT_NAMES + REVO2_RIGHT_HAND_JOINT_NAMES

REVO2_HAND_JOINT_TO_FINGER_ATTR = {
    f"{prefix}{suffix}": (attr, max_angle)
    for prefix in ("left_", "right_")
    for suffix, attr, max_angle in REVO2_FINGER_CONFIG
}

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pyAgxArm.api.agx_arm_factory import PiperCanDefaultConfig


AUTHORITY_STATE_CODES = {
    DeviceState.OFFLINE: AgxDeviceAuthority.STATE_OFFLINE,
    DeviceState.STANDBY: AgxDeviceAuthority.STATE_STANDBY,
    DeviceState.READY: AgxDeviceAuthority.STATE_READY,
    DeviceState.RECOVERING: AgxDeviceAuthority.STATE_RECOVERING,
    DeviceState.FAULTED: AgxDeviceAuthority.STATE_FAULTED,
    DeviceState.STOPPED: AgxDeviceAuthority.STATE_STOPPED,
}


def derive_device_id(can_port) -> str:
    """Name this arm for the authority contract, from its CAN interface.

    The deployed arm interfaces are ``can_nero_left`` and ``can_nero_right``.

    Note this is *not* the coordinator's resource name: the scheduler's resource
    for the left hand is ``left_hand`` while its device is ``hand_left``. The
    two spellings are separate contracts and must not be derived from one
    another — set ``device_id`` explicitly wherever that matters.
    """
    port = str(can_port or "").strip().lower()
    for side in ("left", "right"):
        if port.endswith(side):
            return f"arm_{side}"
    return f"arm_{port}" if port else "arm_unknown"


def resolve_nero_firmware(software_version) -> tuple:
    """Map a reported Nero firmware version to the driver tier that speaks it.

    Returns ``(tier, explanation)``; the explanation is meant for the startup
    log, because nothing in the repository currently records which protocol
    tier the arms actually run on.

    Two defects are fixed here (Phase 1A):

    * **there was no ``NeroFW.V112`` branch at all**, so an arm on firmware 1.12
      was driven with the 1.11 protocol — silently, since both tiers connect;
    * the version was compared as a **string**. The firmware happens to report a
      zero-padded minor ("1.07", "1.11", "1.12"), which made that ordering work
      by accident inside one major version and nowhere else.
    """
    text = str(software_version).strip()
    try:
        major_text, _, minor_text = text.partition(".")
        version = (int(major_text), int(minor_text or 0))
    except ValueError:
        return NeroFW.DEFAULT, (
            f"firmware version '{text}' is not parseable as major.minor; "
            "falling back to the default Nero protocol"
        )

    if version >= (1, 12):
        return NeroFW.V112, f"firmware {text} -> NeroFW.V112"
    if version >= (1, 11):
        return NeroFW.V111, f"firmware {text} -> NeroFW.V111"
    return NeroFW.DEFAULT, f"firmware {text} -> NeroFW.DEFAULT"


class FeedbackSnapshot(NamedTuple):
    """Everything the publish batch reads from the SDK, taken in one go.

    The batch used to make eight-plus separate SDK calls from the publish
    thread, interleaved with ROS message construction. Two problems: it raced
    every other thread that touched the SDK, and the values it published were
    not from one instant — `get_arm_status()` was even read twice per cycle,
    with a chance of disagreeing with itself.

    Acquiring it as one bounded worker task fixes both. Bounded is the word that
    matters: a fixed set of sub-millisecond reads (0.10 ms measured for all
    seven motor states) is nothing like a retry loop bounded only by a 5 s
    timeout, and only the latter would threaten the safety lane.
    """

    joint_angles: object
    motor_states: tuple
    flange_pose: object
    tcp_pose: object
    arm_status: object
    leader_joint_angles: object
    # Health, read in the same instant as the data it judges. These used to run
    # on the publish thread *before* acquisition — they decide whether to
    # recover at all — which is why the SDK still had two callers after the
    # batch moved. Acquiring first and deciding second is what leaves one.
    is_ok: bool
    send_error_count: int
    acquired_at: float


class _StampRejection(NamedTuple):
    """Adapts an authority verdict to the rejection-logging shape."""

    reason: str
    detail: str


class StopVerification(NamedTuple):
    """Outcome of checking whether the arm actually came to rest.

    ``settled`` and ``evidence`` are deliberately separate. A stop that was
    issued but could not be checked is not a stop that failed, and it is not a
    stop that succeeded either — collapsing the two into one boolean is what
    let an unverifiable stop be reported as confirmed.
    """

    settled: bool
    evidence: bool
    detail: str

    @property
    def verified(self) -> bool:
        """True only for a stop confirmed against real feedback."""
        return self.settled and self.evidence


class AgxArmRosNode(Node):
    # Class-level, disabled: a bare instance (tests build one via __new__) and
    # an unmeasured deployment then behave identically, and no call site has to
    # ask whether instrumentation happens to be wired up.
    metrics = RuntimeMetrics(enabled=False)

    # Held-bus classifier state, class-level for the same reason: a bare
    # instance reads a defined value instead of raising, and an interface whose
    # counters cannot be read classifies as "not held" — the fault path it
    # defers is then the one that runs.
    _link_counter_files = None
    _last_link_rx_packets = None
    _last_link_tx_packets = None
    _bus_held_since_monotonic = None
    _bus_held_logged = False
    _last_rx_advance_monotonic = None
    bus_hold_patience_s = 60.0
    bus_hold_min_silence_s = 0.25
    bus_recovery_persistent = True
    bus_recovery_backoff_max_s = 5.0
    bus_recovery_persist_max_s = 300.0

    #: How often a persistent recovery says it is still trying.
    RECOVERY_PROGRESS_EVERY = 10

    def _recovery_backoff_s(self, attempt: int) -> float:
        """Delay before retry ``attempt``: linear ramp, capped.

        Long enough that a held bus costs a handful of attempts rather than
        thousands, short enough that a bus which comes back is picked up within
        one step of it.
        """
        return min(0.5 * (attempt - 1), self.bus_recovery_backoff_max_s)

    def __init__(self):
        super().__init__("agx_arm_ctrl_single_node")

        ### ros parameters
        self._declare_parameters()
        self._load_parameters()
        self._log_parameters()

        ### device authority (built before the SDK so a failed connect is
        ### still reported as a state rather than as silence)
        # An observer: it adopts the writer's generations and refuses to mint
        # its own. This device still stops itself unilaterally — that is a
        # device-level fault on its own epoch and needs nobody — but the
        # unit-wide statement that a new safety era began is one process's job.
        self._unit_safety = UnitSafety(self.device_id, writer=False)
        self._authority = DeviceAuthority(self.device_id, self._unit_safety)
        # The steady-state owner of this device's SDK session. Recovery
        # takes ownership from it explicitly; nothing else touches the SDK.
        self._sdk = SdkWorker(
            self.device_id, metrics=self.metrics, logger=self.get_logger()
        )

        ### AgxArmFactory
        self._init_agx_arm()

        ### effector
        self._init_effector()

        ### publishers
        self._setup_publishers()

        ### subscribers
        self._setup_subscribers()

        ### services
        self._setup_services()

        ### publisher thread
        # Two threads, not one. Acquisition owns the arm's cadence — the health
        # checks, the recovery watchdog and the snapshot the command path
        # decides on all hang off it — while publication is ROS middleware work
        # whose cost has nothing to do with how often the hardware must be read.
        # Coupled, a slow DDS write delayed the next acquisition and with it the
        # watchdog's next sample.
        self.publisher_thread = threading.Thread(
            target=self._acquisition_loop, name="acquisition", daemon=True
        )
        self.publisher_thread.start()
        self.publication_thread = threading.Thread(
            target=self._publication_loop, name="publication", daemon=True
        )
        self.publication_thread.start()

    ### initialization methods
    def _declare_parameters(self):
        self.declare_parameter("can_port", "can_nero_right")
        self.declare_parameter("arm_type", "nero")
        # Identity of this device in the authority contract. Empty derives it
        # from the CAN port (can_nero_left -> arm_left), which is the deployed
        # naming; set it explicitly for anything that does not follow it.
        self.declare_parameter("device_id", "")
        # Fail-closed: an unstamped MIT command carries no commander, no
        # generation and no sequence, so nothing can establish that it is
        # current. Only one node publishes this topic, and it stamps.
        self.declare_parameter("require_command_stamp", True)
        # The direct arm-motion topics predate the authority contract: they
        # carry no commander, no generation and no sequence, so nothing can
        # establish that such a command is current or that its sender is
        # entitled to move this arm. Off by default; a development profile
        # that knowingly wants them says so.
        self.declare_parameter("allow_legacy_motion_ingress", False)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("fast_mode", False)
        self.declare_parameter("speed_percent", 100)
        self.declare_parameter("pub_rate", 200)
        # How often the arm is actually read. Separate from pub_rate, which is
        # a ROS publication rate and was never a statement about how fresh the
        # hardware reading has to be. Tying the two put a 200 Hz read on the
        # session owner in front of a 100 Hz command stream, and on hardware the
        # acquisition lost half its rate to it.
        #
        # 100 Hz is what the consumers justify: the MIT controller runs at
        # 100 Hz (C2 targets 200-250 later, at which point this follows it), and
        # the recovery watchdog decides on feedback_timeout, which is seconds.
        # 0 means "keep the old behaviour and follow pub_rate".
        self.declare_parameter("acquisition_rate_hz", 200.0)
        # Phase 0 baseline instrumentation (C6): off unless asked for, so an
        # unmeasured deployment pays nothing.
        self.declare_parameter("runtime_metrics_enabled", False)
        self.declare_parameter("runtime_metrics_period_s", 10.0)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("effector_type", "none")
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("gripper_default_effort", 1.0)
        self.declare_parameter("publish_gripper_joint", True)
        # Bare gripper commands on control/joint_states. They carry no owner and
        # no generation, so a stale or reordered one cannot be refused; the
        # production path is control/gripper/authorized_trajectory. Development
        # and debugging only, exactly like the hand's equivalent switch.
        self.declare_parameter("allow_legacy_gripper_command_ingress", False)
        self.declare_parameter("omnihand_joint_states_topic", "feedback/omnihand/joint_states")
        # CAN bus recovery (P1): detect TX stalls (ENOBUFS slot leak) / stale
        # feedback and re-establish the link instead of dead-locking until the
        # whole launch is restarted.
        self.declare_parameter("bus_recovery_enabled", True)
        self.declare_parameter("bus_recovery_tx_error_threshold", 1)
        # Frames older than this (kernel RX timestamp of the last parsed
        # feedback frame) count as a dead bus. 0.5 s proved far too tight: the
        # instantaneous readiness signal starves for hundreds of ms under GIL
        # pressure while candump shows uninterrupted feedback, so the watchdog
        # tore down healthy links every ~7 s during active MIT streaming.
        self.declare_parameter("feedback_timeout", 2.0)
        self.declare_parameter("bus_recovery_link_reset", False)
        self.declare_parameter("bus_recovery_max_attempts", 3)
        # How long a live-but-quiet bus is waited out before it is treated as a
        # fault. The external CAN watchdog takes the bus on purpose and gives it
        # back; recovery cannot succeed against it and only spends its budget,
        # after which the lockout latches and every later command is refused on
        # a bus that came back healthy. Measured hold: 25.7 s.
        self.declare_parameter("bus_hold_patience_s", 60.0)
        # Whether recovery keeps trying until feedback returns. An attempt is one
        # disconnect/connect plus a bounded feedback wait, spaced by a backoff —
        # a few frames per attempt, not a load the bus notices. Giving up instead
        # latches a lockout that refuses every command on the bus that comes back,
        # so persistence is the default and the attempt budget is the opt-in.
        self.declare_parameter("bus_recovery_persistent", True)
        self.declare_parameter("bus_recovery_backoff_max_s", 5.0)
        # A persistent recovery still has to end. It owns the SDK session while
        # it runs, so a loop with no terminal bound leaves the node up, silent
        # and unrecoverable, with no exception to show for it. On expiry it
        # stops and latches the lockout like an exhausted attempt budget.
        self.declare_parameter("bus_recovery_persist_max_s", 300.0)
        # How long RX must be silent before the quiet counts as a hold. The arm
        # delivers a complete update every 7-10 ms and this is evaluated at the
        # publish rate, so a per-sample comparison sees silence constantly.
        self.declare_parameter("bus_hold_min_silence_s", 0.25)
        # Minimum quiet time after a completed recovery before the watchdog may
        # fire again. Without it a congested (error-storming) bus re-latches a
        # comm error during the recovery's OWN enable/config sends, and the
        # watchdog re-triggers on the very next publish-loop iteration — a
        # 60-100 ms disconnect/enable loop that floods the sick bus further.
        # Recovering more often than this cannot help: if one full reconnect
        # did not restore the bus, an immediate second one will not either.
        self.declare_parameter("bus_recovery_cooldown_s", 5.0)
        # After a bus recovery, refuse new motion until an operator/supervisor
        # explicitly clears the fault, instead of silently re-arming control on
        # the next healthy tick (plan Phase 1 item 6 / Phase 2 item 3).
        self.declare_parameter("require_fault_ack", True)
        # Hand window: silence the firmware's CAN feedback push while the
        # OmniHand owns the shared side bus (the arm keeps holding its pose).
        # Without this the window frees nothing — measured on hardware, the
        # ~2150 frames/s idle load is the arm's feedback push, not MIT commands.
        self.declare_parameter("hand_window_silence_feedback", True)
        # Hard upper bound on how long feedback may stay silenced. The
        # bus-recovery watchdog is deliberately blind while the push is off, so
        # the silence must be bounded: after this the push is restored (and the
        # watchdog re-armed) even if resume_arm_control never arrives.
        self.declare_parameter("hand_window_max_silence_s", 10.0)
        # Silencing the push is a mode frame like any other, and the SDK drops
        # mode frames silently under bus saturation — which is exactly the
        # condition a hand window runs in. So the silence is verified the only
        # way it can be: the feedback frames must actually STOP advancing.
        # Waiting for that also drains the window's own opening burst off the
        # bus before the hand issues its first CANFD request.
        self.declare_parameter("hand_window_silence_verify_s", 0.4)
        # No new feedback frame for this long counts as silenced. Must stay well
        # above the ~5 ms push period and below feedback_timeout.
        self.declare_parameter("hand_window_silence_quiet_s", 0.08)
        # The MOVE-J hold frame that parks the arm for a hand window is also a
        # single mode frame, and the bus is still flooded when it goes out (the
        # push is only silenced AFTER the hold is verified). On the one-shot
        # shared bus that single frame can lose arbitration and be dropped,
        # leaving the firmware in MIT. So re-assert the same-pose, motionless
        # MOVE-J until the readback confirms the firmware left MIT, bounded here.
        self.declare_parameter("hand_window_hold_assert_s", 1.0)
        self.declare_parameter("hand_window_hold_poll_s", 0.05)

    def _load_parameters(self):
        self.metrics = RuntimeMetrics(
            enabled=bool(self.get_parameter("runtime_metrics_enabled").value),
            report_period_s=float(self.get_parameter("runtime_metrics_period_s").value),
        )
        self.can_port = self.get_parameter("can_port").value
        self.arm_type = self.get_parameter("arm_type").value
        self.allow_legacy_motion_ingress = bool(
            self.get_parameter("allow_legacy_motion_ingress").value
        )
        self.require_command_stamp = bool(
            self.get_parameter("require_command_stamp").value
        )
        self.device_id = (
            self.get_parameter("device_id").value
            or derive_device_id(self.can_port)
        )
        self.auto_enable = self.get_parameter("auto_enable").value
        self.fast_mode = self.get_parameter("fast_mode").value
        self.speed_percent = self.get_parameter("speed_percent").value
        self.pub_rate = self.get_parameter("pub_rate").value
        acquisition_rate = float(self.get_parameter("acquisition_rate_hz").value)
        self.acquisition_rate_hz = (
            acquisition_rate if acquisition_rate > 0.0 else float(self.pub_rate)
        )
        self.enable_timeout = self.get_parameter("enable_timeout").value
        self.effector_type = self.get_parameter("effector_type").value
        self.tcp_offset = self.get_parameter("tcp_offset").value
        self.gripper_default_effort = self.get_parameter("gripper_default_effort").value
        self.publish_gripper_joint = self.get_parameter("publish_gripper_joint").value
        self.allow_legacy_gripper_command_ingress = bool(
            self.get_parameter("allow_legacy_gripper_command_ingress").value
        )
        self.omnihand_joint_states_topic = self.get_parameter("omnihand_joint_states_topic").value
        self.bus_recovery_enabled = self.get_parameter("bus_recovery_enabled").value
        self.bus_recovery_tx_error_threshold = max(
            1, int(self.get_parameter("bus_recovery_tx_error_threshold").value)
        )
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.bus_recovery_link_reset = self.get_parameter("bus_recovery_link_reset").value
        self.bus_recovery_max_attempts = max(
            1, int(self.get_parameter("bus_recovery_max_attempts").value)
        )
        self.bus_hold_patience_s = max(
            0.0, float(self.get_parameter("bus_hold_patience_s").value)
        )
        self.bus_recovery_persistent = bool(
            self.get_parameter("bus_recovery_persistent").value
        )
        self.bus_recovery_backoff_max_s = max(
            0.0, float(self.get_parameter("bus_recovery_backoff_max_s").value)
        )
        self.bus_recovery_persist_max_s = max(
            0.0, float(self.get_parameter("bus_recovery_persist_max_s").value)
        )
        self.bus_hold_min_silence_s = max(
            0.0, float(self.get_parameter("bus_hold_min_silence_s").value)
        )
        # Set when a stall first matched the held-bus signature; None otherwise.
        self._bus_held_since_monotonic = None
        self._bus_held_logged = False
        self._link_counter_files: dict = {}
        self._last_link_rx_packets = None
        self._last_link_tx_packets = None
        self.bus_recovery_cooldown_s = max(
            0.0, float(self.get_parameter("bus_recovery_cooldown_s").value)
        )
        self.require_fault_ack = bool(self.get_parameter("require_fault_ack").value)
        self.hand_window_silence_feedback = bool(
            self.get_parameter("hand_window_silence_feedback").value
        )
        self.hand_window_max_silence_s = max(
            0.5, float(self.get_parameter("hand_window_max_silence_s").value)
        )
        self.hand_window_silence_verify_s = max(
            0.0, float(self.get_parameter("hand_window_silence_verify_s").value)
        )
        self.hand_window_silence_quiet_s = max(
            0.01, float(self.get_parameter("hand_window_silence_quiet_s").value)
        )
        self.hand_window_hold_assert_s = max(
            0.0, float(self.get_parameter("hand_window_hold_assert_s").value)
        )
        self.hand_window_hold_poll_s = max(
            0.001, float(self.get_parameter("hand_window_hold_poll_s").value)
        )
        # After a recovery the node latches this and refuses new motion until
        # clear_fault_lockout is called; feedback keeps flowing throughout.
        self._fault_lockout = False
        self._fault_lockout_logged = False
        self._last_recovery_end_monotonic = 0.0
        self._recovery_cooldown_logged = False
        self._last_tx_congestion_log = 0.0
        self._recover_reason = ""

        if self.arm_type not in ArmModel.__dict__.values():
            self.get_logger().error(
                f"Unsupported arm_type '{self.arm_type}', expected one of {list(ArmModel.__dict__.values())}."
            )
            exit(1)

        if self.gripper_default_effort < 0:
            self.get_logger().warn(
                f"gripper_default_effort should be greater than 0, but got {self.gripper_default_effort}. "
                "Setting it to default value 1.0"
            )
            self.gripper_default_effort = 1.0

        ### variables
        self.is_piper = "piper" in self.arm_type
        self.is_nero = "nero" in self.arm_type
        self.is_switch_seamlessly = True
        self.is_mit_mode = False
        # Leader (zero-force drag) mode silences the normal joint-state push on
        # the firmware, so the bus-recovery watchdog must fall back to the
        # leader-angle stream instead of mistaking that silence for a stall.
        self._leader_mode_active = False
        self._current_motion_mode = None  # tracks last mode ctrl sent to hardware
        self._last_external_mode_reassert = 0.0
        # Step-and-settle hand window: while active the arm is parked in a
        # driver-level normal-mode hold and incoming MIT commands are dropped at
        # this gateway, so the OmniHand owns the shared side CAN bus (plan §3).
        self._hand_window_active = False
        # True while the firmware's feedback push is silenced for a hand window.
        # The arm stays in its CAN-control hold — only the Nero->host feedback
        # stream is off, so the watchdog must not read that silence as a stall.
        self._hand_window_push_silenced = False
        self._hand_window_silence_started = 0.0
        # Three facts: a transport exists and can carry a command
        # (transport_connected); feedback is advancing (control_ready, is_ok);
        # the joints answered the last enable request (enable_flag). Only the
        # transport gates a bootstrap command; motion needs all three plus
        # authority admission.
        self._transport_connected = False
        self.enable_flag = False
        # Set only by a readback, never by a command returning.
        self._enable_verified = False
        self.control_ready = False
        self._control_ready_logged = False
        self.arm_joint_names = list()
        self.arm_joint_count = 0
        # bus recovery state
        self._had_control_ready = False
        self._recovery_in_progress = False
        self._recovery_lock = threading.Lock()
        self._recovery_started_monotonic = 0.0
        self._last_good_feedback_monotonic = time.monotonic()
        # Feedback-frame progress tracking for _check_arm_ready: the vendor
        # frame timestamp advancing is the ground truth for a live bus; the
        # local monotonic clock dates our last observation of an advance, so
        # no cross-clock-domain comparison is ever needed.
        self._last_feedback_frame_ts = None
        self._last_feedback_advance_monotonic = time.monotonic()
        # TX stall is detected node-side from caught send exceptions, so it works
        # regardless of how the underlying pyAgxArm comm reports ENOBUFS.
        self._tx_stall_count = 0
        self._tx_stall_detected = False
        # Phase-0 watchdog instrumentation: tell a genuinely dead bus apart from
        # local scheduling starvation. Active MIT streaming pegs this node near
        # 100 % of a core, which stalls the publish loop and the FPS-based
        # is_ok()/hz signals without the CAN bus ever going down. A loop gap this
        # large means the "staleness" the watchdog sees may be local, not the bus.
        self._loop_overrun_threshold_s = max(2.0 / self.acquisition_rate_hz, 0.2)
        # The overrun threshold is floored at 200 ms because it hunts bus stalls,
        # which makes it blind to the other failure: a loop that never stalls and
        # never reaches its rate either. Feedback ran at 135 Hz of a configured
        # 200 without one warning, and every teach recording inherited the gap as
        # duplicate samples. So the achieved rate is reported on its own.
        self._acq_gap_count = 0
        self._acq_gap_sum_s = 0.0
        self._last_rate_report_monotonic = time.monotonic()
        self._last_loop_monotonic = 0.0
        self._last_loop_gap_s = 0.0
        self._max_loop_gap_s = 0.0
        self._loop_overrun_count = 0
        # recoveries suppressed because the kernel RX timestamp proved the bus
        # was still live while a starvation-sensitive signal read stale/not-ok.
        self._loop_overrun_suppressions = 0
        self._last_overrun_log_monotonic = 0.0
        # The most recent acquisition, for the command callbacks. They used to
        # read the SDK themselves to decide whether the arm may be commanded —
        # a blocking round trip through the session owner, once per command at
        # the control rate — while the publish loop was already acquiring the
        # same values 200 times a second. A snapshot is immutable and replaced
        # whole, so a reader sees one instant or the previous one, never a mix.
        self._latest_snapshot = None
        self._last_stale_ingress_log_monotonic = 0.0
        # Rejected-command bookkeeping. A malformed stream arrives at the
        # control rate, so the log is rate-limited per reason and carries the
        # suppressed count: flooding the log is itself a CPU problem on this
        # Jetson, and the 0E baseline shows how little headroom there is.
        # Latched by an emergency stop, cleared only by clear_fault_lockout.
        # Needed because _sync_authority is a *derived* mapping that runs
        # every publish cycle: a state the e-stop sets directly is erased on
        # the next tick unless something in the gates holds it. Until the
        # unit-safety writer existed, the local unit stop was that latch.
        self._estop_latched = False
        self._command_rejections = {}
        self._last_rejection_log_monotonic = {}
        self._rejection_log_period_s = 2.0
        # recovery trigger category -> count, surfaced on every recovery so a
        # CPU-stress run shows what actually drove each reconnect.
        self._recovery_reason_counts = {}
        # Set by the emergency-stop service when a stop cannot be verified in
        # feedback: the heavyweight link-reset recovery must run on the publish
        # thread (which owns the connection), never from the service thread.
        self._force_recovery = False
        # Silent-TX-loss surfacing: the forked SDK counts send() failures that
        # the swallow-style comm model would otherwise hide once an RX frame
        # clears last_error (plan section 1.3.2). Track the last observed count
        # so a rising count can be logged even while feedback looks healthy.
        self._last_send_error_count = 0
        self._last_tx_loss_log = 0.0

    def _log_parameters(self):
        self.get_logger().info(f"can_port: {self.can_port}")
        self.get_logger().info(f"arm_type: {self.arm_type}")
        self.get_logger().info(f"auto_enable: {self.auto_enable}")
        self.get_logger().info(f"fast_mode: {self.fast_mode}")
        self.get_logger().info(f"speed_percent: {self.speed_percent}")
        self.get_logger().info(f"pub_rate: {self.pub_rate}")
        self.get_logger().info(f"acquisition_rate_hz: {self.acquisition_rate_hz}")
        self.get_logger().info(f"enable_timeout: {self.enable_timeout}")
        self.get_logger().info(f"effector_type: {self.effector_type}")
        self.get_logger().info(f"tcp_offset: {self.tcp_offset}")
        self.get_logger().info(f"gripper_default_effort: {self.gripper_default_effort}")
        self.get_logger().info(f"publish_gripper_joint: {self.publish_gripper_joint}")
        self.get_logger().info(f"omnihand_joint_states_topic: {self.omnihand_joint_states_topic}")
        self.get_logger().info(f"bus_recovery_enabled: {self.bus_recovery_enabled}")
        self.get_logger().info(f"bus_recovery_tx_error_threshold: {self.bus_recovery_tx_error_threshold}")
        self.get_logger().info(f"feedback_timeout: {self.feedback_timeout}")
        self.get_logger().info(f"bus_recovery_link_reset: {self.bus_recovery_link_reset}")
        self.get_logger().info(f"bus_recovery_max_attempts: {self.bus_recovery_max_attempts}")

    def _wait_for_firmware(self) -> None:
        """Poll the firmware query until it answers or the enable timeout runs out."""
        self.firmware = None
        start_time = time.time()
        while time.time() - start_time < self.enable_timeout:
            self.firmware = self.agx_arm.get_firmware()
            if self.firmware:
                return
            time.sleep(0.005)

    def _connect_transport(self) -> bool:
        """Open the SDK session and record that one exists.

        Only ``connect()`` raising means "no transport"; a silent arm still has
        one.
        """
        try:
            self.agx_arm.connect()
        except Exception as exc:
            self._transport_connected = False
            self.get_logger().error(f"CAN transport connect failed: {exc}")
            return False
        self._transport_connected = True
        return True

    def _disconnect_transport(self) -> None:
        """Close the SDK session and record that none exists."""
        self._transport_connected = False
        try:
            self.agx_arm.disconnect()
        except Exception as exc:
            self.get_logger().warn(f"CAN transport disconnect failed: {exc}")

    def _transport_available(self) -> bool:
        """True when a bounded transport-level command may be attempted.

        Not ``is_ok()``: that reports feedback health, which a bootstrap command
        produces rather than requires.
        """
        return self.agx_arm is not None and self._transport_connected

    def _ensure_feedback_push_enabled(
        self, reason: str, *, force: bool = False, direct: bool = False
    ) -> bool:
        """Turn the Nero->host feedback push on.

        A transport/reporting operation, not motion: needs a session only, never
        ``enable_flag``, ``control_ready``, ``is_ok()`` or a snapshot. A stopped
        unit may run it and stay stopped.

        ``force`` sends the frame regardless of ``_hand_window_push_silenced``,
        which records only this node's own silencing; an arm that booted mute
        has the push off while that flag says otherwise. ``direct`` bypasses the
        worker for the callers that own the session outright (bus recovery,
        shutdown).
        """
        if not force and not self._hand_window_push_silenced:
            return True
        if not self._transport_available():
            self.get_logger().warn(
                f"cannot enable the feedback push ({reason}): no CAN transport"
            )
            return False
        if not nero_can_push.supports_can_push(self.agx_arm):
            self.get_logger().warn(
                f"cannot enable the feedback push ({reason}): "
                f"{nero_can_push.UNSUPPORTED_MESSAGE}"
            )
            return False
        try:
            if direct:
                nero_can_push.set_can_push(self.agx_arm, True)
            else:
                self._sdk_write(
                    "enable_can_push",
                    lambda: nero_can_push.set_can_push(self.agx_arm, True),
                )
        except Exception as exc:
            self.get_logger().error(
                f"feedback push ENABLE failed ({reason}): {exc}"
            )
            return False
        # Feedback restarts from here: do not charge the gap to the watchdog.
        now = time.monotonic()
        self._last_good_feedback_monotonic = now
        self._last_feedback_advance_monotonic = now
        return True

    def _recover_silent_arm(self) -> None:
        """Escalate from a push-only bootstrap to a full mode re-assert.

        An arm left in leader/follower mode boots with ``enable_can_push``
        DISABLED and persists that across power cycles. The push-only frame has
        already been sent by the time this runs, so what is left is the
        persisted linkage: ``set_normal_mode`` re-asserts linkage and push
        together. Commands no motion.
        """
        if not self.is_nero:
            return
        self.get_logger().warn(
            "No firmware answer after the feedback-push bootstrap — the arm may "
            "be holding a persisted leader/follower linkage. Re-asserting normal "
            "mode once and retrying."
        )
        try:
            self._sdk_write("set_normal_mode", self.agx_arm.set_normal_mode)
        except Exception as e:
            self.get_logger().error(f"set_normal_mode during startup recovery failed: {e}")
            return
        self._leader_mode_active = False
        self._hand_window_push_silenced = False
        self._hand_window_silence_started = 0.0

    def _init_agx_arm(self):
        # Defaults matching the driver the SDK builds when no tier is given, so
        # an arm whose firmware never answers is still validated against the
        # protocol it is actually being driven with.
        self.firmware_tier = NeroFW.DEFAULT
        # Only _wait_for_firmware assigned this, and it runs only under
        # auto_enable. A bringup with auto_enable:=false — the read-only shape
        # used to inspect an arm without energising it — therefore reached the
        # capability publish with the attribute missing and died during startup.
        self.firmware = None
        self.mit_limits = mit_limits_for_tier(NeroFW.DEFAULT)
        config: PiperCanDefaultConfig = create_agx_arm_config(
            robot=self.arm_type, comm="can", channel=self.can_port
        )
        self.agx_arm = self._measured(AgxArmFactory.create_arm(config))
        self._connect_transport()

        self.arm_joint_names = list(config["joint_limits"].keys())
        # Kept, not just the names: the boundary check needs the bounds to say
        # when a commanded position is outside the joint's configured range.
        self.arm_joint_limits = dict(config["joint_limits"])
        self.arm_joint_count = self.agx_arm.joint_nums

        # Bootstrap before readiness: assert the push on the transport alone,
        # since every readiness signal below is derived from the feedback it
        # produces. Runs without auto_enable too — a read-only bringup needs
        # feedback as well.
        self._ensure_feedback_push_enabled("startup bootstrap", force=True)

        if self.auto_enable:
            # The enable goes out without waiting for feedback; only the
            # verification waits. On old firmware the push can depend on the
            # enabled state, so it is asserted again afterwards.
            if not self._enable_arm(True, self.enable_timeout):
                self.get_logger().error("Failed to auto-enable the arm")

            self._wait_for_firmware()
            if not self.firmware:
                self._ensure_feedback_push_enabled(
                    "still silent after enable", force=True
                )
                self._wait_for_firmware()
            if not self.firmware:
                # An arm whose feedback push is disabled answers nothing, so
                # startup would die here — and with the node dead its
                # set_normal_mode service never comes up, leaving no way to
                # re-enable the push through ROS. Break that deadlock once.
                self._recover_silent_arm()
                self._wait_for_firmware()

            if self.firmware:
                current_version = self.firmware['software_version']
                self.get_logger().info(f"firmware version: {current_version}")
                firmeware_version = PiperFW.DEFAULT
                if self.is_piper:
                    if current_version < "S-V1.8-5":
                        self.is_switch_seamlessly = False
                    if current_version > "S-V1.8-2" and current_version < "S-V1.8-8":
                        firmeware_version = PiperFW.V183
                    elif current_version >= "S-V1.8-8":
                        firmeware_version = PiperFW.V188
                elif self.is_nero:
                    firmeware_version, explanation = resolve_nero_firmware(
                        current_version
                    )
                    # The tier decides the MIT bounds, not just the driver:
                    # the two arms in this unit run different firmware, and the
                    # 1.11 tier bounds feed-forward torque at 16 N·m on every
                    # joint where the default tier bounds it per joint.
                    self.firmware_tier = firmeware_version
                    self.mit_limits = mit_limits_for_tier(firmeware_version)
                    self.get_logger().info(
                        f"Nero protocol tier: {explanation}; MIT torque bound "
                        f"per joint: "
                        f"{[self.mit_limits.torque_limit(j) for j in range(1, 8)]}"
                    )

                if firmeware_version != PiperFW.DEFAULT:
                    self._disconnect_transport()
                    config = create_agx_arm_config(
                        robot=self.arm_type, comm="can", channel=self.can_port,
                        firmeware_version=firmeware_version
                    )
                    self.agx_arm = self._measured(AgxArmFactory.create_arm(config))
                    self._connect_transport()
                    # A new SDK object is a new session: whatever the previous
                    # one asked the firmware for does not carry over, so the
                    # push is asserted again on the object that will actually
                    # be driven.
                    self._ensure_feedback_push_enabled(
                        "reconnected on firmware tier", force=True
                    )
            else:
                # Name each half separately: a transport that exists and
                # feedback that never came have different fixes. See
                # docs/sprint_refactor/reference/l3_command_authority.md.
                self.get_logger().error(
                    "Failed to get firmware version. Transport session: present; "
                    "feedback-push bootstrap: sent; feedback: none; enable: "
                    "unverified. Check in this order: (1) does this interface's "
                    "TX packet counter advance at all (ip -s link show)? If not, "
                    "the frames never leave the host — on Jetson the 40-pin "
                    "header pinmux is discarded by a kernel update, so re-run "
                    "sudo /opt/nvidia/jetson-io/jetson-io.py. (2) arm power, "
                    "E-stop and wiring for this side. (3) does the bus carry "
                    "feedback frames (candump)?"
                )
                exit(1)

            self.agx_arm.set_speed_percent(self.speed_percent)
            self.agx_arm.set_tcp_offset(self.tcp_offset)
        self._check_tx_observability_contract()

    def _measured(self, arm):
        """Wrap the SDK in per-call timing while instrumentation is on.

        Off by default, so an unmeasured deployment pays nothing and gets the
        raw object back.
        """
        if not self.metrics.enabled:
            return arm
        return MeasuredSdk(arm, self.metrics)

    def _check_tx_observability_contract(self) -> None:
        """Warn loudly at startup if the pinned SDK lacks TX-error observability.

        The silent-TX-loss safety signal (plan section 1.3.2) depends on the
        vendor fork's send-error counters. If they are absent — a stale submodule
        pin or a backend without the fork — the node otherwise degrades to no
        signal silently, so surface it here instead of losing it quietly.
        """
        if hasattr(self.agx_arm, "get_send_error_count"):
            self.get_logger().info(
                "TX-loss observability: SDK send-error counters available"
            )
        else:
            self.get_logger().warn(
                "TX-loss observability UNAVAILABLE: arm backend has no "
                "get_send_error_count() (vendor/pyAgxArm fork missing or stale "
                "pin). Silently-dropped arm commands will NOT be surfaced; update "
                "the pinned submodule to the TX-observability fork."
            )

    def _init_effector(self):
        self.gripper: Optional[AgxGripperWrapper] = None
        self.hand: Optional[Revo2Wrapper] = None
        self.omnihand_joint_state: Optional[JointState] = None
        # The gripper is a device of its own for ownership purposes even though
        # it shares this arm's SDK session and CAN socket. Its own generation is
        # what makes a command from a previous owner unexecutable.
        self._gripper_authority: Optional[DeviceAuthority] = None
        self._gripper_target: Optional[tuple] = None

        if self.effector_type == "agx_gripper":
            self.gripper = AgxGripperWrapper(self.agx_arm)
            if self.gripper.initialize():
                self.get_logger().info("AgxGripper initialized successfully")
                self._gripper_authority = DeviceAuthority(
                    f"{self.device_id}_gripper", self._unit_safety
                )
                self._gripper_authority.go_standby("gripper session up")
            else:
                self.get_logger().error("Failed to initialize AgxGripper")
                self.gripper = None
        elif self.effector_type == "revo2" and self.is_switch_seamlessly:
            self.hand = Revo2Wrapper(self.agx_arm)
            if self.hand.initialize():
                self.get_logger().info("Revo2 hand initialized successfully")
            else:
                self.get_logger().error("Failed to initialize Revo2 hand")
                self.hand = None

    def _setup_publishers(self):
        self.joint_states_pub = self.create_publisher(
            JointState, "feedback/joint_states", 1
        )
        # self.flange_pose_pub = self.create_publisher(
        #     PoseStamped, "feedback/flange_pose", 1
        # )
        self.tcp_pose_pub = self.create_publisher(
            PoseStamped, "feedback/tcp_pose", 1
        )
        self.arm_status_pub = self.create_publisher(
            AgxArmStatus, "feedback/arm_status", 1
        )
        # Latched so a late-joining coordinator/supervisor sees the current fault
        # state; True means motion is refused until clear_fault_lockout.
        self.fault_lockout_pub = self.create_publisher(
            Bool, "feedback/fault_lockout",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_fault_lockout()
        # Latched signal that the arm's feedback push is intentionally silenced
        # for a hand window. A separate always-on-ROS topic is needed because
        # the CAN feedback the MIT controller reads is exactly what goes quiet:
        # it must be told out-of-band, or it reads the expected silence as a
        # dead bus and floods gated dead-man commands (measured under teach).
        self.hand_window_active_pub = self.create_publisher(
            Bool, "feedback/hand_window_active",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_hand_window_active()
        # The authoritative device state. Latched, because a controller that
        # joins late must not have to wait for the next transition to learn
        # whether the device accepts motion. Attaching the listener publishes
        # the current state immediately.
        self.authority_pub = self.create_publisher(
            AgxDeviceAuthority, "feedback/authority",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._authority.set_on_change(self._publish_authority)
        # The writer's generations, adopted rather than minted. Latched by the
        # writer, so a driver starting after it still learns the current era.
        self.create_subscription(
            AgxUnitSafety, "/unit_safety", self._unit_safety_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # Fire-and-forget: stopping this device never waits on this call.
        self._unit_stop_client = self.create_client(
            RequestUnitStop, "/unit_safety/request_stop"
        )
        # Latched and published once: what this arm can encode. Fixed for the
        # session, unlike the authority beside it, and the only way a consumer
        # can check a control envelope against *this* arm's protocol tier
        # before commanding rather than after being refused (C8).
        self.capability_pub = self.create_publisher(
            AgxDeviceCapability, "feedback/capability",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_capability()
        self.leader_joint_angles_pub = self.create_publisher(
            JointState, "feedback/leader_joint_angles", 1
        )
        if self.gripper is not None:
            self.gripper_status_pub = self.create_publisher(
                GripperStatus, "feedback/gripper_status", 1
            )
            # Latched like the arm's, so a controller joining late learns the
            # gripper's generation without waiting for the next transition.
            self.gripper_authority_pub = self.create_publisher(
                AgxDeviceAuthority, "feedback/gripper/authority",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self._gripper_authority.set_on_change(self._publish_gripper_authority)
        if self.hand is not None:
            self.hand_status_pub = self.create_publisher(
                HandStatus, "feedback/hand_status", 1
            )

    def _setup_subscribers(self):
        self.create_subscription(
            JointState, "control/joint_states", self._joint_states_callback, 1
        )
        if self.gripper is not None:
            self.create_subscription(
                AuthorizedJointTrajectory, "control/gripper/authorized_trajectory",
                self._gripper_authorized_trajectory_callback, 1,
            )
        self.create_subscription(
            JointState, "control/move_j", self._move_j_callback, 1
        )
        self.create_subscription(
            PoseStamped, "control/move_p", self._move_p_callback, 1
        )
        self.create_subscription(
            PoseStamped, "control/move_l", self._move_l_callback, 1
        )
        self.create_subscription(
            PoseArray, "control/move_c", self._move_c_callback, 1
        )
        self.create_subscription(
            JointState, "control/move_js", self._move_js_callback, 1
        )
        self.create_subscription(
            MoveMITMsg, "control/move_mit", self._move_mit_callback, 1
        )
        if self.effector_type == "omnihand":
            self.create_subscription(
                JointState,
                self.omnihand_joint_states_topic,
                self._omnihand_joint_states_callback,
                1,
            )
        if self.hand is not None:
            self.create_subscription(
                HandCmd, "control/hand", self._hand_cmd_callback, 1
            )
            self.create_subscription(
                HandPositionTimeCmd, "control/hand_position_time", 
                self._hand_position_time_cmd_callback, 1
            )

    def _setup_services(self):
        self.create_service(SetBool, "enable_agx_arm", self._enable_callback)
        self.create_service(Empty, "move_home", self._move_home_callback)
        self.create_service(Trigger, "emergency_stop", self._emergency_stop_callback)
        # The MOVE-J rung on its own, without the emergency stop's fault latch:
        # what a controller that lost its feedback escalates to, and what this
        # node commands on its way out.
        self.create_service(
            Trigger, "hold_current_pose", self._hold_current_pose_callback
        )
        self.create_service(
            Trigger, "clear_fault_lockout", self._clear_fault_lockout_callback
        )
        self.create_service(ClaimDevice, "claim_device", self._claim_device_callback)
        if self.gripper is not None:
            # Never plain claim_device: the arm driver owns that name in this
            # namespace, and the two devices are claimed independently.
            self.create_service(
                ClaimDevice, "control/gripper/claim_device",
                self._gripper_claim_device_callback,
            )
            self.create_service(
                Trigger, "control/gripper/stop", self._gripper_stop_callback
            )
        if self.is_nero:
            self.create_service(Trigger, "set_normal_mode", self._set_normal_mode_callback)
            self.create_service(Trigger, "set_leader_mode", self._set_leader_mode_callback)
            self.create_service(
                Trigger, "prepare_hand_window", self._prepare_hand_window_callback
            )
            self.create_service(
                Trigger, "resume_arm_control", self._resume_arm_control_callback
            )
        if not self.is_switch_seamlessly:
            self.create_service(Empty, "exit_teach_mode", self._exit_teach_mode_callback)

    ### utility methods
    def _float_to_ros_time(self, timestamp: float) -> Time:
        """Convert float timestamp to ROS Time message """
        ros_time = Time()
        ros_time.sec = int(timestamp)
        ros_time.nanosec = int((timestamp - ros_time.sec) * 1e9)
        return ros_time

    def _safe_get_value(self, array, index, default=0.0) -> float:
        if index >= len(array):
            return default
        value = array[index]
        return default if math.isnan(value) else value

    def _fresh_snapshot(self) -> "FeedbackSnapshot":
        """Return the last acquisition, or None if it is too old to decide on.

        The bound is ``feedback_timeout``, the same one the recovery watchdog
        uses: at a 200 Hz loop that is 400 missed cycles, so it does not trip on
        jitter, only on an acquisition path that has stopped. Returning None
        there is deliberate — commanding an arm whose feedback nobody is reading
        is worse than refusing, which is what the old per-command SDK read did.
        """
        snapshot = self._latest_snapshot
        if snapshot is None:
            return None
        if time.monotonic() - snapshot.acquired_at > self.feedback_timeout:
            return None
        return snapshot

    def _check_arm_ready(self, snapshot: "FeedbackSnapshot" = None) -> bool:
        joint_states = (
            snapshot.joint_angles if snapshot is not None else self._sdk_read(
                "get_joint_angles", self.agx_arm.get_joint_angles
            )
        )
        if joint_states is None:
            return False
        # hz is pyAgxArm's instantaneous 0.1 s fps window. In a GIL-saturated
        # process (active MIT streaming pegs this node at ~100 % of a core) a
        # starved window reads 0 while candump shows uninterrupted feedback,
        # which used to reject every command and feed the recovery storm. The
        # authoritative liveness signal is the kernel RX timestamp of the last
        # parsed frame: as long as it keeps advancing within feedback_timeout
        # the arm is ready, whatever the instantaneous window says.
        if self._feedback_frame_advancing(joint_states.timestamp):
            return True
        return joint_states.hz > 0

    def _feedback_frame_advancing(self, frame_ts: float) -> bool:
        if frame_ts != self._last_feedback_frame_ts:
            self._last_feedback_frame_ts = frame_ts
            self._last_feedback_advance_monotonic = time.monotonic()
            return True
        return (
            time.monotonic() - self._last_feedback_advance_monotonic
        ) <= self.feedback_timeout

    def _leader_feedback_fresh(self, snapshot: "FeedbackSnapshot" = None) -> bool:
        """True when the leader-angle stream is actively reporting.

        In leader/drag mode this stream — not ``get_joint_angles()`` — is the
        live feedback the firmware pushes, so the bus-recovery watchdog uses it
        as the health signal while normal joint push is silenced.

        Decided on the acquisition snapshot when one is available; the batch
        already carries this stream.
        """
        if not self.is_nero:
            return False
        if snapshot is not None:
            leader_joint_angles = snapshot.leader_joint_angles
        else:
            leader_joint_angles = self._sdk_read(
                "get_leader_joint_angles", self.agx_arm.get_leader_joint_angles
            )
        return leader_joint_angles is not None and leader_joint_angles.hz > 0

    def _check_arm_connected(self, snapshot: "FeedbackSnapshot" = None) -> bool:
        """True when the arm's feedback stream is healthy.

        Despite the name this is feedback health, not transport presence:
        ``is_ok()`` is an FPS window over received frames. For "may a bootstrap
        command be sent", use :meth:`_transport_available`.
        """
        if snapshot is not None:
            return snapshot.is_ok
        return self.agx_arm is not None and bool(
            self._sdk_read("is_ok", self.agx_arm.is_ok)
        )

    def _sdk_read(self, name: str, fn):
        """One bounded SDK read, through the owner of the session.

        Callers outside the publish loop — services, one-off checks — have no
        snapshot to hand. Routing them here keeps the single-owner invariant
        without every call site having to acquire a whole batch. Returns None if
        the session is not currently ours, which reads as "not ready".
        """
        try:
            return self._sdk.call(name, fn, timeout=self.feedback_timeout)
        except (CallOutcomeUnknown, CallNotExecuted):
            return None
        except Exception:
            return None

    def _sdk_write(self, name: str, fn, *, lane: Lane = Lane.CONTROL, timeout=None):
        """One bounded SDK write, through the owner of the session.

        The counterpart to :meth:`_sdk_read`, and deliberately not as forgiving:
        a read that did not happen reads as "not ready", but a mode change that
        did not land must never read as success. Failures propagate to the
        service handler that asked for it.

        Service callbacks used to call the vendor SDK straight from the executor
        thread. Being on a different thread from the worker, a mode change could
        interleave with a MIT setpoint cycle mid-bracket — the arm ends up in a
        mode neither caller believes it is in, and the evidence is a motion that
        was framed wrongly rather than an error anyone sees.
        """
        return self._sdk.call(
            name, fn,
            timeout=self.feedback_timeout if timeout is None else timeout,
            lane=lane,
        )

    def _check_can_control(self) -> bool:
        if self._fault_lockout:
            # After a bus recovery the node holds an explicit fault lockout and
            # refuses ALL new motion until clear_fault_lockout is called, instead
            # of silently re-arming on the next healthy tick.
            if not self._fault_lockout_logged:
                self._fault_lockout_logged = True
                self.get_logger().warn(
                    "Arm in fault lockout after recovery; motion refused until "
                    "clear_fault_lockout is called"
                )
            return False
        if self._bus_is_held():
            # Nobody is on the bus to acknowledge a frame. Every unacknowledged
            # transmission adds 8 to the controller's transmit error counter, so
            # a 200 Hz MIT stream of seven joint frames each reaches the
            # error-passive threshold of 128 in about 11 ms and then stays there,
            # throttled and backing the TX queue up until it reports ENOBUFS.
            # Nothing is lost by staying quiet: the arm is on the watchdog's
            # MOVE-J hold and the feedback push is what tells us it is back.
            # Silent by design, like the hand window below.
            return False
        if self._hand_window_active:
            # A hand window owns the shared side bus and the arm is parked in a
            # driver-level hold. Drop ALL arm command ingress here (the shared
            # control/joint_states follow path, move_j/js, pose/line/circle, and
            # MIT) so no client can inject arm frames until resume_arm_control.
            # Silent by design — a quiesced window drops commands as normal.
            return False
        if not self.control_ready:
            # Startup warm-up: ignore incoming control commands until a valid
            # joint state stream is available.
            return False
        # Decide on the acquisition, not on a read of our own. This check runs
        # once per command at the control rate; doing its own SDK reads put the
        # subscription thread in the worker queue 100 times a second for values
        # the publish loop had already fetched, and the teach-mode read below
        # went straight to the session, which is the second SDK owner the call
        # counter was reporting.
        snapshot = self._fresh_snapshot()
        if snapshot is None:
            self._refuse_stale_ingress()
            return False
        if not self._check_arm_ready(snapshot):
            self.get_logger().warn("Agx_arm is not connected, cannot control")
            return False
        if not self.enable_flag:
            self.get_logger().warn("Agx_arm is not enabled, cannot control")
            return False
        if not self.is_switch_seamlessly:
            arm_status = snapshot.arm_status
            if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                self.get_logger().warn("Agx_arm is in teach mode, cannot control")
                return False
        return True

    def _refuse_stale_ingress(self) -> None:
        """Refuse commands because nothing is reading this arm's feedback.

        Rate-limited: the refusal arrives at the control rate, and on this
        Jetson the logging is itself a measurable load. Worth saying plainly —
        the firmware holds its last setpoint, so refusing here stops new motion
        but does not stop motion already commanded. That gap is the independent
        watchdog's, not this gate's.
        """
        now = time.monotonic()
        if now - self._last_stale_ingress_log_monotonic < self._rejection_log_period_s:
            return
        self._last_stale_ingress_log_monotonic = now
        self.get_logger().error(
            "arm commands refused: no feedback acquisition within "
            f"{self.feedback_timeout:.1f} s. The arm holds its last setpoint; "
            "this gate stops new commands, not motion already running."
        )

    def _unit_safety_callback(self, msg: AgxUnitSafety) -> None:
        """Adopt a generation from the one writer that may allocate them."""
        adopted = self._unit_safety.observe(
            UnitSafetySnapshot(
                epoch=int(msg.epoch),
                stopped=bool(msg.stopped),
                reason=msg.reason,
                writer_id=msg.writer_id,
                incarnation=msg.writer_incarnation,
                started_ns=int(msg.writer_started_ns),
            )
        )
        if adopted:
            self.get_logger().warn(
                f"unit safety generation {msg.epoch} from '{msg.writer_id}': "
                f"stopped={msg.stopped} ({msg.reason})"
            )
        if self._unit_safety.incarnation_changes:
            self.get_logger().error(
                "unit safety writer RESTARTED "
                f"({self._unit_safety.incarnation_changes} so far): this device "
                "is holding a stop because the new writer cannot vouch for what "
                "happened while it was down. An explicit rearm clears it."
            )
        if self._unit_safety.conflicts:
            self.get_logger().error(
                "unit safety CONTRADICTION seen "
                f"({self._unit_safety.conflicts} so far): more than one process "
                "is allocating generations. The stop is being held; find the "
                "second writer."
            )

    def _request_unit_stop(self, reason: str) -> None:
        """Tell the unit a new safety era began. Never blocks the stop itself.

        The hardware is already being stopped by the caller; this is only the
        unit-wide bookkeeping. If the writer is absent the device stays stopped
        on its own epoch, which is the intended degradation — safety local,
        bookkeeping global — so an unavailable service is a warning, not a
        failure of the stop.
        """
        try:
            if not self._unit_stop_client.service_is_ready():
                self.get_logger().warn(
                    "unit safety writer unavailable; this device is stopped on "
                    "its own epoch but the unit generation did not advance"
                )
                return
            request = RequestUnitStop.Request()
            request.requester = self.device_id
            request.reason = reason
            self._unit_stop_client.call_async(request)
        except Exception as exc:
            self.get_logger().warn(f"requesting a unit stop failed: {exc}")

    def _publish_capability(self) -> None:
        """Announce the control envelope this arm's protocol tier can encode."""
        try:
            msg = AgxDeviceCapability()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.device_id = self.device_id
            msg.protocol_tier = str(self.firmware_tier)
            msg.firmware_version = str(
                (self.firmware or {}).get("software_version", "")
            )
            msg.joint_count = int(self.arm_joint_count)
            limits = self.mit_limits
            msg.max_torque = [
                float(limits.torque_limit(joint))
                for joint in range(1, int(self.arm_joint_count) + 1)
            ]
            msg.max_position = float(limits.p_des[1])
            msg.max_velocity = float(limits.v_des[1])
            msg.max_kp = float(limits.kp[1])
            msg.max_kd = float(limits.kd[1])
            self.capability_pub.publish(msg)
            self.get_logger().info(
                f"device capability: tier={msg.protocol_tier} "
                f"torque={msg.max_torque} vel={msg.max_velocity} kp={msg.max_kp}"
            )
        except Exception as e:
            self.get_logger().error(f"publishing device capability failed: {e}")

    def _publish_authority(self, snapshot) -> None:
        """Publish one authority transition. Never breaks the caller.

        Called from whichever thread caused the transition — the publish loop,
        a service handler, the recovery path — so a publish failure must not
        propagate into a control path.
        """
        try:
            msg = AgxDeviceAuthority()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.device_id = snapshot.device_id
            msg.state = AUTHORITY_STATE_CODES[snapshot.state]
            msg.device_epoch = snapshot.device_epoch
            msg.unit_safety_epoch = snapshot.unit_safety_epoch
            msg.unit_stopped = snapshot.unit_stopped
            msg.motion_ready = snapshot.motion_ready
            msg.owner_id = snapshot.owner_id
            msg.reason = snapshot.reason
            self.authority_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"publishing device authority failed: {e}")

    def _sync_authority(self, reason: str) -> None:
        """Bring the device authority in step with the driver's gates.

        Derived from the existing gates on purpose at this stage. Those gates
        are already what the driver acts on; publishing them as one
        authoritative state is what a controller needs before ownership itself
        moves into the authority. The **epochs are not derived** — they come
        from the authority's own transitions, so a command issued before an
        interruption is rejected after it.

        Cheap and idempotent: a transition that changes nothing publishes
        nothing, so calling this every publish cycle costs a lock.
        """
        authority = self._authority
        if self._unit_safety.stopped:
            # Only a unit rearm leaves this state; nothing below may override it.
            return
        if self._estop_latched:
            # This device stopped itself. It stays refusing motion until an
            # operator acknowledges, with or without a unit-wide generation.
            authority.enter_faulted(f"emergency stop latched: {reason}")
            return
        if self._recovery_in_progress:
            authority.enter_recovering(reason)
            return
        if self._fault_lockout:
            authority.enter_faulted(reason)
            return
        if authority.state is DeviceState.FAULTED:
            # The lockout is gone, so clear_fault_lockout was called.
            # Acknowledging the latch is not arming the device.
            authority.acknowledge_fault(reason)
        if not self.enable_flag:
            authority.go_standby(f"{reason}: arm not enabled")
            return
        if self._hand_window_active:
            authority.go_standby(f"{reason}: hand window holds the arm")
            return
        if not self.control_ready:
            authority.go_standby(f"{reason}: waiting for feedback")
            return
        # enable_flag is the joint enable readback and control_ready is
        # advancing feedback: both are checks, not assumptions, so this rearm
        # is backed by evidence rather than by the absence of a complaint.
        authority.rearm(verified=True, detail=reason)

    def _claim_device_callback(self, request, response):
        """Take or give up command of this device. One commander at a time."""
        verdict = (
            self._authority.claim(request.owner_id)
            if request.claim
            else self._authority.release(request.owner_id)
        )
        snapshot = self._authority.snapshot()
        response.accepted = verdict.accepted
        response.reason = "" if verdict.accepted else verdict.reason.value
        response.device_epoch = snapshot.device_epoch
        response.unit_safety_epoch = snapshot.unit_safety_epoch
        action = "claimed by" if request.claim else "released by"
        if verdict.accepted:
            response.message = (
                f"{self.device_id} {action} '{request.owner_id}' at device "
                f"generation {snapshot.device_epoch}"
            )
            self.get_logger().info(response.message)
        else:
            response.message = verdict.detail
            self.get_logger().warn(
                f"{action.split()[0]} refused for '{request.owner_id}': "
                f"{verdict.detail}"
            )
        return response

    def _legacy_ingress_allowed(self, path: str) -> bool:
        """Whether an unauthenticated arm-motion topic may still move the arm.

        These topics bypass everything the authority contract establishes: no
        commander, no device or unit generation, no sequence. A command on them
        cannot be shown to be current, and its sender cannot be shown to be
        entitled to move this arm — which is the whole point of admission.

        Effector control is deliberately not covered here. The gripper and hand
        are separate devices with their own contract (phase 4D); quarantining
        them through the arm's parameter would be the wrong boundary — each has
        its own switch (``allow_legacy_gripper_command_ingress``).
        """
        if self.allow_legacy_motion_ingress:
            return True
        key = (path, "legacy_ingress")
        count = self._command_rejections.get(key, 0) + 1
        self._command_rejections[key] = count
        now = time.monotonic()
        last = self._last_rejection_log_monotonic.get(key, 0.0)
        if now - last >= self._rejection_log_period_s:
            self._last_rejection_log_monotonic[key] = now
            self.get_logger().warn(
                f"{path} is quarantined: it carries no commander and no control "
                f"generation, so this arm will not move on it "
                f"[{count} refused so far]. Use the stamped MIT path, or set "
                "allow_legacy_motion_ingress:=true for a development profile."
            )
        return False

    def _reject_command(self, path: str, rejection) -> None:
        """Refuse one command at the hardware boundary, audibly but not loudly.

        Every rejection is counted; the log line is rate-limited per reason and
        reports how many were suppressed, so a stream of bad commands is visible
        without the logging itself becoming the load.
        """
        key = (path, rejection.reason)
        count = self._command_rejections.get(key, 0) + 1
        self._command_rejections[key] = count

        now = time.monotonic()
        last = self._last_rejection_log_monotonic.get(key, 0.0)
        if now - last < self._rejection_log_period_s:
            return
        self._last_rejection_log_monotonic[key] = now
        self.get_logger().error(
            f"{path} rejected ({rejection.reason}): {rejection.detail} "
            f"[{count} rejected on this path for this reason so far]"
        )

    def _warn_command_limits(self, path: str, outside: list) -> None:
        """Flag a command past the configured joint limits without refusing it.

        The firmware enforces its own limits, and refusing mid-stream would
        freeze a running impedance loop at its last setpoint. Promoted to a
        rejection once a hardware session shows the controller never
        legitimately crosses a limit.
        """
        key = (path, "joint_limits")
        count = self._command_rejections.get(key, 0) + 1
        self._command_rejections[key] = count
        now = time.monotonic()
        last = self._last_rejection_log_monotonic.get(key, 0.0)
        if now - last < self._rejection_log_period_s:
            return
        self._last_rejection_log_monotonic[key] = now
        self.get_logger().warn(
            f"{path} commands a joint past its configured limit: "
            f"{'; '.join(outside)} [{count} so far, still forwarded]"
        )

    def _publish_fault_lockout(self) -> None:
        try:
            self.fault_lockout_pub.publish(Bool(data=self._fault_lockout))
        except Exception:
            pass

    def _publish_hand_window_active(self) -> None:
        # The MIT controller stands down while this is True, so the flag tracks
        # exactly the intentional feedback silence, not the (wider) gate window:
        # with the push still on the controller has live feedback and behaves.
        try:
            self.hand_window_active_pub.publish(
                Bool(data=self._hand_window_push_silenced)
            )
        except AttributeError:
            # Called before the publisher exists (startup ordering); the initial
            # state is published right after the publisher is created.
            pass
        except Exception:
            pass

    def _set_push_silenced(self, silenced: bool) -> None:
        """Set the push-silenced flag and announce it to the MIT controller."""
        changed = self._hand_window_push_silenced != silenced
        self._hand_window_push_silenced = silenced
        if not silenced:
            self._hand_window_silence_started = 0.0
        if changed:
            self._publish_hand_window_active()

    def _enter_fault_lockout(self, reason: str) -> None:
        """Latch a fault lockout so no new motion is accepted until cleared."""
        if not self.require_fault_ack:
            return
        self.control_ready = False
        self._control_ready_logged = False
        if not self._fault_lockout:
            self._fault_lockout = True
            self._fault_lockout_logged = False
            self.get_logger().error(
                f"FAULT LOCKOUT engaged ({reason}); refusing new motion until "
                "clear_fault_lockout is called. Verify the arm before re-enabling."
            )
            self._publish_fault_lockout()
            # Immediately, not on the next publish tick: a controller aborting
            # on authority loss should not stream into a latched fault.
            self._sync_authority(f"fault lockout: {reason}")

    def _clear_fault_lockout_callback(self, request, response):
        del request
        was_locked = self._fault_lockout
        was_estopped = self._estop_latched
        self._estop_latched = False
        self._fault_lockout = False
        self._fault_lockout_logged = False
        self._last_good_feedback_monotonic = time.monotonic()
        self._publish_fault_lockout()
        # This is the operator's "I have looked at it" surface, so it is also
        # what releases a unit stop. It clears the latch; it does not arm the
        # device — that still needs the gates to come back.
        self._sync_authority("fault lockout cleared")
        response.success = True
        # This clears *this device's* latch only. The unit stop is the writer's
        # to release, through unit_safety/rearm — a device clearing a unit-wide
        # generation is exactly the second-allocator problem this split exists
        # to remove, so it is reported rather than silently done here.
        cleared = []
        if was_estopped:
            cleared.append("emergency stop latch cleared")
        if was_locked:
            cleared.append("fault lockout cleared")
        if self._unit_safety.stopped:
            cleared.append(
                "NOTE: a unit safety stop is still in force; call "
                "unit_safety/rearm to release it"
            )
        response.message = "; ".join(cleared) or (
            "nothing to clear: no fault lockout was active"
        )
        self.get_logger().warn(response.message)
        return response

    def _create_pose_cmd(self, pose: Pose) -> list:
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        pose_xyz = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]
        euler_angles = R.from_quat(quaternion).as_euler("xyz", degrees=False)
        tcp_pose = pose_xyz + euler_angles.tolist()
        flange_pose = self.agx_arm.get_tcp2flange_pose(tcp_pose)
        return flange_pose

    def _wait_motion_done(self, timeout: float = 5.0, poll_interval: float = 0.1) -> bool:
        start_time = time.time()

        while True:
            status = self._sdk_read("get_arm_status", self.agx_arm.get_arm_status)
            if status is not None and status.msg.motion_status == self.agx_arm.ARM_STATUS.MotionStatus.REACH_TARGET_POS_SUCCESSFULLY:
                return True
            
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f"Timeout waiting for arm to motion done after {timeout} seconds"
                )
                return False
            time.sleep(poll_interval)

    # _verify_enable outcomes. Three, not two: "the readback says the opposite"
    # and "there is no readback" are different facts about the arm and only the
    # first is evidence of anything.
    ENABLE_VERIFIED = "verified"
    ENABLE_CONTRADICTED = "contradicted"
    ENABLE_UNAVAILABLE = "unavailable"

    def _request_enable(self, enable: bool, timeout: float = 5.0) -> bool:
        """Send the enable/disable command. Claims nothing about the arm.

        Needs a transport only, no feedback. The return value says the SDK
        accepted the call; whether the joints changed state is
        :meth:`_verify_enable`'s question.

        A retry loop on the calling thread rather than a worker cycle: a cycle
        runs to completion once started, so a 5 s one would block the safety
        lane for 5 s.
        """
        deadline = time.time() + timeout
        action_name = "enable" if enable else "disable"
        if not self._transport_available():
            self.get_logger().error(
                f"cannot request {action_name}: no CAN transport"
            )
            return False
        while not self._sdk_write(
            action_name,
            (lambda: self.agx_arm.enable()) if enable
            else (lambda: self.agx_arm.disable()),
        ):
            if time.time() > deadline:
                self.get_logger().error(
                    f"Timeout waiting for arm to {action_name} after {timeout} seconds"
                )
                return False
            time.sleep(0.01)
        return True

    def _verify_enable(self, enable: bool, timeout: float = 5.0) -> str:
        """Read back what the joints report, and update ``enable_flag`` from it.

        The readback comes from the last low-speed feedback frame, which may
        predate the command, so it gets the budget to agree before counting as a
        contradiction. No readback at all is ``ENABLE_UNAVAILABLE``, not a
        contradiction — a mute arm is evidence in neither direction.
        """
        deadline = time.time() + timeout
        joints_enabled = None
        while True:
            # Through the session owner: the single-owner rule covers the
            # verification half as much as the command half.
            readback = self._sdk_read(
                "get_joint_enable_status",
                lambda: self.agx_arm.get_joint_enable_status(255),
            )
            if readback is not None:
                joints_enabled = bool(readback)
                if joints_enabled == enable:
                    break
            if time.time() >= deadline:
                break
            time.sleep(0.02)

        if joints_enabled is None:
            self._enable_verified = False
            self.get_logger().warn(
                f"no joint enable readback within {timeout:.1f}s; the "
                f"{'enable' if enable else 'disable'} was sent but is unverified"
            )
            return self.ENABLE_UNAVAILABLE

        self.enable_flag = joints_enabled
        if joints_enabled == enable:
            self._enable_verified = True
            action_name = "enable" if enable else "disable"
            self.get_logger().info(
                f"All joints {action_name} status is {self.enable_flag}"
            )
            return self.ENABLE_VERIFIED

        self._enable_verified = False
        action_name = "enable" if enable else "disable"
        self.get_logger().error(
            f"{action_name} was accepted by the arm but the joint readback still "
            f"reports enabled={joints_enabled} after {timeout:.1f}s. Treating the "
            f"arm as NOT {action_name}d."
        )
        return self.ENABLE_CONTRADICTED

    def _enable_arm(self, enable: bool = True, timeout: float = 5.0) -> bool:
        """Request enable/disable and report only what the readback confirmed.

        Composition of :meth:`_request_enable` and :meth:`_verify_enable`, which
        stay separate because the command needs a transport and the verification
        needs feedback.
        """
        start = time.time()
        if not self._request_enable(enable, timeout):
            return False
        remaining = max(0.0, timeout - (time.time() - start))
        return self._verify_enable(enable, remaining) == self.ENABLE_VERIFIED

    @staticmethod
    def _pace(next_tick: float, period_s: float) -> float:
        """Sleep to the next period boundary on the monotonic clock.

        Deliberately not ``Node.create_rate``. A ROS rate is backed by a timer
        the executor has to service, so pacing a hardware I/O loop with one
        makes its cadence depend on ROS middleware load — measured: two rate
        objects on the single-threaded executor dropped acquisition from 98/s to
        39/s, which is the coupling this loop split exists to remove.

        A loop that has fallen behind restarts from now rather than firing a
        burst to catch up: the arm cannot be read retroactively, and a burst
        would land on the worker as a backlog.
        """
        next_tick += period_s
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
            return next_tick
        return time.monotonic()

    ### publication thread
    def _publication_loop(self):
        """Publish the latest acquisition. Owns no cadence of its own.

        ``pub_rate`` is a ceiling, not a rate: a snapshot is published once and
        the loop then waits for a newer one. Publishing faster than the arm is
        read would only restamp the same instant, and a consumer counting
        messages would read the duplicate as fresh data.
        """
        name_os_thread("publication")
        period = 1.0 / float(self.pub_rate)
        next_tick = time.monotonic()
        published_at = 0.0

        while rclpy.ok():
            snapshot = self._latest_snapshot
            if snapshot is not None and snapshot.acquired_at != published_at:
                published_at = snapshot.acquired_at
                try:
                    with self.metrics.time_block("publish_batch"):
                        self._publish_joint_states(snapshot)
                        self._publish_pose(snapshot)
                        self._publish_arm_status(snapshot)
                        self._publish_effector_status()
                        self._publish_leader_joint_angles(snapshot)
                except Exception as exc:
                    # A publication failure must never stop the arm being read.
                    self.get_logger().error(f"publish batch failed: {exc}")
            next_tick = self._pace(next_tick, period)

    ### acquisition thread
    def _acquisition_loop(self):
        name_os_thread("acquisition")
        # Paced by the acquisition rate: this loop's period is how often the arm
        # is read and how often the recovery watchdog gets a sample.
        period = 1.0 / float(self.acquisition_rate_hz)
        next_tick = time.monotonic()

        # acquisition loop
        while rclpy.ok():
            # P0 instrumentation: measure how late this iteration is before the
            # recovery check runs, so a stale-feedback trigger can be attributed
            # to local scheduling starvation vs a real dead bus.
            now = time.monotonic()
            if self._last_loop_monotonic:
                self._last_loop_gap_s = now - self._last_loop_monotonic
                self._acq_gap_count += 1
                self._acq_gap_sum_s += self._last_loop_gap_s
                if self._last_loop_gap_s > self._loop_overrun_threshold_s:
                    self._loop_overrun_count += 1
                    self._max_loop_gap_s = max(
                        self._max_loop_gap_s, self._last_loop_gap_s
                    )
                    if now - self._last_overrun_log_monotonic > 5.0:
                        self._last_overrun_log_monotonic = now
                        self.get_logger().warn(
                            "acquisition-loop overrun: "
                            f"{self._last_loop_gap_s * 1000:.0f} ms gap "
                            f"(> {self._loop_overrun_threshold_s * 1000:.0f} ms; "
                            f"count={self._loop_overrun_count}, "
                            f"peak={self._max_loop_gap_s * 1000:.0f} ms). Feedback "
                            "'staleness' this cycle may be local starvation, not a dead bus."
                        )
            self._last_loop_monotonic = now
            if now - self._last_rate_report_monotonic >= self.ACQ_RATE_REPORT_PERIOD_S:
                if self._acq_gap_count:
                    achieved = self._acq_gap_count / self._acq_gap_sum_s
                    # Feedback publication rides this loop, so this number is also
                    # the ceiling on what a teach recording can capture.
                    self.get_logger().info(
                        f"acquisition loop {achieved:.1f} Hz achieved of "
                        f"{self.acquisition_rate_hz:.0f} Hz configured over "
                        f"{self._acq_gap_sum_s:.0f}s"
                    )
                self._last_rate_report_monotonic = now
                self._acq_gap_count = 0
                self._acq_gap_sum_s = 0.0
            # P1: detect a stalled bus (TX ENOBUFS slot leak or stale feedback)
            # and re-establish the link instead of dead-locking until restart.
            # Never let recovery bookkeeping crash the publish loop.
            if self._recovery_in_progress:
                # Recovery owns the SDK session exclusively while it runs, so
                # this loop must not touch it — but it must keep running.
                # Recovering inline used to block this thread for the whole
                # attempt: 13.1 s measured on hardware, during which nothing
                # published state and nothing drained the CAN RX socket.
                self._publish_authority(self._authority.snapshot())
                self._publish_fault_lockout()
                next_tick = self._pace(next_tick, period)
                continue

            # Acquire first, decide second — one worker request per cycle.
            # The batch is a single logical acquisition, and taking it as one
            # bounded task is what removes the race without paying a queue
            # hand-off eight times over. The health checks below used to run
            # *before* it and read the SDK themselves, which is why the call
            # counter still showed two threads once the batch had moved; they
            # now judge the same instant the data came from.
            #
            # (Phase 1B decouples the cadences: the worker will schedule
            # acquisition itself and this loop will only republish.)
            try:
                snapshot = self._sdk.call(
                    "acquire_feedback_snapshot",
                    self._acquire_feedback_snapshot,
                    timeout=self.feedback_timeout,
                    lane=Lane.ACQUISITION,
                )
            except CallOutcomeUnknown as exc:
                self.get_logger().warn(f"feedback acquisition: {exc}")
                snapshot = None
            except CallNotExecuted:
                # Dropped or rejected: the session is not ours at this moment.
                snapshot = None
            except Exception as exc:
                self.get_logger().error(f"feedback acquisition failed: {exc}")
                snapshot = None

            if snapshot is None:
                next_tick = self._pace(next_tick, period)
                continue
            # Published to the command callbacks before anything else is done
            # with it: they decide on it instead of reading the SDK themselves.
            self._latest_snapshot = snapshot

            try:
                if self._should_recover_bus(snapshot):
                    self._request_recovery()
                    next_tick = self._pace(next_tick, period)
                    continue
            except Exception as e:
                self.get_logger().error(f"bus recovery check failed: {e}")

            self._surface_silent_tx_loss(snapshot)
            self._detect_external_motion_mode_change(snapshot)
            self._sync_authority("publish loop")

            if snapshot.is_ok and self._check_arm_ready(snapshot):
                self._last_good_feedback_monotonic = time.monotonic()
                if not self.control_ready:
                    self.control_ready = True
                    self._had_control_ready = True
                    if not self._control_ready_logged:
                        self.get_logger().info(
                            "Agx_arm feedback is ready, control is now enabled"
                        )
                        self._control_ready_logged = True
            elif self._leader_mode_active and self._leader_feedback_fresh(snapshot):
                # In leader/drag mode the firmware disables the normal
                # joint-state push; the live signal is the leader-angle stream.
                # Treat it as a healthy bus so the watchdog does not
                # false-trigger on that intentional silence.
                self._last_good_feedback_monotonic = time.monotonic()

            # The publication thread picks the snapshot up from here. This loop
            # does not wait for it: the arm's read cadence is not the ROS
            # middleware's to set.
            if self.metrics.due():
                report = self.metrics.report()
                if report:
                    self.get_logger().info(report)
            next_tick = self._pace(next_tick, period)

    ### bus recovery (P1)
    @staticmethod
    def _is_recoverable_can_error(exc: Exception) -> bool:
        """Classify a caught send exception as a recoverable bus stall
        (ENOBUFS TX-queue/slot exhaustion or the link going down)."""
        eno = getattr(exc, "errno", None)
        for err in (exc, getattr(exc, "__cause__", None), getattr(exc, "__context__", None)):
            if err is None:
                continue
            cand = getattr(err, "errno", None)
            if cand is None:
                cand = getattr(err, "error_code", None)
            if cand in (errno.ENOBUFS, errno.ENETDOWN):
                return True
        if eno in (errno.ENOBUFS, errno.ENETDOWN):
            return True
        text = str(exc).lower()
        return (
            "no buffer space available" in text
            or "transmit buffer full" in text
            or "error code 105" in text
            or "network is down" in text
            or "is not up" in text
        )

    def _handle_send_failure(self, where: str, exc: Exception) -> None:
        """Log a send failure and, if it is a recoverable bus stall, arm the
        recovery watchdog handled by the publish thread."""
        self.get_logger().error(f"CAN send failed in {where}: {exc}")
        if self._is_recoverable_can_error(exc):
            self._tx_stall_count += 1
            if self._tx_stall_count >= self.bus_recovery_tx_error_threshold:
                self._tx_stall_detected = True

    def _trigger_recovery(self, category: str, reason: str) -> bool:
        """Record why recovery fired and return True to start it."""
        self._recover_reason = reason
        self._recovery_reason_counts[category] = (
            self._recovery_reason_counts.get(category, 0) + 1
        )
        return True

    def _feedback_actually_stale(self, snapshot: "FeedbackSnapshot" = None) -> bool:
        """Kernel-RX-timestamp confirmation that the bus is genuinely silent.

        The FPS window (`is_ok()`/`hz`) and the node-observed feedback clock both
        go stale under local CPU starvation without the bus going down. The
        kernel receive timestamp of the last parsed frame is the ground truth:
        frames queue in the socket buffer and carry their true arrival times when
        the node resumes, so a still-advancing timestamp means the bus is live.
        """
        if snapshot is not None:
            joint_states = snapshot.joint_angles
        else:
            joint_states = self._sdk_read(
                "get_joint_angles", self.agx_arm.get_joint_angles
            )
        if joint_states is None:
            return True
        return not self._feedback_frame_advancing(joint_states.timestamp)

    def _suppress_recovery_as_starvation(self, signal: str) -> bool:
        """Count and log a recovery suppressed because the bus is still live.

        Returns False so the caller stays connected. The bus is confirmed alive
        by the kernel RX timestamp, so refresh the node-observed feedback clock
        to stop a stale-age trigger from re-firing on the next tick.
        """
        self._loop_overrun_suppressions += 1
        self._last_good_feedback_monotonic = time.monotonic()
        if time.monotonic() - self._last_overrun_log_monotonic > 5.0:
            self._last_overrun_log_monotonic = time.monotonic()
            self.get_logger().warn(
                f"{signal} but kernel feedback frames are still advancing "
                f"(last loop gap {self._last_loop_gap_s * 1000:.0f} ms, "
                f"suppressions={self._loop_overrun_suppressions}); treating as "
                "local starvation, not recovering"
            )
        return False

    def _surface_silent_tx_loss(self, snapshot: "FeedbackSnapshot") -> None:
        """Log commands the SDK dropped silently while feedback looked healthy.

        A rising send-error count while RX keeps flowing is the only evidence
        that arm commands are being dropped, so it is surfaced rather than turned
        into a heavyweight recovery (plan section 1.3.2).

        **What it means depends on the bus topology, and the message says which.**
        On a shared side bus this is usually the hand losing arbitration — common,
        expected, not a dead arm. On the dedicated-per-device topology nothing
        else transmits on this bus, so the same counter means something is wrong
        with the arm's own link. Telling an operator to look at the hand there
        would send them to the wrong device.
        """
        if snapshot is None or snapshot.send_error_count < 0:
            return
        count = snapshot.send_error_count
        if count <= self._last_send_error_count:
            self._last_send_error_count = count
            return
        dropped = count - self._last_send_error_count
        self._last_send_error_count = count
        now = time.monotonic()
        if now - self._last_tx_loss_log <= 5.0:
            return
        self._last_tx_loss_log = now
        # Through the session owner like every other read. This one hid until a
        # provoked fault: it only runs when a send was actually dropped, so the
        # call counter showed one owner in every healthy window and three the
        # moment the bus went down.
        last = self._sdk_read("get_last_send_error", self.agx_arm.get_last_send_error)
        cause = (
            "On the shared side bus this is usually hand-frame arbitration loss, "
            "not a dead arm."
            if handshake_required()
            else "This arm owns its bus, so nothing else is competing for it. "
            "If TX packets are also 0 (ip -s link show), the frames never left "
            "the host: check the Jetson 40-pin header "
            "(sudo /opt/nvidia/jetson-io/jetson-io.py), which a kernel update "
            "discards."
        )
        # Read the feedback side off the snapshot rather than asserting it: the
        # guard above does not check it, and dropped sends with no feedback at
        # all is the combination that says the arm is not on the bus.
        feedback_state = (
            "while feedback is live" if snapshot.is_ok
            else "and NO feedback is arriving either"
        )
        self.get_logger().warn(
            f"silent TX loss: {dropped} send(s) dropped (total {count}) "
            f"{feedback_state} (last: {last}); arm commands may not be reaching "
            f"the firmware. {cause}"
        )

    def _link_counter(self, name: str):
        """One sysfs counter for this arm's CAN interface, or None."""
        if not getattr(self, "can_port", None):
            return None
        try:
            if self._link_counter_files is None:
                self._link_counter_files = {}
            handle = self._link_counter_files.get(name)
            if handle is None:
                handle = open(f"/sys/class/net/{self.can_port}/statistics/{name}")
                self._link_counter_files[name] = handle
            handle.seek(0)
            return int(handle.read())
        except (OSError, ValueError):
            return None

    def _link_is_up(self) -> bool:
        try:
            with open(f"/sys/class/net/{self.can_port}/operstate") as state:
                if state.read().strip() != "up":
                    return False
            with open(f"/sys/class/net/{self.can_port}/carrier") as carrier:
                return carrier.read().strip() == "1"
        except OSError:
            return False

    def _bus_hold_defers_recovery(self) -> bool:
        """True while a live-but-quiet bus should be waited out, not recovered.

        The external CAN watchdog terminates the bus and commands its own
        MOVE-J hold, then gives the bus back. Recovery cannot succeed against
        it — three attempts measured 25.7 s of teardown against a bus that was
        never broken — and the lockout it latches afterwards refuses every
        command on the healthy bus that follows.

        The signature that separates a held bus from a broken one, using only
        kernel counters: the link is up, RX has been silent for longer than the
        arm's own update spacing, and TX was still being accepted when the
        silence began. A bus-off controller stops accepting TX and a downed link
        fails the first check, so both fall through to the normal fault path.
        The signature cannot tell a held bus from an unplugged one — that case
        waits out the patience window and then recovers as before.

        The silence is measured against a clock, not against the previous
        sample: this runs at the publish rate (up to 5 ms apart) while complete
        joint updates arrive every 7-10 ms, so "no new frame since last call"
        is true constantly during healthy streaming.

        Once the hold is entered, TX is no longer required to advance — the
        command stream is deliberately stopped while it lasts, so requiring it
        would end the hold the moment the gate took effect.
        """
        rx = self._link_counter("rx_packets")
        tx = self._link_counter("tx_packets")
        previous_rx, previous_tx = self._last_link_rx_packets, self._last_link_tx_packets
        self._last_link_rx_packets, self._last_link_tx_packets = rx, tx
        if rx is None or tx is None:
            return False

        now = time.monotonic()
        if previous_rx is None or rx != previous_rx:
            self._last_rx_advance_monotonic = now
        if self._last_rx_advance_monotonic is None:
            self._last_rx_advance_monotonic = now
        silent_for = now - self._last_rx_advance_monotonic

        if self._bus_held_since_monotonic is not None:
            # Already holding: only RX coming back ends it. TX is gated off by
            # us, so it says nothing about the bus now.
            still_held = self._link_is_up() and silent_for >= self.bus_hold_min_silence_s
        else:
            tx_accepted = previous_tx is not None and tx > previous_tx
            still_held = (
                self._link_is_up()
                and silent_for >= self.bus_hold_min_silence_s
                and tx_accepted
            )

        if not still_held:
            if self._bus_held_since_monotonic is not None:
                held_for = now - self._bus_held_since_monotonic
                self.get_logger().info(
                    f"CAN RX resumed after {held_for:.1f}s of a held bus; no recovery "
                    "was run and no fault was latched. Control re-arms once feedback "
                    "and the joint enable readback verify."
                )
            self._bus_held_since_monotonic = None
            self._bus_held_logged = False
            return False

        if self._bus_held_since_monotonic is None:
            self._bus_held_since_monotonic = now - silent_for
        waited = now - self._bus_held_since_monotonic
        if waited > self.bus_hold_patience_s:
            if self._bus_held_logged:
                self._bus_held_logged = False
                self.get_logger().error(
                    f"bus held quiet for {waited:.1f}s, past the "
                    f"{self.bus_hold_patience_s:.0f}s patience; treating it as a fault"
                )
            return False
        if not self._bus_held_logged:
            self._bus_held_logged = True
            self.get_logger().warn(
                f"CAN RX silent for {silent_for:.2f}s while the link is up and TX was "
                f"still accepted: the bus is live and quiet, which is what the external "
                f"watchdog's hold looks like. Waiting up to "
                f"{self.bus_hold_patience_s:.0f}s for it to come back instead of "
                f"recovering, and holding the command stream off meanwhile — nobody is "
                f"there to acknowledge it. The arm holds on the watchdog's MOVE-J."
            )
        return True

    def _bus_is_held(self) -> bool:
        """Whether the classifier currently believes the bus is being held."""
        return self._bus_held_since_monotonic is not None

    def _should_recover_bus(self, snapshot: "FeedbackSnapshot" = None) -> bool:
        if self._recovery_in_progress:
            return False
        # Explicit escalation from an unverified emergency stop, checked BEFORE
        # the enable flag. `bus_recovery_enabled` turns off the *watchdog* — the
        # automatic reaction to a stalled bus — and it used to switch this off
        # too, so an operator who disabled the watchdog silently disabled the
        # last resort of an emergency stop that could not confirm the arm
        # stopped. Those are not the same decision and no longer share a switch.
        if self._force_recovery:
            self._force_recovery = False
            return self._trigger_recovery(
                "forced_estop", "forced recovery after unverified emergency stop"
            )
        if not self.bus_recovery_enabled:
            return False
        # A hand window may have silenced the firmware feedback push on purpose:
        # the arm is holding in CAN control, the side bus is quiet so the hand
        # can win arbitration. That silence is requested, not a stall — but the
        # watchdog is blind while it lasts, so bound it hard and restore the push
        # (re-arming the watchdog) if a resume never comes.
        if self._hand_window_push_silenced:
            silent_for = time.monotonic() - self._hand_window_silence_started
            if silent_for > self.hand_window_max_silence_s:
                self.get_logger().error(
                    f"hand window silenced feedback for {silent_for:.1f} s "
                    f"(limit {self.hand_window_max_silence_s:.1f} s); restoring the "
                    "push and re-arming the bus watchdog. Arm commands stay gated "
                    "until resume_arm_control."
                )
                self._restore_feedback_push("max silence exceeded")
            else:
                self._last_good_feedback_monotonic = time.monotonic()
            return False
        # Only arm the watchdog once the bus has been healthy at least once,
        # so the normal startup warm-up is never mistaken for a stall.
        if not self._had_control_ready:
            return False
        # Cooldown after a completed recovery: a still-congested bus latches
        # fresh comm errors during the recovery's own sends, and re-recovering
        # immediately only adds disconnect/enable bursts to the flood.
        if (
            time.monotonic() - self._last_recovery_end_monotonic
        ) < self.bus_recovery_cooldown_s:
            if not self._recovery_cooldown_logged:
                self._recovery_cooldown_logged = True
                self.get_logger().warn(
                    "Bus watchdog re-triggered within "
                    f"{self.bus_recovery_cooldown_s:.1f} s of the last recovery; "
                    "holding off (bus likely still congested)"
                )
            return False
        # A live bus that somebody is holding quiet is not a fault yet, and
        # tearing the link down against one only spends the recovery budget.
        if self._bus_hold_defers_recovery():
            return False
        # Path A: an exception propagated out of a send (raise-style comm model).
        if self._tx_stall_detected:
            return self._trigger_recovery(
                "send_failure",
                f"raised send failures (tx_stall_count={self._tx_stall_count})",
            )
        # Path B: the comm layer swallowed an ENOBUFS/ENETDOWN and only recorded
        # it (last_error / swallow-style comm model). Classify so a benign error
        # never forces a heavyweight reconnect.
        try:
            comm_err = self._sdk_read(
                "has_comm_error",
                lambda: self.agx_arm.has_comm_error() and self.agx_arm.get_comm_error(),
            )
        except Exception:
            comm_err = None
        if comm_err and self._is_recoverable_can_error(comm_err):
            if (
                time.monotonic() - self._last_good_feedback_monotonic
            ) > self.feedback_timeout:
                return self._trigger_recovery(
                    "comm_error_stale",
                    f"latched comm error with stale feedback: {comm_err}",
                )
            # Live feedback + latched ENOBUFS = TX congestion (e.g. a foreign
            # unacked frame retransmitting on the shared bus), NOT a dead bus.
            # A socket reconnect cannot flush the kernel TX queue, and every
            # recovery drops control_ready mid-trajectory (observed as MoveIt
            # GOAL_TOLERANCE_VIOLATED aborts). Consume the latch and stay up;
            # the stale-feedback check below still catches a genuinely dead bus.
            self._clear_comm_error()
            if time.monotonic() - self._last_tx_congestion_log > 10.0:
                self._last_tx_congestion_log = time.monotonic()
                self.get_logger().warn(
                    f"CAN TX congestion (latched: {comm_err}) while feedback is live; "
                    "staying connected instead of recovering. If this persists, check "
                    "the shared bus for a retransmitting unacked frame (hand offline?)"
                )
            return False
        try:
            if not (snapshot.is_ok if snapshot is not None
                    else bool(self._sdk_read("is_ok", self.agx_arm.is_ok))):
                # is_ok() is FPS-based (SDK monitor thread) and false-triggers
                # under whole-process CPU/GIL saturation. Only recover when the
                # kernel RX timestamp confirms the bus is actually silent.
                if self._feedback_actually_stale(snapshot):
                    return self._trigger_recovery("not_ok", "driver reports not ok")
                return self._suppress_recovery_as_starvation("is_ok() reads false")
        except Exception:
            return self._trigger_recovery("is_ok_raised", "is_ok() raised")
        if (time.monotonic() - self._last_good_feedback_monotonic) > self.feedback_timeout:
            # The node-observed feedback clock is stale, but a publish-loop stall
            # ages it without the bus going down. Confirm with the kernel RX
            # timestamp before the heavyweight reconnect.
            if self._feedback_actually_stale(snapshot):
                return self._trigger_recovery(
                    "stale_feedback",
                    f"no ready feedback for {self.feedback_timeout:.1f} s",
                )
            return self._suppress_recovery_as_starvation(
                f"no node-observed feedback for {self.feedback_timeout:.1f} s"
            )
        return False

    def _request_recovery(self) -> None:
        """Hand recovery to its own thread and return immediately.

        The acquisition loop detects the fault, latches the authority, and keeps
        running. It does not perform the recovery: `disconnect` alone blocks for
        a second on a sick bus, and inline recovery cost 13.1 s of publish loop
        on hardware — no state published, no RX socket drained, no timeout
        accounting, and no way to see how long recovery had been running.

        Recovery takes exclusive ownership of the SDK session for its duration.
        That is the invariant: one owner at any instant, not one thread for
        everything.
        """
        with self._recovery_lock:
            if self._recovery_in_progress:
                return
            self._recovery_in_progress = True
            self._recovery_started_monotonic = time.monotonic()
        # Ownership handover, in order. Each step exists because skipping it
        # leaves either a commandable device with no session, or two owners on
        # one session.
        #
        #   1-2  authority -> RECOVERING, so normal admission closes
        #   3    device generation bumped: what was in flight is now stale
        #   4    queued stale work discarded rather than delivered late
        #   5-6  the worker finishes its in-flight call and stops dequeuing
        #   7    recovery takes exclusive SDK ownership (its own thread)
        self._authority.enter_recovering(self._recover_reason or "bus recovery")
        self.control_ready = False
        self._control_ready_logged = False
        self._sdk.set_epoch(self._authority.device_epoch)
        self._hold_before_teardown()
        if not self._sdk.quiesce(timeout=self.enable_timeout):
            # A call is still running. Handing the session over anyway would put
            # two owners on it, which is the race this whole structure removes —
            # so recovery waits for the next tick instead of forcing it.
            self.get_logger().error(
                "cannot start recovery: the SDK worker is still executing a "
                "call and ownership must not be taken from it. Retrying."
            )
            with self._recovery_lock:
                self._recovery_in_progress = False
            return
        thread = threading.Thread(
            target=self._recovery_thread, name=f"recovery-{self.device_id}", daemon=True
        )
        thread.start()

    def _may_auto_enable_after_recovery(self) -> bool:
        """True when recovery may return the arm to ordinary enabled operation.

        Recovery restores transport and feedback so the arm can be diagnosed. It
        may not re-arm one that was stopped: a latched emergency stop or an
        active unit stop outlives the transport fault, and only an operator
        clears them.
        """
        if not self.auto_enable:
            return False
        if self._estop_latched:
            self.get_logger().warn(
                "recovery will not auto-enable: an emergency stop is latched"
            )
            return False
        if self._unit_safety.stopped:
            self.get_logger().warn(
                "recovery will not auto-enable: the unit is STOPPED"
            )
            return False
        return True

    def _hold_before_teardown(self) -> None:
        """Put the arm in a firmware MOVE-J hold before recovery takes the session.

        Runs while the worker still dequeues, which is the only window in which
        the hold can be commanded at all: once recovery owns the session the
        worker is quiesced.

        A host-side MIT command is not a hold, and there is no kp=0 rung below
        MOVE-J: such a command ends the moving setpoint without stiffness, so
        the arm sags through exactly the teardown this hold exists to survive.
        Without a trustworthy pose the mode frame still goes out — it needs no
        pose and leaves the firmware's own position controller holding — but no
        hold is claimed, and the independent watchdog is the boundary there.
        """
        self._restore_feedback_push("pre-recovery hold")
        hold_pose = self._capture_hold_pose()
        if hold_pose is None:
            left_mit = self._leave_mit_without_a_pose()
            self.get_logger().error(
                "pre-recovery firmware hold UNAVAILABLE: no trustworthy joint "
                "feedback, so no MOVE-J hold was commanded and none is claimed. "
                "Normal mode was " + ("requested" if left_mit else "NOT sent")
                + " so the firmware leaves MIT and holds its own pose. The "
                "independent watchdog is the protective boundary here."
            )
            return
        try:
            left_mit, move_mode, attempts = self._assert_firmware_hold(hold_pose)
        except Exception as exc:
            self.get_logger().error(f"pre-recovery firmware hold failed: {exc}")
            return
        if left_mit:
            self.get_logger().warn(
                f"pre-recovery firmware hold established after {attempts} "
                "MOVE-J assertion(s); the arm holds its pose through the "
                "transport teardown"
            )
        else:
            self.get_logger().error(
                f"pre-recovery firmware hold NOT confirmed (move_mode="
                f"{move_mode} after {attempts} assertions); the arm may sag "
                "once the link goes down"
            )

    def _recovery_thread(self) -> None:
        """Run one recovery attempt off the acquisition path."""
        try:
            self._recover_bus()
        except Exception as exc:
            self.get_logger().error(f"recovery thread failed: {exc}")
        finally:
            #   8-9  recovery ran and established a verified state
            #   10   the worker gets the SDK session back, under the generation
            #        recovery left behind, so nothing queued during the handover
            #        arrives against the old one
            self._sdk.set_epoch(self._authority.device_epoch)
            self._sdk.resume()
            with self._recovery_lock:
                self._recovery_in_progress = False
            duration = time.monotonic() - self._recovery_started_monotonic
            self.get_logger().warn(
                f"recovery finished after {duration:.1f}s; the acquisition loop "
                "kept publishing throughout"
            )

    @property
    def recovery_active_s(self) -> float:
        """How long recovery has been running, or 0.0 when it is not.

        Exposed because "hide how long recovery has been active" was one of the
        things the inline version did.
        """
        with self._recovery_lock:
            if not self._recovery_in_progress:
                return 0.0
            return time.monotonic() - self._recovery_started_monotonic

    def _recover_bus(self):
        # A recovery must never run against a deliberately silenced bus: every
        # verification step below reads feedback. Restore the push and drop the
        # hand window — the shared bus is the problem now, not the hand.
        # direct: recovery owns the session and has quiesced the worker, so a
        # queued write would never be dequeued.
        self._restore_feedback_push("bus recovery", direct=True)
        self._hand_window_active = False
        # Gate every control callback off immediately; the existing
        # _check_can_control() guard turns this into a hard streaming stop so no
        # command reaches a half-torn-down bus during recovery.
        self.control_ready = False
        self._control_ready_logged = False
        # Expire the frame-advance window so _wait_for_feedback demands a
        # genuinely NEW frame after the reconnect instead of passing on the
        # pre-recovery grace period.
        self._last_feedback_advance_monotonic = (
            time.monotonic() - self.feedback_timeout - 1.0
        )
        # The motion stop already happened in _hold_before_teardown, while the
        # worker could still carry it. From here this is transport repair only.
        try:
            is_ok = self.agx_arm.is_ok()
        except Exception:
            is_ok = False
        self.get_logger().error(
            f"CAN bus stall detected ({self._recover_reason or 'unknown trigger'}; "
            f"tx_stall_count={self._tx_stall_count}, is_ok={is_ok}); starting recovery "
            f"[reasons so far: {self._recovery_reason_counts}; "
            f"loop overruns: {self._loop_overrun_count} "
            f"(peak {self._max_loop_gap_s * 1000:.0f} ms); "
            f"starvation suppressions: {self._loop_overrun_suppressions}]"
        )
        try:
            recovered = False
            rearmed = None
            attempt = 0
            while True:
                attempt += 1
                if attempt > 1:
                    # Back off between attempts so a sick bus is not flooded with
                    # disconnect/enable bursts, and a long hold costs a handful of
                    # attempts rather than thousands.
                    time.sleep(self._recovery_backoff_s(attempt))
                try:
                    self._disconnect_transport()
                except Exception as e:
                    self.get_logger().warn(f"disconnect during recovery failed: {e}")

                if self.bus_recovery_link_reset:
                    self._reset_can_link()

                try:
                    self._connect_transport()
                except Exception as e:
                    self.get_logger().warn(
                        f"reconnect attempt {attempt} failed: {e}"
                    )
                    time.sleep(0.2)
                    continue

                # None means "not requested"; True/False is what the enable
                # readback confirmed. Recovery reports this instead of implying
                # the arm is armed because the link came back.
                rearmed = None
                try:
                    if self._may_auto_enable_after_recovery():
                        rearmed = self._enable_arm(True, self.enable_timeout)
                    self.agx_arm.set_speed_percent(self.speed_percent)
                    self.agx_arm.set_tcp_offset(self.tcp_offset)
                except Exception as e:
                    if self.auto_enable and rearmed is None:
                        rearmed = False
                    self.get_logger().warn(f"re-arm during recovery failed: {e}")

                self._tx_stall_detected = False
                self._tx_stall_count = 0
                # Force a fresh motion-mode handshake on the next command.
                self._current_motion_mode = None
                self.is_mit_mode = False
                # A reconnect returns the firmware to normal push; drop any stale
                # leader-mode assumption so the watchdog uses normal feedback.
                self._leader_mode_active = False

                if self._wait_for_feedback(self.enable_timeout):
                    # Say what was verified, not what happened to be true
                    # afterwards. The 0E fault test logged "recovery succeeded"
                    # for a bus that had come back on its own, and the re-arm
                    # result was discarded on the way there.
                    if rearmed is None:
                        enable_note = "joints enabled: not requested (auto_enable off)"
                    elif rearmed:
                        enable_note = "joints enabled: confirmed by readback"
                    else:
                        enable_note = "joints enabled: NOT confirmed"
                    recovered = True
                    if rearmed is False:
                        self.get_logger().error(
                            f"CAN bus feedback restored on attempt {attempt}, but "
                            f"the arm is not confirmed enabled ({enable_note}). "
                            "The fault lockout stays until this is resolved."
                        )
                    else:
                        self.get_logger().info(
                            f"CAN bus recovery verified on attempt {attempt}: "
                            f"feedback advancing, {enable_note}"
                        )
                    break
                if self.bus_recovery_persistent:
                    running_for = self.recovery_active_s
                    if running_for >= self.bus_recovery_persist_max_s:
                        self.get_logger().error(
                            f"CAN bus recovery gave up after {running_for:.0f}s and "
                            f"{attempt} attempts ({self.bus_recovery_persist_max_s:.0f}s "
                            "limit). Recovery owns the SDK session, so it ends rather "
                            "than leaving the node up and unable to command."
                        )
                        break
                    # Every attempt is one disconnect/connect plus a bounded
                    # feedback wait, so the cost of waiting is a log line, not
                    # bus load. Say so on a schedule instead of per attempt.
                    if attempt == 1 or attempt % self.RECOVERY_PROGRESS_EVERY == 0:
                        self.get_logger().warn(
                            f"CAN bus recovery attempt {attempt} did not restore "
                            f"feedback; retrying until it does ({running_for:.0f}s of "
                            f"{self.bus_recovery_persist_max_s:.0f}s). Set "
                            "bus_recovery_persistent:=false to give up after "
                            f"{self.bus_recovery_max_attempts} attempts instead."
                        )
                    continue
                self.get_logger().warn(
                    f"CAN bus recovery attempt {attempt} did not restore feedback"
                )
                if attempt >= self.bus_recovery_max_attempts:
                    break

            if not recovered:
                self.get_logger().error(
                    "CAN bus recovery exhausted all attempts; arm remains offline"
                )
        finally:
            # Avoid an immediate re-trigger of the feedback watchdog right after
            # recovery; the publish loop re-arms control_ready on fresh feedback.
            self._last_good_feedback_monotonic = time.monotonic()
            # Consume any comm error latched by the recovery's OWN sends
            # (enable/set_speed/set_tcp on a congested bus): pyAgxArm's
            # last_error is only cleared by a clean DATA frame — error frames
            # never clear it — so during an error storm one latched ENOBUFS
            # would otherwise re-trigger the watchdog on the next iteration.
            self._clear_comm_error()
            self._last_recovery_end_monotonic = time.monotonic()
            self._recovery_cooldown_logged = False
            # The lockout exists so the publish loop cannot re-arm control_ready
            # on the next healthy tick and silently accept motion after a
            # recovery nobody verified. A recovery that ended with feedback
            # advancing AND the joint enable confirmed by readback *is* that
            # verification, so it releases on its own; everything else latches
            # and needs clear_fault_lockout.
            if recovered and rearmed is not False:
                self.get_logger().info(
                    "recovery verified the arm (feedback advancing, joint enable "
                    "confirmed); no fault lockout latched"
                )
            else:
                self._enter_fault_lockout(self._recover_reason or "bus recovery")

    def _clear_comm_error(self) -> None:
        try:
            comm = None
            if hasattr(self.agx_arm, "get_comm"):
                comm = self.agx_arm.get_comm()
            elif hasattr(self.agx_arm, "_ctx"):
                comm = self.agx_arm._ctx.get_comm()
            if comm is not None and hasattr(comm, "last_error"):
                comm.last_error = None
        except Exception:
            pass

    def _reset_can_link(self) -> bool:
        """Bring the SocketCAN interface down/up to flush the qdisc and reset
        gs_usb TX echo slots. Requires privileges; failures are non-fatal and
        fall back to socket-only recovery."""
        channel = self.agx_arm.get_channel() or self.can_port
        for action in ("down", "up"):
            cmd = ["sudo", "ip", "link", "set", channel, action]
            try:
                subprocess.run(cmd, check=True, capture_output=True, timeout=5)
            except Exception as e:
                self.get_logger().warn(
                    f"CAN link reset '{' '.join(cmd)}' failed: {e}; "
                    "continuing with socket-only recovery"
                )
                return False
        self.get_logger().info(f"Reset CAN link {channel} (down/up)")
        return True

    def _wait_for_feedback(self, timeout: float) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self.agx_arm.is_ok() and self._check_arm_ready():
                return True
            time.sleep(0.02)
        return False
    
    ### publish methods
    def _get_gripper_joint_data(self):
        if self.gripper is None or not self.gripper.is_ok():
            return []
        status = self.gripper.get_status()
        if status is None:
            return []

        gripper_joint_map = {
            "gripper_joint1":     0.5,
            "gripper_joint2":    -0.5,
        }
        if self.publish_gripper_joint:
            gripper_joint_map[GRIPPER_JOINT_NAME] = 1.0

        return [
            (name, status.width * scale, 0.0, status.force)
            for name, scale in gripper_joint_map.items()
        ]

    def _get_hand_joint_data(self):
        if self.hand is None or not self.hand.is_ok():
            return []
        finger_pos = self.hand.get_finger_position()
        if finger_pos is None:
            return []
        joint_names = REVO2_LEFT_HAND_JOINT_NAMES if self.hand.is_hand_left() else REVO2_RIGHT_HAND_JOINT_NAMES
        
        result = []
        for joint_name in joint_names:
            attr = REVO2_HAND_JOINT_TO_FINGER_ATTR[joint_name][0]
            max_angle = REVO2_HAND_JOINT_TO_FINGER_ATTR[joint_name][1]
            joint_value = max(0.0, min(max_angle, getattr(finger_pos, attr, 0) * max_angle / 100))
            result.append((joint_name, joint_value, 0.0, 0.0))
        
        return result

    def _get_omnihand_joint_data(self):
        if self.omnihand_joint_state is None:
            return []

        positions = list(self.omnihand_joint_state.position)
        velocities = list(self.omnihand_joint_state.velocity)
        efforts = list(self.omnihand_joint_state.effort)
        result = []

        for index, joint_name in enumerate(self.omnihand_joint_state.name):
            result.append(
                (
                    joint_name,
                    self._safe_get_value(positions, index),
                    self._safe_get_value(velocities, index),
                    self._safe_get_value(efforts, index),
                )
            )

        return result

    def _acquire_feedback_snapshot(self) -> FeedbackSnapshot:
        """Read the whole feedback batch. Runs on the SDK worker, as one task.

        Every read here is bounded and served from already-received frames, so
        the whole batch costs about what one `move_mit` does. Nothing in it
        retries or waits on the bus.
        """
        # No record_sdk_call() here: MeasuredSdk wraps the session and counts
        # every call by name already. Counting again made one read look like
        # two — 14 motor-state reads per cycle for seven joints — which is a
        # measurement claiming load that does not exist.
        joint_angles = self.agx_arm.get_joint_angles()

        motor_states = []
        with self.metrics.time_block("motor_state_reads"):
            for joint_index in range(1, self.arm_joint_count + 1):
                motor_states.append(self.agx_arm.get_motor_states(joint_index))

        flange_pose = self.agx_arm.get_flange_pose()
        tcp_pose = (
            self.agx_arm.get_flange2tcp_pose(flange_pose.msg)
            if flange_pose is not None and flange_pose.hz > 0
            else None
        )
        # Read once and shared: the batch used to fetch this twice per cycle,
        # once for the pose and once for the status, with no guarantee the two
        # agreed.
        arm_status = self.agx_arm.get_arm_status()
        leader_joint_angles = self.agx_arm.get_leader_joint_angles()
        is_ok = bool(self.agx_arm.is_ok())
        get_count = getattr(self.agx_arm, "get_send_error_count", None)
        send_error_count = int(get_count()) if get_count is not None else -1

        return FeedbackSnapshot(
            joint_angles=joint_angles,
            motor_states=tuple(motor_states),
            flange_pose=flange_pose,
            tcp_pose=tcp_pose,
            arm_status=arm_status,
            leader_joint_angles=leader_joint_angles,
            is_ok=is_ok,
            send_error_count=send_error_count,
            acquired_at=time.monotonic(),
        )

    def _publish_joint_states(self, snapshot: FeedbackSnapshot):
        joint_states = snapshot.joint_angles
        # Same rule as _check_arm_ready: the instantaneous hz window starves
        # under GIL pressure while frames still arrive, so the advancing frame
        # timestamp is what decides whether the normal stream is alive.
        normal_stream_alive = joint_states is not None and (
            joint_states.hz > 0 or self._feedback_frame_advancing(joint_states.timestamp)
        )
        if not normal_stream_alive:
            # In leader/drag (freedrive) mode the firmware disables the normal
            # joint-state push, so without this fallback feedback/joint_states
            # freezes at the pre-freedrive pose and MoveIt keeps planning from
            # the stale state. The leader-angle stream carries the live joint
            # positions of the dragged arm; republish them here so the shared
            # feedback surface keeps tracking the freedriven pose.
            if self._leader_mode_active:
                self._publish_leader_states_as_joint_states(snapshot)
            return

        velocitys = []
        efforts = []
        # One blocking SDK round trip per joint, every cycle: hot path 1. The
        # counter records the call and the thread, because "all SDK access from
        # one worker" is the Phase 1 exit criterion and this is where the
        # current answer is measured.
        if True:
            for joint_index in range(1, self.arm_joint_count+1):
                ms = snapshot.motor_states[joint_index - 1]
                if ms is None:
                    return
                velocitys.append(ms.msg.velocity)
                efforts.append(ms.msg.torque)

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(joint_states.timestamp)
        
        joints_data = []
        # arm
        joints_data.extend(
            (joint_name, joint_state, velocity, effort)
            for joint_name, joint_state, velocity, effort in zip(self.arm_joint_names, joint_states.msg, velocitys, efforts)
        )
        # gripper
        joints_data.extend(self._get_gripper_joint_data())
        # hand
        joints_data.extend(self._get_hand_joint_data())
        # omnihand bridge
        joints_data.extend(self._get_omnihand_joint_data())
        if joints_data:
            msg.name, msg.position, msg.velocity, msg.effort =map(list, zip(*joints_data))
            self.joint_states_pub.publish(msg)

    def _publish_leader_states_as_joint_states(self, snapshot: "FeedbackSnapshot"):
        # From the worker's snapshot, not a fresh read off the publish thread,
        # which would be a second owner of the session.
        leader_joint_angles = snapshot.leader_joint_angles
        if leader_joint_angles is None:
            return

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(leader_joint_angles.timestamp)
        joints_data = [
            (joint_name, position, 0.0, 0.0)
            for joint_name, position in zip(self.arm_joint_names, leader_joint_angles.msg)
        ]
        joints_data.extend(self._get_gripper_joint_data())
        joints_data.extend(self._get_hand_joint_data())
        joints_data.extend(self._get_omnihand_joint_data())
        if joints_data:
            msg.name, msg.position, msg.velocity, msg.effort = map(list, zip(*joints_data))
            self.joint_states_pub.publish(msg)

    def _publish_pose(self, snapshot: FeedbackSnapshot):
        flange_pose = snapshot.flange_pose
        if flange_pose is None or flange_pose.hz <= 0:
            return

        tcp_pose = snapshot.tcp_pose

        # pose1 = Pose()
        # pose1.position.x, pose1.position.y, pose1.position.z = flange_pose.msg[0:3]
        # roll, pitch, yaw = flange_pose.msg[3:6]
        # quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        # pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w = quaternion

        pose2 = Pose()
        pose2.position.x, pose2.position.y, pose2.position.z = tcp_pose[0:3]
        roll, pitch, yaw = tcp_pose[3:6]
        quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w = quaternion

        msg = PoseStamped()
        msg.header.stamp = self._float_to_ros_time(flange_pose.timestamp)
        # msg.pose = pose1
        # self.flange_pose_pub.publish(msg)
        msg.pose = pose2
        self.tcp_pose_pub.publish(msg)

    def _publish_arm_status(self, snapshot: FeedbackSnapshot):
        arm_status = snapshot.arm_status
        if arm_status is None:
            return

        msg = AgxArmStatus()
        msg.ctrl_mode = arm_status.msg.ctrl_mode
        msg.arm_status = arm_status.msg.arm_status
        msg.mode_feedback = arm_status.msg.mode_feedback
        msg.teach_status = arm_status.msg.teach_status
        msg.motion_status = arm_status.msg.motion_status
        msg.trajectory_num = arm_status.msg.trajectory_num
        err = arm_status.msg.err_status
        for i in range(self.arm_joint_count):
            angle_limit = getattr(err, f"joint_{i+1}_angle_limit")
            comm_status = getattr(err, f"communication_status_joint_{i+1}")

            msg.joint_angle_limit.append(angle_limit)
            msg.communication_status_joint.append(comm_status)

        # The raw code was never published before, so no consumer could act on
        # an arm fault at all. Its bits are what the flags above decode.
        msg.fault_code = int(getattr(arm_status.msg, "err_code", 0) or 0) & 0xFFFF
        msg.any_fault = bool(
            msg.fault_code
            or any(msg.joint_angle_limit)
            or any(msg.communication_status_joint)
        )

        self.arm_status_pub.publish(msg)

    def _publish_leader_joint_angles(self, snapshot: FeedbackSnapshot):
        leader_joint_angles = snapshot.leader_joint_angles
        if leader_joint_angles is None:
            return

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(leader_joint_angles.timestamp)
        msg.name = self.arm_joint_names
        msg.position = leader_joint_angles.msg
        msg.velocity = [0.0] * self.arm_joint_count
        msg.effort = [0.0] * self.arm_joint_count
        self.leader_joint_angles_pub.publish(msg)

    def _publish_gripper_status(self):
        status = self.gripper.get_status()
        if status is not None:
            msg = GripperStatus()
            msg.header.stamp = self._float_to_ros_time(status.timestamp)
            msg.width = status.width
            msg.force = status.force
            msg.voltage_too_low = status.voltage_too_low
            msg.motor_overheating = status.motor_overheating
            msg.driver_overcurrent = status.driver_overcurrent
            msg.driver_overheating = status.driver_overheating
            msg.sensor_status = status.sensor_status
            msg.driver_error_status = status.driver_error_status
            msg.driver_enable_status = status.driver_enable_status
            msg.homing_status = status.homing_status
            self.gripper_status_pub.publish(msg)

    def _publish_hand_status(self):
        hand_status = self.hand.get_status()
        finger_pos = self.hand.get_finger_position()
        if hand_status is not None:
            msg = HandStatus()
            msg.header.stamp = self._float_to_ros_time(hand_status.timestamp)
            msg.left_or_right = hand_status.left_or_right
            # status
            msg.thumb_tip_status = hand_status.thumb_tip
            msg.thumb_base_status = hand_status.thumb_base
            msg.index_finger_status = hand_status.index_finger
            msg.middle_finger_status = hand_status.middle_finger
            msg.ring_finger_status = hand_status.ring_finger
            msg.pinky_finger_status = hand_status.pinky_finger
            # position
            if finger_pos is not None:
                msg.thumb_tip_pos = finger_pos.thumb_tip
                msg.thumb_base_pos = finger_pos.thumb_base
                msg.index_finger_pos = finger_pos.index_finger
                msg.middle_finger_pos = finger_pos.middle_finger
                msg.ring_finger_pos = finger_pos.ring_finger
                msg.pinky_finger_pos = finger_pos.pinky_finger
            self.hand_status_pub.publish(msg)

    def _publish_effector_status(self):
        self._sync_gripper_authority()
        if self.gripper is not None and self.gripper.is_ok():
            self._publish_gripper_status()
        if self.hand is not None and self.hand.is_ok():
            self._publish_hand_status()

    ### arm control callbacks
    def _control_arm_joints(self, joint_pos):
        arm_joints = {
            name : value
            for name, value in joint_pos.items()
            if name in self.arm_joint_names
        }
        if arm_joints:
            joints = [arm_joints.get(name, 0) for name in self.arm_joint_names]
            try:
                if self.fast_mode:
                    self.agx_arm.move_js(joints)
                    self.is_mit_mode = True
                    self._current_motion_mode = 'js'
                else:
                    self.agx_arm.move_j(joints)
                    self.is_mit_mode = False
                    self._current_motion_mode = 'j'
            except Exception as e:
                self._handle_send_failure("_control_arm_joints", e)

    def _control_gripper_joint(self, joint_pos, joint_effort):
        if self.gripper is None:
            return
        if not self._legacy_gripper_ingress_allowed():
            return

        # gripper_name → width scale
        gripper_joint_map = {
            GRIPPER_JOINT_NAME:   1.0,
            "gripper_joint1":    2.0,
            "gripper_joint2":    2.0,
        }

        matched = next(
            ((name, scale) for name, scale in gripper_joint_map.items()
             if name in joint_pos),
            None,
        )
        if matched is None:
            return

        joint_name, scale = matched
        width = abs(joint_pos[joint_name]) * scale
        # Use default force if effort is 0 or not specified
        force = joint_effort.get(joint_name, self.gripper_default_effort) or self.gripper_default_effort

        # Legacy ingress: this surface carries no authority, so it cannot refuse
        # a stale or reordered command. Use control/gripper/authorized_trajectory.
        self._submit_gripper_move(width, force)

    def _legacy_gripper_ingress_allowed(self) -> bool:
        """Whether a bare gripper command on control/joint_states may execute.

        The gripper's own quarantine, separate from the arm's: a command there
        carries no owner and no generation, so the authority checks have nothing
        to check. Production commanders use
        control/gripper/authorized_trajectory.
        """
        if self.allow_legacy_gripper_command_ingress:
            return True
        key = ("control/joint_states gripper", "legacy_ingress")
        count = self._command_rejections.get(key, 0) + 1
        self._command_rejections[key] = count
        now = time.monotonic()
        last = self._last_rejection_log_monotonic.get(key, 0.0)
        if now - last >= self._rejection_log_period_s:
            self._last_rejection_log_monotonic[key] = now
            self.get_logger().warn(
                "bare gripper commands on control/joint_states are quarantined: "
                "they carry no commander and no device generation, so a stale "
                f"one cannot be refused [{count} refused so far]. Use "
                "control/gripper/authorized_trajectory, or set "
                "allow_legacy_gripper_command_ingress:=true for development."
            )
        return False

    ### gripper device authority
    def _publish_gripper_authority(self, snapshot) -> None:
        """Publish one gripper authority transition. Never breaks the caller."""
        try:
            msg = AgxDeviceAuthority()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.device_id = snapshot.device_id
            msg.state = AUTHORITY_STATE_CODES[snapshot.state]
            msg.device_epoch = snapshot.device_epoch
            msg.unit_safety_epoch = snapshot.unit_safety_epoch
            msg.unit_stopped = snapshot.unit_stopped
            msg.motion_ready = snapshot.motion_ready
            msg.owner_id = snapshot.owner_id
            msg.reason = snapshot.reason
            self.gripper_authority_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"publishing gripper authority failed: {e}")

    def _sync_gripper_authority(self) -> None:
        """Track the gripper's readiness against its own readback.

        ``is_ok()`` is the positive evidence ``rearm`` demands: the effector is
        answering on the bus. An unverified rearm drops the device to STANDBY,
        which is the same thing said the other way round, so one call covers
        both directions.
        """
        if self._gripper_authority is None:
            return
        live = self.gripper is not None and self.gripper.is_ok()
        self._gripper_authority.rearm(
            verified=live,
            detail="gripper readback live" if live else "no gripper readback",
        )

    def _gripper_claim_device_callback(self, request, response):
        """Take or give up command of the gripper, independently of the arm."""
        authority = self._gripper_authority
        verdict = (
            authority.claim(request.owner_id)
            if request.claim
            else authority.release(request.owner_id)
        )
        snapshot = authority.snapshot()
        response.accepted = verdict.accepted
        response.reason = "" if verdict.accepted else verdict.reason.value
        response.device_epoch = snapshot.device_epoch
        response.unit_safety_epoch = snapshot.unit_safety_epoch
        action = "claimed by" if request.claim else "released by"
        if verdict.accepted:
            response.message = (
                f"{snapshot.device_id} {action} '{request.owner_id}' at device "
                f"generation {snapshot.device_epoch}"
            )
            self.get_logger().info(response.message)
        else:
            response.message = verdict.detail
            self.get_logger().warn(
                f"gripper claim refused for '{request.owner_id}': {verdict.detail}"
            )
        return response

    def _gripper_stop_callback(self, request, response):
        """Cancel the pending target and hold the current width.

        A cancel-and-hold, not a latching stop: the gripper re-arms on the next
        admitted command. Only the unit generation can latch it STOPPED.
        """
        del request
        self._gripper_target = None
        status = self.gripper.get_status() if self.gripper is not None else None
        if status is None:
            response.success = False
            response.message = "no gripper readback; nothing commanded"
            self.get_logger().warn(response.message)
            return response
        self._submit_gripper_move(status.width, self.gripper_default_effort)
        response.success = True
        response.message = f"gripper holding at {status.width:.4f} m"
        self.get_logger().info(response.message)
        return response

    def _gripper_authorized_trajectory_callback(self, msg: AuthorizedJointTrajectory):
        """Drive the gripper from a trajectory that carries its own authority.

        Admitted on the stamp the command arrived with, never on a field
        substituted from this node's own state. The final point is the target:
        the gripper closes its own loop on width and has no use for the path.
        """
        if self.gripper is None or self._gripper_authority is None:
            return
        stamp = CommandStamp(
            owner_id=msg.authority.owner_id,
            device_epoch=int(msg.authority.device_epoch),
            unit_safety_epoch=int(msg.authority.unit_safety_epoch),
            sequence=int(msg.authority.sequence),
        )
        verdict = self._gripper_authority.admit(stamp)
        if not verdict:
            self.get_logger().warn(f"gripper command refused: {verdict.detail}")
            return
        if not msg.trajectory.points:
            self.get_logger().warn("gripper trajectory carries no points")
            return

        names = list(msg.trajectory.joint_names)
        positions = list(msg.trajectory.points[-1].positions)
        width = self._gripper_width_from(names, positions)
        if width is None:
            self.get_logger().warn(
                f"gripper trajectory names no known finger joint: {names}"
            )
            return
        self._gripper_target = (width, self.gripper_default_effort)
        self._submit_gripper_move(width, self.gripper_default_effort)

    def _gripper_width_from(self, names, positions) -> Optional[float]:
        """Opening width from whichever finger joint the trajectory carries.

        Each finger travels half the width, and the second one mirrors the
        first, so either alone determines the opening.
        """
        for name, scale in (("gripper_joint1", 2.0), ("gripper_joint2", 2.0)):
            for index, joint_name in enumerate(names):
                if joint_name.endswith(name) and index < len(positions):
                    return abs(positions[index]) * scale
        return None

    def _submit_gripper_move(self, width: float, force: float) -> None:
        """One bounded gripper transmit on the arm's worker. Clamps nothing."""
        if not math.isfinite(width) or not math.isfinite(force):
            self.get_logger().warn(
                f"gripper command not finite (width={width}, force={force})"
            )
            return
        if not (AgxGripperWrapper.WIDTH_MIN <= width <= AgxGripperWrapper.WIDTH_MAX):
            self.get_logger().warn(
                f"gripper width {width:.4f} outside "
                f"[{AgxGripperWrapper.WIDTH_MIN}, {AgxGripperWrapper.WIDTH_MAX}] m"
            )
            return
        if not (AgxGripperWrapper.FORCE_MIN <= force <= AgxGripperWrapper.FORCE_MAX):
            self.get_logger().warn(
                f"gripper force {force:.3f} outside "
                f"[{AgxGripperWrapper.FORCE_MIN}, {AgxGripperWrapper.FORCE_MAX}] N"
            )
            return
        self._sdk.submit(
            "move_gripper",
            lambda: self.gripper.move(width=width, force=force),
            lane=Lane.CONTROL,
            epoch=self._authority.device_epoch,
            replace_key="gripper_move",
        )

    def _control_hand_joints(self, joint_pos):
        hand_joints = {
            name : max(0, min(100, int(value / REVO2_HAND_JOINT_TO_FINGER_ATTR[name][1] * 100)))
            for name, value in joint_pos.items()
            if name in REVO2_HAND_JOINT_NAMES
        }
        if not hand_joints:
            return
    
        if self.hand is None:
            self.get_logger().warn("revo2 hand not initialized")
            return
        finger_kwargs = {
            REVO2_HAND_JOINT_TO_FINGER_ATTR[name][0] : value
            for name, value in hand_joints.items()
            if name in REVO2_HAND_JOINT_TO_FINGER_ATTR
        }
        if finger_kwargs:
            try:
                self.hand.position_ctrl(**finger_kwargs)
            except ValueError as e:
                self.get_logger().warn(str(e))

    def _joint_states_callback(self, msg: JointState):
        if not self._check_can_control():
            return

        joint_pos = {
            name: self._safe_get_value(msg.position, idx)
            for idx, name in enumerate(msg.name)
        }
        joint_effort = {
            name: self._safe_get_value(msg.effort, idx)
            for idx, name in enumerate(msg.name)
        }
        if self._legacy_ingress_allowed("control/joint_states arm follow"):
            self._control_arm_joints(joint_pos)
        # The effectors are separate devices with their own contract, so they
        # carry their own quarantine rather than the arm's.
        self._control_gripper_joint(joint_pos, joint_effort)
        self._control_hand_joints(joint_pos)

    def _omnihand_joint_states_callback(self, msg: JointState):
        self.omnihand_joint_state = msg

    def _move_j_callback(self, msg: JointState):
        if not self._check_can_control():
            return
        if not self._legacy_ingress_allowed("control/move_j"):
            return

        joint_pos = {}
        for idx, joint_name in enumerate(msg.name):
            joint_pos[joint_name] = self._safe_get_value(msg.position, idx)
        joints = [joint_pos.get(i, 0) for i in self.arm_joint_names]
        try:
            # Through the worker even here. The quarantine decides who may enter
            # this path; it does not license a second SDK owner once a developer
            # opens it, which would race the worker and confound a stop test.
            self._sdk_write("legacy_move_j", lambda: self.agx_arm.move_j(joints))
            self.is_mit_mode = False
            self._current_motion_mode = 'j'
        except Exception as e:
            self._handle_send_failure("_move_j_callback", e)

    def _move_p_callback(self, msg: PoseStamped):
        if not self._check_can_control():
            return
        if not self._legacy_ingress_allowed("control/move_p"):
            return

        pose_cmd = self._create_pose_cmd(msg.pose)
        try:
            self.agx_arm.move_p(pose_cmd)
            self.is_mit_mode = False
            self._current_motion_mode = 'p'
        except Exception as e:
            self._handle_send_failure("_move_p_callback", e)

    def _move_l_callback(self, msg: PoseStamped):
        if not self._check_can_control():
            return
        if not self._legacy_ingress_allowed("control/move_l"):
            return

        pose_cmd = self._create_pose_cmd(msg.pose)
        try:
            self.agx_arm.move_l(pose_cmd)
            self.is_mit_mode = False
            self._current_motion_mode = 'l'
        except Exception as e:
            self._handle_send_failure("_move_l_callback", e)

    def _move_c_callback(self, msg: PoseArray):
        if not self._check_can_control():
            return
        if not self._legacy_ingress_allowed("control/move_c"):
            return
        if len(msg.poses) < 3:
            self.get_logger().error(
                f"move_c requires at least 3 poses, but got {len(msg.poses)}"
            )
            return

        pose_start = self._create_pose_cmd(msg.poses[0])
        pose_mid = self._create_pose_cmd(msg.poses[1])
        pose_end = self._create_pose_cmd(msg.poses[2])
        try:
            self.agx_arm.move_c(pose_start, pose_mid, pose_end)
            self.is_mit_mode = False
            self._current_motion_mode = 'c'
        except Exception as e:
            self._handle_send_failure("_move_c_callback", e)

    def _move_js_callback(self, msg: JointState):
        if not self._check_can_control():
            return
        if not self._legacy_ingress_allowed("control/move_js"):
            return

        joint_pos = {}
        for idx, joint_name in enumerate(msg.name):
            joint_pos[joint_name] = self._safe_get_value(msg.position, idx)
        joints = [joint_pos.get(i, 0) for i in self.arm_joint_names]
        try:
            self.agx_arm.move_js(joints)
            self.is_mit_mode = True
            self._current_motion_mode = 'js'
        except Exception as e:
            self._handle_send_failure("_move_js_callback", e)

    def _move_mit_callback(self, msg: MoveMITMsg):
        # Arm command ingress (incl. the hand-window gate) is centralized in
        # _check_can_control(); MIT is dropped there while a hand window is open.
        if not self._check_can_control():
            return

        # Identity before payload: an out-of-date or unowned command is not
        # worth range-checking, and admission advances the sequence watermark
        # only for a command that is otherwise going to be sent.
        if self.require_command_stamp:
            verdict = self._authority.admit(
                CommandStamp(
                    owner_id=msg.owner_id,
                    device_epoch=int(msg.device_epoch),
                    unit_safety_epoch=int(msg.unit_safety_epoch),
                    sequence=int(msg.sequence),
                )
            )
            if not verdict.accepted:
                self._reject_command(
                    "move_mit",
                    _StampRejection(verdict.reason.value, verdict.detail),
                )
                return

        # The hardware boundary is the input contract; SDK clamping is a last
        # protection behind it. A rejected message is rejected whole: the
        # firmware holds its last setpoint, and admitting some joints while
        # dropping others would leave the arm in a pose nobody commanded.
        rejection = validate_mit_command(
            msg.joint_index, msg.p_des, msg.v_des, msg.kp, msg.kd, msg.torque,
            joint_count=self.arm_joint_count, limits=self.mit_limits,
        )
        if rejection is not None:
            self._reject_command("move_mit", rejection)
            return

        outside = positions_outside_joint_limits(
            msg.joint_index, msg.p_des, self.arm_joint_names, self.arm_joint_limits
        )
        if outside:
            self._warn_command_limits("move_mit", outside)

        # Copied out of the message: the task outlives this callback, and what
        # it sends to hardware must not depend on a ROS buffer still being
        # around and unchanged when it runs.
        joint_index = tuple(msg.joint_index)
        p_des = tuple(msg.p_des)
        v_des = tuple(msg.v_des)
        kp = tuple(msg.kp)
        kd = tuple(msg.kd)
        torque = tuple(msg.torque)

        # The setpoint reaches the arm on the session owner's thread, not on
        # this one. This was the last hot-path SDK writer outside the worker and
        # the largest in the system — 700 calls/s measured from the subscription
        # thread — so "exactly one SDK owner at any instant" was true for reads
        # and false for commands until here.
        #
        # Submitted as a *cycle*: one queue entry, executed one frame at a time.
        # The first version sent it as a single task and hardware showed why
        # that is wrong — 6.4 ms mean and 21 ms worst case of work nothing can
        # preempt, which is the whole emergency-stop budget inside one entry.
        # Seven independent submissions would be wrong the other way: two
        # setpoints would interleave and leave the arm holding half of each. A
        # cycle is one instruction to the device and seven preemption points.
        #
        # ``replace_key``: on a streaming path only the newest setpoint is worth
        # sending. A superseded one is dropped while still queued rather than
        # delivered late, which is what keeps a stalled worker from working
        # through a backlog of stale poses.
        steps = [
            ("mit_mode_bracket", self._open_mit_mode_bracket),
        ]
        steps += [
            (
                "move_mit",
                partial(
                    self._send_mit_joint,
                    joint_index[i], p_des[i], v_des[i], kp[i], kd[i], torque[i],
                ),
            )
            for i in range(len(joint_index))
        ]
        call = self._sdk.submit_cycle(
            "send_mit_setpoint",
            steps,
            lane=Lane.CONTROL,
            epoch=self._authority.device_epoch,
            replace_key="move_mit",
            # Closes the bracket the first step opened, whatever happened in
            # between. Leaving auto mode-ctrl disabled would silently change how
            # every later command is framed.
            always=("set_auto_set_motion_mode_enabled", self._close_mit_mode_bracket),
        )
        if call.outcome is not CallOutcome.PENDING:
            # submit_cycle() settles a stale-epoch drop or a full queue on the
            # spot, so a settled call here is the guaranteed-not-sent case and
            # only that. Anything still pending may yet reach the arm and is not
            # reported as refused.
            self._reject_command(
                "move_mit", _StampRejection(call.outcome.value, call.detail)
            )

    def _open_mit_mode_bracket(self) -> None:
        """First step of a setpoint cycle. Runs on the SDK worker.

        The mode bookkeeping lives here rather than in the callback so the flags
        record what was actually sent, in the order the worker sent it. Set from
        the callback they would describe a frame that may not have gone out yet,
        and two queued setpoints would each believe the other had done the mode
        transition.
        """
        # Send ArmMsgModeCtrl once per mode transition, not once per joint.
        # Without this, 7 redundant mode-ctrl frames are sent per callback at
        # 100 Hz, which saturates the CAN TX queue (~700 extra frames/sec/arm).
        if self._current_motion_mode != 'mit':
            self.agx_arm.set_motion_mode('mit')
            self._current_motion_mode = 'mit'
            self.is_mit_mode = True
        self.agx_arm.set_auto_set_motion_mode_enabled(False)

    def _close_mit_mode_bracket(self) -> None:
        self.agx_arm.set_auto_set_motion_mode_enabled(True)

    def _send_mit_joint(self, joint_index, p_des, v_des, kp, kd, t_ff) -> None:
        """One joint frame — the unit a stop can now get in front of."""
        try:
            self.agx_arm.move_mit(
                joint_index=joint_index,
                p_des=p_des,
                v_des=v_des,
                kp=kp,
                kd=kd,
                t_ff=t_ff,
            )
        except Exception as e:
            self._handle_send_failure("move_mit", e)

    ### effector control callbacks
    def _hand_position_time_cmd_callback(self, msg: HandPositionTimeCmd):
        if self.hand is None:
            self.get_logger().warn("revo2 hand not initialized")
            return
        
        try:
            self.hand.position_time_ctrl(
                mode="pos",
                thumb_tip=msg.thumb_tip_pos,
                thumb_base=msg.thumb_base_pos,
                index_finger=msg.index_finger_pos,
                middle_finger=msg.middle_finger_pos,
                ring_finger=msg.ring_finger_pos,
                pinky_finger=msg.pinky_finger_pos,
            )
            self.hand.position_time_ctrl(
                mode="time",
                thumb_tip=msg.thumb_tip_time,
                thumb_base=msg.thumb_base_time,
                index_finger=msg.index_finger_time,
                middle_finger=msg.middle_finger_time,
                ring_finger=msg.ring_finger_time,
                pinky_finger=msg.pinky_finger_time,
            )
        except ValueError as e:
            self.get_logger().error(f"hand control param error: {e}")

    def _hand_cmd_callback(self, msg: HandCmd):
        if self.hand is None:
            self.get_logger().warn("revo2 hand not initialized")
            return
        
        mode_to_method = {
            "position": self.hand.position_ctrl,
            "speed": self.hand.speed_ctrl,
            "current": self.hand.current_ctrl,
        }

        mode = msg.mode.lower()        
        if mode not in mode_to_method:
            self.get_logger().warn(f"unknown hand control mode: {mode}")
            return

        try:
            mode_to_method[mode](
                thumb_tip=msg.thumb_tip,
                thumb_base=msg.thumb_base,
                index_finger=msg.index_finger,
                middle_finger=msg.middle_finger,
                ring_finger=msg.ring_finger,
                pinky_finger=msg.pinky_finger,
            )
        except ValueError as e:
            self.get_logger().error(f"hand control param error: {e}")

    ### service callbacks
    def _enable_callback(self, request, response):
        """Request the enable state, then report only what the readback proved.

        Gated on the transport, not on feedback: a session is enough to attempt
        a bounded command, never enough to report success. An unverified
        ``enable_flag`` does not short-circuit either, or a mute arm would
        answer "already enabled" and skip the bootstrap.
        """
        try:
            if self._enable_verified and request.data == self.enable_flag:
                response.success = True
                response.message = (
                    "Agx_arm already enabled" if request.data
                    else "Agx_arm already disabled"
                )
                return response

            action = "enable" if request.data else "disable"
            if not self._transport_available():
                response.success = False
                response.message = "Agx_arm has no CAN transport"
                self.get_logger().warn(
                    f"no CAN transport, cannot {action} Agx_arm"
                )
                return response

            if not request.data:
                # Close the local motion gate before the request goes out, so
                # nothing new is admitted whatever the readback later says.
                self.control_ready = False
                self._control_ready_logged = False

            sent = self._request_enable(request.data, self.enable_timeout)
            if not sent:
                response.success = False
                response.message = f"Failed to send {action} to Agx_arm"
                return response

            # Verification needs feedback, so ask for it. Commands no motion.
            self._ensure_feedback_push_enabled(f"{action} verification", force=True)
            outcome = self._verify_enable(request.data, self.enable_timeout)
            if outcome == self.ENABLE_VERIFIED:
                response.success = True
                response.message = (
                    "Agx_arm enabled" if request.data else "Agx_arm disabled"
                )
            elif outcome == self.ENABLE_UNAVAILABLE:
                response.success = False
                response.message = (
                    f"{action} was sent but no feedback came back to verify it; "
                    "the arm state is unknown"
                )
            else:
                response.success = False
                response.message = (
                    f"{action} was sent but the joint readback contradicts it"
                )
        except Exception as e:
            response.success = False
            response.message = f"Exception occurred: {str(e)}"
            self.get_logger().error(f"Failed to set enable state: {str(e)}")
        return response

    def _move_home_callback(self, request, response):
        try:
            if not self._check_arm_ready():
                self.get_logger().warn("Agx_arm is not connected, cannot move to home position")
            elif not self.enable_flag:
                self.get_logger().warn("Agx_arm is not enabled, cannot move to home position")
            else:
                if not self.is_switch_seamlessly:
                    arm_status = self._sdk_read(
                        "get_arm_status", self.agx_arm.get_arm_status
                    )
                    if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                        self.get_logger().warn("Agx_arm is in teach mode, cannot move to home position")
                        return response
                    
                home = [0] * self.arm_joint_count
                if self.is_mit_mode:
                    self._sdk_write("move_js", lambda: self.agx_arm.move_js(home))
                else:
                    self._sdk_write("move_j", lambda: self.agx_arm.move_j(home))
                if self._wait_motion_done():
                    self.get_logger().info("Agx_arm moved to home position successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to move to home position: {str(e)}")
        return response

    def _sdk_safety(self, name: str, fn, timeout: float = None):
        """One SDK call on the safety lane, ahead of queued control work.

        For the emergency stop and the pre-recovery firmware hold — the latter
        runs before the worker is quiesced, while this path still works.
        Destructive recovery must **not** come through here: once it has
        quiesced the worker and taken the session, a submission would wait for a
        handover that only completes when recovery ends. It calls the SDK
        directly because at that moment it is the owner.
        """
        return self._sdk.call(
            name, fn,
            timeout=self.feedback_timeout if timeout is None else timeout,
            lane=Lane.SAFETY,
        )

    def _leave_mit_without_a_pose(self) -> bool:
        """Take the firmware out of MIT when no trustworthy pose exists.

        The bottom rung this driver can still reach on its own. ``set_normal_mode``
        is a mode frame: it needs no feedback and no pose, ends the MIT setpoint
        the firmware would otherwise keep executing, and hands the arm to its own
        position controller, which holds it where it is. Unverifiable by
        construction — the same missing feedback that cost us the pose also costs
        us the readback — so it is attempted and reported, never claimed.

        There is deliberately no MIT command here. A kp=0 command has no
        stiffness and sags, so it is not a rung of this ladder at any height.
        """
        try:
            self._sdk_safety("set_normal_mode", self.agx_arm.set_normal_mode)
        except Exception as exc:
            self.get_logger().error(f"could not leave MIT without a pose: {exc}")
            return False
        self.is_mit_mode = False
        self._current_motion_mode = None
        return True

    # An emergency stop is only trustworthy if the arm is confirmed stopped in
    # feedback: under ENOBUFS the SDK silently drops the stop command and still
    # returns success (plan section 1.3.2), so the command alone proves nothing.
    ACQ_RATE_REPORT_PERIOD_S = 30.0

    ESTOP_VELOCITY_THRESHOLD_RAD_S = 0.05
    ESTOP_VERIFY_TIMEOUT_S = 0.5
    # Two feedback frames must be at least this far apart before a finite
    # difference means anything; below it encoder quantisation dominates the
    # estimate and a moving arm can read as settled.
    VELOCITY_MIN_SAMPLE_DT_S = 0.01
    # An unverified stop re-asserts the same hold instead of escalating to a
    # different command. There is no stronger motion primitive on this side:
    # see docs/sprint_refactor/reference/emergency_stop_ladder.md.
    ESTOP_HOLD_ATTEMPTS = 3

    def _sample_joint_positions(self):
        """One timestamped joint-position sample, or None when unavailable.

        On the safety lane: this is how a stop is verified, and a verification
        read that queues behind the very control stream the stop is trying to
        end would report "cannot tell" for the wrong reason.
        """
        try:
            js = self._sdk_safety("get_joint_angles", self.agx_arm.get_joint_angles)
        except Exception:
            return None
        if js is None:
            return None
        try:
            return float(js.timestamp), [float(value) for value in js.msg]
        except (TypeError, ValueError):
            return None

    def _derive_joint_velocities(
        self,
        timeout_s: float,
        poll_s: float,
    ) -> tuple[Optional[list], str]:
        """Per-joint speed from timestamped positions, or None with a reason.

        The SDK's reported motor velocity is not usable: every Nero driver tier
        overwrites it with 0.0 before returning motor feedback (an acknowledged
        vendor workaround, ``# TODO: remove this after the bug is fixed``), and
        the v112 tier inherits that behaviour from v111. Reading it makes a
        moving arm look settled, which is the opposite of what a stop check is
        for, so speed is differentiated from joint positions here instead.

        ``dt`` comes from the feedback timestamps, not the wall clock: a stalled
        bus that keeps returning the same frame yields no evidence rather than a
        confident zero.
        """
        first = self._sample_joint_positions()
        if first is None:
            return None, "no joint feedback"
        t0, q0 = first

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            time.sleep(poll_s)
            sample = self._sample_joint_positions()
            if sample is None:
                return None, "joint feedback stopped mid-measurement"
            t1, q1 = sample
            dt = t1 - t0
            if dt < self.VELOCITY_MIN_SAMPLE_DT_S:
                continue
            if len(q1) != len(q0):
                return None, "joint feedback width changed mid-measurement"
            return [abs(b - a) / dt for a, b in zip(q0, q1)], f"dt={dt * 1e3:.0f}ms"
        return None, "feedback timestamp did not advance"

    def _arm_velocities_settled(
        self,
        threshold_rad_s: float = ESTOP_VELOCITY_THRESHOLD_RAD_S,
        timeout_s: float = ESTOP_VERIFY_TIMEOUT_S,
        poll_s: float = 0.02,
    ) -> "StopVerification":
        """Decide whether the arm is confirmed stopped, and on what evidence.

        Returns a :class:`StopVerification`. ``settled`` alone is not the whole
        answer: a caller must be able to tell "confirmed stopped" from "could
        not tell", because the second is a stop that was *commanded* and never
        verified. Missing or stalled feedback therefore yields
        ``evidence=False`` rather than a phantom success.
        """
        deadline = time.monotonic() + timeout_s
        detail = "no measurement attempted"
        had_evidence = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            velocities, detail = self._derive_joint_velocities(remaining, poll_s)
            if velocities is None:
                return StopVerification(False, had_evidence, detail)
            had_evidence = True
            peak = max(velocities) if velocities else 0.0
            if peak < threshold_rad_s:
                return StopVerification(
                    True, True, f"peak {peak:.3f} rad/s ({detail})"
                )
            detail = f"peak {peak:.3f} rad/s ({detail})"
        return StopVerification(False, had_evidence, detail)

    def _emergency_stop_callback(self, request, response):
        """Best-effort, UNCONDITIONAL, feedback-VERIFIED stop.

        Deliberately not gated on readiness or enable state: during a runaway
        (stalled feedback, recovery in progress) those checks are exactly what
        is broken, and the firmware keeps executing the last MIT command it
        received. Order per attempt: damped MIT zero (needs no feedback) ->
        MOVE-J hold at the current pose -> VERIFY in feedback that joint
        velocities settled. An unverified stop re-asserts the same hold, up to
        ``ESTOP_HOLD_ATTEMPTS`` times, and then asks for a bus-recovery link
        reset as transport repair. It never logs a plain success when the arm is
        not confirmed stopped.

        The MOVE-J hold is the whole ladder on this side. The vendor
        ``electronic_emergency_stop`` is deliberately not used: it is a damped
        descent, so it releases the stiffness that keeps a raised arm up. Where
        no hold can be established, the external CAN watchdog is the boundary —
        it takes the bus when this side stops signalling
        (``docs/sprint_refactor/reference/emergency_stop_ladder.md``).

        Returns a Trigger result so a supervisor can act on the outcome:
        ``success`` is True only when the arm is CONFIRMED stopped in feedback;
        ``message`` states the verified/unverified result and, when the last
        resort forced a bus recovery, that a fault lockout will latch and the
        caller must call ``clear_fault_lockout`` before re-arming motion.
        """
        stopped = False
        recovery_requested = False
        # "No hold was commanded" is not the same failure as "a hold was
        # commanded and could not be verified", and only the second is about the
        # measurement. The caller is told which.
        hold_commanded = False
        # Seeded, because every branch below reports through it. If a stage
        # raises before the settle check runs, the handler used to fall through
        # to an unbound name and the whole callback died — an emergency stop
        # that answered nothing at all.
        verification = StopVerification(
            False, False, "no verification was reached"
        )
        # This device is stopped unilaterally and immediately: a device-level
        # fault on its own epoch, needing no other process. Whatever was issued
        # before this point is stale for this device from here on.
        self._estop_latched = True
        self._authority.enter_faulted("emergency stop requested")
        # Carry the new generation to the worker at once. The safety lane
        # overtakes queued work but does not invalidate it, so without this a
        # MIT cycle queued before the stop still executes after the hold.
        self._sdk.set_epoch(self._authority.device_epoch)
        # The unit-wide statement that a new safety era began is the writer's to
        # make, and is requested without waiting for it.
        self._request_unit_stop("emergency stop requested")

        if self._recovery_in_progress:
            # Recovery owns the SDK session exclusively and is in the middle of
            # tearing the link down. Every stage below would issue a call
            # against that session, competing with the owner for a transport
            # that is not there — and none of them could be verified, because
            # verification reads the same dead link.
            #
            # So: latch, request the unit stop, and say plainly that a new
            # hardware stop cannot be confirmed right now. Recovery attempts the
            # firmware MOVE-J hold before it takes the session, so the arm is
            # covered by that attempt when feedback was still trustworthy; where
            # it was not, the independent watchdog is the boundary.
            response.success = False
            response.message = (
                f"{self.arm_type} stop=unverifiable — the device is RECOVERING "
                f"({self.recovery_active_s:.1f}s so far) and its SDK session "
                "belongs to recovery, so no stop was attempted over it. This "
                "device is latched and refuses motion, and a unit stop was "
                "requested. A NEW hardware stop cannot be confirmed until "
                "recovery ends — cut arm power if the arm is "
                "moving."
            )
            self.get_logger().error(
                "EMERGENCY STOP DURING RECOVERY: latched locally and requested "
                "a unit stop, but issued no SDK call — recovery owns the "
                "session. A new hardware stop is NOT verifiable in this window."
            )
            return response

        # Every stage below is verified in feedback, so an open hand window must
        # not keep that feedback silenced through an emergency stop.
        self._restore_feedback_push("emergency stop")
        try:
            # Every stage goes on the safety lane, which is what puts it in
            # front of the setpoints already queued rather than behind them.
            #
            # The ladder has exactly one rung, re-tried: MOVE-J at the current
            # pose. An unverified stop re-asserts that same hold rather than
            # escalating to a different command, because no stronger motion
            # primitive exists here that still holds the arm up. Nothing below
            # it is a MIT command: a kp=0 zero would end the moving setpoint
            # without stiffness, which sags the arm.
            for attempt in range(1, self.ESTOP_HOLD_ATTEMPTS + 1):
                hold_pose = self._capture_hold_pose(lane=Lane.SAFETY)
                if hold_pose is None:
                    # A pose synthesised from stale feedback would be a wrong
                    # hold, not a missing one. The mode frame still goes out —
                    # it needs no pose, ends the MIT setpoint, and leaves the
                    # firmware's own position controller holding where the arm
                    # is — but it cannot be verified, so nothing is claimed and
                    # the external CAN watchdog owns this regime.
                    left_mit = self._leave_mit_without_a_pose()
                    verification = StopVerification(
                        False, False,
                        "no trustworthy joint feedback, so no pose hold was "
                        "commanded; normal mode "
                        + ("requested" if left_mit else "could not be sent"),
                    )
                    self.get_logger().error(
                        f"{self.arm_type} emergency stop: no trustworthy joint "
                        "feedback, so no MOVE-J hold could be commanded. Normal "
                        "mode was " + ("requested" if left_mit else "NOT sent")
                        + " so the firmware leaves MIT and holds its own pose, "
                        "but nothing here can confirm it. The external CAN "
                        "watchdog is the protective boundary — cut arm power to "
                        "stop the arm."
                    )
                    break

                self._command_firmware_hold(hold_pose)
                hold_commanded = True
                self.get_logger().info(
                    f"Emergency stop hold commanded on {self.arm_type} "
                    f"(attempt {attempt}/{self.ESTOP_HOLD_ATTEMPTS})"
                )

                verification = self._arm_velocities_settled()
                stopped = verification.verified
                if stopped:
                    self.get_logger().info(
                        f"Emergency stop verified: {self.arm_type} joints settled "
                        f"({verification.detail})"
                    )
                    break
                self.get_logger().error(
                    f"Emergency stop NOT verified on attempt {attempt}/"
                    f"{self.ESTOP_HOLD_ATTEMPTS} ({verification.detail}) — "
                    "re-asserting the hold at the pose the arm is at now"
                )

            if not stopped:
                # Transport repair, not a further motion escalation: the link
                # reset re-attempts the hold on its way in and flushes a stuck
                # MIT setpoint the firmware would keep executing. Beyond it the
                # external CAN watchdog is the boundary — it takes the bus once
                # this side stops signalling.
                self.get_logger().error(
                    "Emergency stop still not verified after "
                    f"{self.ESTOP_HOLD_ATTEMPTS} hold attempts — requesting a "
                    "bus-recovery link reset. Firmware has no MIT command "
                    "watchdog: cut arm power to stop it."
                )
                self._force_recovery = True
                recovery_requested = True
        except Exception as e:
            self.get_logger().error(f"Emergency stop failed: {e}")
        # Three outcomes, not two. "Commanded but unverifiable" is its own
        # result: the caller must be able to distinguish an arm that is proven
        # at rest from one whose feedback could not answer the question, and
        # only the first justifies dispatching further motion.
        if stopped:
            response.success = True
            # The latched unit stop has to be named here. Without it a caller
            # reads "confirmed stopped", finds the arm refusing motion, and has
            # no way to know what is holding it or how to release it.
            response.message = (
                f"{self.arm_type} stop=verified — confirmed stopped "
                f"({verification.detail}); this device is latched and refuses "
                "motion until clear_fault_lockout"
            )
        else:
            response.success = False
            if not hold_commanded:
                state = "no_hold_commanded"
                self.get_logger().error(
                    f"EMERGENCY STOP COMMANDED NOTHING for {self.arm_type} "
                    f"({verification.detail}) — no hold could be issued at all; "
                    "treat the arm as still moving; cutting arm power is the only remaining stop"
                )
            elif not verification.evidence:
                state = "commanded_unverifiable"
                self.get_logger().error(
                    f"EMERGENCY STOP COMMANDED BUT UNVERIFIABLE for {self.arm_type} "
                    f"({verification.detail}) — no usable velocity evidence; treat "
                    "the arm as still moving; cutting arm power is the only remaining stop"
                )
            else:
                state = "unverified"
                self.get_logger().error(
                    f"EMERGENCY STOP UNVERIFIED for {self.arm_type} "
                    f"({verification.detail}) — do not trust the software stop; "
                    "cut arm power"
                )
            if recovery_requested:
                # The publish thread will run _recover_bus and latch a fault
                # lockout; the caller (supervisor/operator) owns clearing it.
                response.message = (
                    f"{self.arm_type} stop={state} ({verification.detail}) — forced "
                    "bus recovery requested; fault_lockout=latched, call "
                    "clear_fault_lockout before re-arming. Cut arm power "
                    "if it still moves."
                )
            else:
                response.message = (
                    f"{self.arm_type} stop={state} ({verification.detail}) — "
                    "cut arm power"
                )
        return response

    def _exit_teach_mode_callback(self, request, response):
        try:
            arm_status = self._sdk_read(
                "get_arm_status", self.agx_arm.get_arm_status
            )
            if not self.is_piper:
                self.get_logger().warn("exit teach mode just piper series supported")
                return response

            if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                home = [0] * self.arm_joint_count
                self._sdk_write("move_js", lambda: self.agx_arm.move_js(home))
                time.sleep(2)
                # The vendor teach-exit recipe, and the only remaining use of
                # electronic_emergency_stop in this node: Piper-only, at the home
                # pose, paired with the reset() below that leaves the state again.
                # It is a mode transition, not a rung of the safety ladder — that
                # ladder ends at the MOVE-J hold and never issues this call,
                # because a damped descent is not a hold.
                #
                # Safety lane: it must not queue behind the motion this same
                # sequence just issued.
                self._sdk_write(
                    "electronic_emergency_stop",
                    self.agx_arm.electronic_emergency_stop,
                    lane=Lane.SAFETY,
                )
                self._sdk_write("move_j", lambda: self.agx_arm.move_j(home))
                time.sleep(0.3)
                self._sdk_write("reset", self.agx_arm.reset)
                time.sleep(0.5)
                self._enable_arm(True)
                self._sdk_write("move_j", lambda: self.agx_arm.move_j(home))
                self.get_logger().info("Exited teach mode successfully")
            else:
                self.get_logger().info("Agx_arm is not in teach mode")
        except Exception as e:
            self.get_logger().error(f"Failed to exit teach mode: {e}")
        return response

    def _set_normal_mode_callback(self, request, response):
        del request
        try:
            if not self.is_nero:
                response.success = False
                response.message = "set_normal_mode is only supported for Nero"
                return response
            # Transport only, no feedback and no enable precondition: this is
            # the operator's escalation for an arm holding a persisted
            # leader/follower linkage. Re-asserts a mode, commands no motion;
            # admission still rests on control_ready and the authority.
            if not self._transport_available():
                response.success = False
                response.message = "Agx_arm has no CAN transport"
                return response

            self._sdk_write("set_normal_mode", self.agx_arm.set_normal_mode)
            self.is_mit_mode = False
            self._leader_mode_active = False
            self._current_motion_mode = None
            # This call re-enables the push itself, so any hand-window silence is
            # over — drop the flag (and un-stand-down the MIT controller) without
            # sending a second mode frame.
            self._set_push_silenced(False)
            # Normal joint push resumes now; give it a fresh watchdog window so
            # the transition itself is never read as a stall.
            self._last_good_feedback_monotonic = time.monotonic()
            # The SDK not raising is not evidence the arm changed mode, and this
            # service is reachable on an arm that answers nothing. Say which.
            response.success = True
            if self._check_arm_connected():
                response.message = "Switched to normal mode"
            else:
                response.message = (
                    "normal mode sent, but no feedback is arriving to confirm "
                    "the arm took it"
                )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to switch to normal mode: {e}"
            self.get_logger().error(response.message)
        return response

    # Nero feedback ctrl_mode values that mean the arm is under active command
    # control and holding position (a hand window needs a firm hold, not a
    # backdrivable or idle arm): CAN_CTRL=0x01, TCP_CTRL=0x08.
    _HOLD_CTRL_MODES = frozenset({0x01, 0x08})

    def _ctrl_mode_is_hold(self, ctrl_mode) -> bool:
        """True only when the readback confirms an active holding ctrl_mode.

        This is the source of truth for the handoff, NOT the set_normal_mode
        call: on Nero V112 that call is a firmware no-op (leader/follower only),
        so a hand window must be verified by what the arm actually reports, never
        by assuming the mode switch took. Backdrivable modes (TEACHING=0x02,
        LINKAGE_TEACHING_INPUT=0x06), STANDBY, and UNKNOWN all fail here.
        """
        if ctrl_mode is None:
            return False
        try:
            return int(ctrl_mode) in self._HOLD_CTRL_MODES
        except (TypeError, ValueError):
            return False

    def _move_mode_is_mit(self, mode_feedback) -> bool:
        """True only when the readback POSITIVELY reports a MIT move mode.

        The MIT code is firmware-dependent (0x04 below v111, 0x06 from v111), so
        it is taken from the active driver rather than hardcoded. An unreadable
        or UNKNOWN mode returns False here — it is not a positive MIT reading.
        """
        if mode_feedback is None:
            return False
        try:
            return int(mode_feedback) in nero_can_push.mit_move_mode_codes(
                self.agx_arm
            )
        except (TypeError, ValueError):
            return False

    def _move_mode_is_firmware_hold(self, mode_feedback) -> bool:
        """True only when the readback POSITIVELY reports a non-MIT move mode.

        In MIT the arm only does what the host streams, so a hold needs a mode
        where the vendor's own position controller closes the loop. Confirmation
        therefore requires a known non-MIT code, not merely the absence of a MIT
        one: an unreadable or UNKNOWN mode is no evidence, and the moment a hold
        matters most is exactly when the status read is least likely to answer.
        """
        if mode_feedback is None:
            return False
        try:
            return int(mode_feedback) in nero_can_push.firmware_hold_move_mode_codes(
                self.agx_arm
            )
        except (TypeError, ValueError):
            return False

    def _assert_firmware_hold(self, hold_pose) -> tuple:
        """Re-assert a MOVE-J hold until the firmware confirms it left MIT.

        The one hold this driver has. A single MOVE-J mode frame can be dropped
        on a loaded one-shot bus, leaving the firmware in MIT executing the
        preceding kp=0 damped stop — which is zero stiffness, so the arm sags.
        Re-send the same-pose, motionless MOVE-J until the readback stops
        reporting a MIT move mode, bounded by ``hand_window_hold_assert_s``.

        Runs on the SAFETY lane: the emergency stop and the pre-recovery hold
        both call it while a control stream may still be queued, and the hold
        has to get in front of that stream rather than behind it.

        Returns ``(left_mit, move_mode, attempts)``: whether the firmware is
        confirmed out of MIT, the last move-mode read, and how many sends it
        took.
        """
        self._sdk_safety(
            "set_auto_set_motion_mode_enabled",
            lambda: self.agx_arm.set_auto_set_motion_mode_enabled(True),
        )
        deadline = time.monotonic() + self.hand_window_hold_assert_s
        attempts = 0
        while True:
            self._sdk_safety("move_j", lambda: self.agx_arm.move_j(hold_pose))
            self._current_motion_mode = 'j'
            attempts += 1
            time.sleep(self.hand_window_hold_poll_s)
            status = self._arm_status_msg_safety()
            move_mode = None if status is None else getattr(
                status, "mode_feedback", None
            )
            if self._move_mode_is_firmware_hold(move_mode):
                return True, move_mode, attempts
            if time.monotonic() >= deadline:
                return False, move_mode, attempts

    #: Minimum spacing between motion-mode re-assertions forced by the readback.
    #: The mode frame is sent once per transition on purpose; re-asserting it on
    #: every publish cycle would add ~200 frames/s/arm to the TX queue.
    EXTERNAL_MODE_REASSERT_PERIOD_S = 0.5

    def _detect_external_motion_mode_change(self, snapshot: "FeedbackSnapshot") -> None:
        """Drop the motion-mode cache when the arm reports a mode we did not set.

        ``set_motion_mode`` is sent once per transition, so the cache is what
        decides whether the next setpoint re-frames the arm into MIT. Anything
        that takes the arm out of MIT from outside this driver leaves the cache
        claiming 'mit' while the firmware runs its own position controller, and
        every MIT frame after that is accepted by the bus and ignored by the
        arm. The external CAN watchdog's ``MOVE-J`` hold does exactly this by
        design, and a hold shorter than ``feedback_timeout`` never reaches the
        recovery path that would otherwise invalidate the cache.

        Only a positively reported non-MIT mode counts; an unreadable or unknown
        readback is not evidence of a mode change.
        """
        if self._current_motion_mode != 'mit':
            return
        now = time.monotonic()
        if now - self._last_external_mode_reassert < self.EXTERNAL_MODE_REASSERT_PERIOD_S:
            return
        arm_status = getattr(snapshot, "arm_status", None)
        status = getattr(arm_status, "msg", None)
        move_mode = None if status is None else getattr(status, "mode_feedback", None)
        if not self._move_mode_is_firmware_hold(move_mode):
            return
        self._last_external_mode_reassert = now
        self._current_motion_mode = None
        self.is_mit_mode = False
        self.get_logger().warn(
            f"arm reports move mode {move_mode} while this driver was streaming "
            "MIT; something outside this driver changed the mode (the external "
            "CAN watchdog's MOVE-J hold does). Re-asserting MIT on the next "
            "setpoint."
        )

    def _arm_ctrl_mode(self):
        """Current firmware ctrl_mode from feedback, or None if unreadable."""
        status = self._arm_status_msg()
        return None if status is None else status.ctrl_mode

    def _arm_move_mode(self):
        """Current firmware mode_feedback (MOVE P/J/L/C/MIT/CPV), or None."""
        status = self._arm_status_msg()
        return None if status is None else getattr(status, "mode_feedback", None)

    def _arm_status_msg_safety(self):
        """Arm status read on the safety lane, for the firmware-hold loop.

        The hold polls this while a control stream may still be queued; on the
        default lane the read waits behind that stream and the bounded
        assertion window expires without ever seeing the mode change.
        """
        status = self._sdk.call(
            "get_arm_status", self.agx_arm.get_arm_status,
            timeout=self.feedback_timeout, lane=Lane.SAFETY,
        )
        return None if status is None else status.msg

    def _arm_status_msg(self):
        try:
            status = self._sdk_read("get_arm_status", self.agx_arm.get_arm_status)
        except Exception:
            return None
        if status is None:
            return None
        return status.msg

    def _command_firmware_hold(self, hold_pose) -> bool:
        """Command the firmware to hold ``hold_pose``. The terminal stopped state.

        Returns whether the firmware is confirmed out of MIT. The older Piper
        firmware that cannot switch seamlessly gets a single MOVE-JS instead and
        offers no mode readback to confirm.
        """
        if not self.is_switch_seamlessly:
            self._sdk_safety("move_js", partial(self.agx_arm.move_js, hold_pose))
            self.is_mit_mode = True
            self._current_motion_mode = 'js'
            return True

        # The same bounded re-assertion the hand window uses, not a single
        # MOVE-J. One dropped mode frame leaves the firmware in MIT executing
        # the kp=0 damped stop, which has no stiffness — the arm sags instead
        # of holding.
        left_mit, move_mode, attempts = self._assert_firmware_hold(hold_pose)
        self.is_mit_mode = False
        if not left_mit:
            self.get_logger().error(
                f"emergency stop: firmware still reports MIT "
                f"(move_mode={move_mode}) after {attempts} MOVE-J "
                "assertions; the arm is NOT in a firmware hold"
            )
        return left_mit

    def _capture_hold_pose(self, *, lane: Lane = Lane.DIAGNOSTIC):
        """Current joint pose from trustworthy live feedback, or None.

        The emergency stop reads on the SAFETY lane: a pose read queued behind
        the control stream the stop is displacing answers "no pose", and no pose
        means no hold.
        """
        try:
            if lane is Lane.SAFETY:
                js = self._sdk_safety(
                    "get_joint_angles", self.agx_arm.get_joint_angles
                )
            else:
                js = self._sdk_read(
                    "get_joint_angles", self.agx_arm.get_joint_angles
                )
        except Exception:
            return None
        alive = js is not None and (
            js.hz > 0 or self._feedback_frame_advancing(js.timestamp)
        )
        if not alive:
            return None
        return list(js.msg)

    def _silence_push_frame(self) -> None:
        """Send one feedback-push DISABLE frame through the session owner.

        The paired write to :meth:`_ensure_feedback_push_enabled`.
        """
        self._sdk_write(
            "disable_can_push",
            lambda: nero_can_push.set_can_push(self.agx_arm, False),
        )

    def _silence_feedback_push(self) -> tuple:
        """Stop the firmware feedback push for a hand window.

        This is the only part of the window that actually frees the shared side
        bus. Measured on hardware: the ~2150 frames/s the side bus carries while
        the arm merely holds are the arm's own feedback push (Nero->host, low
        CAN IDs), not MIT commands — gating the commands leaves the rate
        unchanged and the hand's high-ID CANFD frames keep losing arbitration.

        Deliberately NOT done by switching mode. The stock SDK only silences the
        push as a side effect of ``set_leader_mode``/``set_follower_mode``, and
        leader mode is zero-force drag: the firmware has no gravity model for
        this mounting pose and none for the end-effector payload, so the arm
        would sag instead of holding. ``nero_can_push.set_can_push`` sends only
        the push bit and leaves the CAN-control hold in place.
        """
        if not self.hand_window_silence_feedback:
            return False, "feedback push left ON (hand_window_silence_feedback=false)"
        try:
            # A real mode frame on the wire, not local bookkeeping: it obeys the
            # single-owner rule like every other write.
            self._silence_push_frame()
        except Exception as e:
            return False, f"could not silence the feedback push: {e}"
        # Latch the flag BEFORE verifying: the DISABLE frame may land at any
        # moment from here on, and the bus-recovery watchdog must never charge
        # that requested silence to the arm as a stall.
        now = time.monotonic()
        self._hand_window_silence_started = now
        self._last_good_feedback_monotonic = now
        # Also tells the MIT controller to stand down (the feedback it holds on
        # is now gone on purpose), so it does not dead-man-flood the gate.
        self._set_push_silenced(True)

        if self._wait_for_feedback_silenced(self.hand_window_silence_verify_s):
            return True, "feedback push silenced (verified: feedback stopped)"

        # One re-send: a dropped mode frame is the expected failure here, and
        # the bus has had the verify window to drain in the meantime.
        try:
            self._silence_push_frame()
        except Exception as e:
            self._restore_feedback_push("silence re-send failed")
            return False, f"could not re-send the feedback-push silence: {e}"
        if self._wait_for_feedback_silenced(self.hand_window_silence_verify_s):
            return True, "feedback push silenced (verified after one re-send)"

        # The push is provably still running. Do NOT leave the flag latched on a
        # bus that is not silent — but send an explicit ENABLE first, so a
        # late-landing DISABLE cannot mute the arm behind an un-blinded
        # watchdog, and confirm that ENABLE in feedback before un-blinding it.
        self._restore_feedback_push("silence could not be verified")
        if not self._wait_for_feedback_resumed(self.hand_window_silence_verify_s):
            self.get_logger().error(
                "feedback push neither silenced nor confirmed running again; "
                "the bus-recovery watchdog is armed against an arm that may "
                "have gone quiet on a late DISABLE frame"
            )
        return False, (
            "feedback push NOT silenced (still pushing after a re-send); "
            "the shared bus stays flooded"
        )

    def _feedback_frame_ts(self):
        """Kernel timestamp of the last parsed feedback frame.

        Goes through the session owner: this polls at 100 Hz for the whole
        hand-window verify budget.

        A read that did not execute returns a fresh sentinel, not None. None
        legitimately means "no frame yet", so a lost session answering None
        would be indistinguishable from an unchanging timestamp and
        :meth:`_wait_for_feedback_silenced` would call the push quiet. A
        never-equal sentinel restarts the quiet window instead.
        """
        def read():
            js = self.agx_arm.get_joint_angles()
            return None if js is None else js.timestamp

        try:
            return self._sdk.call(
                "get_joint_angles", read, timeout=self.feedback_timeout,
                lane=Lane.DIAGNOSTIC,
            )
        except Exception:
            return object()

    def _wait_for_feedback_silenced(self, timeout_s: float) -> bool:
        """True once the firmware has stopped pushing feedback frames.

        Silence cannot be read off a status field — it is the *absence* of
        frames — so it is measured: the last frame timestamp must stay
        unchanged for ``hand_window_silence_quiet_s``. Any new frame restarts
        that quiet window.
        """
        quiet_s = self.hand_window_silence_quiet_s
        deadline = time.monotonic() + timeout_s
        last_ts = self._feedback_frame_ts()
        quiet_since = time.monotonic()
        while time.monotonic() < deadline:
            time.sleep(0.01)
            current_ts = self._feedback_frame_ts()
            if current_ts != last_ts:
                last_ts = current_ts
                quiet_since = time.monotonic()
                continue
            if time.monotonic() - quiet_since >= quiet_s:
                return True
        return False

    def _restore_feedback_push(self, reason: str, *, direct: bool = False) -> bool:
        """Close out a hand window's silence: re-enable the push, clear the flag.

        Bookkeeping paired with :meth:`_silence_feedback_push`, a no-op when this
        node silenced nothing. The wire operation is
        :meth:`_ensure_feedback_push_enabled`, which startup and operator
        recovery call directly with ``force``.
        """
        if not self._hand_window_push_silenced:
            return True
        ok = self._ensure_feedback_push_enabled(reason, force=True, direct=direct)
        # Un-stand-down the MIT controller: feedback is coming back.
        self._set_push_silenced(False)
        self.get_logger().info(f"feedback push restored ({reason})")
        return ok

    def _wait_for_feedback_resumed(self, timeout_s: float) -> bool:
        """Wait for the firmware to push a genuinely NEW feedback frame."""
        baseline = self._feedback_frame_ts()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            current = self._feedback_frame_ts()
            if current is not None and current != baseline:
                return True
            time.sleep(0.01)
        return False

    def _prepare_hand_window_callback(self, request, response):
        """Quiesce the arm into a VERIFIED normal-mode hold so the OmniHand can
        own the shared side bus (plan section 3).

        The hold is a plain MOVE-J to the current pose, i.e. it is executed and
        closed by the ARM's own position controller — the same controller that
        holds the arm after boot. The node's MIT loop is not the one holding:
        it is gated out at ``_check_can_control`` for the whole window. That
        split is what makes the window safe, because the window then silences
        the feedback push, and a host-side loop with no feedback could not
        compute a correction for any drift.

        Every mode change is confirmed by feedback readback (ctrl_mode, MOVE
        mode, joint velocities), never fire-and-forget: the SDK silently drops
        mode frames under bus saturation (section 1.3.2), which is exactly when
        a handoff runs. Returns success only when the arm is confirmed held by
        the firmware and quiet.
        """
        del request
        if not self.is_nero:
            response.success = False
            response.message = "prepare_hand_window is only supported for Nero"
            return response
        if self._recovery_in_progress or self._force_recovery:
            response.success = False
            response.message = "cannot open a hand window while recovering the bus"
            return response
        if not self.enable_flag or not self._check_arm_connected():
            response.success = False
            response.message = "arm not connected/enabled"
            return response
        hold_pose = self._capture_hold_pose()
        if hold_pose is None:
            response.success = False
            response.message = "no trustworthy feedback to capture a hold pose"
            self.get_logger().error(response.message)
            return response
        # Gate MIT forwarding first so no streamed arm command races the handoff.
        self._hand_window_active = True
        try:
            # Straight to the mode frame: it is what ends the MIT setpoint, and
            # the MOVE-J hold below is what holds the pose. A kp=0 damped zero
            # in between would only add a window with no stiffness in it.
            self._sdk_write("set_normal_mode", self.agx_arm.set_normal_mode)
            self.is_mit_mode = False
            self._leader_mode_active = False
            self._current_motion_mode = None
            # The hold must be executed by the arm's OWN position controller, so
            # the MOVE-J mode frame has to actually land. On the flooded one-shot
            # bus a single one is easily dropped (measured on hardware: the arm
            # stayed in MOVE_MIT after one move_j), so re-assert it until the
            # firmware confirms it left MIT.
            left_mit, move_mode, attempts = self._assert_firmware_hold(hold_pose)
            self._last_good_feedback_monotonic = time.monotonic()
        except Exception as e:
            self._hand_window_active = False
            response.success = False
            response.message = f"failed to command normal-mode hold: {e}"
            self.get_logger().error(response.message)
            return response
        # Verify by readback, not by assuming set_normal_mode took (it is a
        # no-op on V112): the arm must be settled in feedback, report an active
        # holding ctrl_mode (CAN_CTRL/TCP_CTRL), and be in a firmware-executed
        # (non-MIT) move mode, else we do not claim a safe hold and the hand
        # window is not opened.
        # A hand window may only open on a hold proven at rest: an unverifiable
        # settle is not good enough here either, so `.verified` is required
        # rather than the weaker "did not observe motion".
        settle = self._arm_velocities_settled()
        settled = settle.verified
        ctrl_mode = self._arm_ctrl_mode()
        held = self._ctrl_mode_is_hold(ctrl_mode)
        # Who holds the arm matters as much as whether it is held: with the
        # feedback push about to go silent, the host cannot compute any
        # correction, so the firmware's own position controller must own the
        # hold. A MIT move mode here would mean the arm is waiting for commands
        # that will not come. Re-read after the settle poll for the freshest
        # value; the gate has kept MIT frames off the bus throughout.
        move_mode = self._arm_move_mode()
        firmware_holds = self._move_mode_is_firmware_hold(move_mode)
        if settled and held and firmware_holds:
            # Only now — with the hold VERIFIED in feedback — silence the
            # feedback push, because verifying it needs that same feedback.
            silenced, silence_note = self._silence_feedback_push()
            response.success = True
            response.message = (
                f"hand window open: arm settled and held by the firmware "
                f"(ctrl_mode={ctrl_mode}, move_mode={move_mode}, "
                f"move_j x{attempts}), MIT quiesced, {silence_note}"
            )
            if silenced:
                self.get_logger().info(response.message)
            else:
                # Still a valid window (the arm is held and MIT is gated), but
                # the shared bus stays flooded, so hand commands may still time
                # out. Say so instead of implying the bus is free.
                self.get_logger().warn(response.message)
        else:
            self._hand_window_active = False
            response.success = False
            response.message = (
                f"hold NOT verified (settled={settled} [{settle.detail}], "
                f"holding={held}, firmware_holds={firmware_holds}, "
                f"ctrl_mode={ctrl_mode}, move_mode={move_mode}, "
                f"move_j x{attempts}); hand window not opened"
            )
            self.get_logger().error(response.message)
        return response

    def _resume_arm_control_callback(self, request, response):
        """Reopen the shared side bus for arm MIT control after a hand window.

        Verifies healthy arm feedback and no latched comm fault before
        re-admitting MIT commands. Clearing pending hand commands
        (control/omnihand/stop) is the caller's job — the driver owns the arm
        side only.

        The feedback push silenced at window-open is restored FIRST, before any
        health check, because every one of those checks reads that feedback.

        Resume behaviour, stated honestly: this service only reopens the gate. It
        does NOT force the (separate) MIT controller to recapture a hold — during
        the window its commands were dropped at the gate while it kept streaming
        the hold_reference it held at window-open. That reference equals the pose
        parked here by prepare_hand_window, so no far-ahead trajectory is
        replayed. If the arm sagged slightly under the normal-mode hold, the MIT
        loop applies a small, bounded position correction back to that reference
        when the gate reopens — intended and generally desirable, not a snap.
        """
        del request
        if not self.is_nero:
            response.success = False
            response.message = "resume_arm_control is only supported for Nero"
            return response
        if self._recovery_in_progress or self._force_recovery:
            response.success = False
            response.message = "cannot resume arm control while recovering the bus"
            return response
        # Restore the push before anything reads feedback, and wait for a real
        # new frame: the checks below (and the MIT controller that is about to
        # be re-admitted) must run on live data, not on the frozen last frame
        # from before the silence.
        push_restored = self._restore_feedback_push("resume_arm_control")
        feedback_back = self._wait_for_feedback_resumed(self.feedback_timeout)
        if not feedback_back:
            self.get_logger().warn(
                "no new feedback frame after restoring the push; "
                "falling through to the stale-feedback check"
            )
        if self._feedback_actually_stale():
            response.success = False
            response.message = (
                "arm feedback is stale; not resuming "
                f"(push_restored={push_restored})"
            )
            self.get_logger().error(response.message)
            return response
        try:
            comm_err = self._sdk_read(
                "has_comm_error",
                lambda: self.agx_arm.has_comm_error() and self.agx_arm.get_comm_error(),
            )
        except Exception:
            comm_err = None
        if comm_err and self._is_recoverable_can_error(comm_err):
            response.success = False
            response.message = f"comm fault present ({comm_err}); not resuming"
            self.get_logger().error(response.message)
            return response
        self._hand_window_active = False
        self._last_good_feedback_monotonic = time.monotonic()
        response.success = True
        response.message = "arm control resumed: MIT commands re-admitted"
        self.get_logger().info(response.message)
        return response

    def _set_leader_mode_callback(self, request, response):
        del request
        try:
            if not self.is_nero:
                response.success = False
                response.message = "set_leader_mode is only supported for Nero"
                return response
            if not self._check_arm_connected():
                response.success = False
                response.message = "Agx_arm is not connected"
                return response
            if not self.enable_flag:
                response.success = False
                response.message = "Agx_arm is not enabled"
                return response

            self._sdk_write("set_leader_mode", self.agx_arm.set_leader_mode)
            self.is_mit_mode = False
            self._leader_mode_active = True
            self._current_motion_mode = None
            # Leader mode silences the joint push on its own; from here the
            # leader-angle stream is the watchdog's health signal, so hand-window
            # silence bookkeeping no longer applies. The MIT controller has its
            # own leader-mode stand-down, so clear the hand-window one.
            self._set_push_silenced(False)
            # Normal joint push is now silenced; reset the watchdog window so the
            # gap until the first leader-angle sample is not read as a stall.
            self._last_good_feedback_monotonic = time.monotonic()
            response.success = True
            response.message = "Switched to leader mode"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to switch to leader mode: {e}"
            self.get_logger().error(response.message)
        return response


    def hold_current_pose(self, reason: str) -> tuple[bool, str]:
        """MOVE-J at the current pose, latching nothing. The ladder's second rung.

        The emergency stop's hold without its fault lockout, so an ordinary
        escalation — a controller that lost its feedback, this node exiting —
        does not cost the next bring-up a lockout to clear.

        Where no trustworthy pose exists it falls through to the mode frame,
        which needs none. There is no rung below that here: a kp=0 MIT command
        would end the setpoint without stiffness and sag the arm, so where the
        firmware answers nothing at all the external watchdog is the boundary.
        """
        if getattr(self, "_recovery_in_progress", False):
            return False, "recovery owns the SDK session; no hold was commanded"
        if not self.enable_flag:
            return False, "arm is not enabled; there is nothing to hold"
        # No separate health gate: a trustworthy pose is exactly the condition
        # for commanding a hold, and _capture_hold_pose establishes it. A second
        # check would only add a way to skip the hold.
        hold_pose = self._capture_hold_pose(lane=Lane.SAFETY)
        if hold_pose is None:
            left_mit = self._leave_mit_without_a_pose()
            detail = (
                f"{reason}: no trustworthy joint feedback, so no MOVE-J hold "
                "was commanded. Normal mode was "
                + ("requested" if left_mit else "NOT sent")
                + " so the firmware leaves MIT and holds its own pose, but "
                "nothing here can confirm it. Cut arm power if the arm moves."
            )
            self.get_logger().error(detail)
            return False, detail
        try:
            held = self._command_firmware_hold(hold_pose)
        except Exception as e:
            detail = f"{reason}: firmware hold failed: {e}"
            self.get_logger().error(detail)
            return False, detail
        if held:
            detail = f"{reason}: arm holding its pose in the firmware position controller"
            self.get_logger().info(detail)
            return True, detail
        detail = (
            f"{reason}: firmware did not confirm it left MIT — the arm may still "
            "be on the last streamed setpoint. Cut arm power if it moves."
        )
        self.get_logger().error(detail)
        return False, detail

    def _hold_current_pose_callback(self, request, response):
        del request
        response.success, response.message = self.hold_current_pose("hold_current_pose")
        return response

    def hold_on_shutdown(self) -> bool:
        """Park the arm in the firmware MOVE-J hold before this process goes away.

        The firmware keeps executing the last MIT setpoint it received, so
        leaving without a hold hands the arm to whatever was streaming when the
        stack went down.
        """
        held, _detail = self.hold_current_pose("shutdown")
        return held

    def shutdown(self) -> None:
        """Stop the SDK worker this node owns. Idempotent.

        The worker was never shut down: its thread is a daemon, so a process
        exit disposed of it and hid the omission. Anything that destroys the
        node without exiting — a test, a repeated bringup, a composed process —
        kept a thread holding this device's SDK session, which is precisely the
        one-owner invariant the worker exists to provide.
        """
        worker = getattr(self, "_sdk", None)
        if worker is not None:
            worker.shutdown()

    def destroy_node(self) -> bool:
        """Release the SDK session before the node goes away."""
        self.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = AgxArmRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Never leave the firmware with its feedback push silenced: the arm
        # would stay mute on CAN for the next session too.
        if node is not None:
            try:
                # Before the hold, because the hold is verified in feedback.
                # direct: the worker is stopped immediately after this.
                node._restore_feedback_push("node shutdown", direct=True)
            except Exception:
                pass
            try:
                # Last, while the worker is still up to carry it: the arm ends
                # in the firmware hold rather than on whatever setpoint the MIT
                # stream stopped at.
                node.hold_on_shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
