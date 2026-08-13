#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import errno
import rclpy
import math
import threading
import subprocess
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
from agx_arm_ctrl.runtime_metrics import MeasuredSdk, RuntimeMetrics
from agx_arm_msgs.msg import (
    AgxArmStatus, AgxDeviceAuthority, AgxDeviceCapability, AgxUnitSafety,
    GripperStatus,
    HandStatus, HandCmd, HandPositionTimeCmd,
    MoveMITMsg
)
from agx_arm_ctrl.effector import AgxGripperWrapper, Revo2Wrapper
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
        self.publisher_thread = threading.Thread(target=self._publish_thread)
        self.publisher_thread.start()

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
        # Phase 0 baseline instrumentation (C6): off unless asked for, so an
        # unmeasured deployment pays nothing.
        self.declare_parameter("runtime_metrics_enabled", False)
        self.declare_parameter("runtime_metrics_period_s", 10.0)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("effector_type", "none")
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("gripper_default_effort", 1.0)
        self.declare_parameter("publish_gripper_joint", True)
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
        self.enable_timeout = self.get_parameter("enable_timeout").value
        self.effector_type = self.get_parameter("effector_type").value
        self.tcp_offset = self.get_parameter("tcp_offset").value
        self.gripper_default_effort = self.get_parameter("gripper_default_effort").value
        self.publish_gripper_joint = self.get_parameter("publish_gripper_joint").value
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
        # Step-and-settle hand window: while active the arm is parked in a
        # driver-level normal-mode hold and incoming MIT commands are dropped at
        # this gateway, so the OmniHand owns the shared side CAN bus (plan §3).
        self._hand_window_active = False
        # True while the firmware's feedback push is silenced for a hand window.
        # The arm stays in its CAN-control hold — only the Nero->host feedback
        # stream is off, so the watchdog must not read that silence as a stall.
        self._hand_window_push_silenced = False
        self._hand_window_silence_started = 0.0
        self.enable_flag = False
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
        self._loop_overrun_threshold_s = max(2.0 / self.pub_rate, 0.2)
        self._last_loop_monotonic = 0.0
        self._last_loop_gap_s = 0.0
        self._max_loop_gap_s = 0.0
        self._loop_overrun_count = 0
        # recoveries suppressed because the kernel RX timestamp proved the bus
        # was still live while a starvation-sensitive signal read stale/not-ok.
        self._loop_overrun_suppressions = 0
        self._last_overrun_log_monotonic = 0.0
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

    def _recover_silent_arm(self) -> None:
        """Re-enable the firmware feedback push on an arm that answers nothing.

        The arm persists its linkage configuration across power cycles, and both
        ``set_leader_mode`` and ``set_follower_mode`` leave ``enable_can_push``
        DISABLED. An arm last used in one of those modes therefore boots mute:
        it acknowledges frames on the bus but pushes no feedback, so the startup
        firmware query times out and the node used to exit(1) — taking its own
        ``set_normal_mode`` service down with it, so nothing could bring the arm
        back through ROS (observed on hardware 2026-07-24, left arm).

        ``set_normal_mode`` re-asserts the normal linkage AND the push, which is
        exactly what such an arm needs. It commands no motion.
        """
        if not self.is_nero:
            return
        self.get_logger().warn(
            "No firmware answer — the arm may be booted with its CAN feedback "
            "push disabled (persisted leader/follower config). Sending "
            "set_normal_mode once and retrying."
        )
        try:
            self.agx_arm.set_normal_mode()
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
        self.mit_limits = mit_limits_for_tier(NeroFW.DEFAULT)
        config: PiperCanDefaultConfig = create_agx_arm_config(
            robot=self.arm_type, comm="can", channel=self.can_port
        )
        self.agx_arm = self._measured(AgxArmFactory.create_arm(config))
        self.agx_arm.connect()

        self.arm_joint_names = list(config["joint_limits"].keys())
        # Kept, not just the names: the boundary check needs the bounds to say
        # when a commanded position is outside the joint's configured range.
        self.arm_joint_limits = dict(config["joint_limits"])
        self.arm_joint_count = self.agx_arm.joint_nums

        if self.auto_enable:
            if not self._enable_arm(True, self.enable_timeout):
                self.get_logger().error("Failed to auto-enable the arm")

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
                    self.agx_arm.disconnect()
                    config = create_agx_arm_config(
                        robot=self.arm_type, comm="can", channel=self.can_port,
                        firmeware_version=firmeware_version
                    )
                    self.agx_arm = self._measured(AgxArmFactory.create_arm(config))
                    self.agx_arm.connect()
            else:
                self.get_logger().error(
                    "Failed to get firmware version, also after re-asserting the "
                    "feedback push. The arm is not answering on CAN: check power, "
                    "E-stop and wiring for this side, and that the bus carries "
                    "feedback frames (candump)."
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

        if self.effector_type == "agx_gripper":
            self.gripper = AgxGripperWrapper(self.agx_arm)
            if self.gripper.initialize():
                self.get_logger().info("AgxGripper initialized successfully")
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
        if self.hand is not None:
            self.hand_status_pub = self.create_publisher(
                HandStatus, "feedback/hand_status", 1
            )

    def _setup_subscribers(self):
        self.create_subscription(
            JointState, "control/joint_states", self._joint_states_callback, 1
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
        self.create_service(
            Trigger, "clear_fault_lockout", self._clear_fault_lockout_callback
        )
        self.create_service(ClaimDevice, "claim_device", self._claim_device_callback)
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

    def _check_arm_ready(self) -> bool:
        joint_states = self.agx_arm.get_joint_angles()
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

    def _leader_feedback_fresh(self) -> bool:
        """True when the leader-angle stream is actively reporting.

        In leader/drag mode this stream — not ``get_joint_angles()`` — is the
        live feedback the firmware pushes, so the bus-recovery watchdog uses it
        as the health signal while normal joint push is silenced.
        """
        if not self.is_nero:
            return False
        leader_joint_angles = self.agx_arm.get_leader_joint_angles()
        return leader_joint_angles is not None and leader_joint_angles.hz > 0

    def _check_arm_connected(self) -> bool:
        return self.agx_arm is not None and self.agx_arm.is_ok()

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
        if not self._check_arm_ready():
            self.get_logger().warn("Agx_arm is not connected, cannot control")
            return False
        if not self.enable_flag:
            self.get_logger().warn("Agx_arm is not enabled, cannot control")
            return False
        if not self.is_switch_seamlessly:
            arm_status = self.agx_arm.get_arm_status()
            if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                self.get_logger().warn("Agx_arm is in teach mode, cannot control")
                return False
        return True

    def _unit_safety_callback(self, msg: AgxUnitSafety) -> None:
        """Adopt a generation from the one writer that may allocate them."""
        adopted = self._unit_safety.observe(
            UnitSafetySnapshot(
                epoch=int(msg.epoch),
                stopped=bool(msg.stopped),
                reason=msg.reason,
                writer_id=msg.writer_id,
            )
        )
        if adopted:
            self.get_logger().warn(
                f"unit safety generation {msg.epoch} from '{msg.writer_id}': "
                f"stopped={msg.stopped} ({msg.reason})"
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
        them through the arm's parameter would be the wrong boundary.
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
            status = self.agx_arm.get_arm_status()
            if status is not None and status.msg.motion_status == self.agx_arm.ARM_STATUS.MotionStatus.REACH_TARGET_POS_SUCCESSFULLY:
                return True
            
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f"Timeout waiting for arm to motion done after {timeout} seconds"
                )
                return False
            time.sleep(poll_interval)

    def _enable_arm(self, enable: bool = True, timeout: float = 5.0) -> bool:
        """Command enable/disable and report only what the readback confirmed.

        The command returning is not evidence that the joints changed state; the
        per-joint readback is. This used to warn about a contradicting readback
        and then ``return True`` anyway, leaving ``self.enable_flag`` at its
        previous value — so a failed *disable* left the rest of the node
        believing the arm was still commandable. ``enable_flag`` now always
        carries what the readback said, and the return value says whether that
        matched the request.
        """
        start_time = time.time()
        deadline = start_time + timeout
        action_name = "enable" if enable else "disable"

        while not (self.agx_arm.enable() if enable else self.agx_arm.disable()):
            if time.time() > deadline:
                self.get_logger().error(
                    f"Timeout waiting for arm to {action_name} after {timeout} seconds"
                )
                return False
            time.sleep(0.01)

        # The readback is served from the last low-speed feedback frame, which
        # may still predate the command, so give it the remaining budget to
        # agree before calling it a contradiction.
        while True:
            joints_enabled = bool(self.agx_arm.get_joint_enable_status(255))
            if joints_enabled == enable or time.time() >= deadline:
                break
            time.sleep(0.02)

        self.enable_flag = joints_enabled
        if joints_enabled == enable:
            self.get_logger().info(f"All joints {action_name} status is {self.enable_flag}")
            return True

        self.get_logger().error(
            f"{action_name} was accepted by the arm but the joint readback still "
            f"reports enabled={joints_enabled} after {timeout:.1f}s. Treating the "
            f"arm as NOT {action_name}d."
        )
        return False

    ### publisher thread
    def _publish_thread(self):
        rate = self.create_rate(self.pub_rate)

        # publishing loop
        while rclpy.ok():
            # P0 instrumentation: measure how late this iteration is before the
            # recovery check runs, so a stale-feedback trigger can be attributed
            # to local scheduling starvation vs a real dead bus.
            now = time.monotonic()
            if self._last_loop_monotonic:
                self._last_loop_gap_s = now - self._last_loop_monotonic
                if self._last_loop_gap_s > self._loop_overrun_threshold_s:
                    self._loop_overrun_count += 1
                    self._max_loop_gap_s = max(
                        self._max_loop_gap_s, self._last_loop_gap_s
                    )
                    if now - self._last_overrun_log_monotonic > 5.0:
                        self._last_overrun_log_monotonic = now
                        self.get_logger().warn(
                            "publish-loop overrun: "
                            f"{self._last_loop_gap_s * 1000:.0f} ms gap "
                            f"(> {self._loop_overrun_threshold_s * 1000:.0f} ms; "
                            f"count={self._loop_overrun_count}, "
                            f"peak={self._max_loop_gap_s * 1000:.0f} ms). Feedback "
                            "'staleness' this cycle may be local starvation, not a dead bus."
                        )
            self._last_loop_monotonic = now
            # P1: detect a stalled bus (TX ENOBUFS slot leak or stale feedback)
            # and re-establish the link instead of dead-locking until restart.
            # Never let recovery bookkeeping crash the publish loop.
            try:
                if self._should_recover_bus():
                    self._request_recovery()
            except Exception as e:
                self.get_logger().error(f"bus recovery check failed: {e}")

            if self._recovery_in_progress:
                # Recovery owns the SDK session exclusively while it runs, so
                # this loop must not touch it — but it must keep running.
                # Recovering inline used to block this thread for the whole
                # attempt: 13.1 s measured on hardware, during which nothing
                # published state and nothing drained the CAN RX socket.
                self._publish_authority(self._authority.snapshot())
                self._publish_fault_lockout()
                rate.sleep()
                continue

            self._surface_silent_tx_loss()
            self._sync_authority("publish loop")

            if self.agx_arm.is_ok():
                if self._check_arm_ready():
                    self._last_good_feedback_monotonic = time.monotonic()
                    if not self.control_ready:
                        self.control_ready = True
                        self._had_control_ready = True
                        if not self._control_ready_logged:
                            self.get_logger().info("Agx_arm feedback is ready, control is now enabled")
                            self._control_ready_logged = True
                elif self._leader_mode_active and self._leader_feedback_fresh():
                    # In leader/drag mode the firmware disables the normal
                    # joint-state push (enable_can_push=DISABLE); the live signal
                    # is the leader-angle stream. Treat it as a healthy bus so the
                    # recovery watchdog does not false-trigger on that silence.
                    self._last_good_feedback_monotonic = time.monotonic()
                # One timer around the whole publish batch: hot path 1 in
                # reference/critical_cpu_paths.md is this loop's per-joint SDK
                # reads, and the refactor has to show a before/after for it.
                with self.metrics.time_block("publish_batch"):
                    self._publish_joint_states()
                    self._publish_pose()
                    self._publish_arm_status()
                    self._publish_effector_status()
                    self._publish_leader_joint_angles()
            if self.metrics.due():
                report = self.metrics.report()
                if report:
                    self.get_logger().info(report)
            rate.sleep()

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

    def _feedback_actually_stale(self) -> bool:
        """Kernel-RX-timestamp confirmation that the bus is genuinely silent.

        The FPS window (`is_ok()`/`hz`) and the node-observed feedback clock both
        go stale under local CPU starvation without the bus going down. The
        kernel receive timestamp of the last parsed frame is the ground truth:
        frames queue in the socket buffer and carry their true arrival times when
        the node resumes, so a still-advancing timestamp means the bus is live.
        """
        try:
            joint_states = self.agx_arm.get_joint_angles()
        except Exception:
            return True
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

    def _surface_silent_tx_loss(self) -> None:
        """Log commands the SDK dropped silently while feedback looked healthy.

        On the shared bus this is usually the hand losing arbitration, not a dead
        arm, so it is surfaced (not turned into a heavyweight recovery): a rising
        forked send-error count while RX keeps flowing is the only evidence that
        arm commands are being dropped (plan section 1.3.2).
        """
        get_count = getattr(self.agx_arm, "get_send_error_count", None)
        if get_count is None:
            return
        try:
            count = int(get_count())
        except Exception:
            return
        if count <= self._last_send_error_count:
            self._last_send_error_count = count
            return
        dropped = count - self._last_send_error_count
        self._last_send_error_count = count
        now = time.monotonic()
        if now - self._last_tx_loss_log <= 5.0:
            return
        self._last_tx_loss_log = now
        try:
            last = self.agx_arm.get_last_send_error()
        except Exception:
            last = None
        self.get_logger().warn(
            f"silent TX loss: {dropped} send(s) dropped (total {count}) while feedback "
            f"is live (last: {last}); arm commands may not be reaching the firmware. On "
            "the shared bus this is usually hand-frame arbitration loss, not a dead arm."
        )

    def _should_recover_bus(self) -> bool:
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
            comm_err = self.agx_arm.has_comm_error() and self.agx_arm.get_comm_error()
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
            if not self.agx_arm.is_ok():
                # is_ok() is FPS-based (SDK monitor thread) and false-triggers
                # under whole-process CPU/GIL saturation. Only recover when the
                # kernel RX timestamp confirms the bus is actually silent.
                if self._feedback_actually_stale():
                    return self._trigger_recovery("not_ok", "driver reports not ok")
                return self._suppress_recovery_as_starvation("is_ok() reads false")
        except Exception:
            return self._trigger_recovery("is_ok_raised", "is_ok() raised")
        if (time.monotonic() - self._last_good_feedback_monotonic) > self.feedback_timeout:
            # The node-observed feedback clock is stale, but a publish-loop stall
            # ages it without the bus going down. Confirm with the kernel RX
            # timestamp before the heavyweight reconnect.
            if self._feedback_actually_stale():
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
        # Latched before the thread starts, so there is no window in which the
        # device still looks commandable while its session is being torn down.
        self._authority.enter_recovering(self._recover_reason or "bus recovery")
        self.control_ready = False
        self._control_ready_logged = False
        thread = threading.Thread(
            target=self._recovery_thread, name=f"recovery-{self.device_id}", daemon=True
        )
        thread.start()

    def _recovery_thread(self) -> None:
        """Run one recovery attempt off the acquisition path."""
        try:
            self._recover_bus()
        except Exception as exc:
            self.get_logger().error(f"recovery thread failed: {exc}")
        finally:
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
        self._restore_feedback_push("bus recovery")
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
        # Before tearing the link down, override any moving last MIT command in
        # the firmware with a damped zero: during recovery nothing streams, and
        # the firmware otherwise keeps executing the last command it received
        # (runaway observed live during a teach recording).
        if self.is_mit_mode or self._current_motion_mode == 'mit':
            try:
                self._send_damped_stop_mit()
            except Exception:
                pass
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
            for attempt in range(1, self.bus_recovery_max_attempts + 1):
                try:
                    self.agx_arm.disconnect()
                except Exception as e:
                    self.get_logger().warn(f"disconnect during recovery failed: {e}")

                if self.bus_recovery_link_reset:
                    self._reset_can_link()

                try:
                    self.agx_arm.connect()
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
                    if self.auto_enable:
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
                self.get_logger().warn(
                    f"CAN bus recovery attempt {attempt} did not restore feedback"
                )

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
            # Hold an explicit fault lockout after recovery: the publish loop
            # would otherwise re-arm control_ready on the next healthy tick and
            # silently accept motion. Requires clear_fault_lockout to release.
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

    def _publish_joint_states(self):
        self.metrics.record_sdk_call("get_joint_angles")
        joint_states = self.agx_arm.get_joint_angles()
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
                self._publish_leader_states_as_joint_states()
            return

        velocitys = []
        efforts = []
        # One blocking SDK round trip per joint, every cycle: hot path 1. The
        # counter records the call and the thread, because "all SDK access from
        # one worker" is the Phase 1 exit criterion and this is where the
        # current answer is measured.
        with self.metrics.time_block("motor_state_reads"):
            for joint_index in range(1, self.arm_joint_count+1):
                self.metrics.record_sdk_call("get_motor_states")
                ms = self.agx_arm.get_motor_states(joint_index)
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

    def _publish_leader_states_as_joint_states(self):
        leader_joint_angles = self.agx_arm.get_leader_joint_angles()
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

    def _publish_pose(self):
        flange_pose = self.agx_arm.get_flange_pose()
        if flange_pose is None or flange_pose.hz <= 0:
            return
        
        tcp_pose = self.agx_arm.get_flange2tcp_pose(flange_pose.msg)

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

    def _publish_arm_status(self):
        arm_status = self.agx_arm.get_arm_status()
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

    def _publish_leader_joint_angles(self):
        leader_joint_angles = self.agx_arm.get_leader_joint_angles()
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

        try:
            self.gripper.move(width=width, force=force)
        except ValueError as e:
            self.get_logger().warn(str(e))

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
        # Effector control stays: the gripper and hand are separate devices with
        # their own contract, and the arm's quarantine is the wrong boundary.
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
            self.agx_arm.move_j(joints)
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

        try:
            # Send ArmMsgModeCtrl once per mode transition, not once per joint.
            # Without this, 7 redundant mode-ctrl frames are sent per callback at
            # 100 Hz, which saturates the CAN TX queue (~700 extra frames/sec/arm).
            if self._current_motion_mode != 'mit':
                self.agx_arm.set_motion_mode('mit')
                self._current_motion_mode = 'mit'
                self.is_mit_mode = True

            self.agx_arm.set_auto_set_motion_mode_enabled(False)
            try:
                for i in range(len(msg.joint_index)):
                    self.agx_arm.move_mit(
                        joint_index=msg.joint_index[i],
                        p_des=msg.p_des[i],
                        v_des=msg.v_des[i],
                        kp=msg.kp[i],
                        kd=msg.kd[i],
                        t_ff=msg.torque[i],
                    )
            finally:
                self.agx_arm.set_auto_set_motion_mode_enabled(True)
        except Exception as e:
            self._handle_send_failure("_move_mit_callback", e)

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
        try:
            if request.data and self.enable_flag:
                response.success = True
                response.message = "Agx_arm already enabled"
                return response
            if not request.data and not self.enable_flag:
                response.success = True
                response.message = "Agx_arm already disabled"
                return response

            if not self._check_arm_connected():
                response.success = False
                response.message = "Agx_arm is not connected"
                self.get_logger().warn("Agx_arm is not connected, cannot set enable state")
            elif request.data:
                response.success = True if self._enable_arm(True) else False
                response.message = "Agx_arm enabled" if response.success else "Failed to enable Agx_arm"
            else:
                response.success = True if self._enable_arm(False) else False
                response.message = "Agx_arm disabled" if response.success else "Failed to disable Agx_arm"
            
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
                    arm_status = self.agx_arm.get_arm_status()
                    if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                        self.get_logger().warn("Agx_arm is in teach mode, cannot move to home position")
                        return response
                    
                if self.is_mit_mode:
                    self.agx_arm.move_js([0] * self.arm_joint_count)
                else:
                    self.agx_arm.move_j([0] * self.arm_joint_count)
                if self._wait_motion_done():
                    self.get_logger().info("Agx_arm moved to home position successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to move to home position: {str(e)}")
        return response

    def _send_damped_stop_mit(self, kd: float = 1.0) -> None:
        """Zero-velocity, kp=0, kd-damped MIT command for every joint.

        Needs NO feedback, so it works exactly when the readiness checks fail —
        the situation in which the firmware would otherwise keep executing the
        last (possibly moving) MIT command it received.
        """
        self.agx_arm.set_auto_set_motion_mode_enabled(False)
        try:
            for joint_index in range(1, self.arm_joint_count + 1):
                self.agx_arm.move_mit(
                    joint_index=joint_index,
                    p_des=0.0,
                    v_des=0.0,
                    kp=0.0,
                    kd=kd,
                    t_ff=0.0,
                )
        finally:
            self.agx_arm.set_auto_set_motion_mode_enabled(True)

    # An emergency stop is only trustworthy if the arm is confirmed stopped in
    # feedback: under ENOBUFS the SDK silently drops the stop command and still
    # returns success (plan section 1.3.2), so the command alone proves nothing.
    ESTOP_VELOCITY_THRESHOLD_RAD_S = 0.05
    ESTOP_VERIFY_TIMEOUT_S = 0.5
    # Two feedback frames must be at least this far apart before a finite
    # difference means anything; below it encoder quantisation dominates the
    # estimate and a moving arm can read as settled.
    VELOCITY_MIN_SAMPLE_DT_S = 0.01

    def _sample_joint_positions(self):
        """One timestamped joint-position sample, or None when unavailable."""
        try:
            js = self.agx_arm.get_joint_angles()
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
        received. Order: damped MIT zero (needs no feedback) -> position hold at
        the current pose when feedback is trustworthy -> electronic emergency
        stop when it is not. Each stage is then VERIFIED in feedback (joint
        velocities settle); if not verified it escalates to a hard electronic
        stop and finally requests a bus-recovery link reset, and it never logs a
        plain success when the arm is not confirmed stopped.

        Returns a Trigger result so a supervisor can act on the outcome:
        ``success`` is True only when the arm is CONFIRMED stopped in feedback;
        ``message`` states the verified/unverified result and, when the last
        resort forced a bus recovery, that a fault lockout will latch and the
        caller must call ``clear_fault_lockout`` before re-arming motion.
        """
        stopped = False
        recovery_requested = False
        # This device is stopped unilaterally and immediately: a device-level
        # fault on its own epoch, needing no other process. Whatever was issued
        # before this point is stale for this device from here on.
        self._estop_latched = True
        self._authority.enter_faulted("emergency stop requested")
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
            # hardware stop cannot be confirmed right now. The arm is already
            # covered by the damped zero recovery sends before the teardown and
            # by the fault lockout after it; that is a mitigation, and the
            # independent watchdog is the boundary for this regime.
            response.success = False
            response.message = (
                f"{self.arm_type} stop=unverifiable — the device is RECOVERING "
                f"({self.recovery_active_s:.1f}s so far) and its SDK session "
                "belongs to recovery, so no stop was attempted over it. This "
                "device is latched and refuses motion, and a unit stop was "
                "requested. A NEW hardware stop cannot be confirmed until "
                "recovery ends — use the physical emergency stop if the arm is "
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
            if self.is_mit_mode or self._current_motion_mode == 'mit':
                try:
                    self._send_damped_stop_mit()
                except Exception as e:
                    self.get_logger().error(f"Damped MIT stop failed: {e}")

            js = None
            try:
                js = self.agx_arm.get_joint_angles()
            except Exception:
                js = None

            if js is not None and js.hz > 0:
                q = list(js.msg)
                if not self.is_switch_seamlessly:
                    self.agx_arm.move_js(q)
                    self.is_mit_mode = True
                    self._current_motion_mode = 'js'
                else:
                    self.agx_arm.move_j(q)
                    self.is_mit_mode = False
                    self._current_motion_mode = 'j'
                self.get_logger().info(f"Emergency stop command sent to {self.arm_type}")
            else:
                # No trustworthy pose to hold: hard stop is the only safe option.
                self.agx_arm.electronic_emergency_stop()
                self.get_logger().warn(
                    "Emergency stop without valid feedback: sent electronic emergency stop"
                )

            # Verify the stop actually took effect in feedback.
            verification = self._arm_velocities_settled()
            stopped = verification.verified
            if stopped:
                self.get_logger().info(
                    f"Emergency stop verified: {self.arm_type} joints settled "
                    f"({verification.detail})"
                )
            else:
                self.get_logger().error(
                    f"Emergency stop NOT verified ({verification.detail}) — "
                    "escalating to electronic emergency stop"
                )
                try:
                    self.agx_arm.electronic_emergency_stop()
                except Exception as e:
                    self.get_logger().error(f"electronic_emergency_stop failed: {e}")
                verification = self._arm_velocities_settled()
                stopped = verification.verified
                if not stopped:
                    # Last resort: hand the heavyweight link-reset recovery to the
                    # publish thread (owns the connection). This also flushes a
                    # stuck moving MIT setpoint the firmware would keep executing.
                    self.get_logger().error(
                        "Emergency stop STILL not verified after electronic stop — "
                        "requesting bus-recovery link reset. Firmware has no MIT "
                        "command watchdog: use the physical e-stop."
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
            if not verification.evidence:
                self.get_logger().error(
                    f"EMERGENCY STOP COMMANDED BUT UNVERIFIABLE for {self.arm_type} "
                    f"({verification.detail}) — no usable velocity evidence; treat "
                    "the arm as still moving and use the physical e-stop"
                )
            else:
                self.get_logger().error(
                    f"EMERGENCY STOP UNVERIFIED for {self.arm_type} "
                    f"({verification.detail}) — do not trust the software stop; "
                    "use the physical e-stop"
                )
            state = "commanded_unverifiable" if not verification.evidence else "unverified"
            if recovery_requested:
                # The publish thread will run _recover_bus and latch a fault
                # lockout; the caller (supervisor/operator) owns clearing it.
                response.message = (
                    f"{self.arm_type} stop={state} ({verification.detail}) — forced "
                    "bus recovery requested; fault_lockout=latched, call "
                    "clear_fault_lockout before re-arming. Use the physical e-stop "
                    "if it still moves."
                )
            else:
                response.message = (
                    f"{self.arm_type} stop={state} ({verification.detail}) — "
                    "use the physical e-stop"
                )
        return response

    def _exit_teach_mode_callback(self, request, response):
        try:
            arm_status = self.agx_arm.get_arm_status()
            if not self.is_piper:
                self.get_logger().warn("exit teach mode just piper series supported")
                return response

            if arm_status is not None and arm_status.msg.ctrl_mode == self.agx_arm.ARM_STATUS.CtrlMode.TEACHING_MODE:
                self.agx_arm.move_js([0] * self.arm_joint_count)
                time.sleep(2)
                self.agx_arm.electronic_emergency_stop()
                self.agx_arm.move_j([0] * self.arm_joint_count)
                time.sleep(0.3)
                self.agx_arm.reset()
                time.sleep(0.5)
                self._enable_arm(True)
                self.agx_arm.move_j([0] * self.arm_joint_count)
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
            if not self._check_arm_connected():
                response.success = False
                response.message = "Agx_arm is not connected"
                return response
            if not self.enable_flag:
                response.success = False
                response.message = "Agx_arm is not enabled"
                return response

            self.agx_arm.set_normal_mode()
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
            response.success = True
            response.message = "Switched to normal mode"
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
        """False only when the readback positively reports a MIT move mode.

        In MIT the arm only does what the host streams — with the feedback push
        silenced the host streams nothing and no correction can be computed. A
        hand window therefore requires a NON-MIT move mode, where the vendor's
        own position controller closes the loop on the firmware side. An
        unreadable or UNKNOWN mode is not treated as a failure; the observed
        value is reported so a surprising one is visible instead of trusted.
        """
        return not self._move_mode_is_mit(mode_feedback)

    def _assert_firmware_hold(self, hold_pose) -> tuple:
        """Re-assert a MOVE-J hold until the firmware confirms it left MIT.

        A single MOVE-J mode frame can be dropped on the flooded one-shot shared
        bus (the push is still on here — it is only silenced once the hold is
        verified), leaving the firmware in MIT while it executes the preceding
        kp=0 damped stop, which sags. Re-send the same-pose, motionless MOVE-J
        until the readback stops reporting a MIT move mode, bounded by
        ``hand_window_hold_assert_s``.

        Returns ``(left_mit, move_mode, attempts)``: whether the firmware is
        confirmed out of MIT, the last move-mode read, and how many sends it
        took (useful evidence of how lossy the bus was during the handoff).
        """
        self.agx_arm.set_auto_set_motion_mode_enabled(True)
        deadline = time.monotonic() + self.hand_window_hold_assert_s
        attempts = 0
        while True:
            self.agx_arm.move_j(hold_pose)
            self._current_motion_mode = 'j'
            attempts += 1
            time.sleep(self.hand_window_hold_poll_s)
            move_mode = self._arm_move_mode()
            if not self._move_mode_is_mit(move_mode):
                return True, move_mode, attempts
            if time.monotonic() >= deadline:
                return False, move_mode, attempts

    def _arm_ctrl_mode(self):
        """Current firmware ctrl_mode from feedback, or None if unreadable."""
        status = self._arm_status_msg()
        return None if status is None else status.ctrl_mode

    def _arm_move_mode(self):
        """Current firmware mode_feedback (MOVE P/J/L/C/MIT/CPV), or None."""
        status = self._arm_status_msg()
        return None if status is None else getattr(status, "mode_feedback", None)

    def _arm_status_msg(self):
        try:
            status = self.agx_arm.get_arm_status()
        except Exception:
            return None
        if status is None:
            return None
        return status.msg

    def _capture_hold_pose(self):
        """Current joint pose from trustworthy live feedback, or None."""
        try:
            js = self.agx_arm.get_joint_angles()
        except Exception:
            return None
        alive = js is not None and (
            js.hz > 0 or self._feedback_frame_advancing(js.timestamp)
        )
        if not alive:
            return None
        return list(js.msg)

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
            nero_can_push.set_can_push(self.agx_arm, False)
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
            nero_can_push.set_can_push(self.agx_arm, False)
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
        """Kernel timestamp of the last parsed feedback frame, or None."""
        try:
            js = self.agx_arm.get_joint_angles()
        except Exception:
            return None
        return None if js is None else js.timestamp

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

    def _restore_feedback_push(self, reason: str) -> bool:
        """Re-enable the firmware feedback push and re-arm the watchdog.

        Safe to call unconditionally; a no-op when nothing was silenced.
        """
        if not self._hand_window_push_silenced:
            return True
        ok = True
        try:
            nero_can_push.set_can_push(self.agx_arm, True)
        except Exception as e:
            ok = False
            self.get_logger().error(
                f"failed to restore the feedback push ({reason}): {e}"
            )
        # Un-stand-down the MIT controller: feedback is coming back.
        self._set_push_silenced(False)
        # Feedback restarts now: reset both the node-observed clock and the
        # frame-advance window so the silence we asked for is never charged to
        # the bus-recovery watchdog as a stall.
        now = time.monotonic()
        self._last_good_feedback_monotonic = now
        self._last_feedback_advance_monotonic = now
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
            if self.is_mit_mode or self._current_motion_mode == 'mit':
                self._send_damped_stop_mit()
            self.agx_arm.set_normal_mode()
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
            comm_err = self.agx_arm.has_comm_error() and self.agx_arm.get_comm_error()
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

            self.agx_arm.set_leader_mode()
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
                node._restore_feedback_push("node shutdown")
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
