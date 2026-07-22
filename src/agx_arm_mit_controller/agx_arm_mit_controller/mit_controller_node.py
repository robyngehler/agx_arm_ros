from __future__ import annotations

import math
from enum import Enum
from pathlib import Path
import threading
import time
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_msgs.msg import AgxArmStatus, MoveMITMsg

from .feedforward_model import CalibrationModel, load_calibration_model
from .gravity_model import GravityModel, GravityModelError, create_gravity_model
from .model_metadata import default_nero_calibration_path
from .trajectory_buffer import JointTrajectoryBuffer, SampledTrajectoryPoint, duration_to_seconds


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def scale_gravity_feedforward(
    gravity_torque: list[float],
    gravity_scale: float,
    gravity_feedforward_sign: float,
) -> list[float]:
    return [gravity_feedforward_sign * gravity_scale * value for value in gravity_torque]


CTRL_MODE_LINKAGE_TEACHING_INPUT_MODE = 0x06


class ExecutionState(str, Enum):
    DISABLED = "DISABLED"
    IDLE_HOLD = "IDLE_HOLD"
    ARMING = "ARMING"
    EXECUTING_TRAJECTORY = "EXECUTING_TRAJECTORY"
    CANCELING_TO_HOLD = "CANCELING_TO_HOLD"
    HOLDING_FINAL_POINT = "HOLDING_FINAL_POINT"
    LEADER_MODE = "LEADER_MODE"
    FREEDRIVE = "FREEDRIVE"
    STALE_FEEDBACK = "STALE_FEEDBACK"
    FAULTED = "FAULTED"


class NeroMitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mit_controller")

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        )
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("feedback_timeout_s", 0.25)
        self.declare_parameter("auto_enable_on_trajectory", True)
        self.declare_parameter("hold_final_point", True)
        self.declare_parameter("gain_ramp_time_s", 1.0)
        self.declare_parameter("position_error_limit", [0.5] * 7)
        self.declare_parameter("velocity_limit", [2.0] * 7)
        self.declare_parameter("torque_limit", [8.0] * 7)
        self.declare_parameter("kp", [2.0, 3.0, 2.0, 3.0, 2.0, 2.0, 2.0])
        self.declare_parameter("kd", [0.1, 0.1, 0.1, 0.1, 0.12, 0.12, 0.08])
        # Freedrive (zero-force drag) damping: kp is forced to 0 so gravity
        # feedforward makes the arm back-drivable; kd only damps the motion.
        self.declare_parameter("freedrive_kd", [0.1, 0.1, 0.1, 0.1, 0.12, 0.12, 0.08])
        self.declare_parameter("gravity_compensation_enabled", False)
        self.declare_parameter("gravity_backend", "pinocchio")
        self.declare_parameter("gravity_urdf_path", "")
        self.declare_parameter("gravity_scale", 1.0)
        self.declare_parameter("gravity_feedforward_sign", -1.0)
        # Arm base orientation in world (XYZ extrinsic rpy). Rotates the gravity
        # model so compensation is correct for a tilted body mount; [0,0,0] keeps
        # the upright table-mount default.
        self.declare_parameter("gravity_mounting_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("action_name", "arm_controller/follow_joint_trajectory")
        self.declare_parameter("action_feedback_rate_hz", 20.0)
        self.declare_parameter("start_state_tolerance", [0.10] * 7)
        self.declare_parameter("goal_position_tolerance", [0.05] * 7)
        self.declare_parameter("goal_velocity_tolerance", [0.20] * 7)
        self.declare_parameter("goal_time_tolerance_s", 0.5)
        self.declare_parameter("allow_joint_reordering", False)
        self.declare_parameter("input_joint_prefix", "")
        self.declare_parameter("reject_new_goal_while_executing", True)
        self.declare_parameter("enable_debug_joint_trajectory_topic", False)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.auto_enable_on_trajectory = bool(self.get_parameter("auto_enable_on_trajectory").value)
        self.hold_final_point = bool(self.get_parameter("hold_final_point").value)
        self.gain_ramp_time_s = max(0.0, float(self.get_parameter("gain_ramp_time_s").value))
        self.position_error_limit = self._load_float_array("position_error_limit")
        self.velocity_limit = self._load_float_array("velocity_limit")
        self.torque_limit = self._load_float_array("torque_limit")
        self.kp = self._load_float_array("kp")
        self.kd = self._load_float_array("kd")
        self.freedrive_kd = self._load_float_array("freedrive_kd")
        self.gravity_compensation_enabled = bool(self.get_parameter("gravity_compensation_enabled").value)
        self.gravity_backend = str(self.get_parameter("gravity_backend").value)
        self.gravity_urdf_path = str(self.get_parameter("gravity_urdf_path").value)
        self.gravity_scale = float(self.get_parameter("gravity_scale").value)
        self.gravity_feedforward_sign = float(self.get_parameter("gravity_feedforward_sign").value)
        self.gravity_mounting_rpy = [float(v) for v in self.get_parameter("gravity_mounting_rpy").value]
        self.calibration_file = str(self.get_parameter("calibration_file").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.action_feedback_rate_hz = float(self.get_parameter("action_feedback_rate_hz").value)
        self.start_state_tolerance = self._load_float_array("start_state_tolerance")
        self.goal_position_tolerance = self._load_float_array("goal_position_tolerance")
        self.goal_velocity_tolerance = self._load_float_array("goal_velocity_tolerance")
        self.goal_time_tolerance_s = float(self.get_parameter("goal_time_tolerance_s").value)
        self.allow_joint_reordering = bool(self.get_parameter("allow_joint_reordering").value)
        self.input_joint_prefix = str(self.get_parameter("input_joint_prefix").value)
        self.reject_new_goal_while_executing = bool(
            self.get_parameter("reject_new_goal_while_executing").value
        )
        self.enable_debug_joint_trajectory_topic = bool(
            self.get_parameter("enable_debug_joint_trajectory_topic").value
        )

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0")
        if self.action_feedback_rate_hz <= 0.0:
            raise ValueError("action_feedback_rate_hz must be > 0")

        self.enabled = False
        self.enable_time_monotonic = 0.0
        self.last_feedback_monotonic = 0.0
        self.last_joint_feedback_monotonic = 0.0
        self.last_leader_feedback_monotonic = 0.0
        self.feedback_positions: dict[str, float] = {}
        self.feedback_velocities: dict[str, float] = {}
        # Live positions of gravity-payload joints (e.g. OmniHand fingers) that
        # ride behind the arm in an articulated gravity URDF. Filled from the
        # combined feedback/joint_states; joints never seen stay at zero, which
        # equals the frozen static-payload behavior.
        self.payload_feedback_positions: dict[str, float] = {}
        self.gravity_payload_joint_names: frozenset[str] = frozenset()
        self.active_trajectory: Optional[JointTrajectoryBuffer] = None
        self.trajectory_start_monotonic = 0.0
        self.hold_reference: Optional[SampledTrajectoryPoint] = None
        self.last_stale_feedback_log = 0.0
        self._stale_since_monotonic: Optional[float] = None
        self.gravity_model: Optional[GravityModel] = None
        self.calibration_model: Optional[CalibrationModel] = None
        self.leader_mode_active = False
        self.freedrive_active = False
        self.arm_fault_active = False
        self.arm_fault_message = ""
        self.execution_state = ExecutionState.DISABLED
        self.holding_final_point = False
        self.active_goal_handle = None
        self.external_cancel_requested = False
        self.action_feedback_period_s = 1.0 / self.action_feedback_rate_hz

        self._init_feedforward_models()

        self.move_mit_pub = self.create_publisher(MoveMITMsg, "control/move_mit", 10)
        self.reference_pub = self.create_publisher(JointState, "~/reference_joint_states", 10)
        self.execution_state_pub = self.create_publisher(String, "~/execution_state", 10)
        # Diagnostic: the gravity feedforward torque actually commanded (post
        # scale/sign/calibration, clamped). Compare it live against the measured
        # motor effort on feedback/joint_states at the same pose to tune sign/scale
        # — in freedrive (kp=0) this torque is the *only* thing holding the arm.
        self.gravity_ff_pub = self.create_publisher(JointState, "~/gravity_feedforward", 10)
        self.create_subscription(
            JointState,
            "feedback/joint_states",
            self._feedback_callback,
            50,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            "feedback/leader_joint_angles",
            self._leader_feedback_callback,
            50,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            AgxArmStatus,
            "feedback/arm_status",
            self._arm_status_callback,
            20,
            callback_group=self.callback_group,
        )
        if self.enable_debug_joint_trajectory_topic:
            self.create_subscription(
                JointTrajectory,
                "~/joint_trajectory",
                self._trajectory_callback,
                10,
                callback_group=self.callback_group,
            )
        self.create_service(SetBool, "~/enable", self._enable_callback, callback_group=self.callback_group)
        self.create_service(SetBool, "~/freedrive", self._freedrive_callback, callback_group=self.callback_group)
        self.create_service(Empty, "~/hold_current", self._hold_current_callback, callback_group=self.callback_group)
        self.create_service(
            Empty,
            "~/cancel_trajectory",
            self._cancel_trajectory_callback,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_loop,
            callback_group=self.callback_group,
        )
        # Live tuning of the gain/gravity knobs so freedrive sign/scale can be
        # dialled in on hardware without relaunching (e.g. ros2 param set
        # /mit_controller gravity_feedforward_sign 1.0). The control loop reads
        # these attributes every tick.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._publish_execution_state(force=True)
        self.get_logger().info(
            f"MIT controller ready for joints {self.joint_names} at {self.control_rate_hz:.1f} Hz"
        )
        self.get_logger().info(f"MIT gains loaded: kp={self.kp}, kd={self.kd}")

    def _on_set_parameters(self, params) -> "SetParametersResult":
        from rcl_interfaces.msg import SetParametersResult

        live_vectors = {"freedrive_kd", "kp", "kd"}
        live_scalars = {"gravity_scale", "gravity_feedforward_sign"}
        for param in params:
            try:
                if param.name in live_vectors:
                    values = [float(v) for v in param.value]
                    if len(values) != len(self.joint_names):
                        return SetParametersResult(
                            successful=False,
                            reason=f"{param.name} needs {len(self.joint_names)} values",
                        )
                    setattr(self, param.name, values)
                elif param.name in live_scalars:
                    setattr(self, param.name, float(param.value))
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=f"{param.name}: {exc}")
        return SetParametersResult(successful=True)
        self.get_logger().info(
            f"FollowJointTrajectory action available on '{self.action_name}'"
        )
        if self.enable_debug_joint_trajectory_topic:
            self.get_logger().warn(
                "Debug ~/joint_trajectory input is enabled; keep it disabled during production MoveIt execution"
            )

    def _init_feedforward_models(self) -> None:
        calibration_path: Path | None = None
        if self.calibration_file:
            calibration_path = Path(self.calibration_file).expanduser().resolve()
        elif self.gravity_urdf_path:
            # A custom gravity URDF (duo body mount and/or hand payload) does not
            # match the assembly the auto-discovered calibration was fitted on;
            # applying that scale/bias would distort the correct model torques.
            # Load a matching calibration explicitly via calibration_file.
            auto_calibration_path = default_nero_calibration_path()
            if auto_calibration_path.exists():
                self.get_logger().warn(
                    f"Skipping auto calibration {auto_calibration_path}: a custom "
                    "gravity URDF is set and the calibration was fitted for the "
                    "default assembly. Set calibration_file explicitly to override."
                )
        else:
            auto_calibration_path = default_nero_calibration_path()
            if auto_calibration_path.exists():
                calibration_path = auto_calibration_path

        if calibration_path is not None:
            self.calibration_model = load_calibration_model(calibration_path, self.joint_names)
            self.get_logger().info(f"Loaded calibration model from {calibration_path}")
        if self.gravity_compensation_enabled:
            try:
                gravity_urdf_path = self.gravity_urdf_path or None
                self.gravity_model = create_gravity_model(
                    self.gravity_backend, gravity_urdf_path, self.gravity_mounting_rpy
                )
                self.get_logger().info(
                    f"Gravity compensation enabled via {self.gravity_backend} using {self.gravity_model.urdf_path} "
                    f"(mounting_rpy={self.gravity_mounting_rpy})"
                )
                payload_joint_names = self.gravity_model.joint_names[len(self.joint_names):]
                self.gravity_payload_joint_names = frozenset(payload_joint_names)
                if payload_joint_names:
                    self.get_logger().info(
                        "Gravity model articulates "
                        f"{len(payload_joint_names)} payload joints from live feedback: "
                        f"{payload_joint_names}"
                    )
            except GravityModelError as exc:
                self.get_logger().error(str(exc))
                self.gravity_model = None
                self.gravity_compensation_enabled = False

    def _load_float_array(self, name: str) -> list[float]:
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"parameter '{name}' must have {len(self.joint_names)} entries, got {len(values)}"
            )
        return values

    def _joint_state_maps(self, msg: JointState) -> tuple[dict[str, float], dict[str, float]] | None:
        position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
        velocity_map = {
            name: float(value)
            for name, value in zip(msg.name, msg.velocity or [0.0] * len(msg.name))
        }

        missing = [joint for joint in self.joint_names if joint not in position_map]
        if missing:
            return None
        return position_map, velocity_map

    def _set_feedback_state(self, position_map: dict[str, float], velocity_map: dict[str, float]) -> None:
        self.feedback_positions = {joint: position_map[joint] for joint in self.joint_names}
        self.feedback_velocities = {joint: velocity_map.get(joint, 0.0) for joint in self.joint_names}
        self.last_feedback_monotonic = time.monotonic()

    def _publish_execution_state(self, *, force: bool = False) -> None:
        if not force and self.execution_state == self.execution_state:
            pass

        msg = String()
        msg.data = self.execution_state.value
        self.execution_state_pub.publish(msg)

    def _set_execution_state(self, state: ExecutionState, *, force: bool = False) -> None:
        if not force and self.execution_state == state:
            return
        self.execution_state = state
        self._publish_execution_state(force=True)

    def _has_fresh_joint_feedback(self) -> bool:
        if self.last_joint_feedback_monotonic <= 0.0:
            return False
        return (time.monotonic() - self.last_joint_feedback_monotonic) <= self.feedback_timeout_s

    def _has_fresh_leader_feedback(self) -> bool:
        if self.last_leader_feedback_monotonic <= 0.0:
            return False
        return (time.monotonic() - self.last_leader_feedback_monotonic) <= self.feedback_timeout_s

    def _should_use_leader_feedback(self) -> bool:
        return self.leader_mode_active or (self._has_fresh_leader_feedback() and not self._has_fresh_joint_feedback())

    def _feedback_callback(self, msg: JointState) -> None:
        with self.state_lock:
            if self.gravity_payload_joint_names:
                for index, joint_name in enumerate(msg.name):
                    if joint_name in self.gravity_payload_joint_names and index < len(msg.position):
                        self.payload_feedback_positions[joint_name] = float(msg.position[index])

            state_maps = self._joint_state_maps(msg)
            if state_maps is None:
                missing = [joint for joint in self.joint_names if joint not in msg.name]
                self.get_logger().warn(f"feedback/joint_states missing joints {missing}")
                return

            position_map, velocity_map = state_maps
            self.last_joint_feedback_monotonic = time.monotonic()
            self._set_feedback_state(position_map, velocity_map)

    def _leader_feedback_callback(self, msg: JointState) -> None:
        with self.state_lock:
            state_maps = self._joint_state_maps(msg)
            if state_maps is None:
                return

            self.last_leader_feedback_monotonic = time.monotonic()
            if not self._should_use_leader_feedback():
                return

            position_map, velocity_map = state_maps
            self._set_feedback_state(position_map, velocity_map)
            if self.enabled and self.active_trajectory is None:
                self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False

    def _arm_status_callback(self, msg: AgxArmStatus) -> None:
        with self.state_lock:
            was_leader_mode_active = self.leader_mode_active
            self.leader_mode_active = msg.ctrl_mode == CTRL_MODE_LINKAGE_TEACHING_INPUT_MODE

            was_fault_active = self.arm_fault_active
            self.arm_fault_active = int(msg.err_status) != 0
            self.arm_fault_message = (
                f"Arm status fault err_status={int(msg.err_status)}"
                if self.arm_fault_active
                else ""
            )

            if self.leader_mode_active and not was_leader_mode_active and self.active_trajectory is not None:
                self.active_trajectory = None
                self.external_cancel_requested = True
                self.holding_final_point = False
                self.set_execution_state_safe(ExecutionState.LEADER_MODE)
                self.get_logger().warn(
                    "Cancelled active MIT trajectory because the robot entered leader mode"
                )

            if self.arm_fault_active and not was_fault_active:
                self.active_trajectory = None
                if self._has_fresh_feedback():
                    self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False
                self.get_logger().error(self.arm_fault_message)
                self.set_execution_state_safe(ExecutionState.FAULTED)

            if was_leader_mode_active and not self.leader_mode_active and self.enabled and self._has_fresh_feedback():
                self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False

            if was_fault_active and not self.arm_fault_active and self.enabled and self._has_fresh_feedback():
                self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False

    def set_execution_state_safe(self, state: ExecutionState) -> None:
        self._set_execution_state(state)

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        with self.state_lock:
            if self.active_goal_handle is not None:
                self.get_logger().warn(
                    "Ignoring debug JointTrajectory because a FollowJointTrajectory goal is active"
                )
                return

            if not self.enabled:
                self.get_logger().warn(
                    "Rejecting debug JointTrajectory while MIT is disabled; enable mit_controller explicitly first"
                )
                return

            buffer, _, detail = self._validate_trajectory_goal(msg)
            if buffer is None:
                self.get_logger().error(f"Rejected debug JointTrajectory: {detail}")
                return

            self._activate_trajectory(buffer)
            self.get_logger().info(
                f"Accepted debug trajectory with {len(msg.points)} points and {buffer.duration:.3f}s duration"
            )

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self.state_lock:
            if request.data and not self._has_fresh_feedback():
                response.success = False
                response.message = "Cannot enable without fresh feedback/joint_states"
                return response

            self._set_enabled(request.data)
            response.success = True
            response.message = "enabled" if request.data else "disabled"
            return response

    def _freedrive_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self.state_lock:
            if request.data:
                if not self.gravity_compensation_enabled or self.gravity_model is None:
                    response.success = False
                    response.message = "freedrive requires gravity compensation (no gravity model loaded)"
                    return response
                if not self._has_fresh_feedback():
                    response.success = False
                    response.message = "Cannot enter freedrive without fresh feedback/joint_states"
                    return response
                if not self.enabled:
                    self._set_enabled(True)
                self.active_trajectory = None
                self.holding_final_point = False
                self.freedrive_active = True
                self._set_execution_state(ExecutionState.FREEDRIVE)
                self.get_logger().info("Freedrive on (zero-force, gravity-compensated)")
                response.success = True
                response.message = "freedrive on"
            else:
                self.freedrive_active = False
                if self._has_fresh_feedback():
                    self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False
                self._set_execution_state(ExecutionState.IDLE_HOLD)
                self.get_logger().info("Freedrive off (holding current pose)")
                response.success = True
                response.message = "freedrive off"
            return response

    def _hold_current_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        with self.state_lock:
            if not self._has_fresh_feedback():
                self.get_logger().warn("Ignoring hold_current request without fresh feedback")
                return response
            self.freedrive_active = False
            self.active_trajectory = None
            self.hold_reference = self._capture_current_reference()
            self.holding_final_point = False
            self._set_execution_state(ExecutionState.IDLE_HOLD)
            self.get_logger().info("Captured current joint state as MIT hold target")
            return response

    def _cancel_trajectory_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        with self.state_lock:
            self.external_cancel_requested = True
            self.active_trajectory = None
            if self._has_fresh_feedback():
                self.hold_reference = self._capture_current_reference()
            self.holding_final_point = False
            self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
            self.get_logger().info("Cancelled active MIT trajectory")
            return response

    def _set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self.enable_time_monotonic = time.monotonic()
        if enabled:
            if self._has_fresh_feedback():
                self.hold_reference = self._capture_current_reference()
                self.holding_final_point = False
            if self.leader_mode_active:
                self._set_execution_state(ExecutionState.LEADER_MODE)
            elif self.arm_fault_active:
                self._set_execution_state(ExecutionState.FAULTED)
            elif self.gain_ramp_time_s > 0.0:
                self._set_execution_state(ExecutionState.ARMING)
            else:
                self._set_execution_state(ExecutionState.IDLE_HOLD)
            self.get_logger().info("MIT controller enabled")
        else:
            self.active_trajectory = None
            self.hold_reference = None
            self.holding_final_point = False
            self.freedrive_active = False
            self.external_cancel_requested = True
            self._set_execution_state(ExecutionState.DISABLED)
            self.get_logger().info("MIT controller disabled")

    def _has_fresh_feedback(self) -> bool:
        if not self.feedback_positions:
            return False
        return (time.monotonic() - self.last_feedback_monotonic) <= self.feedback_timeout_s

    def _capture_current_reference(self) -> SampledTrajectoryPoint:
        return SampledTrajectoryPoint(
            positions=tuple(self.feedback_positions[joint] for joint in self.joint_names),
            velocities=(0.0,) * len(self.joint_names),
            efforts=(0.0,) * len(self.joint_names),
        )

    def _activate_trajectory(self, buffer: JointTrajectoryBuffer) -> None:
        if self.auto_enable_on_trajectory and not self.enabled:
            self._set_enabled(True)
        self.freedrive_active = False
        self.active_trajectory = buffer
        self.trajectory_start_monotonic = time.monotonic()
        self.hold_reference = None
        self.holding_final_point = False
        self.external_cancel_requested = False
        self._set_execution_state(ExecutionState.EXECUTING_TRAJECTORY)

    def _trajectory_buffer_from_message(self, msg: JointTrajectory) -> JointTrajectoryBuffer:
        return JointTrajectoryBuffer.from_ros_message(
            self.joint_names,
            msg,
            allow_joint_reordering=self.allow_joint_reordering,
            input_joint_prefix=self.input_joint_prefix,
        )

    def _validation_error_code(self, message: str) -> int:
        if "joint_names" in message:
            return FollowJointTrajectory.Result.INVALID_JOINTS
        return FollowJointTrajectory.Result.INVALID_GOAL

    def _position_errors(self, desired: SampledTrajectoryPoint) -> list[float]:
        return [
            float(desired.positions[index]) - self.feedback_positions.get(joint_name, 0.0)
            for index, joint_name in enumerate(self.joint_names)
        ]

    def _velocity_errors(self, desired: SampledTrajectoryPoint) -> list[float]:
        return [
            float(desired.velocities[index]) - self.feedback_velocities.get(joint_name, 0.0)
            for index, joint_name in enumerate(self.joint_names)
        ]

    def _validate_start_state(self, buffer: JointTrajectoryBuffer) -> str:
        errors = self._position_errors(buffer.initial_point)
        violations = [
            (self.joint_names[index], errors[index], self.start_state_tolerance[index])
            for index in range(len(self.joint_names))
            if math.fabs(errors[index]) > self.start_state_tolerance[index]
        ]
        if not violations:
            return ""

        joint_name, error_value, tolerance = max(violations, key=lambda item: math.fabs(item[1]))
        return (
            f"Start state mismatch on {joint_name}: {error_value:.3f} rad exceeds "
            f"tolerance {tolerance:.3f} rad"
        )

    def _validate_trajectory_goal(
        self,
        trajectory: JointTrajectory,
    ) -> tuple[Optional[JointTrajectoryBuffer], int, str]:
        if not self._has_fresh_feedback():
            return (
                None,
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Cannot accept trajectory without fresh feedback/joint_states",
            )
        if self.leader_mode_active:
            return (
                None,
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Cannot accept trajectory while leader mode is active",
            )
        if self.arm_fault_active:
            return (
                None,
                FollowJointTrajectory.Result.INVALID_GOAL,
                self.arm_fault_message or "Cannot accept trajectory while an arm fault is active",
            )
        if self.active_goal_handle is not None:
            if self.reject_new_goal_while_executing:
                return (
                    None,
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Another FollowJointTrajectory goal is already executing",
                )
            return (
                None,
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Goal preemption is not implemented yet; keep reject_new_goal_while_executing enabled",
            )

        try:
            buffer = self._trajectory_buffer_from_message(trajectory)
        except ValueError as exc:
            detail = str(exc)
            return None, self._validation_error_code(detail), detail

        detail = self._validate_start_state(buffer)
        if detail:
            return None, FollowJointTrajectory.Result.INVALID_GOAL, detail

        return buffer, FollowJointTrajectory.Result.SUCCESSFUL, ""

    def _actual_point(self) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [self.feedback_positions.get(joint, 0.0) for joint in self.joint_names]
        point.velocities = [self.feedback_velocities.get(joint, 0.0) for joint in self.joint_names]
        point.effort = [0.0] * len(self.joint_names)
        return point

    def _point_from_sample(self, sample: SampledTrajectoryPoint) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(sample.positions)
        point.velocities = list(sample.velocities)
        point.effort = list(sample.efforts)
        return point

    def _error_point(self, desired: SampledTrajectoryPoint) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = self._position_errors(desired)
        point.velocities = self._velocity_errors(desired)
        point.effort = [0.0] * len(self.joint_names)
        return point

    def _goal_tolerances_from_request(
        self,
        goal_request: FollowJointTrajectory.Goal,
    ) -> tuple[list[float], list[float], float]:
        position_tolerance = list(self.goal_position_tolerance)
        velocity_tolerance = list(self.goal_velocity_tolerance)
        tolerance_by_name = {tolerance.name: tolerance for tolerance in goal_request.goal_tolerance if tolerance.name}

        for index, joint_name in enumerate(self.joint_names):
            tolerance = tolerance_by_name.get(joint_name)
            if tolerance is None:
                continue
            if float(tolerance.position) > 0.0:
                position_tolerance[index] = float(tolerance.position)
            if float(tolerance.velocity) > 0.0:
                velocity_tolerance[index] = float(tolerance.velocity)

        goal_time_tolerance = duration_to_seconds(goal_request.goal_time_tolerance)
        if goal_time_tolerance <= 0.0:
            goal_time_tolerance = self.goal_time_tolerance_s

        return position_tolerance, velocity_tolerance, goal_time_tolerance

    def _goal_within_tolerance(
        self,
        desired: SampledTrajectoryPoint,
        position_tolerance: list[float],
        velocity_tolerance: list[float],
    ) -> tuple[bool, str]:
        position_errors = self._position_errors(desired)
        velocity_errors = self._velocity_errors(desired)

        for index, joint_name in enumerate(self.joint_names):
            if math.fabs(position_errors[index]) > position_tolerance[index]:
                return (
                    False,
                    f"Goal position tolerance violated on {joint_name}: {position_errors[index]:.3f} > {position_tolerance[index]:.3f}",
                )
            if math.fabs(velocity_errors[index]) > velocity_tolerance[index]:
                return (
                    False,
                    f"Goal velocity tolerance violated on {joint_name}: {velocity_errors[index]:.3f} > {velocity_tolerance[index]:.3f}",
                )

        return True, ""

    def _position_error_limit_violation(self, desired: SampledTrajectoryPoint) -> str:
        errors = self._position_errors(desired)
        for index, joint_name in enumerate(self.joint_names):
            if math.fabs(errors[index]) > self.position_error_limit[index]:
                return (
                    f"Position error limit exceeded on {joint_name}: {errors[index]:.3f} > {self.position_error_limit[index]:.3f}"
                )
        return ""

    def _success_result(self) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _failed_result(self, code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal):
        with self.state_lock:
            _, _, detail = self._validate_trajectory_goal(goal_request.trajectory)
        if detail:
            self.get_logger().error(f"Rejected FollowJointTrajectory goal: {detail}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_follow_joint_trajectory(self, goal_handle):
        with self.state_lock:
            buffer, error_code, detail = self._validate_trajectory_goal(goal_handle.request.trajectory)
            if buffer is None:
                goal_handle.abort()
                return self._failed_result(error_code, detail)

            self.active_goal_handle = goal_handle
            self._activate_trajectory(buffer)
            start_time = self.trajectory_start_monotonic

        self.get_logger().info(
            f"Accepted FollowJointTrajectory goal with {len(goal_handle.request.trajectory.points)} points and {buffer.duration:.3f}s duration"
        )

        position_tolerance, velocity_tolerance, goal_time_tolerance = self._goal_tolerances_from_request(
            goal_handle.request
        )
        next_feedback_time = 0.0

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    with self.state_lock:
                        self.active_trajectory = None
                        if self._has_fresh_feedback():
                            self.hold_reference = self._capture_current_reference()
                        self.holding_final_point = False
                        self.active_goal_handle = None
                        self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
                    goal_handle.canceled()
                    return self._failed_result(
                        FollowJointTrajectory.Result.INVALID_GOAL,
                        "Goal canceled",
                    )

                with self.state_lock:
                    if self.external_cancel_requested:
                        self.external_cancel_requested = False
                        self.active_goal_handle = None
                        self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
                        goal_handle.canceled()
                        return self._failed_result(
                            FollowJointTrajectory.Result.INVALID_GOAL,
                            "Goal canceled by external MIT cancel request",
                        )

                    if not self.enabled:
                        self.active_goal_handle = None
                        goal_handle.abort()
                        return self._failed_result(
                            FollowJointTrajectory.Result.INVALID_GOAL,
                            "MIT controller was disabled while executing the goal",
                        )

                    if self.leader_mode_active:
                        self.active_goal_handle = None
                        goal_handle.abort()
                        return self._failed_result(
                            FollowJointTrajectory.Result.INVALID_GOAL,
                            "Robot entered leader mode while executing the goal",
                        )

                    if self.arm_fault_active:
                        self.active_goal_handle = None
                        goal_handle.abort()
                        return self._failed_result(
                            FollowJointTrajectory.Result.INVALID_GOAL,
                            self.arm_fault_message or "Arm fault while executing the goal",
                        )

                    if not self._has_fresh_feedback():
                        # Mirror the position-limit abort: never leave the stale
                        # trajectory armed, or a feedback comeback snaps the arm to
                        # a far-ahead point. Feedback is not fresh here, so we
                        # cannot capture a hold now — clear it and let the control
                        # loop recapture the current pose when feedback returns.
                        self.active_trajectory = None
                        self.hold_reference = None
                        self.holding_final_point = False
                        self.active_goal_handle = None
                        self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
                        goal_handle.abort()
                        return self._failed_result(
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                            "Feedback became stale while executing the goal",
                        )

                    elapsed = time.monotonic() - start_time
                    desired = buffer.sample(elapsed)
                    position_limit_detail = self._position_error_limit_violation(desired)
                    if position_limit_detail:
                        self.active_trajectory = None
                        if self._has_fresh_feedback():
                            self.hold_reference = self._capture_current_reference()
                        self.holding_final_point = False
                        self.active_goal_handle = None
                        self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
                        goal_handle.abort()
                        return self._failed_result(
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                            position_limit_detail,
                        )

                    now = time.monotonic()
                    if now >= next_feedback_time:
                        feedback = FollowJointTrajectory.Feedback()
                        feedback.joint_names = list(self.joint_names)
                        feedback.desired = self._point_from_sample(desired)
                        feedback.actual = self._actual_point()
                        feedback.error = self._error_point(desired)
                        goal_handle.publish_feedback(feedback)
                        next_feedback_time = now + self.action_feedback_period_s

                    if elapsed >= buffer.duration:
                        within_tolerance, detail = self._goal_within_tolerance(
                            buffer.final_point,
                            position_tolerance,
                            velocity_tolerance,
                        )
                        if within_tolerance:
                            self.active_goal_handle = None
                            if self.hold_final_point:
                                self.hold_reference = buffer.final_point
                                self.holding_final_point = True
                            else:
                                self.hold_reference = self._capture_current_reference()
                                self.holding_final_point = False
                            self.active_trajectory = None
                            self._set_execution_state(
                                ExecutionState.HOLDING_FINAL_POINT
                                if self.holding_final_point
                                else ExecutionState.IDLE_HOLD
                            )
                            goal_handle.succeed()
                            return self._success_result()

                        if elapsed >= buffer.duration + goal_time_tolerance:
                            self.active_trajectory = None
                            if self._has_fresh_feedback():
                                self.hold_reference = self._capture_current_reference()
                            self.holding_final_point = False
                            self.active_goal_handle = None
                            self._set_execution_state(ExecutionState.CANCELING_TO_HOLD)
                            goal_handle.abort()
                            return self._failed_result(
                                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                detail,
                            )

                time.sleep(min(0.02, self.action_feedback_period_s))
        finally:
            with self.state_lock:
                if self.active_goal_handle is goal_handle:
                    self.active_goal_handle = None

    def _gain_scale(self) -> float:
        if self.gain_ramp_time_s <= 0.0:
            return 1.0
        elapsed = time.monotonic() - self.enable_time_monotonic
        return max(0.0, min(1.0, elapsed / self.gain_ramp_time_s))

    def _compute_feedforward(self, reference: SampledTrajectoryPoint) -> list[float]:
        if not self.gravity_compensation_enabled or self.gravity_model is None:
            return [float(value) for value in reference.efforts]

        gravity_torque = self.gravity_model.compute_gravity(
            [self.feedback_positions[joint_name] for joint_name in self.joint_names],
            extra_joint_positions=(
                self.payload_feedback_positions if self.gravity_payload_joint_names else None
            ),
        )
        scaled_torque = scale_gravity_feedforward(
            gravity_torque,
            self.gravity_scale,
            self.gravity_feedforward_sign,
        )
        if self.calibration_model is not None:
            scaled_torque = self.calibration_model.apply(scaled_torque)
        return [scaled_torque[index] + float(reference.efforts[index]) for index in range(len(self.joint_names))]

    def _reference_from_state(self) -> Optional[SampledTrajectoryPoint]:
        # sample from trajectory if active, otherwise hold reference or current state
        if self.active_trajectory is not None:
            elapsed = time.monotonic() - self.trajectory_start_monotonic
            sampled = self.active_trajectory.sample(elapsed)
            if elapsed >= self.active_trajectory.duration:
                if self.hold_final_point:
                    self.hold_reference = sampled
                    self.holding_final_point = True
                self.active_trajectory = None
            return sampled
        if self.hold_reference is None and self._has_fresh_feedback():
            self.hold_reference = self._capture_current_reference()
            self.holding_final_point = False
        return self.hold_reference

    def _control_loop(self) -> None:
        with self.state_lock:
            if not self.enabled:
                self._set_execution_state(ExecutionState.DISABLED)
                return

            if self.leader_mode_active or self._should_use_leader_feedback():
                self._set_execution_state(ExecutionState.LEADER_MODE)
                return

            if self.arm_fault_active:
                self._set_execution_state(ExecutionState.FAULTED)
                return

            if not self._has_fresh_feedback():
                now = time.monotonic()
                if self._stale_since_monotonic is None:
                    self._stale_since_monotonic = now
                    # Drop any active trajectory the instant feedback goes stale:
                    # its start clock keeps running through the outage, so a
                    # feedback comeback would sample a far-ahead point and snap
                    # the arm with MIT gains. Recapture the current pose as the
                    # hold reference when feedback returns instead.
                    if self.active_trajectory is not None:
                        self.active_trajectory = None
                        self.hold_reference = None
                        self.holding_final_point = False
                if now - self.last_stale_feedback_log > 1.0:
                    self.get_logger().warn(
                        "Feedback is stale; streaming damped-stop MIT commands (dead-man)"
                    )
                    self.last_stale_feedback_log = now
                self._set_execution_state(ExecutionState.STALE_FEEDBACK)
                # Dead-man: the firmware executes the LAST received MIT command
                # indefinitely — going silent here left a moving arm moving
                # (runaway observed live during a teach recording). Stream a
                # kd-damped zero-velocity command instead so the firmware's
                # active setpoint is a stop, not the last motion.
                self._publish_damped_stop_command(now - self._stale_since_monotonic)
                return
            self._stale_since_monotonic = None

            if self.freedrive_active:
                self._publish_freedrive_command()
                self._set_execution_state(ExecutionState.FREEDRIVE)
                return

            # get reference hold or trajectory pose
            reference = self._reference_from_state()
            if reference is None:
                return

            # scale gain if control was recently enabled to provide a smooth ramp-up of the controller effort
            gain_scale = self._gain_scale()
            feedforward = self._compute_feedforward(reference)

            cmd = MoveMITMsg()
            cmd.joint_index = list(range(1, len(self.joint_names) + 1))
            cmd.p_des = []
            cmd.v_des = []
            cmd.kp = []
            cmd.kd = []
            cmd.torque = []

            for index, joint_name in enumerate(self.joint_names):
                # set to hold or trajectory pose
                current_position = self.feedback_positions[joint_name]
                desired_position = float(reference.positions[index])
                desired_velocity = clamp(float(reference.velocities[index]), self.velocity_limit[index])
                desired_torque = clamp(float(feedforward[index]), self.torque_limit[index])
                position_error = desired_position - current_position

                # (un)comment the following block to switch a joint to (not) hold if it exceeds the position error limit
                if math.fabs(position_error) > self.position_error_limit[index]:
                    self.get_logger().warn(
                        f"Joint {joint_name} exceeded position error limit; switching that joint to hold"
                    )
                    desired_position = current_position
                    desired_velocity = 0.0
                    desired_torque = 0.0
                
                cmd.p_des.append(desired_position)
                cmd.v_des.append(desired_velocity)
                cmd.kp.append(self.kp[index] * gain_scale)
                cmd.kd.append(self.kd[index] * gain_scale)
                cmd.torque.append(desired_torque)

            if self.active_trajectory is not None:
                self._set_execution_state(ExecutionState.EXECUTING_TRAJECTORY)
            elif self.holding_final_point and self.hold_reference is not None:
                self._set_execution_state(ExecutionState.HOLDING_FINAL_POINT)
            elif gain_scale < 1.0:
                self._set_execution_state(ExecutionState.ARMING)
            else:
                self._set_execution_state(ExecutionState.IDLE_HOLD)

            self.move_mit_pub.publish(cmd)
            self._publish_reference(reference)
            self._publish_gravity_feedforward(cmd.torque)

    # Dead-man torque schedule: keep the frozen gravity feedforward through a
    # short grace window (a stale blip in freedrive must not sag the arm), then
    # ramp it to zero — a feedforward frozen for a pose the arm has left can
    # actively drive it, while pure kd damping can only brake.
    STALE_STOP_TORQUE_GRACE_S = 1.0
    STALE_STOP_TORQUE_RAMP_S = 2.0

    def _publish_damped_stop_command(self, stale_duration_s: float) -> None:
        joint_count = len(self.joint_names)
        positions = [0.0] * joint_count
        torques = [0.0] * joint_count
        if all(name in self.feedback_positions for name in self.joint_names):
            positions = [self.feedback_positions[name] for name in self.joint_names]
            ramp_progress = (
                stale_duration_s - self.STALE_STOP_TORQUE_GRACE_S
            ) / self.STALE_STOP_TORQUE_RAMP_S
            torque_scale = 1.0 - min(max(ramp_progress, 0.0), 1.0)
            if torque_scale > 0.0:
                reference = SampledTrajectoryPoint(
                    positions=tuple(positions),
                    velocities=(0.0,) * joint_count,
                    efforts=(0.0,) * joint_count,
                )
                feedforward = self._compute_feedforward(reference)
                torques = [
                    torque_scale * clamp(float(feedforward[index]), self.torque_limit[index])
                    for index in range(joint_count)
                ]

        cmd = MoveMITMsg()
        cmd.joint_index = list(range(1, joint_count + 1))
        cmd.p_des = [float(value) for value in positions]
        cmd.v_des = [0.0] * joint_count
        cmd.kp = [0.0] * joint_count
        cmd.kd = [float(value) for value in self.freedrive_kd]
        cmd.torque = torques
        self.move_mit_pub.publish(cmd)

    def _publish_freedrive_command(self) -> None:
        """Zero-force, gravity-compensated command so the arm is back-drivable.

        kp is forced to 0 (no position hold) and only the gravity feedforward
        torque plus kd damping is applied, so the operator can hand-move the arm
        while the model carries its own weight. This is our software leader mode:
        unlike the firmware drag mode it honours the mounting pose baked into the
        gravity model. p_des tracks the live position so a later kp>0 hand-off
        never snaps to a stale target.
        """
        positions = [self.feedback_positions[joint_name] for joint_name in self.joint_names]
        reference = SampledTrajectoryPoint(
            positions=tuple(positions),
            velocities=(0.0,) * len(self.joint_names),
            efforts=(0.0,) * len(self.joint_names),
        )
        feedforward = self._compute_feedforward(reference)

        cmd = MoveMITMsg()
        cmd.joint_index = list(range(1, len(self.joint_names) + 1))
        cmd.p_des = [float(value) for value in positions]
        cmd.v_des = [0.0] * len(self.joint_names)
        cmd.kp = [0.0] * len(self.joint_names)
        cmd.kd = [float(value) for value in self.freedrive_kd]
        cmd.torque = [
            clamp(float(feedforward[index]), self.torque_limit[index])
            for index in range(len(self.joint_names))
        ]
        self.move_mit_pub.publish(cmd)
        self._publish_reference(reference)
        self._publish_gravity_feedforward(cmd.torque)

    def _publish_reference(self, reference: SampledTrajectoryPoint) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in reference.positions]
        msg.velocity = [float(value) for value in reference.velocities]
        msg.effort = [float(value) for value in reference.efforts]
        self.reference_pub.publish(msg)

    def _publish_gravity_feedforward(self, torque: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.effort = [float(value) for value in torque]
        self.gravity_ff_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NeroMitControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        # Best-effort dead-man on shutdown: the firmware keeps executing the
        # last MIT command after our stream stops, so leave it a damped stop
        # as the final setpoint (pure kd damping, zero torque). Requires the
        # arm driver to still be up to reach the bus.
        try:
            if node.enabled:
                for _ in range(5):
                    node._publish_damped_stop_command(float("inf"))
                    time.sleep(0.02)
        except Exception:
            pass
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()