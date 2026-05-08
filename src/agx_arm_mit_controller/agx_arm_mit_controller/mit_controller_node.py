from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory

from agx_arm_msgs.msg import MoveMITMsg

from .feedforward_model import CalibrationModel, load_calibration_model
from .gravity_model import GravityModel, GravityModelError, create_gravity_model
from .model_metadata import default_nero_calibration_path
from .trajectory_buffer import JointTrajectoryBuffer, SampledTrajectoryPoint


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def scale_gravity_feedforward(
    gravity_torque: list[float],
    gravity_scale: float,
    gravity_feedforward_sign: float,
) -> list[float]:
    return [gravity_feedforward_sign * gravity_scale * value for value in gravity_torque]


class NeroMitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("agx_arm_nero_mit_controller")

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        )
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("feedback_timeout_s", 0.25)
        self.declare_parameter("auto_enable_on_trajectory", True)
        self.declare_parameter("hold_final_point", True)
        self.declare_parameter("gain_ramp_time_s", 1.0)
        self.declare_parameter("position_error_limit", [0.5] * 7)
        self.declare_parameter("velocity_limit", [2.0] * 7)
        self.declare_parameter("torque_limit", [8.0] * 7)
        self.declare_parameter("kp", [1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.5])
        self.declare_parameter("kd", [0.2, 0.3, 0.2, 0.3, 0.12, 0.12, 0.08])
        self.declare_parameter("gravity_compensation_enabled", False)
        self.declare_parameter("gravity_backend", "pinocchio")
        self.declare_parameter("gravity_urdf_path", "")
        self.declare_parameter("gravity_scale", 1.0)
        self.declare_parameter("gravity_feedforward_sign", -1.0)
        self.declare_parameter("calibration_file", "")

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
        self.gravity_compensation_enabled = bool(self.get_parameter("gravity_compensation_enabled").value)
        self.gravity_backend = str(self.get_parameter("gravity_backend").value)
        self.gravity_urdf_path = str(self.get_parameter("gravity_urdf_path").value)
        self.gravity_scale = float(self.get_parameter("gravity_scale").value)
        self.gravity_feedforward_sign = float(self.get_parameter("gravity_feedforward_sign").value)
        self.calibration_file = str(self.get_parameter("calibration_file").value)

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0")

        self.enabled = False
        self.enable_time_monotonic = 0.0
        self.last_feedback_monotonic = 0.0
        self.feedback_positions: dict[str, float] = {}
        self.feedback_velocities: dict[str, float] = {}
        self.active_trajectory: Optional[JointTrajectoryBuffer] = None
        self.trajectory_start_monotonic = 0.0
        self.hold_reference: Optional[SampledTrajectoryPoint] = None
        self.last_stale_feedback_log = 0.0
        self.gravity_model: Optional[GravityModel] = None
        self.calibration_model: Optional[CalibrationModel] = None

        self._init_feedforward_models()

        self.move_mit_pub = self.create_publisher(MoveMITMsg, "control/move_mit", 10)
        self.reference_pub = self.create_publisher(JointState, "~/reference_joint_states", 10)
        self.create_subscription(JointState, "feedback/joint_states", self._feedback_callback, 50)
        self.create_subscription(JointTrajectory, "~/joint_trajectory", self._trajectory_callback, 10)
        self.create_service(SetBool, "~/enable", self._enable_callback)
        self.create_service(Empty, "~/hold_current", self._hold_current_callback)
        self.create_service(Empty, "~/cancel_trajectory", self._cancel_trajectory_callback)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info(
            f"MIT controller ready for joints {self.joint_names} at {self.control_rate_hz:.1f} Hz"
        )

    def _init_feedforward_models(self) -> None:
        calibration_path: Path | None = None
        if self.calibration_file:
            calibration_path = Path(self.calibration_file).expanduser().resolve()
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
                self.gravity_model = create_gravity_model(self.gravity_backend, gravity_urdf_path)
                self.get_logger().info(
                    f"Gravity compensation enabled via {self.gravity_backend} using {self.gravity_model.urdf_path}"
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

    def _feedback_callback(self, msg: JointState) -> None:
        position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
        velocity_map = {
            name: float(value)
            for name, value in zip(msg.name, msg.velocity or [0.0] * len(msg.name))
        }

        missing = [joint for joint in self.joint_names if joint not in position_map]
        if missing:
            self.get_logger().warn(f"feedback/joint_states missing joints {missing}")
            return

        self.feedback_positions = {joint: position_map[joint] for joint in self.joint_names}
        self.feedback_velocities = {joint: velocity_map.get(joint, 0.0) for joint in self.joint_names}
        self.last_feedback_monotonic = time.monotonic()

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        try:
            buffer = JointTrajectoryBuffer.from_ros_message(self.joint_names, msg)
        except ValueError as exc:
            self.get_logger().error(f"Rejected JointTrajectory: {exc}")
            return

        self.active_trajectory = buffer
        self.trajectory_start_monotonic = time.monotonic()
        self.hold_reference = None
        if self.auto_enable_on_trajectory and not self.enabled:
            self._set_enabled(True)
        self.get_logger().info(
            f"Accepted trajectory with {len(msg.points)} points and {buffer.duration:.3f}s duration"
        )

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data and not self._has_fresh_feedback():
            response.success = False
            response.message = "Cannot enable without fresh feedback/joint_states"
            return response

        self._set_enabled(request.data)
        response.success = True
        response.message = "enabled" if request.data else "disabled"
        return response

    def _hold_current_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        if not self._has_fresh_feedback():
            self.get_logger().warn("Ignoring hold_current request without fresh feedback")
            return response
        self.active_trajectory = None
        self.hold_reference = self._capture_current_reference()
        self.get_logger().info("Captured current joint state as MIT hold target")
        return response

    def _cancel_trajectory_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        self.active_trajectory = None
        if self._has_fresh_feedback():
            self.hold_reference = self._capture_current_reference()
        self.get_logger().info("Cancelled active MIT trajectory")
        return response

    def _set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self.enable_time_monotonic = time.monotonic()
        if enabled:
            if self._has_fresh_feedback():
                self.hold_reference = self._capture_current_reference()
            self.get_logger().info("MIT controller enabled")
        else:
            self.active_trajectory = None
            self.hold_reference = None
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

    def _gain_scale(self) -> float:
        if self.gain_ramp_time_s <= 0.0:
            return 1.0
        elapsed = time.monotonic() - self.enable_time_monotonic
        return max(0.0, min(1.0, elapsed / self.gain_ramp_time_s))

    def _compute_feedforward(self, reference: SampledTrajectoryPoint) -> list[float]:
        if not self.gravity_compensation_enabled or self.gravity_model is None:
            return [float(value) for value in reference.efforts]

        gravity_torque = self.gravity_model.compute_gravity(
            [self.feedback_positions[joint_name] for joint_name in self.joint_names]
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
                self.active_trajectory = None
            return sampled
        if self.hold_reference is None and self._has_fresh_feedback():
            self.hold_reference = self._capture_current_reference()
        return self.hold_reference

    def _control_loop(self) -> None:
        if not self.enabled:
            return

        if not self._has_fresh_feedback():
            now = time.monotonic()
            if now - self.last_stale_feedback_log > 1.0:
                self.get_logger().warn("Paused MIT command publishing because feedback is stale")
                self.last_stale_feedback_log = now
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

        self.move_mit_pub.publish(cmd)
        self._publish_reference(reference)

    def _publish_reference(self, reference: SampledTrajectoryPoint) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in reference.positions]
        msg.velocity = [float(value) for value in reference.velocities]
        msg.effort = [float(value) for value in reference.efforts]
        self.reference_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NeroMitControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()