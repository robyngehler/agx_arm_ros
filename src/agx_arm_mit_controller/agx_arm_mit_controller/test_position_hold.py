from __future__ import annotations

import argparse
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger


DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


class PositionHoldTestNode(Node):
    def __init__(self, joint_names: list[str]) -> None:
        super().__init__("mit_position_hold_test")
        self.joint_names = list(joint_names)
        self.feedback_positions: dict[str, float] = {}
        self.last_feedback_monotonic = 0.0

        self.create_subscription(JointState, "feedback/joint_states", self._joint_state_callback, 20)
        self.enable_arm_client = self.create_client(SetBool, "enable_agx_arm")
        self.set_normal_mode_client = self.create_client(Trigger, "set_normal_mode")
        self.enable_mit_client = self.create_client(SetBool, "mit_controller/enable")
        self.hold_current_client = self.create_client(Empty, "mit_controller/hold_current")
        self.parameter_client = self.create_client(GetParameters, "mit_controller/get_parameters")

    def _joint_state_callback(self, msg: JointState) -> None:
        position_map = {name: float(value) for name, value in zip(msg.name, msg.position)}
        missing = [joint for joint in self.joint_names if joint not in position_map]
        if missing:
            return
        self.feedback_positions = {joint: position_map[joint] for joint in self.joint_names}
        self.last_feedback_monotonic = time.monotonic()

    def wait_for_services(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        clients = [
            self.enable_arm_client,
            self.set_normal_mode_client,
            self.enable_mit_client,
            self.hold_current_client,
            self.parameter_client,
        ]
        while time.monotonic() < deadline and rclpy.ok():
            if all(client.wait_for_service(timeout_sec=0.2) for client in clients):
                return True
        return False

    def wait_for_fresh_feedback(self, timeout_s: float, freshness_s: float = 0.5) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.feedback_positions and (time.monotonic() - self.last_feedback_monotonic) <= freshness_s:
                return True
        return False

    def current_positions(self) -> list[float]:
        return [self.feedback_positions[joint] for joint in self.joint_names]

    def call_enable_arm(self, enabled: bool, timeout_s: float) -> bool:
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(response.message)
        return bool(response.success)

    def call_set_normal_mode(self, timeout_s: float) -> bool:
        future = self.set_normal_mode_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(response.message)
        return bool(response.success)

    def call_enable_mit(self, enabled: bool, timeout_s: float) -> bool:
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_mit_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(response.message)
        return bool(response.success)

    def call_hold_current(self, timeout_s: float) -> bool:
        future = self.hold_current_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return bool(future.done() and future.result() is not None)

    def log_controller_parameters(self, timeout_s: float) -> None:
        request = GetParameters.Request()
        request.names = [
            "gravity_compensation_enabled",
            "gravity_scale",
            "gravity_feedforward_sign",
            "gravity_urdf_path",
            "calibration_file",
        ]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().warn("Could not query MIT controller parameters for hold test")
            return

        values = future.result().values
        if len(values) != len(request.names):
            self.get_logger().warn("MIT controller parameter query returned an unexpected result")
            return

        gravity_enabled = bool(values[0].bool_value)
        gravity_scale = float(values[1].double_value)
        gravity_feedforward_sign = float(values[2].double_value)
        gravity_urdf_path = values[3].string_value
        calibration_file = values[4].string_value
        self.get_logger().info(
            "MIT hold test parameters: "
            f"gravity_compensation_enabled={gravity_enabled}, "
            f"gravity_scale={gravity_scale}, "
            f"gravity_feedforward_sign={gravity_feedforward_sign}, "
            f"gravity_urdf_path='{gravity_urdf_path or '<auto>'}', "
            f"calibration_file='{calibration_file or '<auto/none>'}'"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a basic MIT position-hold test without any trajectory playback")
    parser.add_argument("--service-timeout", type=float, default=5.0, help="Timeout for required ROS services")
    parser.add_argument("--feedback-timeout", type=float, default=3.0, help="Timeout waiting for fresh feedback/joint_states")
    parser.add_argument("--duration", type=float, default=8.0, help="How long to hold the captured pose in seconds")
    parser.add_argument("--report-interval", type=float, default=1.0, help="How often to print drift summaries during the hold")
    parser.add_argument("--no-auto-enable-arm", action="store_true", help="Skip calling enable_agx_arm before the test")
    parser.add_argument("--leave-mit-enabled", action="store_true", help="Do not disable MIT when the test exits")
    parser.add_argument("--joint-names", nargs="+", default=DEFAULT_JOINT_NAMES, help="Joint names to track during the hold test")
    return parser.parse_args()


def _format_joint_drift(joint_names: list[str], drifts: list[float]) -> str:
    return ", ".join(
        f"{joint_name}={drift:.3f} rad" for joint_name, drift in zip(joint_names, drifts)
    )


def main() -> None:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be > 0")
    if args.report_interval <= 0.0:
        raise ValueError("--report-interval must be > 0")

    rclpy.init()
    node = PositionHoldTestNode(args.joint_names)
    try:
        if not node.wait_for_services(args.service_timeout):
            raise RuntimeError("Required ROS services are not available for the hold test")

        node.log_controller_parameters(args.service_timeout)

        if not args.no_auto_enable_arm and not node.call_enable_arm(True, args.service_timeout):
            raise RuntimeError("Failed to enable arm through enable_agx_arm")

        if not node.call_set_normal_mode(args.service_timeout):
            raise RuntimeError("Failed to switch robot to normal mode before MIT hold test")

        if not node.wait_for_fresh_feedback(args.feedback_timeout):
            raise RuntimeError("Did not receive fresh feedback/joint_states before MIT hold test")

        print("Place the robot at the pose you want to test, then press Enter to capture and hold it.")
        input()

        if not node.wait_for_fresh_feedback(args.feedback_timeout):
            raise RuntimeError("Did not receive fresh feedback/joint_states when capturing hold pose")

        reference_positions = node.current_positions()

        if not node.call_enable_mit(True, args.service_timeout):
            raise RuntimeError("Failed to enable MIT controller for hold test")
        if not node.call_hold_current(args.service_timeout):
            raise RuntimeError("Failed to call mit_controller/hold_current")

        print(f"Holding captured pose for {args.duration:.1f}s. Watch whether the arm sags or stays in place.")
        hold_start = time.monotonic()
        next_report = hold_start + args.report_interval
        max_abs_drift = [0.0] * len(args.joint_names)
        final_abs_drift = [0.0] * len(args.joint_names)

        while rclpy.ok() and (time.monotonic() - hold_start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.feedback_positions:
                continue
            current_positions = node.current_positions()
            current_abs_drift = [
                abs(current - reference)
                for current, reference in zip(current_positions, reference_positions)
            ]
            final_abs_drift = current_abs_drift
            max_abs_drift = [
                max(previous, current)
                for previous, current in zip(max_abs_drift, current_abs_drift)
            ]

            now = time.monotonic()
            if now >= next_report:
                print(
                    f"[{now - hold_start:4.1f}s] current drift: "
                    f"{_format_joint_drift(args.joint_names, current_abs_drift)}"
                )
                next_report = now + args.report_interval

        print("Hold test finished.")
        print(f"Final drift: {_format_joint_drift(args.joint_names, final_abs_drift)}")
        print(f"Peak drift:  {_format_joint_drift(args.joint_names, max_abs_drift)}")
    finally:
        try:
            if not args.leave_mit_enabled:
                node.call_enable_mit(False, args.service_timeout)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()