#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from agx_arm_msgs.msg import OmniHandStatus, OmniHandTactileRaw


JOINT_SUFFIXES = [
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "index_abad_joint",
    "index_pip_joint",
    "middle_pip_joint",
    "ring_abad_joint",
    "ring_pip_joint",
    "pinky_abad_joint",
    "pinky_pip_joint",
]


def build_joint_names(hand_side: str) -> list[str]:
    prefix = f"{hand_side}_"
    return [f"{prefix}{suffix}" for suffix in JOINT_SUFFIXES]


@dataclass
class OmniHandStatusSnapshot:
    backend_name: str
    control_mode: str
    connected: bool
    initialized: bool
    is_mock: bool
    communication_fault: bool
    active_joint_temperatures_c: list[float]
    active_joint_currents_a: list[float]
    active_joint_stalled: list[bool]
    active_joint_over_temperature: list[bool]
    active_joint_over_current: list[bool]
    status_text: str


@dataclass
class OmniHandTactileSnapshot:
    backend_name: str
    layout_name: str
    values: list[float]


class MockOmniHandBackend:

    def __init__(self, hand_side: str, tactile_sample_count: int) -> None:
        self.hand_side = hand_side
        self.backend_name = "mock_backend"
        self.control_mode = "idle"
        self.connected = True
        self.initialized = True
        self.is_mock = True
        self.communication_fault = False
        self.status_text = "mock backend ready"
        self.joint_names = build_joint_names(hand_side)
        self.positions = [0.0] * len(self.joint_names)
        self.temperatures_c = [25.0] * len(self.joint_names)
        self.currents_a = [0.0] * len(self.joint_names)
        self.stalled = [False] * len(self.joint_names)
        self.over_temperature = [False] * len(self.joint_names)
        self.over_current = [False] * len(self.joint_names)
        self.tactile_values = [0.0] * max(0, tactile_sample_count)

    def get_joint_names(self) -> list[str]:
        return list(self.joint_names)

    def apply_joint_targets(self, target_map: dict[str, float], control_mode: str) -> int:
        matched_joint_count = 0
        for index, joint_name in enumerate(self.joint_names):
            if joint_name in target_map:
                self.positions[index] = float(target_map[joint_name])
                matched_joint_count += 1

        if matched_joint_count == 0:
            raise ValueError("received command with no recognized OmniHand joints")

        self.control_mode = control_mode
        self.status_text = (
            f"applied mock {control_mode} command with {matched_joint_count} commanded joints"
        )
        return matched_joint_count

    def apply_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            raise ValueError("received JointTrajectory with no points")

        final_point = msg.points[-1]
        if len(final_point.positions) != len(msg.joint_names):
            raise ValueError("joint_names and final point positions length mismatch")

        target_map = dict(zip(msg.joint_names, final_point.positions, strict=True))
        self.apply_joint_targets(target_map, "joint_trajectory")

    def stop(self) -> None:
        self.control_mode = "stopped"
        self.currents_a = [0.0] * len(self.joint_names)
        self.status_text = "mock stop requested"

    def read_joint_state(self) -> list[float]:
        return list(self.positions)

    def read_status(self) -> OmniHandStatusSnapshot:
        return OmniHandStatusSnapshot(
            backend_name=self.backend_name,
            control_mode=self.control_mode,
            connected=self.connected,
            initialized=self.initialized,
            is_mock=self.is_mock,
            communication_fault=self.communication_fault,
            active_joint_temperatures_c=list(self.temperatures_c),
            active_joint_currents_a=list(self.currents_a),
            active_joint_stalled=list(self.stalled),
            active_joint_over_temperature=list(self.over_temperature),
            active_joint_over_current=list(self.over_current),
            status_text=self.status_text,
        )

    def read_tactile(self) -> OmniHandTactileSnapshot:
        return OmniHandTactileSnapshot(
            backend_name=self.backend_name,
            layout_name="flat_array",
            values=list(self.tactile_values),
        )


class OmniHandBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("omnihand_bridge_node")

        self.declare_parameter("omnihand_type", "left")
        self.declare_parameter("backend_type", "mock")
        self.declare_parameter("pub_rate", 50.0)
        self.declare_parameter("tactile_sample_count", 32)
        self.declare_parameter("joint_states_command_topic", "control/joint_states")

        self.hand_side = str(self.get_parameter("omnihand_type").value)
        self.backend_type = str(self.get_parameter("backend_type").value)
        self.pub_rate = float(self.get_parameter("pub_rate").value)
        self.tactile_sample_count = int(self.get_parameter("tactile_sample_count").value)
        self.joint_states_command_topic = str(
            self.get_parameter("joint_states_command_topic").value
        )

        if self.hand_side not in ("left", "right"):
            raise ValueError("omnihand_type must be 'left' or 'right'")

        if self.backend_type != "mock":
            self.get_logger().warn(
                "Only backend_type=mock is implemented in Sprint 2; falling back to mock backend"
            )
            self.backend_type = "mock"

        self.backend = MockOmniHandBackend(
            hand_side=self.hand_side,
            tactile_sample_count=self.tactile_sample_count,
        )
        self.joint_names = self.backend.get_joint_names()

        self.hand_joint_states_pub = self.create_publisher(
            JointState, "feedback/omnihand/joint_states", 10
        )
        self.status_pub = self.create_publisher(
            OmniHandStatus, "feedback/omnihand/status", 10
        )
        self.tactile_pub = self.create_publisher(
            OmniHandTactileRaw, "feedback/omnihand/tactile_raw", 10
        )

        self.create_subscription(
            JointState,
            self.joint_states_command_topic,
            self._joint_states_command_callback,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "control/omnihand/joint_trajectory",
            self._joint_trajectory_callback,
            10,
        )
        self.create_service(Trigger, "control/omnihand/stop", self._stop_callback)

        timer_period = 1.0 / self.pub_rate if self.pub_rate > 0.0 else 0.02
        self.create_timer(timer_period, self._publish_feedback)

        self.get_logger().info(
            "OmniHand bridge started with "
            f"hand_side={self.hand_side}, backend_type={self.backend_type}, "
            f"joint_states_command_topic={self.joint_states_command_topic}"
        )

    def _joint_states_command_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        target_map = {
            joint_name: float(msg.position[index])
            for index, joint_name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if not target_map:
            return

        try:
            self.backend.apply_joint_targets(target_map, "joint_state")
        except ValueError:
            # Shared control/joint_states frequently contains arm-only updates.
            return

    def _joint_trajectory_callback(self, msg: JointTrajectory) -> None:
        try:
            self.backend.apply_trajectory(msg)
        except ValueError as exc:
            self.get_logger().error(f"Rejected OmniHand JointTrajectory: {exc}")
            return

        unknown_names = [name for name in msg.joint_names if name not in self.joint_names]
        if unknown_names:
            self.get_logger().warn(
                f"Ignored unknown OmniHand joints in trajectory: {', '.join(unknown_names)}"
            )

    def _stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.backend.stop()
        response.success = True
        response.message = "OmniHand mock backend stopped"
        return response

    def _publish_feedback(self) -> None:
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self.joint_names)
        joint_state.position = self.backend.read_joint_state()
        self.hand_joint_states_pub.publish(joint_state)

        status_snapshot = self.backend.read_status()
        status_msg = OmniHandStatus()
        status_msg.header.stamp = stamp
        status_msg.hand_side = self.hand_side
        status_msg.backend_name = status_snapshot.backend_name
        status_msg.control_mode = status_snapshot.control_mode
        status_msg.connected = status_snapshot.connected
        status_msg.initialized = status_snapshot.initialized
        status_msg.is_mock = status_snapshot.is_mock
        status_msg.communication_fault = status_snapshot.communication_fault
        status_msg.active_joint_temperatures_c = status_snapshot.active_joint_temperatures_c
        status_msg.active_joint_currents_a = status_snapshot.active_joint_currents_a
        status_msg.active_joint_stalled = status_snapshot.active_joint_stalled
        status_msg.active_joint_over_temperature = status_snapshot.active_joint_over_temperature
        status_msg.active_joint_over_current = status_snapshot.active_joint_over_current
        status_msg.status_text = status_snapshot.status_text
        self.status_pub.publish(status_msg)

        tactile_snapshot = self.backend.read_tactile()
        tactile_msg = OmniHandTactileRaw()
        tactile_msg.header.stamp = stamp
        tactile_msg.hand_side = self.hand_side
        tactile_msg.backend_name = tactile_snapshot.backend_name
        tactile_msg.layout_name = tactile_snapshot.layout_name
        tactile_msg.values = tactile_snapshot.values
        self.tactile_pub.publish(tactile_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    try:
        node = OmniHandBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()