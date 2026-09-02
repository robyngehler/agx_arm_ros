from __future__ import annotations

from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_mit_tools.motion_registry import (
	canonical_arm_joints,
	canonical_gripper_joints,
)


# Joint names that live in one arm's name space and therefore take that arm's
# prefix, from the single source of truth (agx_arm_description
# duo_motion_registry.yaml), not a local copy. Everything else passes through
# unchanged, which is what an OmniHand needs: its joints are already
# side-qualified at the source and carry no arm prefix in the URDF.
ARM_PREFIXED_JOINTS = set(canonical_arm_joints()) | set(canonical_gripper_joints())


def adapt_joint_name(name: str, joint_prefix: str, mode: str) -> str:
	joint_name = str(name)
	if not joint_prefix:
		return joint_name
	if mode == "prepend":
		if joint_name.startswith(joint_prefix):
			return joint_name
		if joint_name not in ARM_PREFIXED_JOINTS:
			return joint_name
		return f"{joint_prefix}{joint_name}"
	if mode == "strip":
		if joint_name.startswith(joint_prefix):
			return joint_name[len(joint_prefix):]
		return joint_name
	raise ValueError(f"Unsupported adapter mode: {mode}")


def adapt_joint_names(joint_names: Sequence[str], joint_prefix: str, mode: str) -> list[str]:
	return [adapt_joint_name(name, joint_prefix, mode) for name in joint_names]


class JointStateNameAdapter(Node):
	def __init__(self) -> None:
		super().__init__("joint_state_name_adapter")

		self.declare_parameter("input_topic", "feedback/joint_states")
		self.declare_parameter("output_topic", "feedback/prefixed_joint_states")
		self.declare_parameter("joint_prefix", "")
		self.declare_parameter("mode", "prepend")

		input_topic = str(self.get_parameter("input_topic").value)
		output_topic = str(self.get_parameter("output_topic").value)
		self.joint_prefix = str(self.get_parameter("joint_prefix").value)
		self.mode = str(self.get_parameter("mode").value)

		if self.mode not in {"prepend", "strip"}:
			raise ValueError("mode must be either 'prepend' or 'strip'")

		self.publisher = self.create_publisher(JointState, output_topic, 20)
		self.create_subscription(JointState, input_topic, self._callback, 20)

	def _callback(self, msg: JointState) -> None:
		adapted = JointState()
		adapted.header = msg.header
		adapted.name = adapt_joint_names(msg.name, self.joint_prefix, self.mode)
		adapted.position = list(msg.position)
		adapted.velocity = list(msg.velocity)
		adapted.effort = list(msg.effort)
		self.publisher.publish(adapted)


def main() -> None:
	rclpy.init()
	node = JointStateNameAdapter()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = ["adapt_joint_name", "adapt_joint_names", "main"]