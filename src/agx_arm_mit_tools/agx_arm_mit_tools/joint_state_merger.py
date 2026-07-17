from __future__ import annotations

from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .joint_state_name_adapter import adapt_joint_names


def _stamp_ns(message: JointState) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _merged_joint_state(messages: Sequence[JointState | None], joint_prefixes: Sequence[str]) -> JointState:
    merged = JointState()
    for message, joint_prefix in zip(messages, joint_prefixes):
        if message is None:
            continue
        # Newest source stamp wins; taking the last source's header instead
        # would freeze the whole merged stream on one stalled source.
        if _stamp_ns(message) >= _stamp_ns(merged):
            merged.header = message.header
        merged.name.extend(adapt_joint_names(message.name, joint_prefix, "prepend"))
        merged.position.extend(list(message.position))
        merged.velocity.extend(list(message.velocity))
        merged.effort.extend(list(message.effort))
    return merged


class JointStateMerger(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_merger")

        self.declare_parameter("source_topics", ["feedback/joint_states"])
        self.declare_parameter("joint_prefixes", [""])
        self.declare_parameter("output_topic", "feedback/prefixed_joint_states")

        source_topics = [str(value) for value in self.get_parameter("source_topics").value]
        self.joint_prefixes = [str(value) for value in self.get_parameter("joint_prefixes").value]
        output_topic = str(self.get_parameter("output_topic").value)

        if not source_topics:
            raise ValueError("source_topics must not be empty")
        if len(source_topics) != len(self.joint_prefixes):
            raise ValueError("source_topics and joint_prefixes must have the same length")

        self.publisher = self.create_publisher(JointState, output_topic, 20)
        self.latest_messages: list[JointState | None] = [None] * len(source_topics)

        for index, source_topic in enumerate(source_topics):
            self.create_subscription(
                JointState,
                source_topic,
                lambda msg, source_index=index: self._callback(source_index, msg),
                20,
            )

    def _callback(self, source_index: int, msg: JointState) -> None:
        self.latest_messages[source_index] = msg
        self.publisher.publish(_merged_joint_state(self.latest_messages, self.joint_prefixes))


def main() -> None:
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["JointStateMerger", "main"]
