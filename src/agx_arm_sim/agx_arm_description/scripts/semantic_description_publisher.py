#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SemanticDescriptionPublisher(Node):
    def __init__(self, topic: str, file_path: str) -> None:
        super().__init__('semantic_description_publisher')
        content = Path(file_path).read_text(encoding='utf-8')
        self.message = String(data=content)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(String, topic, qos)
        self.publisher.publish(self.message)
        self.create_timer(1.0, self._publish)

    def _publish(self) -> None:
        self.publisher.publish(self.message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='robot_description_semantic')
    parser.add_argument('--file-path', required=True)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SemanticDescriptionPublisher(args.topic, args.file_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
