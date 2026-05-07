from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .trajectory_io import load_recorded_trajectory, recorded_to_joint_trajectory


class SavedTrajectoryExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("saved_trajectory_executor")
        self.publisher = self.create_publisher(JointTrajectory, "mit_controller/joint_trajectory", 10)
        self.enable_client = self.create_client(SetBool, "mit_controller/enable")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Load and execute a saved MIT trajectory recording")
    parser.add_argument("trajectory_path", help="Path to a saved recording JSON file")
    parser.add_argument("--service-timeout", type=float, default=5.0, help="Timeout for MIT enable service")
    parser.add_argument("--publish-repetitions", type=int, default=3, help="How often to republish the trajectory")
    parser.add_argument("--publish-interval", type=float, default=0.2, help="Seconds between trajectory publishes")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trajectory = load_recorded_trajectory(Path(args.trajectory_path))
    joint_trajectory = recorded_to_joint_trajectory(trajectory)

    rclpy.init()
    node = SavedTrajectoryExecutorNode()
    try:
        if not node.enable_client.wait_for_service(timeout_sec=args.service_timeout):
            raise RuntimeError("MIT controller enable service is not available")

        request = SetBool.Request()
        request.data = True
        future = node.enable_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.service_timeout)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError("Failed to enable MIT controller before publishing trajectory")

        for _ in range(max(1, args.publish_repetitions)):
            node.publisher.publish(joint_trajectory)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, args.publish_interval))

        print(
            f"Published saved trajectory '{trajectory.name}' with {len(trajectory.points)} points "
            f"and {trajectory.duration:.2f}s duration"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()