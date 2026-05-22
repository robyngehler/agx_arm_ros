#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply a simple box-obstacle planning scene for the current MoveIt workspace."
    )
    parser.add_argument("--config", required=True, help="Path to the JSON obstacle config.")
    parser.add_argument(
        "--namespace",
        default="",
        help="Optional ROS namespace for the target MoveIt instance.",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the apply_planning_scene service.",
    )
    return parser.parse_args()


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main() -> int:
    args = parse_args()

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from rclpy.node import Node
    from shape_msgs.msg import SolidPrimitive

    payload = json.loads(Path(args.config).expanduser().resolve().read_text(encoding="utf-8"))
    frame_id = str(payload.get("frame_id", "world"))
    obstacles = list(payload.get("obstacles", []))

    if not obstacles:
        raise RuntimeError(f"No obstacles were defined in {args.config}")

    rclpy.init()
    node = Node("apply_simple_obstacles")
    try:
        namespace = args.namespace.strip("/")
        service_name = f"/{namespace}/apply_planning_scene" if namespace else "/apply_planning_scene"
        client = node.create_client(ApplyPlanningScene, service_name)

        if not client.wait_for_service(timeout_sec=args.service_timeout):
            raise RuntimeError(f"Service {service_name} was not available within {args.service_timeout:.1f}s")

        scene = PlanningScene()
        scene.is_diff = True
        for obstacle in obstacles:
            if obstacle.get("type") != "box":
                raise RuntimeError(
                    f"Obstacle '{obstacle.get('id', '<unknown>')}' uses unsupported type {obstacle.get('type')}"
                )

            dimensions = [float(value) for value in obstacle["dimensions"]]
            position = [float(value) for value in obstacle["position"]]
            rpy = [float(value) for value in obstacle.get("rpy", [0.0, 0.0, 0.0])]
            qx, qy, qz, qw = quaternion_from_rpy(rpy[0], rpy[1], rpy[2])

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions

            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            collision = CollisionObject()
            collision.id = str(obstacle["id"])
            collision.header.frame_id = str(obstacle.get("frame_id", frame_id))
            collision.primitives = [primitive]
            collision.primitive_poses = [pose]
            collision.operation = CollisionObject.ADD
            scene.world.collision_objects.append(collision)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.service_timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("Timed out while applying the planning scene")
        if not future.result().success:
            raise RuntimeError("MoveIt rejected the simple obstacle planning scene")

        node.get_logger().info(f"Applied {len(scene.world.collision_objects)} simple planning obstacles")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())