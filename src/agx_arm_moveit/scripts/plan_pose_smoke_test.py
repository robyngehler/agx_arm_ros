#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass


HOME_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

HOME_JOINT_POSITIONS = [0.0] * len(HOME_JOINT_NAMES)


@dataclass(frozen=True)
class OffsetCandidate:
    label: str
    xyz: tuple[float, float, float]


DEFAULT_CANDIDATES = [
    OffsetCandidate("x_plus_3cm", (0.03, 0.0, 0.0)),
    OffsetCandidate("z_plus_3cm", (0.0, 0.0, 0.03)),
    OffsetCandidate("y_plus_3cm", (0.0, 0.03, 0.0)),
    OffsetCandidate("x_minus_3cm", (-0.03, 0.0, 0.0)),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a representative near-home MoveIt pose-planning smoke test by computing FK for the "
            "home joint state and then requesting OMPL plans to small offset targets."
        )
    )
    parser.add_argument(
        "--namespace",
        default="",
        help="Optional ROS namespace for the target MoveIt instance.",
    )
    parser.add_argument(
        "--group-name",
        default="nero_arm",
        help="MoveIt planning group to test.",
    )
    parser.add_argument(
        "--tip-link",
        default="tcp_link",
        help="End-effector link used for FK and pose constraints.",
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="Constraint frame for the pose goal.",
    )
    parser.add_argument(
        "--pipeline-id",
        default="ompl",
        help="Planning pipeline ID to request.",
    )
    parser.add_argument(
        "--allowed-planning-time",
        type=float,
        default=5.0,
        help="Allowed planning time in seconds.",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the FK and planning services.",
    )
    parser.add_argument(
        "--position-radius",
        type=float,
        default=0.005,
        help="Radius in meters for the spherical position constraint region.",
    )
    parser.add_argument(
        "--orientation-tolerance",
        type=float,
        default=0.10,
        help="Absolute tolerance in radians for each orientation axis.",
    )
    return parser.parse_args()


def _qualify(namespace: str, name: str) -> str:
    ns = namespace.strip("/")
    return f"/{ns}/{name}" if ns else f"/{name}"


def main() -> int:
    args = parse_args()

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint, RobotState
    from moveit_msgs.srv import GetMotionPlan, GetPositionFK
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive

    def wait_for_result(node: Node, future, timeout_s: float, description: str):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Timed out while waiting for {description}")
        return future.result()

    def make_home_state() -> RobotState:
        state = RobotState()
        joint_state = JointState()
        joint_state.name = list(HOME_JOINT_NAMES)
        joint_state.position = list(HOME_JOINT_POSITIONS)
        joint_state.velocity = [0.0] * len(HOME_JOINT_NAMES)
        joint_state.effort = [0.0] * len(HOME_JOINT_NAMES)
        state.joint_state = joint_state
        return state

    def build_pose_constraints(target_pose: Pose) -> Constraints:
        constraints = Constraints()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(args.position_radius)]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = args.base_frame
        position_constraint.link_name = args.tip_link
        position_constraint.constraint_region.primitives = [sphere]
        position_constraint.constraint_region.primitive_poses = [target_pose]
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = args.base_frame
        orientation_constraint.link_name = args.tip_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = float(args.orientation_tolerance)
        orientation_constraint.absolute_y_axis_tolerance = float(args.orientation_tolerance)
        orientation_constraint.absolute_z_axis_tolerance = float(args.orientation_tolerance)
        orientation_constraint.weight = 1.0

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    rclpy.init()
    node = Node("moveit_pose_plan_smoke_test")
    try:
        fk_service = _qualify(args.namespace, "compute_fk")
        plan_service = _qualify(args.namespace, "plan_kinematic_path")

        fk_client = node.create_client(GetPositionFK, fk_service)
        plan_client = node.create_client(GetMotionPlan, plan_service)

        if not fk_client.wait_for_service(timeout_sec=args.service_timeout):
            raise RuntimeError(f"Service {fk_service} was not available within {args.service_timeout:.1f}s")
        if not plan_client.wait_for_service(timeout_sec=args.service_timeout):
            raise RuntimeError(f"Service {plan_service} was not available within {args.service_timeout:.1f}s")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = args.base_frame
        fk_request.fk_link_names = [args.tip_link]
        fk_request.robot_state = make_home_state()
        fk_response = wait_for_result(
            node,
            fk_client.call_async(fk_request),
            args.service_timeout,
            f"FK response from {fk_service}",
        )

        if fk_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"FK failed with MoveIt error code {fk_response.error_code.val} for {args.tip_link}"
            )
        if not fk_response.pose_stamped:
            raise RuntimeError(f"FK returned no poses for {args.tip_link}")

        home_pose = fk_response.pose_stamped[0].pose
        node.get_logger().info(
            "Home FK pose: "
            f"x={home_pose.position.x:.4f}, y={home_pose.position.y:.4f}, z={home_pose.position.z:.4f}"
        )

        last_error = None
        for candidate in DEFAULT_CANDIDATES:
            target_pose = Pose()
            target_pose.position.x = home_pose.position.x + candidate.xyz[0]
            target_pose.position.y = home_pose.position.y + candidate.xyz[1]
            target_pose.position.z = home_pose.position.z + candidate.xyz[2]
            target_pose.orientation = home_pose.orientation

            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = args.group_name
            request.motion_plan_request.pipeline_id = args.pipeline_id
            request.motion_plan_request.num_planning_attempts = 1
            request.motion_plan_request.allowed_planning_time = float(args.allowed_planning_time)
            request.motion_plan_request.max_velocity_scaling_factor = 0.1
            request.motion_plan_request.max_acceleration_scaling_factor = 0.1
            request.motion_plan_request.start_state = make_home_state()
            request.motion_plan_request.goal_constraints = [build_pose_constraints(target_pose)]
            request.motion_plan_request.workspace_parameters.header.frame_id = args.base_frame
            request.motion_plan_request.workspace_parameters.min_corner.x = -1.5
            request.motion_plan_request.workspace_parameters.min_corner.y = -1.5
            request.motion_plan_request.workspace_parameters.min_corner.z = -1.5
            request.motion_plan_request.workspace_parameters.max_corner.x = 1.5
            request.motion_plan_request.workspace_parameters.max_corner.y = 1.5
            request.motion_plan_request.workspace_parameters.max_corner.z = 1.5

            response = wait_for_result(
                node,
                plan_client.call_async(request),
                args.service_timeout,
                f"motion plan response from {plan_service}",
            )
            error_code = response.motion_plan_response.error_code.val
            point_count = len(response.motion_plan_response.trajectory.joint_trajectory.points)

            if error_code == MoveItErrorCodes.SUCCESS and point_count > 0:
                node.get_logger().info(
                    "Representative pose plan succeeded: "
                    f"candidate={candidate.label}, points={point_count}, pipeline={args.pipeline_id}"
                )
                return 0

            last_error = (candidate.label, error_code, point_count)
            node.get_logger().warn(
                "Pose plan candidate failed: "
                f"candidate={candidate.label}, error_code={error_code}, points={point_count}"
            )

        raise RuntimeError(
            "No representative pose plan candidate succeeded"
            if last_error is None
            else (
                "No representative pose plan candidate succeeded; last result="
                f"candidate={last_error[0]}, error_code={last_error[1]}, points={last_error[2]}"
            )
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())