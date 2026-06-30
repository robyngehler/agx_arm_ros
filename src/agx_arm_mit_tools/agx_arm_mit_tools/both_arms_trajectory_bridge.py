"""Fan-out FollowJointTrajectory bridge for the ``both_arms`` group.

The Sprint-6 coordinator builds one combined ``both_arms`` FollowJointTrajectory
goal (left joints then right joints) and sends it to
``both_arms_controller/follow_joint_trajectory`` (see
``agx_arm_coordination/config/arm_config.yaml``). On the Duo the two arms live on
separate CAN side buses behind separate per-arm controllers, so there is no single
controller that owns all 14 joints. This bridge is that missing seam: it exposes
the combined action server, **splits** the incoming trajectory by joint prefix
into a left and a right sub-trajectory, and **forwards** each as its own
``FollowJointTrajectory`` goal to the per-arm action servers, then aggregates the
two results into one.

It composes the existing per-arm FollowJointTrajectory path (each side is the MIT
controller's own action server, or the ``agx_arm_mit_tools`` FJT->MIT adapter)
rather than forking arm execution — so ``Trajectory+both_arms`` runs through the
same controllers as the per-arm groups. The default downstream names match the
``left_arm`` / ``right_arm`` groups in ``arm_config.yaml``:

    both_arms_controller/follow_joint_trajectory   (this server)
      -> left_arm_controller/follow_joint_trajectory
      -> right_arm_controller/follow_joint_trajectory

Success requires both sides to succeed; any side failing (or a cancel) aborts and
cancels the other side. Joints are routed purely by prefix, so the combined goal
may interleave joint order freely as long as every name starts with the left or
right prefix.
"""

from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _split_indices(joint_names: list[str], prefix: str) -> list[int]:
    return [index for index, name in enumerate(joint_names) if name.startswith(prefix)]


def _subset_point(point: JointTrajectoryPoint, indices: list[int]) -> JointTrajectoryPoint:
    out = JointTrajectoryPoint()
    out.time_from_start = point.time_from_start
    if point.positions:
        out.positions = [point.positions[i] for i in indices]
    if point.velocities:
        out.velocities = [point.velocities[i] for i in indices]
    if point.accelerations:
        out.accelerations = [point.accelerations[i] for i in indices]
    if point.effort:
        out.effort = [point.effort[i] for i in indices]
    return out


def split_trajectory(trajectory: JointTrajectory, indices: list[int], names: list[str]) -> JointTrajectory:
    sub = JointTrajectory()
    sub.header = trajectory.header
    sub.joint_names = names
    sub.points = [_subset_point(point, indices) for point in trajectory.points]
    return sub


class _SideGoal:
    """Tracks one downstream per-arm FollowJointTrajectory goal."""

    def __init__(self, side: str) -> None:
        self.side = side
        self.done = False
        self.success = False
        self.message = ""
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

    def attach(self, goal_future) -> None:
        self._goal_future = goal_future

    def mark(self, success: bool, message: str) -> None:
        self.done = True
        self.success = success
        self.message = message

    def poll(self) -> None:
        if self.done:
            return
        if self._goal_future is not None and self._result_future is None:
            if not self._goal_future.done():
                return
            self._goal_handle = self._goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                self.mark(False, f"{self.side}: goal rejected by per-arm controller")
                return
            self._result_future = self._goal_handle.get_result_async()
            return
        if self._result_future is not None and self._result_future.done():
            result = self._result_future.result().result
            ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            self.mark(ok, result.error_string or f"{self.side}: error_code={result.error_code}")

    def cancel(self) -> None:
        if self._goal_handle is not None and not self.done:
            self._goal_handle.cancel_goal_async()


class BothArmsTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("both_arms_follow_joint_trajectory")

        self.declare_parameter("action_name", "both_arms_controller/follow_joint_trajectory")
        self.declare_parameter("left_action_name", "left_arm_controller/follow_joint_trajectory")
        self.declare_parameter("right_action_name", "right_arm_controller/follow_joint_trajectory")
        self.declare_parameter("left_joint_prefix", "left_arm_")
        self.declare_parameter("right_joint_prefix", "right_arm_")
        self.declare_parameter("server_wait_timeout_s", 5.0)
        self.declare_parameter("poll_period_s", 0.05)

        action_name = str(self.get_parameter("action_name").value)
        self.left_prefix = str(self.get_parameter("left_joint_prefix").value)
        self.right_prefix = str(self.get_parameter("right_joint_prefix").value)
        self.server_wait_timeout_s = float(self.get_parameter("server_wait_timeout_s").value)
        self.poll_period_s = float(self.get_parameter("poll_period_s").value)

        self._cb_group = ReentrantCallbackGroup()
        self.left_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter("left_action_name").value),
            callback_group=self._cb_group,
        )
        self.right_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter("right_action_name").value),
            callback_group=self._cb_group,
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"both_arms FJT bridge up: server={action_name}, "
            f"left<-{self.get_parameter('left_action_name').value}, "
            f"right<-{self.get_parameter('right_action_name').value}"
        )

    # --- goal validation -----------------------------------------------------

    def _route_indices(self, joint_names: list[str]) -> tuple[list[int], list[int], list[str]]:
        left = _split_indices(joint_names, self.left_prefix)
        right = _split_indices(joint_names, self.right_prefix)
        routed = set(left) | set(right)
        unknown = [joint_names[i] for i in range(len(joint_names)) if i not in routed]
        return left, right, unknown

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        names = list(goal_request.trajectory.joint_names)
        if not names:
            self.get_logger().error("rejected both_arms goal: trajectory has no joint_names")
            return GoalResponse.REJECT
        left, right, unknown = self._route_indices(names)
        if unknown:
            self.get_logger().error(
                f"rejected both_arms goal: joints match neither '{self.left_prefix}' nor "
                f"'{self.right_prefix}': {unknown}"
            )
            return GoalResponse.REJECT
        if not left or not right:
            self.get_logger().error(
                "rejected both_arms goal: expected both a left and a right joint set "
                f"(left={len(left)}, right={len(right)})"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    # --- execution -----------------------------------------------------------

    def _fail(self, goal_handle, message: str) -> FollowJointTrajectory.Result:
        self.get_logger().error(message)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = message
        goal_handle.abort()
        return result

    def _execute(self, goal_handle) -> FollowJointTrajectory.Result:
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        left_idx, right_idx, _ = self._route_indices(names)
        left_names = [names[i] for i in left_idx]
        right_names = [names[i] for i in right_idx]

        if not self.left_client.wait_for_server(timeout_sec=self.server_wait_timeout_s):
            return self._fail(goal_handle, "left per-arm controller not available")
        if not self.right_client.wait_for_server(timeout_sec=self.server_wait_timeout_s):
            return self._fail(goal_handle, "right per-arm controller not available")

        left_goal = FollowJointTrajectory.Goal()
        left_goal.trajectory = split_trajectory(trajectory, left_idx, left_names)
        right_goal = FollowJointTrajectory.Goal()
        right_goal.trajectory = split_trajectory(trajectory, right_idx, right_names)

        left = _SideGoal("left")
        right = _SideGoal("right")
        left.attach(self.left_client.send_goal_async(left_goal))
        right.attach(self.right_client.send_goal_async(right_goal))
        self.get_logger().info(
            f"dispatched both_arms split: left={len(left_names)}j, right={len(right_names)}j, "
            f"{len(trajectory.points)} point(s)"
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                left.cancel()
                right.cancel()
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "both_arms goal canceled"
                return result

            left.poll()
            right.poll()

            # Abort fast: if one side fails, cancel the other and abort.
            for failed, other in ((left, right), (right, left)):
                if failed.done and not failed.success:
                    other.cancel()
                    return self._fail(
                        goal_handle, f"both_arms aborted: {failed.message}"
                    )

            if left.done and right.done and left.success and right.success:
                goal_handle.succeed()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            time.sleep(self.poll_period_s)

        return self._fail(goal_handle, "ROS shutdown while executing both_arms trajectory")


def main() -> None:
    rclpy.init()
    node = BothArmsTrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["main", "BothArmsTrajectoryBridge", "split_trajectory"]
