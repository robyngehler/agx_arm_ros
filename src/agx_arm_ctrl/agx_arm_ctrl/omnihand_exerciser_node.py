#!/usr/bin/env python3
"""Solo exerciser for the OmniHand ROS bridge.

The bridge launch (``start_omnihand_bridge.launch.py``) brings the hand up on
its own, but it only *listens*; there was no convenient way to drive its
ROS-wrapped surface for grasp/skill bring-up. This node drives the hand over
the same path MoveIt uses — the per-side ``FollowJointTrajectory`` action —
and can call the safe-stop service.

Command path (default): FollowJointTrajectory goal ->
``/<side_namespace>/<side>_omnihand_controller/follow_joint_trajectory`` ->
omnihand_follow_joint_trajectory bridge -> ``control/omnihand/joint_trajectory``
-> omnihand_bridge (retry until readback verifies) -> SDK -> hand. The side
namespace (for example ``left_arm``) is resolved from the duo motion registry,
so the exerciser reaches the bridge inside the Duo bringup namespaces where a
bare root-namespace publish never arrives. If the action server is not up
(standalone bridge without the trajectory node), the exerciser falls back to
publishing the JointTrajectory directly on the bridge's namespaced
``control/omnihand/joint_trajectory`` topic.

Typical workflow against the Duo bringup:

    # 1) watch feedback
    ros2 topic echo /left_arm/feedback/omnihand/joint_states

    # 2) drive a pose over the MoveIt path; --model MUST match the bridge
    ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --side left --gesture zero

    # list the poses defined for a model
    ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --list

Against a standalone (root-namespace) bridge from
``start_omnihand_bridge.launch.py``, pass ``--namespace ''`` so the exerciser
does not target the registry's Duo side namespace.

Legacy mode: pass ``--topic <topic>`` to publish JointState commands on the
shared command topic instead (for example ``/left_arm/control/joint_states``);
the topic must then already include the side namespace.

The poses are model-specific and come from the single source of truth for that
model: ``config/omnihand_pro_gestures.yaml`` for ``o12_pro`` (12 joints, derived
from the Pro ``demo_set_angle.py``) and ``config/omnihand_gestures.yaml`` for the
legacy ``o10`` (10 joints). ``--model`` MUST match the running bridge's
``hand_model`` or the published joint names will not line up. They are calibrated
for the right hand; the bridge clamps every target to the selected side's joint
limits, so a right-tuned preset on a left hand is safe but may not look identical.
``zero`` is the all-zeros motor reference (safe bring-up pose). NOTE: the o12_pro
set is still vendor-demo bootstrap (``zero``, ``fist_vendor_demo``); calibrated
``open``/grasp poses must be measured on the Pro hardware before use.
"""

from __future__ import annotations

import argparse

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agx_arm_ctrl.motion_registry import arm_sides
from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, HAND_MODELS, get_hand_model
from agx_arm_ctrl.omnihand_bridge_node import (
    build_joint_names,
    load_gesture_presets,
    resolve_gesture_presets,
)


def resolve_side_namespace(hand_side: str) -> str:
    """Return the side's runtime namespace (e.g. ``left_arm``) from the registry.

    Empty when the registry is unreadable or the side is unknown, which matches
    a standalone (unnamespaced) bridge bringup.
    """
    try:
        return str(arm_sides().get(hand_side, {}).get("namespace", "")).strip("/")
    except Exception:
        return ""


def _absolute_name(namespace: str, relative_name: str) -> str:
    if namespace:
        return f"/{namespace}/{relative_name}"
    return f"/{relative_name}"


# Gesture NAMES are model-specific (the o12_pro presets differ from the legacy
# O10 set), so they are resolved per --model at runtime rather than baked into the
# argparse choices. The per-side joint VALUES are likewise resolved at runtime
# (right = canonical, left = mirrored) via resolve_gesture_presets(side, model).
def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drive the OmniHand over the MoveIt FollowJointTrajectory path.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--model",
        choices=sorted(HAND_MODELS),
        default=DEFAULT_HAND_MODEL,
        help=(
            "OmniHand model whose joint layout and presets to use; must match the "
            f"running bridge's hand_model (default: {DEFAULT_HAND_MODEL})."
        ),
    )
    parser.add_argument(
        "--gesture",
        help="Single pose to send (see --list for the selected model's poses).",
    )
    parser.add_argument(
        "--sequence",
        help="Comma-separated poses to cycle, e.g. open,fist,open.",
    )
    parser.add_argument(
        "--namespace",
        default=None,
        help=(
            "Side namespace of the running bridge (e.g. left_arm). Default: "
            "auto-resolved for --side from the duo motion registry."
        ),
    )
    parser.add_argument(
        "--topic",
        default=None,
        help=(
            "Legacy mode: publish JointState commands on this exact topic (e.g. "
            "/left_arm/control/joint_states) instead of the FollowJointTrajectory "
            "action path. The topic must already include the side namespace."
        ),
    )
    parser.add_argument(
        "--move-s",
        type=float,
        default=1.0,
        help="Trajectory duration sent with each pose, in seconds.",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=2.0,
        help="Seconds to wait on each pose before sending the next.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate while holding a pose in legacy --topic mode, in Hz.",
    )
    parser.add_argument(
        "--list", action="store_true", help="List available poses and exit."
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Call control/omnihand/stop after the poses (or immediately if no pose).",
    )
    return parser.parse_args(argv)


class OmniHandExerciser(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("omnihand_exerciser")
        self.args = args
        self.model = get_hand_model(args.model)
        self.joint_names = build_joint_names(args.side, self.model)
        self.gestures = resolve_gesture_presets(args.side, self.model)
        self.namespace = (
            args.namespace.strip("/")
            if args.namespace is not None
            else resolve_side_namespace(args.side)
        )
        self.action_name = _absolute_name(
            self.namespace, f"{args.side}_omnihand_controller/follow_joint_trajectory"
        )
        self.trajectory_topic = _absolute_name(
            self.namespace, "control/omnihand/joint_trajectory"
        )
        self.stop_service = _absolute_name(self.namespace, "control/omnihand/stop")

        self.legacy_publisher = None
        self.trajectory_publisher = None
        self.action_client: ActionClient | None = None
        if args.topic is not None:
            self.legacy_publisher = self.create_publisher(JointState, args.topic, 10)
            self.get_logger().info(
                f"Exercising model={self.model.name} side={args.side} in legacy "
                f"JointState mode on topic '{args.topic}' ({len(self.joint_names)} joints)"
            )
        else:
            self.action_client = ActionClient(
                self, FollowJointTrajectory, self.action_name
            )
            self.get_logger().info(
                f"Exercising model={self.model.name} side={args.side} via action "
                f"'{self.action_name}' ({len(self.joint_names)} joints, "
                f"namespace='{self.namespace or '<root>'}')"
            )

    def _build_trajectory(self, name: str, positions: list[float]) -> JointTrajectory:
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"pose '{name}' has {len(positions)} values, "
                f"expected {len(self.joint_names)}"
            )
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        move_s = max(0.0, float(self.args.move_s))
        point.time_from_start = DurationMsg(
            sec=int(move_s), nanosec=int((move_s % 1.0) * 1e9)
        )
        trajectory.points = [point]
        return trajectory

    def send_pose(self, name: str) -> None:
        positions = self.gestures[name]
        self.get_logger().info(f"-> {name}")
        if self.legacy_publisher is not None:
            self._publish_legacy_pose(name, positions)
        elif not self._send_action_goal(name, positions):
            self._publish_trajectory_fallback(name, positions)
        self._sleep(self.args.hold_s)

    def _send_action_goal(self, name: str, positions: list[float]) -> bool:
        """Send the pose as a FollowJointTrajectory goal; True when handled."""
        assert self.action_client is not None
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f"action server '{self.action_name}' unavailable; falling back to "
                f"topic '{self.trajectory_topic}'"
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_trajectory(name, positions)

        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"pose '{name}' goal was rejected")
            return True

        result_timeout = float(self.args.move_s) + 10.0
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        response = result_future.result()
        if response is None:
            self.get_logger().warn(f"pose '{name}' result timed out")
            return True
        result = response.result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"pose '{name}' delivered")
        else:
            self.get_logger().warn(
                f"pose '{name}' finished with error_code={result.error_code} "
                f"'{result.error_string}'"
            )
        return True

    def _publish_trajectory_fallback(self, name: str, positions: list[float]) -> None:
        if self.trajectory_publisher is None:
            self.trajectory_publisher = self.create_publisher(
                JointTrajectory, self.trajectory_topic, 10
            )
            # Give DDS discovery a moment so the first publish is not dropped.
            self._sleep(0.5)
        self.trajectory_publisher.publish(self._build_trajectory(name, positions))

    def _publish_legacy_pose(self, name: str, positions: list[float]) -> None:
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"pose '{name}' has {len(positions)} values, "
                f"expected {len(self.joint_names)}"
            )
        msg = JointState()
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in positions]
        period = 1.0 / self.args.rate if self.args.rate > 0 else 0.05
        deadline = self.get_clock().now().nanoseconds + int(self.args.hold_s * 1e9)
        assert self.legacy_publisher is not None
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.legacy_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)

    def _sleep(self, duration_s: float) -> None:
        deadline = self.get_clock().now().nanoseconds + int(duration_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call_stop(self) -> None:
        client = self.create_client(Trigger, self.stop_service)
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"{self.stop_service} service unavailable")
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is not None:
            self.get_logger().info(f"stop: success={result.success} '{result.message}'")
        else:
            self.get_logger().warn("stop service call did not return")


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)

    # Gesture names are model-specific; resolve them for the selected --model.
    model = get_hand_model(args.model)
    gesture_names = sorted(load_gesture_presets(model))

    if args.list:
        print(f"Available OmniHand poses for model {model.name}:")
        for name in gesture_names:
            print(f"  {name}")
        return

    poses: list[str] = []
    if args.sequence:
        poses = [name.strip() for name in args.sequence.split(",") if name.strip()]
        unknown = [name for name in poses if name not in gesture_names]
        if unknown:
            raise SystemExit(
                f"unknown pose(s) for model {model.name}: {', '.join(unknown)}; "
                f"available: {', '.join(gesture_names)}"
            )
    elif args.gesture:
        if args.gesture not in gesture_names:
            raise SystemExit(
                f"unknown pose '{args.gesture}' for model {model.name}; "
                f"available: {', '.join(gesture_names)}"
            )
        poses = [args.gesture]
    elif not args.stop:
        raise SystemExit("nothing to do: pass --gesture, --sequence, --stop, or --list")

    rclpy.init()
    try:
        node = OmniHandExerciser(args)
        for name in poses:
            node.send_pose(name)
        if args.stop:
            node.call_stop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
