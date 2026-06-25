#!/usr/bin/env python3
"""Solo exerciser for the OmniHand ROS bridge.

The bridge launch (``start_omnihand_bridge.launch.py``) brings the hand up on
its own, but it only *listens*; there was no convenient way to drive its
ROS-wrapped surface for grasp/skill bring-up. This node publishes named hand
poses to the bridge command topic and can call the safe-stop service, so the
full ROS path (command topic -> bridge -> SDK -> hand, feedback back out) can
be exercised solo.

Typical solo workflow:

    # 1) bring up the bridge against the real hand
    ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
        backend_type:=sdk omnihand_type:=right

    # 2) watch feedback
    ros2 topic echo /feedback/omnihand/joint_states

    # 3) drive a pose (one-shot); --model MUST match the bridge's hand_model
    ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --side right --gesture zero

    # list the poses defined for a model
    ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --list

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
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, HAND_MODELS, get_hand_model
from agx_arm_ctrl.omnihand_bridge_node import (
    build_joint_names,
    load_gesture_presets,
    resolve_gesture_presets,
)


# Gesture NAMES are model-specific (the o12_pro presets differ from the legacy
# O10 set), so they are resolved per --model at runtime rather than baked into the
# argparse choices. The per-side joint VALUES are likewise resolved at runtime
# (right = canonical, left = mirrored) via resolve_gesture_presets(side, model).
def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drive the OmniHand ROS bridge with named poses.",
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
        help="Single pose to publish (see --list for the selected model's poses).",
    )
    parser.add_argument(
        "--sequence",
        help="Comma-separated poses to cycle, e.g. open,fist,open.",
    )
    parser.add_argument(
        "--topic",
        default="control/joint_states",
        help="Command topic the bridge subscribes to.",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=2.0,
        help="Seconds to hold each pose (republished at --rate) before the next.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate while holding a pose, in Hz.",
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
        self.publisher = self.create_publisher(JointState, args.topic, 10)
        self.get_logger().info(
            f"Exercising model={self.model.name} side={args.side} on topic "
            f"'{args.topic}' ({len(self.joint_names)} joints)"
        )

    def _publish_pose(self, name: str, positions: list[float]) -> None:
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"pose '{name}' has {len(positions)} values, "
                f"expected {len(self.joint_names)}"
            )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in positions]
        self.publisher.publish(msg)

    def hold_pose(self, name: str) -> None:
        positions = self.gestures[name]
        period = 1.0 / self.args.rate if self.args.rate > 0 else 0.05
        deadline = self.get_clock().now().nanoseconds + int(self.args.hold_s * 1e9)
        self.get_logger().info(f"-> {name}")
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            self._publish_pose(name, positions)
            rclpy.spin_once(self, timeout_sec=period)

    def call_stop(self) -> None:
        client = self.create_client(Trigger, "control/omnihand/stop")
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("control/omnihand/stop service unavailable")
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
            node.hold_pose(name)
        if args.stop:
            node.call_stop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
