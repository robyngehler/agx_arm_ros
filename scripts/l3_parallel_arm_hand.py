#!/usr/bin/env python3
"""Same-side arm and hand motion runs in parallel on dedicated buses (L3).

Each device owns its own CAN bus, so arm and hand motion on one side need no
handover window. This measures that: both are commanded together and every bus
is counted while they run.

**This exercises the quarantined development MOVE-J ingress, not the production
stamped path.** It publishes to `<side>_arm/control/move_j`, so the arm driver
must run with `allow_legacy_motion_ingress:=true` — the run refuses to start
otherwise. Evidence from it covers dedicated-bus parallelism; production stop
semantics need the separate MIT/FJT case.

**Arm motion is restricted to joints 3 and 5**, which rotate about the stretched
arm axis without extending it, and to small offsets from a home pose captured
once at the start. The arm returns to that captured pose before every move, so
offsets cannot accumulate — commanding relative to live feedback ratchets the
arm outward a little further each cycle.

Usage:
  python3 scripts/l3_parallel_arm_hand.py --sides right
  python3 scripts/l3_parallel_arm_hand.py --sides left,right --cycles 5
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_msgs.msg import DeviceCommandStamp, HandJointTarget
from agx_arm_msgs.srv import ClaimDevice

# Only these joints may be commanded: they rotate about the stretched arm axis
# without extending the arm.
MOVING_JOINTS = (3, 5)
ARM_IFACE = {"left": "can_nero_left", "right": "can_nero_right"}
HAND_IFACE = {"left": "hand_left", "right": "hand_right"}


def counters(iface: str) -> dict:
    """RX/TX packets, errors and drops for one interface."""
    lines = subprocess.run(
        ["ip", "-s", "link", "show", iface], capture_output=True, text=True
    ).stdout.splitlines()
    out = {}
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith(("RX:", "TX:")):
            key = stripped[:2].lower()
            fields = lines[index + 1].split()
            out[f"{key}_packets"] = int(fields[1])
            out[f"{key}_errors"] = int(fields[2])
            out[f"{key}_dropped"] = int(fields[3])
    return out


def delta(before: dict, after: dict) -> dict:
    return {k: after.get(k, 0) - before.get(k, 0) for k in before}


class ParallelRun(Node):
    def __init__(self, sides: list[str]) -> None:
        super().__init__("l3_parallel_arm_hand")
        self.sides = sides
        self.arm_pose: dict[str, list[float]] = {}
        # Captured once. Every command is home + offset, never live + offset:
        # the live pose already contains the previous offset, so commanding
        # against it walks the arm outward one step per cycle.
        self.home_pose: dict[str, list[float]] = {}
        self.arm_names: dict[str, list[str]] = {}
        self.hand_pose: dict[str, tuple[list[str], list[float]]] = {}
        self.move_pub = {}
        self.hand_pub = {}
        self.claim = {}
        self.epochs: dict[str, tuple[int, int]] = {}
        self.sequence: dict[str, int] = {side: 0 for side in sides}
        self.owner_id = f"reactive:{self.get_name()}"

        for side in sides:
            arm_ns = f"/{side}_arm"
            self.move_pub[side] = self.create_publisher(
                JointState, f"{arm_ns}/control/move_j", 10
            )
            self.hand_pub[side] = self.create_publisher(
                HandJointTarget, f"/{side}_hand/control/omnihand/joint_target", 10
            )
            self.claim[side] = self.create_client(
                ClaimDevice, f"/{side}_hand/control/omnihand/claim_device"
            )
            self.create_subscription(
                JointState, f"{arm_ns}/feedback/joint_states",
                lambda msg, s=side: self._arm_feedback(s, msg), 10,
            )
            self.create_subscription(
                JointState, f"/{side}_hand/feedback/omnihand/joint_states",
                lambda msg, s=side: self._hand_feedback(s, msg), 10,
            )

    def _arm_feedback(self, side: str, msg: JointState) -> None:
        names = [n for n in msg.name if n.startswith(f"{side}_joint")] or list(msg.name)
        if not names:
            return
        index = {n: i for i, n in enumerate(msg.name)}
        self.arm_names[side] = names
        self.arm_pose[side] = [float(msg.position[index[n]]) for n in names]

    def _hand_feedback(self, side: str, msg: JointState) -> None:
        if msg.name:
            self.hand_pose[side] = (list(msg.name), [float(p) for p in msg.position])

    def wait_for_feedback(self, timeout_s: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(s in self.arm_pose for s in self.sides) and all(
                s in self.hand_pose for s in self.sides
            ):
                return True
        missing = [s for s in self.sides if s not in self.arm_pose or s not in self.hand_pose]
        print(f"FAIL  no feedback from: {missing}")
        return False

    def claim_hands(self, timeout_s: float = 10.0) -> bool:
        for side in self.sides:
            if not self.claim[side].wait_for_service(timeout_sec=timeout_s):
                print(f"FAIL  no claim service for {side} hand")
                return False
            request = ClaimDevice.Request()
            request.owner_id = self.owner_id
            request.claim = True
            future = self.claim[side].call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            response = future.result()
            if response is None or not response.accepted:
                print(f"FAIL  {side} hand claim refused: "
                      f"{getattr(response, 'detail', 'no response')}")
                return False
            self.epochs[side] = (response.device_epoch, response.unit_safety_epoch)
            print(f"  claimed {side} hand at device_epoch={response.device_epoch}")
        return True

    def release_hands(self) -> None:
        for side in self.sides:
            request = ClaimDevice.Request()
            request.owner_id = self.owner_id
            request.claim = False
            future = self.claim[side].call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def check_legacy_profile(self) -> bool:
        """Refuse to run unless every arm driver has opened the quarantine.

        Without it the arm silently refuses every command while the buses still
        look busy with feedback, and the run reports a parallelism it never
        exercised.
        """
        import subprocess as _sp

        for side in self.sides:
            node_name = f"/{side}_arm/agx_arm_ctrl_single_node"
            result = _sp.run(
                ["ros2", "param", "get", node_name, "allow_legacy_motion_ingress"],
                capture_output=True, text=True, timeout=20,
            )
            if "True" not in result.stdout:
                print(
                    f"FAIL  {node_name} has allow_legacy_motion_ingress="
                    f"{result.stdout.strip() or 'unreadable'}. This harness "
                    "publishes to the quarantined control/move_j surface, which "
                    "that driver will refuse. Restart it with "
                    "allow_legacy_motion_ingress:=true."
                )
                return False
        return True

    def capture_home(self) -> None:
        """Freeze the starting pose that every command is measured against."""
        for side in self.sides:
            self.home_pose[side] = list(self.arm_pose[side])

    def command_arm(self, side: str, offset: float) -> None:
        """Command home + offset on joints 3 and 5; everything else unchanged."""
        msg = JointState()
        msg.name = list(self.arm_names[side])
        positions = list(self.home_pose[side])
        for joint in MOVING_JOINTS:
            if joint - 1 < len(positions):
                positions[joint - 1] += offset
        msg.position = positions
        self.move_pub[side].publish(msg)

    def command_hand(self, side: str, offset: float) -> None:
        names, positions = self.hand_pose[side]
        device_epoch, unit_epoch = self.epochs[side]
        self.sequence[side] += 1
        stamp = DeviceCommandStamp()
        stamp.owner_id = self.owner_id
        stamp.device_epoch = device_epoch
        stamp.unit_safety_epoch = unit_epoch
        stamp.sequence = self.sequence[side]
        msg = HandJointTarget()
        msg.authority = stamp
        msg.joint_names = list(names)
        msg.positions = [p + offset for p in positions]
        self.hand_pub[side].publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sides", default="right")
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--arm-offset", type=float, default=0.05)
    parser.add_argument("--hand-offset", type=float, default=0.05)
    parser.add_argument("--dwell-s", type=float, default=1.5)
    args = parser.parse_args()

    sides = [s.strip() for s in args.sides.split(",") if s.strip()]
    ifaces = [ARM_IFACE[s] for s in sides] + [HAND_IFACE[s] for s in sides]

    rclpy.init()
    node = ParallelRun(sides)
    try:
        print(f"sides under test: {', '.join(sides)}")
        print("PROFILE: quarantined development MOVE-J ingress "
              "(allow_legacy_motion_ingress:=true required on every arm driver).")
        print("         This is NOT the production stamped arm path; keep its "
              "evidence separate from production stop semantics.")
        if not node.check_legacy_profile():
            return 2
        if not node.wait_for_feedback():
            return 2
        node.capture_home()
        for side in sides:
            print(f"  {side} arm holding {len(node.arm_pose[side])} joints, "
                  f"hand {len(node.hand_pose[side][0])} joints")
        if not node.claim_hands():
            return 2

        before = {i: counters(i) for i in ifaces}
        cpu_before = time.process_time()
        wall_before = time.monotonic()
        for cycle in range(args.cycles):
            # Home on every odd cycle, so each move starts from the captured
            # pose rather than from wherever the last one ended.
            offset = args.arm_offset if cycle % 2 == 0 else 0.0
            hand_offset = args.hand_offset if cycle % 2 == 0 else 0.0
            # Both sides, both devices, in the same instant.
            for side in sides:
                node.command_arm(side, offset)
                node.command_hand(side, hand_offset)
            end = time.monotonic() + args.dwell_s
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            print(f"  cycle {cycle + 1}/{args.cycles} "
                  f"(arm offset {offset:+.3f} rad on joints {MOVING_JOINTS})")

        # Return every arm to the pose it started in.
        for side in sides:
            node.command_arm(side, 0.0)
            node.command_hand(side, 0.0)
        end = time.monotonic() + args.dwell_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        elapsed = time.monotonic() - wall_before
        cpu = time.process_time() - cpu_before
        after = {i: counters(i) for i in ifaces}
        node.release_hands()

        print(f"\nwindow: {elapsed:.1f} s, harness CPU {cpu:.2f} s")
        print(f"{'interface':<18} {'RX/s':>9} {'TX/s':>9} "
              f"{'rx_err':>7} {'tx_err':>7} {'rx_drop':>8} {'tx_drop':>8}")
        failed = False
        for iface in ifaces:
            d = delta(before[iface], after[iface])
            print(f"{iface:<18} {d['rx_packets'] / elapsed:9.1f} "
                  f"{d['tx_packets'] / elapsed:9.1f} "
                  f"{d['rx_errors']:7d} {d['tx_errors']:7d} "
                  f"{d['rx_dropped']:8d} {d['tx_dropped']:8d}")
            if d["rx_errors"] or d["tx_errors"] or d["tx_dropped"]:
                failed = True
            if d["tx_packets"] == 0:
                print(f"  FAIL {iface} transmitted nothing")
                failed = True

        print()
        print(f"{'FAIL' if failed else 'OK  '} every bus carried traffic with "
              "no errors or drops")
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
