#!/usr/bin/env python3
"""Stale and foreign hand commands are refused on real hardware (L3, 4D).

Until the command stamp landed, the bridge built a command's identity from its
own current epoch and its own counter, then checked that identity against the
state it came from. The stale-epoch and out-of-order checks therefore compared
each value with itself and passed unconditionally — the code ran and could never
refuse anything. That was provable at L1 only; this is the same claim on a real
hand, on a real bus.

**This run does not move the hand.** Every command it sends carries the hand's
own measured pose as its target, so admission is exercised end to end while the
commanded position is the position the hand is already in. What is under test is
which commands reach the hardware, not what they do when they get there.

Usage:
  python3 scripts/l3_hand_command_authority.py [--side right] [--iface hand_right]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

BRIDGE = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_bridge"


class Step:
    def __init__(self, name: str) -> None:
        self.name = name
        self.passed: bool | None = None
        self.detail = ""

    def settle(self, passed: bool, detail: str) -> "Step":
        self.passed = passed
        self.detail = detail
        print(f"  [{'PASS' if passed else 'FAIL'}] {self.name}: {detail}", flush=True)
        return self

    def invalid(self, detail: str) -> "Step":
        self.passed = None
        self.detail = detail
        print(f"  [SKIP] {self.name}: {detail}", flush=True)
        return self


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="right")
    ap.add_argument("--iface", default="hand_right")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from agx_arm_msgs.msg import DeviceCommandStamp, HandJointTarget
    from agx_arm_msgs.srv import ClaimDevice
    from rcl_interfaces.msg import Log
    from sensor_msgs.msg import JointState

    ns = f"/{args.side}_hand"
    # The node half must be this process's real node name: the bridge revokes a
    # claim whose owner is not in the graph, and a mismatched name gets the claim
    # pulled mid-run — which reads as "no commander" on the next command rather
    # than as the check under test.
    node_name = "l3_hand_command_authority"
    owner = f"reactive:{node_name}"
    steps: list[Step] = []

    bridge = subprocess.Popen(
        [BRIDGE, "--ros-args", "-r", f"__ns:={ns}",
         "-p", f"omnihand_type:={args.side}",
         "-p", "backend_type:=sdk",
         "-p", f"can_interface:={args.iface}",
         "-p", "hand_pub_rate:=20.0",
         "-p", "hand_joint_read_rate:=20.0"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    time.sleep(8.0)

    rclpy.init()
    node = Node(node_name)
    state: dict = {"js": None}
    node.create_subscription(
        JointState, f"{ns}/feedback/omnihand/joint_states",
        lambda m: state.__setitem__("js", m), 10)
    pub = node.create_publisher(HandJointTarget, f"{ns}/control/omnihand/joint_target", 10)
    # Refusals are counted off /rosout, not off the bridge's piped stdout.
    # Python block-buffers a pipe, so a drained refusal can be attributed to the
    # step after the one that caused it — which is how this run first reported
    # the stale-epoch check failing and the valid command being refused.
    refusal_log: list[str] = []
    node.create_subscription(
        Log, "/rosout",
        lambda m: refusal_log.append(m.msg)
        if "refused hand command" in m.msg else None,
        200)
    claim_cli = node.create_client(ClaimDevice, f"{ns}/control/omnihand/claim_device")

    def spin(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def claim(take: bool, who: str = owner):
        req = ClaimDevice.Request()
        req.owner_id = who
        req.claim = take
        fut = claim_cli.call_async(req)
        end = time.monotonic() + 5.0
        while time.monotonic() < end and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result()

    def send(names, positions, owner_id, dev, unit, seq) -> None:
        msg = HandJointTarget()
        stamp = DeviceCommandStamp()
        stamp.owner_id = owner_id
        stamp.device_epoch = dev
        stamp.unit_safety_epoch = unit
        stamp.sequence = seq
        msg.authority = stamp
        msg.joint_names = list(names)
        msg.positions = [float(p) for p in positions]
        pub.publish(msg)
        spin(0.6)

    try:
        spin(3.0)
        if state["js"] is None or not state["js"].name:
            print("no joint feedback from the hand; is it powered and on the bus?")
            return 2
        names = list(state["js"].name)
        # The hand's own measured pose: commanding it is a no-op physically.
        hold = list(state["js"].position)
        print(f"holding pose captured for {len(names)} joints (no motion will result)")

        if not claim_cli.wait_for_service(timeout_sec=5.0):
            print("claim service never appeared")
            return 2

        resp = claim(True)
        if resp is None or not resp.accepted:
            print(f"could not claim the hand: {resp and resp.message}")
            return 2
        dev, unit = int(resp.device_epoch), int(resp.unit_safety_epoch)
        print(f"claimed at device_epoch={dev} unit_safety_epoch={unit}")

        def refusals() -> int:
            return len(refusal_log)

        def drain() -> None:
            # Let /rosout deliver anything the last command produced before the
            # next one is sent, so a refusal is attributed to its own step.
            spin(0.8)

        drain()
        base = refusals()

        # 1. A correctly stamped command is admitted.
        send(names, hold, owner, dev, unit, 1)
        drain()
        steps.append(Step("a correctly stamped command is admitted").settle(
            refusals() == base,
            f"no refusal logged (refusals still {refusals()})"))

        # 2. Foreign owner.
        before = refusals()
        send(names, hold, "reactive:someone_else", dev, unit, 2)
        drain()
        steps.append(Step("a foreign owner is refused").settle(
            refusals() > before, f"refusals {before} -> {refusals()}"))

        # 3. Out-of-order sequence (below the watermark set by #1).
        before = refusals()
        send(names, hold, owner, dev, unit, 1)
        drain()
        steps.append(Step("an out-of-order sequence is refused").settle(
            refusals() > before, f"refusals {before} -> {refusals()}"))

        # 4. Stale unit-safety generation.
        before = refusals()
        send(names, hold, owner, dev, unit + 5, 9)
        drain()
        steps.append(Step("an unknown unit-safety generation is refused").settle(
            refusals() > before, f"refusals {before} -> {refusals()}"))

        # 5. Stale device epoch, after a real handover.
        claim(False)
        resp2 = claim(True)
        if resp2 is None or not resp2.accepted:
            steps.append(Step("a stale device epoch is refused").invalid(
                "could not re-claim the hand"))
        else:
            new_dev = int(resp2.device_epoch)
            if new_dev == dev:
                steps.append(Step("a stale device epoch is refused").invalid(
                    f"the epoch did not advance across a handover ({dev})"))
            else:
                before = refusals()
                send(names, hold, owner, dev, unit, 50)
                drain()
                steps.append(Step("a stale device epoch is refused").settle(
                    refusals() > before,
                    f"epoch {dev} -> {new_dev}; refusals {before} -> {refusals()}"))
                # And the current epoch still works, so the gate is not just shut.
                before = refusals()
                send(names, hold, owner, new_dev, unit, 1)
                drain()
                steps.append(Step("the current epoch is still admitted").settle(
                    refusals() == before,
                    f"refusals {before} -> {refusals()} for a correctly stamped command"))
        claim(False)
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except Exception:
            bridge.kill()

    print()
    print("refusals seen on /rosout, in order:")
    for i, line in enumerate(refusal_log, 1):
        print(f"  {i}. {line}")
    print()
    failed = [s for s in steps if s.passed is False]
    skipped = [s for s in steps if s.passed is None]
    print(f"{len(steps) - len(failed) - len(skipped)} passed, "
          f"{len(failed)} failed, {len(skipped)} inconclusive")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
