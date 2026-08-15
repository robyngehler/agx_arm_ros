#!/usr/bin/env python3
"""Grasp -> contact -> hold -> handover -> trajectory, on a real hand (L3).

This is the run that exercises the whole hand contract at once, on hardware,
with something physically in the hand. Five claims are checked, and each one has
failed at least once in this repository:

1. **Contact stops the motion**, not the clock. A reactive grasp ends where the
   tactile sensor says.
2. **The hold is silent.** After a confirmed grasp the skill controller used to
   republish its target at 20 Hz forever; it now monitors and says nothing. The
   check is a message count on the command topic, because "we removed the
   publish" is a claim about code and this is a claim about the bus.
3. **The trajectory primitive cannot preempt the reactive one.** The bridge is
   fail-closed and one owner holds the hand, so a trajectory goal issued during
   the hold has to be refused — not queued, not interleaved.
4. **The handover is an epoch boundary.** Releasing and re-claiming advances the
   device epoch, which is what makes a late command from the previous owner
   unexecutable.
5. **A stop stops the hand.** Two defects lived here: the stop re-sent the
   travel target instead of the measured pose, and the unit safety path never
   reached `backend.stop()` at all. Neither has been exercised on hardware.

The contact threshold is calibrated at the start of the run rather than typed in:
the tactile scale on the O12 Pro is uncalibrated, so a hardcoded number either
never triggers or triggers on noise. It samples the resting hand and takes a
margin above the worst sample it saw.

Hardware: this moves a real hand. Put the object in the right hand's path first.

Usage:
  python3 scripts/l3_hand_handover_run.py [--side right] [--namespace /right_arm]
      [--contact-margin 3.0] [--grasp-timeout 8.0]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time

BRIDGE = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_bridge"
SKILL = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_skill_controller"
FJT = "install/agx_arm_ctrl/lib/agx_arm_ctrl/omnihand_follow_joint_trajectory"


class Step:
    """One checked claim, with the evidence that settled it."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.passed: bool | None = None
        self.detail = ""

    def settle(self, passed: bool, detail: str) -> "Step":
        self.passed = passed
        self.detail = detail
        mark = "PASS" if passed else "FAIL"
        print(f"  [{mark}] {self.name}: {detail}", flush=True)
        return self

    def invalid(self, detail: str) -> "Step":
        """The precondition never held, so this step proved nothing.

        Reported separately from a failure on purpose. A step that ran against
        the wrong state is not evidence in either direction, and counting it as
        a pass is how a run of six green checks can establish nothing.
        """
        self.passed = None
        self.detail = detail
        print(f"  [SKIP] {self.name}: {detail}", flush=True)
        return self


class Harness:
    def __init__(self, side: str, namespace: str) -> None:
        import rclpy
        from agx_arm_msgs.action import PerformAction
        from agx_arm_msgs.msg import AgxDeviceAuthority, OmniHandStatus, OmniHandTactileRaw
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.side = side
        self.ns = namespace.rstrip("/")
        self.node = Node("l3_handover_probe")
        self.PerformAction = PerformAction
        self.FollowJointTrajectory = FollowJointTrajectory

        self.perform = ActionClient(self.node, PerformAction, f"{self.ns}/perform")
        self.fjt = ActionClient(
            self.node,
            FollowJointTrajectory,
            f"{self.ns}/{side}_omnihand_controller/follow_joint_trajectory",
        )
        self.stop_client = self.node.create_client(
            Trigger, f"{self.ns}/control/omnihand/stop"
        )

        self.joint_names: list[str] = []
        self.positions: list[float] = []
        self.tactile_layout = ""
        self.tactile_values: list[float] = []
        self.authority = None
        self.command_count = 0
        self.status_count = 0

        self.node.create_subscription(
            JointState, f"{self.ns}/feedback/omnihand/joint_states", self._on_joints, 10
        )
        self.node.create_subscription(
            OmniHandTactileRaw,
            f"{self.ns}/feedback/omnihand/tactile_raw",
            self._on_tactile,
            10,
        )
        self.node.create_subscription(
            OmniHandStatus, f"{self.ns}/feedback/omnihand/status", self._on_status, 10
        )
        # The command topic is watched, not driven: claim 2 is a count on it.
        self.node.create_subscription(
            JointState, f"{self.ns}/control/joint_states", self._on_command, 50
        )
        self.node.create_subscription(
            AgxDeviceAuthority,
            f"{self.ns}/feedback/authority",
            self._on_authority,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    # --- callbacks ----------------------------------------------------------

    def _on_joints(self, msg) -> None:
        self.joint_names = list(msg.name)
        self.positions = list(msg.position)

    def _on_tactile(self, msg) -> None:
        self.tactile_layout = msg.layout_name
        self.tactile_values = list(msg.values)

    def _on_status(self, msg) -> None:
        self.status_count += 1

    def _on_command(self, msg) -> None:
        self.command_count += 1

    def _on_authority(self, msg) -> None:
        self.authority = msg

    # --- plumbing -----------------------------------------------------------

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self, timeout: float = 40.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                self.positions
                and self.tactile_values
                and self.perform.server_is_ready()
                and self.fjt.server_is_ready()
                and self.stop_client.service_is_ready()
            ):
                return True
        return False

    def contact_score(self, sensors: list[str] | None = None) -> float:
        from agx_arm_ctrl.omnihand.skills import contact_score, parse_tactile

        reading = parse_tactile(self.tactile_layout, list(self.tactile_values), 1)
        return contact_score(reading, sensors or [], "max")

    def epoch(self) -> int:
        return int(self.authority.device_epoch) if self.authority is not None else -1

    def owner(self) -> str:
        return str(self.authority.owner_id) if self.authority is not None else ""

    def _send_goal(self, client, goal, feedback_cb=None, timeout: float = 60.0):
        send = client.send_goal_async(goal, feedback_callback=feedback_cb)
        deadline = time.monotonic() + timeout
        while not send.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        handle = send.result() if send.done() else None
        if handle is None:
            return None, None
        if not handle.accepted:
            return handle, None
        result_future = handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        return handle, (result_future.result() if result_future.done() else None)

    def skill(self, skill_name: str, metadata: dict | None = None, timeout: float = 60.0):
        goal = self.PerformAction.Goal()
        goal.action_id = f"l3_{skill_name}"
        goal.actiontype_id = "Gripper"
        goal.robot_id = f"{self.side}_hand"
        goal.activity_id = "l3_handover"
        payload = {"skill_name": skill_name}
        payload.update(metadata or {})
        goal.metadata_json = json.dumps(payload)
        self.last_states: list[str] = []
        self.last_scores: list[float] = []

        def on_feedback(msg) -> None:
            self.last_states.append(msg.feedback.state)
            self.last_scores.append(float(msg.feedback.contact_score))

        return self._send_goal(self.perform, goal, on_feedback, timeout)

    def trajectory(self, positions: list[float], seconds: float, timeout: float = 30.0):
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9)
        )
        goal.trajectory.points = [point]
        return self._send_goal(self.fjt, goal, None, timeout)

    def stop_hand(self) -> bool:
        from std_srvs.srv import Trigger

        future = self.stop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        response = future.result() if future.done() else None
        return bool(response and response.success)

    def destroy(self) -> None:
        self.node.destroy_node()


def process_cpu_percent(pid: int, window_s: float) -> float:
    """Percent of ONE core over the window. One core is a Python node's ceiling."""
    ticks = os.sysconf("SC_CLK_TCK")

    def cpu_ticks() -> int:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
            fields = handle.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])

    before = cpu_ticks()
    time.sleep(window_s)
    return (cpu_ticks() - before) / ticks / window_s * 100.0


def launch(side: str, namespace: str) -> list[subprocess.Popen]:
    common = ["--ros-args", "-r", f"__ns:={namespace}", "-p", f"omnihand_type:={side}"]
    procs = [
        subprocess.Popen(
            [BRIDGE, *common,
             "-p", "backend_type:=sdk",
             "-p", "hand_model:=o12_pro",
             "-p", f"can_interface:=hand_{side}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        ),
        subprocess.Popen(
            [SKILL, *common], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        ),
        subprocess.Popen(
            [FJT, *common,
             "-p", f"action_name:={side}_omnihand_controller/follow_joint_trajectory"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        ),
    ]
    return procs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="right", choices=["right", "left"])
    parser.add_argument("--namespace", default=None)
    parser.add_argument("--contact-margin", type=float, default=3.0)
    parser.add_argument("--grasp-timeout", type=float, default=10.0)
    parser.add_argument("--hold-window", type=float, default=5.0)
    args = parser.parse_args()
    if args.namespace is None:
        args.namespace = f"/{args.side}_arm"

    for path in (BRIDGE, SKILL, FJT):
        if not os.path.exists(path):
            print(f"!! {path} not found — build agx_arm_ctrl first")
            return 1

    import rclpy

    rclpy.init()
    procs = launch(args.side, args.namespace)
    steps: list[Step] = []
    harness = None
    try:
        harness = Harness(args.side, args.namespace)
        if not harness.wait_ready():
            print("!! the stack did not come up (bridge/skill/trajectory not all ready)")
            return 1
        print(f"stack up on {args.namespace}, {len(harness.joint_names)} joints\n")

        # --- 0. calibrate the threshold against the resting hand -------------
        print("0. calibrating contact against the resting hand", flush=True)
        harness.skill("open_hand", {"timeout_sec": 4.0})
        harness.spin(1.0)
        rest = []
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            harness.spin(0.1)
            rest.append(harness.contact_score())
        rest_max = max(rest) if rest else 0.0
        idle_cpu = process_cpu_percent(procs[0].pid, 4.0)
        print(f"   bridge idle (no owner): {idle_cpu:.1f} % of a core", flush=True)
        threshold = rest_max + args.contact_margin
        print(
            f"   resting contact score max={rest_max:.3f} over {len(rest)} samples "
            f"-> threshold {threshold:.3f}\n",
            flush=True,
        )

        # --- 1. grasp until contact -----------------------------------------
        print("1. reactive grasp: closing until the tactile sensor says stop", flush=True)
        step = Step("contact ends the motion")
        _, result = harness.skill(
            "grasp_glass_until_contact",
            {
                "contact_threshold": threshold,
                "stable_samples": 3,
                "timeout_sec": args.grasp_timeout,
                "contact_aggregation": "max",
                "completion_policy": {"on_success": "hold_internal"},
            },
            timeout=args.grasp_timeout + 20.0,
        )
        if result is None:
            step.settle(False, "no result from the skill action")
        else:
            res = result.result
            held = res.final_state == "GRASP_HOLDING"
            step.settle(
                held and res.success,
                f"state={res.final_state} score={res.final_contact_score:.3f} "
                f"(threshold {threshold:.3f}) msg='{res.message}'",
            )
        steps.append(step)
        if harness.last_scores:
            print(
                f"   contact trace over {len(harness.last_scores)} feedback samples: "
                f"min={min(harness.last_scores):.3f} max={max(harness.last_scores):.3f}",
                flush=True,
            )
        print(f"   states seen: {' -> '.join(dict.fromkeys(harness.last_states))}", flush=True)
        grasp_epoch = harness.epoch()
        grasp_owner = harness.owner()
        print(f"   owner={grasp_owner or '(none)'} epoch={grasp_epoch}\n", flush=True)

        # --- 2. the hold is silent ------------------------------------------
        print(f"2. watching the command topic for {args.hold_window:g}s of hold", flush=True)
        holding = grasp_owner.startswith("reactive:")
        harness.command_count = 0
        pose_before = list(harness.positions)
        hold_cpu = process_cpu_percent(procs[0].pid, args.hold_window)
        harness.spin(0.5)
        commands = harness.command_count
        pose_after = list(harness.positions)
        drift = (
            max(abs(a - b) for a, b in zip(pose_before, pose_after))
            if pose_before and len(pose_before) == len(pose_after)
            else float("nan")
        )
        evidence = (
            f"{commands} command message(s) in {args.hold_window:g}s; "
            f"pose drift {drift:.4f} rad; "
            f"bridge at {hold_cpu:.1f} % of a core (tactile at the reactive rate)"
        )
        step = Step("the hold sends nothing")
        steps.append(
            step.settle(commands == 0, evidence)
            if holding
            else step.invalid(f"no reactive owner held the hand — {evidence}")
        )

        # --- 3. the trajectory primitive cannot preempt the hold ------------
        print("\n3. trajectory goal while the reactive owner holds", flush=True)
        handle, result = harness.trajectory(list(harness.positions), 1.0, timeout=20.0)
        refused = handle is not None and not handle.accepted
        if not refused and result is not None:
            refused = result.result.error_code != 0
        detail = "goal rejected by the action server" if (handle and not handle.accepted) else (
            f"error_code={result.result.error_code} '{result.result.error_string}'"
            if result is not None
            else "no result"
        )
        step = Step("a trajectory cannot preempt a grasp")
        steps.append(
            step.settle(refused, detail)
            if holding
            else step.invalid(f"nothing held the hand, so nothing was preempted — {detail}")
        )

        # --- 4. handover ------------------------------------------------------
        print("\n4. handover: the reactive owner releases", flush=True)
        harness.skill("release_glass", {"timeout_sec": 5.0}, timeout=30.0)
        harness.spin(1.0)
        released_epoch = harness.epoch()
        steps.append(
            Step("the handover advances the device epoch").settle(
                released_epoch > grasp_epoch and harness.owner() == "",
                f"epoch {grasp_epoch} -> {released_epoch}, "
                f"owner now '{harness.owner() or '(none)'}'",
            )
        )

        # --- 5. the trajectory primitive now owns the hand -------------------
        print("\n5. trajectory goal on the free hand", flush=True)
        target = list(harness.positions)
        handle, result = harness.trajectory(target, 2.0, timeout=30.0)
        ok = (
            handle is not None
            and handle.accepted
            and result is not None
            and result.result.error_code == 0
        )
        steps.append(
            Step("the trajectory primitive can take the hand").settle(
                ok,
                f"accepted={handle.accepted if handle else False} "
                f"error_code={result.result.error_code if result else 'n/a'}",
            )
        )

        # --- 6. a stop stops the hand ----------------------------------------
        print("\n6. stop during a moving trajectory", flush=True)
        closing = [min(1.2, p + 0.5) for p in harness.positions]
        pose_at_goal_start = list(harness.positions)
        send = harness.fjt.send_goal_async(_traj_goal(harness, closing, 6.0))
        deadline = time.monotonic() + 10.0
        while not send.done() and time.monotonic() < deadline:
            harness.rclpy.spin_once(harness.node, timeout_sec=0.02)
        handle = send.result() if send.done() else None
        harness.spin(1.5)
        moving_pose = list(harness.positions)
        stopped = harness.stop_hand()
        harness.spin(0.4)
        settled = list(harness.positions)
        harness.spin(2.0)
        after = list(harness.positions)
        motion_after_stop = (
            max(abs(a - b) for a, b in zip(settled, after)) if settled and after else float("nan")
        )
        moved_before = (
            max(abs(a - b) for a, b in zip(moving_pose, settled))
            if moving_pose and settled
            else float("nan")
        )
        was_moving = (
            handle is not None
            and handle.accepted
            and max(abs(a - b) for a, b in zip(pose_at_goal_start, moving_pose)) > 0.02
        )
        evidence = (
            f"stop acknowledged={stopped}; moved {moved_before:.4f} rad while "
            f"stopping, then {motion_after_stop:.4f} rad over the next 2 s"
        )
        step = Step("a stop halts the hand and holds it")
        steps.append(
            step.settle(stopped and motion_after_stop < 0.02, evidence)
            if was_moving
            else step.invalid(
                "the hand never started moving, so a stop was not tested — "
                f"goal accepted={handle.accepted if handle else False}; {evidence}"
            )
        )
        if handle is not None and handle.accepted:
            cancel = handle.cancel_goal_async()
            deadline = time.monotonic() + 5.0
            while not cancel.done() and time.monotonic() < deadline:
                harness.rclpy.spin_once(harness.node, timeout_sec=0.02)

        # --- leave the hand open ---------------------------------------------
        print("\n7. returning the hand to open", flush=True)
        harness.skill("open_hand", {"timeout_sec": 5.0}, timeout=30.0)
        harness.spin(1.0)

    finally:
        if harness is not None:
            harness.destroy()
        rclpy.shutdown()
        for proc in procs:
            proc.send_signal(signal.SIGINT)
        for proc in procs:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    print("\n=== result ===")
    for step in steps:
        mark = "PASS" if step.passed else ("SKIP" if step.passed is None else "FAIL")
        print(f"  {mark}  {step.name}")
        print(f"        {step.detail}")
    failed = [s for s in steps if s.passed is False]
    skipped = [s for s in steps if s.passed is None]
    held = [s for s in steps if s.passed is True]
    print(
        f"\n{len(held)} claim(s) held, {len(failed)} failed, "
        f"{len(skipped)} could not be tested"
    )
    return 1 if (failed or skipped) else 0


def _traj_goal(harness, positions, seconds):
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectoryPoint

    goal = harness.FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(harness.joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
    goal.trajectory.points = [point]
    return goal


if __name__ == "__main__":
    sys.exit(main())
