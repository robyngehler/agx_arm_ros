#!/usr/bin/env python3
"""Shared orchestration for the operator demo scripts.

The demo scripts beside this file each name one stack and one activity; all the
behaviour is here. Nothing in this module is a new ROS surface: it starts the
existing launches, waits for the surfaces they provide, and runs the existing
``run_activity`` client.

WHY run_activity RUNS IN THE FOREGROUND. It already owns the safe cancel path —
the first Ctrl+C cancels the activity, the second escalates to the unit emergency
stop, the third gives up. So the launches are started in their own session and
``run_activity`` is left in this process's group, which is what the terminal
sends SIGINT to. This wrapper ignores its own SIGINT and waits: tearing the
launches down while the coordinator is still unwinding would take the arms'
driver away mid-cancel.

WHY THE STEP NUMBERS COME FROM THE ACTIVITY YAML. ``--from-id`` counts operator
steps, and this script reads them with the coordinator's own loader and
scheduler (``ActivityCatalogue`` + ``operator_steps``) from the same files the
coordinator runs. There is no second step model to drift.

Run these from a shell that has sourced the workspace — they import
agx_arm_coordination and rclpy, the same way any ROS command does.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "src" / "agx_arm_coordination" / "config"


# --- what a stack is --------------------------------------------------------

@dataclass(frozen=True)
class LaunchSpec:
    package: str
    launch_file: str
    args: tuple[str, ...] = ()

    def command(self) -> list[str]:
        return ["ros2", "launch", self.package, self.launch_file, *self.args]

    def label(self) -> str:
        return f"{self.package}/{self.launch_file}"


@dataclass(frozen=True)
class StackSpec:
    """One operator flow: what to bring up, what to wait for, what to run."""

    name: str
    activity: str
    launches: tuple[LaunchSpec, ...]
    #: Extra readiness beyond COMMON_READINESS, for stacks that need more.
    extra_services: tuple[str, ...] = ()
    extra_actions: tuple[str, ...] = ()
    description: str = ""


#: The arm stack every flow needs. `duo_hand` keeps both OmniHands in the model
#: even for the arm-only flows: dropping them would drop their mass and geometry
#: out of gravity and collision checking, which is not a saving worth making on
#: a packing move.
ARM_COMPONENTS = LaunchSpec(
    package="agx_arm_ctrl",
    launch_file="start_agx_arm_components.launch.py",
    args=(
        "mode:=moveit_mit",
        "execution_profile:=duo_hand",
        "follow:=true",
        "planning_pipelines:=ompl",
        "use_rviz:=false",
    ),
)

#: The tea demo's arm stack. `duo_hand_external_bridge` models the hands but does
#: not own them — start_tea_demo.launch.py starts the hand bridges itself, and
#: two owners of one vendor SDK session is the thing the authority model exists
#: to prevent. The payload mass is required: the grip action attaches the teapot
#: gravity model part way through the sequence.
ARM_COMPONENTS_TEA = LaunchSpec(
    package="agx_arm_ctrl",
    launch_file="start_agx_arm_components.launch.py",
    args=(
        "mode:=moveit_mit",
        "execution_profile:=duo_hand_external_bridge",
        "follow:=true",
        "planning_pipelines:=ompl",
        "use_rviz:=false",
        "payload_mass_kg:=1.0",
    ),
)

#: The block demo's arm stack. `duo_gripper` puts an AGX parallel gripper on each
#: arm instead of an OmniHand, which is what gives the two gripper trajectory
#: servers the activity commands.
ARM_COMPONENTS_GRIPPER = LaunchSpec(
    package="agx_arm_ctrl",
    launch_file="start_agx_arm_components.launch.py",
    args=(
        "mode:=moveit_mit",
        "execution_profile:=duo_gripper",
        "follow:=true",
        "planning_pipelines:=ompl",
        "use_rviz:=false",
    ),
)

COORDINATION = LaunchSpec(
    package="agx_arm_coordination",
    launch_file="start_coordination.launch.py",
)

TEA_COORDINATION = LaunchSpec(
    package="agx_arm_coordination",
    launch_file="start_tea_demo.launch.py",
    args=("backend_type:=sdk",),
)

#: Waited for before anything is offered to the operator. A process that exists
#: is not a stack that works, and every name here is one the next step uses.
COMMON_SERVICES = (
    "/left_arm/emergency_stop",
    "/right_arm/emergency_stop",
    "/unit_safety/rearm",
)
COMMON_ACTIONS = ("/move_action", "/execute_activity")
COMMON_TOPICS = ("/left_arm/feedback/joint_states", "/right_arm/feedback/joint_states")

#: The coordinator's event stream, followed during the run so a failure can be
#: reported as the step number to resume from.
EVENT_TOPIC = "/events"


def arm_flow(name: str, activity: str, description: str) -> StackSpec:
    """A pack or unpack flow: arms only, no hand skill controllers."""
    return StackSpec(
        name=name,
        activity=activity,
        launches=(ARM_COMPONENTS, COORDINATION),
        description=description,
    )


#: A gripper flow additionally waits for the two trajectory servers it commands.
#: Without them a gripper action fails at dispatch, which on the block demo is
#: step 2 of 63 — after the arms have already moved to the start pose.
GRIPPER_ACTIONS = (
    "/left_arm/gripper_controller/follow_joint_trajectory",
    "/right_arm/gripper_controller/follow_joint_trajectory",
)

BLOCK_RESTACK = StackSpec(
    name="start_block_restack",
    activity="block_restack_v1",
    launches=(ARM_COMPONENTS_GRIPPER, COORDINATION),
    extra_actions=GRIPPER_ACTIONS,
    description="Restack four blocks, handing each from the left gripper to the right.",
)


TEA_DEMO = StackSpec(
    name="start_tea_demo",
    activity="tea_pour_duo_v2",
    launches=(ARM_COMPONENTS_TEA, TEA_COORDINATION),
    extra_actions=("/left_hand/perform", "/right_hand/perform"),
    description="Grip a tea can with the left arm and hand, pour it, put it back.",
)


# --- the activity's own step model ------------------------------------------

def _catalogue_and_units():
    from agx_arm_coordination.graph_loader import ActivityCatalogue
    from agx_arm_coordination.motion_registry import bus_topology
    from agx_arm_coordination.graph_model import robot_units

    units = robot_units(bus_topology())
    return ActivityCatalogue.from_config_dir(CONFIG_DIR, units), units


def activity_steps(activity: str):
    """``(steps, resumable_step_numbers, actions)`` for one activity.

    Read through the coordinator's own loader and scheduler, from the same YAML
    the coordinator runs, so a step number here means what it means there.
    """
    from agx_arm_coordination.graph_model import operator_steps
    from agx_arm_coordination.operator_resume import resumable_steps

    catalogue, units = _catalogue_and_units()
    steps = operator_steps(catalogue.get_activity_plan(activity), catalogue.actions, units)
    return steps, resumable_steps(steps, catalogue.actions), catalogue.actions


def check_from_id(activity: str, from_id) -> None:
    """Refuse an impossible resume before anything is launched.

    The coordinator refuses it too, but only once the whole stack is up — which
    on this machine is the expensive half of the operation.
    """
    if from_id is None:
        return
    from agx_arm_coordination.operator_resume import ResumeError, resume_seed

    catalogue, units = _catalogue_and_units()
    try:
        resume_seed(catalogue.get_activity_plan(activity), catalogue.actions, units, from_id)
    except ResumeError as exc:
        raise SystemExit(f"--from-id {from_id}: {exc}")


def describe_steps(activity: str, from_id) -> str:
    steps, resumable, _ = activity_steps(activity)
    lines = [f"{activity}: {len(steps)} operator steps"]
    if from_id:
        lines.append(f"  starting at step {from_id}")
    skipped = set(resumable) ^ set(range(1, len(steps) + 1))
    if skipped:
        lines.append(
            "  steps that replay a taught path (not resume points): "
            + ", ".join(str(step) for step in sorted(skipped))
        )
    return "\n".join(lines)


# --- readiness and progress, over one short-lived ROS node ------------------

class _StackWatcher:
    """Waits for the surfaces a stack must provide, then follows its events."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        self._rclpy = rclpy
        rclpy.init()
        self._node = Node("agx_demo_stack_watcher")
        self.last_completed_action = ""
        self._node.create_subscription(
            self._event_type(), EVENT_TOPIC, self._on_event, 10
        )

    @staticmethod
    def _event_type():
        from agx_arm_msgs.msg import RobotEvent

        return RobotEvent

    def _on_event(self, msg) -> None:
        if msg.event_type == "completed" and msg.action_id:
            self.last_completed_action = msg.action_id

    def missing(self, spec: StackSpec) -> list[str]:
        from rclpy.action import get_action_names_and_types

        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        services = {name for name, _ in self._node.get_service_names_and_types()}
        topics = {name for name, _ in self._node.get_topic_names_and_types()}
        actions = {name for name, _ in get_action_names_and_types(self._node)}
        wanted = [
            *((name, services) for name in COMMON_SERVICES + spec.extra_services),
            *((name, actions) for name in COMMON_ACTIONS + spec.extra_actions),
            *((name, topics) for name in COMMON_TOPICS),
        ]
        return [name for name, present in wanted if name not in present]

    def wait_until_ready(self, spec: StackSpec, timeout_s: float) -> list[str]:
        """Returns the names still missing when the timeout ran out — empty is ready."""
        deadline = time.monotonic() + timeout_s
        missing = self.missing(spec)
        last_report = 0.0
        while missing and time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.25)
            missing = self.missing(spec)
            now = time.monotonic()
            if missing and now - last_report > 10.0:
                last_report = now
                print(f"  still waiting for: {', '.join(missing)}", flush=True)
        return missing

    def pump(self, seconds: float) -> None:
        """Service the event subscription while the activity runs."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def close(self) -> None:
        try:
            self._node.destroy_node()
        finally:
            if self._rclpy.ok():
                self._rclpy.shutdown()


# --- launches ---------------------------------------------------------------

class _Launches:
    """The background launches, in their own session so no terminal signal reaches them."""

    def __init__(self, specs, log_dir: Path) -> None:
        self._specs = specs
        self._log_dir = log_dir
        self._procs: list[tuple[LaunchSpec, subprocess.Popen, Path]] = []

    def start(self) -> None:
        for spec in self._specs:
            log_path = self._log_dir / f"{spec.package}-{spec.launch_file}.log"
            handle = log_path.open("w")
            print(f"  starting {spec.label()} -> {log_path}", flush=True)
            proc = subprocess.Popen(
                spec.command(),
                stdout=handle,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                # Its own session: a Ctrl+C in the terminal must reach
                # run_activity, not the stack it is cancelling against.
                start_new_session=True,
            )
            self._procs.append((spec, proc, log_path))

    def died(self):
        """A launch that exited on its own, or None."""
        for spec, proc, log_path in self._procs:
            if proc.poll() is not None:
                return spec, proc.returncode, log_path
        return None

    def stop(self) -> None:
        for spec, proc, _ in reversed(self._procs):
            if proc.poll() is not None:
                continue
            print(f"  stopping {spec.label()}", flush=True)
            self._signal_group(proc, signal.SIGINT)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and any(
            proc.poll() is None for _, proc, _ in self._procs
        ):
            time.sleep(0.2)
        for spec, proc, _ in self._procs:
            if proc.poll() is None:
                print(f"  {spec.label()} did not stop; killing it", flush=True)
                self._signal_group(proc, signal.SIGKILL)

    @staticmethod
    def _signal_group(proc: subprocess.Popen, sig) -> None:
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass


# --- the run ----------------------------------------------------------------

def _run_activity_command(spec: StackSpec, args) -> list[str]:
    command = [
        "ros2", "run", "agx_arm_coordination", "run_activity",
        "--activity", spec.activity,
    ]
    if args.from_id is not None:
        command += ["--from-id", str(args.from_id)]
    if args.metadata_json:
        command += ["--metadata-json", args.metadata_json]
    return command


def _next_from_id(spec: StackSpec, watcher: _StackWatcher) -> str:
    """What the operator should pass to pick this run up again.

    The mapping lives in operator_resume beside the one the coordinator uses, so
    the number printed here is the number the coordinator will accept.
    """
    from agx_arm_coordination.operator_resume import next_resume_step

    steps, resumable, _ = activity_steps(spec.activity)
    done, following = next_resume_step(steps, resumable, watcher.last_completed_action)
    if not done:
        return "nothing completed; rerun without --from-id"
    if following is None:
        return f"step {done} completed; no later step is a valid resume point"
    return (
        f"step {done} completed ({watcher.last_completed_action}); "
        f"resume with --from-id {following}"
    )


def _warn_if_a_dropped_connection_would_orphan_the_stack() -> None:
    """Over SSH without tmux, a dropped link leaves the stack running unattended.

    The launches deliberately run in their own session so a terminal Ctrl+C
    reaches run_activity instead of them. The same property means a SIGHUP kills
    only this wrapper: the teardown never runs, and the arms keep a live driver
    with nobody supervising it.
    """
    if os.environ.get("TMUX") or os.environ.get("STY"):
        return
    if not (os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY")):
        return
    print(
        "WARNING: this is an SSH session and not a tmux/screen one. If the\n"
        "         connection drops, this wrapper dies and the launches keep\n"
        "         running with nothing to shut them down. Prefer:\n"
        "             tmux new -A -s demo\n",
        file=sys.stderr,
    )


def run_stack(spec: StackSpec, args) -> int:
    check_from_id(spec.activity, args.from_id)
    _warn_if_a_dropped_connection_would_orphan_the_stack()

    log_dir = Path(args.log_dir) if args.log_dir else Path(
        tempfile.mkdtemp(prefix=f"{spec.name}-")
    )
    log_dir.mkdir(parents=True, exist_ok=True)

    print(f"== {spec.name}")
    if spec.description:
        print(f"   {spec.description}")
    print(describe_steps(spec.activity, args.from_id))
    print(f"   logs: {log_dir}")

    launches = _Launches(spec.launches, log_dir)
    watcher = None
    try:
        launches.start()
        watcher = _StackWatcher()
        print(f"waiting up to {args.timeout_sec:.0f}s for the stack to come up")
        missing = watcher.wait_until_ready(spec, args.timeout_sec)
        if missing:
            dead = launches.died()
            if dead:
                dead_spec, code, log_path = dead
                print(
                    f"{dead_spec.label()} exited with {code}; see {log_path}",
                    file=sys.stderr,
                )
            print(f"stack never became ready; missing: {', '.join(missing)}", file=sys.stderr)
            return 1

        print("stack is ready:")
        for name in COMMON_ACTIONS + spec.extra_actions:
            print(f"  action  {name}")
        print(f"  activity {spec.activity}")

        if not args.no_prompt:
            try:
                input("\nPress Enter to start, or Ctrl+C to abort: ")
            except (EOFError, KeyboardInterrupt):
                print("\nnot started")
                return 130

        if args.dry_run:
            print("dry run: the stack is up and no activity was sent")
            return 0

        return _execute(spec, args, watcher)
    finally:
        if watcher is not None:
            watcher.close()
        launches.stop()


def _execute(spec: StackSpec, args, watcher: _StackWatcher) -> int:
    """Run the activity in the foreground and let it own the interrupt."""
    command = _run_activity_command(spec, args)
    print(f"\n$ {' '.join(command)}\n", flush=True)

    # From here the terminal's Ctrl+C belongs to run_activity: first press
    # cancels the activity, second escalates to the unit emergency stop. This
    # wrapper must not act on it, or it would tear the stack down underneath.
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        proc = subprocess.Popen(command)
        while proc.poll() is None:
            watcher.pump(0.2)
        code = proc.returncode
    finally:
        signal.signal(signal.SIGINT, previous)

    watcher.pump(0.5)
    if code == 0:
        print("\nactivity completed")
    else:
        print(f"\nactivity did not complete (exit {code})", file=sys.stderr)
        print(f"  {_next_from_id(spec, watcher)}", file=sys.stderr)
        print(
            "  after an emergency stop or a bus recovery, re-arm explicitly before "
            "resuming:\n"
            "    ros2 service call /left_arm/clear_fault_lockout std_srvs/srv/Trigger\n"
            "    ros2 service call /right_arm/clear_fault_lockout std_srvs/srv/Trigger\n"
            "    ros2 service call /unit_safety/rearm std_srvs/srv/Trigger",
            file=sys.stderr,
        )
    return code


# --- the shared CLI ---------------------------------------------------------

def build_parser(description: str, with_speed: bool = False) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--from-id", "--from_id", dest="from_id", type=int, default=None, metavar="N",
        help="resume at operator step N (1-based); refused if step N replays a taught path",
    )
    parser.add_argument(
        "--no-prompt", action="store_true",
        help="start as soon as the stack is ready instead of waiting for Enter",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="bring the stack up and verify readiness, send no activity",
    )
    parser.add_argument(
        "--timeout-sec", type=float, default=120.0,
        help="how long to wait for the stack to provide its surfaces",
    )
    parser.add_argument(
        "--log-dir", default="",
        help="where to write the launch logs (default: a fresh temporary directory)",
    )
    parser.add_argument(
        "--metadata-json", default="",
        help="extra run-time overrides passed through to run_activity",
    )
    if with_speed:
        parser.add_argument(
            "--speed", choices=("fast", "slow"), default="slow",
            help="slow visits every intermediate anchor; fast plans one long "
                 "transition instead. Slow first when anything is in front of the unit",
        )
        parser.add_argument(
            "--fast", dest="speed", action="store_const", const="fast",
            help="alias for --speed fast",
        )
        parser.add_argument(
            "--slow", dest="speed", action="store_const", const="slow",
            help="alias for --speed slow",
        )
    return parser


def main_for(spec_or_factory, description: str, with_speed: bool = False) -> None:
    """Entry point for one demo script."""
    parser = build_parser(description, with_speed=with_speed)
    args = parser.parse_args()
    spec = spec_or_factory(args) if callable(spec_or_factory) else spec_or_factory
    raise SystemExit(run_stack(spec, args))
