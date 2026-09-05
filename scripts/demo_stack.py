#!/usr/bin/env python3
"""Shared orchestration for the operator demo scripts.

Two roles, two processes, one stack:

  start_demo_stack.py         brings the launches up and stays alive owning them
  unpack_*.py / wave.py / …   attach to that stack and run one activity
  stop_demo_stack.py          asks the supervisor to shut down

WHY THE SPLIT. A stack that belongs to one activity is torn down when that
activity ends — including when it is cancelled, which is when the operator most
wants it. The resume hint printed on a failure named a coordinator that the same
function had just shut down. The stack now outlives every activity run against
it, so a cancel, a resume and the next activity all reuse one bring-up.

WHICH UNIT THIS IS. ``AGX_UNIT`` (top or bottom), set in ~/.bashrc by
scripts/isolate_ros_graph.sh alongside the ROS domain. It decides which
execution profile the stack comes up with, and every activity script refuses to
run on the unit it was not written for. ``--unit`` overrides it.

WHY run_activity RUNS IN THE FOREGROUND. It already owns the safe cancel path —
the first Ctrl+C cancels the activity, the second escalates to the unit
emergency stop. The activity wrapper ignores its own SIGINT so the terminal's
interrupt belongs to run_activity. Nothing tears the stack down behind it any
more; that is the supervisor's, in its own pane.

WHY THE STEP NUMBERS COME FROM THE ACTIVITY YAML. ``--from-id`` counts operator
steps, read with the coordinator's own loader and scheduler
(``ActivityCatalogue`` + ``operator_steps``) from the files the coordinator runs.
There is no second step model to drift.

Run these from a shell that has sourced the workspace — they import
agx_arm_coordination and rclpy, the same way any ROS command does.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "src" / "agx_arm_coordination" / "config"

#: Persistent, unlike a temp dir: the logs of a failed run are the only account
#: of it. gitignored at the repo root.
LOG_ROOT = REPO_ROOT / "logs" / "demo_stack"

#: Not /run/user/<uid>: systemd removes that when the user's last login session
#: ends, which is the dropped-SSH case a tmux supervisor exists to survive.
STATE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "agx_demo_stack"


# --- which unit this is -----------------------------------------------------

UNIT_ENV_VAR = "AGX_UNIT"
UNIT_NAMES = ("top", "bottom")


def resolve_unit(explicit: str | None = None) -> str:
    """The unit this machine is, from --unit or AGX_UNIT."""
    unit = (explicit or os.environ.get(UNIT_ENV_VAR, "")).strip().lower()
    if unit in UNIT_NAMES:
        return unit
    if unit:
        raise SystemExit(f"unknown unit '{unit}'; expected one of {', '.join(UNIT_NAMES)}")
    raise SystemExit(
        f"this machine does not say which unit it is: {UNIT_ENV_VAR} is unset.\n"
        f"  set it once with:  ./scripts/isolate_ros_graph.sh --unit top|bottom\n"
        f"  or pass --unit for this command only"
    )


def require_unit(expected: str, explicit: str | None = None) -> str:
    """Refuse an activity written for the other unit."""
    unit = resolve_unit(explicit)
    if unit != expected:
        raise SystemExit(
            f"this is the {unit} unit and that activity belongs to the {expected} unit.\n"
            f"  run the {unit} scripts here, or pass --unit {expected} if {UNIT_ENV_VAR} is wrong"
        )
    return unit


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

    def log_name(self) -> str:
        return f"{self.package}-{self.launch_file}.log"


def arm_components(execution_profile: str, *, extra: tuple[str, ...] = ()) -> LaunchSpec:
    return LaunchSpec(
        package="agx_arm_ctrl",
        launch_file="start_agx_arm_components.launch.py",
        args=(
            "mode:=moveit_mit",
            f"execution_profile:={execution_profile}",
            "follow:=true",
            "planning_pipelines:=ompl",
            "use_rviz:=false",
            *extra,
        ),
    )


COORDINATION = LaunchSpec(
    package="agx_arm_coordination",
    launch_file="start_coordination.launch.py",
)

#: The tea demo's coordination launch: it starts the hand bridges itself, which
#: is why its arm components model the hands without owning them. Two owners of
#: one vendor SDK session is what the authority model exists to prevent.
TEA_COORDINATION = LaunchSpec(
    package="agx_arm_coordination",
    launch_file="start_tea_demo.launch.py",
    args=("backend_type:=sdk",),
)

#: What the components launch must provide before coordination is started.
#: Every name here is one a later step uses; a process that exists is not a
#: stack that works.
COMPONENT_SERVICES = (
    "/left_arm/emergency_stop",
    "/right_arm/emergency_stop",
    "/unit_safety/rearm",
)
COMPONENT_ACTIONS = ("/move_action",)
COMPONENT_TOPICS = ("/left_arm/feedback/joint_states", "/right_arm/feedback/joint_states")

#: What the coordination launch adds. Waited for separately, because the
#: coordinator's action clients only wait for their servers at dispatch time —
#: /execute_activity appears whether or not an arm ever did.
COORDINATION_ACTIONS = ("/execute_activity",)

#: The coordinator's event stream, followed during a run so a failure can be
#: reported as the step number to resume from.
EVENT_TOPIC = "/events"


@dataclass(frozen=True)
class UnitSpec:
    """One stack: what to bring up, in order, and what each phase must provide."""

    unit: str
    stack: str
    execution_profile: str
    launches: tuple[LaunchSpec, ...]
    extra_component_actions: tuple[str, ...] = ()
    extra_coordination_actions: tuple[str, ...] = ()
    description: str = ""


def top_stack() -> UnitSpec:
    """Top unit: both arms with both OmniHands in the model.

    ``duo_hand`` keeps the hands in the model even though pack, unpack and wave
    never command them: dropping them would drop their mass and geometry out of
    gravity and collision checking. No SDK backend is started — nothing here
    moves a hand.
    """
    return UnitSpec(
        unit="top",
        stack="demo",
        execution_profile="duo_hand",
        launches=(arm_components("duo_hand"), COORDINATION),
        description="top unit: both arms, OmniHands modelled but not driven",
    )


def bottom_stack(*, grippers: bool = False) -> UnitSpec:
    """Bottom unit: both arms, optionally carrying their AGX grippers.

    ``duo_arm`` is what the bottom unit runs today. ``duo_gripper`` additionally
    models and drives a gripper per arm — validated by the block restack demo —
    and then the two gripper trajectory servers are part of readiness, because a
    gripper action that fails at dispatch fails after the arms have already moved.
    """
    profile = "duo_gripper" if grippers else "duo_arm"
    return UnitSpec(
        unit="bottom",
        stack="demo",
        execution_profile=profile,
        launches=(arm_components(profile), COORDINATION),
        extra_component_actions=GRIPPER_ACTIONS if grippers else (),
        description=f"bottom unit: both arms ({profile})",
    )


GRIPPER_ACTIONS = (
    "/left_arm/gripper_controller/follow_joint_trajectory",
    "/right_arm/gripper_controller/follow_joint_trajectory",
)

#: The tea demo's hand skill controllers, started by TEA_COORDINATION.
HAND_PERFORM_ACTIONS = ("/left_hand/perform", "/right_hand/perform")


def tea_stack(unit: str) -> UnitSpec:
    """The tea demo's stack: hands modelled and driven, teapot mass declared.

    ``duo_hand_external_bridge`` models the hands but does not own them — the
    coordination launch starts the bridges. The payload mass is required: the
    grip action attaches the teapot gravity model part way through the sequence.
    """
    return UnitSpec(
        unit=unit,
        stack="tea",
        execution_profile="duo_hand_external_bridge",
        launches=(
            arm_components("duo_hand_external_bridge", extra=("payload_mass_kg:=1.0",)),
            TEA_COORDINATION,
        ),
        extra_coordination_actions=HAND_PERFORM_ACTIONS,
        description="tea demo: both arms and both OmniHands, teapot mass declared",
    )


def block_stack(unit: str) -> UnitSpec:
    """The block restack's stack: an AGX gripper on each arm instead of a hand."""
    return UnitSpec(
        unit=unit,
        stack="block",
        execution_profile="duo_gripper",
        launches=(arm_components("duo_gripper"), COORDINATION),
        extra_component_actions=GRIPPER_ACTIONS,
        description="block restack: both arms with AGX grippers",
    )


STACK_CHOICES = ("demo", "tea", "block")


def unit_stack(unit: str, *, stack: str = "demo", grippers: bool = False) -> UnitSpec:
    """The stack to bring up on this unit."""
    if stack == "tea":
        return tea_stack(unit)
    if stack == "block":
        return block_stack(unit)
    if unit == "top":
        if grippers:
            raise SystemExit("--grippers applies to the bottom unit; the top unit carries hands")
        return top_stack()
    return bottom_stack(grippers=grippers)


# --- the supervisor's state file --------------------------------------------

@dataclass
class StackState:
    unit: str
    stack: str
    pid: int
    log_dir: str
    started: str
    execution_profile: str

    @staticmethod
    def path(unit: str) -> Path:
        return STATE_DIR / f"{unit}.json"

    def write(self) -> Path:
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        path = self.path(self.unit)
        path.write_text(json.dumps(self.__dict__, indent=2) + "\n")
        return path

    @classmethod
    def read(cls, unit: str) -> "StackState | None":
        try:
            data = json.loads(cls.path(unit).read_text())
            return cls(**data)
        except (OSError, ValueError, TypeError):
            return None

    def alive(self) -> bool:
        try:
            os.kill(self.pid, 0)
        except (ProcessLookupError, PermissionError):
            return False
        return True

    def remove(self) -> None:
        self.path(self.unit).unlink(missing_ok=True)


def running_supervisor(unit: str) -> StackState | None:
    """The supervisor for this unit, or None — clearing a state file it outlived."""
    state = StackState.read(unit)
    if state is None:
        return None
    if not state.alive():
        state.remove()
        return None
    return state


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

    def missing(self, services=(), actions=(), topics=()) -> list[str]:
        from rclpy.action import get_action_names_and_types

        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        have_services = {name for name, _ in self._node.get_service_names_and_types()}
        have_topics = {name for name, _ in self._node.get_topic_names_and_types()}
        have_actions = {name for name, _ in get_action_names_and_types(self._node)}
        wanted = [
            *((name, have_services) for name in services),
            *((name, have_actions) for name in actions),
            *((name, have_topics) for name in topics),
        ]
        return [name for name, present in wanted if name not in present]

    def wait_for(self, phase: str, timeout_s: float, **wanted) -> list[str]:
        """Returns the names still missing when the timeout ran out — empty is ready."""
        deadline = time.monotonic() + timeout_s
        missing = self.missing(**wanted)
        last_report = 0.0
        while missing and time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.25)
            missing = self.missing(**wanted)
            now = time.monotonic()
            if missing and now - last_report > 10.0:
                last_report = now
                print(f"  [{phase}] still waiting for: {', '.join(missing)}", flush=True)
        return missing

    def pump(self, seconds: float) -> None:
        """Service the event subscription while an activity runs."""
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
    """The launches, in their own session so no terminal signal reaches them directly."""

    def __init__(self, log_dir: Path) -> None:
        self._log_dir = log_dir
        self._procs: list[tuple[LaunchSpec, subprocess.Popen]] = []

    def start(self, spec: LaunchSpec) -> None:
        log_path = self._log_dir / spec.log_name()
        handle = log_path.open("w")
        print(f"  starting {spec.label()} -> {log_path}", flush=True)
        proc = subprocess.Popen(
            spec.command(),
            stdout=handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            # Its own session: the supervisor decides when these stop, and in
            # which order, rather than a terminal signal reaching all of them.
            start_new_session=True,
        )
        self._procs.append((spec, proc))

    def died(self):
        """A launch that exited on its own, or None."""
        for spec, proc in self._procs:
            if proc.poll() is not None:
                return spec, proc.returncode, self._log_dir / spec.log_name()
        return None

    def stop(self) -> None:
        """Newest first, one at a time: coordination unwinds before its arms go away."""
        for spec, proc in reversed(self._procs):
            if proc.poll() is not None:
                continue
            print(f"  stopping {spec.label()}", flush=True)
            self._signal_group(proc, signal.SIGINT)
            if not self._wait(proc, 20.0):
                print(f"  {spec.label()} did not stop on SIGINT; terminating", flush=True)
                self._signal_group(proc, signal.SIGTERM)
                if not self._wait(proc, 10.0):
                    print(f"  {spec.label()} did not terminate; killing it", flush=True)
                    self._signal_group(proc, signal.SIGKILL)

    @staticmethod
    def _wait(proc: subprocess.Popen, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.2)
        return proc.poll() is not None

    @staticmethod
    def _signal_group(proc: subprocess.Popen, sig) -> None:
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass


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
    """Refuse an impossible resume before anything is sent."""
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


# --- role 1: the supervisor -------------------------------------------------

def _warn_if_a_dropped_connection_would_orphan_the_stack() -> None:
    """Over SSH without tmux, a dropped link leaves the stack running unattended.

    The launches run in their own session so the supervisor decides when they
    stop. The same property means a SIGHUP kills only the supervisor: the
    teardown never runs, and the arms keep a live driver with nobody supervising
    it. The state file is then the only way to find what is still running.
    """
    if os.environ.get("TMUX") or os.environ.get("STY"):
        return
    if not (os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY")):
        return
    print(
        "WARNING: this is an SSH session and not a tmux/screen one. If the\n"
        "         connection drops, this supervisor dies and the launches keep\n"
        "         running with nothing to shut them down. Prefer:\n"
        "             tmux new -A -s demo\n",
        file=sys.stderr,
    )


def run_supervisor(spec: UnitSpec, args) -> int:
    """Bring the stack up in order, then stay alive owning it."""
    existing = running_supervisor(spec.unit)
    if existing is not None:
        print(
            f"a {spec.unit} stack supervisor is already running (pid {existing.pid},\n"
            f"logs {existing.log_dir}). Stop it first:\n"
            f"    ./scripts/stop_demo_stack.py",
            file=sys.stderr,
        )
        return 1

    _warn_if_a_dropped_connection_would_orphan_the_stack()

    log_dir = Path(args.log_dir) if args.log_dir else (
        LOG_ROOT / f"{spec.unit}_{datetime.now():%Y%m%d-%H%M%S}"
    )
    log_dir.mkdir(parents=True, exist_ok=True)

    print(f"== {spec.unit} demo stack")
    print(f"   {spec.description}")
    print(f"   logs: {log_dir}")

    interrupted = _install_shutdown_handler()
    launches = _Launches(log_dir)
    watcher = None
    state = None
    try:
        watcher = _StackWatcher()

        components, coordination = spec.launches
        print("bringing up components")
        launches.start(components)
        missing = watcher.wait_for(
            "components",
            args.timeout_sec,
            services=COMPONENT_SERVICES,
            actions=COMPONENT_ACTIONS + spec.extra_component_actions,
            topics=COMPONENT_TOPICS,
        )
        if missing:
            return _report_not_ready(launches, missing, "components")

        # Only now: the coordinator's action clients wait for their servers at
        # dispatch, so starting it against absent arms fails at the first motion
        # rather than at bring-up.
        print("components ready; bringing up coordination")
        launches.start(coordination)
        missing = watcher.wait_for(
            "coordination",
            args.timeout_sec,
            actions=COORDINATION_ACTIONS + spec.extra_coordination_actions,
        )
        if missing:
            return _report_not_ready(launches, missing, "coordination")

        state = StackState(
            unit=spec.unit,
            stack=spec.stack,
            pid=os.getpid(),
            log_dir=str(log_dir),
            started=datetime.now().isoformat(timespec="seconds"),
            execution_profile=spec.execution_profile,
        )
        state.write()

        print()
        print(f"{spec.unit.upper()} {spec.stack.upper()} STACK READY")
        print(f"  profile  {spec.execution_profile}")
        for name in (COORDINATION_ACTIONS + spec.extra_coordination_actions
                     + COMPONENT_ACTIONS + spec.extra_component_actions):
            print(f"  action   {name}")
        print()
        print("Run activities from another pane; this one stays up. Stop it with")
        print("Ctrl+C here, or ./scripts/stop_demo_stack.py from anywhere.")

        return _supervise(launches, interrupted)
    finally:
        if state is not None:
            state.remove()
        if watcher is not None:
            watcher.close()
        print("shutting the stack down")
        launches.stop()


def _install_shutdown_handler() -> dict:
    """SIGINT and SIGTERM ask for an orderly shutdown instead of unwinding here."""
    flag = {"stop": False}

    def _on_signal(signum, _frame):
        if not flag["stop"]:
            print(f"\nreceived {signal.Signals(signum).name}; shutting down", flush=True)
        flag["stop"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    return flag


def _supervise(launches: _Launches, interrupted: dict) -> int:
    """Stay alive until asked to stop, or until a launch exits on its own."""
    while not interrupted["stop"]:
        dead = launches.died()
        if dead:
            spec, code, log_path = dead
            print(
                f"\n{spec.label()} exited on its own with {code}; see {log_path}",
                file=sys.stderr,
            )
            return 1
        time.sleep(0.5)
    return 0


def _report_not_ready(launches: _Launches, missing: list[str], phase: str) -> int:
    dead = launches.died()
    if dead:
        spec, code, log_path = dead
        print(f"{spec.label()} exited with {code}; see {log_path}", file=sys.stderr)
    print(f"{phase} never became ready; missing: {', '.join(missing)}", file=sys.stderr)
    return 1


# --- role 2: one activity against a running stack ---------------------------

@dataclass(frozen=True)
class ActivitySpec:
    """One operator flow: what it runs, and what it needs to be running against.

    ``unit`` binds a flow to the unit it was written for, so a top-unit fold is
    refused on the bottom unit rather than planned against the wrong arms. It is
    None for a flow that is not specific to one unit.
    """

    name: str
    activity: str
    unit: str | None = None
    stack: str = "demo"
    description: str = ""
    extra_actions: tuple[str, ...] = field(default=())


def run_activity(spec: ActivitySpec, args) -> int:
    """Verify the right stack is up on the right unit, then run one activity."""
    unit = require_unit(spec.unit, args.unit) if spec.unit else resolve_unit(args.unit)
    check_from_id(spec.activity, args.from_id)

    state = running_supervisor(unit)
    if state is None:
        print(
            f"no demo stack is running on the {unit} unit.\n"
            f"Start it first, in its own pane:\n"
            f"    ./scripts/start_demo_stack.py{'' if spec.stack == 'demo' else ' --stack ' + spec.stack}",
            file=sys.stderr,
        )
        return 1
    if state.stack != spec.stack:
        print(
            f"the running stack is '{state.stack}' and this flow needs '{spec.stack}'.\n"
            f"    ./scripts/stop_demo_stack.py\n"
            f"    ./scripts/start_demo_stack.py{'' if spec.stack == 'demo' else ' --stack ' + spec.stack}",
            file=sys.stderr,
        )
        return 1

    print(f"== {spec.name}")
    if spec.description:
        print(f"   {spec.description}")
    print(describe_steps(spec.activity, args.from_id))
    print(f"   stack: {state.stack}, pid {state.pid}, {state.execution_profile}")
    print(f"   logs:  {state.log_dir}")

    watcher = _StackWatcher()
    try:
        missing = watcher.wait_for(
            "stack",
            args.ready_timeout_sec,
            actions=COORDINATION_ACTIONS + spec.extra_actions,
            services=COMPONENT_SERVICES,
            topics=COMPONENT_TOPICS,
        )
        if missing:
            print(
                f"the stack is running but not serving: {', '.join(missing)}\n"
                f"  see {state.log_dir}",
                file=sys.stderr,
            )
            return 1

        if not args.no_prompt:
            try:
                input("\nPress Enter to start, or Ctrl+C to abort: ")
            except (EOFError, KeyboardInterrupt):
                print("\nnot started")
                return 130

        return _execute(spec, args, watcher)
    finally:
        watcher.close()


def _run_activity_command(spec: ActivitySpec, args) -> list[str]:
    command = [
        "ros2", "run", "agx_arm_coordination", "run_activity",
        "--activity", spec.activity,
    ]
    if args.from_id is not None:
        command += ["--from-id", str(args.from_id)]
    if args.metadata_json:
        command += ["--metadata-json", args.metadata_json]
    return command


def _execute(spec: ActivitySpec, args, watcher: _StackWatcher) -> int:
    """Run the activity in the foreground and let it own the interrupt."""
    command = _run_activity_command(spec, args)
    print(f"\n$ {' '.join(command)}\n", flush=True)

    # The terminal's Ctrl+C belongs to run_activity: first press cancels the
    # activity, second escalates to the unit emergency stop. This wrapper must
    # not act on it. The stack is not affected either way — it is another
    # process, and a cancelled activity leaves it up to be resumed against.
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
            "  the stack is still up; after an emergency stop or a bus recovery, "
            "re-arm explicitly before resuming:\n"
            "    ros2 service call /left_arm/clear_fault_lockout std_srvs/srv/Trigger\n"
            "    ros2 service call /right_arm/clear_fault_lockout std_srvs/srv/Trigger\n"
            "    ros2 service call /unit_safety/rearm std_srvs/srv/Trigger",
            file=sys.stderr,
        )
    return code


def _next_from_id(spec: ActivitySpec, watcher: _StackWatcher) -> str:
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


# --- the shared CLIs --------------------------------------------------------

def _add_unit_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--unit", choices=UNIT_NAMES, default=None,
        help=f"override {UNIT_ENV_VAR} for this command",
    )


def supervisor_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    _add_unit_argument(parser)
    parser.add_argument(
        "--stack", choices=STACK_CHOICES, default="demo",
        help="which stack to bring up: the unit's pack/unpack/wave stack (demo), "
             "the tea demo's, or the block restack's",
    )
    parser.add_argument(
        "--grippers", action="store_true",
        help="bottom unit demo stack: duo_gripper instead of duo_arm",
    )
    parser.add_argument(
        "--timeout-sec", type=float, default=120.0,
        help="how long each bring-up phase may take to provide its surfaces",
    )
    parser.add_argument(
        "--log-dir", default="",
        help=f"where to write the launch logs (default: a timestamped dir under {LOG_ROOT})",
    )
    return parser


def activity_parser(description: str, with_speed: bool = False) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    _add_unit_argument(parser)
    parser.add_argument(
        "--from-id", "--from_id", dest="from_id", type=int, default=None, metavar="N",
        help="resume at operator step N (1-based); refused if step N replays a taught path",
    )
    parser.add_argument(
        "--no-prompt", action="store_true",
        help="start immediately instead of waiting for Enter",
    )
    parser.add_argument(
        "--ready-timeout-sec", type=float, default=20.0,
        help="how long to wait for the running stack to answer before giving up",
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
    """Entry point for one activity script."""
    parser = activity_parser(description, with_speed=with_speed)
    args = parser.parse_args()
    spec = spec_or_factory(args) if callable(spec_or_factory) else spec_or_factory
    raise SystemExit(run_activity(spec, args))
