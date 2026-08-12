"""L2 integration harness: one activity end to end, no hardware.

This is the regression net the V02 refactor is built on. Every phase rewrites
ownership across the coordinator, the arm driver and the hand stack, and until
now nothing exercised those three together — the existing tests drive each
node's state machine in isolation via ``__new__``, which cannot catch a contract
that drifts *between* nodes.

What runs here is a real ROS graph:

* real ``omnihand_bridge`` per side on the **mock backend** (no CAN, no SDK);
* real ``omnihand_skill_controller`` per side;
* real ``coordinator`` driving ``hands_open_release_v1`` — the activity the repo
  already documents as completing against the mock backend;
* an :mod:`l2_arm_double` stand-in for the arm driver, which has no mock backend
  of its own.

Only the arm hardware is faked. The coordinator's dispatch, the sync barriers,
the hand-window handoff and the skill-to-bridge command path are the real code.

**Isolation.** The harness runs in its own ``ROS_DOMAIN_ID`` and refuses to
start if that domain is already populated. This machine routinely has live
hardware nodes on the default domain, and a harness that spawns identically
named nodes into that graph is a hazard, not a test.

Test level: **L2** (see the ``test-ladder`` skill). It proves wiring and
ordering. It proves nothing about CAN timing, CPU, or motion — those need L3.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

ACTIVITY = "hands_open_release_v1"
SIDES = ("left", "right")

# Never the ambient domain. See the isolation note in the module docstring.
L2_ROS_DOMAIN_ID = "77"

# The mock hand backend answers immediately, but the coordinator still waits on
# action servers and service discovery; keep the budget generous enough that a
# loaded Jetson does not produce a false failure.
DISCOVERY_TIMEOUT_S = 40.0
ACTIVITY_TIMEOUT_S = 90.0


def _ros_available() -> bool:
    return shutil.which("ros2") is not None and bool(os.environ.get("ROS_DISTRO"))


pytestmark = pytest.mark.skipif(
    not _ros_available(),
    reason="L2 needs a sourced ROS environment with the workspace overlay installed",
)


def _l2_env() -> dict:
    """Environment for every harness process and every ros2 CLI call."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = L2_ROS_DOMAIN_ID
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    return env


def _ros2_list(kind: str) -> set[str]:
    try:
        out = subprocess.run(
            ["ros2", kind, "list"],
            capture_output=True, text=True, timeout=20.0, env=_l2_env(),
        )
    except subprocess.TimeoutExpired:
        return set()
    return {line.strip() for line in out.stdout.splitlines() if line.strip()}


class _Graph:
    """Starts the node processes and tears them down deterministically."""

    def __init__(self, log_dir: Path) -> None:
        self._procs: list[tuple[str, subprocess.Popen]] = []
        self._logs: dict[str, Path] = {}
        self._handles: list = []
        self._log_dir = log_dir

    def spawn(self, label: str, argv: list[str]) -> None:
        # Redirect to a file rather than a pipe: assertions read node output
        # while the graph is still running, and reading a live pipe blocks.
        log_path = self._log_dir / f"{label}.log"
        handle = open(log_path, "w", encoding="utf-8")
        self._handles.append(handle)
        proc = subprocess.Popen(
            argv,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=_l2_env(),
        )
        self._procs.append((label, proc))
        self._logs[label] = log_path

    def output(self, label: str) -> str:
        path = self._logs.get(label)
        if path is None or not path.exists():
            return ""
        return path.read_text(encoding="utf-8", errors="replace")

    def assert_all_alive(self) -> None:
        for label, proc in self._procs:
            if proc.poll() is not None:
                raise AssertionError(
                    f"{label} exited early with code {proc.returncode}:\n"
                    f"{self.output(label)[-3000:]}"
                )

    def shutdown(self) -> None:
        """Escalate until nothing is left.

        SIGINT alone is not enough: the coordinator installs its own handler so
        Ctrl+C can unwind a running activity, and with no activity in flight the
        process can keep spinning. A harness that leaks a node poisons the next
        run's domain guard, so the escalation is unconditional.
        """
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            alive = [(lbl, p) for lbl, p in self._procs if p.poll() is None]
            if not alive:
                break
            for _label, proc in reversed(alive):
                try:
                    os.killpg(os.getpgid(proc.pid), sig)
                except (ProcessLookupError, PermissionError):
                    pass
            deadline = time.monotonic() + (8.0 if sig != signal.SIGKILL else 5.0)
            for _label, proc in reversed(alive):
                try:
                    proc.wait(timeout=max(0.0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
        for handle in self._handles:
            handle.close()


def _hand_side_argv(side: str) -> list[tuple[str, list[str]]]:
    namespace = f"{side}_hand"
    bridge = [
        "ros2", "run", "agx_arm_ctrl", "omnihand_bridge",
        "--ros-args",
        "-r", f"__ns:=/{namespace}",
        "-p", "backend_type:=mock",
        "-p", f"omnihand_type:={side}",
        # Idle rates only; the mock backend does no I/O, but keeping them low
        # makes the harness cheap enough to run on the Jetson under load.
        "-p", "pub_rate:=20.0",
        "-p", "joint_read_rate:=10.0",
    ]
    skill = [
        "ros2", "run", "agx_arm_ctrl", "omnihand_skill_controller",
        "--ros-args",
        "-r", f"__ns:=/{namespace}",
        "-p", f"omnihand_type:={side}",
    ]
    return [(f"{side}_bridge", bridge), (f"{side}_skill", skill)]


def _assert_domain_is_empty() -> None:
    """Refuse to run if anything already lives in the L2 domain.

    A populated test domain means either a leaked harness from an earlier run or
    a real deployment misconfigured onto it. Both make the results meaningless
    and, in the second case, put live hardware in reach of the harness.
    """
    nodes = _ros2_list("node")
    if nodes:
        raise AssertionError(
            f"ROS_DOMAIN_ID={L2_ROS_DOMAIN_ID} is not empty: {sorted(nodes)}. "
            "Clear it before running L2; the harness will not share a graph."
        )


@pytest.fixture(scope="module")
def graph(tmp_path_factory):
    log_dir = tmp_path_factory.mktemp("l2")
    call_log = log_dir / "arm_double_calls.log"
    call_log.touch()

    g = _Graph(log_dir)
    g.call_log = call_log  # type: ignore[attr-defined]
    try:
        _assert_domain_is_empty()

        for side in SIDES:
            for label, argv in _hand_side_argv(side):
                g.spawn(label, argv)

        g.spawn(
            "arm_double",
            [
                "python3", str(Path(__file__).parent / "l2_arm_double.py"),
                "--ros-args", "-p", f"call_log_path:={call_log}",
            ],
        )
        g.spawn(
            "coordinator",
            [
                "ros2", "run", "agx_arm_coordination", "coordinator",
                "--ros-args",
                # Hands-only activity, so the arm path is never planned; dry run
                # keeps MoveIt out of the harness entirely.
                "-p", "arm_dry_run:=true",
                "-p", "handoff_enabled:=true",
            ],
        )

        _wait_for_graph(g)
        yield g
    finally:
        g.shutdown()


def _wait_for_graph(g: _Graph) -> None:
    """Block until the coordinator's children are discoverable."""
    required_actions = [f"/{side}_hand/perform" for side in SIDES]
    required_services = [f"/{side}_arm/prepare_hand_window" for side in SIDES]

    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    missing: list[str] = []
    while time.monotonic() < deadline:
        g.assert_all_alive()
        actions = _ros2_list("action")
        services = _ros2_list("service")
        missing = [a for a in required_actions if a not in actions]
        missing += [s for s in required_services if s not in services]
        if not missing:
            return
        time.sleep(1.0)
    raise AssertionError(
        f"graph did not come up within {DISCOVERY_TIMEOUT_S:.0f}s; missing: {missing}\n"
        f"--- coordinator ---\n{g.output('coordinator')[-2000:]}"
    )


def test_activity_completes_on_mock_backend(graph):
    """The coordinator drives a hand activity to success without hardware.

    Guards the whole dispatch chain: catalogue load, graph validation, sync
    barriers, hand-window handoff, PerformAction dispatch, and the skill
    controller's command path into the bridge.
    """
    result = subprocess.run(
        ["ros2", "run", "agx_arm_coordination", "run_activity",
         "--activity", ACTIVITY,
         "--timeout-sec", str(int(ACTIVITY_TIMEOUT_S - 20))],
        capture_output=True, text=True, timeout=ACTIVITY_TIMEOUT_S, env=_l2_env(),
    )
    combined = f"{result.stdout}\n{result.stderr}"

    graph.assert_all_alive()
    assert result.returncode == 0, (
        f"activity client failed (rc={result.returncode})\n"
        f"--- client ---\n{combined[-2000:]}\n"
        f"--- coordinator ---\n{graph.output('coordinator')[-2000:]}"
    )
    assert "success" in combined.lower(), (
        f"activity did not report success:\n{combined[-2000:]}"
    )


def test_hand_window_is_bracketed_per_side(graph):
    """Every hand action is bracketed by prepare/resume on its own side.

    This is the ordering the four-bus topology makes optional (constraint C1):
    once arm and hand no longer share a bus, the bracket becomes degraded-mode
    behaviour. Pinning it here means phase 2B has to change this test
    deliberately rather than silently drop the guarantee.
    """
    calls = [
        line.strip()
        for line in graph.call_log.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert calls, "the arm double recorded no calls at all"

    for side in SIDES:
        side_calls = [c for c in calls if c.startswith(f"{side}:")]
        prepares = side_calls.count(f"{side}:prepare_hand_window")
        resumes = side_calls.count(f"{side}:resume_arm_control")
        assert prepares > 0, f"no prepare_hand_window for {side}: {calls}"
        assert prepares == resumes, (
            f"{side} left a hand window open: {prepares} prepare(s) vs "
            f"{resumes} resume(s) — {side_calls}"
        )
        assert side_calls[0] == f"{side}:prepare_hand_window", (
            f"{side} did something before opening its window: {side_calls}"
        )
        assert side_calls[-1] == f"{side}:resume_arm_control", (
            f"{side} did not end on resume_arm_control: {side_calls}"
        )


def test_a_second_concurrent_activity_is_refused(graph):
    """One activity owns the unit at a time (integration plan 1C).

    The coordinator used to accept every goal unconditionally on a reentrant
    callback group, so two overlapping requests would both have been dispatched
    against the same arms. The guard is deliberately in place *before* the
    parallel-operation work, which multiplies the ways two activities can
    interleave.

    Two separate client processes cannot prove this: the mock activity finishes
    in under a second and process startup jitter is larger than that, so the
    goals do not overlap. The probe sends the second goal from the same process
    the moment the first is accepted.

    Placed last in the module on purpose: it runs an extra activity, and the
    call-log assertions above read the arm double's whole history.
    """
    probe = subprocess.run(
        ["python3", str(Path(__file__).parent / "l2_double_goal.py"),
         "--activity", ACTIVITY,
         "--timeout-sec", str(int(ACTIVITY_TIMEOUT_S - 20))],
        capture_output=True, text=True, timeout=ACTIVITY_TIMEOUT_S, env=_l2_env(),
    )
    graph.assert_all_alive()
    assert probe.returncode == 0, (
        f"double-goal probe failed (rc={probe.returncode})\n"
        f"--- probe ---\n{probe.stdout[-1500:]}\n{probe.stderr[-1500:]}"
    )
    report = json.loads(probe.stdout.strip().splitlines()[-1])
    context = (
        f"\nreport={report}\n"
        f"--- coordinator ---\n{graph.output('coordinator')[-2000:]}"
    )

    assert report["first"].get("success") is True, (
        f"the first activity did not run{context}"
    )
    second = report["second"]
    assert second.get("success") is not True, (
        f"both concurrent activities ran; the unit accepted two commanders{context}"
    )
    # Refused at the door or refused by the claim — both are correct, and the
    # refusal has to carry a reason either way.
    if second.get("accepted"):
        assert "already running" in second.get("message", ""), (
            f"the refused activity gave no reason{context}"
        )
