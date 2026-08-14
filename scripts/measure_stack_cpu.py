#!/usr/bin/env python3
"""Per-node and per-thread CPU for a whole running stack, over a fixed window.

A Python port of `measure_stack_cpu.sh`, and the one to prefer. The shell version
paces itself with `sleep`, which some sandboxes neuter — and a skipped sleep does
not fail, it silently turns a 15 s window into a 0 s one and reports whatever
noise is left. That has already produced one wrong measurement in this repo.

What binds is not the machine — a 12-core Jetson has headroom — but saturation
*per process*: these are GIL-bound Python nodes, so ~100 % of ONE core is the
practical ceiling for a single node however many cores idle. Percent of one core
is therefore reported first, with percent of machine alongside.

Per-thread output matters more than it looks. A node's cost can sit entirely in
a thread nobody wrote: a vendor SDK's internal reader loop belongs to the process
that opened the session, and no amount of profiling our own call sites will find
it. Threads named at OS level (`runtime_metrics.name_os_thread`) identify
themselves; anything still showing the process name is unlabelled, not anonymous.

Desktop load is reported separately rather than ignored: a browser and an editor
are worth ~20 % of a core on this host, and a measurement that does not name them
lets that drift into the robot's numbers.

Usage: python3 scripts/measure_stack_cpu.py [seconds] [--threads] [--top N]
"""

from __future__ import annotations

import argparse
import os
import time

TICKS = os.sysconf("SC_CLK_TCK")
CORES = os.cpu_count() or 1
SHELLS = {"bash", "sh", "dash", "pgrep", "sleep", "awk", "sed", "grep", "ps", "top"}
EXTRA_NODES = {"rviz2", "move_group", "robot_state_publisher"}


def _read(path: str) -> str:
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        return ""


def _argv(pid: int) -> list[str]:
    return [part for part in _read(f"/proc/{pid}/cmdline").split("\0") if part]


def ros_pids() -> dict[int, str]:
    """Workspace ROS nodes, selected by executable rather than by pattern.

    A `pgrep -f` pattern also matches the measuring process itself, and a greedy
    match across a long command line captures spaces and shifts every column
    after it — that mis-reported one thread as three before it was fixed.
    """
    found: dict[int, str] = {}
    self_pid = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == self_pid:
            continue
        comm = _read(f"/proc/{pid}/comm").strip()
        if comm in SHELLS:
            continue
        argv = _argv(pid)
        executable = argv[1] if len(argv) > 1 else ""
        if "/agx_arm_ros/install/" in executable:
            found[pid] = os.path.basename(executable)
        elif executable.endswith("/bin/ros2"):
            found[pid] = "ros2-launch"
        elif comm in EXTRA_NODES or comm.startswith("static_transform"):
            found[pid] = comm
    return found


def snapshot(pids: dict[int, str]) -> dict[tuple[int, int], tuple[str, str, int]]:
    """(pid, tid) -> (node name, thread name, cpu ticks)."""
    taken: dict[tuple[int, int], tuple[str, str, int]] = {}
    for pid, name in pids.items():
        try:
            tids = os.listdir(f"/proc/{pid}/task")
        except OSError:
            continue
        for tid in tids:
            stat = _read(f"/proc/{pid}/task/{tid}/stat")
            if not stat:
                continue
            # The comm field is parenthesised and may contain spaces, so split
            # on the LAST ')' rather than tokenising the whole line.
            fields = stat.rsplit(")", 1)[1].split()
            cpu = int(fields[11]) + int(fields[12])
            thread = _read(f"/proc/{pid}/task/{tid}/comm").strip()
            taken[(pid, int(tid))] = (name, thread, cpu)
    return taken


def desktop_load(ros: set[int]) -> list[tuple[str, float]]:
    """Non-ROS processes worth more than 1 % of a core, sampled over 1 s."""
    def cpu_of(pid: int) -> int:
        stat = _read(f"/proc/{pid}/stat")
        if not stat:
            return 0
        fields = stat.rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])

    candidates = {
        int(e): _read(f"/proc/{e}/comm").strip()
        for e in os.listdir("/proc")
        if e.isdigit() and int(e) not in ros
    }
    before = {pid: cpu_of(pid) for pid in candidates}
    time.sleep(1.0)
    rows = []
    for pid, name in candidates.items():
        percent = (cpu_of(pid) - before.get(pid, 0)) / TICKS * 100.0
        if percent > 1.0 and name:
            rows.append((name, percent))
    return sorted(rows, key=lambda row: -row[1])[:6]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("window", nargs="?", type=float, default=10.0)
    parser.add_argument("--threads", action="store_true")
    parser.add_argument("--top", type=int, default=22)
    args = parser.parse_args()

    pids = ros_pids()
    if not pids:
        print("no workspace ROS nodes running")
        return 1

    print(f"=== stack CPU over {args.window:g}s, {CORES} cores ===")
    print("-- non-ROS load (percent of one core each; named, not ignored) --")
    for name, percent in desktop_load(set(pids)):
        print(f"   {name:<18} {percent:5.1f}%")

    before = snapshot(pids)
    time.sleep(args.window)
    after = snapshot(pids)

    per_node: dict[str, float] = {}
    per_thread: dict[str, float] = {}
    for key, (node, thread, cpu) in after.items():
        if key not in before:
            continue
        delta = cpu - before[key][2]
        if delta <= 0:
            continue
        percent = delta / TICKS / args.window * 100.0
        per_node[node] = per_node.get(node, 0.0) + percent
        per_thread[f"{node}  {thread}"] = per_thread.get(f"{node}  {thread}", 0.0) + percent

    print()
    print("-- per node (percent of ONE core: the ceiling that actually binds) --")
    for node, percent in sorted(per_node.items(), key=lambda row: -row[1]):
        print(f"   {node:<30} {percent:7.1f}% of a core  {percent / CORES:5.1f}% of machine")

    if args.threads:
        print()
        print("-- per thread --")
        for label, percent in sorted(per_thread.items(), key=lambda r: -r[1])[: args.top]:
            print(f"   {label:<46} {percent:7.1f}%")

    total = sum(per_node.values())
    print()
    print(f"TOTAL ROS: {total:.1f}% of a core ({total / CORES:.1f}% of machine)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
