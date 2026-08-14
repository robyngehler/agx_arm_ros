#!/usr/bin/env bash
# Per-node and per-thread CPU for a whole running stack, over a fixed window.
#
# Point-measuring one node answers the wrong question. What binds is not the
# machine — a 12-core Jetson has headroom — but saturation *per process*: these
# are GIL-bound Python nodes, so ~100 % of ONE core is the practical ceiling for
# a single node no matter how many cores are idle. This reports percent of one
# core first for that reason, with percent of machine alongside.
#
# Desktop load is reported separately rather than ignored. A browser and an
# editor are worth ~20 % of a core on this host, and a measurement that does not
# name them lets that drift into the robot's numbers.
#
# Threads are named at OS level by the nodes themselves
# (`agx_arm_ctrl/runtime_metrics.py:name_os_thread`); anything still showing the
# process name is a thread nobody has labelled yet, not an anonymous cost.
#
# Usage: bash scripts/measure_stack_cpu.sh [seconds] [--threads]
set -uo pipefail

WINDOW="${1:-10}"
SHOW_THREADS="${2:-}"

TICKS=$(getconf CLK_TCK)
CORES=$(nproc)

# ROS nodes launched from this workspace, plus the coordinator and any vendor
# bridges. Matched on the install path so a stray editor process cannot join.
ros_pids() {
    pgrep -f "install/.*/lib/|ros2 launch|rviz2|move_group|robot_state_publisher" 2>/dev/null \
        | sort -u
}

snapshot() {
    for pid in $(ros_pids); do
        [ -r "/proc/$pid/stat" ] || continue
        local name
        name=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null \
            | grep -oE "[a-z_0-9]+\.launch\.py|lib/[a-z_0-9]+/[a-z_0-9]+|rviz2|move_group" \
            | tail -1 | sed 's#.*/##')
        [ -z "$name" ] && name="pid$pid"
        for task in /proc/"$pid"/task/*; do
            [ -r "$task/stat" ] || continue
            echo "$pid:$(basename "$task") $name/$(cat "$task/comm" 2>/dev/null) $(awk '{print $14+$15}' "$task/stat")"
        done
    done
}

echo "=== stack CPU over ${WINDOW}s, ${CORES} cores ==="
echo "desktop load at start (percent of one core each):"
ps -eo comm,pcpu --sort=-pcpu | awk 'NR>1 && $2>1.0 && $1 !~ /python3|ros2|rviz/ {printf "  %-16s %5.1f%%\n", $1, $2}' | head -6

before=$(snapshot)
sleep "$WINDOW"
after=$(snapshot)

report=$(join <(echo "$before" | sort) <(echo "$after" | sort) \
    | awk -v ticks="$TICKS" -v w="$WINDOW" '
        { delta = $5 - $3; if (delta <= 0) next
          split($2, parts, "/")
          core = delta / ticks / w * 100
          per_node[parts[1]] += core
          printf "%s\t%s\t%.1f\n", parts[1], parts[2], core }
        END { for (n in per_node) printf "NODE\t%s\t%.1f\n", n, per_node[n] }')

echo
echo "per node (percent of ONE core — the ceiling that actually binds):"
echo "$report" | awk -F'\t' -v cores="$CORES" '$1=="NODE" {printf "  %-34s %7.1f%% of a core  %5.1f%% of machine\n", $2, $3, $3/cores}' | sort -k2 -rn

if [ "$SHOW_THREADS" = "--threads" ]; then
    echo
    echo "per thread:"
    echo "$report" | awk -F'\t' '$1!="NODE" {printf "  %-24s %-18s %7.1f%%\n", $1, $2, $3}' | sort -k3 -rn | head -25
fi

echo
echo "$report" | awk -F'\t' -v cores="$CORES" '$1=="NODE" {s+=$3} END {printf "TOTAL ROS: %.1f%% of a core (%.1f%% of machine)\n", s, s/cores}'
