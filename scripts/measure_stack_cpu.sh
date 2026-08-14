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
# Processes are selected by their executable, never by matching the whole
# command line: a `pgrep -f` pattern also matches the measuring shell, and a
# greedy match across a long command line captures spaces and silently corrupts
# the columns. That mis-reported one thread as three before it was fixed.
#
# Usage: bash scripts/measure_stack_cpu.sh [seconds] [--threads]
set -uo pipefail

WINDOW="${1:-10}"
SHOW_THREADS="${2:-}"

TICKS=$(getconf CLK_TCK)
CORES=$(nproc)
SELF=$$

# A ROS node of ours is a process whose argv[1] is an executable installed into
# this workspace, or a known ROS tool. Shells, pgrep and the sampler itself are
# excluded by construction rather than by pattern.
ros_pids() {
    local pid comm exe
    for pid in $(ls /proc | grep -E '^[0-9]+$'); do
        [ "$pid" = "$SELF" ] && continue
        [ -r "/proc/$pid/cmdline" ] || continue
        comm=$(cat "/proc/$pid/comm" 2>/dev/null) || continue
        case "$comm" in
            bash|sh|dash|pgrep|sleep|awk|sed|grep|ps|top) continue ;;
        esac
        exe=$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | sed -n 2p)
        case "$exe" in
            */agx_arm_ros/install/*|*/opt/ros/*/bin/ros2) echo "$pid" ;;
            *) case "$comm" in
                   rviz2|move_group|robot_state_publisher|static_transform*) echo "$pid" ;;
               esac ;;
        esac
    done
}

PIDS=$(ros_pids)
[ -z "$PIDS" ] && { echo "no workspace ROS nodes running" >&2; exit 1; }

node_name() {  # pid -> a stable, whitespace-free label
    local pid="$1" exe
    exe=$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | sed -n 2p)
    case "$exe" in
        */bin/ros2) echo "ros2-launch" ;;
        */*)        basename "$exe" ;;
        *)          cat "/proc/$pid/comm" 2>/dev/null || echo "pid$pid" ;;
    esac
}

snapshot() {
    local pid task
    for pid in $PIDS; do
        [ -d "/proc/$pid/task" ] || continue
        local name; name=$(node_name "$pid")
        for task in /proc/"$pid"/task/*; do
            [ -r "$task/stat" ] || continue
            echo "$pid:$(basename "$task")|$name|$(cat "$task/comm" 2>/dev/null)|$(awk '{print $14+$15}' "$task/stat")"
        done
    done
}

echo "=== stack CPU over ${WINDOW}s, ${CORES} cores ==="
echo "-- non-ROS load at start (percent of one core each; named, not ignored) --"
ps -eo pid,comm,pcpu --sort=-pcpu | awk -v skip="$(echo $PIDS | tr ' ' ',')" '
    BEGIN { n=split(skip, a, ","); for (i=1;i<=n;i++) ros[a[i]]=1 }
    NR>1 && $3>1.0 && !($1 in ros) { printf "   %-18s %5.1f%%\n", $2, $3 }' | head -6

before=$(snapshot)
sleep "$WINDOW"
after=$(snapshot)

joined=$(join -t'|' <(echo "$before" | sort -t'|' -k1,1) <(echo "$after" | sort -t'|' -k1,1) \
    | awk -F'|' -v ticks="$TICKS" -v w="$WINDOW" '
        { delta = $7 - $4; if (delta <= 0) next
          printf "%s|%s|%.2f\n", $2, $3, delta / ticks / w * 100 }')

echo
echo "-- per node (percent of ONE core: the ceiling that actually binds) --"
echo "$joined" | awk -F'|' -v cores="$CORES" '
    { n[$1] += $3 }
    END { for (k in n) printf "   %-30s %7.1f%% of a core  %5.1f%% of machine\n", k, n[k], n[k]/cores }' \
    | sort -k2 -rn

if [ "$SHOW_THREADS" = "--threads" ]; then
    echo
    echo "-- per thread --"
    echo "$joined" | awk -F'|' '{ t[$1"  "$2] += $3 }
        END { for (k in t) printf "   %-46s %7.1f%%\n", k, t[k] }' | sort -k3 -rn | head -22
fi

echo
echo "$joined" | awk -F'|' -v cores="$CORES" '{ s += $3 }
    END { printf "TOTAL ROS: %.1f%% of a core (%.1f%% of machine)\n", s, s/cores }'
