#!/usr/bin/env bash
# Per-thread CPU for a running node, over a fixed window.
#
# Exists because "which thread is burning the CPU" was previously unanswerable:
# Python thread names stay inside the interpreter, so every thread showed up
# under the process name. The nodes now name their threads at OS level
# (`agx_arm_ctrl/runtime_metrics.py:name_os_thread`), which makes
# /proc/<pid>/task/*/comm the cheapest honest answer — no profiler attached to
# a node whose timing is the thing under test.
#
# Reports percent of ONE core. Divide by nproc for percent of the machine; the
# two support very different conclusions and the baseline docs quote both.
#
# Usage: bash scripts/measure_thread_cpu.sh <pid-or-pattern> [seconds]
set -uo pipefail

TARGET="${1:?usage: measure_thread_cpu.sh <pid-or-pattern> [seconds]}"
WINDOW="${2:-10}"

if [[ "$TARGET" =~ ^[0-9]+$ ]]; then
    PID="$TARGET"
else
    PID=$(pgrep -f "$TARGET" | head -1)
fi
[ -z "${PID:-}" ] && { echo "no process matching '$TARGET'" >&2; exit 1; }
[ -d "/proc/$PID" ] || { echo "pid $PID is not running" >&2; exit 1; }

TICKS=$(getconf CLK_TCK)
CORES=$(nproc)

snapshot() {
    for task in /proc/"$PID"/task/*; do
        [ -r "$task/stat" ] || continue
        echo "$(basename "$task") $(cat "$task/comm" 2>/dev/null) $(awk '{print $14+$15}' "$task/stat" 2>/dev/null)"
    done
}

before=$(snapshot)
sleep "$WINDOW"
after=$(snapshot)

echo "pid $PID, ${WINDOW}s window, ${CORES} cores"
printf "%-18s %12s %12s\n" "thread" "% of core" "% of machine"
join <(echo "$before" | sort) <(echo "$after" | sort) \
    | awk -v ticks="$TICKS" -v w="$WINDOW" -v cores="$CORES" '
        { delta = $5 - $3
          if (delta <= 0) next
          core = delta / ticks / w * 100
          printf "%-18s %11.1f%% %11.1f%%\n", $2, core, core / cores }' \
    | sort -k2 -rn

total=$(join <(echo "$before" | sort) <(echo "$after" | sort) \
    | awk -v ticks="$TICKS" -v w="$WINDOW" '{ d=$5-$3; if (d>0) s+=d } END { printf "%.1f", s/ticks/w*100 }')
printf "%-18s %11.1f%% %11.1f%%\n" "TOTAL" "$total" "$(echo "$total $CORES" | awk '{print $1/$2}')"
