#!/bin/bash
#
# measure_can_baseline.sh — per-interface CAN traffic and host CPU over a window.
#
# The Phase 0 baseline needs rates, not the cumulative counters `ip -s` prints:
# a bus that faulted for an hour last week and one saturating right now look
# identical in totals. This samples every CAN interface twice and reports the
# delta per second, alongside per-process CPU for the ROS nodes in the graph.
#
# Purely passive: it sends nothing on any bus and touches no device. Safe to run
# next to live hardware, and safe while an arm is enabled.
#
# Usage:
#   ./scripts/measure_can_baseline.sh [seconds] [label]
#
# Reference: docs/sprint_refactor/planning/integration_plan.md (0D, 0E)

set -euo pipefail

WINDOW="${1:-10}"
LABEL="${2:-baseline}"

interfaces() {
    ip -br link show type can 2>/dev/null | awk '{print $1}'
}

# RX/TX packets and drops for one interface, as a single line of numbers.
counters() {
    local iface="$1"
    ip -s -d link show "$iface" | awk '
        /RX:/ { getline; rxb=$1; rxp=$2; rxe=$3; rxd=$4 }
        /TX:/ { getline; txb=$1; txp=$2; txe=$3; txd=$4 }
        /re-started/ { getline; restarts=$1; buserr=$2; arbit=$3; busoff=$6 }
        END { print rxp, txp, rxd, txd, rxe, txe, restarts+0, busoff+0, arbit+0 }
    '
}

mapfile -t IFACES < <(interfaces)
if [ "${#IFACES[@]}" -eq 0 ]; then
    echo "no CAN interfaces are up" >&2
    exit 1
fi

echo "=== CAN baseline: ${LABEL} (${WINDOW}s window, $(date -Is)) ==="

declare -A BEFORE
for iface in "${IFACES[@]}"; do
    BEFORE["$iface"]="$(counters "$iface")"
done

# CPU snapshot over the same window, so traffic and load are comparable.
CPU_LOG="$(mktemp)"
if command -v pidstat >/dev/null 2>&1; then
    (pidstat -u -h -C 'omnihand|agx_arm|coordinator|mit_controller|move_group' \
        "$WINDOW" 1 >"$CPU_LOG" 2>/dev/null || true) &
    CPU_PID=$!
else
    CPU_PID=""
fi

sleep "$WINDOW"
[ -n "$CPU_PID" ] && wait "$CPU_PID" 2>/dev/null || true

printf '\n%-16s %10s %10s %10s %8s %8s  %s\n' \
    IFACE RX/s TX/s DROP/s BUS-OFF RESTART STATE
for iface in "${IFACES[@]}"; do
    read -r b_rxp b_txp b_rxd _b_txd _b_rxe _b_txe b_rst b_off _b_arb <<<"${BEFORE[$iface]}"
    read -r a_rxp a_txp a_rxd _a_txd _a_rxe _a_txe a_rst a_off _a_arb <<<"$(counters "$iface")"

    rx=$(( (a_rxp - b_rxp) / WINDOW ))
    tx=$(( (a_txp - b_txp) / WINDOW ))
    drop=$(( (a_rxd - b_rxd) / WINDOW ))
    off=$(( a_off - b_off ))
    rst=$(( a_rst - b_rst ))

    # A bus is only "ok" if it moved frames without losing any. Silence is not
    # health: an arm that answers nothing looks identical to one powered off.
    if [ "$off" -gt 0 ] || [ "$rst" -gt 0 ]; then
        state="FAULTING (bus-off/restart during the window)"
    elif [ "$drop" -gt 0 ]; then
        state="DROPPING (host cannot drain fast enough)"
    elif [ "$rx" -eq 0 ] && [ "$tx" -eq 0 ]; then
        state="SILENT (no traffic — device off, or push disabled)"
    elif [ "$rx" -eq 0 ]; then
        state="TX ONLY (no answers coming back)"
    else
        state="ok"
    fi

    printf '%-16s %10d %10d %10d %8d %8d  %s\n' \
        "$iface" "$rx" "$tx" "$drop" "$off" "$rst" "$state"
done

if [ -s "$CPU_LOG" ]; then
    echo
    echo "=== process CPU over the same window ==="
    cat "$CPU_LOG"
fi
rm -f "$CPU_LOG"
