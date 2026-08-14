#!/usr/bin/env bash
# Sample CAN RX state across a provoked fault, to answer what the kernel loses
# while nothing is draining the socket.
#
# Who drains the socket matters and was long assumed wrong: it is the vendor
# SDK's own reader thread (`driver_context._read_loop` on an arm), not the
# driver's publish loop. So a stalled publisher does not stop the drain — what
# stops it is recovery calling `stop_th()` while it tears the link down, and the
# question is how many frames are lost between the bus returning and the reader
# restarting.
#
# On a HAND the reader is the vendor SDK's C++ receive thread, and until
# 2026-08-14 it drained by spinning on a non-blocking read() at 100 % of a core
# (fixed in the tracked fork; see docs/assets/omnihand/). It now waits in poll(),
# so a hand's drain no longer competes for CPU with whatever else is running.
# A link-down measures the fault path, not an overflow: a downed link delivers
# nothing to buffer. The overflow-capable case is bus up with the reader stopped.
#
# The per-socket buffer is `net.core.rmem_default`, NOT `net.core.rmem_max`:
# python-can never calls setsockopt(SO_RCVBUF), so raising only the ceiling
# changes nothing. Both are printed here so the run records what was in force.
#
# Usage: bash scripts/measure_rx_drain.sh <iface> [seconds] [interval]
#   then, from another shell while it runs:
#     sudo ip link set <iface> down; sleep 15; sudo ip link set <iface> up
set -uo pipefail

IFACE="${1:?usage: measure_rx_drain.sh <iface> [seconds] [interval]}"
WINDOW="${2:-60}"
INTERVAL="${3:-0.2}"

echo "# iface=$IFACE window=${WINDOW}s interval=${INTERVAL}s"
echo "# rmem_default=$(sysctl -n net.core.rmem_default 2>/dev/null) (the buffer a socket actually gets)"
echo "# rmem_max=$(sysctl -n net.core.rmem_max 2>/dev/null) (a ceiling nothing requests)"
echo "# txqueuelen=$(cat /sys/class/net/"$IFACE"/tx_queue_len 2>/dev/null)"
echo "t_s,state,rx_packets,rx_dropped,rx_missed,rx_errors,bus_off,restarts"

start=$(date +%s.%N)
end=$(echo "$start + $WINDOW" | bc)

while (( $(echo "$(date +%s.%N) < $end" | bc) )); do
    now=$(date +%s.%N)
    state=$(ip -br link show "$IFACE" 2>/dev/null | awk '{print $2}')
    [ -z "$state" ] && state="ABSENT"
    read -r pkts drops missed errs <<<"$(ip -s link show "$IFACE" 2>/dev/null \
        | awk '/RX:/{getline; print $2, $4, $5, $3}')"
    read -r restarts busoff <<<"$(ip -s -d link show "$IFACE" 2>/dev/null \
        | awk '/re-started/{getline; print $1, $6}')"
    printf "%.1f,%s,%s,%s,%s,%s,%s,%s\n" \
        "$(echo "$now - $start" | bc)" "$state" \
        "${pkts:-NA}" "${drops:-NA}" "${missed:-NA}" "${errs:-NA}" \
        "${busoff:-NA}" "${restarts:-NA}"
    sleep "$INTERVAL"
done
