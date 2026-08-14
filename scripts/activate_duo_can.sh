#!/bin/bash
#
# activate_duo_can.sh — bring up all four Duo CAN interfaces, by physical slot.
#
# One bus per device (V02 topology, integration_plan.md constraint C1):
#
#   can_nero_right   right arm    native mttcan  @ c310000.mttcan
#   can_nero_left    left  arm    native mttcan  @ c320000.mttcan
#   hand_right       right hand   PEAK USB-CAN FD @ USB port 1-4.3
#   hand_left        left  hand   PEAK USB-CAN FD @ USB port 1-4.4
#
# Interfaces are matched by **parent device**, not by the kernel's canN index.
# The two hand adapters are identical hardware, so their canN numbers depend on
# enumeration order and can swap between boots; the USB port path cannot. Getting
# this wrong points a hand's commands at the other hand, which is why the mapping
# is anchored to the slot and why the script refuses to guess.
#
# Idempotent: an interface already named and up is reconfigured in place.
#
# Usage:
#   sudo ./scripts/activate_duo_can.sh              # all four
#   sudo ./scripts/activate_duo_can.sh arms         # arms only
#   sudo ./scripts/activate_duo_can.sh hands        # hands only
#   ./scripts/activate_duo_can.sh --show            # report state, change nothing
#
# Verify afterwards with ./scripts/measure_can_baseline.sh — an interface that is
# UP but silent is not the same as a working bus.

set -euo pipefail

# target name -> parent device (physical slot)
declare -A SLOT=(
    [can_nero_right]="c310000.mttcan"
    [can_nero_left]="c320000.mttcan"
    [hand_right]="1-4.3:1.0"
    [hand_left]="1-4.4:1.0"
)
ARMS=(can_nero_right can_nero_left)
HANDS=(hand_right hand_left)

# Validated OmniHand FD timing; the Nero arm accepts classic frames on the same
# interface, so one parameter set serves both device types.
BITRATE=1000000
SAMPLE_POINT=0.8
DBITRATE=5000000
DSAMPLE_POINT=0.8
RESTART_MS=100
ONE_SHOT=on
# Socket TX ring depth. The kernel default of 10 is smaller than a single MIT
# setpoint, which is a burst of seven frames at the control rate onto a bus
# already carrying the arm's own feedback push. Measured on hardware
# (docs/sprint_refactor/reference/sdk_latency_budget.md): raising it to 1000
# roughly halves both the mean per-frame transmit cost and its worst case.
TX_QUEUE_LEN=1000
# The publish/parse thread can stall under GIL pressure, and nothing drains the
# RX socket while it does; a small kernel buffer then drops frames. See
# docs/sprint_refactor/reference/phase0_baseline.md.
RMEM_MAX=4194304

iface_for_slot() {
    local want="$1" dev name
    for dev in /sys/class/net/*; do
        name="$(basename "$dev")"
        [ -e "$dev/type" ] || continue
        [ "$(cat "$dev/type" 2>/dev/null)" = "280" ] || continue   # ARPHRD_CAN
        if [ "$(basename "$(readlink -f "$dev/device" 2>/dev/null)" 2>/dev/null)" = "$want" ]; then
            echo "$name"
            return 0
        fi
    done
    return 1
}

show() {
    printf '%-16s %-18s %-8s %s\n' TARGET SLOT STATE PRESENT-AS
    local target current state
    for target in "${!SLOT[@]}"; do
        if current="$(iface_for_slot "${SLOT[$target]}")"; then
            state="$(ip -br link show "$current" 2>/dev/null | awk '{print $2}')"
            printf '%-16s %-18s %-8s %s\n' "$target" "${SLOT[$target]}" "${state:-?}" "$current"
        else
            printf '%-16s %-18s %-8s %s\n' "$target" "${SLOT[$target]}" MISSING "-"
        fi
    done
}

bring_up() {
    local target="$1" slot="${SLOT[$target]}" current
    if ! current="$(iface_for_slot "$slot")"; then
        echo "  $target: NOT FOUND at $slot — device unplugged, or (arms) the" >&2
        echo "            Jetson 40-pin header is not configured for CAN." >&2
        return 1
    fi

    ip link set "$current" down 2>/dev/null || true
    ip link set "$current" type can \
        bitrate "$BITRATE" sample-point "$SAMPLE_POINT" \
        dbitrate "$DBITRATE" dsample-point "$DSAMPLE_POINT" \
        fd on restart-ms "$RESTART_MS" one-shot "$ONE_SHOT"

    if [ "$current" != "$target" ]; then
        if ip -br link show "$target" >/dev/null 2>&1; then
            echo "  $target: name already taken by another interface — refusing" >&2
            return 1
        fi
        ip link set "$current" name "$target"
    fi
    ip link set "$target" txqueuelen "$TX_QUEUE_LEN" \
        || echo "  warning: could not set txqueuelen on '$target'" >&2
    ip link set "$target" up
    echo "  $target: up on $slot (was $current, txqueuelen=$TX_QUEUE_LEN)"
}

case "${1:---all}" in
    --show|show) show; exit 0 ;;
    arms)  targets=("${ARMS[@]}") ;;
    hands) targets=("${HANDS[@]}") ;;
    --all|all|"") targets=("${ARMS[@]}" "${HANDS[@]}") ;;
    *) echo "usage: $0 [arms|hands|--all|--show]" >&2; exit 2 ;;
esac

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo (this modifies network interfaces)." >&2
    exit 1
fi

current_rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [ "$current_rmem" -lt "$RMEM_MAX" ]; then
    sysctl -q -w net.core.rmem_max="$RMEM_MAX"
    echo "raised net.core.rmem_max to $RMEM_MAX"
fi

failed=0
for target in "${targets[@]}"; do
    bring_up "$target" || failed=1
done

echo
show
exit "$failed"
