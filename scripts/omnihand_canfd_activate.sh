#!/bin/bash

set -u

# USB CAN FD adapter activation for the OmniHand.
# NOTE: the current/standard OmniHand transport is the Jetson NATIVE mttcan side
# bus via scripts/activate_native_can.sh (5 Mbit transceiver, shared with the
# arm). Use this script only for a separate USB CAN FD adapter. Defaults match
# the validated vendor timing: 1M/5M, 0.8 sample points, one-shot.
TARGET_NAME="${1:-can_omnihand}"
ARBITRATION_BITRATE="${2:-1000000}"
DATA_BITRATE="${3:-5000000}"
SELECTOR="${4:-}"
RESTART_MS="${5:-100}"
TX_QUEUE_LEN="${6:-256}"
SAMPLE_POINT="${7:-0.8}"
DSAMPLE_POINT="${8:-0.8}"
ONE_SHOT="${9:-on}"

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Error: required command '$1' is not installed."
        exit 1
    fi
}

require_command ip
require_command ethtool

mapfile -t CAN_INTERFACES < <(ip -br link show type can | awk '{print $1}')

print_interfaces() {
    local iface
    for iface in "${CAN_INTERFACES[@]}"; do
        local bus_info
        bus_info=$(ethtool -i "$iface" 2>/dev/null | awk '/bus-info/ {print $2}')
        echo "- $iface (${bus_info:-no-bus-info})"
    done
}

if [ "${#CAN_INTERFACES[@]}" -eq 0 ]; then
    echo "Error: no Linux CAN interfaces are present."
    echo "If lsusb shows the OmniHand adapter but no canX exists, the adapter is not bound to a Linux CAN driver yet."
    exit 1
fi

resolve_interface() {
    local match_count=0
    local candidate

    if [ -z "$SELECTOR" ]; then
        if [ "${#CAN_INTERFACES[@]}" -eq 1 ]; then
            INTERFACE_NAME="${CAN_INTERFACES[0]}"
            return 0
        fi

        echo "Error: multiple CAN interfaces are present; provide an interface name or USB bus-info selector."
        print_interfaces
        return 1
    fi

    for candidate in "${CAN_INTERFACES[@]}"; do
        if [ "$candidate" = "$SELECTOR" ]; then
            INTERFACE_NAME="$candidate"
            return 0
        fi
    done

    local matched_iface=""
    for candidate in "${CAN_INTERFACES[@]}"; do
        local bus_info
        bus_info=$(ethtool -i "$candidate" 2>/dev/null | awk '/bus-info/ {print $2}')
        if [ "$bus_info" = "$SELECTOR" ]; then
            matched_iface="$candidate"
            match_count=$((match_count + 1))
        fi
    done

    if [ "$match_count" -eq 1 ]; then
        INTERFACE_NAME="$matched_iface"
        return 0
    fi

    if [ "$match_count" -gt 1 ]; then
        echo "Error: selector '$SELECTOR' matches multiple CAN interfaces. Choose one interface explicitly."
        print_interfaces
        return 1
    fi

    echo "Error: no CAN interface matches selector '$SELECTOR'."
    print_interfaces
    return 1
}

INTERFACE_NAME=""
if ! resolve_interface; then
    exit 1
fi

echo "Using CAN interface $INTERFACE_NAME"

sudo ip link set "$INTERFACE_NAME" down

if ! sudo ip link set "$INTERFACE_NAME" type can bitrate "$ARBITRATION_BITRATE" sample-point "$SAMPLE_POINT" dbitrate "$DATA_BITRATE" dsample-point "$DSAMPLE_POINT" fd on restart-ms "$RESTART_MS" one-shot "$ONE_SHOT"; then
    echo "Error: interface '$INTERFACE_NAME' rejected CAN FD configuration."
    echo "This usually means the driver or adapter exposes classic CAN only."
    ip -details link show "$INTERFACE_NAME"
    exit 1
fi

if [ "$INTERFACE_NAME" != "$TARGET_NAME" ]; then
    echo "Renaming $INTERFACE_NAME to $TARGET_NAME"
    sudo ip link set "$INTERFACE_NAME" name "$TARGET_NAME"
    INTERFACE_NAME="$TARGET_NAME"
fi

sudo ip link set "$INTERFACE_NAME" txqueuelen "$TX_QUEUE_LEN"
sudo ip link set "$INTERFACE_NAME" up

DETAILS=$(ip -details link show "$INTERFACE_NAME")

if ! printf '%s\n' "$DETAILS" | grep -q 'mtu 72'; then
    echo "Error: interface '$INTERFACE_NAME' came up without CAN FD MTU 72."
    printf '%s\n' "$DETAILS"
    exit 1
fi

if ! printf '%s\n' "$DETAILS" | grep -q "dbitrate $DATA_BITRATE"; then
    echo "Error: interface '$INTERFACE_NAME' does not report the requested data bitrate $DATA_BITRATE."
    printf '%s\n' "$DETAILS"
    exit 1
fi

echo "CAN FD interface '$INTERFACE_NAME' is ready."
printf '%s\n' "$DETAILS"