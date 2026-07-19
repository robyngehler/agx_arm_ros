#!/bin/bash
#
# activate_native_can.sh — bring up the Jetson NATIVE CAN controllers (mttcan,
# 40-pin header) as per-side CAN FD buses for the Duo system. This is the
# standard, validated transport (see docs/sprint5/evidence/can_transport_decision.md
# and docs/errors_and_fixes.md).
#
# Convention (sprint5):
#   can0 -> can_nero_right   (right side: right arm + right OmniHand)
#   can1 -> can_nero_left    (left  side: left  arm + left  OmniHand)
#
# The bus is brought up in CAN FD mode with the validated OmniHand timing
# (1 Mbit arbitration / 5 Mbit data, 0.8 sample points). A CAN FD SocketCAN
# interface transmits BOTH classic frames (to the Nero arm) and FD+BRS frames
# (to the OmniHand), so one side bus can carry the arm and its hand together.
#
# Transceiver: Adafruit CAN Pal (TJA1051T/3), confirmed BRS-capable at 5 Mbit.
# Both the Nero arm (classic CAN 1M) and the OmniHand (CAN FD 5M BRS) run over
# this same transceiver on the 40-pin header.
#
# TDCR (Transmitter Delay Compensation): the TJA1051T/3 requires the TDC offset
# to be set via the mttcan sysfs attribute BEFORE bringing the interface up.
# The validated value is 0x800 for this transceiver. This must be done while the
# interface is DOWN. The devmem/register approach (0xC310048) and the custom
# DTB/extlinux.conf boot-entry approach were both investigated and do not work —
# the sysfs path is the only confirmed method.
#
#   one-shot on : every frame is a single attempt; an unacknowledged frame is
#                 dropped instead of retransmitted forever. This remains the
#                 stable shared-bus baseline because it avoids retransmission
#                 buildup after missing ACK or bus contention.
#   restart-ms  : auto-recover from bus-off.
#   txqueuelen  : socket TX ring depth. The mttcan/kernel default (10) is tiny,
#                 so an arm command burst (7 MIT frames per control cycle) can
#                 overrun it and surface as ENOBUFS ("no buffer space [105]")
#                 before one-shot even helps. A deeper queue absorbs the burst.
#
# Shared arm+hand bus (right side = right arm + right OmniHand on can_nero_right):
#   the arm firmware push (autonomous) + MIT command TX contends with the hand's
#   CANFD request/response. Because the OmniHand CANFD IDs are high (low
#   arbitration priority), with ONE_SHOT=on the hand LOSES arbitration to the arm
#   and its single-attempt frames are dropped -> "请求超时" / the bridge goes silent
#   the moment the MIT controller starts (idle-hold sends little, so the hand is
#   fine there). Historical ONE_SHOT=off experiments can improve hand progress
#   under load, but they also reintroduce retransmission buildup and are not the
#   recommended runtime baseline. Prefer ONE_SHOT=on plus explicit hand-command
#   windows once the arm is in a safe static hold. See
#   docs/control/bringups/teach_and_run.md and docs/sprint6/errors_and_fixes.md.
#
# Usage:
#   sudo bash activate_native_can.sh                 # both side buses (defaults)
#   sudo bash activate_native_can.sh right           # right side only
#   sudo bash activate_native_can.sh left            # left side only
#   FD=0 sudo bash activate_native_can.sh            # classic-only (arm, no hand)
#   ONE_SHOT=off TX_QUEUE_LEN=1000 sudo bash activate_native_can.sh  # historical shared arm+hand experiment only
#   TDCR_VALUE=0x900 sudo bash activate_native_can.sh  # override TDCR for a different transceiver
#
# Idempotent: an interface already renamed to its target is reconfigured in place.

set -u

BITRATE="${BITRATE:-1000000}"
SAMPLE_POINT="${SAMPLE_POINT:-0.8}"
DBITRATE="${DBITRATE:-5000000}"
DSAMPLE_POINT="${DSAMPLE_POINT:-0.8}"
RESTART_MS="${RESTART_MS:-100}"
ONE_SHOT="${ONE_SHOT:-on}"      # stable shared-bus baseline; set off only for controlled transport experiments
TX_QUEUE_LEN="${TX_QUEUE_LEN:-1000}"  # socket TX ring depth; kernel default (10) is too small for arm command bursts
FD="${FD:-1}"                   # 1 = CAN FD (arm+hand), 0 = classic only (arm)
TDCR_VALUE="${TDCR_VALUE:-0x800}" # TJA1051T/3 (Adafruit CAN Pal) validated value
SIDE="${1:-both}"

# side -> "source_iface:target_name"
declare -A ARM_MAP=(
    [right]="can0:can_nero_right"
    [left]="can1:can_nero_left"
)

iface_exists() { ip link show "$1" >/dev/null 2>&1; }

# Set mttcan TDC offset via sysfs while the interface is down.
# Must be called with the current (pre-rename) interface name.
set_tdc_offset() {
    local iface="$1"
    local tdc_path
    tdc_path=$(find /sys/devices/platform/bus@0 -path "*/net/$iface/tdc_offset" 2>/dev/null | head -1)
    if [ -n "$tdc_path" ]; then
        echo "$TDCR_VALUE" | tee "$tdc_path" >/dev/null \
            && echo "  TDCR: $tdc_path = $TDCR_VALUE" \
            || echo "  warning: could not write TDCR to $tdc_path" >&2
    else
        echo "  note: tdc_offset sysfs entry not found for '$iface' — TDCR skipped." >&2
    fi
}

build_type_cmd() {
    local work="$1"
    local -a cmd=(ip link set "$work" type can bitrate "$BITRATE" sample-point "$SAMPLE_POINT")
    if [ "$FD" = "1" ]; then
        cmd+=(dbitrate "$DBITRATE" dsample-point "$DSAMPLE_POINT" fd on)
    fi
    cmd+=(restart-ms "$RESTART_MS" one-shot "$ONE_SHOT")
    printf '%s\n' "${cmd[*]}"
}

activate_one() {
    local src="$1" target="$2" work=""

    if iface_exists "$target"; then
        work="$target"                       # already renamed -> reconfigure in place
    elif iface_exists "$src"; then
        work="$src"
    else
        echo "ERROR: neither '$src' nor '$target' exists. Is native CAN (mttcan) enabled on the 40-pin header?" >&2
        return 1
    fi

    local mode; [ "$FD" = "1" ] && mode="CAN FD 1M/5M" || mode="classic 1M"
    echo "Configuring '$work' -> '$target' ($mode, sample-point=$SAMPLE_POINT, restart-ms=$RESTART_MS, one-shot=$ONE_SHOT)"
    ip link set "$work" down || return 1

    # Set TDC offset before configuring and bringing up the interface.
    # Required for BRS at 5 Mbit with the TJA1051T/3 (Adafruit CAN Pal).
    if [ "$FD" = "1" ]; then
        set_tdc_offset "$work"
    fi

    # berr-reporting is best-effort: not every controller exposes it.
    local type_cmd; type_cmd="$(build_type_cmd "$work")"
    if ! $type_cmd berr-reporting on 2>/dev/null; then
        if ! $type_cmd; then
            echo "ERROR: failed to set CAN type on '$work' (check FD/one-shot support and the transceiver)." >&2
            return 1
        fi
        echo "  note: berr-reporting not supported on '$work', skipped."
    fi

    if [ "$work" != "$target" ]; then
        ip link set "$work" name "$target" || return 1
    fi

    # Deepen the TX ring before bringing the link up so an arm command burst does
    # not immediately surface as ENOBUFS on a shared arm+hand bus.
    ip link set "$target" txqueuelen "$TX_QUEUE_LEN" \
        && echo "  txqueuelen: $target = $TX_QUEUE_LEN" \
        || echo "  warning: could not set txqueuelen on '$target'." >&2

    ip link set "$target" up || return 1
    echo "  OK: $target is up (one-shot=$ONE_SHOT, txqueuelen=$TX_QUEUE_LEN)."
    if [ "$FD" = "1" ]; then
        ip -details link show "$target" | grep -q 'mtu 72' \
            || echo "  warning: '$target' did not report CAN FD MTU 72 — verify the transceiver supports FD/BRS."
    fi
}

if [ "$EUID" -ne 0 ]; then
    echo "Run with sudo (this modifies network interfaces)." >&2
    exit 1
fi

rc=0
case "$SIDE" in
    both)
        for s in right left; do
            IFS=':' read -r src target <<< "${ARM_MAP[$s]}"
            activate_one "$src" "$target" || rc=1
        done
        ;;
    right|left)
        IFS=':' read -r src target <<< "${ARM_MAP[$SIDE]}"
        activate_one "$src" "$target" || rc=1
        ;;
    *)
        echo "Usage: $0 [both|right|left]" >&2
        exit 2
        ;;
esac

exit "$rc"
