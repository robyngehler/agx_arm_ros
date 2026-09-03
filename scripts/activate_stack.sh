#!/bin/bash
#
# activate_stack.sh — bring the Duo CAN buses up and prove they carry clean traffic.
#
# activate_duo_can.sh does the slot-correct bring-up and is called from here
# unchanged, so the interface-to-slot mapping stays declared in exactly one file.
# This script adds the two things an operator needs around it.
#
# 1. VERIFICATION BEYOND LINK STATE. activate_duo_can.sh's own header says it:
#    "an interface that is UP but silent is not the same as a working bus". Nor
#    is one that is up, carrying frames, and accumulating errors. Both are
#    checked here over a sampling window.
#
# 2. A BOUNDED RECOVERY CHAIN. On this Jetson the arm buses sometimes come up
#    unusable after a boot and need the mttcan driver reloaded — sometimes more
#    than once. That is the manual
#      rmmod mttcan; modprobe mttcan; sudo bash scripts/activate_duo_can.sh
#    cycle, done here with a verification between attempts instead of by eye.
#
# WHAT "HEALTHY" MEANS HERE, and why the two device types differ:
#
#   arms   an arm pushes joint feedback unprompted, with no host node running —
#          measured 2026-09-03 at ~3178 frames/s per arm on an idle stack. So RX
#          must be advancing, the controller must be ERROR-ACTIVE, and the error
#          counters must not move over the window.
#   hands  a hand is polled, not pushed, so it is silent until a bridge talks to
#          it. Requiring RX here would fail a healthy hand. Presence, UP,
#          ERROR-ACTIVE and flat error counters only.
#
# CALIBRATION CAVEAT. The reported first-start symptom is "messages rising but
# rviz/MoveIt never comes up", which a plain RX check passes. The discriminator
# used here is the error counters, and it has NOT been measured in that failing
# state. That is why --recover exists as its own mode: it runs the reload chain
# unconditionally, which is what the operator does by hand today. When the
# failing state is captured (--show during it), tighten the automatic trigger.
#
# The reload needs the buses free. Nothing may hold a CAN socket, so this script
# refuses to reload while an arm driver or hand bridge is running rather than
# fighting the kernel for the device.
#
# Usage:
#   sudo ./scripts/activate_stack.sh                 # activate + verify all four
#   sudo ./scripts/activate_stack.sh arms            # arms only
#   sudo ./scripts/activate_stack.sh --recover       # go straight to the reload chain
#   ./scripts/activate_stack.sh --show               # report, change nothing (no sudo)
#   ./scripts/activate_stack.sh --verify-only        # sample and judge, change nothing
#
# Options:
#   --attempts N   recovery cycles before failing hard (default 3)
#   --window S     verification sampling window in seconds (default 2)

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTIVATE="$REPO_ROOT/scripts/activate_duo_can.sh"

ARMS=(can_nero_right can_nero_left)
HANDS=(hand_right hand_left)
ARM_MODULE=mttcan
HAND_MODULE=peak_usb

#: Processes that open a CAN socket on these buses. The reload refuses while one
#: is alive; killing it is the operator's call, not this script's.
BUS_HOLDERS=(agx_arm_ctrl_single_node omnihand_bridge_node)

ATTEMPTS=3
WINDOW=2
GROUP=all
MODE=activate

while [ $# -gt 0 ]; do
    case "$1" in
        --show|show)      MODE=show ;;
        --verify-only)    MODE=verify ;;
        --recover)        MODE=recover ;;
        --attempts)       ATTEMPTS="${2:?--attempts needs a number}"; shift ;;
        --window)         WINDOW="${2:?--window needs seconds}"; shift ;;
        arms)             GROUP=arms ;;
        hands)            GROUP=hands ;;
        --all|all)        GROUP=all ;;
        -h|--help)        sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "usage: $0 [arms|hands|--all] [--show|--verify-only|--recover]" \
                "[--attempts N] [--window S]" >&2; exit 2 ;;
    esac
    shift
done

targets() {
    case "$GROUP" in
        arms)  printf '%s\n' "${ARMS[@]}" ;;
        hands) printf '%s\n' "${HANDS[@]}" ;;
        *)     printf '%s\n' "${ARMS[@]}" "${HANDS[@]}" ;;
    esac
}

is_arm() {
    local candidate="$1" arm
    for arm in "${ARMS[@]}"; do
        [ "$arm" = "$candidate" ] && return 0
    done
    return 1
}

rx_packets() {
    cat "/sys/class/net/$1/statistics/rx_packets" 2>/dev/null || echo ""
}

# "state restarts bus-errors arbit-lost error-warn error-pass bus-off tec rec"
# for one interface, or nothing when it does not exist. The berr counters are
# the transmit and receive error counters the controller itself keeps; `ip -d`
# prints them beside the state.
bus_stats() {
    ip -s -d link show "$1" 2>/dev/null | awk '
        /state (ERROR|BUS-OFF|STOPPED|SLEEPING)/ {
            for (i = 1; i <= NF; i++) {
                if ($i == "state") { state = $(i + 1) }
                if ($i == "tx" && $(i - 1) == "berr-counter") { tec = $(i + 1) }
                if ($i == "rx" && $(i - 2) == "berr-counter") { rec = $(i + 1) }
            }
        }
        /re-started .*bus-off/ { getline; r = $1; be = $2; al = $3; ew = $4; ep = $5; bo = $6 }
        END {
            if (state == "") exit 1
            print state, r + 0, be + 0, al + 0, ew + 0, ep + 0, bo + 0, tec + 0, rec + 0
        }
    '
}

present() {
    [ -e "/sys/class/net/$1" ]
}

link_up() {
    [ "$(cat "/sys/class/net/$1/operstate" 2>/dev/null)" = "up" ]
}

# Sample every target once, wait, sample again, and print one line each. Sets
# FAILED to the space-separated list of targets that did not pass.
FAILED=""
sample_and_judge() {
    local judge="$1" target
    local -A rx0 stats0
    for target in $(targets); do
        rx0["$target"]="$(rx_packets "$target")"
        stats0["$target"]="$(bus_stats "$target")"
    done
    sleep "$WINDOW"

    FAILED=""
    printf '%-16s %-13s %-8s %-11s %-9s %s\n' \
        TARGET STATE RX/s ERR-DELTA BERR VERDICT
    for target in $(targets); do
        local verdict=ok reason=""
        if ! present "$target"; then
            printf '%-16s %-13s %-8s %-11s %-9s %s\n' \
                "$target" MISSING - - - "not present"
            FAILED="$FAILED $target"
            continue
        fi

        local before="${stats0[$target]}" after
        after="$(bus_stats "$target")"
        read -r state0 r0 be0 al0 ew0 ep0 bo0 _ _ <<<"$before"
        read -r state1 r1 be1 al1 ew1 ep1 bo1 tec rec <<<"$after"
        local rx1 rate="?" err_delta
        rx1="$(rx_packets "$target")"
        if [ -n "${rx0[$target]}" ] && [ -n "$rx1" ]; then
            rate="$(awk -v a="${rx0[$target]}" -v b="$rx1" -v w="$WINDOW" \
                'BEGIN { printf "%.0f", (b - a) / w }')"
        fi
        err_delta="$(( (r1 - r0) + (be1 - be0) + (al1 - al0) \
                     + (ew1 - ew0) + (ep1 - ep0) + (bo1 - bo0) ))"

        if ! link_up "$target"; then
            verdict=fail; reason="link is not up"
        elif [ "$state1" != "ERROR-ACTIVE" ]; then
            verdict=fail; reason="controller is $state1"
        elif [ "$err_delta" -gt 0 ]; then
            verdict=fail; reason="error counters advanced by $err_delta"
        elif is_arm "$target" && [ "$rate" != "?" ] && [ "$rate" -eq 0 ]; then
            # Only the arms push unprompted; see the header.
            verdict=fail; reason="no feedback frames (arm should push unprompted)"
        fi

        printf '%-16s %-13s %-8s %-11s %-9s %s\n' \
            "$target" "$state1" "$rate" "$err_delta" "$tec/$rec" \
            "${reason:-ok}"
        [ "$verdict" = fail ] && FAILED="$FAILED $target"
    done
    FAILED="${FAILED# }"
    [ "$judge" = judge ] || return 0
    [ -z "$FAILED" ]
}

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Run with sudo (this modifies network interfaces and kernel modules)." >&2
        exit 1
    fi
}

# Refuse a reload while something owns a bus. rmmod would fail anyway, but it
# fails as EBUSY with no hint about who is holding it.
refuse_if_bus_is_held() {
    local holder pids all=""
    for holder in "${BUS_HOLDERS[@]}"; do
        pids="$(pgrep -f "$holder" 2>/dev/null | tr '\n' ' ')"
        [ -n "$pids" ] && all="$all $holder(${pids% })"
    done
    if [ -n "$all" ]; then
        echo "refusing to reload a CAN driver: the buses are in use by$all" >&2
        echo "Shut the ROS stack down first; the reload takes the interfaces away." >&2
        return 1
    fi
    return 0
}

can_interface_count() {
    ip -br link show type can 2>/dev/null | wc -l
}

# The netdevs do not exist the instant modprobe returns; activate_duo_can.sh
# would then report the slot as NOT FOUND and the whole attempt would be spent
# on a race. Waits for the interface count to come back, bounded.
wait_for_interfaces() {
    local want="$1" waited=0
    while [ "$(can_interface_count)" -lt "$want" ]; do
        if [ "$waited" -ge 50 ]; then
            echo "  only $(can_interface_count) of $want CAN interfaces came back" >&2
            return 1
        fi
        sleep 0.1
        waited=$((waited + 1))
    done
    return 0
}

reload_module() {
    local module="$1" before
    before="$(can_interface_count)"
    if ! grep -q "^$module " /proc/modules; then
        echo "  $module is not loaded; loading it"
        before=$((before + 1))
    else
        echo "  removing $module"
        if ! rmmod "$module"; then
            echo "  could not remove $module — something still holds it" >&2
            return 1
        fi
    fi
    if ! modprobe "$module"; then
        echo "  could not load $module" >&2
        return 1
    fi
    wait_for_interfaces "$before" || return 1
    echo "  $module reloaded, $(can_interface_count) CAN interfaces present"
    return 0
}

activate_group() {
    local group="$1"
    bash "$ACTIVATE" "$group" >/dev/null 2>&1
    return $?
}

# One recovery cycle for whichever device types are failing.
recover_once() {
    local failing="$1" want_arms=0 want_hands=0 target rc=0
    for target in $failing; do
        if is_arm "$target"; then want_arms=1; else want_hands=1; fi
    done
    refuse_if_bus_is_held || return 1

    if [ "$want_arms" = 1 ]; then
        echo "recovering the arm buses"
        reload_module "$ARM_MODULE" || rc=1
        activate_group arms || rc=1
    fi
    if [ "$want_hands" = 1 ]; then
        echo "recovering the hand buses"
        reload_module "$HAND_MODULE" || rc=1
        activate_group hands || rc=1
    fi
    return "$rc"
}

case "$MODE" in
    show)
        bash "$ACTIVATE" --show
        echo
        sample_and_judge report
        exit 0
        ;;
    verify)
        if sample_and_judge judge; then
            echo
            echo "all verified buses are healthy"
            exit 0
        fi
        echo
        echo "not healthy: $FAILED" >&2
        exit 1
        ;;
esac

require_root

if [ "$MODE" = activate ]; then
    echo "activating the $GROUP CAN interfaces"
    case "$GROUP" in
        arms)  activate_group arms ;;
        hands) activate_group hands ;;
        *)     activate_group --all ;;
    esac
    echo
    if sample_and_judge judge; then
        echo
        echo "stack activated and verified"
        exit 0
    fi
    echo
    echo "verification failed for: $FAILED — entering recovery"
fi

# --recover starts here directly: the reload chain is what the operator runs by
# hand today, and it is available without waiting for the automatic trigger to
# be calibrated against the failing state.
if [ "$MODE" = recover ] && [ -z "${FAILED:-}" ]; then
    FAILED="$(targets | tr '\n' ' ')"
    FAILED="${FAILED% }"
    echo "forced recovery of: $FAILED"
fi

attempt=1
while [ "$attempt" -le "$ATTEMPTS" ]; do
    echo
    echo "recovery attempt $attempt of $ATTEMPTS"
    recover_once "$FAILED" || {
        echo "recovery could not run; not retrying" >&2
        exit 1
    }
    echo
    if sample_and_judge judge; then
        echo
        echo "recovered after $attempt attempt(s)"
        exit 0
    fi
    echo
    echo "still not healthy: $FAILED"
    attempt=$((attempt + 1))
done

echo >&2
echo "FAILED after $ATTEMPTS recovery attempts: $FAILED" >&2
echo "Check power and cabling, then ./scripts/activate_stack.sh --show." >&2
exit 1
