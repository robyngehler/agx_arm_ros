#!/bin/bash
#
# isolate_ros_graph.sh — keep this unit's ROS graph off the network.
#
# Two units on one router share a DDS graph, and this stack names its topics by
# side, not by unit: both publish /left_arm/feedback/joint_states, both offer
# /right_arm/emergency_stop and /execute_activity, and /tf is global with
# identical frame names on both. One trajectory command then reaches two arm
# drivers. Nothing here needs the network — each unit brings up its own launches
# and runs run_activity against them locally (scripts/demo_stack.py) — so the
# graph is confined to loopback and given a domain of its own.
#
#   ROS_LOCALHOST_ONLY=1   DDS discovery and traffic stay on lo. This is the
#                          actual isolation: it holds even if both units end up
#                          on the same domain.
#   ROS_DOMAIN_ID=<n>      Different UDP ports per unit. Redundant on purpose —
#                          it still separates the two if someone unsets
#                          ROS_LOCALHOST_ONLY for a debugging session.
#
# WHAT IT COSTS. No RViz, `ros2 topic echo` or rqt from a laptop against this
# unit. The demo stacks run with use_rviz:=false anyway.
#
# WHERE IT APPLIES. The exports go in ~/.bashrc, which bash reads for
# interactive shells only — so an interactive SSH session has them and
# `ssh <host> '<command>'` does not (docs/sprint_scripts/headless_operation.md
# §2). The demos are operated from an interactive tmux session.
#
# Usage:
#   ./scripts/isolate_ros_graph.sh --domain 41    # top unit
#   ./scripts/isolate_ros_graph.sh --domain 42    # bottom unit
#   ./scripts/isolate_ros_graph.sh --show         # report, change nothing
#   ./scripts/isolate_ros_graph.sh --revert       # remove the managed block

set -uo pipefail

BASHRC="${HOME}/.bashrc"
BEGIN_MARK="# >>> agx_arm_ros ros graph isolation >>>"
END_MARK="# <<< agx_arm_ros ros graph isolation <<<"

# The L2 activity harness runs on 77 and refuses to start when that domain is
# not empty (src/agx_arm_coordination/test/test_l2_activity_integration.py).
RESERVED_DOMAIN=77
# Above 101 the DDS port range starts overlapping the ephemeral ports.
MAX_DOMAIN=101

MODE=""
DOMAIN=""

usage() {
    echo "usage: $0 --domain <0-${MAX_DOMAIN}> | --show | --revert" >&2
    exit 2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --domain) DOMAIN="${2:-}"; MODE=apply; shift 2 || usage ;;
        --domain=*) DOMAIN="${1#*=}"; MODE=apply; shift ;;
        --show|show) MODE=show; shift ;;
        --revert)    MODE=revert; shift ;;
        -h|--help)   sed -n '2,34p' "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) usage ;;
    esac
done
[ -n "$MODE" ] || usage

# Run as the operator, not through sudo: the file being edited is the login
# shell's ~/.bashrc, and under sudo that is root's.
if [ "$(id -u)" -eq 0 ]; then
    echo "Do not run this with sudo — it would write ${BASHRC} for root." >&2
    exit 1
fi

managed_block() {
    grep -A3 -F "$BEGIN_MARK" "$BASHRC" 2>/dev/null | grep -E '^export ROS_' || true
}

report() {
    local nodes
    echo "shell now       : ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}" \
         "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset, means 0>}"
    if grep -qF "$BEGIN_MARK" "$BASHRC" 2>/dev/null; then
        echo "next session    : $(managed_block | paste -sd' ' -) (from ${BASHRC})"
    else
        echo "next session    : no managed block in ${BASHRC}"
    fi
    echo "rmw             : ${RMW_IMPLEMENTATION:-<unset, Humble default rmw_fastrtps_cpp>}"
    if command -v ros2 >/dev/null; then
        nodes="$(timeout 10 ros2 node list 2>/dev/null | grep -c .)"
        echo "nodes visible   : ${nodes:-0} (in the domain this shell is in)"
    else
        echo "nodes visible   : ros2 not on PATH in this shell"
    fi
}

if [ "$MODE" = show ]; then
    report
    exit 0
fi

# --- both apply and revert rewrite the file, so back it up once per run ------

if [ ! -f "$BASHRC" ]; then
    echo "${BASHRC} does not exist." >&2
    exit 1
fi

BACKUP="${BASHRC}.$(date +%Y%m%d-%H%M%S).bak"
cp "$BASHRC" "$BACKUP" || exit 1

# Drop any previous block so a second run replaces it instead of stacking.
sed -i "\|^${BEGIN_MARK}\$|,\|^${END_MARK}\$|d" "$BASHRC"

if [ "$MODE" = revert ]; then
    if grep -qF "$BEGIN_MARK" "$BASHRC"; then
        echo "failed to remove the managed block; ${BASHRC} restored from ${BACKUP}" >&2
        cp "$BACKUP" "$BASHRC"
        exit 1
    fi
    echo "removed the managed block from ${BASHRC} (backup: ${BACKUP})"
    echo "the graph goes back on the network in the next new shell."
    exit 0
fi

# --- apply ------------------------------------------------------------------

case "$DOMAIN" in
    ''|*[!0-9]*) echo "--domain needs a number, got '${DOMAIN}'" >&2; cp "$BACKUP" "$BASHRC"; exit 2 ;;
esac
if [ "$DOMAIN" -gt "$MAX_DOMAIN" ]; then
    echo "--domain ${DOMAIN} is above ${MAX_DOMAIN}; the DDS ports would reach into the ephemeral range." >&2
    cp "$BACKUP" "$BASHRC"; exit 2
fi
if [ "$DOMAIN" -eq "$RESERVED_DOMAIN" ]; then
    echo "--domain ${RESERVED_DOMAIN} is the L2 test harness domain; it refuses to start when the domain is not empty." >&2
    cp "$BACKUP" "$BASHRC"; exit 2
fi
if [ "$DOMAIN" -eq 0 ]; then
    echo "WARNING: 0 is the default domain, so any unconfigured ROS machine in the room joins it." >&2
    echo "         ROS_LOCALHOST_ONLY=1 still keeps it off the network. Prefer a non-zero domain." >&2
fi

BLOCK="${BEGIN_MARK}
# Managed by scripts/isolate_ros_graph.sh — change it through that script.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=${DOMAIN}
${END_MARK}"

# Before the first workspace sourcing, so both units' ~/.bashrc read the same
# way when they are compared side by side. Appended if there is no such line.
# No blank line around it, so removing the marker range is an exact inverse and
# an apply/revert cycle leaves the file byte-identical.
if grep -qE '^\s*source .*/setup\.bash' "$BASHRC"; then
    awk -v block="$BLOCK" '
        !inserted && /^[[:space:]]*source .*\/setup\.bash/ { print block; inserted = 1 }
        { print }
    ' "$BASHRC" > "${BASHRC}.tmp" && mv "${BASHRC}.tmp" "$BASHRC"
else
    printf '%s\n' "$BLOCK" >> "$BASHRC"
fi

if ! grep -qF "export ROS_DOMAIN_ID=${DOMAIN}" "$BASHRC"; then
    echo "the block did not land in ${BASHRC}; restoring ${BACKUP}" >&2
    cp "$BACKUP" "$BASHRC"
    exit 1
fi

echo "wrote to ${BASHRC} (backup: ${BACKUP}):"
managed_block | sed 's/^/  /'
echo

# The daemon caches the graph of the domain it was started in, and it is not
# restarted by a new login shell. Stop it in the domain it is running in — that
# is the current shell's, which is still the old one.
if command -v ros2 >/dev/null; then
    timeout 15 ros2 daemon stop >/dev/null 2>&1 \
        && echo "stopped the ros2 daemon (it restarts in the new domain on the next command)"
fi

# Excluding this script and its caller: -f matches any command line carrying the
# words, including the shell that invoked this one.
running="$(pgrep -f 'ros2 launch|run_activity' 2>/dev/null \
           | grep -vxE "$$|$PPID" | wc -l)"
if [ "$running" -gt 0 ]; then
    echo
    echo "WARNING: ${running} ROS launch/activity process(es) are still running. They keep the" >&2
    echo "         old domain until they are stopped, and one of them owns an arm driver." >&2
fi

cat <<NEXT

A script cannot export into the shell that called it. For the shell you are in:

    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=${DOMAIN}

Every new interactive session picks it up on its own. Verify with:

    ./scripts/isolate_ros_graph.sh --show

and, with the other unit's stack up, that 'nodes visible' counts only this
unit's nodes.
NEXT
