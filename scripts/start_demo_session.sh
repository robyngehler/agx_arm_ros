#!/usr/bin/env bash
#
# start_demo_session.sh — one command between an SSH login and a stack that
# takes activities.
#
# THE ORDER IS THE POINT. Platform knobs, then CAN buses, then the ROS stack.
# Bus activation takes the interfaces down and up, so it must not run under a
# live stack; this script skips that step when a supervisor is already up rather
# than reordering the demo around it.
#
# THIS SCRIPT DOES NOT OWN THE STACK. The supervisor goes into its own tmux
# session and stays there, so a dropped SSH connection takes this script and
# leaves the stack supervised. That is also why tmux is a hard requirement here
# and only a warning in start_demo_stack.py.
#
# WHY bash -ic. ~/.bashrc sources ROS only for interactive shells, so a tmux
# pane command run through the default shell has no ros2 on its PATH. The
# supervisor is therefore started under an explicit interactive bash.
#
# Usage:
#   ./scripts/start_demo_session.sh                # platform, buses, stack
#   ./scripts/start_demo_session.sh --stack tea    # the tea stack instead
#   ./scripts/start_demo_session.sh --grippers     # bottom unit: duo_gripper
#   ./scripts/start_demo_session.sh --clock-boost  # also pin clocks and idle states
#   ./scripts/start_demo_session.sh --no-platform  # leave the platform knobs alone
#   ./scripts/start_demo_session.sh --no-can       # leave the buses alone
#   ./scripts/start_demo_session.sh --status       # report, change nothing

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=agx-stack
STATE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/agx_demo_stack"

DO_PLATFORM=1
DO_CAN=1
CLOCK_BOOST=0
MODE=start
STACK=demo
GRIPPERS=0
READY_TIMEOUT=300

while [ $# -gt 0 ]; do
    case "$1" in
        --status)       MODE=status ;;
        --no-platform)  DO_PLATFORM=0 ;;
        --no-can)       DO_CAN=0 ;;
        --clock-boost)  CLOCK_BOOST=1 ;;
        --grippers)     GRIPPERS=1 ;;
        --stack)        STACK="${2:?--stack needs demo or tea}"; shift ;;
        --ready-timeout) READY_TIMEOUT="${2:?--ready-timeout needs seconds}"; shift ;;
        -h|--help)      sed -n '2,28p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "usage: $0 [--status] [--stack demo|tea] [--grippers]" \
                "[--clock-boost] [--no-platform] [--no-can]" >&2; exit 2 ;;
    esac
    shift
done

case "$STACK" in demo|tea) ;; *) echo "--stack takes demo or tea" >&2; exit 2 ;; esac

step() { printf '\n== %s\n' "$*"; }
note() { printf '   %s\n' "$*"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --- preconditions ----------------------------------------------------------

# Not under sudo: the steps that need root take it themselves, and root has no
# AGX_UNIT and a different home, so the stack would be started as root and its
# state file written where the operator's stop command will not look.
if [ "$(id -u)" -eq 0 ]; then
    fail "run this as the operator, not with sudo — the platform and CAN steps
  take root themselves. As root there is no AGX_UNIT, and the stack's state
  file would land outside your home where stop_demo_stack.py cannot find it.
      ./scripts/start_demo_session.sh"
fi

UNIT="${AGX_UNIT:-}"
if [ -z "$UNIT" ]; then
    fail "this machine does not say which unit it is: AGX_UNIT is unset.
  set it once with:  ./scripts/isolate_ros_graph.sh --unit top|bottom|stacking
  then open a new session"
fi
case "$UNIT" in top|bottom|stacking) ;; *) fail "unknown AGX_UNIT '$UNIT'" ;; esac

STATE_FILE="$STATE_DIR/$UNIT.json"

if ! command -v tmux >/dev/null 2>&1; then
    fail "tmux is not installed, and the stack supervisor has to outlive this
  session — a dropped SSH connection would otherwise leave the launches
  running with nothing to shut them down.
      sudo apt install tmux"
fi

supervisor_pid() {
    [ -f "$STATE_FILE" ] || return 1
    python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["pid"])' \
        "$STATE_FILE" 2>/dev/null || return 1
}

stack_running() {
    local pid
    pid="$(supervisor_pid)" || return 1
    kill -0 "$pid" 2>/dev/null
}

# --- status -----------------------------------------------------------------

if [ "$MODE" = status ]; then
    printf 'unit          %s   domain %s   localhost_only %s\n' \
        "$UNIT" "${ROS_DOMAIN_ID:-UNSET}" "${ROS_LOCALHOST_ONLY:-UNSET}"
    if stack_running; then
        printf 'stack         RUNNING (supervisor pid %s)\n' "$(supervisor_pid)"
        printf '              %s\n' "$STATE_FILE"
    else
        printf 'stack         not running\n'
    fi
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        printf 'tmux          session %s exists\n' "$SESSION"
    else
        printf 'tmux          no %s session\n' "$SESSION"
    fi
    printf '\nplatform\n'
    "$REPO_ROOT/scripts/jetson_presentation_mode.sh" status || true
    "$REPO_ROOT/scripts/jetson_clock_boost.sh" status || true
    printf '\nCAN buses\n'
    "$REPO_ROOT/scripts/activate_stack.sh" --show || true
    exit 0
fi

# --- 1. platform ------------------------------------------------------------

if [ "$DO_PLATFORM" = 1 ]; then
    step "platform: taking off the power saving that costs latency"
    "$REPO_ROOT/scripts/jetson_presentation_mode.sh" on
    if [ "$CLOCK_BOOST" = 1 ]; then
        step "platform: pinning clocks and idle states"
        note "the unit now draws its full budget whether or not it is working"
        "$REPO_ROOT/scripts/jetson_clock_boost.sh" on
    fi
else
    step "platform: skipped (--no-platform)"
fi

# --- 2. CAN buses -----------------------------------------------------------

if stack_running; then
    step "CAN buses: skipped — a $UNIT stack is already up (pid $(supervisor_pid))"
    note "activation takes the interfaces down and up; that is not safe under a"
    note "live stack. Stop it first if the buses need attention:"
    note "    ./scripts/stop_demo_stack.py"
elif [ "$DO_CAN" = 1 ]; then
    step "CAN buses: activating and verifying"
    if ! sudo "$REPO_ROOT/scripts/activate_stack.sh"; then
        fail "the CAN buses did not come up healthy.
  Check power and cabling, then:
      ./scripts/activate_stack.sh --show
      sudo ./scripts/activate_stack.sh --recover"
    fi
else
    step "CAN buses: skipped (--no-can)"
fi

# --- 3. the ROS stack, in its own tmux session ------------------------------

if stack_running; then
    step "stack: already running (supervisor pid $(supervisor_pid))"
else
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        # remain-on-exit keeps the pane after a stop, so a normal shutdown leaves
        # a dead session behind. Clear that one — its output is also in the log
        # dir. A pane still running something is not ours to close.
        if [ "$(tmux list-panes -t "$SESSION" -F '#{pane_dead}' 2>/dev/null | head -1)" = 1 ]; then
            step "stack: clearing the finished '$SESSION' session from the last run"
            tmux kill-session -t "$SESSION"
        else
            fail "a tmux session '$SESSION' exists but holds no running supervisor.
  Look at it, then close it:
      tmux attach -t $SESSION
      tmux kill-session -t $SESSION"
        fi
    fi

    SUPERVISOR="./scripts/start_demo_stack.py"
    [ "$STACK" = tea ] && SUPERVISOR="$SUPERVISOR --stack tea"
    [ "$GRIPPERS" = 1 ] && SUPERVISOR="$SUPERVISOR --grippers"

    step "stack: starting the supervisor in tmux session '$SESSION'"
    note "$SUPERVISOR"
    # remain-on-exit keeps a failed bring-up's output readable in its pane, and
    # gives this script a precise "the supervisor exited" signal to wait on.
    # tmux writes its own "Pane is dead" line there; remain-on-exit-format, which
    # would say more, needs tmux 3.4 and this unit has 3.2a.
    tmux new-session -d -s "$SESSION" -c "$REPO_ROOT" "bash -ic '$SUPERVISOR'" \
        \; set-window-option -t "$SESSION" remain-on-exit on

    note "waiting for it to report READY (up to ${READY_TIMEOUT}s)"
    deadline=$(( SECONDS + READY_TIMEOUT ))
    while :; do
        if stack_running; then
            break
        fi
        if ! tmux has-session -t "$SESSION" 2>/dev/null \
           || [ "$(tmux list-panes -t "$SESSION" -F '#{pane_dead}' 2>/dev/null | head -1)" = 1 ]; then
            echo >&2
            echo "the supervisor exited before the stack was ready:" >&2
            tmux capture-pane -p -t "$SESSION" 2>/dev/null | grep -v '^\s*$' | tail -25 >&2
            echo >&2
            fail "bring-up failed. The pane is kept: tmux attach -t $SESSION"
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            fail "the stack was not ready after ${READY_TIMEOUT}s.
  Watch it: tmux attach -t $SESSION"
        fi
        sleep 1
    done
fi

# --- what to do next --------------------------------------------------------

cat <<EOF

$(printf '%s' "${UNIT^^}") ${STACK^^} STACK READY

  watch it      tmux attach -t $SESSION      (detach: Ctrl+B, D)
  state         $STATE_FILE
  status        ./scripts/start_demo_session.sh --status

activities on this unit:
EOF

case "$UNIT:$STACK" in
    top:demo)      cat <<'EOF'
  ./scripts/unpack_top_unit.py
  ./scripts/wave.py
  ./scripts/pack_top_unit.py
EOF
    ;;
    bottom:demo)   cat <<'EOF'
  ./scripts/unpack_bottom_unit.py --slow
  ./scripts/pack_bottom_unit.py --slow
EOF
    ;;
    stacking:demo) cat <<'EOF'
  ./scripts/start_block_restack.py
EOF
    ;;
    *:tea)         cat <<'EOF'
  ./scripts/start_tea_demo.py
EOF
    ;;
esac

cat <<EOF

  stop the stack   ./scripts/stop_demo_stack.py
  release the unit sudo ./scripts/jetson_presentation_mode.sh off$([ "$CLOCK_BOOST" = 1 ] && printf '\n                   sudo ./scripts/jetson_clock_boost.sh off')
EOF
