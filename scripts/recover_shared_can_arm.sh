#!/bin/bash
#
# recover_shared_can_arm.sh — disconnect-safe recovery for the shared arm+hand
# side CAN bus (shared-CAN step-and-settle plan section 2, validation section
# 6.2 item 3).
#
# The dangerous case this exists for: the bus stalls or disconnects while the
# arm is under active MIT control. The Nero firmware has no MIT command watchdog
# and keeps executing the last command it received, so silencing the stream is
# NOT a safe state. This helper runs the clean-stop-then-reset sequence the
# driver's own reconnect watchdog cannot do on its own — it also stops the hand
# and cancels arm goals BEFORE flushing the link, and requires an explicit
# verified success before any arm motion is re-enabled.
#
# Order (each step is best-effort and timeout-guarded; the sequence continues so
# a single unreachable node never blocks the physical stop):
#   1. cancel the active MIT trajectory and request an MIT hold
#   2. stop the OmniHand so pending hand retries stop hammering the side bus
#      before the link reset (and are not killed mid-command by it)
#   3. driver emergency stop (damped MIT zero -> verify in feedback -> escalate)
#   4. CAN link down/up to flush the qdisc / stuck frames
#   5. wait for the driver watchdog to reconnect and feedback to resume
#   6. force normal mode and VERIFY it via the service's readback-checked result
#   7. re-check the hand backend (survival across a link reset is UNTESTED, 6.2.5)
#   8. gate: succeed only when feedback is live AND normal mode is verified
#
# Usage:
#   sudo -v && ./scripts/recover_shared_can_arm.sh [right|left]
# Env overrides:
#   ARM_NS=right_arm     service namespace (duo runtime); empty = unprefixed teach setup
#   IFACE=can_nero_right override the CAN interface name
#   SVC_TIMEOUT=5        per-service-call timeout (s)
#   FEEDBACK_TIMEOUT=10  how long to wait for feedback to resume (s)
#   DRY_RUN=1            print the intended sequence without touching ROS or the link

set -uo pipefail

SIDE="${1:-right}"
ARM_NS="${ARM_NS:-}"
SVC_TIMEOUT="${SVC_TIMEOUT:-5}"
FEEDBACK_TIMEOUT="${FEEDBACK_TIMEOUT:-10}"
DRY_RUN="${DRY_RUN:-0}"

case "$SIDE" in
  right) IFACE="${IFACE:-can_nero_right}" ;;
  left)  IFACE="${IFACE:-can_nero_left}" ;;
  *) echo "usage: $0 [right|left]  (env: ARM_NS, IFACE, SVC_TIMEOUT, FEEDBACK_TIMEOUT, DRY_RUN)" >&2; exit 2 ;;
esac

log() { echo "[recover $(date +%H:%M:%S)] $*" >&2; }

ns() {
  if [ -n "$ARM_NS" ]; then echo "/$ARM_NS/$1"; else echo "/$1"; fi
}

# Best-effort service calls. In DRY_RUN they only print and report reachable.
call_empty() {
  if [ "$DRY_RUN" = 1 ]; then log "  DRYRUN service(Empty) $1"; return 0; fi
  timeout "$SVC_TIMEOUT" ros2 service call "$1" std_srvs/srv/Empty >/dev/null 2>&1
}
call_trigger() {  # echoes the service response so callers can grep success
  if [ "$DRY_RUN" = 1 ]; then log "  DRYRUN service(Trigger) $1"; echo "success=True"; return 0; fi
  timeout "$SVC_TIMEOUT" ros2 service call "$1" std_srvs/srv/Trigger 2>/dev/null
}
link_reset() {
  if [ "$DRY_RUN" = 1 ]; then log "  DRYRUN ip link set $IFACE down/up"; return 0; fi
  sudo ip link set "$IFACE" down && sudo ip link set "$IFACE" up
}

log "shared-CAN recovery: side=$SIDE iface=$IFACE ns='${ARM_NS:-<root>}' dry_run=$DRY_RUN"

# 1. cancel the active MIT trajectory + request a hold
log "1/8 cancel MIT trajectory + hold"
call_empty "$(ns mit_controller/cancel_trajectory)" || log "  cancel_trajectory unreachable (continuing)"
call_empty "$(ns mit_controller/hold_current)" || log "  hold_current unreachable (continuing)"

# 2. stop the hand BEFORE the link reset
log "2/8 stop the OmniHand"
call_trigger "$(ns control/omnihand/stop)" >/dev/null || log "  omnihand/stop unreachable (continuing)"

# 3. verified driver emergency stop
log "3/8 driver emergency stop (damped MIT zero, verified, escalates)"
call_empty "$(ns emergency_stop)" || log "  emergency_stop unreachable (continuing)"

# 4. flush the link
log "4/8 CAN link reset: $IFACE down/up (needs sudo)"
if link_reset; then log "  link reset ok"; else log "  LINK RESET FAILED — check privileges / interface name"; fi

# 5. wait for the driver watchdog to reconnect and feedback to resume
log "5/8 wait up to ${FEEDBACK_TIMEOUT}s for feedback to resume"
fb_ok=0
if [ "$DRY_RUN" = 1 ]; then
  fb_ok=1; log "  DRYRUN assume feedback resumed"
else
  deadline=$(( $(date +%s) + FEEDBACK_TIMEOUT ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if timeout 2 ros2 topic echo --once "$(ns feedback/joint_states)" sensor_msgs/msg/JointState >/dev/null 2>&1; then
      fb_ok=1; break
    fi
  done
fi
[ "$fb_ok" = 1 ] && log "  feedback resumed" || log "  feedback did NOT resume"

# 6. force + verify normal mode
log "6/8 force + verify normal mode"
nm_out="$(call_trigger "$(ns set_normal_mode)")"
if echo "$nm_out" | grep -qi "success=True"; then
  nm_ok=1; log "  normal mode verified"
else
  nm_ok=0; log "  normal mode NOT verified: ${nm_out:-<no response>}"
fi

# 7. re-check the hand backend after the link reset (UNTESTED whether it survives)
log "7/8 re-check hand backend (survival across down/up is UNTESTED, plan 6.2.5)"
call_trigger "$(ns control/omnihand/stop)" >/dev/null || log "  hand still unreachable — bridge may need restart"

# 8. explicit success gate
if [ "$fb_ok" = 1 ] && [ "$nm_ok" = 1 ]; then
  log "8/8 RECOVERY OK — feedback live and normal mode verified. Re-enable motion deliberately."
  exit 0
else
  log "8/8 RECOVERY INCOMPLETE — do NOT re-enable arm motion (fb_ok=$fb_ok nm_ok=$nm_ok)."
  log "     Firmware has no MIT command watchdog: use the PHYSICAL e-stop if the arm still moves."
  exit 1
fi
