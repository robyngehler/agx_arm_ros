# Proposal: CAN ENOBUFS & MoveIt Configuration Jump (AGX Nero Arm, Jetson Orin AGX)

**Status:** Draft · **Date:** 2026-06-12

## 1. Problem Summary

1. **ENOBUFS (Error 105)** on `can_nero_left/right` (gs_usb, USB-CAN adapter) after the first
   plan-&-execute cycle. No further transmission is possible until the ROS launch file is
   restarted. The CAN interface itself is not brought down/up in between.
2. **Configuration jump** during execution: the left arm jumps by ~π on joint1
   (`PATH_TOLERANCE_VIOLATED: -3.066 > 0.500`), triggered after replanning around an obstacle.

## 2. Root Cause Analysis

### 2.1 gs_usb TX Slot Leak (primary suspect, new)

`dmesg` repeatedly shows `Unexpected unused echo id N` and once
`Unexpected out of range echo id 4096`. This is a known TX echo accounting problem
of the gs_usb driver (kernel 5.15.122-tegra, out-of-tree module) in combination with the
adapter firmware. Consequence: hardware TX slots (typically ~10 per device) are leaked and
remain occupied until the interface is restarted. After enough leaks, every transmit attempt
returns **ENOBUFS permanently — regardless of bus state**. This explains the
"works exactly once per restart" pattern.

### 2.2 ACK Stall (secondary path)

If the arm enters an error/silent state (e.g. after PATH_TOLERANCE_VIOLATED), no ACK is
present on the bus. With ACK errors, a CAN controller **never goes bus-off** per
specification (TEC saturates at error-passive) → `restart-ms` never triggers, the controller
retransmits indefinitely, the TX queue fills up → ENOBUFS. `one-shot` would mitigate this,
but the current adapter firmware does not advertise it (missing from ctrlmodes,
`RTNETLINK: Operation not supported`).

### 2.3 Follow-up Deadlock in the Driver

After ENOBUFS the arm driver gets stuck: `is_ok() == False` → no joint states →
`control_ready == False` → callbacks return early → the MIT controller rejects goals.
No recovery path exists.

### 2.4 Configuration Jump (MoveIt)

A jump of ~π on joint1 points to a goal IK solution in a distant configuration
(elbow/base flip); RRTConnect plans toward it without any joint-space distance preference.
Aggravating factor: stale joint states caused by the bus stall → planning starts from a
wrong start pose.

## 3. Measures

### P1 — ENOBUFS Recovery in the Driver (immediate, no HW changes)

- Use `bus.send(msg, timeout=0)` non-blocking; catch `ENOBUFS` and treat it as
  "bus stalled".
- Recovery sequence: stop MIT streaming → `ip link set <if> down && up`
  (flushes the qdisc, resets gs_usb TX slots and pending retransmits) →
  enable handshake with the arm → set `control_ready = True` only once feedback
  frames are received again.
- RX watchdog: if no status frames arrive from the arm for >N ms, proactively stop
  streaming (prevents ENOBUFS instead of reacting to it).

### P2 — Firmware/Driver Update (root cause 2.1 & 2.2)

- Flash the adapter firmware to a current candleLight_fw release (dfu-util).
  Expected: `GS_CAN_FEATURE_ONE_SHOT` becomes available → `one-shot on` works;
  additionally includes fixes in echo handling.
- Update the gs_usb driver (newer kernel or backport/DKMS); the echo-id fixes
  landed upstream after 5.15.
- Afterwards in `can_activate.sh`: `type can bitrate 1000000 restart-ms 100 one-shot on`.

### P3 — Interface Hardening in `can_activate.sh`

- Set `txqueuelen 1000` (burst buffer; not a fix for 2.1/2.2, but reduces false alarms).
- Enable `berr-reporting on` for diagnostics; log `ip -details -statistics link show`
  after errors.

### P4 — MoveIt: Enable TracIK Instead of Handling Edge Cases

- **Activate the already integrated TracIK kinematics plugin** with
  `solve_type: Distance` → IK solutions minimize joint-space distance to the seed and
  suppress configuration flips at the source. Note: TracIK is an IK solver,
  not a planner — RRTConnect remains the planner; both work together.
- Call `set_start_state_to_current_state()` before every plan + freshness check on the
  joint states (couples with P1: never plan on stale data after a bus stall).
- Check joint1 feedback for ±π wrapping and unwrap in the driver if necessary.
- Keep `allowed_start_tolerance = 0.01` unchanged (safety net).

## 4. Order & Effort

| Prio | Effort | Risk |
|------|--------|------|
| 1 | P1 ENOBUFS recovery + watchdog | low |
| 2 | P4 TracIK (Distance) + start-state check | low (plugin available) |
| 3 | P2 firmware/driver update | medium (flashing on target system) |
| 4 | P3 script hardening | low |

## 4a. Implementation Status (2026-06-12)

- **P1 — done (pending hardware validation).**
  - `pyAgxArm` now records a *sticky* TX-stall indicator that a successful `recv()`
    does not clear (`CanCommImpl.tx_error_count` / `clear_tx_error()`), exposed via
    `arm_driver_abstract.get_tx_error_count()` / `clear_tx_error()`. This is the
    reliable ENOBUFS slot-leak signal even while RX frames keep arriving.
  - `agx_arm_ctrl_single_node` gained a bus-recovery state machine in the publish
    thread: it detects a TX stall (`tx_error_count`), a lost link (`is_ok()` False)
    or stale feedback (`feedback_timeout`), gates all control callbacks off via
    `control_ready`, then disconnect → optional `ip link down/up` → reconnect →
    re-enable → wait for fresh feedback before re-arming. New parameters:
    `bus_recovery_enabled` (True), `bus_recovery_tx_error_threshold` (1),
    `feedback_timeout` (0.5 s), `bus_recovery_link_reset` (False, needs privileges
    for `sudo ip link`), `bus_recovery_max_attempts` (3).
  - The RX watchdog of §3-P1 is folded into the same detector (no separate thread).
- **P4 — already wired; no code change.** `_moveit_config_builder._build_kinematics`
  already assigns TracIK with `solve_type: Distance` to `left_arm`/`right_arm` for
  every non-default profile (incl. `both_arms`), overriding `config/kinematics.yaml`
  at `build_moveit_config`. Planning is triggered from RViz, so the
  `set_start_state_to_current_state()` item is an RViz-usage note, not repo code.
  With TracIK Distance active and P1 in place, the joint1 ~π jump is most likely the
  execution-time consequence of the bus stall (§2.1/2.2) rather than a goal-IK flip.
- **P3 — done.** `config/can_interface_roles.json` gains `nero_left`/`nero_right`
  roles and raises `nero` `tx_queue_len` to 1000; `prepare_can_interfaces.py`
  accepts the new roles. `can_activate.sh` and `can_muti_activate.sh` now apply
  `restart-ms`, `txqueuelen` (default 1000) and best-effort `berr-reporting on`.
- **P2 — not actionable here** (adapter firmware flash + gs_usb/kernel update on the
  target). `one-shot on` remains unavailable until the firmware advertises it.

## 5. Acceptance Criteria

- Multiple consecutive plan-&-execute cycles without restarting the launch file.
- After a provoked arm fault (e-stop), the driver recovers automatically (<5 s)
  without a permanent ENOBUFS state.
- No more `Unexpected unused echo id` messages in `dmesg` after P2.
- No joint1 jump >0.5 rad between the planned start trajectory and the actual state
  over 20 planning cycles with an obstacle in the octomap.
