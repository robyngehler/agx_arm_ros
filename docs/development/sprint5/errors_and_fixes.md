# Sprint 5 — Errors & Fixes

## ENOBUFS bus stall on the Duo arms (resolved by native CAN + one-shot)

**Symptom.** With both arms on USB `gs_usb` adapters, after the first MoveIt plan&execute the CAN
TX path wedged: `Failed to transmit: No buffer space available [Error Code 105]`. No further
motion until the whole launch was restarted. It rarely surfaced for a single arm.

**Root cause.** Not per-channel CAN bandwidth and not command routing (each arm has its own
namespaced `move_mit` stream on its own channel; the 200 Hz publish thread reads cached RX state
and emits no CAN requests). The amplifier was **shared USB + the `gs_usb` TX-echo slot leak**:
`lsusb -t` confirmed both adapters on one host (Bus 01) behind one hub (1-4) at 12 Mbit
full-speed, sharing a transaction translator. Concurrent TX plus CPU contention starving the
RX/echo-consume thread leaked `gs_usb` echo slots until TX returned ENOBUFS permanently.
Background: `docs/assets/control/single_vs_multi_arm_control_chain.md`.

**Fix (the real one).** Migrate the arms to the Jetson **native `mttcan`** controllers
(`can0`/`can1`, 40-pin header) with **`one-shot on`**. `one-shot` stops the controller from
endlessly retransmitting an unacknowledged frame (the behaviour that spammed the bus and filled
the TX queue); `mttcan` supports it, the `gs_usb` firmware did not. This removes the shared USB
host/TT and the echo-slot leak entirely. See `planning/can_transport_decision.md`.

**Secondary mitigations (now defense-in-depth, not the fix).**
- Node-side bus-recovery watchdog in `agx_arm_ctrl` (detect a stalled ENOBUFS path, reconnect,
  reset `control_ready`). Kept as a safety net; no longer the primary remedy.
- `restart-ms` / `txqueuelen` hardening in the legacy USB `can_activate.sh` now only matters for
  any remaining USB CAN adapter, not the natively-wired arms or the native OmniHand bus.

## Control-layer ran a stale, frozen pyAgxArm (resolved by editable install + submodule pin)

**Symptom.** Code changes to `pyAgxArm` had no effect on the running arms.

**Root cause.** The ROS stack runs on system `python3.10`, which imported a **non-editable
pyAgxArm snapshot frozen at 2026-04-09** in `~/.local`. All newer SDK work (Nero v112, comm
rework) lived only in a conda **base / python 3.13** editable install that cannot import `rclpy`,
so it never drove the ROS runtime.

**Fix.** Editable-install pyAgxArm into system 3.10 (`--no-build-isolation`, system setuptools
predates PEP 660) and **vendor it as a pinned git submodule** at `vendor/pyAgxArm`
(`control-layer-pin-2026-06-12`). Details: `docs/project/control_layer_and_dependencies.md`.

## joint1 ~π configuration jump on Duo (PATH_TOLERANCE_VIOLATED)

**Finding.** TracIK with `solve_type: Distance` is already loaded for the `left_arm`/`right_arm`
groups (confirmed in the bringup log), so the IK-branch flip is already suppressed at the solver.
**Open:** add `set_start_state_to_current_state()` + a joint-state freshness check and a joint1
±π unwrap in the planning path. Tracked in `open_questions.md`.

## left arm "CAN port not UP" (setup error, now moot)

Both arms were brought up with the same USB bus-info, so the second `can_activate.sh` renamed the
first interface away. Moot after the native `can0`/`can1` migration.
