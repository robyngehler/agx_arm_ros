# Step-and-Settle Implementation Summary (arm↔hand handshake)

Companion to `shared_can_step_and_settle_integration_plan.md`. It records, minimally but exhaustively,
every problem addressed while implementing that plan — the fix and the rationale for each — plus the
review remediation, the teach/storage work, and what is deliberately out of scope or still
hardware-dependent. Ordered by theme; commit hashes are on branch `ROS2_Duo_System`.

Baseline: plan commit `13cde02`. All work below sits on top of it.

## 1. MIT setpoint safety — never leave a moving arm

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| The Nero firmware has no MIT command watchdog: it runs the last command forever, so killing nodes / going silent leaves a moving arm (runaway observed live). | On stale feedback (and shutdown) stream a kd-damped zero-velocity dead-man stop instead of pausing. | Silence is not a safe state; a damped stop is the firmware's last setpoint. | `ec75b83` |
| The stale-feedback abort left `active_trajectory` and its monotonic clock armed; when feedback returned the loop sampled a far-ahead point → position snap under MIT gains. | Drop the trajectory on the first stale tick, reset the hold so the current pose is recaptured on return; the action abort clears it and enters `CANCELING_TO_HOLD`. | Never resume a trajectory whose clock ran through the outage. | `203ce83` |

## 2. Bus-recovery watchdog — recover on real loss, not local starvation

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| Feedback went "stale" under GIL/CPU saturation (active MIT pegs the node) while the bus was alive, triggering heavyweight reconnects mid-trajectory. | Frame-advance liveness from the kernel RX timestamp; recovery cooldown; consume latched comm errors; do not reconnect on TX congestion while feedback is live. | The FPS window and node clock starve locally; the kernel RX timestamp is ground truth. | `9fd6504`, `2d4988a`, `b78a965`, `71affb9` |
| Both the `not is_ok()` and node-clock-stale recovery branches still false-triggered under whole-process starvation. | Gate both branches on `_feedback_actually_stale()` (kernel RX timestamp); add Phase-0 publish-loop jitter instrumentation + per-reason recovery counters. | Recover only when frames actually stopped advancing; make attribution observable. | `2c5787a` |

## 3. Emergency stop — verify the effect, don't log a phantom success

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| `emergency_stop` sent a command and logged success unconditionally; under ENOBUFS the SDK silently drops the command, so a "stopped" arm could still be moving. | Verify joint velocities settle in feedback; escalate to `electronic_emergency_stop`, then a forced link-reset recovery; report UNVERIFIED instead of success. | A stop command proves nothing; trust the observed effect. | `1850e10` |

## 4. Arm↔hand window handoff — the core step-and-settle contract

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| The scheduler treated same-side arm and hand as independent, though they share one physical CAN bus. | `graph_model.ROBOT_UNITS` gains `left_can_bus`/`right_can_bus`; same-side arm and hand now conflict; `both_arms` owns both. | Serialize what physically contends. | `4fa1737` |
| No primitive to hand bus ownership from arm to hand. | `prepare_hand_window` / `resume_arm_control` driver services: quiesce the arm into a hold and gate MIT, then reopen; the driver is the MIT gateway (`control/move_mit` → `agx_arm.move_mit`). | Quiescing belongs at the last hop to hardware. | `5827876` |
| The window guard was only checked in `_move_mit_callback`, so `move_j`/`js`, pose/line/circle and the follow path could still inject arm frames. | Move the guard into `_check_can_control`, the single gate every arm command funnels through. | "Hand owns the bus" must hold for all ingress, not just MIT. | `043a78a` |
| The hold claimed "normal mode" but only checked "not teaching"; on Nero V112 `set_normal_mode` is a firmware no-op, so the claim could be false. | Verify by `ctrl_mode` readback (must be `CAN_CTRL`/`TCP_CTRL`) plus settled velocities; report the actual mode. | Trust the readback, not the call — version-agnostic and V112-honest. | `2eefec6` |
| The services existed but nothing called them: the scheduler serialized but never performed the verified handoff, and an arm "completing" in MoveIt does not stop MIT hold streaming. | Coordinator calls `prepare_hand_window` before every hand action and `resume_arm_control` after (all exit paths reopen). | Wire the safety goal end to end. | `b862e6b` |
| MoveIt hand planning-group goals reach the FJT bridge directly, bypassing the handshake → the hand lost arbitration under arm MIT (`请求超时`). | The FJT bridge wraps every goal in `prepare_hand_window` → run → `resume_arm_control`; tolerant of a hand-only bringup. | Any hand command on the shared bus needs the window, transparently to the caller. | `90fc7a8` |
| The teach manager could not switch the hand off/on or trigger hand commands. | Hand mode (`g`): capture (`c`) the current hand pose into `omnihand_pro_gestures.yaml`, replay (`f`) a selected skill, each wrapped in a per-command handshake. | Make the hand usable alongside the always-on arm MIT teach loop. | `25378b7` |

## 5. Fault handling & observability

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| After a bus recovery the publish loop silently re-armed `control_ready` and accepted motion again. | Latch a fault lockout (`require_fault_ack`): refuse all motion until `clear_fault_lockout`; publish latched `feedback/fault_lockout` for supervisor visibility. | A recovery is a fault; require an explicit acknowledgement. | `4993bcf` |
| Silent TX loss was invisible: `last_error` is cleared by every successful RX, so a TX-side ENOBUFS was wiped within milliseconds and the once-per-tick poller missed it. | Vendor fork adds `last_send_error` (survives RX) + monotonic send/recv counters; the node logs a rising count while feedback is live; a startup check warns loudly if the fork is absent. | Make dropped commands observable; do not degrade silently. | `c0c174f`, `4993bcf` (+ pinned fork) |
| Only a manual `ip link down/up` + restart reliably stopped the arm after a disconnect-while-moving; auto-recovery does not stop the hand or cancel arm goals. | `scripts/recover_shared_can_arm.sh`: cancel MIT + hold → stop the hand → verified e-stop → link reset → wait for feedback → verify normal mode → succeed only when confirmed. | A disconnect-safe, operator-runnable sequence with an explicit success gate. | `cbe57e3` |

## 6. Teach & storage

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| Anchor-pose side lived in an `_L`/`_R` name suffix; a prior fix wrote a `both_arms` capture as two suffixed 7-DoF poses. Renaming broke side detection and `both_arms` was never a first-class entry. | Store the resource explicitly: `name: {robot_id, q: [...]}`; `both_arms` is one 14-DoF entry. Legacy bare lists still load (suffix fallback). | The resource is chosen in the UI at save time, so store it as a fact. | `7848a5c` |

## 7. Transport tooling

| Problem | Fix | Rationale | Commit |
|---|---|---|---|
| Unclear whether transceiver TDC timing was the lever for hand starvation; one-shot policy was in flux. | Add `scripts/can_tdc_sweep.py`; keep `one-shot on` as the shared-bus safety baseline. | Sweeps showed a wide flat TDCO window and 0 % hand delivery under MIT load at every TDCO — timing is not the lever; retransmission buildup (one-shot off) risks arm runaway. | `fefefaa` |

## 8. Deliberately out of scope (MVP decisions)

- **Concurrent grasp-and-carry** (hand keeps holding while the arm moves on the same bus): out of scope. A
  held grasp is assumed to need no active connection; the flow is strictly sequential (arm moves → quiesce
  → hand acts → resume → arm moves). The coordinator hand-hold tracking was implemented then reverted.

## 8b. Post-review remediation (2026-07-23)

Closes three plan items an integration review found still open. Ordered by theme.

| Problem | Fix | Rationale |
|---|---|---|
| Phase 5 gap: the operator-facing `agx_arm_duo_soft_estop` only fanned into `mit_controller/cancel_trajectory` + `hold_current`, bypassing the verified, self-escalating driver e-stop — the central e-stop was the *weakest* stop path (plan §1.5, §5). | The duo e-stop now issues the instant soft hold on every arm, then hard-escalates each side to the hand-coordinated recovery service (below), falling back to the arm driver's own verified `emergency_stop` when that node is absent. | The operator button must reach the same last-resort stop+recover the driver already implements, not a strictly weaker path. |
| Plan §2.2 deliverable 2 (programmatic recovery) was missing: disconnect recovery existed only as the operator-run `scripts/recover_shared_can_arm.sh`, so the coordinator/supervisor could not invoke it in-graph. | New `agx_arm_shared_can_recovery` node (`agx_arm_mit_tools`) exposes a per-side `recover_<side>` Trigger mirroring the script's order (cancel → **stop hand before any link reset** → verified arm e-stop that self-escalates to a bus-recovery link reset → wait for feedback → force+verify normal mode → re-check hand). The bash script stays as the sudo-capable, ROS-graph-independent fallback. Launched alongside the duo e-stop in the multi-arm MoveIt bringup. | Automation needs the same disconnect-safe sequence the operator has, without shelling out. |
| The `agx_arm_ctrl` `emergency_stop` was an `Empty` service: its verified/`UNVERIFIED` result and any forced-recovery lockout were log-only, unreadable by a supervisor (plan §2.4). | `emergency_stop` is now a `Trigger`: `success` is True only when the arm is confirmed stopped; `message` reports the result and, when the last resort forced a recovery, `fault_lockout=latched`. **Public ROS contract change** (`std_srvs/Empty` → `std_srvs/Trigger`); the recovery helper and duo e-stop updated to match. | A stop service must return whether the stop is trustworthy, not only log it. |
| Fault-lockout coordination was implicit: neither the script nor a recovery path told the initiator that motion stays refused until `clear_fault_lockout`. | The recovery service and script now **report** `fault_lockout=latched` back to the caller and never clear it themselves — deliberate re-arming stays the initiator's decision. | Recovery is a fault; clearing the lockout is a separate, deliberate act by whoever owns re-arming. |
| The `resume_arm_control` docstring over-promised that "the MIT controller re-captures its own hold reference when it resumes". | Docstring corrected: resume only reopens the gate; the MIT loop keeps its window-open hold reference (equal to the parked pose), so any sag yields a small, bounded, intended position correction — not a snap. | State what the code guarantees, not more. |

## 9. Open / hardware-dependent

- **V112 `set_normal_mode` no-op:** `prepare_hand_window` now fails honestly (reporting the real `ctrl_mode`)
  if the arm does not reach a `CAN_CTRL`/`TCP_CTRL` hold on V112; the actual V112 hold mechanism must be
  confirmed on hardware.
- **Startup hand connection error + backoff:** tolerant-by-design (retries); real recovery depends on the
  hand/CAN/SDK being reachable.
- **Hardware validation (plan §6.2):** CPU-stress, disconnect-under-MIT, silent-TX-loss, and hand-across-
  link-reset tests were not run in the development environment — verify on the Jetson target.

## Validation

Unit tests green across the touched packages (`agx_arm_ctrl`, `agx_arm_coordination`,
`agx_arm_mit_controller`, `agx_arm_mit_demos`, plus the vendor fork's comm test). The §8b remediation
adds pure-helper tests for the recovery ordering (`test_shared_can_recovery.py`: hand-stop before arm
e-stop before normal-mode), the duo e-stop path helpers, and the script's Trigger e-stop + lockout
handoff — all green. Runtime and hardware paths (service calls, hand feedback/command, mode readback on
real firmware, and the in-graph `recover_<side>` sequence end to end) need on-robot verification.
