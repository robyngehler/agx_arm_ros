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

## 8c. Root cause on hardware (2026-07-24): the window did not free the bus

First full-handshake hardware run on the Jetson: the handshake itself behaved exactly as designed (arm
settles, verified `CAN_CTRL` hold, window opens and closes), yet every hand command still returned
`请求超时`. A/B `candump` on `can_nero_left` measured **~2150 frames/s while the arm merely holds**, and
**inside an open window the rate was unchanged (~2180 f/s)** — all low arm IDs (`0x2Ax`/`0x25x`/`0x26x`).
So the load is the **arm's own feedback push (Nero→host)**, not MIT commands (those *are* gated), and the
hand's high-ID, low-priority CANFD frames keep losing every arbitration (dropped, not retried, under the
one-shot baseline). `motor Input size does not match expected motor count` is a downstream cascade of the
timed-out empty read, not a joint-count config bug.

| Problem | Fix | Rationale |
|---|---|---|
| `prepare_hand_window` called `set_normal_mode()`, and the Nero driver's `set_normal_mode()` sets `enable_can_push = ENABLE`. The window **actively turned the flood back on** and could not free the bus by construction. | The window now silences the feedback push itself after the hold is verified, and restores it on resume (`hand_window_silence_feedback`, default on). | The push — not the command stream — is what starves the hand. |
| The only stock APIs that silence the push are `set_leader_mode`/`set_follower_mode`, i.e. they bundle "quiet bus" with a **mode switch**. Leader mode is zero-force drag: the firmware has no gravity model for this mounting pose and none for the end-effector payload, so the arm would sag. | New repo-owned `agx_arm_ctrl/nero_can_push.py` sends only the mode frame's push bit (`move_mode = 255` = no change), keeping the arm in the CAN-control hold it is already in after `agx_arm_ctrl` enables it. | The window needs a *quiet* arm, not a *limp* one. The vendor SDK stays untouched (pinned submodule). |
| Silencing the push blinds the bus-recovery watchdog (no feedback looks exactly like a dead bus). | `_should_recover_bus` treats a requested silence as healthy, but bounded by `hand_window_max_silence_s` (default 10 s): past that the push is restored and the watchdog re-armed, while arm commands stay gated until `resume_arm_control`. Recovery, `emergency_stop`, `set_normal_mode`, `set_leader_mode` and node shutdown all restore the push first — every one of them verifies in feedback. | A deliberate silence must never be read as a stall, and must never outlive its window. |
| A missing/failed silencing would have been invisible. | `prepare_hand_window` still opens the window (arm held, MIT gated) but reports and logs it as a **warning** naming the reason. | An honest window that cannot free the bus beats a silent one. |
| **Who** holds the arm was never verified — only *that* it was held (`ctrl_mode`). The hold is a MOVE-J executed by the arm's own position controller, but `move_j` emits its MOVE-J mode frame through `_maybe_set_motion_mode`, which the MIT streaming path disables around its batches: a concurrent batch would have made `move_j` skip the frame, leaving the firmware in MIT — waiting for host commands the window is about to cut off, with no feedback left to compute a correction. | Force auto mode-setting on before the `move_j`, and additionally verify `mode_feedback` is **not** a MIT move mode before silencing. The MIT code is read from the active driver (`0x04` below firmware v111, `0x06` from v111 — the two arms run 1.06 and 1.11, so a hardcoded set would misjudge one). Unknown encodings pass but are reported. | With the push silenced only a firmware-closed loop can correct drift; the host has nothing to close a loop with. |

## 8d. Second hardware run (2026-07-27): the MOVE-J hold frame was being dropped

`components.launch` in `moveit_mit`/`duo_hand`, both arms up (right 1.06, left 1.11), a MoveIt `left_hand`
goal. The §8c `firmware_holds` check fired exactly as intended and **refused the window honestly**:

```
hold NOT verified (settled=True, holding=True, firmware_holds=False,
                   ctrl_mode=CAN_CTRL(0x1), move_mode=MOVE_MIT(0x6)); hand window not opened
```

Read-only `candump` of `0x2A1` confirmed both arms sit in their MIT move mode under the always-on MIT
hold (left `01 00 06`, right `01 00 04`). So after `prepare_hand_window` sent its single `move_j`, the
left arm was still in `MOVE_MIT`: the one MOVE-J mode frame lost arbitration on the still-flooded
one-shot bus and was dropped — precisely the "SDK drops mode frames under saturation" failure the whole
effort is about. The preceding damped stop is `kp=0` (velocity damping, **no** position hold), so a
dropped MOVE-J would also leave the arm sagging; the refusal reopens the gate and the external MIT
controller recaptures the hold, so there is no safety gap — but the window can never open.

| Problem | Fix | Rationale |
|---|---|---|
| The MOVE-J hold frame is sent once. On the flooded one-shot bus (the push is still on — it is only silenced *after* the hold is verified) that single frame is easily dropped, so the firmware stays in MIT and the window is refused every time. | `_assert_firmware_hold` re-sends the same-pose, motionless MOVE-J until the readback stops reporting a MIT move mode, bounded by `hand_window_hold_assert_s` (1 s) / `hand_window_hold_poll_s` (50 ms). The attempt count is surfaced in the service message (`move_j xN`) as live evidence of how lossy the bus was. | A mode frame that must land on a saturated bus needs the same verified-retry treatment every other handoff step already gets; sending it once and trusting the readback is exactly what fails. |

Order matters and is enforced: capture pose → **re-assert MOVE-J until the firmware confirms it left
MIT** → verify hold in feedback → silence push; and on resume: **restore push → wait for a new frame**
→ health checks → reopen the gate.

## 8e. Third hardware run (2026-07-27, later): window validated, and a second, independent bottleneck

With the §8d retry in place the window now **opens reliably on hardware**: the log shows
`hand window open: … move_j x2 …, MIT quiesced, feedback push silenced (verified: feedback stopped)`.
The core diagnosis stands and the fix works: the arm feedback push saturates the shared bus with
high-priority (low-ID) frames, the hand's low-priority CANFD frames lose against them under one-shot,
and **silencing the push (the window) is what lets the hand own the bus** — WITHOUT the teach stack a
hand skill lands in a handful of attempts. Two follow-on fixes made the window well-behaved under the
real stack:

| Problem | Fix | Rationale |
|---|---|---|
| The external MIT controller lost its (intentionally silenced) feedback during a window and dead-manned: it streamed 50 Hz damped-stop commands into the gate, wasting CPU and logging "Feedback is stale" every second. | The driver publishes a latched `feedback/hand_window_active` Bool; the MIT controller stands down (new `HAND_WINDOW` state, publishes nothing) while it is true and recaptures the hold when it clears. | The arm is held by the firmware during the window, so the controller must be *told* the silence is expected instead of reading it as a dead bus. |
| The MoveIt FJT bridge held the window open for the whole trajectory duration + margin, and the teach manager for a fixed `hand_settle_sec` — both keep the arm silenced (and, under teach, the dead-man going) far longer than the hand needs, and the fixed teach dwell could close the window mid-retry. | FJT closes on verified delivery (`OmniHandStatus.command_pending` cleared), not duration (was P2). Teach's `_await_hand_delivery` holds its window on the same signal instead of a blind dwell. | The OmniHand moves autonomously once it has the target; free the bus the moment delivery is confirmed, and never close it mid-retry. |

**Additional, independent problem surfaced by the interface stats — NOT a correction of the above.**
`ip -s -d link show` over a full teach session: `arbit-lost` = 9–10 of ~2.5M TX, but **~108k RX frames
`dropped` per bus** and the left bus in `ERROR-WARNING` (TEC 107). So beyond arbitration there is a
second bottleneck the window alone cannot fix: the kernel CAN **RX socket buffer** (`net.core.rmem_max`
default ~208 KB ≈ 270 frames ≈ ~125 ms at 2150 f/s) overflows during the 200 ms+ publish-loop overruns
the full teach stack causes (CPU starvation), dropping frames — including the OmniHand's CANFD
*response* frames — so the hand still `请求超时`s **even with an open window** under heavy load.

| Problem | Fix | Rationale |
|---|---|---|
| Under the full teach stack, CPU stalls (200 ms+ publish-loop overruns) overflow the ~125 ms RX socket buffer and drop hand response frames → `请求超时` even inside an open window. | `activate_native_can.sh` raises `net.core.rmem_max`/`rmem_default` to 4 MB (≈ 2 s of buffer; `RMEM_MAX` env, `0` to skip). | A scheduling hiccup must not cost hand responses; deeper buffering absorbs the stall. The CPU load itself is a `sprint_refactor` target. |
| On a shared bus the whole handshake is mandatory; with a dedicated hand bus it is pure overhead. | `hand_bus:=shared|dedicated` launch arg turns the handshake off end to end (FJT `handshake_enabled`); teach has `--no-hand-window`. Wired through components → moveit/multi-arm → per-arm driver. | Make the shared-bus workaround switchable so a second CAN line restores parallel arm+hand operation. |

## 9. Open / hardware-dependent

- **V112 `set_normal_mode` no-op:** `prepare_hand_window` now fails honestly (reporting the real `ctrl_mode`)
  if the arm does not reach a `CAN_CTRL`/`TCP_CTRL` hold on V112; the actual V112 hold mechanism must be
  confirmed on hardware. (On the tested robot the hold *was* verified as `CAN_CTRL` — see §8c.)
- **Push silencing (§8c) + MOVE-J retry (§8d): hardware-validated (2026-07-27).** The window opens
  (`move_j x2`), the push is verified silent, and without the teach stack a hand skill lands. The MIT
  stand-down and FJT/teach close-on-delivery (§8e) are also on hardware.
- **P2 (FJT closes on duration): DONE (§8e).** FJT and teach now close on verified delivery.
- **Remaining teach unreliability is CPU-bound, not window logic (§8e).** Under the full teach stack the
  RX socket buffer overflows during CPU stalls and drops hand response frames. The rmem raise is a
  mitigation; the root CPU-load reduction is a `sprint_refactor` target (see the critical CPU paths there).
- **Startup hand connection error + backoff:** tolerant-by-design (retries); real recovery depends on the
  hand/CAN/SDK being reachable.
- **Dedicated hand bus:** the `hand_bus:=dedicated` path (parallel operation) is wired but unexercised —
  validate once the second CAN adapter is in place (point the hand's `can_interface` at it).

## Validation

Unit tests green across the touched packages (`agx_arm_ctrl`, `agx_arm_coordination`,
`agx_arm_mit_controller`, `agx_arm_mit_demos`, plus the vendor fork's comm test). The §8b remediation
adds pure-helper tests for the recovery ordering (`test_shared_can_recovery.py`: hand-stop before arm
e-stop before normal-mode), the duo e-stop path helpers, and the script's Trigger e-stop + lockout
handoff — all green. Runtime and hardware paths (service calls, hand feedback/command, mode readback on
real firmware, and the in-graph `recover_<side>` sequence end to end) need on-robot verification.
