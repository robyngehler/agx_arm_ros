# Shared CAN Step-and-Settle Integration Plan

Status: investigation and integration plan, 2026-07-19. Updated same day after inspecting the pinned
`vendor/pyAgxArm` submodule (`37d87e6`); the SDK findings in section 1.3 revise parts of Phase 1 and
add one previously undocumented hazard (silent TX command loss, section 1.3.2).

This note captures the current shared arm-plus-hand CAN behavior after the repo cleanup and turns it
into an implementation plan for explicit arm-hand ownership switching. It complements the current
stable guidance in `docs/errors_and_fixes.md`, `docs/control/bringups/teach_and_run.md`, and
`docs/sprint5/evidence/can_transport_decision.md`.

Historical lineage:

- the deleted Sprint 5 proposal `docs/development/sprint5/planning/arm_plus_hand_shared_can_proposal.md`
  was removed during the 2026-07-18 docs cleanup
- its stable safety guidance was promoted into the top-level docs above
- the remaining integration work is not solved by those summaries, so this note records the current
  code-level gaps and a package-scoped plan

## 1. Current implementation findings

### 1.1 `agx_arm_ctrl` owns the current bus-recovery logic

The active recovery logic is in `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`, not in a
repo-visible pyAgxArm transport layer.

Current behavior:

- `_handle_send_failure()` classifies propagated send exceptions as recoverable when they look like
  `ENOBUFS` or `ENETDOWN`, increments `_tx_stall_count`, and arms `_tx_stall_detected`
- `_should_recover_bus()` triggers recovery on any of:
  - `_tx_stall_detected`
  - `agx_arm.has_comm_error() and agx_arm.get_comm_error()` if that error also classifies as
    recoverable
  - `not agx_arm.is_ok()`
  - `(time.monotonic() - _last_good_feedback_monotonic) > feedback_timeout`
- `_recover_bus()` immediately drops `control_ready`, disconnects, optionally executes `sudo ip link
  set <channel> down` and `up`, reconnects, optionally re-enables the arm, restores speed and TCP,
  clears motion-mode bookkeeping, and waits for feedback again

Important consequences:

- the watchdog is evaluated from the driver's publish thread, not from an SDK-side transport thread
- `_last_good_feedback_monotonic` is refreshed only when the node observes valid feedback inside that
  thread
- a local scheduling stall can therefore look identical to a real feedback stall if the process does
  not get CPU for longer than `feedback_timeout`

This matches the current top-level warning that heavy CPU load can freeze the timer and falsely start
bus recovery.

### 1.2 The current stale-feedback trigger is vulnerable to false positives

The current logic does not distinguish these cases:

1. the CAN bus is actually stalled or down
2. the pyAgxArm RX thread is healthy, but the ROS publish thread was not scheduled for a while
3. the process is CPU-saturated and both feedback publication and watchdog evaluation resume late

Why this matters:

- the stale path is treated as equally severe as a real send failure
- the reaction is heavyweight: disconnect, optional link reset, reconnect, auto-enable, and motion
  mode reset
- this can interrupt an otherwise healthy arm control path and inject a recovery sequence while MoveIt
  and the MIT controller still believe they own the arm

The current code uses `time.monotonic()` and a node-local freshness marker, but it does not:

- verify a loop overrun separately from true transport loss
- require repeated stale observations before recovery
- compare the driver's last real feedback timestamp against wall-clock age
- coordinate recovery with the MIT controller state machine before reconnecting

### 1.3 pyAgxArm SDK audit (pinned submodule `37d87e6`, inspected 2026-07-19)

The `vendor/pyAgxArm` submodule was initialized and audited at the pinned commit. The repo documents
the intended ownership split in `docs/project/control_layer_and_dependencies.md` (pyAgxArm is the
pinned runtime SDK input; repo-level recovery stays in `agx_arm_ctrl`; SDK changes are reserved for
functionality that truly must live below ROS). The audit confirms the node-side watchdog is the only
backoff implementation, and it establishes four SDK facts that change parts of this plan.

#### 1.3.1 `send()` swallows ENOBUFS/ENETDOWN — the raise-style TX path is dead code

`CanCommImpl.send()` (`pyAgxArm/protocols/can_protocol/comms/can_comm.py`) catches all send
exceptions, records them in `last_error`, and for `ENOBUFS`/`ENETDOWN` **returns silently** without
raising. Only unclassified hard errors (for example ENODEV) close the bus and re-raise.

Consequence: the node's `_handle_send_failure()` / `_tx_stall_detected` path can never fire for the
errors it targets — they never propagate out of `comm.send()`. The only live detection path for
buffer exhaustion is the `has_comm_error()` poll ("Path B"). The Phase 1 recommendation below is
adjusted accordingly.

#### 1.3.2 `last_error` is self-clearing and racy — silent TX command loss is invisible (new hazard)

Every successful `send()` **or** `recv()` resets `last_error = None`. On a healthy feedback push
stream the SDK RX thread clears a TX-side `ENOBUFS` within milliseconds, and the node samples
`has_comm_error()` only once per publish tick, so detection is a race that is almost always lost.

For the documented gs_usb failure mode (TX slot leak while RX keeps flowing) the system therefore
shows: commands silently dropped, `is_ok()` true, feedback fresh, `has_comm_error()` false — **no
recovery trigger at all**. The arm simply stops obeying while everything looks healthy. This is more
dangerous than the false-positive stale trigger this note originally focused on, and it directly
affects the `agx_arm_ctrl` `emergency_stop` service, which sends `move_j(q)`/`move_js(q)`
fire-and-forget and then logs success: under ENOBUFS an emergency stop can silently do nothing.

#### 1.3.3 Kernel RX timestamps already exist — the exact signal Phase 1 needs

The table-driven parser stamps every cached feedback message with `rx_can_frame.timestamp`
(`drivers/core/table_driven.py`), which for SocketCAN is the **kernel receive timestamp**. It is
reachable through `get_joint_angles().timestamp`. Key property: if the process is CPU-starved,
frames queue in the socket buffer and carry their true arrival times when drained, so after a stall
`time.time() - js.timestamp` reliably distinguishes "bus kept running, only the node stalled" from
"bus actually went silent". This resolves the case-1/2/3 ambiguity of section 1.2 without any SDK
change. (Wall-clock caveat: NTP jumps; acceptable on the Jetson target, or compare deltas.)

Related: `is_ok()` is FPS-based and computed in the SDK monitor thread
(`drivers/core/submodel_driver_context_abstract.py`). Under publish-thread-only starvation it stays
stable (good), but under whole-process GIL/CPU saturation the parse path also stops, so `is_ok()`
**can itself false-trigger**. The stale debounce in Phase 1 must therefore cover the `not is_ok()`
branch of `_should_recover_bus()`, not only the `feedback_timeout` branch.

#### 1.3.4 Driver primitives for the handoff already exist

`set_normal_mode()` (also re-enables CAN push), `electronic_emergency_stop()` (damped stop),
`reset()`, `disable()`, and `get_arm_status().msg.ctrl_mode` readback are all present in the pinned
Nero driver. Phase 3 is implementable without SDK changes — but per 1.3.2 every mode change must be
verified by readback, never fire-and-forget.

### 1.4 MIT control currently pauses on stale feedback, but does not complete a safe ownership handoff

Verified in code: the stale-feedback abort in `_execute_follow_joint_trajectory()`
(`mit_controller_node.py`, around lines 845-851) is the **only** abort path that neither clears
`active_trajectory` nor sets an execution state (compare the position-limit path at lines 856-862,
which does both). The concrete failure: the control loop pauses publishing in `STALE_FEEDBACK`, but
when feedback returns (for example after a bus recovery), `_reference_from_state()` samples the
still-active trajectory at a monotonic `elapsed` time that kept running during the outage and
commands a far-ahead point with MIT gains — a position snap.

The production MIT controller in `src/agx_arm_mit_controller/agx_arm_mit_controller/mit_controller_node.py`
already has the right idea at a high level:

- the control loop stops publishing `MoveMITMsg` when feedback is stale
- active MoveIt goals abort when feedback becomes stale
- `~/hold_current` and `~/cancel_trajectory` services exist
- leader-mode and arm-fault transitions can cancel active trajectories

But the current behavior is not a complete safe-stop path for shared-CAN operation:

- the stale-feedback exit path in `_execute_follow_joint_trajectory()` aborts the action goal without
  explicitly clearing `active_trajectory` or forcing a driver-level hold/mode handoff
- the `STALE_FEEDBACK` state only pauses ROS-side publishing; it does not guarantee the arm firmware is
  returned to a safe normal-mode hold
- the existing hold and cancel services only manipulate MIT-controller state; they do not cycle the CAN
  link, do not force `set_normal_mode`, and do not invoke an arm-driver hold primitive

This leaves a dangerous gap when the arm loses connectivity during active MIT control: software above
the bus can stop sending, but the arm-side controller may still keep the last command or controller mode
until the CAN link is reset and the stack is restarted.

### 1.5 The current soft e-stop is MIT-only

`src/agx_arm_mit_tools/agx_arm_mit_tools/duo_soft_estop.py` fans `/emergency_stop` into per-arm
`mit_controller/cancel_trajectory` and `mit_controller/hold_current` services.

That is useful for normal trajectory cancellation, but it is not sufficient for the disconnect case
described above because it does not:

- switch the arm driver back to normal mode
- send a driver-level static hold in normal mode
- cycle the CAN interface
- reconnect `agx_arm_ctrl`
- require a post-recovery acknowledgement before motion resumes

The `agx_arm_ctrl` emergency-stop service is also not a complete answer: it simply sends the current
joint pose back through `move_j()` or `move_js()` and assumes the bus is still working. Per section
1.3.2 that assumption is unverifiable with the pinned SDK: under ENOBUFS the stop command is silently
dropped and the service still logs success. A trustworthy e-stop must verify the effect in feedback
(joint velocities going to zero) and escalate to `electronic_emergency_stop()` and then a link reset
when verification fails.

### 1.6 OmniHand already contains shared-bus mitigations, but not ownership arbitration

The OmniHand bridge in `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py` already carries two
important mitigations for the shared side bus:

- `joint_read_rate` is decoupled from ROS `pub_rate`, so hand polling can be reduced without slowing
  the ROS republish path
- hand commands are retried and verified against joint readback until success or give-up
- `control/omnihand/stop` clears a pending command and requests a backend hold

These are good transport mitigations, but they do not solve the ownership problem. They reduce hand-side
traffic and improve eventual delivery, yet the arm and hand can still compete for the same side bus if
the arm remains under active MIT control.

### 1.7 The coordination layer still models arm and hand as independent resources

`src/agx_arm_coordination/agx_arm_coordination/graph_model.py` currently defines:

- `left_arm`, `right_arm`, `both_arms`
- `left_hand`, `right_hand`

as independent resource units, and the catalogue in `src/agx_arm_coordination/config/catalogue.yaml`
still says:

- `CAN-bus tokens deferred until sprint-5 bus-load validation shows contention`

This is now outdated. The repo's current guidance already confirms shared-bus contention and demands
explicit hand-command windows. Without side-bus tokens, the scheduler still treats same-side arm and hand
actions as non-conflicting, even if a particular activity graph happens to serialize them with edges.

## 2. Safety conclusion from the current code

The repo already contains the pieces of a safe Step-and-Settle runtime, but they are not yet composed
into one ownership contract.

Today the system has:

- a driver-local reconnect watchdog
- a MIT controller that can pause on stale feedback and hold the current pose in software
- a hand bridge that can retry and stop hand commands
- a coordinator that can serialize actions by abstract resources

And it has two hazards the original draft under-weighted, both confirmed by the SDK audit:

- **silent TX command loss** (section 1.3.2): the exact documented gs_usb failure mode produces no
  recovery trigger at all, and can neutralize the current soft e-stop without any error surfacing
- **trajectory snap after stale recovery** (section 1.4): the stale abort leaves `active_trajectory`
  armed, so a feedback comeback can command a far-ahead trajectory point with MIT gains

What it does not yet have is one explicit state machine that says:

- who owns the side bus right now
- how arm control is quiesced before a hand window starts
- how the arm is held safely while the hand owns the bus
- what exact recovery sequence is executed after a disconnect during MIT control
- when motion is allowed to resume after recovery

## 3. Target Step-and-Settle state machine

Recommended ownership state machine per side:

```text
ARM_ACTIVE
  -> ARM_SETTLING
  -> ARM_DRIVER_HOLD
  -> HAND_WINDOW
  -> ARM_REACQUIRE
  -> ARM_ACTIVE

Any state
  -> FAULT_LOCKOUT
  -> RECOVER_LINK
  -> ARM_DRIVER_HOLD
  -> ARM_REACQUIRE
```

State meaning:

- `ARM_ACTIVE`: MoveIt and the MIT controller own the side; hand traffic is limited to passive feedback
- `ARM_SETTLING`: cancel or finish active MIT trajectory, wait until the arm reaches a static pose
- `ARM_DRIVER_HOLD`: hand off from MIT streaming to a low-traffic driver-level hold in normal mode
- `HAND_WINDOW`: hand skills own the side bus; arm MIT streaming remains quiesced
- `ARM_REACQUIRE`: stop hand retries, confirm bus health, re-enable MIT, capture hold, and return
  control to the arm path
- `FAULT_LOCKOUT`: refuse new goals after disconnect, stale recovery escalation, or bus-off event until
  explicit recovery succeeds
- `RECOVER_LINK`: execute the recovery sequence that can include `ip link down/up`, driver reconnect,
  driver re-enable, and mode reset

## 4. Package-scoped integration plan

### Phase 0 - Instrument before changing behavior

Owner: `agx_arm_ctrl`, `agx_arm_mit_controller`, docs.

Add enough telemetry to distinguish real bus loss from local scheduling delay:

- log publish-loop jitter in `agx_arm_ctrl` before calling stale-feedback recovery
- record last driver feedback timestamp separately from last node-observed publish success
- surface explicit counters for:
  - send-failure-triggered recoveries
  - stale-feedback-triggered recoveries
  - loop-overrun suppressions
  - MIT stale-feedback aborts

Success criterion: a CPU stress run can tell whether recovery was triggered by real send failure,
driver comm fault, or local loop starvation.

### Phase 1 - Make stale recovery conservative and comm-aware

Owner: `src/agx_arm_ctrl`, `vendor/pyAgxArm` (tracked fork).

Change the watchdog so stale feedback alone is no longer enough for immediate heavy recovery, and
make silent TX loss observable at all.

Recommended changes:

1. Treat the `has_comm_error()` poll ("Path B") as the only live detection path for
   `ENOBUFS`/`ENETDOWN`: per section 1.3.1 these are swallowed by the SDK and never propagate, so
   the `_tx_stall_detected` raise-style path cannot fire for them. Keep the exception classification
   only for genuinely propagated hard errors.
2. Patch the tracked vendor fork (allowed by the repo promotion rules; this is functionality that
   truly must live below ROS) to make TX loss observable: separate monotonic TX/RX error counters
   plus a `last_send_error` that is **not** cleared by RX success. Without this, section 1.3.2 makes
   ENOBUFS detection a race the node almost always loses.
3. Use the kernel RX timestamp (section 1.3.3, `get_joint_angles().timestamp`) as the stale arbiter:
   - detect stale node-observed feedback
   - check whether the publish loop itself overran badly
   - check whether the kernel-stamped driver feedback timestamp actually stopped advancing
   - require several consecutive stale confirmations before escalating
4. Apply the same debounce to the `not is_ok()` branch of `_should_recover_bus()`: per section
   1.3.3, `is_ok()` itself can false-trigger under whole-process CPU/GIL saturation.
5. Suppress automatic reconnect if the only evidence is a local loop overrun.
6. Gate post-reconnect auto-resume so the driver does not silently return to control-ready while the
   higher layers still assume a faulted state.

This preserves defense-in-depth recovery for real transport faults while removing the false-positive
timer trap under CPU saturation and closing the false-negative silent-loss gap.

### Phase 2 - Add a real disconnect-safe recovery path

Owner: `src/agx_arm_ctrl`, `src/agx_arm_mit_controller`, `src/agx_arm_mit_tools`, `scripts/`.

Add one explicit emergency path for the dangerous case "disconnect while MIT is active".

Recommended deliverables:

1. A repo-owned recovery helper in `scripts/`, for example
   `scripts/recover_shared_can_arm.sh`, that performs, in order:
   - call `mit_controller/cancel_trajectory`
   - request MIT quiesce / hold capture
   - call `control/omnihand/stop` so pending hand retries do not keep hammering the side bus and are
     not killed mid-command by the link reset
   - request a driver-level normal-mode hold if still reachable
   - bring the CAN interface down and up
   - reconnect and re-enable `agx_arm_ctrl`
   - force normal mode after reconnect and **verify it via `get_arm_status()` readback**
   - verify (or restore) the hand backend after the link reset — whether the OmniHand transport
     survives an `ip link down/up` is untested and must be validated in section 6.2
   - require an explicit success state before any arm action is re-enabled
2. A ROS-side service wrapper in `agx_arm_mit_tools` or `agx_arm_ctrl` so the same sequence can be
   invoked programmatically by the coordinator or supervisor node.
3. A lockout flag so MoveIt and MIT refuse new motion goals until recovery finishes and the operator or
   supervisor explicitly clears the fault.
4. Harden the existing `agx_arm_ctrl` `emergency_stop` service: verify the stop took effect in
   feedback (joint velocities go to zero within a bound), escalate to
   `electronic_emergency_stop()` and then to this recovery sequence when verification fails, and
   report failure instead of unconditionally logging success (section 1.3.2).

This is the minimum package-scoped answer to the current hazard that only a manual `can down/up` and
controller restart reliably stop the arm after a bad disconnect.

### Phase 3 - Introduce an explicit arm-to-hand handoff service

Owner: `src/agx_arm_ctrl`, `src/agx_arm_mit_controller`.

The desired Step-and-Settle behavior needs one side-local handoff primitive, not just ad hoc service
calls from many tools.

Recommended interface shape:

- `prepare_hand_window` service or action
- `resume_arm_control` service or action

`prepare_hand_window` should perform one atomic sequence:

1. reject if the side is already faulted or recovering
2. cancel any active MIT trajectory
3. wait for the arm to settle
4. capture the current pose as MIT hold target
5. switch the driver to normal mode and command a static hold at the current joint pose
6. quiesce MIT periodic publishing so the hand owns the side bus
7. return success only when the bus is quiet and the arm is in the hold state

Every mode-changing step must be verified by feedback readback (`get_arm_status().msg.ctrl_mode`,
joint velocities), never fire-and-forget: per section 1.3.2 the SDK silently drops mode frames under
bus saturation, which is exactly the condition a handoff is most likely to run into. The SDK
primitives needed here (`set_normal_mode`, `electronic_emergency_stop`, `reset`, `disable`, status
readback) all exist in the pinned driver (section 1.3.4), so no SDK change is required for this
phase.

`resume_arm_control` should:

1. stop or clear pending hand commands
2. verify healthy arm feedback and no comm fault
3. re-enable MIT if needed
4. capture the current arm pose as the first MIT hold reference
5. reopen the side for arm trajectories

This handoff should be the only supported entry to a hand-command window on the shared bus.

### Phase 4 - Model the side bus as a coordinator resource

Owner: `src/agx_arm_coordination`.

Update `graph_model.ROBOT_UNITS` so same-side arm and hand actions conflict by shared bus ownership.

Recommended resource model:

- `left_arm` -> `{left_arm, left_can_bus}`
- `right_arm` -> `{right_arm, right_can_bus}`
- `both_arms` -> `{left_arm, right_arm, left_can_bus, right_can_bus}`
- `left_hand` -> `{left_hand, left_can_bus}`
- `right_hand` -> `{right_hand, right_can_bus}`

Then update the comments and any planning docs/catalogue references that still say CAN-bus tokens are
deferred.

This does two things:

- same-side arm and hand actions cannot be scheduled concurrently by accident
- the existing coordinator becomes the natural owner of Step-and-Settle sequencing for demo activities

### Phase 5 - Keep MoveIt as the arm executor, not the bus arbiter

Owner: `src/agx_arm_moveit`, `src/agx_arm_coordination`.

MoveIt should remain the trajectory executor only. It should not decide when a hand window is safe.

Practical implication:

- MoveIt still owns `FollowJointTrajectory` / `MoveGroup` execution through MIT
- the coordinator or a side-handoff manager owns the transition into and out of hand windows
- the existing `agx_arm_duo_soft_estop` can stay as the central "soft hold both arms" helper, but it
  should delegate to the new side-safe recovery or handoff primitives rather than stopping at
  `cancel_trajectory` + `hold_current`

## 5. Concrete code changes to prioritize

Highest-value small changes first:

1. In `mit_controller_node.py`, fix the stale-feedback abort path (section 1.4): clear
   `active_trajectory`, capture a fresh hold reference, and enter `CANCELING_TO_HOLD` like the other
   abort paths do. This is small, safety-relevant, and independent of all instrumentation — do it
   before Phase 0.
2. Make silent TX loss observable: vendor-fork counters plus a verified (effect-checked) emergency
   stop (sections 1.3.2, Phase 1 item 2, Phase 2 item 4).
3. In `agx_arm_ctrl_single_node.py`, rebuild the stale watchdog on kernel RX timestamps with
   loop-overrun awareness, covering both the `feedback_timeout` and the `not is_ok()` branches
   (Phase 1 items 3-4).
4. Add one driver-level hold or quiesce service that explicitly returns the arm to normal-mode static
   hold before a hand window, with mode readback verification.
5. Add a repo-owned recovery helper script for disconnect-while-moving, including the hand-side stop
   before the link reset.
6. Add `left_can_bus` and `right_can_bus` resource tokens in the coordination layer (verified
   trivial in `graph_model.ROBOT_UNITS`; can be done in parallel at any time).

## 6. Validation plan

### 6.1 Offline and unit-level

- add a unit test for `agx_arm_ctrl` recovery classification that proves stale-feedback recovery is not
  entered after a simulated publish-loop pause without comm errors
- add a unit test proving stale recovery IS entered when the mocked kernel RX timestamp stops
  advancing, and NOT entered when it keeps advancing during a simulated node stall (section 1.3.3)
- add a test against a mocked comm layer proving silent send drops (swallowed ENOBUFS) are surfaced
  by the forked SDK counters and by the effect-verified emergency stop
- add a MIT-controller test that a stale-feedback abort clears or deactivates the active trajectory
  and that feedback returning after a stale pause never samples the old trajectory clock
- add coordination tests proving `right_arm` conflicts with `right_hand` once side-bus tokens are added
- add a dry-run test for the recovery helper ordering, including the hand-stop step

### 6.2 Hardware validation

On Jetson or equivalent hardware only:

1. CPU stress test with a healthy bus: verify no false CAN recovery is triggered by local load alone.
2. Shared-bus hand window test:
   - arm enters static hold
   - MIT streaming quiesces
   - hand command completes
   - arm reacquires control without reconnecting
3. Disconnect-under-MIT test:
   - active arm motion
   - force bus loss or missing ACK scenario
   - recovery helper stops motion, cycles link, returns stack to normal-mode hold
   - no automatic trajectory resume without explicit re-enable
4. Silent-TX-loss test: saturate the TX path until ENOBUFS while feedback keeps flowing, confirm the
   forked SDK counters surface the drops, and confirm the verified emergency stop escalates instead
   of logging success.
5. Hand-across-link-reset test: verify whether the OmniHand backend survives `ip link down/up` or
   needs an explicit reconnect, and encode the answer in the recovery helper.
6. MoveIt resume test after hand window and after disconnect recovery.

## 7. Summary

The repo already knows that the safe operational rule is "arm owns the side bus, and the hand only gets
explicit command windows after the arm has settled". The remaining work is to turn that rule into code:

- the immediate MIT stale-abort trajectory fix (section 1.4)
- observable TX loss and an effect-verified emergency stop (section 1.3.2)
- conservative, kernel-timestamp-based stale detection in `agx_arm_ctrl`
- disconnect-safe stop and recovery helpers that include the hand side
- a side-local arm-to-hand handoff API with readback-verified mode changes
- side-bus resource tokens in the coordinator

That combination is the missing Step-and-Settle implementation. The SDK audit (section 1.3) showed
the pinned pyAgxArm already provides the needed feedback signal (kernel RX timestamps) and all
driver primitives for the handoff; the only SDK-side change worth making is the small vendor-fork
patch that makes TX errors observable.