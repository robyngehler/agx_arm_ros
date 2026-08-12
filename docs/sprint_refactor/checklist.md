# Sprint Refactor - Checklist

Phases follow `planning/integration_plan.md`, which is the canonical plan.
Binding constraints C1 (one CAN bus per device), C2 (MIT rate is a requirement),
C3 (pinned submodule vs development checkout), C4 (test ladder), C5 (message
policy), C6 (instrumentation form), C7 (bus topology is one declared fact), and
C8 (the two arms speak different protocol tiers, permanently) are defined there.

Priority: safety, CPU relief, and parallel operation come before demo work.
`docs/sprint6/` adapts afterwards.

## Sprint setup

- [x] Create the `docs/sprint_refactor/` surface on branch
      `ROS2_Duo_System_V02`.
- [x] Move the coordination refactor proposal into the sprint surface.
- [x] Cross-check the proposal against the current code and record the entry
      points that will drive the migration.
- [x] Re-verify the cross-check against the working tree (2026-08-11) and fold
      the four-bus topology into the plan.
- [x] Freeze the open decisions: wrappers as degraded mode, hardware slot for
      the safety checks, instrumentation form, refactor-before-demo priority.
- [x] Freeze the remaining contract decisions: no separate hand lease, epoch in
      `MoveMITMsg`, in-phase message migration, MoveIt hand FJT debug-only,
      degraded-mode removal reviewed at Phase 5 close-out.

## Phase 0 - Hygiene, harness, safety baseline

### 0A Guidance and topology hygiene

- [x] Repoint the sprint entrypoints from `docs/sprint6/` to
      `docs/sprint_refactor/`: `CLAUDE.md`, `.claude/rules/context-routing.md`,
      `.github/copilot-instructions.md`.
- [x] Correct the ROS contract rules that now point the wrong way: `AGENTS.md`
      shared `control/joint_states` and the "do not map onto Revo2 messages"
      rule, plus the `CLAUDE.md` mirror.
- [x] Update `.claude/rules/omnihand-bridge.md` for the per-device bus, hand
      ownership, and the C5 consolidation.
- [x] Banner the operational docs that describe the shared bus and hand window
      as normal operation: `docs/control/bringups/teach_and_run.md`,
      `docs/control/bringups/tea_demo.md`,
      `docs/assets/omnihand/omnihand_solo_bringup_and_load_test.md`.
- [x] Mark the `docs/sprint6/` step-and-settle planning notes superseded and
      record that sprint6 adapts after the refactor.
- [x] Update `scripts/activate_native_can.sh` header, which still documents
      can0/can1 as shared side buses.
- [x] Correct all `pyAgxArm` references to `vendor/pyAgxArm` and document the C3
      workflow.
- [x] Record the MIT control-rate requirement (>= 100 Hz, target 200-250 Hz).
- [ ] Remove the stray untracked `vendor/Omnihand-2025-SDK/` checkout (legacy
      non-Pro `o10` SDK, no longer used; the runtime targets
      `vendor/OmniHand-Pro-2025`).

### 0B Regression harness and test ladder

- [x] Add an L2 mock integration test covering coordinator -> arm driver -> hand
      bridge for one activity including a hand action.
- [x] Encode the C4 test ladder as a `.claude/skills/` workflow with a
      `.github/skills/` mirror, following the `commit-quality` skill's shape.
- [~] Define the `tea_pour_left_v1` regression criteria — **deferred by
      decision**; the L2 harness is the standing regression net until the demo
      is re-taught against the new contracts.

### 0C Honest velocity and stop semantics

- [x] Provide a trustworthy velocity source (derived from timestamped positions)
      with an explicit validity flag.
- [x] Verify it against the protocol value: settled from the MIT pcaps — the
      firmware reports 0 (+/-1) while the joints move, so there is no protocol
      value to compare against and the derived source is the only one.
- [x] Separate `commanded` from `feedback_verified` in stop reporting.
- [x] Define and implement the coordinator response to a `commanded`-only stop,
      including the Ctrl+C stop ladder from commit `8e8fc44`.
- [ ] Audit the undocumented `current *= -1` vendor mutation in `driver.py:541`
      (origin traced to `cea1cb9`; sign still unconfirmed against a known load).
- [x] Extend `test_emergency_stop_verify.py` for the new outcomes.

### 0D Baseline instrumentation

- [x] Add in-node log counters for loop duration, callback duration, SDK call
      origin with thread id (`agx_arm_ctrl/runtime_metrics.py`, off by default).
- [x] Record the external tooling recipe as a runnable script:
      `scripts/measure_can_baseline.sh` reports per-interface rates and drops
      over a window plus process CPU, and sends nothing on any bus.

### 0E Hardware baseline (L3, safety slot)

Captured in `reference/phase0_baseline.md`, nine scenarios across three grants:
communication-only, then hand gestures plus one minimal arm move, then fault
injection.

- [x] Capture idle with no ROS nodes running: ~5430 frames/s of drain before any
      of our code runs.
- [x] Capture one arm driver, no motion: 71.6 % of one core, 198 Hz loop,
      1587 SDK calls/s, publish batch 1.10 ms mean / 2.73 ms max.
- [x] Capture two MIT arms under load (pcaps): 2849 f/s per bus, MIT at
      100 Hz per joint, feedback rate unchanged from idle.
- [x] Capture dual-arm hold, one MIT arm, one hand action.
- [x] Capture same-side arm-and-hand in parallel: both completed, no drops,
      arm buses unaffected — the case C1 exists to allow.
- [ ] Capture both sides arm-and-hand in parallel (blocked: `hand_left`
      cable fault makes its half meaningless until replaced).
- [x] Capture bus-fault and recovery: detection took ~2 s after a 1.6 s
      starvation misdiagnosis, the loop stalled 10 s in one gap, and the
      "recovery succeeded" claim was the bus returning on its own.
- [x] Record per-interface CAN counters and RX drop counts for every captured
      scenario (`scripts/measure_can_baseline.sh`).
- [x] Settle the wire-level velocity question (`evidence/*.pcap`,
      `scripts/analyze_can_pcap.py`): the firmware does not report velocity.

## Phase 1 - Device authority, serialized SDK access, unit guard

### 1A Device authority, epoch, serialized SDK

The rules and the mechanism are separated deliberately: the model is built and
proven at L1 first, then routed through the runtime. A checked box in the first
group means the behaviour is decided and tested, not that the driver uses it
yet.

Rules and mechanism (L1, `agx_arm_ctrl/device_authority.py`, `sdk_worker.py`):

- [x] Per-device `device_epoch` plus a unit-wide `unit_safety_epoch`, with an
      L1 test that an arm recovery does not invalidate the same-side hand and
      that a unit stop does.
- [x] Admission at the boundary: state, owner, both epochs, and a per-epoch
      sequence watermark, each with a structured reject reason.
- [x] Single-commander ownership: claim, release, and safety revoke, each
      bumping the device epoch.
- [x] Separate fault acknowledge from verified rearm — acknowledging clears the
      latch and arms nothing, and `rearm` refuses without positive evidence.
- [x] Serialized SDK worker: one named thread per device, safety lane ahead of
      queued motion, stale-epoch work dropped instead of delivered late,
      superseded setpoints replaced, and a call that never ran distinguishable
      from one that failed.

Routed through the runtime:

- [x] Validated on hardware, both arms (`reference/phase1a_hardware_validation.md`):
      enable readback, per-tier MIT bounds, the four rejection paths, a full
      MIT hold with zero rejections, and the authority through e-stop,
      lockout-clear and a disable/enable cycle.
- [x] Each arm driver builds its own authority and publishes it latched on
      `feedback/authority` (`AgxDeviceAuthority`), derived from the gates the
      driver already acts on — enable readback, feedback readiness, fault
      lockout, recovery, hand window — with the epochs coming from the
      authority's own transitions rather than from those gates.
- [ ] The two hand transport authorities. Only the arms publish so far.
- [ ] Route all arm SDK calls for one device through the worker.
- [ ] Reject stale epoch and out-of-order sequence on the live command path.
- [ ] Synchronise `unit_safety_epoch` across processes. Each node currently
      keeps its own, so a unit stop raised in one node is not yet seen by the
      others; the coordinator calls every side's stop service, which covers the
      current e-stop path but is not the same guarantee.
- [ ] Extend `MoveMITMsg` with the epochs and a sequence; add `srv/` to
      `src/agx_arm_msgs`.
- [ ] Make the MIT controller consume the authoritative device state instead of
      `feedback/hand_window_active`, and abort on authority loss.
- [x] Make CAN recovery report what it verified — 0E showed "recovery
      succeeded" for a bus that returned on its own. The re-arm result was
      being discarded; the log line now names feedback and the enable readback
      separately, and a restored bus with an unconfirmed enable is an error.
- [x] Hardware-boundary command validation for MIT: duplicate or unknown joint
      indexes, empty commands, non-finite values, and values the protocol
      cannot encode are refused whole, before the SDK sees them. Rejections are
      counted per reason and logged rate-limited, because a malformed stream
      arrives at the control rate.
- [x] Bound the MIT values per **firmware tier**, not per arm model. The first
      version applied the default tier's per-joint torque table to both arms,
      which against the 1.11 arm would have refused legitimate commands on
      joints 5-7 and admitted impossible ones on joints 1-2 (L3, 2026-08-12).
- [ ] Promote the joint-limit check from flagged to refused. A position past a
      joint's *configured* limit is currently warned and still forwarded:
      refusing mid-stream would freeze a running impedance loop at its last
      setpoint, and no hardware session has yet shown the controller never
      legitimately crosses a limit.
- [ ] Replace the unassigned `AgxArmStatus.err_status` with a documented
      structured error representation.
- [x] Fix the enable readback: a contradicted enable used to warn and return
      success, leaving `enable_flag` stale. The readback now decides both the
      flag and the return value, with a short settle window for a lagging frame.
- [x] Fix the firmware-version parsing: there was no `NeroFW.V112` branch at
      all, so a 1.12 arm ran on the 1.11 protocol, and versions were compared
      as strings. `resolve_nero_firmware` parses numerically and logs the tier,
      which nothing recorded before.
- [ ] Make forced e-stop recovery independent of the optional normal-recovery
      setting.
- [ ] Disable or quarantine direct legacy arm motion ingress for coordinated
      hardware profiles.
- [ ] Stress-validate MIT streaming plus e-stop, recovery, and enable/disable
      churn.
- [ ] Confirm one SDK thread per arm with the counter under a **full** stack.

Exercised on hardware 2026-08-12. The enable readback confirmed on the first
attempt on both arms, so the stricter check introduces no spurious failures. The
protocol tier is now recorded, and it is **not the same on both arms**: right
1.06 (default tier), left 1.11 (`NeroFW.V111`). See `errors_and_fixes.md` and
`open_questions.md` — the tiers differ in MIT frame encoding, so nothing may
assume the two arms are protocol-identical.

### 1B Feedback snapshot and driver CPU reduction

- [ ] Separate acquisition cadence from publication cadence; one immutable
      snapshot per cycle, no SDK getters in callbacks.
- [ ] Target the whole publish batch, not only the per-joint reads (measured at
      ~10 % of it).
- [ ] Ensure a stalled loop cannot stop draining the RX socket (0E: one 10 s gap).
- [ ] Measure the arm-driver CPU improvement against the 0E baseline (71.6 % of
      a core at rest, 1.10 ms mean batch against a 5 ms period).

### 1C One active unit activity — the small guard

Pulled ahead of Phase 2 on review: the rule must hold before parallelism exists.

- [x] `READY` accepts one activity; `EXECUTING` rejects every further goal with
      a structured reason. The goal callback refuses at the door; the claim
      inside execute is authoritative, because two goals can pass the door
      check at once on a reentrant callback group.
- [x] One authoritative unit activity state and failure reason
      (`agx_arm_coordination/unit_activity.py`), replacing a running-flag that
      nothing consulted before dispatching.
- [x] No polling-loop or event-queue work here — that stays in Phase 3.
- [x] L2 regression: a second goal sent while an activity runs is rejected.
      Two client processes cannot show this — the mock activity finishes in
      under a second, less than process startup jitter — so the probe sends the
      second goal from the same process the moment the first is accepted.

Exit gate met at L2 with mock backends and an arm double. Nothing here touches
hardware.

## Phase 2 - Parallel operation

### 2A Declare the topology, model the four buses

- [ ] Add `bus_topology` to the registry as the single declared fact (C7) and
      `omnihand.sides.<side>.can_port`; bump the schema version.
- [ ] Remove the bridge's derivation of its interface from the arm `can_port`;
      fail closed for hardware profiles instead of falling back to
      `can_nero_right`.
- [ ] Replace process-global `OMNIHAND_SOCKETCAN_IFACE` selection with explicit
      backend construction.
- [ ] Adopt `scripts/activate_duo_can.sh` as the supported bring-up and retire
      `activate_native_can.sh` / `omnihand_canfd_activate.sh`.
- [ ] Rewrite the operational docs bannered in 0A for the four-bus reality.

### 2B Parallel resource model, handoff derived not configured

- [ ] Derive the scheduler's bus tokens from `bus_topology`; under
      `dedicated_per_device`, `<side>_arm` and `<side>_hand` stop sharing one.
- [ ] Derive `handoff_enabled` from the same value; nothing reads it directly.
- [ ] Remove the MIT stand-down on `feedback/hand_window_active`.
- [ ] Keep `prepare_hand_window` / `resume_arm_control` only as the
      `shared_per_side` implementation.
- [ ] Fix the stale TX-loss warning that blames "hand-frame arbitration loss on
      the shared bus".
- [ ] Add tests for the newly reachable interleavings.

### 2C Hand arbitration and transport efficiency

Moved ahead of the coordinator rewrite: 115 % of a core for 27 frames/s makes
this the largest measured CPU consumer in the system.

- [ ] Implement the frozen contract: single-goal arbitration plus `owner_id`,
      `device_epoch` and `sequence`; no separate lease.
- [ ] Reject stale-epoch and out-of-order hand commands at the bridge boundary.
- [ ] Profile the O12 Pro backend at SDK-call level: which calls, how many per
      setpoint, how long each blocks.
- [ ] Eliminate the read-before-write round trip in the full-joint command path.
- [ ] Decouple command verification, joint readback, tactile and status into
      separate schedules.
- [ ] Stop polling entirely while no hand action is active.
- [ ] Remove the recurring post-grasp hold traffic.
- [ ] Bound and record the SDK round trips per commanded setpoint.
- [ ] Measure the hand-bridge CPU reduction against the 115 % baseline.
- [ ] Validate parallel same-side arm and hand motion without CAN RX drops.
- [ ] Verify the `shared_per_side` topology still executes an activity.

## Phase 3 - Event-driven coordinator and strict synchronization

The exclusivity guard landed in 1C; this is the conversion itself.

- [ ] Replace polling and `time.sleep` with event-driven completion handling and
      a low-rate deadline watchdog.
- [ ] Make SIGINT with no activity in flight exit rather than spin (0B finding).
- [ ] Extend the 1C guard to the full unit activity state machine, with cleanup
      as part of completion.
- [ ] Migrate the Ctrl+C stop ladder and replay planning onto the event model
      without weakening them.
- [ ] Make `sync_flag` merge strict: merge-or-fail, never independent fallback.
- [ ] Add cleanup deadlines and structured failure reasons for child shutdown.
- [ ] Validate concurrent-goal rejection, cancellation, cleanup, and the
      parallel interleavings from 2C.

## Phase 4 - Registry, manifest, and contract consolidation

- [ ] Define the resolved manifest contract and schema/version bump.
- [ ] Reduce execution profiles to selection-only composition.
- [ ] Generate MoveIt controller config, joint-state merger inputs, and launch
      parameter dictionaries from the resolved manifest.
- [ ] Move coordinator resource claims to manifest-driven data.
- [ ] Keep the MoveIt hand FJT path non-default in coordinated production
      profiles.
- [ ] Remove or quarantine the legacy unprefixed `moveit_controllers.yaml`.
- [ ] Consolidate `HandCmd`, `HandPositionTimeCmd`, `HandStatus`,
      `GripperStatus`, and `OmniHandStatus` into one abstract hand contract per
      C5, with a caller migration note (4D).
- [ ] Carry owner identity, control epoch, and sequence in hand commands.
- [ ] Validate joint values at the bridge; remove SDK read-before-write; reject
      partial commands without a valid cache.
- [ ] Distinguish `commanded`, `delivery_verified`, and `contact_confirmed`
      completion.
- [ ] Add manifest-hash consistency checks across runtime nodes.
- [ ] Validate source/install path resolution and fail-closed behavior.
- [ ] Add a repository check for duplicated authoritative values.

## Phase 5 - Runtime consolidation and close-out measurements

- [ ] Make MIT action completion event-driven with one trajectory sampler.
- [ ] Decompose the MIT tick before optimising it (trajectory sample, feedback
      snapshot, gravity/RNEA, command construction, ROS publish, action
      feedback/tolerance, locking/executor). "Gravity dominates" is a hypothesis;
      the same assumption about the arm driver's SDK reads was measured wrong.
- [ ] Then reduce whatever the decomposition names, so the rate can rise toward
      200-250 Hz (C2).
- [ ] Split OmniHand bridge timers by command verification, tactile, and status
      semantics.
- [ ] Bound executor thread counts and keep each vendor SDK session in its own
      process.
- [ ] Remove duplicate hand-joint aggregation from arm driver output.
- [ ] Re-run CPU and CAN baselines, including the parallel scenarios, and compare
      them against 0E.

## Phase 6 - Unit contract skeleton

- [ ] Freeze the unit-level public contract skeleton for later multi-unit work.

## Every phase

- [ ] `tea_pour_left_v1` still runs after the phase closes.
- [ ] L1 and L2 pass before any hardware run; L3 evidence recorded or its absence
      stated explicitly.

## Documentation follow-through

- [ ] Promote only stable runtime-contract changes into `docs/assets/`.
- [ ] Promote only stable operational changes into `docs/control/`.
- [ ] Update `docs/project/` if package boundaries, ownership, or generated
      artifact policy change.
