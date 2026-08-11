# Sprint Refactor - Checklist

Phases follow `planning/integration_plan.md`, which is the canonical plan.
Binding constraints C1 (one CAN bus per device), C2 (MIT rate is a requirement),
C3 (pinned submodule vs development checkout), C4 (test ladder), C5 (message
policy), and C6 (instrumentation form) are defined there.

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
- [ ] Define the `tea_pour_left_v1` regression criteria enforced after every
      phase.

### 0C Honest velocity and stop semantics

- [x] Provide a trustworthy velocity source (derived from timestamped positions)
      with an explicit validity flag.
- [ ] Verify it against the protocol value using the C3 development checkout
      (L3: needs a patched vendor driver on hardware — every tier zeroes it,
      so there is no software-only comparison).
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

- [ ] Capture idle, dual-arm hold, one MIT arm, two MIT arms, one hand action.
- [ ] Capture same-side arm-and-hand in parallel (new scenario under C1).
- [ ] Capture both sides arm-and-hand in parallel.
- [ ] Capture bus-fault and recovery cases.
- [ ] Record per-interface CAN counters and RX drop counts for every scenario.

## Phase 1 - Side authority, serialized SDK access, arm feedback budget

- [ ] Add an authoritative per-side state contract with a control epoch.
- [ ] Route all arm SDK calls for one side through one serialized worker or
      command queue, with a priority lane for emergency stop.
- [ ] Reject stale-epoch commands at the hardware boundary.
- [ ] Make the MIT controller consume the authoritative side state instead of
      `feedback/hand_window_active`, and abort on authority loss.
- [ ] Separate fault acknowledge from verified rearm.
- [ ] Add full hardware-boundary command validation (duplicate or missing joint
      indexes, empty commands, non-finite values, out-of-range values).
- [ ] Replace the unassigned `AgxArmStatus.err_status` with a documented
      structured error representation.
- [ ] Fix enable readback and firmware-version parsing; make forced e-stop
      recovery independent of the optional normal-recovery setting.
- [ ] Disable or quarantine direct legacy arm motion ingress for coordinated
      hardware profiles.
- [ ] Add the `srv/` directory to `src/agx_arm_msgs` for the new contracts.
- [ ] Separate acquisition cadence from publication cadence; one immutable
      feedback snapshot per cycle, no SDK getters in callbacks (1E).
- [ ] Stress-validate MIT streaming plus e-stop, recovery, and enable/disable
      churn.
- [ ] Measure the arm-driver CPU improvement against the 0E baseline.

## Phase 2 - Parallel operation

### 2A Four-bus topology

- [ ] Add `omnihand.sides.<side>.can_port` to the registry and bump the schema
      version.
- [ ] Remove the bridge's derivation of its interface from the arm `can_port`;
      fail closed for hardware profiles instead of falling back to
      `can_nero_right`.
- [ ] Replace process-global `OMNIHAND_SOCKETCAN_IFACE` selection with explicit
      backend construction.
- [ ] Update CAN bring-up for four interfaces with deterministic adapter naming.
- [ ] Rewrite the operational docs bannered in 0A for the four-bus reality.

### 2B Close the hand-window path

- [ ] Make parallel the default mode and step-and-settle a selectable degraded
      mode, using the existing `handoff_enabled` coordinator parameter as the
      switch.
- [ ] Remove the MIT stand-down on `feedback/hand_window_active` from the
      parallel path.
- [ ] Keep `prepare_hand_window` / `resume_arm_control` only as the degraded-mode
      implementation, documented as such, with no new work invested.

### 2C Parallel resource model

- [ ] Split `ROBOT_UNITS` so `<side>_arm` and `<side>_hand` no longer share one
      bus token.
- [ ] Allow concurrent same-side arm and hand scheduling.
- [ ] Re-couple them through configuration in degraded mode, not through code.
- [ ] Add tests for the newly reachable interleavings.

### 2D Hand commander arbitration

- [ ] Enforce one commander per hand device.
- [ ] Decide explicit ownership contract versus single-goal action semantics
      plus owner id and epoch.
- [ ] Stop bridge polling and retries outside active hand ownership.
- [ ] Remove recurring host-side hold traffic after a grasp completes.
- [ ] Validate parallel same-side arm and hand motion on hardware without CAN RX
      drops.
- [ ] Verify the degraded mode still executes an activity when selected.

## Phase 3 - Coordinator exclusivity and event-driven execution

- [ ] Enforce one active unit activity in `coordinator_node.py`.
- [ ] Replace polling and `time.sleep` with event-driven completion handling and
      a low-rate deadline watchdog.
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
- [ ] Reduce per-tick gravity-compensation cost so the control rate can rise
      toward 200-250 Hz (C2). May be pulled forward if 0E shows it dominating.
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
