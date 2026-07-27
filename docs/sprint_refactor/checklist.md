# Sprint Refactor - Checklist

## Sprint setup

- [x] Create the `docs/sprint_refactor/` surface on branch
      `ROS2_Duo_System_V02`.
- [x] Move the coordination refactor proposal into the sprint surface.
- [x] Cross-check the proposal against the current code and record the entry
      points that will drive the migration.

## Phase 0 - Safety baseline and instrumentation

- [ ] Replace the synthetic zero velocity path in `pyAgxArm`, or add a
      trustworthy derived-velocity fallback with timestamps and explicit limits.
- [ ] Change soft e-stop result wording and state handling so "commanded" and
      "velocity-verified" are separate outcomes until velocity is trustworthy.
- [ ] Add narrow validation for velocity reporting and stop verification.
- [ ] Add CPU, loop-duration, SDK-call, and CAN-traffic instrumentation needed
      for the pre-refactor baseline. Known hot paths to instrument first are
      recorded in `reference/critical_cpu_paths.md` (driver 200 Hz per-joint SDK
      reads, RX-drain coupling, MIT gravity comp, hand CANFD polling).
- [ ] Capture the baseline scenarios on hardware: idle, dual-arm hold, one MIT
      arm, two MIT arms, one hand window, and recovery/fault cases.

## Phase 1 - Side authority and serialized SDK access

- [ ] Add an authoritative per-side state contract with a control epoch.
- [ ] Route all arm SDK calls for one side through one serialized worker or
      command queue.
- [ ] Reject stale-epoch commands at the hardware boundary.
- [ ] Separate fault acknowledge from verified rearm.
- [ ] Disable or quarantine direct legacy arm motion ingress for coordinated
      hardware profiles.
- [ ] Stress-validate MIT streaming plus handover, e-stop, recovery, and
      enable/disable churn.

## Phase 2 - Leased hand control

- [ ] Add repo-owned lease action/service/message contracts in
      `src/agx_arm_msgs`.
- [ ] Replace Trigger-only handover semantics with lease acquire/release logic
      in `src/agx_arm_ctrl`.
- [ ] Require side state, control epoch, and lease identity in the OmniHand
      skill and bridge paths.
- [ ] Remove recurring host-side hold traffic after a grasp completes.
- [ ] Stop bridge polling and retries outside a valid hand lease.
- [ ] Fail the coordinator activity if lease release or arm resume is not
      verified.
- [ ] Validate zero hand TX and read-request traffic during same-side arm
      ownership.

## Phase 3 - Coordinator exclusivity and event-driven execution

- [ ] Enforce one active unit activity in `coordinator_node.py`.
- [ ] Replace 20 Hz child polling with event-driven completion handling and a
      low-rate deadline watchdog.
- [ ] Make `sync_flag` merge strict: merge-or-fail, never independent fallback.
- [ ] Move coordinator resource claims to manifest-driven data.
- [ ] Add cleanup deadlines and structured failure reasons for child shutdown.
- [ ] Validate concurrent-goal rejection, cancellation, and cleanup behavior.

## Phase 4 - Registry, profiles, and generated runtime artifacts

- [ ] Define the resolved manifest contract and schema/version bump.
- [ ] Reduce execution profiles to selection-only composition.
- [ ] Generate MoveIt controller config, joint-state merger inputs, and launch
      parameter dictionaries from the resolved manifest.
- [ ] Remove or quarantine the legacy unprefixed `moveit_controllers.yaml` from
      Duo launch paths.
- [ ] Add manifest-hash consistency checks across runtime nodes.
- [ ] Validate source/install path resolution and fail-closed behavior.

## Phase 5 - Runtime consolidation and close-out measurements

- [ ] Replace unconditional high-rate arm feedback work with state-aware
      acquisition and publication budgets.
- [ ] Make MIT action completion event-driven with one trajectory sampler.
- [ ] Split OmniHand bridge timers by command verification, tactile, and status
      semantics.
- [ ] Remove duplicate hand-joint aggregation from arm driver output.
- [ ] Re-run CPU and CAN baselines and compare them against Phase 0.
- [ ] Freeze the unit-level public contract skeleton for later multi-unit work.

## Documentation follow-through

- [ ] Promote only stable runtime-contract changes into `docs/assets/`.
- [ ] Promote only stable operational changes into `docs/control/`.
- [ ] Update `docs/project/` if package boundaries, ownership, or generated
      artifact policy change.