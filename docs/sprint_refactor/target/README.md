# Sprint Refactor Target
status: ACTIVE_REFACTOR_ENTRYPOINT
last_updated: 2026-08-12
branch: ROS2_Duo_System_V02

Sprint Refactor is the V02 migration surface for the coordination, safety, and
runtime-consolidation work proposed for the current Duo Nero system.

## Main goal

Cross-check the coordination architecture proposal against the current codebase
and turn it into a phase-ordered integration plan that can be implemented
without losing control of the present Duo baseline.

`planning/integration_plan.md` is the canonical plan. The proposal is the
architectural input; where they disagree, the plan wins.

## Scope

1. Validate the proposal findings against the current code in `agx_arm_ctrl`,
   `agx_arm_mit_controller`, `agx_arm_coordination`, `agx_arm_moveit`,
   `agx_arm_msgs`, `agx_arm_description`, and `vendor/pyAgxArm`.
2. Freeze the migration order, ownership boundaries, and validation gates for
   branch `ROS2_Duo_System_V02`.
3. Keep the work scoped to one Duo unit on one Jetson: no `ros2_control`
   rewrite, no full C++ rewrite, no multi-unit consensus in this sprint surface.
4. Keep hardware-touching validation explicit and separate from editor-only or
   x86 read-only checks.

## Binding constraints

Defined in `planning/integration_plan.md`:

- **C1** one CAN bus per device (arms `can0`/`can1` native, hands `can2`/`can3`
  on USB-CAN FD adapters); same-side arm and hand motion may run in parallel and
  step-and-settle is demoted to a degraded-topology fallback
- **C2** MIT control rate is a requirement (>= 100 Hz, target 200-250 Hz), so CPU
  savings must come from per-tick cost
- **C3** the pinned `vendor/pyAgxArm` submodule is the execution path; vendor
  development happens in a separate checkout and lands as an explicit pin bump
- **C4** test ladder L1 unit -> L2 mock -> L3 hardware, with L1 and L2 required
  before any hardware run
- **C5** native ROS interfaces first, statically defined fields, hand messages
  consolidated into one abstract contract
- **C6** instrumentation as in-node log counters plus external tooling; no new
  public ROS metrics contract in Phase 0
- **C7** bus topology is one declared fact (`bus_topology`); the scheduler's
  claims and the handoff both derive from it rather than being set separately
- **C8** the two arms run different firmware (right 1.06, left 1.11) and cannot
  be flashed; mixed protocol tiers are the baseline, so anything derived from
  the protocol is per tier and any assumption of symmetry is a defect

## Priority

The refactor takes priority over demo work on three axes: safety, CPU relief,
and parallel operation. `docs/sprint6/` adapts to the resulting contracts
afterwards rather than competing for the same hardware and the same files.

## Current status

- The proposal is stored inside this sprint surface.
- The read-only code cross-check is complete, re-verified on 2026-08-11, and all
  findings still hold.
- The plan has been updated for the four-bus hardware topology, the MIT rate
  requirement, the vendor-submodule workflow, the test ladder, and the message
  policy.
- **Phase 0 is complete** for the authorised scenarios: guidance hygiene, the
  L2 harness, honest velocity and stop semantics, in-node instrumentation, and a
  nine-scenario hardware baseline (`reference/phase0_baseline.md`).
- **Phase 1A is under way and validated on hardware** for what has landed:
  boundary validation, per-tier MIT bounds, the published device authority, and
  the stop and enable paths (`reference/phase1a_hardware_validation.md`). Still
  open in 1A: MIT consuming the authority, the command stamp, and routing SDK
  calls through the serialized worker.
- The hardware session also established C8 — the two arms are on different,
  unflashable firmware — and found six defects, all logged in
  `errors_and_fixes.md`.
- `tea_pour_left_v1` is the end-to-end regression benchmark for every phase.

## Deliverables

- retained proposal and code cross-check evidence
- phase-ordered integration plan with sub-phases and validation gates
- sprint-local checklist, errors, and open questions
- a clean handoff path into stable docs once contracts land

## Working set

- `coordination_architecture_refactor_proposal.md`
- `planning/integration_plan.md`
- `reference/proposal_code_crosscheck.md`
- `checklist.md`
- `errors_and_fixes.md`
- `open_questions.md`

## Documentation boundary

This sprint surface is the working area for the V02 refactor. It does not
override the stable operational docs under `docs/control/`, the stable package
rules under `docs/project/`, or the long-lived runtime facts under
`docs/assets/`.