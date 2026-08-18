# Sprint Refactor Target

status: RC_CLOSED — follow-up phases active
last_updated: 2026-08-18
branch: ROS2_Duo_System_V02_refactor

Sprint Refactor is the V02 migration surface for the coordination, safety, and
runtime-consolidation work on the Duo Nero system.

## Main goal

Give the unit a deterministic control-integrity layer — one owner per device,
generations that invalidate stale commands, one serialized SDK owner per device
— and make same-side arm and hand motion run in parallel, without losing control
of the running Duo baseline while it happens.

## The three files that matter

| File | Role |
| --- | --- |
| [`planning/integration_plan.md`](../planning/integration_plan.md) | **Canonical.** Phases, order, binding constraints C1–C8 |
| [`planning/decision_record.md`](../planning/decision_record.md) | **Why.** Every decision, what forced it, what it replaced, what it left open |
| [`../checklist.md`](../checklist.md) | **How far.** Per-item state and the measured evidence behind each claim |

`reference/` holds the measured evidence; `errors_and_fixes.md` the defects and
what closed them; `open_questions.md` what is still undecided.

## Scope

1. One Duo unit on one Jetson. No `ros2_control` rewrite, no full C++ rewrite,
   no multi-unit consensus in this sprint surface.
2. Keep package boundaries intact unless a new repo-owned ROS contract requires
   a new interface in `src/agx_arm_msgs`.
3. Keep hardware-touching validation explicit and separate from editor-only or
   x86 read-only checks (constraint C4).

## Binding constraints

Defined in `planning/integration_plan.md`, with their rationale in
`planning/decision_record.md` Part I:

- **C1** one CAN bus per device (arms `can_nero_left`/`can_nero_right` native,
  hands `hand_left`/`hand_right` on USB-CAN FD adapters); same-side arm and hand
  motion may run in parallel and step-and-settle is a degraded-topology fallback
- **C2** MIT control rate is a requirement (>= 100 Hz, target 200-250 Hz), so CPU
  savings must come from per-tick cost
- **C3** the pinned `vendor/pyAgxArm` submodule is the execution path; vendor
  development happens in a separate checkout and lands as an explicit pin bump
- **C4** test ladder L1 unit -> L2 mock -> L3 hardware, with L1 and L2 required
  before any hardware run
- **C5** native ROS interfaces first, statically defined fields, hand interfaces
  abstract enough for any hand. The command half is settled; the status half is
  open
- **C6** instrumentation as in-node log counters plus external tooling
- **C7** bus topology is one declared fact (`bus_topology`); the scheduler's
  claims and the handoff both derive from it rather than being set separately
- **C8** the two arms run different firmware (right 1.06, left 1.11) and cannot
  be flashed; anything derived from the protocol is per tier, and any assumption
  of symmetry is a defect

## Current status

- **Phase 0 complete**: guidance hygiene, the L2 harness, honest velocity and
  stop semantics, in-node instrumentation, and a nine-scenario hardware baseline
  (`reference/phase0_baseline.md`).
- **Phase 1 complete**: four device authorities, two epoch levels, the frozen
  command stamp on the wire, live admission at the hardware boundary, the
  serialized `SdkWorker` with four priority lanes, the single unit-safety writer
  with incarnation-ordered restart, and the one-activity guard. Validated on
  hardware (`reference/phase1a_hardware_validation.md`,
  `reference/sdk_latency_budget.md`).
- **Phase 2 substantially complete**: four-bus topology declared in the
  registry, fail-closed hand interface resolution, parallel resource model
  derived from `bus_topology`, hand single-commander arbitration, and the hand's
  own `SdkWorker`. Parallel same-side operation proven on hardware on both
  sides and both sides at once. Residual transport-efficiency items are listed
  in the checklist under 2C.
- **Phase 3 partially complete**: event-driven child completion, atomic sync
  groups, merge-or-fail synchronization, bounded cleanup with structured
  reasons. The full unit-activity state machine and the Ctrl+C stop-ladder
  migration remain.
- **Phases 4-6 open**: manifest and profile consolidation, the hand *status*
  contract, MIT tick decomposition, executor/process policy, close-out
  measurements, and the unit contract skeleton.
- **Refactor Runtime RC closed 2026-08-17**, every gate item proven on hardware.
  Sprint-6 and coordinated demo work resume against these contracts.

## Documentation boundary

This sprint surface is the working area for the V02 refactor. It does not
override the stable operational docs under `docs/control/`, the stable package
rules under `docs/project/`, or the long-lived runtime facts under
`docs/assets/`. Contracts that have settled are promoted out of here:

- the authority / SDK-ownership / recovery / watchdog architecture is
  `docs/project/control_integrity_architecture.md`
- the hand bridge contract is `.claude/rules/omnihand-bridge.md` and its
  `.github/` mirror
- the vendor hand SDK's capabilities and the deferred control work are
  `docs/assets/omnihand/omnihand_pro_analysis.md`
