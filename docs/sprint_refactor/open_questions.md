# Sprint Refactor - Open Questions

## Resolved during the initial code cross-check (2026-07-27)

- Active Duo MoveIt controller config is generated in
  `src/agx_arm_moveit/launch/_moveit_config_builder.py` from the selected
  `arm_instances`; it already creates namespaced arm controllers and, when an
  OmniHand is present, namespaced hand FJT controllers.
- The only standalone unprefixed `moveit_controllers.yaml` currently in the
  workspace is
  `src/agx_arm_sim/Moveit2/nero_gripper_moveit_config/config/moveit_controllers.yaml`.
  It belongs to the legacy standalone MoveIt package and should be treated as a
  legacy surface, not Duo runtime truth.
- `execution_profiles.yaml` already defers some frame and prefix values to
  `duo_motion_registry.yaml`, so the refactor should extend the existing
  registry-resolution path instead of replacing it wholesale.

## Resolved 2026-08-11

- **Shared side bus.** Superseded by hardware: each device has its own CAN
  interface (arms native `can0`/`can1`, hands on FD-capable USB adapters
  `can2`/`can3`). Same-side arm and hand motion may run in parallel.
  Step-and-settle is demoted to a degraded-topology fallback. Recorded as
  constraint C1 in `planning/integration_plan.md`.
- **Autonomous hand setpoint hold.** No longer gates the design; it was the
  deciding question only while arm and hand shared a bus. It moves to the Phase 0
  measurement list, where it decides how far recurring hold traffic can be
  removed in 2D.
- **Vendor SDK change workflow.** The pinned submodule
  (`vendor/pyAgxArm`, tag `control-layer-pin-2026-07-24`) stays unchanged as the
  execution path. Development happens in a separate checkout of the same fork,
  with the upstream vendor remote configured so upstream updates stay mergeable;
  relevant work is pushed, tagged, and then lands here as an explicit pin bump.
  Recorded as C3.
- **MIT control rate.** 100 Hz is the stability minimum, 200-250 Hz the target.
  The rate is a requirement, so CPU savings must come from per-tick cost.
  Recorded as C2.
- **Plan versus proposal precedence.** `planning/integration_plan.md` is
  canonical; relevant proposal items missing from it were folded in and are
  listed in its `Consolidated from the proposal` table.
- **Message strategy.** Native ROS interfaces are used where they already carry
  the meaning; repo-owned interfaces are added only for what ROS lacks, with
  statically defined fields, and the existing hand messages are consolidated
  into one abstract hand contract rather than extended. Recorded as C5.
- **Test escalation.** L1 unit -> L2 mock/integration -> L3 hardware end-to-end,
  with L1 and L2 required before any hardware run on every platform. Recorded as
  C4 and to be encoded as a `.claude/skills/` workflow.
- **Handover services.** `prepare_hand_window` and `resume_arm_control` are kept,
  but as the implementation of a selectable *degraded-topology mode*, not as
  migration scaffolding. Parallel operation is the new normal mode. The switch is
  the existing `handoff_enabled` coordinator parameter. Note that the flag alone
  does not produce parallel operation: `ROBOT_UNITS` must also stop sharing a bus
  token per side (plan 2B and 2C).
- **Hardware slot.** Reserved for the first safety checks: velocity truth, stop
  semantics, and the 0E baseline including the new parallel scenarios.
- **Instrumentation form.** In-node log counters plus external tooling for the
  MVP; no new public ROS metrics contract in Phase 0. Recorded as C6.
- **Sprint priority.** The refactor takes priority on safety, CPU relief, and
  parallel operation; the demo is not meaningful before those land, and
  `docs/sprint6/` adapts afterwards.
- **Old Phase 2 "leased hand control".** Struck. What survives is redistributed
  across plan 2A-2D and 4D; the struck items are listed in the plan so they do
  not get resurrected.
- **Hand SDK and hardware variants.** There are two hand devices: OmniHand
  (`o10`, 10 actuated joints) and OmniHand Pro (`o12_pro`, 12 actuated joints);
  they differ in actuated joint count and tactile feedback. The Pro is current
  and the runtime targets `vendor/OmniHand-Pro-2025`. `vendor/Omnihand-2025-SDK`
  is the legacy non-Pro SDK and is no longer used; the stray untracked checkout
  is cleanup, not a contract question. The consolidated hand contract must carry
  joint count, joint naming, and tactile layout as data (C5).

## Contract decisions frozen 2026-08-11

These are decided so implementation does not stall on them. Each carries the
condition that would reopen it; none is expected to reopen before the phase that
implements it.

- **No separate hand ownership contract.** Single-commander arbitration is
  achieved through the existing single-goal action semantics of the hand skill
  controller plus `owner_id`, `control_epoch`, and `sequence` fields in the
  consolidated hand command. Rationale: the lease existed to arbitrate a shared
  bus (C1 removed that); what remains is "one commander per device", which the
  action server already enforces, and the epoch plus sequence reject late
  messages across ownership transitions. Adds no new interface. *Reopens if* a
  second legitimate commander must coexist with the skill controller.
- **Extend `MoveMITMsg` with `control_epoch` and `sequence`;** do not introduce
  `ArmMitCommand`. Rationale: C5 creates only what is missing, and a parallel
  message would require migrating the hot streaming path twice. Legacy ingress
  is isolated by the profile gate in plan 1D, not by a second message type.
  Note that adding fields is an ABI change requiring a coordinated workspace
  rebuild, which is acceptable inside one workspace. *Reopens if* an
  out-of-workspace consumer of `MoveMITMsg` appears.
- **Consolidated hand messages are migrated and then removed inside Phase 4.**
  Every caller of `HandCmd`, `HandPositionTimeCmd`, `HandStatus`,
  `GripperStatus`, and `OmniHandStatus` is in-repo
  (`agx_arm_ctrl`, `agx_arm_mit_demos`); there are no external consumers, so no
  deprecation window is needed beyond the phase. The 1-DoF AGX gripper is the
  degenerate case that validates the abstraction rather than an exception to it.
- **The MoveIt hand FJT path is debug and development only**, non-default in
  coordinated production profiles. Rationale: two command sources for one device
  contradict single-commander arbitration.
- **The degraded step-and-settle mode has no removal date yet.** It is retained
  while a single-bus fallback remains physically possible, and its removal is
  reviewed at the Phase 5 close-out against the measured four-bus evidence. No
  new work is invested in it in the meantime.

## Measurements the Phase 0 baseline must answer

Not blockers: these are the questions 0E exists to answer, and the hardware slot
is reserved for them.

- Can per-interface SocketCAN frame counts, loop jitter, and per-thread CPU be
  captured without disturbing timing on the target Jetson?
- Do the two USB-CAN FD adapters enumerate deterministically, or is udev-based
  stable naming required before the four-bus topology can be trusted?
- How much CPU headroom remains for the 200-250 Hz MIT target once both arms and
  both hands can be active at the same time?
- What update rate and filtering does position-derived velocity need for e-stop
  verification without masking motion?
- Does the OmniHand Pro hold its setpoint without host traffic? No longer gates
  the design, but it decides how far the recurring hold traffic can be removed
  in 2D.

## Rollout, decided 2026-08-11

- `sprint_refactor` stays the dedicated migration surface until the V02 program
  closes; it is the current implementation entrypoint and does not fold into a
  numbered sprint mid-flight.
- The remaining `docs/sprint6/` hardware-validation items do not run first. They
  become regression criteria for the refactor, and sprint6 resumes afterwards
  against the new contracts.
- Stable docs start absorbing the new contracts per phase, not at the end: each
  phase promotes its own settled surface (`docs/control/` in 2A for bring-up,
  `docs/assets/` in Phase 4 for runtime contracts, `docs/project/` when package
  or generated-artifact policy changes). Operational docs carry supersession
  banners until the phase that rewrites them.
