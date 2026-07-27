# Sprint Refactor Target
status: ACTIVE_REFACTOR_ENTRYPOINT
last_updated: 2026-07-27
branch: ROS2_Duo_System_V02

Sprint Refactor is the V02 migration surface for the coordination, safety, and
runtime-consolidation work proposed for the current Duo Nero system.

## Main goal

Cross-check the coordination architecture proposal against the current codebase
and turn it into a phase-ordered integration plan that can be implemented
without losing control of the present Duo baseline.

## Scope

1. Validate the proposal findings against the current code in `agx_arm_ctrl`,
   `agx_arm_mit_controller`, `agx_arm_coordination`, `agx_arm_moveit`,
   `agx_arm_msgs`, `agx_arm_description`, and `pyAgxArm`.
2. Freeze the migration order, ownership boundaries, and validation gates for
   branch `ROS2_Duo_System_V02`.
3. Keep the work scoped to one Duo unit on one Jetson: no `ros2_control`
   rewrite, no full C++ rewrite, no multi-unit consensus in this sprint surface.
4. Keep hardware-touching validation explicit and separate from editor-only or
   x86 read-only checks.

## Current status

- The proposal is now stored inside this sprint surface.
- An initial read-only code cross-check is complete and confirms the major
  proposal findings.
- Implementation has not started in this sprint surface yet.

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