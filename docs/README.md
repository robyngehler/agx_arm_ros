# agx_arm_ros Docs Hub

This directory is the human-facing documentation hub for the current Duo baseline.

## What lives where

- `control/`: how to start, test, and operate the current system
  - `environment.md`: Python environment split, ROS overlays, build and test wrappers, and platform caveats
  - `bringups/launches.md`: canonical baseline, tool, and demo bringup map
  - `bringups/teach_and_run.md`: teach, record, replay, and coordinator-facing motion flow
- `project/`: stable repository structure, architecture, and component ownership
- `assets/`: stable component facts, validation state, and runtime integration notes
- `sprint1/` through `sprint6/`: sprint targets, checklists, errors, open questions, and retained evidence
- `sprint_refactor/`: V02 coordination, safety, and runtime refactor proposal, code cross-check, and phased integration plan
- `sprint_physAI/`: pre-sprint Physical AI exploration retained outside the main sprint line

## Global navigation

- `target/README.md`: repo documentation target and ownership rules
- `checklist.md`: current repo-wide integration checklist and cleanup status
- `errors_and_fixes.md`: recurring repo-wide issues, current mitigations, and validated fixes
- `open_questions.md`: cross-cutting design questions that remain intentionally open

## Current canonical entrypoints

- Current environment and wrapper rules: `control/environment.md`
- Current hardware bringup: `control/bringups/launches.md`
- Current teach and replay workflow: `control/bringups/teach_and_run.md`
- Current package and staging rules: `project/repository_structure.md`
- Provisioning a new Jetson host: `project/jetson_migration.md`
- Current architecture diagrams: `project/architecture.md`
- Current repo target and documentation ownership rules: `target/README.md`
- Current implementation focus: `sprint_refactor/` (V02 refactor; canonical plan
  `sprint_refactor/planning/integration_plan.md`)
- `sprint6/` is paused and adapts to the refactor contracts afterwards; its step-and-settle
  and hand-window notes are superseded by the per-device CAN topology

For operational launches, prefer the `control/bringups/launches.md` taxonomy first: pick a baseline,
then add the matching tool or demo on top. In the normal MoveIt or MIT wrapper path,
`start_agx_arm_components.launch.py` should normally be selected through `execution_profile` presets
rather than rebuilt from one-off launch argument combinations.

## Notes on authority

- Package-local READMEs may keep package-local behavior, parameters, and focused examples.
- The canonical system bringup matrix lives in `control/`.
- Stable environment and wrapper rules live in `control/environment.md`.
- Historical evidence stays in the matching sprint surface or in git history; it should not silently override stable docs.