# agx_arm_ros Docs Hub

This directory is the human-facing documentation hub for the current Duo baseline and the active
documentation migration.

## What lives where

- `control/`: how to start and operate the current system
  - `environment.md`: Python environment split, ROS overlays, build and test wrappers, and platform caveats
  - `bringups/launches.md`: canonical baseline, tool, and demo bringup map
  - `teach_and_run.md`: teach, record, replay, and coordinator-facing motion flow
- `project/`: stable repository structure, architecture, and component ownership
- `assets/`: stable component facts, validation state, and runtime integration notes
- `sprint1/` through `sprint6/`: first-class sprint targets, checklists, errors, and open questions
- `development/`: roadmap, progress, component routing, and historical working evidence that is still being migrated

## Global navigation

- `target/README.md`: global target, documentation migration plan, and current sub-phase focus
- `target/legacy_doc_inventory.md`: repo-wide cleanup inventory for legacy docs and old shim paths
- `checklist.md`: current repo-wide migration checklist and consistency status
- `errors_and_fixes.md`: recurring repo-wide issues, current mitigations, and validated fixes
- `open_questions.md`: cross-cutting decisions that are still intentionally open

## Current canonical entrypoints

- Current environment and wrapper rules: `control/environment.md`
- Current hardware bringup: `control/bringups/launches.md`
- Current teach and replay workflow: `control/teach_and_run.md`
- Current package and staging rules: `project/repository_structure.md`
- Current architecture diagrams: `project/architecture.md`
- Current repo target and migration plan: `target/README.md`
- Current sprint entrypoints during the migration: `sprint6/` plus `development/README.md`

For operational launches, prefer the top-level `control/bringups/launches.md` taxonomy first: pick a baseline,
then add the matching tool or demo on top. In the normal MoveIt or MIT wrapper path,
`start_agx_arm_components.launch.py` should normally be selected through `execution_profile` presets
rather than rebuilt from one-off launch argument combinations.

## Migration status

- `control/environment.md` is now the canonical environment page
- `control/bringups/launches.md` is now the canonical launch page
- top-level `docs/sprintX/` surfaces now exist as migration entrypoints while detailed evidence still lives under `development/`
- the root `README.md` and `README_EN.md` now act as short project entrypoints

## Notes on authority

- Package-local READMEs should describe package-local behavior and constraints.
- Stable operational launch combinations belong in `control/`.
- Stable environment and wrapper rules belong in `control/environment.md`.
- Historical sprint notes stay under `development/sprintN/` until promoted or moved and should not silently override stable docs.