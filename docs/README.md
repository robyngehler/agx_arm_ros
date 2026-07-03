# agx_arm_ros Docs Hub

This directory is the human-facing documentation hub for the current Duo baseline.

## What lives where

- `control/`: how to start and operate the current system
  - `bringup.md`: baseline, tool, and demo bringup map
  - `teach_and_run.md`: teach, record, replay, and coordinator-facing motion flow
- `project/`: stable repository structure, architecture, and environment workflows
- `assets/`: stable component facts, validation state, and runtime integration notes
- `development/`: roadmap, progress, cross-sprint checklist/fixes/open questions, and sprint-local working evidence

## Global navigation

- `checklist.md`: current repo-wide docs and consistency checklist
- `errors_and_fixes.md`: recurring repo-wide issues and the validated fixes
- `open_questions.md`: cross-cutting decisions that are still intentionally open

## Current canonical entrypoints

- Current hardware bringup: `control/bringup.md`
- Current teach and replay workflow: `control/teach_and_run.md`
- Current package and staging rules: `project/repository_structure.md`
- Current Python build/runtime split: `project/python_environment_workflow.md`
- Current sprint coordination overview: `development/README.md`

For operational launches, prefer the top-level `control/bringup.md` taxonomy first: pick a baseline, then add the matching tool or demo on top. In the normal MoveIt/MIT wrapper path, `start_agx_arm_components.launch.py` should normally be selected through `execution_profile` presets rather than rebuilt from one-off launch argument combinations.

## Notes on authority

- Package-local READMEs should describe package-local behavior and constraints.
- Stable operational launch combinations belong in `control/`.
- Historical sprint notes stay under `development/sprintN/` and should not silently override stable docs.