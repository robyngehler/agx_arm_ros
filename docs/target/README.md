# Repository Documentation Target

status: ACTIVE_BASELINE
last_updated: 2026-07-19
scope: repo-wide documentation ownership and cleanup closure

## Purpose

This file defines the stable target state for repo documentation after the cleanup pass.

It is the repo-level documentation policy surface: more specific than `README.md`, less durable than
`AGENTS.md`, and less transient than sprint-local notes.

## Current target layout

```text
README.md
docs/
  README.md
  checklist.md
  target/
    README.md
  errors_and_fixes.md
  open_questions.md
  control/
    environment.md
    bringups/
      launches.md
      teach_and_run.md
  project/
    architecture.md
    repository_structure.md
    components/
      README.md
      <stable component docs when needed>
  assets/
    <stable inventories, validation, and runtime/component facts>
  sprintX/
    target/README.md
    checklist.md
    errors_and_fixes.md
    open_questions.md
    <optional evidence and working material>
```

## Ownership rules

- `README.md`: short repo entrypoint only.
- `docs/README.md`: human docs hub and routing page.
- `docs/control/`: canonical operational source of truth.
- `docs/project/`: stable repository structure, architecture, and component ownership.
- `docs/assets/`: stable factual inventories, validation state, and runtime/component analysis.
- `docs/sprintX/`: working notes, sprint targets, retained evidence, and historical context.
- `docs/checklist.md`, `docs/errors_and_fixes.md`, and `docs/open_questions.md`: repo-wide status,
  recurring issues, and open design questions.
- `docs/target/README.md`: repo documentation target, ownership rules, and cleanup-closure policy.

## Package README rule

Package-local READMEs may keep:

- package-local behavior and constraints
- focused examples for that package
- parameter notes that help use the package in isolation

Package-local READMEs should not keep:

- a second repo-wide bringup matrix
- duplicate environment policy
- competing architecture summaries that override `docs/project/`
- sprint-level working notes that belong in `docs/sprintX/`

## Agent guidance rule

`AGENTS.md` remains the durable engineering contract. `CLAUDE.md`, `.claude/`, `.github/`, and the
matching instruction or rule mirrors must stay concise and route to the canonical docs above instead
of repeating them.

## Cleanup closure

The documentation migration is closed for the current baseline.

Closed outcomes:

- `docs/development/` is retired
- `docs/control/bringup.md` is retired
- `docs/project/python_environment_workflow.md` is retired
- `docs/project/repo_interaction_diagrams.md` is retired
- repo-wide references now point at the canonical `docs/control/`, `docs/project/`, and sprint surfaces
- git history and retained sprint evidence are the audit trail; no separate legacy-doc inventory is kept

## Repo-wide constraints that must remain visible

- Hardware gate: ask before any hardware-touching action; `sudo` is allowed only after explicit
  approval for the session.
- Platform split: separate Jetson or other `aarch64` ROS plus hardware validation from x86 or
  editor-only analysis.
- Environment split: build and test on system Python; use `scripts/run_in_ros_conda.sh -- <command>`
  for Conda-backed runtime commands.
- Shared CAN safety: keep `one-shot on` as the stable baseline, avoid sustained concurrent
  arm-plus-hand command pressure on the same side bus, and route detailed operating guidance through
  `docs/errors_and_fixes.md` and `docs/control/bringups/teach_and_run.md`.

## Promotion rule

Promote information into stable docs only when it is reusable beyond one sprint or one debug
session. Otherwise keep it in the matching sprint surface or rely on git history.

## Current live focus

The current repo-wide work focus is no longer doc migration. It is Sprint 6 runtime hardening and
hardware validation, especially:

- shared-bus arm-plus-hand operating limits
- Duo-hand hardware validation
- coordinator and Hefeweizen demo sign-off

