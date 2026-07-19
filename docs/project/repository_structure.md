# Repository Structure

status: ACTIVE_BASELINE
last_updated: 2026-07-19

## Purpose

This document defines the current canonical workspace structure of the Duo baseline and the stable
documentation split after the cleanup pass.

## Current Workspace Roles

| Surface | Current Path | Role | Rule |
| --- | --- | --- | --- |
| Canonical robot description | `src/agx_arm_sim/agx_arm_description` | Source of truth for Nero, Revo2, and repo-owned OmniHand description assets | Reuse directly for canonical shared assets; do not create another long-term description source of truth. |
| Duo system staging description | `src/duo_body_description` | Current staging package for Duo body plus configurable right or left arm-hand system assembly and description-only bringup | Use as the documented staging surface; do not copy full Nero or OmniHand asset trees into it unnecessarily. |
| MoveIt baseline | `src/agx_arm_moveit` | Current Nero MoveIt configuration, fake `ros2_control`, RViz, and OmniHand simulation profile | Reuse directly and generalize in place; do not fork a second MoveIt package for the Duo system baseline. |
| Runtime arm bridge | `src/agx_arm_ctrl` | Real arm ROS node, launch surfaces, and control-facing integration points | Reuse directly for runtime integration; add OmniHand bridge work without breaking the current arm path. |
| MIT controller runtime | `src/agx_arm_mit_controller` | Integrated `FollowJointTrajectory` execution, MIT command generation, shared trajectory/gravity libraries, and curated controller configs | Reuse directly; keep production controller semantics stable while the current baseline continues to harden the interfaces around it. |
| MIT demos | `src/agx_arm_mit_demos` | Interactive leader recording, saved-trajectory playback, and wakeword teach-and-trigger workflows | Keep app-layer demo entry points here instead of the controller runtime package. |
| MIT tools | `src/agx_arm_mit_tools` | Debug bridges, hold validation, gravity comparison/calibration, and other non-production helpers | Keep bridge/debug/calibration entry points here so runtime ownership stays narrow. |
| Custom ROS messages | `src/agx_arm_msgs` | Repo-owned message layer for controller and future OmniHand-specific diagnostics | Extend here for OmniHand-specific status and tactile messages. |
| Activity coordination | `src/agx_arm_coordination` | Sprint 6 Activity-DAG coordinator, coordinator-internal performer routing, resource model, and YAML graph/catalogue loader for coordinated dual-arm + dual-hand tasks | Keep orchestration here; the OmniHand skill controller stays in `src/agx_arm_ctrl` and arm execution reuses the existing `both_arms`/per-arm FollowJointTrajectory path. |
| Vendored OmniHand SDK | `vendor/OmniHand-Pro-2025` | Third-party SDK, vendor ROS examples, and upstream asset source | Keep vendored; treat as upstream input, not as the public repo contract. |
| Vendored Python SDK runtime pin | `vendor/pyAgxArm` | Pinned Nero SDK fork, MDH tooling, and end-effector driver examples used by the repo runtime baseline | Keep this as the repo-owned runtime pin; do active upstream sync and local SDK development in an external checkout, then retag and bump this vendor pin. |

## Documentation Layout

| Surface | Current Path | Role |
| --- | --- | --- |
| Global docs hub and cross-cutting status | `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`, `docs/target/README.md` | Global navigation plus repo-wide checklist, fixes, open questions, and the repo documentation target |
| Operational bringup + teach SoT | `docs/control` | How to run the system: `environment.md`, `bringups/launches.md`, and `bringups/teach_and_run.md` |
| Component, runtime, and integration docs | `docs/assets` | Asset/repo inventories plus OmniHand, MIT, and control component docs |
| Stable sprint entrypoints | `docs/sprint1/` through `docs/sprint6/` | Sprint-level targets, checklists, errors, open questions, and retained evidence |
| Human repository structure | `docs/project` | Workspace structure, architecture, stable component ownership, and long-term roadmap surfaces |
| Copilot-native guidance | `.github` plus `AGENTS.md` | Repo-local Copilot instructions, skills, agents, and the durable engineering contract |

The future `docs/simulation`, `docs/hand`, and related trees are still valid targets, but they should
be created only when there is stable content to promote into them.

## Source Tree Boundaries

Treat these directories as source-managed:

- `src/`
- `vendor/`
- `docs/`
- `.github/`
- `tools/`
- `config/`
- `scripts/`

Treat these directories as generated or runtime-managed:

- `build/`
- `install/`
- `log/`
- `logs/` when used for run outputs or analysis artifacts rather than curated references

An external sibling checkout such as `../pyAgxArm` is outside this repo. It is part of the SDK
development workflow, not part of the repo-owned source tree here: pull upstream there, land local
changes there, tag the fork there, then advance `vendor/pyAgxArm` in this workspace.

## Current Structure Rules

1. Keep `src/agx_arm_sim/agx_arm_description` as the canonical long-term description package, but allow `src/duo_body_description` as the documented Duo staging package for body-mounted system bringup.
2. Do not split off a new MoveIt package for the Duo system unless the current package becomes unmaintainable.
3. Keep the OmniHand adapter below ROS and keep the public ROS bridge repo-owned inside `src/agx_arm_ctrl` in the current baseline.
4. Add hand-specific runtime surfaces in a way that preserves the current Nero arm runtime path.
5. Keep the integrated MIT action server and `/control/move_mit` ownership in `src/agx_arm_mit_controller`.
6. Put demo applications and workflow tooling under `src/agx_arm_mit_demos` or `src/agx_arm_mit_tools` rather than widening the controller runtime package.
7. Generalize description and launch surfaces to be arm-count-aware from the start; the first executable Duo target is `body + right arm + right OmniHand`, then mirror to the left side.
8. Keep Sprint 6 task orchestration in `src/agx_arm_coordination` (coordinator + performer routing + YAML graph/catalogue loader); do not move the OmniHand skill controller out of `src/agx_arm_ctrl` or fork a second arm execution path for it.
9. Prefer promotion into this stable docs tree over adding more ad hoc sprint notes once a decision is settled.
10. Keep repo-wide checklist, error-and-fix, and open-question summaries in top-level `docs/*.md`; do not recreate a duplicate development-layer summary surface.
11. Keep roadmap intent in `docs/project/roadmap_and_phases.md`, current status in `docs/checklist.md`, and component routing in `docs/project/components/implementation_map.md`.

## Current Deliverables Anchored To This Structure

- `.claude/rules/package-naming.md`
- `.claude/rules/generated-vs-source-assets.md`
- `.claude/rules/local-agent-workflow.md`
- `.claude/rules/ros2-development.md`
- `docs/project/architecture.md`
- `AGENTS.md` and the Copilot-native `.github/` guidance mirrors
- repo-owned OmniHand bridge skeleton and message extensions aligned with the package boundaries above
- Duo body system staging package and the current sprint documentation aligned with the package boundaries above