# Repository Structure

status: ACTIVE_SPRINT3_4_DUO_SYSTEM_STAGING
last_updated: 2026-05-28

## Purpose

This document defines the current workspace structure that Sprint 2 through Sprint 4 treat as canonical.

It is intentionally based on the repo as it exists today, not on a future rename pass.

## Current Workspace Roles

| Surface | Current Path | Role | Sprint 2 Rule |
| --- | --- | --- | --- |
| Canonical robot description | `src/agx_arm_sim/agx_arm_description` | Source of truth for Nero, Revo2, and repo-owned OmniHand description assets | Reuse directly for canonical shared assets; do not create another long-term description source of truth. |
| Duo system staging description | `src/duo_body_description` | Temporary staging package for Duo body plus configurable right/left arm-hand system assembly and description-only bringup | Use for Sprint 3 and Sprint 4 body-mounted system bringup; do not copy full Nero or OmniHand asset trees into it unnecessarily. |
| MoveIt baseline | `src/agx_arm_moveit` | Current Nero MoveIt configuration, fake `ros2_control`, RViz, and OmniHand simulation profile | Reuse directly and generalize in place; do not fork a second MoveIt package for the Duo system baseline. |
| Runtime arm bridge | `src/agx_arm_ctrl` | Real arm ROS node, launch surfaces, and control-facing integration points | Reuse directly for runtime integration; add OmniHand bridge work without breaking the current arm path. |
| MIT controller runtime | `src/agx_arm_mit_controller` | Integrated `FollowJointTrajectory` execution, MIT command generation, shared trajectory/gravity libraries, and curated controller configs | Reuse directly; keep production controller semantics stable while Sprint 2 standardizes interfaces around it. |
| MIT demos | `src/agx_arm_mit_demos` | Interactive leader recording, saved-trajectory playback, and wakeword teach-and-trigger workflows | Keep app-layer demo entry points here instead of the controller runtime package. |
| MIT tools | `src/agx_arm_mit_tools` | Debug bridges, hold validation, gravity comparison/calibration, and other non-production helpers | Keep bridge/debug/calibration entry points here so runtime ownership stays narrow. |
| Custom ROS messages | `src/agx_arm_msgs` | Repo-owned message layer for controller and future OmniHand-specific diagnostics | Extend here for OmniHand-specific status and tactile messages. |
| Vendored OmniHand SDK | `vendor/Omnihand-2025-SDK` | Third-party SDK, vendor ROS examples, and upstream asset source | Keep vendored; treat as upstream input, not as the public repo contract. |
| Python reference workspace | `pyAgxArm` | Nero SDK, MDH tooling, and end-effector driver examples | Reuse as reference or backend support, not as the canonical ROS integration surface. |

## Documentation Layout

| Surface | Current Path | Role |
| --- | --- | --- |
| Stable factual inventories | `docs/assets` | Promoted asset and repository state documents |
| Stable integration/control docs | `docs/control` | OmniHand integration decisions, MIT model notes, and runtime plans |
| Development coordination and working notes | `docs/development` | Fixed roadmap, progress, and component-routing docs plus sprint logs and working sets |
| Sprint 2 policy docs | `docs/project` | Workspace structure, naming, and workflow policy |
| Copilot-native guidance | `.github` plus `AGENTS.md` | Repo-local Copilot instructions, skills, agents, and the durable engineering contract |

The future `docs/planning`, `docs/simulation`, `docs/hand`, and related trees are still valid targets, but they should be created when there is stable content to promote into them.

## Source Tree Boundaries

Treat these directories as source-managed:

- `src/`
- `vendor/`
- `docs/`
- `.github/`
- `tools/`
- `config/`
- `scripts/`
- `pyAgxArm/`

Treat these directories as generated or runtime-managed:

- `build/`
- `install/`
- `log/`
- `logs/` when used for run outputs or analysis artifacts rather than curated references

## Current Structure Rules

1. Keep `src/agx_arm_sim/agx_arm_description` as the canonical long-term description package, but allow `src/duo_body_description` as the documented Sprint 3 and Sprint 4 staging package for body-mounted system bringup.
2. Do not split off a new MoveIt package for the Duo system unless the current package becomes unmaintainable.
3. Keep the OmniHand adapter below ROS and keep the public ROS bridge repo-owned inside `src/agx_arm_ctrl` during Sprint 2.
4. Add hand-specific runtime surfaces in a way that preserves the current Nero arm runtime path.
5. Keep the integrated MIT action server and `/control/move_mit` ownership in `src/agx_arm_mit_controller`.
6. Put demo applications and workflow tooling under `src/agx_arm_mit_demos` or `src/agx_arm_mit_tools` rather than widening the controller runtime package.
7. Generalize description and launch surfaces to be arm-count-aware from the start; the first executable Duo target is `body + right arm + right OmniHand`, then mirror to the left side.
8. Prefer promotion into this stable docs tree over adding more ad hoc sprint notes once a decision is settled.

## Current Deliverables Anchored To This Structure

- `docs/project/package_naming.md`
- `docs/project/generated_vs_source_assets.md`
- `docs/project/local_agent_workflow.md`
- `docs/project/ros2_development_practices.md`
- `docs/project/repo_interaction_diagrams.md`
- `AGENTS.md` and the Copilot-native `.github/` guidance mirrors
- repo-owned OmniHand bridge skeleton and message extensions aligned with the package boundaries above
- Duo body system staging package and Sprint 3/Sprint 4 documentation aligned with the package boundaries above