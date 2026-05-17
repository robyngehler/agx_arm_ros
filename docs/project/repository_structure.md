# Repository Structure

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-14

## Purpose

This document defines the current workspace structure that Sprint 2 treats as canonical.

It is intentionally based on the repo as it exists today, not on a future rename pass.

## Current Workspace Roles

| Surface | Current Path | Role | Sprint 2 Rule |
| --- | --- | --- | --- |
| Canonical robot description | `src/agx_arm_sim/agx_arm_description` | Source of truth for Nero, Revo2, and repo-owned OmniHand description assets | Reuse directly; do not introduce a second discoverable description package. |
| MoveIt baseline | `src/agx_arm_moveit` | Current Nero MoveIt configuration, fake `ros2_control`, RViz, and OmniHand simulation profile | Reuse directly; do not fork a second MoveIt package during Sprint 2. |
| Runtime arm bridge | `src/agx_arm_ctrl` | Real arm ROS node, launch surfaces, and control-facing integration points | Reuse directly for runtime integration; add OmniHand bridge work without breaking the current arm path. |
| MIT controller path | `src/agx_arm_mit_controller` | JointTrajectory-to-MIT execution, gravity model, replay, and validation tools | Reuse directly; keep controller semantics stable while Sprint 2 standardizes interfaces around it. |
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

## Sprint 2 Structure Rules

1. Do not reintroduce a duplicate `agx_arm_description` package or a second URDF source of truth.
2. Do not split off a new MoveIt package during Sprint 2 unless the current package becomes unmaintainable.
3. Keep the OmniHand adapter below ROS and keep the public ROS bridge repo-owned inside `src/agx_arm_ctrl` during Sprint 2.
4. Add hand-specific runtime surfaces in a way that preserves the current Nero arm runtime path.
5. Prefer promotion into this stable docs tree over adding more ad hoc sprint notes once a decision is settled.

## Immediate Sprint 2 Deliverables Anchored To This Structure

- `docs/project/package_naming.md`
- `docs/project/generated_vs_source_assets.md`
- `docs/project/local_agent_workflow.md`
- `docs/project/ros2_development_practices.md`
- `docs/project/repo_interaction_diagrams.md`
- `AGENTS.md` and the Copilot-native `.github/` guidance mirrors
- repo-owned OmniHand bridge skeleton and message extensions aligned with the package boundaries above