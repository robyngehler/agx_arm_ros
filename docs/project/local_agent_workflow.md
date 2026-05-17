# Local Agent Workflow

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-14

## Purpose

This document defines how developers and local agents should work inside this repo during Sprint 2.

The workflow is optimized for minimal duplication, stable ownership boundaries, and reproducible promotion from working notes into stable docs.

## Start Here

Before changing package structure or OmniHand integration surfaces, read these documents first:

- `AGENTS.md`
- `.github/copilot-instructions.md`
- `docs/project/ros2_development_practices.md`
- `docs/assets/repository_asset_inventory.md`
- `docs/assets/nero_asset_validation.md`
- `docs/assets/omnihand_asset_validation.md`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`
- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/generated_vs_source_assets.md`

Treat `.github/instructions/` and `.github/skills/` as concise Copilot-native mirrors of the stable docs above, not as a separate competing policy tree.

## Working Rules

1. Start from the canonical owning package instead of creating a parallel implementation surface.
2. Reuse `src/agx_arm_sim/agx_arm_description` for description assets and `src/agx_arm_moveit` for planning baselines.
3. Keep the OmniHand adapter below ROS and keep the public ROS bridge repo-owned.
4. Do not map OmniHand onto the existing Revo2-specific ROS message contract.
5. Keep the OmniHand bridge in `agx_arm_ctrl` during Sprint 2 unless a real technical boundary requires a split later.
6. Do not introduce package renames during Sprint 2 unless a real technical boundary requires them.

## Preferred Change Order

When adding new Sprint 2 functionality, use this order:

1. define or confirm the public contract in docs,
2. add repo-owned messages when standard ROS types are not sufficient,
3. add the smallest bridge or backend skeleton that exercises the contract,
4. validate with the narrowest available check,
5. promote the result into stable docs if it changes the repo baseline.

## Fork And Vendor Workflow

The vendored OmniHand SDK now tracks the workspace fork as its default submodule remote.

Sprint 2 workflow for vendor changes:

1. land portability or safety patches in the workspace fork first,
2. keep the original Agibot repository available as `upstream` for sync and review,
3. document which patches are repo-local and which are intended for upstream submission,
4. avoid relying on untracked dirty vendor state as the canonical implementation.

## Promotion Workflow

Use `docs/development/` for working logs and in-progress notes.

Keep only the roadmap, progress monitor, and component map at the top of `docs/development/`. Put sprint-local discovery, checklist, error/fix, and niche implementation notes under `docs/development/sprintN/`.

Promote stable outputs to the top-level docs tree when they become the canonical answer. Sprint 2 policy and structure decisions should live under `docs/project/` once settled.

Keep `.github/` guidance concise and synchronized with the stable docs it mirrors.

## Validation Expectations

Every Sprint 2 change should include the narrowest useful validation available for the touched surface.

Examples:

- message definitions: interface generation or package-level diagnostics
- Python bridge code: import or syntax checks plus editor diagnostics
- launch files: launch-file diagnostics or package-level build checks
- policy docs: editor diagnostics plus cross-check against current repo state

## Anti-Patterns

Avoid these during Sprint 2:

- creating a second discoverable description package
- creating a duplicate MoveIt package for the same Nero baseline
- using vendor ROS topics as the public repo contract
- treating generated build outputs as source of truth
- letting `.github/` instructions drift away from `docs/project/` or `docs/control/`
- reopening already-settled naming decisions without a concrete implementation blocker