---
description: "Use when planning and sequencing work in agx_arm_ros. Covers the preferred change order, docs promotion flow, and validation expectations for Sprint 2."
---

# Local Agent Workflow

Use a small, reproducible change order and promote stable answers into the top-level docs tree.

## Start Here

Read these first when changing package boundaries or OmniHand integration surfaces:

- `AGENTS.md`
- `.github/copilot-instructions.md`
- `docs/project/ros2_development_practices.md`
- `docs/assets/repository_asset_inventory.md`
- `docs/assets/nero_asset_validation.md`
- `docs/assets/omnihand_asset_validation.md`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`

## Preferred Change Order

1. confirm the public contract in docs or instructions
2. add or adjust repo-owned messages only when standard ROS types are not enough
3. make the smallest bridge, launch, or backend change that exercises the contract
4. run the narrowest useful validation
5. update stable docs when the repo baseline changed

## Working Rules

- start from the owning package instead of creating a parallel surface
- keep the OmniHand bridge in `agx_arm_ctrl` during Sprint 2
- keep the public ROS contract agx_arm-centric
- keep sprint-local evidence in `docs/development/sprintN/` and keep only roadmap, progress, and component routing at the top of `docs/development/`
- do not map OmniHand onto the Revo2-specific message contract
- keep `.github/` guidance in sync with the stable docs it mirrors

## Validation Expectations

- use editor diagnostics for touched files
- prefer `colcon build --packages-select ...` for message, launch, and bridge work
- call out explicitly when hardware validation could not be run in the current environment