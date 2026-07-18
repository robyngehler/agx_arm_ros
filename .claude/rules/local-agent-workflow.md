---
paths:
  - "src/**"
  - "docs/**"
  - "**/*.launch.py"
  - "**/*.msg"
---

# Local Agent Workflow

*Use when planning and sequencing work in agx_arm_ros. Covers the preferred change order, docs promotion
flow, and validation expectations for the current Duo baseline.*

Use a small, reproducible change order and promote stable answers into the top-level docs tree.

## Start Here

Read these first when changing package boundaries or OmniHand integration surfaces:

- `AGENTS.md`
- `CLAUDE.md`
- `docs/control/environment.md`
- `docs/control/bringups/launches.md`
- `.claude/rules/ros2-development.md`
- `docs/assets/repository_asset_inventory.md`
- `docs/assets/nero_asset_validation.md`
- `docs/assets/omnihand_asset_validation.md`
- `docs/assets/omnihand/omnihand_ros_integration_options.md`
- `docs/assets/omnihand/omnihand_wrapper_integration_plan.md`

## Preferred Change Order

1. confirm the public contract in docs or rules
2. add or adjust repo-owned messages only when standard ROS types are not enough
3. make the smallest bridge, launch, or backend change that exercises the contract
4. run the narrowest useful validation
5. update stable docs when the repo baseline changed

## Working Rules

- start from the owning package instead of creating a parallel surface
- use `src/duo_body_description` only as the documented Duo staging surface for Duo body system bringup; keep long-term canonical description ownership in `src/agx_arm_sim/agx_arm_description`
- keep the OmniHand bridge in `agx_arm_ctrl` in the current baseline
- keep the public ROS contract agx_arm-centric
- make description and bringup surfaces arm-count-aware from the start, with `body + right arm + right OmniHand` as the current executable Duo target
- keep sprint entrypoints in `docs/sprintN/` and keep detailed historical evidence in `docs/development/sprintN/` only for migration, delete if the cleanup is done, also delete this reference to the old architecture afterwards; keep only roadmap, progress, and component routing at the top of `docs/development/`
- do not map OmniHand onto the Revo2-specific message contract
- ask before any hardware-touching action; if hardware access is granted, `sudo` is allowed for repo CAN workflows in the intended hardware environment
- keep `.claude/` guidance in sync with the stable docs it mirrors

## Validation Expectations

- use editor diagnostics for touched files
- prefer `bash ./scripts/colcon_build_system_python.sh --packages-select ...` for message, launch, and bridge work when the environment supports it
- run `colcon test --packages-select ...` from a system-Python ROS shell when relevant tests exist
- call out explicitly when hardware validation could not be run in the current environment
