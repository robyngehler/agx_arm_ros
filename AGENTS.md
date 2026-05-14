# AGENTS.md

This repository keeps durable, tool-neutral engineering rules here and uses `.github/` as the Copilot-native adapter layer.

## Scope

- Preserve the current package boundaries during Sprint 2.
- Keep implementation changes small, package-scoped, and validated.
- Keep documentation aligned with any public contract or workflow change.

## Source Of Truth Order

1. `README.md` and `README_EN.md` for repository overview.
2. This file for durable engineering constraints.
3. `docs/assets/` for stable repository and asset validation.
4. `docs/control/` for OmniHand and runtime integration decisions.
5. `docs/project/` for current Sprint 2 package, naming, and workflow policy.
6. `.github/copilot-instructions.md` for the Copilot operating model.
7. `.github/instructions/` for targeted task guidance.
8. `.github/skills/` for reusable workflows.

## Workspace Rules

- Keep `src/agx_arm_sim/agx_arm_description` as the single discoverable description package.
- Keep `src/agx_arm_moveit` as the current MoveIt baseline during Sprint 2.
- Keep runtime arm and hand integration in `src/agx_arm_ctrl` during Sprint 2.
- Keep the OmniHand bridge in `src/agx_arm_ctrl` for now; only revisit a package split after a non-mock backend proves a separate boundary is useful.
- Extend `src/agx_arm_msgs` for repo-owned OmniHand messages instead of creating a second message package.
- Treat `vendor/Omnihand-2025-SDK` as upstream input, not as the public ROS contract.

## ROS Contract Rules

- Keep the public ROS surface agx_arm-centric.
- Prefer shared `control/joint_states` and combined `feedback/joint_states` when coordinating arm and end-effector motion.
- Use `feedback/omnihand/*` for hand-only diagnostics, status, and debugging.
- Keep `control/omnihand/joint_trajectory` only as a bridge-specific compatibility surface until a later action or controller contract is finalized.
- Do not map OmniHand onto `HandCmd`, `HandPositionTimeCmd`, or `HandStatus`.

## Documentation And Source Rules

- Treat `.github/` as a concise Copilot-native mirror of the stable policy and integration docs under `docs/project/` and `docs/control/`.
- Update `docs/control/` when the OmniHand command, feedback, launch, or package contract changes.
- Update `docs/project/` when package boundaries, naming rules, or working workflow changes.
- Treat `.github/`, `docs/`, `src/`, `scripts/`, `config/`, and `vendor/` as source-managed.
- Do not treat `build/`, `install/`, `log/`, or transient run outputs as canonical source.

## Validation Rules

- Prefer package-scoped `colcon build --packages-select ...` while iterating.
- Run diagnostics on touched files.
- For launch, message, or bridge changes, include at least one executable package-level validation step.
- If live hardware validation cannot be run, say so explicitly.

## Definition Of Done

- The change respects the current package boundaries.
- The smallest relevant docs were updated.
- The Copilot-native `.github` mirrors remain consistent with the stable docs.
- The narrowest relevant validation was run, or the limitation was called out.