# AGENTS.md

This repository keeps durable, tool-neutral engineering rules here and uses `.github/` as the Copilot-native adapter layer.

## Scope

- Preserve the current stable package boundaries; during Sprint 3 and Sprint 4, allow `src/duo_body_description` as a temporary staging surface for Duo body system assembly while the long-term canonical description and planning ownership remains under the existing `agx_arm_*` packages.
- Keep implementation changes small, package-scoped, and validated.
- Keep documentation aligned with any public contract or workflow change.

## Source Of Truth Order

1. `README.md` and `README_EN.md` for repository overview.
2. This file for durable engineering constraints.
3. `docs/assets/` for component architecture, validation, and OmniHand/runtime integration docs.
4. `docs/project/` for human-facing repository structure and architecture.
5. `.github/instructions/` and `.claude/rules/` for agent workflow, naming, and ROS2-practice rules (these do not live in `docs/`).
6. `.github/copilot-instructions.md` for the Copilot operating model.
7. `.github/skills/` for reusable workflows.

## Workspace Rules

- Keep `src/agx_arm_sim/agx_arm_description` as the canonical long-term description package and source of shared Nero and OmniHand assets.
- Allow `src/duo_body_description` as the current Sprint 3 and Sprint 4 staging package for Duo body plus configurable arm-hand system assembly; do not duplicate full Nero or OmniHand asset trees there.
- Keep `src/agx_arm_moveit` as the current MoveIt baseline and generalize it in place rather than forking a second MoveIt package for the Duo system.
- Keep runtime arm and hand integration in `src/agx_arm_ctrl` during Sprint 2.
- Keep production MIT control ownership in `src/agx_arm_mit_controller`.
- Keep MIT demo and workflow apps in `src/agx_arm_mit_demos` instead of the controller runtime package.
- Keep MIT debug bridges, hold checks, and calibration helpers in `src/agx_arm_mit_tools`.
- Keep the OmniHand bridge in `src/agx_arm_ctrl` for now; only revisit a package split after a non-mock backend proves a separate boundary is useful.
- Extend `src/agx_arm_msgs` for repo-owned OmniHand messages instead of creating a second message package.
- Treat `vendor/OmniHand-Pro-2025` as upstream input, not as the public ROS contract.
- Generalize description and launch surfaces to be arm-count-aware from the start; the first executable Duo target is `body + right arm + right OmniHand`, then mirror to the left side.

## ROS Contract Rules

- Keep the public ROS surface agx_arm-centric.
- Prefer shared `control/joint_states` and combined `feedback/joint_states` when coordinating arm and end-effector motion.
- Use `feedback/omnihand/*` for hand-only diagnostics, status, and debugging.
- Keep `control/omnihand/joint_trajectory` only as a bridge-specific compatibility surface until a later action or controller contract is finalized.
- Do not map OmniHand onto `HandCmd`, `HandPositionTimeCmd`, or `HandStatus`.

## Documentation And Source Rules

- Treat `.github/instructions/` and `.claude/rules/` as the agent-facing rule layer (workflow, naming, package, ROS2 practice); keep them consistent with the human docs under `docs/project/` and `docs/assets/`.
- Update `docs/assets/` when an OmniHand or runtime component contract (command, feedback, launch, package) changes.
- Update `docs/project/` when repository structure or architecture changes; update `.github/instructions/` and `.claude/rules/` when workflow, naming, or package-split rules change.
- Treat `.github/`, `docs/`, `src/`, `scripts/`, `config/`, and `vendor/` as source-managed.
- Do not treat `build/`, `install/`, `log/`, or transient run outputs as canonical source.

## Validation Rules

- Prefer package-scoped `colcon build --packages-select ...` while iterating.
- Run diagnostics on touched files.
- For launch, message, or bridge changes, include at least one executable package-level validation step.
- If live hardware validation cannot be run, say so explicitly.

## Definition Of Done

- The change respects the current package boundaries, including any temporary staging surface explicitly documented in `docs/project/`.
- The smallest relevant docs were updated.
- The Copilot-native `.github` mirrors remain consistent with the stable docs.
- The narrowest relevant validation was run, or the limitation was called out.