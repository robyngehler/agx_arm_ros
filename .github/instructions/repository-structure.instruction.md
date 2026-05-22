---
description: "Use when creating, moving, or extending package surfaces in agx_arm_ros. Covers the current Sprint 2 package boundaries and where new work belongs."
---

# Repository Structure

Use the current workspace layout as the implementation truth during Sprint 2.

## Canonical Package Roles

- `src/agx_arm_sim/agx_arm_description`: single discoverable description package for Nero, Revo2, and repo-owned OmniHand assets
- `src/agx_arm_moveit`: current MoveIt baseline and simulation path
- `src/agx_arm_ctrl`: runtime arm bridge, launch surfaces, and current end-effector integration point
- `src/agx_arm_mit_controller`: runtime MIT controller node, shared trajectory/gravity libraries, and curated controller configs
- `src/agx_arm_mit_demos`: interactive recorder, playback, and wakeword demo workflows around the MIT stack
- `src/agx_arm_mit_tools`: debug bridges, hold validation, and calibration helpers around the MIT stack
- `src/agx_arm_msgs`: repo-owned ROS messages
- `vendor/Omnihand-2025-SDK`: upstream SDK input, not the public repo contract

## Sprint 2 Placement Rules

- keep the OmniHand bridge in `src/agx_arm_ctrl` for now
- do not create a second discoverable description package
- do not fork a second MoveIt package for the same Nero baseline
- keep production MIT execution ownership in `src/agx_arm_mit_controller`
- place app-layer demos under `src/agx_arm_mit_demos` instead of the controller runtime package
- place debug, calibration, and legacy helper entry points under `src/agx_arm_mit_tools`
- extend `src/agx_arm_msgs` instead of creating an OmniHand-only message package
- use `.github/` for Copilot-native instruction mirrors, not as a replacement for stable project docs

## Documentation Split

- `docs/assets/`: stable factual inventories and validation state
- `docs/control/`: stable OmniHand and runtime integration decisions
- `docs/development/`: fixed roadmap, progress, and component-routing docs plus sprint working folders
- `docs/project/`: stable Sprint 2 package, naming, and workflow policy
- `.github/`: concise Copilot-native instruction, skill, and agent layer

## Escalation Rule

Create a new package only when a stable public ROS contract, dependency boundary, or rebuild boundary truly requires it.