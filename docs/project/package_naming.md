# Package Naming

status: ACTIVE_SPRINT3_4_DUO_SYSTEM_STAGING
last_updated: 2026-05-28

## Purpose

This document defines how package names should be interpreted during the current Sprint 2 through Sprint 4 transition.

The primary rule is to avoid rename churn while the OmniHand bridge and common environment are still being stabilized.

## Naming Policy

Use the current repo package names as the implementation truth.

Use the roadmap package names as logical roles, not as an instruction to rename working packages immediately.

## Current Canonical Packages

| Current Package | Current Role | Sprint 2 Naming Rule |
| --- | --- | --- |
| `agx_arm_description` | Canonical description package under `src/agx_arm_sim` | Keep the existing name. |
| `duo_body_description` | Temporary staging package for Duo body system assembly and description-only bringup | Keep the existing name while Sprint 3 and Sprint 4 validate the body-mounted multi-arm system; decide later whether to promote or merge it. |
| `agx_arm_moveit` | Current Nero MoveIt baseline | Keep the existing name. |
| `agx_arm_ctrl` | Runtime arm bridge and launch package | Keep the existing name. |
| `agx_arm_mit_controller` | MIT-controller trajectory and gravity path | Keep the existing name. |
| `agx_arm_msgs` | Repo-owned ROS message layer | Keep the existing name and extend it when OmniHand needs repo-owned message types. |

## Roadmap Role Mapping

| Roadmap Role Name | Current Repo Surface | Sprint 2 Guidance |
| --- | --- | --- |
| `nero_description` | `src/agx_arm_sim/agx_arm_description` | Treat as a logical alias only. Do not rename the package now. |
| `duo_system_description` | `src/duo_body_description` | Treat as the current staging surface for body-mounted multi-arm and multi-hand bringup, not as a final long-term package boundary yet. |
| `nero_moveit_config` | `src/agx_arm_moveit` | Treat as a logical alias only. Do not rename the package now. |
| `nero_control_bridge` | split across `src/agx_arm_ctrl` and `src/agx_arm_mit_controller` | Preserve the current split until a real consolidation need is proven. |
| `omnihand_driver_ros2` | logical future bridge role only | Keep the actual OmniHand bridge in `src/agx_arm_ctrl` during Sprint 2; a split-out package remains optional later. |
| `omnihand_description` | repo-owned OmniHand assets inside `agx_arm_description` | Keep assets in the canonical description package for now. |

## When To Create A New Package

Create a new package only when at least one of these becomes true:

1. the code has a stable public ROS contract of its own,
2. it needs a separate dependency boundary from `agx_arm_ctrl`,
3. it would otherwise force unrelated rebuilds or package coupling,
4. the repo would be clearer with a dedicated installable surface than with another internal module.

Temporary staging packages are allowed when an integration slice needs a narrow installable system-assembly surface before the long-term canonical package boundary is settled. If that happens, document the staging role and the expected promotion or merge path in `docs/project/`.

Current Sprint 2 decision:

- do not split the OmniHand bridge out of `agx_arm_ctrl` yet
- revisit that only after the non-mock backend proves a separate package boundary is useful
- keep `duo_body_description` as the documented Sprint 3 and Sprint 4 staging package while the body-mounted system bringup and multi-arm-safe description path are still being validated

## Preferred Naming For New Sprint 2 Surfaces

If a new package is needed during Sprint 2, prefer these names:

- `omnihand_driver_ros2` for a dedicated repo-owned ROS bridge package
- `omnihand_backend` only for a non-ROS internal library or module, not for the public ROS contract

These are reserved future names, not current required package creations.

Avoid creating packages whose only purpose is to mirror vendor naming.

## Names To Avoid In The Current Baseline

- do not create additional description packages beyond `agx_arm_description` and the documented `duo_body_description` staging surface
- do not create a second discoverable description package for OmniHand alone
- do not create `nero_moveit_config` as a duplicate of `agx_arm_moveit`
- do not create a second message package just for OmniHand while `agx_arm_msgs` already exists
- do not create `omnihand_driver_ros2` during Sprint 2 only to match roadmap wording
- do not rename existing packages purely to match roadmap wording

## Message Naming

For new OmniHand-specific ROS messages, use repo-owned names under `agx_arm_msgs`:

- `OmniHandStatus.msg`
- `OmniHandTactileRaw.msg`

Do not reuse the existing Revo2-specific `HandStatus.msg`, `HandCmd.msg`, or `HandPositionTimeCmd.msg` for OmniHand.