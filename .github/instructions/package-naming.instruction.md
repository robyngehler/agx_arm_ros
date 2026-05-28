---
description: "Use when naming new packages, messages, launch arguments, or deciding whether OmniHand should split out of agx_arm_ctrl during Sprint 2."
---

# Package Naming

Avoid rename churn while Sprint 2 is still stabilizing the shared environment and OmniHand bridge.

## Current Canonical Names

- keep `agx_arm_description`, `duo_body_description`, `agx_arm_moveit`, `agx_arm_ctrl`, `agx_arm_mit_controller`, and `agx_arm_msgs`
- treat roadmap names as logical roles, not immediate rename instructions

## Duo System Naming Rules

- keep `duo_body_description` as the documented Sprint 3 and Sprint 4 staging package while the body-mounted multi-arm system is still being validated
- keep canonical shared Nero and OmniHand assets in `agx_arm_description`
- use distinct arm prefixes such as `left_arm_` and `right_arm_` when composing the Duo system so the arm chain does not collide with OmniHand `left_base_link` and `right_base_link`

## OmniHand Naming Rules

- keep the bridge in `agx_arm_ctrl` during Sprint 2
- if a later dedicated ROS bridge package becomes necessary, prefer `omnihand_driver_ros2`
- if a later non-ROS backend library becomes necessary, prefer `omnihand_backend`
- avoid packages that only mirror vendor naming

## Message And Surface Naming Rules

- keep OmniHand-specific messages under `agx_arm_msgs`
- use repo-owned names such as `OmniHandStatus` and `OmniHandTactileRaw`
- do not reuse `HandCmd`, `HandPositionTimeCmd`, or `HandStatus` for OmniHand
- use `omnihand_type:=left|right` for side selection and normalized `left_*` or `right_*` joint names in ROS-facing surfaces

## Split Decision Rule

Do not split the bridge into a new package during Sprint 2 unless one of these becomes true:

1. the bridge acquires a stable ROS contract independent from `agx_arm_ctrl`
2. it needs a dependency boundary that `agx_arm_ctrl` should not carry
3. it creates enough unrelated rebuild churn that a dedicated package is materially clearer

Do not create additional description packages beyond `agx_arm_description` and the documented `duo_body_description` staging surface unless a new long-term boundary is explicitly promoted into `docs/project/`.