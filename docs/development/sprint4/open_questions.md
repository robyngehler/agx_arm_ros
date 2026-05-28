# Sprint 4 Open Questions

## Package Boundary

- Should `src/duo_body_description` remain a standalone package after Sprint 4, or should its stable outputs be promoted into `src/agx_arm_sim/agx_arm_description` once the Duo system baseline settles?
- If it remains separate for longer, what is the explicit promotion or retirement criterion so it does not become an undocumented parallel source of truth?

## Naming And Bringup

- Should the long-term multi-arm runtime use per-arm namespaces, per-arm prefixes, or both?
- How should a future `>2` arm-hand configuration be represented without hardcoding only `left` and `right` everywhere in the higher-level launch and planning surfaces?

## MoveIt And Controller Generalization

- Should `agx_arm_moveit` grow a generated multi-profile configuration path, or is a single composable Xacro/SRDF surface sufficient for the current two-arm target?
- What is the minimum change to the MIT-controller RViz path that preserves the current single-arm workflow while making room for a second arm?
- Which pieces of the current control stack should stay shared, and which should become per-arm once the second arm is added?

## Early Task-Level Contract

- What is the smallest stable planning and execution contract needed for the first coordinated dual-arm task?
- Is the first two-arm pouring benchmark better represented as two synchronized arm plans, one coupled planning group, or a higher-level task orchestration layer above per-arm plans?