# Sprint 4 Open Questions

## Package Boundary

- Should `src/duo_body_description` remain a standalone package after Sprint 4, or should its stable outputs be promoted into `src/agx_arm_sim/agx_arm_description` once the Duo system baseline settles?
- If it remains separate for longer, what is the explicit promotion or retirement criterion so it does not become an undocumented parallel source of truth?

## Naming And Bringup

- Should the long-term multi-arm runtime use per-arm namespaces, per-arm prefixes, or both?
-> answer: use prefix for a common tf and control group. One body and multiple arms are treated as one robot.
- How should a future `>2` arm-hand configuration be represented without hardcoding only `left` and `right` everywhere in the higher-level launch and planning surfaces?
-> answer: right now one base can contain max 2 arms, which equals one robot. >2 arms suggest an other body-2arm-robot, seperated by another namespace. (for now, a common tf and planning structure is still somewhat needed.)

## MoveIt And Controller Generalization

- Should `agx_arm_moveit` grow a generated multi-profile configuration path, or is a single composable Xacro/SRDF surface sufficient for the current two-arm target?
-> answer: multi-profile is better since one arm can operate on its own. Keep consistent with the prefix approach to allow common namespaces and make a plan to best keep left_*.srdf, right_*.srdf and duo_*.srdf synced.
- What is the minimum change to the MIT-controller RViz path that preserves the current single-arm workflow while making room for a second arm?
-> answer: not investigated yet, please investigate and take the above decisions into account.
- Which pieces of the current control stack should stay shared, and which should become per-arm once the second arm is added?
-> answer: not fully clear, but a single arm definitely needs a single MIT controller (different gravity, tools, ...), the control and path planning and collision avoidance should be shared. Tools like playback a recorded trajectory can be shared but should live as one instance each.

## Early Task-Level Contract

- What is the smallest stable planning and execution contract needed for the first coordinated dual-arm task?
-> answer: shared planning and collision checks at least.
- Is the first two-arm pouring benchmark better represented as two synchronized arm plans, one coupled planning group, or a higher-level task orchestration layer above per-arm plans?
-> answer: one coupled planning group but merged with a task orchestration above... f.e. as a first step, recorded trajectories for each arm should be synced or performed after one each other. But before performing there should be a merged planning in the common planner of trajectories.