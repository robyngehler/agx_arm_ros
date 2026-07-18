# Components

This index points to the stable component surfaces that matter for the current baseline.

## Runtime and planning

- `src/agx_arm_ctrl`: arm runtime bridge, launch surfaces, and current OmniHand integration point
- `src/agx_arm_mit_controller`: integrated execution, gravity-aware MIT control, and FJT ownership
- `src/agx_arm_moveit`: planning baseline, fake-controller compatibility path, and MoveIt launch composition
- `src/agx_arm_coordination`: dual-arm and dual-hand task orchestration through the Activity-DAG coordinator

## Description and models

- `src/agx_arm_sim/agx_arm_description`: canonical long-term description assets
- `src/duo_body_description`: current Duo staging package for body-mounted system assembly

## Messages and diagnostics

- `src/agx_arm_msgs`: repo-owned messages for OmniHand status and tactile data
- `../assets/omnihand/`: stable OmniHand validation notes, setup notes, and bridge-facing runtime details

## Supporting references

- `../architecture.md`: cross-component runtime and launch flow
- `../repository_structure.md`: package ownership and documentation boundaries
- `../../control/bringups/launches.md`: runnable system entrypoints
- `../../control/environment.md`: environment, wrapper, and platform rules