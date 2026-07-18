# Sprint 1 Target

status: HISTORICAL_ENTRYPOINT
last_updated: 2026-07-18

Sprint 1 was the repository and asset discovery phase.

## Main goal

Establish the first trustworthy picture of:

- canonical Nero description ownership
- the active MoveIt and MIT controller surfaces
- the initial OmniHand vendor and asset situation
- missing AGV/base and broader simulation inputs

## What Sprint 1 settled

- `src/agx_arm_sim/agx_arm_description` became the canonical long-term description package
- the duplicate root `agx_arm_description` package and the old `agx_arm_urdf` submodule dependency
	were retired
- the active workspace surface narrowed to the Nero-focused baseline
- the first stable asset and inventory docs were promoted into `docs/assets/`
- the early OmniHand direction became wrapper-first rather than vendor-ROS-first

## Stable outputs promoted elsewhere

- `docs/assets/repository_asset_inventory.md`
- `docs/assets/nero_asset_validation.md`
- `docs/assets/omnihand_asset_validation.md`
- `docs/assets/agv_cad_inventory.md`
- `docs/assets/mit_controller/mit_controller_model_inventory.md`
- `docs/assets/omnihand/omnihand_wrapper_integration_plan.md`

## Historical evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../evidence/mit_soft_control_and_gravity_proposal.md`