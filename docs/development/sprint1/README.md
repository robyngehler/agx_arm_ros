# Sprint 1 Working Notes

## Purpose

This folder tracks Sprint 1 implementation details for the "Repository and Asset Discovery" objective in the physical AI roadmap.

It captures confirmed repository state as of 2026-05-13, unresolved gaps, and the document set that can later be promoted into the stable top-level docs tree.

As of 2026-05-12, the stable Sprint 1 outputs have been promoted into the top-level docs tree. This folder remains the implementation log and working-note source for that promotion.

Promoted stable Sprint 1 outputs:

- `docs/assets/repository_asset_inventory.md`
- `docs/assets/nero_asset_validation.md`
- `docs/assets/omnihand_asset_validation.md`
- `docs/assets/agv_cad_inventory.md`
- `docs/control/mit_controller_model_inventory.md`
- `docs/control/omnihand_phase1_joint_map.md`
- `docs/control/omnihand_phase1_run_log.md`
- `docs/control/omnihand_wrapper_integration_plan.md`

Current working location during implementation:

- `docs/development/sprint1/assets/`
- `docs/development/sprint1/control/`
- `docs/development/sprint1/hand/`
- `docs/development/sprint1/checklist.md`
- `docs/development/sprint1/errors_and_fixes.md`

Program-level coordination now lives in:

- `docs/development/nero_physical_ai_roadmap.md`
- `docs/development/nero_physical_ai_progress.md`
- `docs/development/component_implementation_map.md`

Do not use this Sprint 1 folder as the cross-sprint source of truth.

## Current Snapshot

| Area | Status | Summary |
| --- | --- | --- |
| Nero URDF/Xacro + meshes | CONFIRMED | `src/agx_arm_sim/agx_arm_description/agx_arm_urdf` is now the canonical in-repo asset tree for Nero/Revo2 URDF/Xacro variants and DAE meshes. |
| Unified MoveIt config | CONFIRMED | `src/agx_arm_moveit` now exposes a single Nero-focused MoveIt package with `none` / `agx_gripper` / `revo2`. |
| MIT controller model path | CONFIRMED | `src/agx_arm_mit_controller` auto-discovers `nero_description.urdf` and `config/nero_gravity_calibration.json`. |
| Isaac/USD assets | PARTIALLY_AVAILABLE | Canonical `src/agx_arm_sim/agx_arm_description` contains a confirmed `nero_gripper_d435.usd`, but the promoted Isaac asset set is still incomplete. |
| OmniHand assets and SDK | PARTIALLY_AVAILABLE | Vendor SDK is now vendored at `vendor/Omnihand-2025-SDK` with Python/C++ APIs, ROS2 node/message packages, and URDF/mesh assets; repo-side `aarch64` socket build/import and isolated probing now work, but live hardware validation is still blocked on device response. |
| AGV/base CAD | MISSING | No STEP, STL, OBJ, or MJCF assets and no AGV description packages were found locally. |

## Relevant Changes Already In Repo

| Commit | Relevance |
| --- | --- |
| `c56826b` | Unified MoveIt configuration into `src/agx_arm_moveit`, added launch/config builder logic, and removed older generated per-arm MoveIt packages. |
| `1c1d90e` | Historical upstream change that updated `agx_arm_urdf` before Sprint 1 detached that dependency into a fixed local asset tree. |
| `b511952` | Added gravity calibration assets, proposal docs, and improved URDF/model metadata for MIT gravity work. |
| `f887363` | Stabilized MIT gravity hold/playback, added the position-hold tool, refined model metadata, and improved default controller profiles. |
| `b4976bc` | Documented the validated Nero MIT workflow. |
| `7e06863` | Latest MIT cleanup for leader-mode retargeting and shutdown behavior. |

## Scope Adjustments From The Roadmap

- Sprint 1 can close the Nero planning/control discovery slice now.
- Sprint 1 can close the repo-side OmniHand discovery and isolated `aarch64` socket build/import slice now, but live hardware validation still depends on an actual hand, adapter, or vendor-supported deployment path.
- Sprint 1 still cannot close AGV discovery without new repositories, CAD files, or hardware-side documentation being added to the workspace.
- Isaac Sim discovery should still be treated as partial because USD coverage is only confirmed for `nero_gripper_d435` plus companion files.
- Sprint 1 resolved the description-package ambiguity by canonicalizing `src/agx_arm_sim/agx_arm_description`.
- The legacy root package `src/agx_arm_description` has now been removed from the workspace, and `agx_arm_urdf` has been detached from submodule management and pruned to the fixed local `nero/` + `revo2/` asset tree.

## Document Map

- `checklist.md`: adjusted Sprint 1 task list with completed items checked.
- `errors_and_fixes.md`: development issues encountered during this implementation pass.
- `open_questions.md`: explicit open product and integration questions left unresolved on purpose.
- `assets/repository_asset_inventory.md`: repo/package inventory and reuse guidance.
- `assets/nero_asset_validation.md`: Nero planning/control/simulation asset assessment.
- `assets/omnihand_asset_validation.md`: current OmniHand gap analysis.
- `assets/agv_cad_inventory.md`: current AGV/base CAD inventory status.
- `control/mit_controller_model_inventory.md`: MIT controller assumptions, model paths, and interfaces.
- `hand/omnihand_sdk_integration.md`: vendored SDK inventory, wrapper-first decision, and the source material for the isolated bring-up plan.

## Inputs Used For This Pass

- `README.md`
- `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md`
- `docs/development/sprint1/control/proposal.md`
- `docs/development/sprint1/control/proposal_urdf_gravity_compensation.md`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/README.md`
- `src/agx_arm_moveit/README.md`
- `src/agx_arm_moveit/config/kinematics.yaml`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_mit_controller/README.md`
- `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py`
- `src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_model.py`
- `src/agx_arm_sim/README.md`
- `src/agx_arm_sim/agx_arm_description/README.md`
- `vendor/Omnihand-2025-SDK/README.md`
- `vendor/Omnihand-2025-SDK/document/zh_cn/API_ROS2.md`
- `pyAgxArm/README.md`
- `pyAgxArm/docs/nero/nero_api.md`
- recent git history in `agx_arm_ros`