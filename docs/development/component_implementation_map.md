# Component And Implementation Map

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-14

## Purpose

This document answers three questions:

1. which repo surface owns each major component
2. where the stable documentation for that component lives
3. where sprint-level details should be written instead of creating another master document

## Fixed Coordination Docs

Use these three top-level development docs for cross-sprint coordination:

- `docs/development/nero_physical_ai_roadmap.md`
- `docs/development/nero_physical_ai_progress.md`
- `docs/development/component_implementation_map.md`

## Component Map

| Component | Canonical code or asset surface | Stable docs | Working notes and deep dives |
| --- | --- | --- | --- |
| Nero description and model assets | `src/agx_arm_sim/agx_arm_description` | `docs/assets/repository_asset_inventory.md`, `docs/assets/nero_asset_validation.md`, `docs/project/repository_structure.md`, `docs/project/repo_interaction_diagrams.md` | `docs/development/sprint1/assets/repository_asset_inventory.md`, `docs/development/sprint1/assets/description_layer.md`, `docs/development/sprint1/assets/nero_asset_validation.md` |
| MoveIt baseline | `src/agx_arm_moveit` | `docs/assets/nero_asset_validation.md`, `docs/project/repository_structure.md`, `docs/project/repo_interaction_diagrams.md` | `docs/development/sprint1/assets/moveit2_schema.md`, `docs/development/sprint2/README.md`, `docs/development/sprint2/checklist.md` |
| Runtime arm bridge and ROS2 launch surfaces | `src/agx_arm_ctrl` | `docs/project/ros2_development_practices.md`, `docs/project/repo_interaction_diagrams.md`, `docs/control/omnihand_ros_integration_options.md`, `docs/control/omnihand_wrapper_integration_plan.md` | `docs/development/sprint2/README.md`, `docs/development/sprint2/checklist.md`, `docs/development/sprint2/open_questions.md` |
| MIT controller and gravity workflow | `src/agx_arm_mit_controller` | `docs/control/mit_controller_model_inventory.md` | `docs/development/mit_trajectory_recording_and_playback.md`, `docs/development/proposal.md`, `docs/development/proposal_urdf_gravity_compensation.md`, `docs/development/sprint1/control/mit_controller_model_inventory.md` |
| OmniHand SDK input and repo-owned bridge direction | `vendor/Omnihand-2025-SDK` as upstream input and `src/agx_arm_ctrl` as the repo-owned bridge surface | `docs/assets/omnihand_asset_validation.md`, `docs/control/omnihand_ros_integration_options.md`, `docs/control/omnihand_phase1_joint_map.md`, `docs/control/omnihand_phase1_run_log.md`, `docs/control/omnihand_wrapper_integration_plan.md` | `docs/development/sprint1/hand/omnihand_sdk_integration.md` |
| Repo-owned ROS messages | `src/agx_arm_msgs` | `docs/project/ros2_development_practices.md`, `docs/project/repo_interaction_diagrams.md`, `docs/control/omnihand_ros_integration_options.md` | `docs/development/sprint2/README.md`, `docs/development/sprint2/open_questions.md` |
| AGV/base integration | external assets today and a future repo-owned package only after assets exist | `docs/assets/agv_cad_inventory.md`, `docs/development/nero_physical_ai_roadmap.md` | `docs/development/sprint1/assets/agv_cad_inventory.md` |
| Simulation and Isaac surfaces | `src/agx_arm_sim` plus future promoted docs | `docs/assets/nero_asset_validation.md`, `docs/development/nero_physical_ai_roadmap.md` | `docs/development/sprint1/assets/dataflow.md` and future simulation sprint folders |
| Skills, datasets, and learning | future repo surfaces and promoted docs | `docs/development/nero_physical_ai_roadmap.md` until stable docs exist | future sprint folders only |

## Sprint Folder Convention

Each sprint folder under `docs/development/sprintN/` should own the sprint-local record instead of adding another top-level coordination doc.

Minimum expected files:

- `README.md`
- `checklist.md`
- `errors_and_fixes.md`
- `open_questions.md`

Optional subfolders should be created only when the sprint needs them, for example:

- `assets/`
- `control/`
- `hand/`
- `planning/`
- `simulation/`
- `learning/`

## Targeted Working Notes Outside Sprint Folders

These top-level development docs are retained because they are targeted slice notes, not cross-sprint coordination docs:

- `docs/development/mit_trajectory_recording_and_playback.md`: validated Nero MIT workflow note
- `docs/development/proposal.md`: historical MIT soft-control proposal kept for design context
- `docs/development/proposal_urdf_gravity_compensation.md`: historical gravity-model proposal kept for design context

If one of these notes becomes the canonical repo answer, promote the stable part into `docs/assets/`, `docs/control/`, or `docs/project/` and keep the working note scoped to its original slice.