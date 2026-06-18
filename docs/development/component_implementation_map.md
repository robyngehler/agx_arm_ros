# Component And Implementation Map

status: ACTIVE_SPRINT5_CAN_TRANSPORT
last_updated: 2026-06-15

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
| Duo body system assembly and multi-arm staging | `src/duo_body_description` plus shared assets from `src/agx_arm_sim/agx_arm_description` | `docs/project/repository_structure.md`, `.claude/rules/package-naming.md`, `.claude/rules/ros2-development.md` | `docs/development/sprint3/README.md`, `docs/development/sprint3/checklist.md`, `docs/development/sprint4/README.md`, `docs/development/sprint4/checklist.md`, `docs/development/sprint4/planning/duo_system_integration_direction.md` |
| MoveIt baseline | `src/agx_arm_moveit` | `docs/assets/nero_asset_validation.md`, `docs/project/repository_structure.md`, `docs/project/repo_interaction_diagrams.md` | `docs/development/sprint1/assets/moveit2_schema.md`, `docs/development/sprint2/README.md`, `docs/development/sprint2/checklist.md`, `docs/development/sprint3/README.md`, `docs/development/sprint4/README.md` |
| Runtime arm bridge and ROS2 launch surfaces | `src/agx_arm_ctrl` | `.claude/rules/ros2-development.md`, `docs/project/repo_interaction_diagrams.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md`, `docs/assets/omnihand/omnihand_wrapper_integration_plan.md` | `docs/development/sprint2/README.md`, `docs/development/sprint2/checklist.md`, `docs/development/sprint2/open_questions.md`, `docs/development/sprint4/README.md`, `docs/development/sprint4/open_questions.md` |
| MIT controller and gravity workflow | `src/agx_arm_mit_controller` | `docs/assets/mit_controller/mit_controller_model_inventory.md` | `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md`, `docs/development/sprint1/control/proposal.md`, `docs/development/sprint1/control/proposal_urdf_gravity_compensation.md`, `docs/development/sprint1/control/mit_controller_model_inventory.md` |
| OmniHand SDK input and repo-owned bridge direction | `vendor/Omnihand-2025-SDK` as upstream input and `src/agx_arm_ctrl` as the repo-owned bridge surface | `docs/assets/omnihand_asset_validation.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md`, `docs/assets/omnihand/omnihand_phase1_joint_map.md`, `docs/assets/omnihand/omnihand_phase1_run_log.md`, `docs/assets/omnihand/omnihand_wrapper_integration_plan.md` | `docs/development/sprint1/hand/omnihand_sdk_integration.md` |
| Repo-owned ROS messages | `src/agx_arm_msgs` | `.claude/rules/ros2-development.md`, `docs/project/repo_interaction_diagrams.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md` | `docs/development/sprint2/README.md`, `docs/development/sprint2/open_questions.md` |
| CAN transport and bring-up | `scripts/activate_native_can.sh` (native side buses), `scripts/omnihand_canfd_activate.sh` (USB FD), `config/can_interface_roles.json` | `docs/assets/control/basic_control_scripts.md`, `docs/assets/omnihand/omnihand_canfd_setup.md` | `docs/development/sprint5/planning/can_transport_decision.md`, `docs/development/sprint5/errors_and_fixes.md` |
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

Keep targeted controller notes under the owning sprint folder, for example `docs/development/sprint2/control/` for the current validated workflow and `docs/development/sprint1/control/` for historical proposal context.

For the current Duo body direction change, keep cross-sprint intent in the three top-level development docs, keep the implementation record in `docs/development/sprint4/`, and treat package-local notes under `src/duo_body_description/` as supporting context rather than the cross-sprint source of truth.