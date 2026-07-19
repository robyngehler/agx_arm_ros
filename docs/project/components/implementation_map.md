# Component And Implementation Map

status: ACTIVE_BASELINE
last_updated: 2026-07-19

## Purpose

This document answers three questions:

1. which repo surface owns each major component
2. where the stable documentation for that component lives
3. where sprint-level details should be written instead of creating another master document

## Fixed Coordination Docs

Use these stable docs for cross-sprint coordination:

- `docs/project/roadmap_and_phases.md`
- `docs/checklist.md`
- `docs/project/components/implementation_map.md`

For **how to run** any of the components below (launch + arguments), see the operational SoT
`docs/control/bringups/launches.md` and `docs/control/bringups/teach_and_run.md`.

## Component Map

| Component | Canonical code or asset surface | Stable docs | Working notes and deep dives |
| --- | --- | --- | --- |
| Nero description and model assets | `src/agx_arm_sim/agx_arm_description` | `docs/assets/repository_asset_inventory.md`, `docs/assets/nero_asset_validation.md`, `docs/project/repository_structure.md`, `docs/project/architecture.md` | `docs/sprint1/target/README.md` |
| Duo body system assembly and multi-arm staging | `src/duo_body_description` plus shared assets from `src/agx_arm_sim/agx_arm_description` | `docs/project/repository_structure.md`, `.claude/rules/package-naming.md`, `.claude/rules/ros2-development.md` | `docs/sprint3/target/README.md`, `docs/sprint3/checklist.md`, `docs/sprint4/target/README.md`, `docs/sprint4/checklist.md`, `docs/sprint4/evidence/duo_system_integration_direction.md` |
| MoveIt baseline | `src/agx_arm_moveit` | `docs/assets/nero_asset_validation.md`, `docs/project/repository_structure.md`, `docs/project/architecture.md` | `docs/sprint1/target/README.md`, `docs/sprint2/target/README.md`, `docs/sprint2/checklist.md`, `docs/sprint3/target/README.md`, `docs/sprint4/target/README.md` |
| Runtime arm bridge and ROS2 launch surfaces | `src/agx_arm_ctrl` | `.claude/rules/ros2-development.md`, `docs/project/architecture.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md`, `docs/assets/omnihand/omnihand_wrapper_integration_plan.md` | `docs/sprint2/target/README.md`, `docs/sprint2/checklist.md`, `docs/sprint2/open_questions.md`, `docs/sprint4/target/README.md`, `docs/sprint4/open_questions.md` |
| MIT controller and gravity workflow | `src/agx_arm_mit_controller` | `docs/assets/mit_controller/mit_controller_model_inventory.md` | `docs/sprint2/evidence/mit_runtime_history.md`, `docs/sprint1/evidence/mit_soft_control_and_gravity_proposal.md` |
| OmniHand SDK input and repo-owned bridge direction | `vendor/OmniHand-Pro-2025` as upstream input and `src/agx_arm_ctrl` as the repo-owned bridge surface | `docs/assets/omnihand_asset_validation.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md`, `docs/assets/omnihand/omnihand_active_joint_map.md`, `docs/assets/omnihand/omnihand_vendor_sdk_aarch64.md`, `docs/assets/omnihand/omnihand_wrapper_integration_plan.md` | `docs/sprint1/target/README.md` |
| Repo-owned ROS messages | `src/agx_arm_msgs` | `.claude/rules/ros2-development.md`, `docs/project/architecture.md`, `docs/assets/omnihand/omnihand_ros_integration_options.md` | `docs/sprint2/target/README.md`, `docs/sprint2/open_questions.md` |
| CAN transport and bring-up | `scripts/activate_native_can.sh` (native side buses), `scripts/omnihand_canfd_activate.sh` (USB FD), `config/can_interface_roles.json` | `docs/assets/control/basic_control_scripts.md`, `docs/assets/omnihand/omnihand_canfd_setup.md` | `docs/sprint5/evidence/can_transport_decision.md`, `docs/sprint5/errors_and_fixes.md` |
| AGV/base integration | external assets today and a future repo-owned package only after assets exist | `docs/assets/agv_cad_inventory.md`, `docs/project/roadmap_and_phases.md` | `docs/sprint1/open_questions.md` |
| Simulation and Isaac surfaces | `src/agx_arm_sim` plus future promoted docs | `docs/assets/nero_asset_validation.md`, `docs/project/roadmap_and_phases.md` | `docs/sprint1/open_questions.md` and future simulation sprint folders |
| Skills, datasets, and learning | future repo surfaces and promoted docs | `docs/project/roadmap_and_phases.md` until stable docs exist | future sprint folders only |

## Sprint Folder Convention

Use `docs/sprintN/` as the user-facing sprint entrypoints and keep any surviving historical evidence
inside the matching sprint surface.

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

Keep targeted controller notes under the owning historical sprint surface only while they still add
unique evidence, for example `docs/sprint2/evidence/` for the earlier validated workflow lineage and
`docs/sprint1/evidence/` for the early MIT soft-control proposal context.

For the current Duo body direction change, keep cross-sprint intent in the stable roadmap plus checklist docs, use `docs/sprint4/` as the user-facing sprint surface, and treat `docs/sprint4/evidence/` plus package-local notes under `src/duo_body_description/` as supporting historical context rather than the cross-sprint source of truth.