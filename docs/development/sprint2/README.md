# Sprint 2 Working Notes

## Purpose

This folder tracks Sprint 2 implementation details for the "Common Environment and Package Structure Merge" objective in the physical AI roadmap.

It captures the current shared ROS2 contract, package-boundary baseline, launch and runtime understanding, and the remaining work needed before Sprint 2 can hand off cleanly into Sprint 3.

Program-level coordination lives in:

- `docs/development/nero_physical_ai_roadmap.md`
- `docs/development/nero_physical_ai_progress.md`
- `docs/development/component_implementation_map.md`

Do not use this Sprint 2 folder as the cross-sprint source of truth.

## Promoted Stable Sprint 2 Outputs

- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/generated_vs_source_assets.md`
- `docs/project/local_agent_workflow.md`
- `docs/project/ros2_development_practices.md`
- `docs/project/repo_interaction_diagrams.md`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`

## Current Working Location During Implementation

- `docs/development/sprint2/checklist.md`
- `docs/development/sprint2/errors_and_fixes.md`
- `docs/development/sprint2/omnihand_canfd_driver_investigation.md`
- `docs/development/sprint2/open_questions.md`

## Current Snapshot

| Area | Status | Summary |
| --- | --- | --- |
| Runtime package ownership | CONFIRMED | `src/agx_arm_ctrl`, `src/agx_arm_moveit`, `src/agx_arm_sim/agx_arm_description`, and `src/agx_arm_msgs` remain the active Sprint 2 surfaces. |
| Shared ROS2 contract | CONFIRMED | The repo-owned contract is centered on shared `control/joint_states`, combined `feedback/joint_states`, and hand-only debug topics under `feedback/omnihand/*`. |
| OmniHand simulation-first runtime | CONFIRMED | A repo-owned mock `omnihand_bridge` launch and node exist and integrate with the current MoveIt and control path. |
| Launch and file interaction visibility | CONFIRMED | Stable Mermaid diagrams now live in `docs/project/repo_interaction_diagrams.md`. |
| Non-mock OmniHand backend | OPEN | The current bridge is still mock-backed; real backend bring-up and live hardware validation remain open. |

## Scope Adjustments From The Roadmap

- Sprint 2 is still on course as the common environment and contract-hardening phase.
- The repo already has the package-boundary baseline, OmniHand mock bridge skeleton, and simulation-first MoveIt integration expected in this phase.
- The remaining Sprint 2 work is now less about repo discovery and more about runtime hardening, first real backend integration, and validating the hand contract against live behavior.
- Sprint 3 can now start on arm-only MoveIt/MIT validation slices and the minimum naming/description groundwork that do not reopen the shared ROS2 contract, launch ownership, or long-term package-placement decisions.
- Sprint 4 description-only Duo system bringup can now start in parallel as long as it uses the documented `src/duo_body_description` staging package and does not fork the runtime contract away from `agx_arm_ctrl` or `agx_arm_moveit`.

## Document Map

- `checklist.md`: Sprint 2 task list and completion state.
- `errors_and_fixes.md`: issues encountered while making the Sprint 2 baseline understandable and reusable.
- `omnihand_canfd_driver_investigation.md`: verified Jetson-side findings and install paths for real OmniHand CAN FD bring-up.
- `open_questions.md`: unresolved runtime and contract questions that still affect Sprint 2 exit criteria.

## Inputs Used For This Pass

- `src/agx_arm_ctrl/launch/start_single_agx_arm.launch.py`
- `src/agx_arm_ctrl/launch/start_single_agx_arm_moveit.launch.py`
- `src/agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py`
- `src/agx_arm_ctrl/launch/start_omnihand_bridge.launch.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py`
- `src/agx_arm_moveit/launch/demo.launch.py`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_moveit/launch/rsp.launch.py`
- `src/agx_arm_moveit/config/agx_arm.urdf.xacro`
- `src/agx_arm_moveit/config/agx_arm.srdf.xacro`
- `src/agx_arm_moveit/config/agx_arm.ros2_control.xacro`
- `src/agx_arm_moveit/config/initial_positions.yaml`
- `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`