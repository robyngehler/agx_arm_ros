# Sprint 4 Working Notes

## Purpose

This folder tracks Sprint 4 implementation details for the first Duo body plus Nero plus OmniHand system baseline.

It captures the current right-first system bringup path, the documented `src/duo_body_description` staging package, and the remaining work to generalize the still single-arm RViz, MoveIt, and controller-facing surfaces.

The current baseline is a staged description-plus-debug slice, not a finished shared runtime or multi-arm MoveIt baseline.

Program-level coordination lives in:

- `docs/development/nero_physical_ai_roadmap.md`
- `docs/development/nero_physical_ai_progress.md`
- `docs/development/component_implementation_map.md`

Do not use this Sprint 4 folder as the cross-sprint source of truth.

## Current Working Location During Implementation

- `docs/development/sprint4/checklist.md`
- `docs/development/sprint4/errors_and_fixes.md`
- `docs/development/sprint4/open_questions.md`
- `docs/development/sprint4/planning/duo_system_integration_direction.md`

## Current Snapshot

| Area | Status | Summary |
| --- | --- | --- |
| Duo system staging package | CONFIRMED | `src/duo_body_description` is now the documented staging package for Duo body plus configurable arm-hand system assembly, and the package builds cleanly in the active ROS overlay again. |
| Right-side description baseline | CONFIRMED | Prefix-safe Nero arm composition, side-selectable OmniHand attachment, and a description-only `display_duo_system.launch.py` bringup path are landed for `body + right arm + right OmniHand`; on 2026-05-28, `xacro`/`check_urdf` and the headless launch both succeeded for the right-side slice. |
| Left-side mirror | PARTIAL | The left arm and left OmniHand now validate structurally through `xacro`/`check_urdf` and the headless launch path, but the visual RViz frame audit is still pending. |
| Duo-aware MIT RViz debug slice | PARTIAL | The existing single-arm RViz and MIT debug surface can now target staged Duo Xacros via `custom_model`, `custom_model_xacro_args`, and `input_joint_prefix`, but the currently supported slice is still the control-topic path with `follow:=false`; no feedback-side prefix adaptation is landed yet. |
| MoveIt multi-arm generalization | OPEN | `agx_arm_moveit` still exposes the single-arm `nero_arm` baseline plus effector-specific profiles. Prefix-aligned `right_arm`, `left_arm`, and `both_arms` SRDF/config outputs remain open work. |
| Coordinated system demo target | CONFIRMED | The first representative two-arm benchmark is a coordinated Hefeweizen pouring workflow, using one coupled planning group with orchestration above per-arm execution. |

## Achieved Vs Open Boundary

Achieved in the current Sprint 4 baseline:

- `src/duo_body_description` is a documented staging package with a prefix-safe body plus arm plus hand composition path.
- The staged Duo system passed package-scoped `xacro` / `check_urdf` and headless description-only launch validation for the right and left slices.
- The first Duo-aware MIT RViz debug slice exists through the current single-arm launch surface, but only on the RViz control-topic path.

Still open before Sprint 4 exit:

- the graphical RViz frame audit and physical mount measurement for the staged body geometry
- feedback-side prefix adaptation or another shared-state adapter before treating the Duo custom-model path as `follow:=true` capable
- reusable per-arm plus shared runtime launch surfaces in `agx_arm_ctrl` and `agx_arm_mit_controller`
- prefix-aligned MoveIt outputs, including `right_arm`, `left_arm`, `both_arms`, and hand-aware variants

## Scope Adjustments From The Roadmap

- Sprint 4 now starts with `body + right arm + right OmniHand` rather than waiting for a full dual-arm stack from day one.
- The top-level Xacro and bringup surfaces must still remain arm-count-aware from the start so the left side can be mirrored without another structural rewrite.
- Runtime bridge ownership stays in `agx_arm_ctrl`, controller ownership stays in `agx_arm_mit_controller`, and MoveIt ownership stays in `agx_arm_moveit`; `src/duo_body_description` is the current staging surface, not a permanent fork of those responsibilities.
- Isaac and broader simulation work stay sequenced behind this body-mounted system baseline.

## Document Map

- `checklist.md`: Sprint 4 task list and completion state.
- `errors_and_fixes.md`: local issues encountered while pushing the Duo system baseline forward.
- `open_questions.md`: unresolved package, launch, runtime, and planning questions that still affect Sprint 4 exit criteria.
- `planning/duo_system_integration_direction.md`: current direction change, landed first steps, and next implementation slices.

## Inputs Used For This Pass

- `src/duo_body_description/CMakeLists.txt`
- `src/duo_body_description/package.xml`
- `src/duo_body_description/launch/display_duo_system.launch.py`
- `src/duo_body_description/urdf/duo_body.xacro`
- `src/duo_body_description/urdf/duo_system.urdf.xacro`
- `src/duo_body_description/urdf/nero_arm_macro.xacro`
- `src/agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py`
- `src/agx_arm_mit_controller/launch/start_nero_mit_controller.launch.py`
- `src/agx_arm_mit_tools/agx_arm_mit_tools/joint_state_trajectory_bridge.py`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_moveit/launch/move_group.launch.py`
- `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_left_hand.xacro`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_right_hand.xacro`
- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/ros2_development_practices.md`
- `docs/development/sprint3/README.md`
- `docs/development/sprint3/checklist.md`