# Sprint 4 Working Notes

## Purpose

This folder tracks Sprint 4 implementation details for the first Duo body plus Nero plus OmniHand system baseline.

It captures the current right-first system bringup path, the documented `src/duo_body_description` staging package, the landed prefixed MoveIt and MIT wrapper paths, the first hand-aware per-arm config profiles, the fixed-pose OmniHand gravity payload slice, and the remaining work to finish the graphical and live multi-arm runtime surfaces.

The current baseline is no longer only a description-plus-debug slice: the first shared prefixed MoveIt and MIT wrapper contract is landed, including the per-arm hand-aware config path and a central soft e-stop helper for the shared dual-arm wrappers, but it is not yet a finished dual-hand or live dual-hardware baseline.

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
| Right-side description baseline | CONFIRMED | Prefix-safe Nero arm composition, side-selectable OmniHand attachment, and a description-only `display_duo_system.launch.py` bringup path are landed for `body + right arm + right OmniHand`; on 2026-05-28, `xacro`/`check_urdf` and the headless launch both succeeded for the right-side slice, and the Duo display launch now prefers workspace source RViz/Xacro files during local Sprint 4 iteration. |
| Left-side mirror | PARTIAL | The left arm and left OmniHand now validate structurally through `xacro`/`check_urdf` and the headless launch path, but the visual RViz frame audit is still pending. |
| Duo-aware MIT RViz debug slice | PARTIAL | The existing single-arm RViz and MIT debug surface can now target staged Duo Xacros via `custom_model`, `custom_model_xacro_args`, `input_joint_prefix`, the landed feedback-side JointState prefix adapter, and the custom-model TCP-parent hook; the right-side `tcp_offset` of `0.005` m in X is now preserved on that path, and a 2026-05-31 live launch confirmed the prefixed RViz `follow:=true` path plus adapter wiring without the hardware driver. A new `start_multi_agx_arm_rviz.launch.py` surface now also fans one shared Duo RViz soft-target JointState topic into one MIT controller plus one debug bridge per arm namespace for the first `both_arms` graphical contract. |
| Prefixed per-arm MoveIt path | CONFIRMED | `agx_arm_moveit start_moveit.launch.py` is now the canonical package-local MoveIt entrypoint, with `demo.launch.py` reduced to a compatibility alias. The package-local and combined `agx_arm_ctrl` wrappers accept prefixed custom body models through `robot_name`, `custom_model`, `custom_model_xacro_args`, `input_joint_prefix`, `arm_base_frame`, and `arm_tip_frame`, while the MIT controller strips the same prefix on incoming trajectories. The 2026-06-02 headless right-arm pass reached `You can start planning now!`, and the launch ownership now matches the canonical naming. |
| MoveIt multi-arm generalization | PARTIAL | The first profile selector is landed through `moveit_profile:=right_arm|left_arm|both_arms`. The staged custom-model path no longer depends on manual prefix/frame wiring, and the combined wrapper now starts one MIT controller per declared `arm_instances` entry, keeps the per-arm runtimes namespace-scoped, and merges prefixed feedback back into one MoveIt/RViz stream. The shared config-based wrappers also resolve `execution_profile:=left_hand|right_hand` onto the first hand-aware per-arm path. The current `both_arms` and `duo_arm` contracts remain explicitly arm-only and stay decomposed into one per-arm MIT action server per namespace; live dual-hardware validation and hand-aware dual-arm semantics remain open work. |
| Dual-arm soft e-stop coordination | CONFIRMED | The shared dual-arm MIT wrappers now start `agx_arm_duo_soft_estop`, which exposes one central `/emergency_stop` surface and per-arm `hold_<namespace>` hooks. The current contract fans `cancel_trajectory` plus `hold_current` into each MIT namespace so the steady-hold path is centralized today while future per-arm fixation stays possible without another launch redesign. |
| Gravity payload handling | CONFIRMED | The MIT launch now derives a gravity URDF slice from the staged Duo custom model and keeps an active OmniHand as a fixed-pose payload by freezing the hand joints at zero pose. Gravity compensation still acts only on the seven Nero arm joints; dynamic hand-pose compensation remains future work. |
| Coordinated system demo target | CONFIRMED | The first representative two-arm benchmark is a coordinated Hefeweizen pouring workflow, using one coupled planning group with orchestration above per-arm execution. |

## Achieved Vs Open Boundary

Achieved in the current Sprint 4 baseline:

- `src/duo_body_description` is a documented staging package with a prefix-safe body plus arm plus hand composition path.
- The staged Duo system passed package-scoped `xacro` / `check_urdf` and headless description-only launch validation for the right and left slices.
- The first Duo-aware MIT RViz debug slice exists through the current single-arm launch surface, and the follow-side prefix-adapter plus custom-model TCP-parent hooks are now landed for the current prefixed right-arm path.
- The prefixed single-arm-on-body MoveIt path now starts cleanly against the staged right-arm Duo custom model and reaches a ready `move_group` state with the prefixed feedback topic and MIT action route.
- The first hand-aware config-based launch path is landed through `execution_profile:=left_hand|right_hand`, which resolves the staged Duo model, the per-arm prefix/frame defaults, the generated SRDF hand group, and the per-arm OmniHand bridge selection in one place.
- The shared dual-arm MIT wrappers now centralize the current soft e-stop path through `agx_arm_duo_soft_estop` while keeping per-arm hold hooks available for a later selective-fix policy.
- The derived Duo gravity model now keeps the active OmniHand as a fixed-pose payload so the mounted hand mass is included without changing the seven-DOF MIT controller contract.

Still open before Sprint 4 exit:

- the graphical RViz frame audit and physical mount measurement for the staged body geometry
- hand-aware dual-arm planning and execution semantics beyond the landed per-arm `left_hand` and `right_hand` config profiles
- live dual-hardware validation of the namespace-scoped MIT plus agx_arm_ctrl runtime path
- coordinated-task safety evidence for collision margins, one-arm abort propagation, and staged-scene calibration

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
- `src/agx_arm_mit_tools/agx_arm_mit_tools/duo_soft_estop.py`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_moveit/launch/move_group.launch.py`
- `src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_launch_utils.py`
- `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_left_hand.xacro`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_right_hand.xacro`
- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/ros2_development_practices.md`
- `docs/development/sprint3/README.md`
- `docs/development/sprint3/checklist.md`