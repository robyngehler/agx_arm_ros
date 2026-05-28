# Sprint 4 Working Notes

## Purpose

This folder tracks Sprint 4 implementation details for the first Duo body plus Nero plus OmniHand system baseline.

It captures the current right-first system bringup path, the documented `src/duo_body_description` staging package, and the remaining work to generalize the still single-arm RViz, MoveIt, and controller-facing surfaces.

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
| Duo system staging package | STARTED | `src/duo_body_description` is now the documented staging package for Duo body plus configurable arm-hand system assembly. |
| Right-side description baseline | STARTED | Prefix-safe Nero arm composition, side-selectable OmniHand attachment, and a description-only `display_duo_system.launch.py` bringup path are landed for `body + right arm + right OmniHand`. |
| Left-side mirror | PLANNED | The left arm and left OmniHand should be added only after the right-side frame and mount validation is complete. |
| Multi-arm-safe runtime and MoveIt generalization | OPEN | `agx_arm_ctrl`, the MIT-controller RViz path, and `agx_arm_moveit` still assume a single active arm chain in several places. |
| Coordinated system demo target | PLANNED | The first representative two-arm benchmark is a coordinated pouring workflow, with Hefeweizen pouring as the reference example. |

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
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_left_hand.xacro`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/urdf/omnihand_right_hand.xacro`
- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/ros2_development_practices.md`
- `docs/development/sprint3/README.md`
- `docs/development/sprint3/checklist.md`