# ROS2 Development Practices

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-14

## Purpose

This document defines the common ROS2-native development rules for this repo.

Use it when the task changes topics, services, actions, messages, launch surfaces, node behavior, runtime validation, or where a ROS2 decision should be captured.

## Core ROS2 Rules

1. Keep the public ROS surface agx_arm-centric.
2. Reuse the existing owning packages before creating a new ROS2 surface.
3. Prefer shared `control/joint_states` and combined `feedback/joint_states` for coordinated arm-plus-hand flows.
4. Keep hand-only diagnostics and debug topics under `feedback/omnihand/*`.
5. Use standard ROS messages when they already fit; add repo-owned message types in `src/agx_arm_msgs` only when the repo needs semantics that standard types do not provide.
6. Do not treat vendor ROS packages or vendor topic names as the public repo contract.

## Package And Launch Guidance

- extend `src/agx_arm_ctrl` for the current runtime bridge surface during Sprint 2
- extend `src/agx_arm_moveit` for the current planning baseline during Sprint 2
- keep repo-owned OmniHand messages under `src/agx_arm_msgs`
- avoid rename churn while the shared ROS2 contract is still stabilizing
- extend existing launch entry points before adding new parallel bringup surfaces

## Value Capture Rules

Capture the result in the narrowest stable place that matches the decision:

- `docs/project/`: stable repo structure, naming, workflow, and ROS2 practice decisions
- `docs/control/`: stable runtime, bridge, controller, and OmniHand contract decisions
- `docs/assets/`: stable factual inventories and validation state
- `docs/development/nero_physical_ai_roadmap.md`: long-horizon roadmap sequencing only
- `docs/development/nero_physical_ai_progress.md`: cross-sprint status and blockers only
- `docs/development/component_implementation_map.md`: component ownership and document routing only
- `docs/development/sprintN/`: discovery notes, checklist management, errors and fixes, blocked work, and niche implementation details

Keep `.github/` as the concise Copilot-native mirror of these stable docs, not as a separate source of truth.

## Validation Expectations

For ROS2-native changes, prefer this order:

1. editor diagnostics on the touched files
2. `colcon build --packages-select ...` for the touched packages
3. `colcon test --packages-select ...` when tests exist for the touched package
4. `source install/setup.bash` and use `ros2 node list`, `ros2 topic list`, or `ros2 service list` when runtime graph changes need confirmation

Call out explicitly when live hardware validation could not be run.

## Decision Filter

Before adding a ROS2 surface, answer these questions:

1. does the change belong to an existing package that already owns the runtime surface?
2. can a standard ROS message type express the contract cleanly?
3. does the change need to be reflected in `docs/control/` or `docs/project/`?
4. should the detailed evidence stay in a sprint folder instead of another top-level document?