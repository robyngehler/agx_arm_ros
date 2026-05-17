# agx_arm_ros Copilot Instructions

This repository uses `AGENTS.md` as the durable engineering contract and `.github/` as the Copilot-native adapter layer. Keep the always-on context small and load only the instruction or skill that matches the task.

## Operating Model

1. Read `README.md` or `README_EN.md` for repository orientation.
2. Read `AGENTS.md` for the durable rules.
3. Load the single most relevant file from `.github/instructions/`.
4. Consult the matching canonical doc under `docs/project/` or `docs/control/` when the task changes package boundaries, naming, public runtime contracts, or ROS2-native development decisions.
5. Load a skill from `.github/skills/` only when the task needs a reusable workflow.

## Source Of Truth Order

- repository overview: `README.md`, `README_EN.md`
- durable engineering contract: `AGENTS.md`
- stable package and workflow policy: `docs/project/`
- stable runtime and OmniHand decisions: `docs/control/`
- Copilot operating model: this file
- targeted Copilot rules: `.github/instructions/`
- reusable Copilot workflows: `.github/skills/`
- specialist personas: `.github/agents/`

Do not load every instruction by default. Match context to the task.

## What Lives Where

- routing and context minimization: `.github/instructions/context-routing.instruction.md`
- Sprint 2 package boundaries: `.github/instructions/repository-structure.instruction.md`
- naming and package-split policy: `.github/instructions/package-naming.instruction.md`
- ROS2-native development and value capture: `.github/instructions/ros2-development.instruction.md`
- source versus generated asset rules: `.github/instructions/generated-vs-source-assets.instruction.md`
- local workflow and promotion order: `.github/instructions/local-agent-workflow.instruction.md`
- OmniHand bridge contract and runtime surface: `.github/instructions/omnihand-bridge.instruction.md`

These instructions are concise Copilot-native mirrors of the current stable docs under `docs/project/` and `docs/control/`.

## Skills

Use these only for task-shaped work:

- `omnihand_bridge_work`

## Custom Agents

Use these when a task benefits from a narrower persona:

- `ros-backend`
- `integration-review`

## Default Expectations

- keep the public ROS surface agx_arm-centric
- keep the OmniHand bridge in `agx_arm_ctrl` during Sprint 2
- prefer shared `control/joint_states` and combined `feedback/joint_states` for coordinated arm-plus-hand flows
- use repo-owned `agx_arm_msgs` messages for OmniHand-specific diagnostics and tactile payloads
- use `.github/instructions/ros2-development.instruction.md` for ROS2-native questions and value-capture decisions
- update stable docs when public contracts change

## Quick Runtime Reference

- build touched packages: `colcon build --packages-select <pkg_name>`
- run package tests: `colcon test --packages-select <pkg_name>`
- source overlay: `source install/setup.bash`
- inspect runtime graph: `ros2 node list`, `ros2 topic list`, `ros2 service list`