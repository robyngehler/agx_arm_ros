# agx_arm_ros — Claude Code Project Instructions

@AGENTS.md

<!-- maintainer note: this is the Claude Code mirror of the former .github/copilot-instructions.md.
     Keep it in sync with docs/project/ (human repo structure) and docs/assets/ (component docs). Claude Code reads CLAUDE.md, not AGENTS.md,
     so AGENTS.md is pulled in via the @AGENTS.md import above. -->

This repository keeps `AGENTS.md` as the durable engineering contract and `.claude/` as the Claude Code
adapter layer. Keep the always-on context small and load only the rule, skill, or subagent that matches
the task.

## Operating Model

1. Read `README.md` or `README_EN.md` for repository orientation.
2. `AGENTS.md` is imported above and carries the durable rules.
3. Read the single most relevant file from `.claude/rules/` for the task. Path-scoped rules also
   auto-load when you touch matching files.
4. Consult the matching human doc under `docs/project/` (repo structure) or `docs/assets/` (component
   and runtime contracts) when the task changes package boundaries, public runtime contracts, or
   component architecture. Naming and ROS2-practice rules live in `.claude/rules/`.
5. Use a skill from `.claude/skills/` only when the task needs a reusable workflow.

## Source Of Truth Order

- repository overview: `README.md`, `README_EN.md`
- durable engineering contract: `AGENTS.md`
- human repository structure and architecture: `docs/project/`
- component, runtime, and OmniHand docs: `docs/assets/`
- Claude Code operating model: this file (`CLAUDE.md`)
- targeted rules: `.claude/rules/`
- reusable workflows: `.claude/skills/`
- specialist personas: `.claude/agents/`

Do not load every rule by default. Match context to the task.

## What Lives Where

- routing and context minimization: `.claude/rules/context-routing.md`
- Sprint 2 package boundaries: `.claude/rules/repository-structure.md`
- naming and package-split policy: `.claude/rules/package-naming.md`
- ROS2-native development and value capture: `.claude/rules/ros2-development.md`
- source versus generated asset rules: `.claude/rules/generated-vs-source-assets.md`
- local workflow and promotion order: `.claude/rules/local-agent-workflow.md`
- OmniHand bridge contract and runtime surface: `.claude/rules/omnihand-bridge.md`
- current working notes: `docs/development/sprint5/` (CAN transport & control-layer); Duo body baseline in `docs/development/sprint4/`

The files under `.claude/rules/` are the canonical agent-facing rule layer (workflow, naming,
package-split, ROS2 practice). The human docs under `docs/project/` and `docs/assets/` describe repo
structure and component contracts; keep the two consistent.

## Skills

Use these only for task-shaped work:

- `omnihand_bridge_work`

## Subagents

Use these when a task benefits from a narrower persona (delegate via the `/agents` UI or by naming them):

- `ros-backend`
- `integration-review`

## Default Expectations

- keep the public ROS surface agx_arm-centric
- treat `src/duo_body_description` as the current Sprint 3 and Sprint 4 staging package for Duo body
  system assembly while keeping `src/agx_arm_sim/agx_arm_description` and `src/agx_arm_moveit` as the
  long-term canonical surfaces
- keep the OmniHand bridge in `agx_arm_ctrl` during Sprint 2
- prefer shared `control/joint_states` and combined `feedback/joint_states` for coordinated
  arm-plus-hand flows
- use repo-owned `agx_arm_msgs` messages for OmniHand-specific diagnostics and tactile payloads
- use `.claude/rules/ros2-development.md` for ROS2-native questions and value-capture decisions
- update stable docs when public contracts change
- keep `.claude/` guidance in sync with the stable docs it mirrors

## Quick Runtime Reference

- build touched packages: `colcon build --packages-select <pkg_name>`
- run package tests: `colcon test --packages-select <pkg_name>`
- source overlay: `source install/setup.bash`
- inspect runtime graph: `ros2 node list`, `ros2 topic list`, `ros2 service list`
