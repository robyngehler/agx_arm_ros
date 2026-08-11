# agx_arm_ros Copilot Instructions

This repository uses `AGENTS.md` as the durable engineering contract and `.github/` as the Copilot-native adapter layer. Keep the always-on context small and load only the instruction or skill that matches the task.

## Operating Model

1. Read `README.md` or `README_EN.md` for repository orientation.
2. Read `docs/README.md` and the matching top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, or `docs/open_questions.md` when the task is cross-cutting or documentation-heavy.
3. Read `AGENTS.md` for the durable rules.
4. Load the single most relevant file from `.github/instructions/`.
5. Consult the matching canonical doc under `docs/control/`, `docs/project/`, or `docs/assets/` when the task changes operational workflow, package boundaries, naming, public runtime contracts, or ROS2-native development decisions.
6. Load a skill from `.github/skills/` only when the task needs a reusable workflow.

## Source Of Truth Order

- repository overview: `README.md`, `README_EN.md`
- global docs hub and repo-wide status: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`
- durable engineering contract: `AGENTS.md`
- stable operational workflow: `docs/control/`
- stable package and workflow policy: `docs/project/`
- stable runtime and OmniHand decisions: `docs/assets/`
- Copilot operating model: this file
- targeted Copilot rules: `.github/instructions/`
- reusable Copilot workflows: `.github/skills/`
- specialist personas: `.github/agents/`

Do not load every instruction by default. Match context to the task.

## What Lives Where

- routing and context minimization: `.github/instructions/context-routing.instruction.md`
- package boundaries: `.github/instructions/repository-structure.instruction.md`
- naming and package-split policy: `.github/instructions/package-naming.instruction.md`
- ROS2-native development and value capture: `.github/instructions/ros2-development.instruction.md`
- source versus generated asset rules: `.github/instructions/generated-vs-source-assets.instruction.md`
- local workflow and promotion order: `.github/instructions/local-agent-workflow.instruction.md`
- OmniHand bridge contract and runtime surface: `.github/instructions/omnihand-bridge.instruction.md`
- current sprint entrypoint: `docs/sprint_refactor/` (V02 refactor; canonical plan `docs/sprint_refactor/planning/integration_plan.md`)
- detailed planning and reference notes: `docs/sprint_refactor/planning/`, `docs/sprint_refactor/reference/`
- `docs/sprint6/` is paused and adapts to the refactor contracts afterwards; its step-and-settle notes are superseded
- global docs hub and repo-wide summaries: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`

These instructions are concise Copilot-native mirrors of the current stable docs under `docs/project/` and `docs/assets/`.

## Skills

Use these only for task-shaped work:

- `commit-quality` — before every commit, no size threshold
- `omnihand_bridge_work`

## Custom Agents

Use these when a task benefits from a narrower persona:

- `ros-backend`
- `integration-review`
- `docs-keeper`

## Default Expectations

- keep the public ROS surface agx_arm-centric
- treat `src/duo_body_description` as the current Duo staging package for system assembly while keeping `src/agx_arm_sim/agx_arm_description` and `src/agx_arm_moveit` as the long-term canonical surfaces
- keep the OmniHand bridge in `agx_arm_ctrl` in the current baseline
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; shared `control/joint_states` is the current hand command flow and is legacy (V02 target: one abstract hand command with owner identity, control epoch, and sequence)
- each device owns its own CAN bus (arms `can0`/`can1` native, hands `can2`/`can3` on USB-CAN FD adapters); same-side arm and hand motion may run in parallel and the shared-bus hand window is a selectable degraded mode
- use repo-owned `agx_arm_msgs` messages for OmniHand-specific diagnostics and tactile payloads
- ask before any hardware-touching action; if hardware access is granted, `sudo` is allowed for repo CAN workflows in the intended hardware environment
- use `.github/instructions/ros2-development.instruction.md` for ROS2-native questions and value-capture decisions
- update stable docs when public contracts change
- keep `.github/` guidance in sync with the matching `.claude/` adapter file — a rule changed in one layer only is a defect
- follow the `commit-quality` skill before every commit, and name the level the evidence came from when the change touches CAN, timing, or motion

## Quick Runtime Reference

- build touched packages: `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg_name>`
- run package tests from a system-Python ROS shell: `colcon test --packages-select <pkg_name>`
- Conda runtime command: `bash ./scripts/run_in_ros_conda.sh -- <command>`
- inspect runtime graph: `ros2 node list`, `ros2 topic list`, `ros2 service list`