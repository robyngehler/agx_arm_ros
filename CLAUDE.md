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
2. Read `docs/README.md` and the matching top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, or `docs/open_questions.md` when the task is cross-cutting or documentation-heavy.
3. `AGENTS.md` is imported above and carries the durable rules.
4. Read the single most relevant file from `.claude/rules/` for the task. Path-scoped rules also
   auto-load when you touch matching files.
5. Consult the matching human doc under `docs/control/`, `docs/project/`, or `docs/assets/` when the
  task changes operational workflow, package boundaries, public runtime contracts, or component
  architecture. Naming and ROS2-practice rules live in `.claude/rules/`.
6. Use a skill from `.claude/skills/` only when the task needs a reusable workflow.

## Source Of Truth Order

- repository overview: `README.md`, `README_EN.md`
- global docs hub and repo-wide status: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`
- durable engineering contract: `AGENTS.md`
- how to run the system (bringup launch/arguments, teach loop): `docs/control/`
- human repository structure and architecture: `docs/project/`
- component, runtime, and OmniHand docs: `docs/assets/`
- Claude Code operating model: this file (`CLAUDE.md`)
- targeted rules: `.claude/rules/`
- reusable workflows: `.claude/skills/`
- specialist personas: `.claude/agents/`

Do not load every rule by default. Match context to the task.

## What Lives Where

- routing and context minimization: `.claude/rules/context-routing.md`
- package boundaries and current roles: `.claude/rules/repository-structure.md`
- naming and package-split policy: `.claude/rules/package-naming.md`
- ROS2-native development and value capture: `.claude/rules/ros2-development.md`
- source versus generated asset rules: `.claude/rules/generated-vs-source-assets.md`
- local workflow and promotion order: `.claude/rules/local-agent-workflow.md`
- OmniHand bridge contract and runtime surface: `.claude/rules/omnihand-bridge.md`
- how to run the system (environment + bringup + teach): `docs/control/environment.md`, `docs/control/bringups/launches.md`, `docs/control/bringups/teach_and_run.md`
- global docs hub and repo-wide summaries: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`
- current sprint entrypoint: `docs/sprint_refactor/` (V02 refactor; the canonical
  plan is `docs/sprint_refactor/planning/integration_plan.md`)
- current sprint planning and reference notes: `docs/sprint_refactor/planning/`, `docs/sprint_refactor/reference/`
- `docs/sprint6/` is paused and adapts to the refactor contracts afterwards; its
  step-and-settle notes are superseded by the per-device CAN topology

The files under `.claude/rules/` are the canonical agent-facing rule layer (workflow, naming,
package-split, ROS2 practice). The human docs under `docs/project/` and `docs/assets/` describe repo
structure and component contracts; keep the two consistent.

## Skills

Use these only for task-shaped work:

- `commit-quality` — before every commit, no size threshold
- `test-ladder` — before validating any change, to pick the level and name it
- `omnihand_bridge_work`

## Subagents

Use these when a task benefits from a narrower persona (delegate via the `/agents` UI or by naming them):

- `ros-backend`
- `integration-review`
- `docs-keeper`

## Default Expectations

- keep the public ROS surface agx_arm-centric
- treat `src/duo_body_description` as the Duo body staging package for system assembly while keeping
  `src/agx_arm_sim/agx_arm_description` and `src/agx_arm_moveit` as the long-term canonical surfaces
- keep the OmniHand bridge in `agx_arm_ctrl`
- keep exactly one owner of a device's SDK session at any instant: steady-state
  calls go through that device's serialized worker on a declared priority lane,
  and recovery is the one exception because it takes the session off the worker.
  This holds for the arms today. A hand reaches its SDK from one thread only
  because the bridge spins single-threaded — an accident, not an invariant, and
  with no lane priority, so a stop waits behind ordinary work (phase 2C)
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; shared
  `control/joint_states` is the current hand command flow and is legacy (V02 target: one abstract hand
  command with owner identity, control epoch, and sequence)
- the two arms run different, unflashable firmware (right 1.06 default tier, left 1.11 `NeroFW.V111`);
  anything derived from the protocol is per tier, not per robot model, and a measurement names its arm
- each device owns its own CAN bus: arms on `can_nero_left`/`can_nero_right` (native), hands on
  `hand_left`/`hand_right` (USB-CAN FD adapters); same-side arm and hand motion may run in parallel
  and the shared-bus hand window is a selectable degraded mode
- a hand has two production motion primitives — trajectory execution
  (`<side>_omnihand_controller/follow_joint_trajectory`) and reactive
  contact-seeking motion (the skill controller) — and exactly one owner at a
  time. Both claim `control/omnihand/claim_device` before commanding; the bridge
  is fail-closed, so an unclaimed hand executes nothing
- `control/omnihand/stop` is a cancel-and-hold, not a latching device stop
- use repo-owned `agx_arm_msgs` messages for hand diagnostics and tactile payloads, with statically
  defined fields
- ask before any hardware-touching action; if hardware access is granted, `sudo` is allowed for repo CAN workflows in the intended hardware environment
- use `docs/control/environment.md` and `.claude/rules/ros2-development.md` for environment and ROS2-native decisions
- update stable docs when public contracts change
- keep `.claude/` guidance in sync with the stable docs it mirrors, and with the
  matching `.github/` adapter file — a rule changed in one layer only is a defect
- follow the `commit-quality` skill before every commit, and name the level the
  evidence came from when the change touches CAN, timing, or motion

## Quick Runtime Reference

- build touched packages: `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg_name>`
- run package tests from a system-Python ROS shell: `colcon test --packages-select <pkg_name>`
- Conda runtime command: `bash ./scripts/run_in_ros_conda.sh -- <command>`
- inspect runtime graph: `ros2 node list`, `ros2 topic list`, `ros2 service list`
