---
description: "Use when deciding which repository guidance to load for a task in agx_arm_ros. Covers context minimization, when to use repo policy docs, and when to use Copilot instructions versus skills."
---

# Context Routing For agx_arm_ros

This repository is intentionally organized so Copilot can load only the guidance needed for the current task.

## Routing Order

1. Read `README.md` or `README_EN.md` for repository orientation.
2. Read `docs/README.md` and the matching top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, or `docs/open_questions.md` when the task is cross-cutting or documentation-heavy.
3. Read `AGENTS.md` for the durable engineering contract.
4. Load one file from `.github/instructions/` that matches the task.
5. Read the matching canonical docs under `docs/control/`, `docs/project/`, or `docs/assets/` when a public contract or workflow decision is involved.
6. Load a skill from `.github/skills/` only when the task needs a repeatable workflow.

## Choose Instructions When

- the task is mainly about repository conventions, package boundaries, naming, workflow, or runtime contracts
- the task is about ROS2 topics, services, actions, messages, launch surfaces, runtime validation, or value capture
- you need concise rules for OmniHand bridge behavior, docs promotion, or source-versus-generated assets
- the task changes launch surfaces, messages, or package placement and you need the current repo policy first

## Choose Skills When

- `commit-quality`: before every commit, amend, or history rewrite — no size threshold
- `test-ladder`: before validating a change, to choose L1/L2/L3 and state the level the evidence came from
- `omnihand_bridge_work`: a repeatable OmniHand bridge implementation or refactor workflow
- the task needs a small checklist for docs, messages, launch wiring, and validation in one slice

## Choose A Custom Agent When

- `ros-backend`: focused ROS2 Python/C++ backend implementation or refactor in its own context window
- `integration-review`: end-to-end consistency review across contracts, boundaries, docs, and validation
- `docs-keeper`: reconcile the docs tree with the actual code state at a sprint boundary or after a large merge, including agent-layer mirror drift

## Practical Mapping

- package placement and current package roles: `repository-structure.instruction.md`
- naming and package-split rules: `package-naming.instruction.md`
- ROS2-native development and value capture: `ros2-development.instruction.md`
- generated-versus-source decisions: `generated-vs-source-assets.instruction.md`
- local change order and promotion workflow: `local-agent-workflow.instruction.md`
- OmniHand runtime contract and bridge surface: `omnihand-bridge.instruction.md`
- operational workflow: `docs/control/environment.md`, `docs/control/bringups/launches.md`, `docs/control/bringups/teach_and_run.md`
- stable architecture and component ownership: `docs/project/architecture.md`, `docs/project/components/README.md`
- current sprint entrypoint: `docs/sprint_refactor/` with detailed planning and reference notes under `docs/sprint_refactor/planning/` and `docs/sprint_refactor/reference/`; `planning/integration_plan.md` is the canonical plan and carries the binding constraints C1-C6
- `docs/sprint6/` is paused until the refactor contracts land; treat its step-and-settle and hand-window notes as superseded
- global docs hub and repo-wide summaries: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`

## Context Minimization Rules

- do not load every `.github/instructions/` file by default
- prefer the one dominant instruction first
- add the matching `docs/project/` or `docs/assets/` file only when the task changes a stable repo decision
- when editing code, gather only the files needed to finish the current slice safely