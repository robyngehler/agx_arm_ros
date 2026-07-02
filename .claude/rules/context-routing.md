<!-- No `paths` frontmatter: this rule is always loaded because it describes how to load everything else.
     It is the Claude Code mirror of .github/instructions/context-routing.instruction.md. -->

# Context Routing For agx_arm_ros (Claude Code)

*Use when deciding which repository guidance to load for a task. Covers context minimization, when to
read a rule under `.claude/rules/`, and when to reach for a skill or subagent.*

This repository is organized so Claude Code loads only the guidance needed for the current task.

## Loading Order

1. Read `README.md` or `README_EN.md` for repository orientation.
2. Read `docs/README.md` and the matching top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, or `docs/open_questions.md` when the task is cross-cutting or documentation-heavy.
3. `AGENTS.md` is imported by `CLAUDE.md` and carries the durable engineering contract.
4. Read one file from `.claude/rules/` that matches the task. Path-scoped rules also auto-load when you
   touch matching files.
5. Read the matching canonical docs under `docs/control/` (how to run it), `docs/project/`, or
   `docs/assets/` when a bringup, public contract, or workflow decision is involved.
6. Use a skill from `.claude/skills/` only when the task needs a repeatable workflow.
7. Delegate to a subagent in `.claude/agents/` when a narrower persona helps.

## Read A Rule When

- the task is mainly about repository conventions, package boundaries, naming, workflow, or runtime contracts
- the task is about ROS2 topics, services, actions, messages, launch surfaces, runtime validation, or value capture
- you need concise rules for OmniHand bridge behavior, docs promotion, or source-versus-generated assets
- the task changes launch surfaces, messages, or package placement and you need the current repo policy first

## Use A Skill When

- the task is a repeatable OmniHand bridge implementation or refactor workflow
- the task needs a small checklist for docs, messages, launch wiring, and validation in one slice

## Use A Subagent When

- `ros-backend`: focused ROS2 Python/C++ backend implementation or refactor in its own context window
- `integration-review`: end-to-end consistency review across contracts, boundaries, docs, and validation

## Practical Mapping

- package placement and current package roles: `.claude/rules/repository-structure.md`
- naming and package-split rules: `.claude/rules/package-naming.md`
- ROS2-native development and value capture: `.claude/rules/ros2-development.md`
- generated-versus-source decisions: `.claude/rules/generated-vs-source-assets.md`
- local change order and promotion workflow: `.claude/rules/local-agent-workflow.md`
- OmniHand runtime contract and bridge surface: `.claude/rules/omnihand-bridge.md`
- how to run the system (bringup launch/arguments, teach loop): `docs/control/bringup.md`, `docs/control/teach_and_run.md`
- global docs hub and repo-wide summaries: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`

## Context Minimization Rules

- do not load every `.claude/rules/` file by default
- prefer the one dominant rule first
- add the matching `docs/project/` or `docs/assets/` file only when the task changes a stable repo decision
- when editing code, gather only the files needed to finish the current slice safely
- path-scoped rules load automatically when you touch matching files; reach for the rest deliberately
