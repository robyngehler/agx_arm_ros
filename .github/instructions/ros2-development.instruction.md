---
description: "Use when making ROS2-native decisions in agx_arm_ros such as topics, services, messages, launch surfaces, runtime validation, or value capture."
---

# ROS2 Development

Load this instruction for ROS2-native questions and decisions.

## Core Rules

- keep the public ROS surface agx_arm-centric
- reuse the current owning packages before creating a new ROS2 surface
- when the task is multi-arm or multi-hand bringup, make description and launch surfaces arm-count-aware from the start
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; shared `control/joint_states` is the current hand command flow and is legacy, since the V02 target is one abstract hand command with owner identity, control epoch, and sequence (`docs/sprint_refactor/planning/integration_plan.md`, C5 and 4D)
- keep hand-only diagnostics under `feedback/omnihand/*`
- use standard ROS messages first and extend `agx_arm_msgs` only for repo-owned semantics
- do not treat vendor ROS packages or vendor topics as the public repo contract
- keep `colcon build` on system Python; use the repo-owned wrappers for optional Conda runtime and development shells instead of mixing build and runtime interpreters

## Value Capture

- top-level `docs/*.md` for global navigation and repo-wide checklist, fixes, and open questions
- `docs/control/` for how to run the system (environment, bringup launch/arguments, teach loop)
- `docs/project/` for stable structure, architecture, naming, workflow, and ROS2 practice decisions
- `docs/assets/` for stable runtime and bridge contracts
- `docs/assets/` for stable factual inventories and validation state
- `docs/sprintN/` for sprint-level targets, checklist, errors, open questions, and retained evidence
- `docs/project/roadmap_and_phases.md` for long-term roadmap intent and thematic phases
- `docs/checklist.md` for current sprint status and cross-sprint blockers
- use `docs/sprint_refactor/` for the current top-level sprint routing and `docs/sprint_refactor/planning/` plus `docs/sprint_refactor/reference/` for the detailed working record
- keep `.github/` guidance synchronized with the stable docs it mirrors

## Validation

- run diagnostics on touched files
- prefer `bash ./scripts/colcon_build_system_python.sh --packages-select ...` for touched-package builds when the environment supports it
- run `colcon test --packages-select ...` from a system-Python ROS shell when relevant tests exist
- say explicitly when hardware validation could not be run
- when Python environment drift is part of the issue, use `scripts/colcon_build_system_python.sh` for builds and `scripts/run_in_ros_conda.sh` for optional runtime commands