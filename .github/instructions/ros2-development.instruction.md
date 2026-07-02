---
description: "Use when making ROS2-native decisions in agx_arm_ros such as topics, services, messages, launch surfaces, runtime validation, or value capture."
---

# ROS2 Development

Load this instruction for ROS2-native questions and decisions.

## Core Rules

- keep the public ROS surface agx_arm-centric
- reuse the current owning packages before creating a new ROS2 surface
- when the task is multi-arm or multi-hand bringup, make description and launch surfaces arm-count-aware from the start
- prefer shared `control/joint_states` and combined `feedback/joint_states` for coordinated arm-plus-hand flows
- keep hand-only diagnostics under `feedback/omnihand/*`
- use standard ROS messages first and extend `agx_arm_msgs` only for repo-owned semantics
- do not treat vendor ROS packages or vendor topics as the public repo contract
- keep `colcon build` on system Python; use the repo-owned wrappers for optional Conda runtime and development shells instead of mixing build and runtime interpreters

## Value Capture

- top-level `docs/*.md` for global navigation and repo-wide checklist, fixes, and open questions
- `docs/project/` for stable structure, naming, workflow, and ROS2 practice decisions
- `docs/assets/` for stable runtime and bridge contracts
- `docs/assets/` for stable factual inventories and validation state
- top-level `docs/development/` docs only for roadmap, progress, and component routing
- `docs/development/sprintN/` for discovery, checklist, error/fix, and blocker tracking
- use `docs/development/sprint4/` for the current Duo body system implementation record
- keep `.github/` guidance synchronized with the stable docs it mirrors

## Validation

- run diagnostics on touched files
- prefer `colcon build --packages-select ...` for touched packages
- run `colcon test --packages-select ...` when relevant tests exist
- say explicitly when hardware validation could not be run
- when Python environment drift is part of the issue, use `scripts/colcon_build_system_python.sh` for builds and `scripts/run_in_ros_conda.sh` for optional runtime commands