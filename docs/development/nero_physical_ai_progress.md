# Nero Physical AI Progress Monitor

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-17

## Purpose

This is the single cross-sprint progress tracker for the common ROS2 physical AI roadmap.

Use the top-level development docs like this:

- `docs/development/nero_physical_ai_roadmap.md`: roadmap intent, phases, and sprint sequencing
- `docs/development/nero_physical_ai_progress.md`: current status, active focus, and blockers
- `docs/development/component_implementation_map.md`: component ownership, code locations, and document routing

## Status Legend

- `COMPLETE`: finished locally except for explicitly recorded carryover items
- `ACTIVE`: current implementation and coordination focus
- `PLANNED`: sequenced but not yet the primary work slice
- `BLOCKED`: waiting on another repo-local dependency
- `EXTERNAL`: waiting on hardware, vendor support, or assets outside this workspace

## Current Snapshot

| Area | Status | Summary |
| --- | --- | --- |
| Sprint 1 asset and repo baseline | COMPLETE | Stable outputs were promoted into `docs/assets/` and `docs/control/`; the remaining gaps are external. |
| Sprint 2 common environment and package structure merge | ACTIVE | Shared ROS2 semantics, package boundaries, Sprint 2 working notes, and stable interaction diagrams are now in place; the remaining gate is real-backend and runtime validation work. |
| Sprint 3 Nero planning and control hardening | PLANNED | Hardening follows once the Sprint 2 repo and ROS2 contract stops moving. |
| Sprint 4 Nero plus OmniHand common baseline | PLANNED | Depends on the shared bridge boundary and normalized ROS2 contract from Sprint 2. |
| Sprint 5 and later | PLANNED / EXTERNAL | Later phases stay roadmap items until AGV assets, broader simulation assets, and more hardware validation exist. |

## Roadmap Sprint Status

| Sprint | Focus | Status | Current gate |
| --- | --- | --- | --- |
| 1 | Asset audit and model baseline | COMPLETE | Closed locally except for AGV assets, broader USD coverage, and live OmniHand hardware validation. |
| 2 | Common environment and package structure merge | ACTIVE | Package boundaries, simulation-first OmniHand integration, and stable repo interaction docs are in place; the remaining gate is the first non-mock backend and validated runtime behavior. |
| 3 | Nero planning and control baseline hardening | PLANNED | Start once Sprint 2 contract work is stable enough to validate the current planning and control path as-is. |
| 4 | Nero plus OmniHand common baseline | PLANNED | Start after the shared bridge boundary and normalized hand semantics are stable. |
| 5 | Static AGV/base integration | EXTERNAL | Blocked on AGV/base CAD, mounting references, and coordinate definitions not present in this workspace. |
| 6 | Combined collision and planning validation | PLANNED | Wait for hand and base geometry to stabilize first. |
| 7 | Isaac Sim digital twin integration | PLANNED | Wait for broader model-variant and USD coverage. |
| 8 | Hardware-in-the-loop and replay | PLANNED | Wait for the simulation and runtime baselines to be reproducible. |
| 9 | Grasping MVP | PLANNED | Wait for the arm-plus-hand baseline and early perception setup. |
| 10 | Dexterous grasping model evaluation | PLANNED | Wait for a measurable grasping baseline. |
| 11 | Scripted skill library | PLANNED | Wait for deterministic planning and execution slices to stabilize. |
| 12 | Demonstration data pipeline | PLANNED | Wait for skill and logging schemas to stop moving. |
| 13 | Isaac Lab task development | PLANNED | Wait for simulation and dataset baselines. |
| 14 | Diffusion-based skill policies | PLANNED | Wait for task and demonstration pipelines. |
| 15 | GPU-accelerated manipulation evaluation | PLANNED | Wait for the collision, perception, and model stack to mature. |
| 16 | VLA and Physical AI policy evaluation | PLANNED | Wait for deterministic skills and safety boundaries. |
| 17 | Cosmos and synthetic data workflow | PLANNED | Wait for simulation replay and dataset maturity. |

## Active Sprint Focus

- keep the public ROS2 surface agx_arm-centric while Sprint 2 settles
- keep stable repo policy in `docs/project/` and runtime contracts in `docs/control/`
- keep the launch, runtime, file-composition, and config-dataflow diagrams aligned with the real code paths
- keep only three cross-sprint coordination docs at the top of `docs/development/`
- put discovery, checklist, error/fix, and niche implementation details into sprint folders
- keep the wakeword-triggered demo and recording/playback helper documented as adjacent tooling in `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md`; it supports later interaction work but is not a roadmap gate by itself

## Cross-Sprint Blockers

- AGV/base CAD, mounting data, and coordinate definitions are still missing locally
- OmniHand live hardware validation still depends on a responsive device path and adapter
- broader Isaac/USD asset coverage is still incomplete

## Update Rules

- update this file when sprint status, active focus, or blockers change
- update `docs/development/nero_physical_ai_roadmap.md` only when roadmap sequencing or phase intent changes
- update `docs/development/component_implementation_map.md` when ownership or document routing changes
- keep sprint evidence in `docs/development/sprintN/`
- promote stable outputs into `docs/assets/`, `docs/control/`, or `docs/project/`