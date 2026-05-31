# Nero Physical AI Progress Monitor

status: ACTIVE_SPRINT2_RUNTIME_PLUS_SPRINT3_ARM_HARDENING_PLUS_SPRINT4_DUO_BRINGUP
last_updated: 2026-05-28

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
| Sprint 2 common environment and package structure merge | ACTIVE | Shared ROS2 semantics, package boundaries, Sprint 2 working notes, and stable interaction diagrams are in place; the remaining gate is still the first non-mock OmniHand backend plus runtime validation on a real hand path. |
| Sprint 3 Nero planning and control hardening | ACTIVE | TRAC-IK, six-profile MoveIt bringup, a live `/compute_ik` call, a repo-owned OMPL pose-plan smoke test, and a non-hardware MIT trajectory audit are now validated locally; the remaining gates are broader planning-path evidence and smaller-scope crash isolation beyond the still-reproduced `move_group` teardown fault. |
| Sprint 4 Duo body plus OmniHand system baseline | ACTIVE | `src/duo_body_description` is now the documented staging package for the body-mounted system slice; ROS-native `xacro`/`check_urdf`, headless bringup, and a first control-topic Duo-aware MIT RViz debug path are validated locally, and the remaining gates are RViz/physical mount review plus the generalization of shared single-arm control and MoveIt surfaces. |
| Sprint 5 and later | PLANNED / EXTERNAL | Later phases stay roadmap items until AGV assets, broader simulation assets, and more hardware validation exist. |

## Roadmap Sprint Status

| Sprint | Focus | Status | Current gate |
| --- | --- | --- | --- |
| 1 | Asset audit and model baseline | COMPLETE | Closed locally except for AGV assets, broader USD coverage, and live OmniHand hardware validation. |
| 2 | Common environment and package structure merge | ACTIVE | Package boundaries, simulation-first OmniHand integration, and stable repo interaction docs are in place; the remaining gate is the first non-mock backend plus validated runtime behavior on a real hand path. |
| 3 | Nero planning and control baseline hardening | ACTIVE | Proceed on full-profile execution-path evidence and a smaller reproducible crash-isolation path now that a representative OMPL pose plan is verified and the OMPL-only teardown crash still reproduces. |
| 4 | Duo body plus OmniHand system baseline | ACTIVE | RViz and physical mount review must complete first, then the current single-arm RViz, MoveIt, and controller-facing surfaces must be generalized in place from the new Duo-aware debug baseline. |
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

- keep the public ROS2 surface agx_arm-centric while Sprint 2 runtime work settles
- keep Sprint 2 scoped to OmniHand backend, SDK smoke-test, and runtime-graph validation rather than reopening package placement
- run Sprint 3 on Nero arm MoveIt/MIT hardening plus only the minimal naming and description groundwork needed by the Duo system slice
- treat `src/duo_body_description` as the documented Sprint 3 and Sprint 4 staging package for body-mounted system bringup, not as a final long-term replacement for `src/agx_arm_sim/agx_arm_description`
- make description and bringup surfaces arm-count-aware from the start, with `body + right arm + right OmniHand` as the current executable target and the left side as the immediate follow-on
- treat sim-only MoveIt profile sweeps across effectors as valid Sprint 3 evidence before real-arm collision-checked execution
- use `src/agx_arm_moveit/scripts/plan_pose_smoke_test.py` as the current repo-owned representative near-home OMPL pose-planning check for Sprint 3
- keep `nero_arm` as the monolithic active planning group in the current single-arm MoveIt surface while Sprint 4 documents the future `right_arm`, `left_arm`, and `both_arms` split for the Duo system
- keep `nero_tool0` as the canonical Nero flange alias and `tcp_link` as the distinct TCP frame instead of collapsing those semantics together
- use TRAC-IK as the current MoveIt IK baseline and source the external `~/workspace/trac_ik_ws` overlay on Humble / Jetson when the apt package is unavailable
- use the new `custom_model`, `custom_model_xacro_args`, and `input_joint_prefix` hooks in the current RViz debug path as the first Duo-aware controller-facing slice on the control-topic side; feedback-side prefix adaptation and multi-arm MoveIt outputs remain open work
- keep Isaac and broader simulation work sequenced after the first validated Duo body system baseline
- keep stable repo policy in `docs/project/` and runtime contracts in `docs/control/`
- keep only three cross-sprint coordination docs at the top of `docs/development/`
- put discovery, checklist, error/fix, and niche implementation details into sprint folders
- keep the current Duo body integration record in `docs/development/sprint4/`
- keep the wakeword-triggered demo and recording/playback helper documented as adjacent tooling in `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md`; it supports later interaction work but is not a roadmap gate by itself

## Cross-Sprint Blockers

- AGV/base CAD, mounting data, and coordinate definitions are still missing locally
- OmniHand live hardware validation still depends on a responsive device path and adapter
- broader Isaac/USD asset coverage is still incomplete
- current RViz, MoveIt, and controller-facing launch surfaces are still primarily single-arm oriented
- the Duo custom-model RViz path still lacks feedback-side prefix adaptation for a clean `follow:=true` contract
- the Duo system still needs a graphical RViz pass and physical body measurements to confirm the staged mount transforms
- the current `move_group` teardown crash still reproduces on this Humble/aarch64 host even when the launch is reduced to `planning_pipelines:=ompl`

## Update Rules

- update this file when sprint status, active focus, or blockers change
- update `docs/development/nero_physical_ai_roadmap.md` only when roadmap sequencing or phase intent changes
- update `docs/development/component_implementation_map.md` when ownership or document routing changes
- keep sprint evidence in `docs/development/sprintN/`
- promote stable outputs into `docs/assets/`, `docs/control/`, or `docs/project/`