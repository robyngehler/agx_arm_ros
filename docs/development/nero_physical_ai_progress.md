# Nero Physical AI Progress Monitor

status: SPRINT6_ACTIVE_COORDINATED_TASKS
last_updated: 2026-07-02

## Purpose

This is the single cross-sprint progress tracker for the common ROS2 physical AI roadmap.

- `docs/development/nero_physical_ai_roadmap.md`: long-term roadmap intent and thematic phases
- `docs/development/nero_physical_ai_progress.md`: current status, active focus, and blockers
- `docs/development/component_implementation_map.md`: component ownership, code locations, doc routing

Note on numbering: the **execution sprints** (`docs/development/sprintN/`) are the work iterations and
are the source of truth for "where we are". The roadmap's thematic phases (Section 5 below) are the
long-term plan. Execution sprints 1–4 delivered roadmap phases 1–4; execution **sprint 5 is a
runtime/CAN-transport stabilization iteration** that sits ahead of the roadmap's thematic phase 5
(AGV/base).

## Status Legend

- `COMPLETE`: delivered locally except for explicitly recorded carryover
- `ACTIVE`: current implementation focus
- `PLANNED`: sequenced but not the current slice
- `EXTERNAL`: waiting on hardware, vendor, or assets outside this workspace

## Current Snapshot (execution sprints)

| Sprint | Area | Status | Summary |
| --- | --- | --- | --- |
| 1 | Asset audit and model baseline | COMPLETE | Asset inventories and validation landed; remaining gaps (AGV CAD, broader USD) are external. |
| 2 | Common environment and OmniHand bridge | COMPLETE | Shared ROS2 semantics, package boundaries, and the repo-owned OmniHand bridge with the **vendor SDK backend** (active 10-joint command, status, tactile) are in place. |
| 3 | Nero planning and control hardening | COMPLETE | TRAC-IK + OMPL planning validated; `/compute_ik`, MoveIt profiles, and the MIT control path are working. |
| 4 | Duo body + OmniHand system baseline | COMPLETE | Shared macro/xacro-driven URDF, dynamic SRDF, and arm-count-aware MoveIt (`right_arm`/`left_arm`/`both_arms`) are landed. **OMPL + TRAC-IK planning succeeds for the Duo groups**, and a **joint arm + OmniHand bringup with small live movements** ran via `start_agx_arm_components.launch.py`. |
| 5 | CAN transport + arm-plus-hand | COMPLETE | Native `mttcan` CAN FD side buses and the pinned `pyAgxArm` runtime are now the repo baseline. Shared arm+hand bus behavior remains documented as a carried operational caveat, not the owning sprint target. |
| 6 | Coordinated tasks + skill layer | ACTIVE | Coordinator, dual-arm teach flow, and OmniHand skill abstractions are the current focus. Shared bus validation remains a dependency where the live workflow still couples arm and hand on one side bus. |

## Active Sprint Focus (execution sprint 6)

- prepare the **first demo task: coordinated Hefeweizen pouring** — decompose into per-arm recorded
  or planned trajectories above per-arm MIT execution, using the existing `both_arms` planning plus
  the OmniHand grasp; this is the roadmap's pouring reference task (Section 5, Sprint 4 output)
- keep the shared-bus runtime caveat explicit: shared arm+hand bringup may still need a workflow-
  specific native CAN profile (`ONE_SHOT=off`, lower MIT control rate) even though the arm-only
  native CAN baseline is stable — see `docs/control/teach_and_run.md`
- keep validating coordinator dispatch, dual-arm teach capture, and the semantic hand-skill layer on
  top of the current MoveIt + MIT + OmniHand bridge baseline
- keep the public ROS2 surface agx_arm-centric; keep `both_arms` execution per-arm at the MIT action
  boundary until a coordinated fault/safety model is documented
- keep the OmniHand bridge in `agx_arm_ctrl`; keep `src/duo_body_description` as the staging package
  and promote stable Duo outputs back into the canonical `agx_arm_*` packages

## Cross-Sprint Blockers / Open

- **Resolved:** the Duo ENOBUFS bus stalls (native CAN + `one-shot`), the control-layer drift
  (`pyAgxArm` submodule pin), and the native CAN-FD/BRS path for the OmniHand (5 Mbit transceiver).
- arm + hand on one shared side bus is feasible but the **bus-load budget is not yet confirmed** for
  sustained coordinated motion
- **live multi-cycle** plan&execute and a coordinated-task fault model (one-arm abort, stop
  propagation, collision-scene calibration) still need hardware evidence
- the first **demo task (Hefeweizen pouring)** is not yet defined as an executable slice
- AGV/base CAD, mounting data, and coordinate definitions are still missing locally (external)
- broader Isaac/USD asset coverage is still incomplete (external)

## Update Rules

- update this file when sprint status, active focus, or blockers change
- update `docs/development/nero_physical_ai_roadmap.md` only when roadmap sequencing or phase intent changes
- update `docs/development/component_implementation_map.md` when ownership or document routing changes
- keep sprint evidence in `docs/development/sprintN/`
- promote stable outputs into `docs/assets/` (components/runtime) or `docs/project/` (repo structure)
