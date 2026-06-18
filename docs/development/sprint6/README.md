# Sprint 6 — Coordinated Hefeweizen Pour (skills + coordinator)

**Target.** The first executable coordinated dual-arm + dual-hand task: pour a Hefeweizen. This
sprint builds the **orchestration and hand-skill layer** on top of the existing arm/hand control:

1. An **OmniHand skill controller** that turns *semantic* skills (`grasp_glass_until_contact`,
   `release_glass`, `open_hand`, …) into vendor-SDK gestures/presets and confirms them with
   **tactile feedback**. The public layer only ever sees the semantic `skill_name`; the backend
   owns the `skill_name → vendor gesture/preset/joint` mapping. Behavior (hold after grasp, cancel,
   timeout, contact-loss) is expressed as `completion_policy` / `fallback_policy`, not vendor commands.
2. A **coordinator** that runs an **Activity-DAG** (nodes = hardware actions, parallel paths,
   `sync_flag` barriers) and dispatches arm trajectories (`both_arms` MoveIt/MIT) and hand skills via
   a performer-helper, with clean fault propagation.

It reuses the proven Activity-DAG / resource-token / performer pattern from a related multi-robot
project (the cetibar coordinator), adapted to the Nero Duo system and our repo conventions.

**Depends on sprint 5:** the native CAN FD side buses and arm+hand bus-load validation. The demo is
the roadmap's pouring reference task.

## What already exists (foundation)

- Arm control via the MIT controller; `both_arms` MoveIt planning (OMPL + TRAC-IK).
- OmniHand vendor-SDK bridge backend (`omnihand_backend_type:=sdk`, active 10-joint command, status,
  tactile) — but the hands have not been exercised individually as *skills* yet.

## What this sprint adds

- hand **skill abstraction** + **grasp/release controller** with tactile thresholds (missing piece 1+2)
- a **coordinator** + **activity graph** that orders recorded/planned actions across both arms and
  hands (missing piece 3)

## Layout

- `planning/hefeweizen_pour_proposal.md` — the detailed task proposal (sequence, catalogue, faults).
- `planning/architecture_and_repo_integration.md` — how the cetibar pattern maps onto our packages
  and ROS contracts, plus the open architecture decisions.
- `planning/hand_skill_backend_mapping.md` — semantic `skill_name` → backend gesture + tactile design.
- `planning/hefeweizen_activity_graph.md` — the canonical `hefeweizen_pour_v1` graph.
- `reference/` — read-only reference from the related project (coordinator node, db bridge, overview).
- `checklist.md`, `open_questions.md`, `errors_and_fixes.md` — sprint working files.
