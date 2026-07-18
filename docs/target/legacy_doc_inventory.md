# Legacy Doc Inventory

status: ACTIVE_MIGRATION_TRACKER
last_updated: 2026-07-18

This file is the cleanup inventory for old documentation surfaces that still need one of these
outcomes:

- retire after reference cleanup
- rewrite into a short historical note
- consolidate several overlapping notes into one proposal or one error summary
- keep only as temporary migration evidence until the replacement is complete

The intent is a clean migration without backwards-compatibility promises. If a legacy document is no
longer needed after promotion or consolidation, delete it.

## Repo-wide search targets

Search these exact paths or names after each cleanup wave:

```text
docs/development/
docs/control/bringup.md
docs/project/python_environment_workflow.md
docs/project/repo_interaction_diagrams.md
```

Useful text searches:

```text
docs/control/bringup.md
docs/project/python_environment_workflow.md
docs/project/repo_interaction_diagrams.md
start_single_agx_arm_moveit.launch.py
demo.launch.py
one-shot off
can_nero
can0
```

## Compatibility shims

Action: remove these once no active references remain.

```text
docs/control/bringup.md
docs/project/python_environment_workflow.md
docs/project/repo_interaction_diagrams.md
```

## Top-level development surfaces

Action: keep only roadmap, progress, and transitional component-routing notes; retire duplicate
global tracking surfaces.

```text
docs/development/README.md
docs/development/checklist.md
docs/development/component_implementation_map.md
docs/development/errors_and_fixes.md
docs/development/mismatches_todo.md
docs/development/nero_physical_ai_progress.md
docs/development/nero_physical_ai_roadmap.md
docs/development/open_questions.md
```

## Sprint 1 migration status

Sprint 1 is already migrated into `docs/sprint1/`.

Keep only this surviving historical evidence note there:

```text
docs/sprint1/evidence/mit_soft_control_and_gravity_proposal.md
```

## Sprint 2 migration status

Sprint 2 is already migrated into `docs/sprint2/`.

Keep only these surviving historical evidence notes there:

```text
docs/sprint2/evidence/mit_runtime_history.md
docs/sprint2/evidence/omnihand_canfd_transport_history.md
```

## Sprint 3 migration status

Sprint 3 is already migrated into `docs/sprint3/`.

Keep only these surviving historical evidence notes there:

```text
docs/sprint3/evidence/stage2_mit_follow_joint_trajectory_proposal.md
docs/sprint3/evidence/trac_ik_humble_jetson_repro.md
```

## Sprint 4 migration status

Sprint 4 is already migrated into `docs/sprint4/`.

Keep only this surviving historical evidence note there:

```text
docs/sprint4/evidence/duo_system_integration_direction.md
```

## Sprint 5 migration status

Sprint 5 is already migrated into `docs/sprint5/`.

Keep only this surviving historical evidence note there:

```text
docs/sprint5/evidence/can_transport_decision.md
```

## Sprint 6 legacy docs

Action: separate durable coordinator design from raw debug logs, session handoff notes, validation
logs, and imported reference dumps; keep only one crafted proposal lineage per topic.

```text
docs/development/sprint6/README.md
docs/development/sprint6/checklist.md
docs/development/sprint6/errors_and_fixes.md
docs/development/sprint6/open_questions.md
docs/development/sprint6/planning/architecture_and_repo_integration.md
docs/development/sprint6/planning/debug.md
docs/development/sprint6/planning/debug_recordings.md
docs/development/sprint6/planning/duo_both_hands_moveit_gap.md
docs/development/sprint6/planning/gravity_payload_api_plan.md
docs/development/sprint6/planning/hand_skill_backend_mapping.md
docs/development/sprint6/planning/hefeweizen_activity_graph.md
docs/development/sprint6/planning/hefeweizen_pour_proposal.md
docs/development/sprint6/planning/hefeweizen_validation_log.md
docs/development/sprint6/planning/omnihand_gesture_mapping.md
docs/development/sprint6/planning/session_handoff_2026-06-29.md
docs/development/sprint6/planning/teaching_demo_03-07-2026-debug-moveit_plan_error.md
docs/development/sprint6/planning/teaching_demo_03-07-2026.md
docs/development/sprint6/reference/cetibar_coordination_overview.md
docs/development/sprint6/reference/coordinator_node_ref.md
docs/development/sprint6/reference/db_bridge_ref.md
```

## Other development-track docs

Action: keep only if they still provide unique value beyond git history and stable promoted docs.

```text
docs/development/sprint_physAI/brainstorm.md
```

## Current priority cleanup queue

1. Retire duplicate top-level development tracking files.
2. Remove runnable legacy control instructions from sprint notes.
3. Consolidate overlapping proposal families:
   - Sprint 1 MIT soft-control proposals
   - Sprint 2 OmniHand CAN FD bringup proposals and investigation
   - Sprint 5 shared CAN transport proposal versus transport decision
   - Sprint 6 coordinator and Hefeweizen proposal chain versus debug logs
4. Delete promoted asset-source notes once repo-wide references are clean.