## Information about Target Coordinator and Action-Activity flow

### Sample Description from related MR project
```text
# Coordination – Multi-Robot Coordination Node

## Overview

The `coordination` package provides a ROS 2 node for multi-robot coordination and resource management. It orchestrates activity compositions across multiple robots (UR, Portal, Panda) using exclusive resource tokens and centralized scheduling.

## Features

- **Activity execution action server** at `/coord/execute_activity`
- **Continuous frontier scheduling** for activity DAGs loaded from `db_bridge`
- **Current lab resource rules** from `ros_ws/config/namespaces.yaml`
- **Parallel execution where allowed** for independent resources
- **Global sync groups** using `sync_flag`
- **Coordinator events** on `/coord/events`

Current scope is intentionally narrow:

- no planner or collision reasoning
- no geometry-based interference checks
- no public resource lock services yet
- no full coordinator-visible system state topic yet

## Interfaces

### Published Topics
- `/coord/events` (`cetibar_msgs/RobotEvent`) – coordinator execution events

### Subscribed Topics
- `/ur_1/events` (`cetibar_msgs/RobotEvent`) – UR robot events
- `/portal/events` (`cetibar_msgs/RobotEvent`) – Portal events
- `/panda_1/events` (`cetibar_msgs/RobotEvent`) – Panda events (future)

### Services
- none yet

### Actions
- `/coord/execute_activity` (`cetibar_msgs/PerformActivity`) – coordinator-owned activity execution

## Current Scheduling Rules

The coordinator deliberately shifts geometric responsibility to the user for now.
It only enforces the currently agreed execution constraints:

- actions on the same robot are serialized
- `ur_1` and `portal` share `R_UR_PORT` and therefore never run in parallel
- `panda_1` uses `R_FRANKA` and may run in parallel with `R_UR_PORT`
- nodes with the same `sync_flag` are launched together as one barrier group
- if a `sync_flag` group itself contains a resource conflict, execution is rejected

This keeps execution aligned with the old `CompositionHelper` graph semantics while avoiding fake planner guarantees.

## Known Limitations

These points are intentionally left for later work:

- sync-start is still not precise: goals inside a sync group are dispatched back-to-back and accepted sequentially via `send_goal_async`, not started with a tighter synchronization primitive
- scheduling is deterministic and greedy: it starts the first resource-safe frontier groups it can, but it does not search for a globally better parallel schedule
- no planner-based collision reasoning exists yet; operator-created graphs remain responsible for geometric safety beyond the explicit resource rules
- no public coordinator lock APIs or richer coordinator state topics exist yet

## Resource Management Example

**Scenario:** Two activities compete for Portal-UR system

```
Activity A: UR pick-and-place (requires R_UR_PORT)
Activity B: Portal repositioning (requires R_UR_PORT)

Coordinator:
1. Activity A requests R_UR_PORT → GRANTED (lock acquired)
2. Activity B requests R_UR_PORT → QUEUED (lock unavailable)
3. Activity A completes → RELEASE R_UR_PORT
4. Activity B requests R_UR_PORT → GRANTED (lock acquired)
```

**Parallel Execution:**
```
Activity A: UR pick-and-place (requires R_UR_PORT)
Activity C: Panda assembly (requires R_FRANKA)

Coordinator:
1. Activity A requests R_UR_PORT → GRANTED
2. Activity C requests R_FRANKA → GRANTED (parallel execution allowed)
3. Both activities run simultaneously (no physical interference)
```

## Execution Model

**Database Schema:** `database/src/DatabaseObjects.py` (existing)
- `ActivityComposition` table: Activity metadata, sub-activity sequence
- `ActivityUnit` table: Individual sub-activities (robot actions)

1. Load activity graph from `db_bridge/get_activity_plan`
2. Validate graph via `db_bridge/validate_activity`
3. Resolve action metadata via `db_bridge/get_action_detail`
4. Build ready-node frontier from DAG dependencies
5. Identify startable groups from the current frontier and active resource usage
6. Dispatch each newly startable action through `performer_helper/perform`
7. Monitor child completion continuously and fail fast on the first child failure or timeout
8. Release internal resource tokens as children finish and continue until the full activity completes

```