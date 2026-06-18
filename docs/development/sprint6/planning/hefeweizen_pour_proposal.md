# Proposal: Coordinated Hefeweizen Pouring Demo

status: PROPOSED
scope: execution sprint 6 (depends on sprint 5 CAN/bus validation) / first coordinated dual-arm + dual-hand task
system: Duo Nero arms + dual OmniHand Pro + MoveIt / MIT execution path

## 1. Purpose

This proposal defines the first executable coordinated demo task for the Duo Nero system: pouring a Hefeweizen using both arms and both OmniHands.

The goal is not to build a fully autonomous perception-driven bartender. The goal is to create a deterministic, reproducible, and debuggable first task slice that validates the following system capabilities:

- coordinated dual-arm execution using the existing `both_arms` planning and execution path
- hand skill execution through a ROS-owned abstraction layer above the OmniHand vendor SDK
- tactile-confirmed grasping for glass and bottle handling
- Activity-DAG based task orchestration using the existing coordinator concept
- clear fault propagation between hand skills, arm trajectories, and the coordinator

This demo should become the first reference task for later skill-level manipulation, data collection, and physical AI policy evaluation.

## 2. Current Assumptions

The proposal assumes the following baseline is already available or close to available:

- the Nero arm control path is functional through the MIT controller layer
- the Duo body model and prefixed left/right arm descriptions are available
- MoveIt planning for `right_arm`, `left_arm`, and `both_arms` is available
- OmniHand vendor SDK communication is available through the existing bridge layer
- tactile feedback can be read from the OmniHand bridge
- arm + hand operation on the same native CAN side bus is under validation in Sprint 5

The following parts are not assumed to be fully solved yet:

- stable hand skill abstraction above raw vendor gestures
- calibrated tactile thresholds for bottle and glass grasps
- robust coordinated task fault handling
- sustained bus-load validation for arm + hand per side bus
- collision-validated task-space object setup for glass and bottle

## 3. Design Decision

The first MVP should treat the actual pouring motion as one combined `both_arms` trajectory rather than two independently synchronized per-arm trajectories.

Reason:

- the coordinator can launch sync groups, but goal dispatch is still sequential at the ROS action boundary
- small start-time offsets are acceptable for opening or closing hands, but not ideal for a tightly coordinated pouring trajectory
- a single `both_arms` trajectory keeps the relative arm timing explicit in one trajectory representation
- later versions may split execution again once a tighter synchronization and fault model exists

Therefore:

```text
Hand actions  -> separate hand skill actions
Arm motions   -> combined `both_arms` trajectory actions where coordination matters
Coordinator   -> orders and monitors actions, but does not fake hard real-time sync
```

## 4. Target MVP Behavior

The initial task uses fixed object positions and deterministic scripted/recorded trajectories.

Suggested role assignment:

```text
left arm + left OmniHand   -> grasp and hold the glass
right arm + right OmniHand -> grasp and tilt the bottle
```

The sequence is:

1. move both arms to pregrasp poses
2. open both hands
3. move both hands to glass and bottle grasp poses
4. close both hands until tactile contact is confirmed
5. lift glass and bottle into pouring start pose
6. execute a coordinated pouring trajectory with `both_arms`
7. return glass and bottle to placement poses
8. release both objects
9. retract both arms to a safe pose

## 5. Required Components

### 5.1 OmniHand Skill Controller

A new or extended ROS node should provide a semantic hand skill interface above the vendor SDK.

Recommended package location:

```text
src/agx_arm_ctrl/
```

Recommended node names:

```text
/right_hand/omnihand_skill_controller
/left_hand/omnihand_skill_controller
```

Responsibilities:

- resolve `action_id` + `metadata.skill_name` to a vendor-side skill preset, gesture, or joint command sequence inside the OmniHand backend
- monitor tactile feedback during grasp execution
- decide success or failure based on contact thresholds and timeout policy
- keep the hand in a holding state after successful grasp
- expose action feedback, result, diagnostics, and events
- support cancel/stop behavior

Initial semantic skill set:

```text
open_hand
grasp_glass_until_contact
grasp_bottle_until_contact
release_glass
release_bottle
stop_hand
```

The public database/action layer must not expose vendor-internal `gesture_id` values. For the MVP, `metadata.skill_name` is the canonical semantic key. The OmniHand backend owns the mapping from `skill_name` to whichever vendor SDK gesture, custom preset, or joint command sequence is currently used. If a vendor-side custom skill is introduced later, it should keep the same `skill_name` exposed through the shared database.

Internal state machine:

```text
IDLE
OPENING
CLOSING_UNTIL_CONTACT
GRASP_HOLDING
RELEASING
FAILED
```

### 5.2 Hand Skill Action Interface

A dedicated action interface is preferred for the hand skill controller.

Recommended action name:

```text
HandSkill.action
```

Suggested goal fields:

```text
string hand_id
string skill_name
string[] contact_sensors
float32 contact_threshold
int32 stable_samples
float32 timeout_sec
string completion_policy
string fallback_policy
```

For the MVP, `completion_policy` and `fallback_policy` may also live only in the action metadata JSON and be interpreted by the performer/helper or hand controller. They should describe behavior, not vendor commands. Examples: `hold_internal_on_success`, `finish_when_open`, `stop_and_hold_on_cancel`, `safe_release_on_failure`.

Suggested result fields:

```text
bool success
string message
float32 final_contact_score
string final_state
```

Suggested feedback fields:

```text
string state
float32 progress
float32 contact_score
```

For the MVP, this can also be implemented behind the existing `PerformAction` abstraction if adding a new public action type would slow down the sprint.

### 5.3 Performer Helper Routing

The performer helper should route action execution by `actiontype_id` and `robot_id`.

Initial routing:

```text
Trajectory + robot_id == both_arms
  -> both_arms trajectory executor / MoveIt FJT / MIT split execution path

Trajectory + robot_id == left_arm or right_arm
  -> per-arm MIT execution path

Gripper + robot_id == left_hand or right_hand
  -> corresponding OmniHandSkillController
```

The coordinator should not directly call vendor SDKs or hand-specific nodes. It should only execute database-backed actions through the performer helper.

### 5.4 Coordinator Node

The existing coordinator concept should be reused for the Nero demo.

Recommended package location:

```text
src/agx_arm_coordination/
```

Responsibilities:

- expose an activity execution action server
- load an activity graph from the database bridge
- validate the graph before execution
- schedule ready DAG nodes
- enforce resource rules
- dispatch child actions through the performer helper
- monitor child action feedback and results
- cancel active children on failure or cancellation
- publish coordinator events

The coordinator should stay intentionally narrow for this MVP:

- no online collision reasoning
- no perception-based replanning
- no hard real-time synchronization guarantee
- no direct low-level hardware control

## 6. Resource Model

The MVP should define explicit resource tokens for arms and hands.

Initial resources:

```yaml
resources:
  R_LEFT_ARM:
    robots: [left_arm]
  R_RIGHT_ARM:
    robots: [right_arm]
  R_BOTH_ARMS:
    robots: [both_arms]
  R_LEFT_HAND:
    robots: [left_hand]
  R_RIGHT_HAND:
    robots: [right_hand]
```

Optional CAN-side-bus resources can be added if active bus contention becomes a problem:

```yaml
resources:
  R_LEFT_CAN_BUS:
    robots: [left_arm, left_hand]
  R_RIGHT_CAN_BUS:
    robots: [right_arm, right_hand]
```

Important note:

Tactile hold monitoring should not be modeled as a long-running coordinator action during arm motion. After a successful grasp, the hand controller should internally hold the grasp and publish status. Otherwise, the coordinator may unnecessarily block arm and hand resources on the same side bus.

Normal planned releases should remain regular `Gripper` actions in the Activity graph. Emergency or cancel behavior should not be encoded as separate `hold_*` or `release_*` catalogue actions unless the operator intentionally wants them as explicit graph nodes. For failures and cancellation, use semantic fallback policies that the hand controller resolves internally.

## 7. Demo Action Catalogue

The following actions should be created for the first demo iteration.

### 7.1 Hand Actions

```text
left_hand_open
right_hand_open
left_hand_grasp_glass
right_hand_grasp_bottle
left_hand_release_glass
right_hand_release_bottle
```

Catalogue rule:

- `action_id` identifies the concrete database action used in a task graph.
- `metadata.skill_name` identifies the semantic hand skill the backend must execute.
- no `metadata.gesture_id` should be exposed at this layer.
- object-specific release actions are allowed when the release behavior differs by skill or object; generic `release_object` actions are only appropriate if the backend behavior is truly identical.
- hold behavior should not be a separate coordinator action. It should be expressed as `completion_policy` and handled internally by the hand controller after a successful grasp.

Example metadata for a glass grasp:

```yaml
action_id: left_hand_grasp_glass
actiontype_id: Gripper
robot_id: left_hand
metadata:
  skill_name: grasp_glass_until_contact
  object: glass
  contact_sensors: [thumb, index, middle]
  contact_threshold: 0.35
  stable_samples: 5
  timeout_sec: 4.0
  completion_policy:
    on_success: hold_internal
    passive_contact_monitoring: true
  fallback_policy:
    on_cancel: stop_and_hold
    on_timeout: stop_and_hold
    on_contact_loss: abort_activity
```

Example metadata for a bottle grasp:

```yaml
action_id: right_hand_grasp_bottle
actiontype_id: Gripper
robot_id: right_hand
metadata:
  skill_name: grasp_bottle_until_contact
  object: bottle
  contact_sensors: [thumb, index, middle, ring]
  contact_threshold: 0.35
  stable_samples: 5
  timeout_sec: 4.0
  completion_policy:
    on_success: hold_internal
    passive_contact_monitoring: true
  fallback_policy:
    on_cancel: stop_and_hold
    on_timeout: stop_and_hold
    on_contact_loss: abort_activity
```

Example metadata for an object-specific release:

```yaml
action_id: right_hand_release_bottle
actiontype_id: Gripper
robot_id: right_hand
metadata:
  skill_name: release_bottle
  object: bottle
  timeout_sec: 3.0
  completion_policy:
    on_success: finish_when_open
  fallback_policy:
    on_cancel: stop_motion
    on_timeout: report_failure
```

The exact tactile sensor names, thresholds, backend skill mappings, and release behavior must be calibrated on hardware. The database should store semantic `skill_name` values only; the backend should hide the vendor gesture/custom-preset mapping.


### 7.2 Arm Actions

```text
both_arms_home_to_pregrasp
both_arms_pregrasp_to_grasp
both_arms_lift_to_pour_start
both_arms_pour_profile_v1
both_arms_return_to_place
both_arms_retract_home
```

Example metadata:

```yaml
action_id: both_arms_pour_profile_v1
actiontype_id: Trajectory
robot_id: both_arms
metadata:
  planning_group: both_arms
  source: recorded_or_planned
  velocity_scaling: 0.15
  acceleration_scaling: 0.15
  description: Coordinated bottle tilt and glass stabilization trajectory
```

## 8. Activity Graph

The first complete activity should be called:

```text
hefeweizen_pour_v1
```

Recommended nodes:

| action_no | action_id | robot_id | actiontype_id | purpose |
| ---: | --- | --- | --- | --- |
| 10 | both_arms_home_to_pregrasp | both_arms | Trajectory | move both arms to safe pregrasp poses |
| 20 | left_hand_open | left_hand | Gripper | open glass hand |
| 21 | right_hand_open | right_hand | Gripper | open bottle hand |
| 30 | both_arms_pregrasp_to_grasp | both_arms | Trajectory | move hands to object grasp poses |
| 40 | left_hand_grasp_glass | left_hand | Gripper | tactile-confirmed glass grasp |
| 41 | right_hand_grasp_bottle | right_hand | Gripper | tactile-confirmed bottle grasp |
| 50 | both_arms_lift_to_pour_start | both_arms | Trajectory | lift both objects into pouring start pose |
| 60 | both_arms_pour_profile_v1 | both_arms | Trajectory | coordinated pouring motion |
| 70 | both_arms_return_to_place | both_arms | Trajectory | return objects to placement pose |
| 80 | left_hand_release_glass | left_hand | Gripper | release glass |
| 81 | right_hand_release_bottle | right_hand | Gripper | release bottle |
| 90 | both_arms_retract_home | both_arms | Trajectory | retract to safe home pose |

Recommended edges:

```text
10 -> 20
10 -> 21
20 -> 30
21 -> 30
30 -> 40
30 -> 41
40 -> 50
41 -> 50
50 -> 60
60 -> 70
70 -> 80
70 -> 81
80 -> 90
81 -> 90
```

Optional sync flags:

```text
20 and 21 -> sync_flag 1
40 and 41 -> sync_flag 2
80 and 81 -> sync_flag 3
```

Do not split the actual pour into separate left and right arm actions for the MVP. Use one combined `both_arms` trajectory for action 60.

## 9. Implementation Plan

### Step 1: Validate Hand Skills Standalone

Run the hand skill controller without the coordinator.

Test actions:

```text
left_hand_open
left_hand_grasp_glass
left_hand_release_glass
right_hand_open
right_hand_grasp_bottle
right_hand_release_bottle
```

Success criteria:

- backend skill mapping resolves `skill_name` to the correct vendor SDK command or custom preset
- tactile feedback is received and timestamped
- contact threshold detection works
- stable sample filtering works
- timeout failure works
- result state is reported correctly
- hold state persists after successful grasp

### Step 2: Connect Hand Skills to Performer Helper

Execute hand actions through the same `PerformAction` path used by the coordinator.

Success criteria:

- action metadata is loaded correctly
- `Gripper` actions route to the correct hand skill controller
- result and feedback propagate back to the performer helper
- failures are returned as structured action failures

### Step 3: Validate Combined Arm Trajectories

Execute each `both_arms` trajectory independently.

Success criteria:

- trajectory joint ordering is correct
- both arms move with acceptable relative timing
- multiple consecutive executions work without restarting launch files
- slow velocity scaling is respected
- cancel/stop behavior is safe

### Step 4: Validate Mini Activity Graphs

Before running the full pouring graph, test smaller activities:

```text
hands_open_close_release_v1
both_arms_pregrasp_grasp_retract_v1
both_arms_lift_pour_return_v1
```

Success criteria:

- coordinator loads and validates the activity graph
- independent hand actions can run in parallel where allowed
- resource conflicts are detected
- child action failure cancels active children
- progress feedback is meaningful

### Step 5: Run Full Hefeweizen Activity

Execute:

```text
hefeweizen_pour_v1
```

Recommended test escalation:

```text
1. no objects
2. dummy glass and dummy bottle
3. empty glass and empty bottle
4. water trial
5. real Hefeweizen trial
```

## 10. Fault Handling Requirements

### 10.1 Hand Skill Failures

A hand skill must fail if:

- the vendor SDK call fails
- tactile feedback is stale or unavailable
- contact threshold is not reached before timeout
- the hand reports an error state
- optional force/current limits are exceeded

On failure, the hand skill controller should return a structured failure result. The coordinator should abort the activity and cancel active child actions.

### 10.2 Grasp Slip During Arm Motion

The hand controller should publish status while holding an object.

Minimal MVP behavior:

- publish a warning event if contact score drops below a slip threshold
- optionally fail the current activity if a critical slip threshold is crossed

This should be implemented as passive monitoring, not as a long-running coordinator action.

### 10.3 Arm Execution Failures

Arm trajectory execution must fail if:

- trajectory execution is rejected
- execution times out
- controller reports an error
- cancellation or emergency stop occurs

The coordinator should fail fast and cancel all active child actions.

## 11. Validation Checklist

### Hand Layer

- [ ] left hand opens reliably
- [ ] right hand opens reliably
- [ ] left hand closes until tactile contact on glass dummy
- [ ] right hand closes until tactile contact on bottle dummy
- [ ] grasp timeout works
- [ ] release works
- [ ] hold state persists after successful grasp
- [ ] tactile status remains available during arm motion

### Arm Layer

- [ ] `both_arms_home_to_pregrasp` executes
- [ ] `both_arms_pregrasp_to_grasp` executes
- [ ] `both_arms_lift_to_pour_start` executes
- [ ] `both_arms_pour_profile_v1` executes slowly and smoothly
- [ ] `both_arms_return_to_place` executes
- [ ] `both_arms_retract_home` executes
- [ ] multiple consecutive plan/execute cycles work without launch restart

### Coordinator Layer

- [ ] activity graph can be loaded
- [ ] activity graph validation works
- [ ] resource serialization works
- [ ] parallel hand actions work where allowed
- [ ] child action feedback is aggregated
- [ ] child failure aborts the full activity
- [ ] cancellation cancels active children

### Demo Layer

- [ ] full graph runs without objects
- [ ] full graph runs with dummy objects
- [ ] full graph runs with empty glass and bottle
- [ ] full graph runs with water
- [ ] real Hefeweizen trial is attempted only after all previous checks pass

## 12. Open Questions

The following questions require hardware validation:

- Which backend skill mappings are best for glass and bottle grasping?
- Which tactile sensors are reliable for detecting stable contact on each object?
- What contact thresholds are robust across repeated grasps?
- Does arm + hand on one native side bus remain stable during sustained coordinated motion?
- Should one-shot CAN mode be used for the hand under all demo conditions?
- What is the safest fallback if one hand loses contact during the pouring trajectory?
- How much pouring angle and duration are needed for a visually successful but low-risk first demo?

## 13. Non-Goals For The MVP

The first demo should not attempt to solve:

- visual object detection
- online grasp synthesis
- dexterous in-hand manipulation
- liquid-level estimation
- adaptive pouring from perception
- real-time dual-arm replanning
- fully autonomous recovery after object slip
- simulation-to-real policy transfer

These are later skill, perception, and physical AI milestones.

## 14. Expected Output

At the end of this implementation slice, the repository should contain:

```text
src/agx_arm_ctrl/
  omnihand_skill_controller implementation or extension

src/agx_arm_msgs/
  optional HandSkill.action or equivalent message additions

src/agx_arm_coordination/
  coordinator node adapted from the existing Activity-DAG concept
  scheduler/resource model
  launch/config files

docs/development/sprint6/planning/
  this proposal
  architecture_and_repo_integration.md
  hand_skill_backend_mapping.md
  hefeweizen_activity_graph.md
  hefeweizen_validation_log.md
```

The first successful milestone is not a perfect beer pour. The first successful milestone is a reproducible full graph execution where both arms and both hands are coordinated, grasp confirmation is tactile-based, and failures are surfaced cleanly through the coordinator.

The beer can become impressive after the control path becomes boring.
