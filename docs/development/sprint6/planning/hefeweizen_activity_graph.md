# Activity Graph — `hefeweizen_pour_v1`

The canonical, machine-readable form of the demo (proposal §7–§8). For the MVP this YAML is the
storage backing `get_activity_plan` / `get_action_detail` / `validate_activity` (architecture
decision §8). Final home: `src/agx_arm_coordination/config/` (and resources in `config/`). Tactile
sensors, thresholds, presets, and trajectories are calibrated on hardware.

## Resources

```yaml
resources:
  R_LEFT_ARM:   { robots: [left_arm] }
  R_RIGHT_ARM:  { robots: [right_arm] }
  R_BOTH_ARMS:  { robots: [both_arms] }
  R_LEFT_HAND:  { robots: [left_hand] }
  R_RIGHT_HAND: { robots: [right_hand] }
  # CAN-bus tokens deferred until sprint-5 bus-load validation shows contention:
  # R_LEFT_CAN_BUS:  { robots: [left_arm, left_hand] }
  # R_RIGHT_CAN_BUS: { robots: [right_arm, right_hand] }
```

`both_arms` shares the joints of `left_arm`/`right_arm`, so the coordinator must treat
`R_BOTH_ARMS` as conflicting with both per-arm tokens (no per-arm action may run while a
`both_arms` action holds the arms).

## Action catalogue

Hand actions (`Gripper`) — only `skill_name` is public; the backend owns the vendor mapping:

```yaml
actions:
  left_hand_open:
    actiontype_id: Gripper
    robot_id: left_hand
    metadata: { skill_name: open_hand, timeout_sec: 3.0,
                completion_policy: { on_success: finish_when_open } }
  right_hand_open:
    actiontype_id: Gripper
    robot_id: right_hand
    metadata: { skill_name: open_hand, timeout_sec: 3.0,
                completion_policy: { on_success: finish_when_open } }

  left_hand_grasp_glass:
    actiontype_id: Gripper
    robot_id: left_hand
    metadata:
      skill_name: grasp_glass_until_contact
      object: glass
      contact_sensors: [thumb, index, middle]
      contact_threshold: 0.35      # placeholder, calibrate
      stable_samples: 5
      timeout_sec: 4.0
      completion_policy: { on_success: hold_internal, passive_contact_monitoring: true }
      fallback_policy:   { on_cancel: stop_and_hold, on_timeout: stop_and_hold, on_contact_loss: abort_activity }

  right_hand_grasp_bottle:
    actiontype_id: Gripper
    robot_id: right_hand
    metadata:
      skill_name: grasp_bottle_until_contact
      object: bottle
      contact_sensors: [thumb, index, middle, ring]
      contact_threshold: 0.35      # placeholder, calibrate
      stable_samples: 5
      timeout_sec: 4.0
      completion_policy: { on_success: hold_internal, passive_contact_monitoring: true }
      fallback_policy:   { on_cancel: stop_and_hold, on_timeout: stop_and_hold, on_contact_loss: abort_activity }

  left_hand_release_glass:
    actiontype_id: Gripper
    robot_id: left_hand
    metadata: { skill_name: release_glass, object: glass, timeout_sec: 3.0,
                completion_policy: { on_success: finish_when_open },
                fallback_policy: { on_cancel: stop_motion, on_timeout: report_failure } }
  right_hand_release_bottle:
    actiontype_id: Gripper
    robot_id: right_hand
    metadata: { skill_name: release_bottle, object: bottle, timeout_sec: 3.0,
                completion_policy: { on_success: finish_when_open },
                fallback_policy: { on_cancel: stop_motion, on_timeout: report_failure } }
```

Arm actions (`Trajectory`, one combined `both_arms` trajectory per coordinated motion):

```yaml
  both_arms_home_to_pregrasp:   { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15 } }
  both_arms_pregrasp_to_grasp:  { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15 } }
  both_arms_lift_to_pour_start: { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15 } }
  both_arms_pour_profile_v1:    { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15,
                description: Coordinated bottle tilt and glass stabilization } }
  both_arms_return_to_place:    { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15 } }
  both_arms_retract_home:       { actiontype_id: Trajectory, robot_id: both_arms,
    metadata: { planning_group: both_arms, source: recorded_or_planned, velocity_scaling: 0.15, acceleration_scaling: 0.15 } }
```

## Activity `hefeweizen_pour_v1`

```yaml
activity: hefeweizen_pour_v1
nodes:
  - { action_no: 10, action_id: both_arms_home_to_pregrasp }
  - { action_no: 20, action_id: left_hand_open,          sync_flag: 1 }
  - { action_no: 21, action_id: right_hand_open,         sync_flag: 1 }
  - { action_no: 30, action_id: both_arms_pregrasp_to_grasp }
  - { action_no: 40, action_id: left_hand_grasp_glass,   sync_flag: 2 }
  - { action_no: 41, action_id: right_hand_grasp_bottle, sync_flag: 2 }
  - { action_no: 50, action_id: both_arms_lift_to_pour_start }
  - { action_no: 60, action_id: both_arms_pour_profile_v1 }
  - { action_no: 70, action_id: both_arms_return_to_place }
  - { action_no: 80, action_id: left_hand_release_glass,    sync_flag: 3 }
  - { action_no: 81, action_id: right_hand_release_bottle,  sync_flag: 3 }
  - { action_no: 90, action_id: both_arms_retract_home }
edges:
  [ [10,20],[10,21],[20,30],[21,30],[30,40],[30,41],[40,50],[41,50],
    [50,60],[60,70],[70,80],[70,81],[80,90],[81,90] ]
```

Notes:

- the two hands open (20/21), grasp (40/41), and release (80/81) as `sync_flag` barrier pairs and
  may run in parallel (independent `R_LEFT_HAND` / `R_RIGHT_HAND`).
- the pour (60) is **one** combined `both_arms` trajectory — do not split it into per-arm actions
  for the MVP (proposal §3, §8).
- after a successful grasp, the hand holds internally (`completion_policy.on_success: hold_internal`)
  while the arms move (50/60/70); hold is **not** a graph node.

## Mini graphs for staged validation (proposal §9 Step 4)

```yaml
hands_open_close_release_v1:       [10? no] use only 20,21,40,41,80,81 with their edges
both_arms_pregrasp_grasp_retract_v1: 10 -> 30 -> 40/41 -> 90
both_arms_lift_pour_return_v1:       50 -> 60 -> 70
```

(Express each as its own small `activity:` block when implemented.)

## Validation escalation

`no objects → dummy glass+bottle → empty glass+bottle → water → real Hefeweizen` (proposal §9 Step 5).
Record runs in `hefeweizen_validation_log.md`.
