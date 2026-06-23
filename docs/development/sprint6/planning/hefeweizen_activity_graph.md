# Activity Graph — `hefeweizen_pour_v1`

The canonical, machine-readable form of the demo (proposal §7–§8). For the MVP this YAML is the
storage backing `get_activity_plan` / `get_action_detail` / `validate_activity` (architecture
decision §8). Final home: `src/agx_arm_coordination/config/` (and resources in `config/`). Tactile
sensors, thresholds, presets, and trajectories are calibrated on hardware.

## Task Composition Design

Movements fall into two categories by planning source:

- **MoveIt-planned (collision-aware):** transitions between anchor poses. The planner picks a
  collision-free path; only the waypoints (anchor poses) are fixed. Used wherever the arms carry
  objects and must avoid each other or the environment.
- **Recorded trajectories (functional):** taught, replayed motions where the exact Cartesian path
  matters — cap opener engagement, pouring tilt profile, alignment before pouring. These are fixed
  and not replanned at runtime.

This separation keeps collision-aware positioning out of the functional motions and vice versa.

### Arm Responsibility

- **Right arm:** all bottle interactions (grasp, cap opener, pour-side, placement).
- **Left arm:** all glass interactions (grasp, stabilize during pour, placement).

The distinction is consistent throughout grasp, pre-pour, and placement phases.

### Anchor Poses

Anchor poses define the fixed waypoints for MoveIt-planned transitions. Each pose is chosen so the
linear approach/retraction between adjacent poses avoids the held objects.

| Pose | Purpose |
|------|---------|
| `Idle_L` / `Idle_R` | Init pose; arms clear and ready, do not obstruct the workspace |
| `Pre_Grip_L` / `Pre_Grip_R` | Approach pose; glass/bottle can be gripped with a straight linear move from here |
| `Post_Grip_L` / `Post_Grip_R` | Lifted pose after grasp; linear retract clears the resting surface |
| `Init_Working_L` / `Init_Working_R` | In-front-of-robot pose; start point for cap-opener and pour interactions |
| `Pre_Place_L` / `Pre_Place_R` | Pre-placement pose; object can be placed with a linear descend (bottle may reuse `Pre_Grip_R`) |
| `Post_Place_L` / `Post_Place_R` | Lifted pose after placement; linear retract avoids knocking the object |

### Functional Trajectories

Recorded motions where the Cartesian path is fixed:

| Trajectory | Arms | Notes |
|-----------|------|-------|
| `open_bottle` | right only | Cap-opener engagement at fixed station |
| `align_glass_bottle` | both (coordinated) | Relative alignment before tilt |
| `pour_into_glass` | both (coordinated) | Tilt profile, both arms move together |
| `separate_glass_bottle` | both (coordinated) | Separate after pour for safe independent placement |

### Safety Constraints

- MoveIt transitions must be planned with the gripped object included in the collision model once grasped.
- Anchor pose pairs (`Pre_Grip`, `Post_Grip`, `Pre_Place`, `Post_Place`) bracket every linear approach
  and retraction so the planned motion never passes through the object's rest position.
- Functional trajectories run only from their designated anchor entry pose; the coordinator must
  confirm the prior anchor was reached before dispatching.


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

Arm actions — MoveIt-planned transitions between anchor poses:

```yaml
  # Idle → Pre_Grip  (MoveIt, collision-aware, no objects held yet)
  both_arms_home_to_pregrasp:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Idle_L, Idle_R], to_pose: [Pre_Grip_L, Pre_Grip_R],
                velocity_scaling: 0.15, acceleration_scaling: 0.15 }

  # Pre_Grip → Grasp  (linear approach, MoveIt Cartesian)
  both_arms_pregrasp_to_grasp:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Pre_Grip_L, Pre_Grip_R], to_pose: [grasp_L, grasp_R],
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }

  # Grasp → Post_Grip  (linear retract, MoveIt Cartesian; objects now in collision model)
  both_arms_lift_to_post_grip:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [grasp_L, grasp_R], to_pose: [Post_Grip_L, Post_Grip_R],
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }

  # Post_Grip → Init_Working  (MoveIt, both arms move to working zone)
  both_arms_to_init_working:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Post_Grip_L, Post_Grip_R], to_pose: [Init_Working_L, Init_Working_R],
                velocity_scaling: 0.15, acceleration_scaling: 0.15 }

  # Init_Working → Pre_Place  (MoveIt, after functional trajectories complete)
  both_arms_working_to_preplace:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Init_Working_L, Init_Working_R], to_pose: [Pre_Place_L, Pre_Place_R],
                velocity_scaling: 0.15, acceleration_scaling: 0.15 }

  # Pre_Place → Place  (linear descend, MoveIt Cartesian)
  both_arms_preplace_to_place:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Pre_Place_L, Pre_Place_R], to_pose: [place_L, place_R],
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }

  # Place → Post_Place  (linear retract after release, MoveIt Cartesian)
  both_arms_retract_to_post_place:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [place_L, place_R], to_pose: [Post_Place_L, Post_Place_R],
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }

  # Post_Place → Idle  (MoveIt, arms clear)
  both_arms_retract_home:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: moveit_planned,
                from_pose: [Post_Place_L, Post_Place_R], to_pose: [Idle_L, Idle_R],
                velocity_scaling: 0.15, acceleration_scaling: 0.15 }
```

Arm actions — recorded functional trajectories (fixed Cartesian path, not replanned):

```yaml
  # Right arm only: cap-opener engagement at fixed station
  right_arm_open_bottle:
    actiontype_id: Trajectory
    robot_id: right_arm
    metadata: { planning_group: right_arm, source: recorded,
                entry_pose: Init_Working_R,
                velocity_scaling: 0.10, acceleration_scaling: 0.10,
                description: Bottle cap opener engagement — fixed station trajectory }

  # Both arms coordinated: align relative pose before tilt
  both_arms_align_glass_bottle:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: recorded,
                entry_pose: [Init_Working_L, Init_Working_R],
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }

  # Both arms coordinated: tilt and pour profile
  both_arms_pour_profile_v1:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: recorded,
                velocity_scaling: 0.10, acceleration_scaling: 0.10,
                description: Coordinated bottle tilt and glass stabilization }

  # Both arms coordinated: separate glass and bottle for safe independent placement
  both_arms_separate_glass_bottle:
    actiontype_id: Trajectory
    robot_id: both_arms
    metadata: { planning_group: both_arms, source: recorded,
                velocity_scaling: 0.10, acceleration_scaling: 0.10 }
```

## Activity `hefeweizen_pour_v1`

```yaml
activity: hefeweizen_pour_v1
nodes:
  # --- Phase 1: Position to grasp (MoveIt-planned) ---
  - { action_no: 10, action_id: both_arms_home_to_pregrasp }           # Idle → Pre_Grip
  - { action_no: 20, action_id: left_hand_open,           sync_flag: 1 }
  - { action_no: 21, action_id: right_hand_open,          sync_flag: 1 }
  - { action_no: 30, action_id: both_arms_pregrasp_to_grasp }          # Pre_Grip → grasp (linear)

  # --- Phase 2: Grasp and lift (MoveIt-planned retract) ---
  - { action_no: 40, action_id: left_hand_grasp_glass,    sync_flag: 2 }
  - { action_no: 41, action_id: right_hand_grasp_bottle,  sync_flag: 2 }
  - { action_no: 50, action_id: both_arms_lift_to_post_grip }          # grasp → Post_Grip (linear)
  - { action_no: 55, action_id: both_arms_to_init_working }            # Post_Grip → Init_Working

  # --- Phase 3: Functional trajectories (recorded) ---
  - { action_no: 60, action_id: right_arm_open_bottle }                # cap opener, right arm only
  - { action_no: 70, action_id: both_arms_align_glass_bottle }         # relative alignment
  - { action_no: 80, action_id: both_arms_pour_profile_v1 }            # tilt + pour
  - { action_no: 90, action_id: both_arms_separate_glass_bottle }      # separate for placement

  # --- Phase 4: Place and retract (MoveIt-planned) ---
  - { action_no: 100, action_id: both_arms_working_to_preplace }       # Init_Working → Pre_Place
  - { action_no: 110, action_id: both_arms_preplace_to_place }         # Pre_Place → place (linear)
  - { action_no: 120, action_id: left_hand_release_glass,   sync_flag: 3 }
  - { action_no: 121, action_id: right_hand_release_bottle, sync_flag: 3 }
  - { action_no: 130, action_id: both_arms_retract_to_post_place }     # place → Post_Place (linear)
  - { action_no: 140, action_id: both_arms_retract_home }              # Post_Place → Idle

edges:
  [ [10,20],[10,21],[20,30],[21,30],
    [30,40],[30,41],[40,50],[41,50],
    [50,55],[55,60],[60,70],[70,80],[80,90],
    [90,100],[100,110],[110,120],[110,121],[120,130],[121,130],
    [130,140] ]
```

Notes:

- Hands open (20/21), grasp (40/41), and release (120/121) run as `sync_flag` barrier pairs in
  parallel (independent `R_LEFT_HAND` / `R_RIGHT_HAND`).
- Phase 3 functional trajectories (60–90) run sequentially; each recorded trajectory requires the
  prior one to complete before dispatching.
- `right_arm_open_bottle` (60) holds `R_RIGHT_ARM` only; left arm idles at `Init_Working_L`.
- The pour (80) and separation (90) are `both_arms` recorded trajectories — do not split per-arm.
- After a successful grasp, the hand holds internally (`completion_policy.on_success: hold_internal`)
  throughout Phase 3 and Phase 4 arm movements; hold is not a graph node.
- `both_arms_preplace_to_place` (110) is the release approach; release (120/121) fires at the
  bottom, then the arms retract linearly to `Post_Place` before the MoveIt return-home motion.

## Mini graphs for staged validation (proposal §9 Step 4)

```yaml
# Phase 1 smoke test: open hands, linear approach, grasp, linear retract
hands_and_grasp_v1:          20/21 → 30 → 40/41 → 50

# Phase 2 smoke test: full grasp-to-working-zone transition
grasp_to_working_v1:         10 → 20/21 → 30 → 40/41 → 50 → 55

# Phase 3 smoke test: functional trajectories only (start from Init_Working)
functional_trajectories_v1:  60 → 70 → 80 → 90

# Phase 4 smoke test: placement and retract
place_and_home_v1:           100 → 110 → 120/121 → 130 → 140
```

(Express each as its own small `activity:` block when implemented.)

## Validation escalation

`no objects → dummy glass+bottle → empty glass+bottle → water → real Hefeweizen` (proposal §9 Step 5).
Record runs in `hefeweizen_validation_log.md`.
