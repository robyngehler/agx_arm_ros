# Proposal: Stage 2 Integrated MIT `FollowJointTrajectory` Controller for Nero

> Historical proposal: the integrated MIT action-server path is now part of the current runtime.
> Use `docs/control/bringup.md` and `docs/control/teach_and_run.md` for the active workflow instead of this design note.

## 1. Purpose

This proposal defines the next clean integration step between MoveIt 2 / RViz and the custom MIT controller for the AgileX Nero arm.

The goal is to replace the current transitional bridge stack with a single ROS-native trajectory execution interface:

```text
MoveIt / RViz
  → /arm_controller/follow_joint_trajectory
  → agx_arm_nero_mit_controller
  → /control/move_mit
  → agx_arm_ctrl_single_node
  → pyAgxArm / CAN
  → Nero
```

This is the preferred intermediate architecture before a full `ros2_control` hardware interface is implemented.

The central design principle is:

> MoveIt owns high-level trajectory execution.  
> The MIT controller owns trajectory sampling, safety handling, gravity/feedforward, impedance gains, and low-level MIT command generation.  
> `agx_arm_ctrl_single_node` remains the hardware gateway to `pyAgxArm` and CAN.

---

## 2. Current System Summary

### 2.1 `agx_arm_ctrl_single_node`

Current role: **hardware gateway**.

Responsibilities already present:

- connects to the arm via `pyAgxArm`
- publishes real feedback:
  - `feedback/joint_states`
  - `feedback/tcp_pose`
  - `feedback/arm_status`
  - `feedback/leader_joint_angles`
  - optional gripper / hand feedback
- exposes vendor and low-level command topics:
  - `control/joint_states`
  - `control/move_j`
  - `control/move_p`
  - `control/move_l`
  - `control/move_c`
  - `control/move_js`
  - `control/move_mit`
- exposes services:
  - `enable_agx_arm`
  - `move_home`
  - `emergency_stop`
  - `set_normal_mode`
  - `set_leader_mode`

This node should remain the low-level hardware-facing ROS node.

### 2.2 `agx_arm_nero_mit_controller`

Current role: **MIT execution controller**.

Responsibilities already present:

- subscribes to:
  - `feedback/joint_states`
  - `feedback/leader_joint_angles`
  - `feedback/arm_status`
  - `~/joint_trajectory`
- publishes:
  - `control/move_mit`
  - `~/reference_joint_states`
- provides:
  - `~/enable`
  - `~/hold_current`
  - `~/cancel_trajectory`
- samples a `JointTrajectoryBuffer`
- applies gain ramping
- clamps velocity and feedforward torque
- optionally computes gravity/feedforward from a URDF-backed model
- cancels active MIT trajectory when leader mode becomes active
- holds final or current reference when no trajectory is active

This node should become the official MoveIt trajectory execution endpoint.

### 2.3 `mit_follow_joint_trajectory`

Current role: **temporary MoveIt action bridge**.

Responsibilities currently present:

- exposes a `FollowJointTrajectory` action at `arm_controller/follow_joint_trajectory`
- validates incoming trajectory using `JointTrajectoryBuffer`
- enables the MIT controller through `mit_controller/enable`
- forwards the trajectory to `mit_controller/joint_trajectory`
- publishes action feedback based on desired / actual / error
- cancels through `mit_controller/cancel_trajectory`

This should be merged into `agx_arm_nero_mit_controller` for Stage 2.

### 2.4 `mit_joint_state_bridge`

Current role: **manual/debug soft-target bridge**.

Responsibilities currently present:

- subscribes to `mit_controller/soft_target_joint_states`
- converts target joint states into a single-point `JointTrajectory`
- publishes to `mit_controller/joint_trajectory`
- optionally enables the MIT controller

This should not be part of the production MoveIt execution path.

---

## 3. Proposed Stage 2 Architecture

### 3.1 High-Level Architecture

```text
                             ┌──────────────────────┐
                             │      RViz / MoveIt    │
                             │ MotionPlanning Plugin │
                             └───────────┬──────────┘
                                         │
                                         │ FollowJointTrajectory action
                                         ▼
                   ┌─────────────────────────────────────┐
                   │ agx_arm_nero_mit_controller         │
                   │                                     │
                   │ - /arm_controller/follow_joint_...  │
                   │ - trajectory validation             │
                   │ - trajectory sampling               │
                   │ - MIT reference generation          │
                   │ - gravity/feedforward               │
                   │ - safety checks                     │
                   │ - action feedback/result            │
                   └───────────┬─────────────────────────┘
                               │
                               │ MoveMITMsg
                               ▼
                   ┌─────────────────────────────────────┐
                   │ agx_arm_ctrl_single_node            │
                   │                                     │
                   │ - /control/move_mit                 │
                   │ - pyAgxArm / CAN                    │
                   │ - real feedback publishing          │
                   └───────────┬─────────────────────────┘
                               │
                               ▼
                             Nero Arm
                               │
                               ▼
                   ┌─────────────────────────────────────┐
                   │ feedback/joint_states               │
                   │ feedback/arm_status                 │
                   │ feedback/leader_joint_angles        │
                   └─────────────────────────────────────┘
```

### 3.2 Primary Design Change

The `FollowJointTrajectory` action server moves from the temporary bridge node into the MIT controller node.

Current transitional path:

```text
MoveIt
  → mit_follow_joint_trajectory
  → mit_controller/joint_trajectory
  → agx_arm_nero_mit_controller
  → control/move_mit
```

Target Stage 2 path:

```text
MoveIt
  → agx_arm_nero_mit_controller:/arm_controller/follow_joint_trajectory
  → internal JointTrajectoryBuffer
  → control/move_mit
```

This removes one intermediary node and avoids duplicated trajectory ownership.

---

## 4. Scope

### 4.1 In Scope

- integrate `control_msgs/action/FollowJointTrajectory` directly into `agx_arm_nero_mit_controller`
- keep `agx_arm_ctrl_single_node` as the hardware-facing gateway
- keep `/control/move_mit` as the low-level MIT command output
- keep real measured arm feedback as the only source of truth for MoveIt/RViz state
- create a MoveIt controller configuration targeting `/arm_controller/follow_joint_trajectory`
- add goal validation, cancel handling, feedback, result handling, and tolerances
- make the action server compatible with MoveIt 2 execution
- keep debug/manual paths available but clearly separated
- add launch/config variants for MIT MoveIt execution, debug soft-target mode, and direct vendor-command mode

### 4.2 Out of Scope

- full `ros2_control` hardware interface
- replacement of `agx_arm_ctrl_single_node`
- direct hardware control from MoveIt without the MIT controller
- OmniHand dexterous planning
- cuMotion integration
- Isaac Sim / Isaac Lab integration
- complete refactoring of the vendor driver
- replacing emergency stop / mode services already provided by `agx_arm_ctrl_single_node`

---

## 5. ROS Interface Contract

### 5.1 Node

```text
package: agx_arm_mit_controller
node:    agx_arm_nero_mit_controller
```

### 5.2 Action Server

The MIT controller shall expose:

```text
/arm_controller/follow_joint_trajectory
```

Type:

```text
control_msgs/action/FollowJointTrajectory
```

This is the only production MoveIt trajectory execution input.

### 5.3 Subscriptions

Required:

```text
feedback/joint_states
feedback/leader_joint_angles
feedback/arm_status
```

Optional / later:

```text
feedback/omnihand/joint_states
feedback/gripper_status
feedback/hand_status
```

### 5.4 Publications

Required:

```text
control/move_mit
```

Recommended diagnostics:

```text
mit_controller/reference_joint_states
mit_controller/execution_state
mit_controller/execution_diagnostics
```

Optional standardization relay:

```text
feedback/joint_states → /joint_states
```

The relay/remap should be handled in launch configuration, not by inventing another state source. We are building a controller, not a rumor mill.

### 5.5 Services

Keep existing MIT services:

```text
mit_controller/enable
mit_controller/hold_current
mit_controller/cancel_trajectory
```

Recommended additions:

```text
mit_controller/clear_fault
mit_controller/set_control_mode
```

Do not duplicate vendor services already provided by `agx_arm_ctrl_single_node` unless a dedicated safety abstraction is introduced.

### 5.6 Low-Level Hardware Gateway

`agx_arm_ctrl_single_node` remains responsible for:

```text
control/move_mit
enable_agx_arm
emergency_stop
set_normal_mode
set_leader_mode
feedback/joint_states
feedback/arm_status
feedback/leader_joint_angles
```

---

## 6. MoveIt Configuration

Create a dedicated MoveIt controller config:

```text
agx_arm_moveit/config/moveit_controllers_mit.yaml
```

Recommended content:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
```

Launch requirement:

```text
MoveIt must target /arm_controller/follow_joint_trajectory.
The AgileX mock ros2_control execution path must not be started for real MIT control.
```

Recommended launch file:

```text
agx_arm_mit_controller/launch/nero_mit_moveit.launch.py
```

Expected nodes:

```text
agx_arm_ctrl_single_node
agx_arm_nero_mit_controller
robot_state_publisher
move_group
rviz2
optional: joint state relay feedback/joint_states → /joint_states
```

Do not launch these for production MoveIt execution:

```text
mit_follow_joint_trajectory
mit_joint_state_bridge
ros2_control mock trajectory execution path
```

---

## 7. Controller State Machine

The MIT controller should expose a single internal execution state machine.

Recommended states:

```text
DISABLED
IDLE_HOLD
ARMING
EXECUTING_TRAJECTORY
CANCELING_TO_HOLD
HOLDING_FINAL_POINT
LEADER_MODE
STALE_FEEDBACK
FAULTED
```

### State Semantics

#### `DISABLED`

- no MIT commands are published
- incoming goals may be rejected or may trigger auto-enable if configured

#### `IDLE_HOLD`

- controller is enabled
- controller holds current or last reference
- no active trajectory goal

#### `ARMING`

- transition state for gain ramping
- used after enable or before trajectory execution

#### `EXECUTING_TRAJECTORY`

- active `FollowJointTrajectory` goal exists
- trajectory is sampled in the control loop
- action feedback is published

#### `CANCELING_TO_HOLD`

- active goal canceled
- controller captures current measured joint state or nearest safe reference
- action returns canceled

#### `HOLDING_FINAL_POINT`

- trajectory finished
- final reference is held if `hold_final_point == true`

#### `LEADER_MODE`

- robot entered leader/teaching mode
- active trajectory must be aborted
- MIT command publishing must pause

#### `STALE_FEEDBACK`

- feedback timeout exceeded
- command publishing is paused
- active trajectory must be aborted or held according to safety policy

#### `FAULTED`

- arm status indicates fault
- position error, command validation, or hardware status violated safety policy
- no new goals accepted until cleared

---

## 8. Action Execution Semantics

### 8.1 Goal Acceptance

A new goal is accepted only if:

- feedback is fresh
- all required joints are present
- trajectory contains exactly the controlled arm joints, or can be safely reordered
- trajectory has at least one point
- time values are strictly increasing
- requested positions are within configured limits
- requested velocities are within configured limits if provided
- no leader mode is active
- no unrecovered arm fault is active

Recommended policy:

```text
Strict joint-name validation by default.
Allow reordering only if explicitly enabled.
Reject unknown or missing arm joints.
```

### 8.2 Start-State Validation

Before execution, compare the current measured state with the first trajectory point.

Recommended parameter:

```yaml
start_state_tolerance: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
```

If the first trajectory point is too far from measured state:

- reject before execution, or
- insert a controlled transition segment only if explicitly enabled.

Default should be rejection. The robot is not a teleportation API, despite what some planners emotionally believe.

### 8.3 Preemption Policy

Recommended default:

```text
reject_new_goal_while_executing: true
```

Alternative later:

```text
preempt_current_goal_with_hold_then_execute
```

Do not silently replace active trajectories.

### 8.4 Sampling

The existing `JointTrajectoryBuffer` remains the source for sampled references.

Sampling should happen inside the MIT controller control loop, not inside a separate bridge node.

### 8.5 Action Feedback

Publish `FollowJointTrajectory.Feedback` at a lower rate than the MIT control loop.

Recommended:

```yaml
action_feedback_rate_hz: 20.0
```

Feedback fields:

```text
joint_names
desired
actual
error
```

Where:

```text
desired = current sampled trajectory point
actual  = latest measured feedback/joint_states
error   = desired - actual
```

### 8.6 Completion

A goal succeeds only if:

- trajectory duration elapsed
- feedback is fresh
- final joint error is within goal tolerance
- no arm status fault occurred
- no leader mode transition occurred
- no cancel request was received

Recommended default goal tolerance:

```yaml
goal_position_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
goal_velocity_tolerance: [0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20]
goal_time_tolerance_s: 0.5
```

These values are placeholders and must be tuned locally.

### 8.7 Cancel

On cancel:

1. stop sampling the active trajectory
2. capture current measured joint positions
3. enter `CANCELING_TO_HOLD`
4. command hold with ramped gains
5. mark the action goal as canceled

Cancel must not disable the hardware unless a separate emergency policy is triggered.

### 8.8 Abort

Abort the active goal if:

- feedback becomes stale
- leader mode becomes active
- arm status indicates fault
- position error exceeds configured hard limit
- command publishing fails
- hardware emergency stop is triggered
- the controller is disabled externally

---

## 9. Safety Rules

### 9.1 Command Ownership

Only one production command owner is allowed.

In MoveIt MIT mode:

```text
Allowed:
  MoveIt → /arm_controller/follow_joint_trajectory
  MIT → /control/move_mit

Not allowed:
  MoveIt mock ros2_control → control/joint_states
  joint_state_bridge → mit_controller/joint_trajectory
  manual control/move_j while active MIT trajectory is executing
```

### 9.2 Feedback Ownership

Measured robot state is authoritative:

```text
feedback/joint_states
```

Optional remap:

```text
feedback/joint_states → /joint_states
```

Do not use `JointState` as a production command format for MoveIt execution.

### 9.3 Watchdogs

Required:

```yaml
feedback_timeout_s: 0.25
command_timeout_s: 0.10
action_feedback_rate_hz: 20.0
```

### 9.4 Limits

Required:

```yaml
position_error_limit
velocity_limit
torque_limit
kp
kd
```

The currently commented position error clamp in the MIT control loop should be revisited and converted into an explicit policy:

```text
warn_only
hold_joint
abort_goal
fault_controller
```

Recommended default for real hardware:

```text
abort_goal + hold_current
```

### 9.5 Mode Handling

If `feedback/arm_status` reports leader/teaching mode:

- abort active trajectory
- stop publishing MIT commands
- enter `LEADER_MODE`
- resume only after normal mode and fresh feedback are confirmed

---

## 10. Implementation Plan

### Step 1 — Preserve Current Working Behavior

Before refactoring:

- record current successful launch command
- record topic graph
- record one successful MIT trajectory execution
- save sample rosbag:
  - `feedback/joint_states`
  - `feedback/arm_status`
  - `control/move_mit`
  - `mit_controller/reference_joint_states`
  - current `FollowJointTrajectory` action if used

### Step 2 — Add Action Server to MIT Controller

In `mit_controller_node.py`:

- import `FollowJointTrajectory`
- add `ActionServer`
- add parameters:
  - `action_name`
  - `action_feedback_rate_hz`
  - `start_state_tolerance`
  - `goal_position_tolerance`
  - `goal_velocity_tolerance`
  - `goal_time_tolerance_s`
  - `allow_joint_reordering`
  - `reject_new_goal_while_executing`
- implement:
  - `goal_callback`
  - `cancel_callback`
  - `execute_callback`
  - feedback publishing
  - result generation

Recommended default:

```yaml
action_name: "/arm_controller/follow_joint_trajectory"
```

### Step 3 — Unify Trajectory Ownership

The action server should set `self.active_trajectory` directly.

Remove production dependency on:

```text
~/joint_trajectory
```

Recommended policy:

- keep `~/joint_trajectory` only as debug input, disabled by default
- or guard it behind a parameter:

```yaml
enable_debug_joint_trajectory_topic: false
```

### Step 4 — Introduce Explicit Execution State

Add an enum-style execution state.

Recommended internal field:

```python
self.execution_state = ExecutionState.DISABLED
```

Publish state periodically:

```text
mit_controller/execution_state
```

Message type can initially be `std_msgs/String`; later define a custom status message if needed.

### Step 5 — Harden Safety Behavior

Add explicit handling for:

- stale feedback
- leader mode
- active arm status faults
- excessive position error
- cancel while executing
- external disable while executing

### Step 6 — Create MoveIt MIT Config

Add:

```text
agx_arm_moveit/config/moveit_controllers_mit.yaml
```

Update or create launch file:

```text
agx_arm_mit_controller/launch/nero_mit_moveit.launch.py
```

Launch file should avoid the AgileX mock trajectory execution path.

### Step 7 — Deprecate Temporary Bridge

Keep `follow_joint_trajectory_action.py` for one transition cycle, but mark it deprecated.

Recommended name if retained:

```text
legacy_mit_follow_joint_trajectory_bridge.py
```

Add runtime warning:

```text
This bridge is deprecated. Use agx_arm_nero_mit_controller action server directly.
```

### Step 8 — Restrict `joint_state_trajectory_bridge`

Keep for manual tests only.

Recommended launch argument:

```yaml
enable_soft_target_bridge: false
```

Default must be false in MoveIt execution launch files.

---

## 11. Testing Plan

### 11.1 Unit Tests

Target:

```text
test_mit_follow_joint_trajectory_controller.py
```

Test cases:

- valid trajectory accepted
- missing joint rejected
- unknown joint rejected
- reordered joint names accepted only if enabled
- non-monotonic timestamps rejected
- empty trajectory rejected
- start state violation rejected
- goal tolerance success
- goal tolerance violation
- cancel transitions to hold
- stale feedback aborts goal
- leader mode aborts goal

### 11.2 Fake Hardware Test

Use fake feedback publisher:

```text
fake_feedback_node → feedback/joint_states
```

Check:

- action server accepts goal
- action feedback fields are populated
- `control/move_mit` is published
- cancel stops trajectory
- final success requires tolerance

### 11.3 Hardware Low-Speed Test

Recommended conditions:

- no payload
- low gains
- low torque limits
- short trajectory
- workspace clear
- physical E-stop ready

Test sequence:

1. enable arm
2. confirm fresh feedback
3. send small single-joint trajectory
4. send multi-joint trajectory
5. cancel trajectory mid-motion
6. force stale feedback test if safe
7. switch leader mode during inactive state
8. verify MoveIt RViz execution

### 11.4 MoveIt Integration Test

Required checks:

```bash
ros2 action list | grep follow_joint_trajectory
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /feedback/joint_states
ros2 topic echo /control/move_mit
```

MoveIt RViz test:

- plan to nearby pose
- execute
- verify action result
- verify real feedback follows
- verify RViz current state tracks measured robot

---

## 12. Acceptance Criteria

Stage 2 is accepted when:

- MoveIt can execute a planned Nero arm trajectory through `/arm_controller/follow_joint_trajectory`
- no separate `mit_follow_joint_trajectory` bridge is running
- no `joint_state_trajectory_bridge` is used during MoveIt execution
- MIT controller owns the active trajectory and action result
- `agx_arm_ctrl_single_node` remains the only hardware gateway
- `/control/move_mit` is the only MIT low-level arm command topic used during execution
- real measured feedback drives RViz and MoveIt current state
- cancel from MoveIt causes controlled hold
- stale feedback aborts or pauses safely
- leader mode aborts active execution safely
- final action success is based on actual joint error tolerance, not only elapsed time
- the launch setup prevents simultaneous command paths

---

## 13. Open Discovery Tasks for Local Agents

Local agents should inspect the repository before implementation and report findings.

### 13.1 MoveIt Config Discovery

Find:

```text
agx_arm_moveit/config/*
agx_arm_moveit/launch/*
```

Report:

- active controller config files
- whether `moveit_controllers_none.yaml` or similar is used
- where `ros2_control_node` is launched
- how `follow` / `control_topic` is configured
- how Nero-specific joint count is handled

### 13.2 Launch Graph Discovery

Find all launch files that start:

```text
agx_arm_ctrl_single_node
agx_arm_nero_mit_controller
mit_follow_joint_trajectory
mit_joint_state_bridge
ros2_control_node
move_group
rviz2
```

Report possible conflicting command publishers.

### 13.3 Topic Ownership Discovery

Run on hardware or simulated setup:

```bash
ros2 topic info /control/move_mit --verbose
ros2 topic info /control/joint_states --verbose
ros2 topic info /feedback/joint_states --verbose
ros2 action info /arm_controller/follow_joint_trajectory
```

Report:

- publishers
- subscribers
- duplicate command paths
- unexpected action servers

### 13.4 Safety Discovery

Inspect how `feedback/arm_status` encodes:

- fault state
- leader mode
- normal mode
- motion status
- communication errors
- joint angle limits

Map these fields into MIT execution abort conditions.

### 13.5 Namespace Discovery

Decide whether controller names should be global or namespaced.

Candidate global names:

```text
/arm_controller/follow_joint_trajectory
/joint_states
/feedback/joint_states
/control/move_mit
```

Candidate namespaced names:

```text
/nero/arm_controller/follow_joint_trajectory
/nero/joint_states
/nero/feedback/joint_states
/nero/control/move_mit
```

Default for initial compatibility:

```text
Use AgileX-compatible non-namespaced topics.
```

---

## 14. Risks and Mitigations

### Risk: Two active command paths

Mitigation:

- launch validation
- explicit mode parameter
- warn if `control/joint_states` or debug trajectory topic is active during MoveIt MIT mode
- document command ownership

### Risk: Action reports success while robot did not reach target

Mitigation:

- final goal tolerance check from measured feedback
- goal timeout handling
- stale feedback abort

### Risk: Controller continues after feedback loss

Mitigation:

- feedback watchdog
- immediate command pause
- abort active action
- hold only after fresh feedback returns

### Risk: Leader mode conflicts with MIT control

Mitigation:

- leader mode detection from `feedback/arm_status`
- abort active action
- stop publishing MIT commands

### Risk: MoveIt start state mismatch

Mitigation:

- start-state tolerance check
- reject unsafe trajectories
- require RViz current state to follow real robot feedback

### Risk: Too much refactor at once

Mitigation:

- preserve existing `~/joint_trajectory` path as debug-only for one transition cycle
- keep old action bridge available but disabled
- test with fake feedback before hardware

---

## 15. Migration Strategy

### Phase 1 — Add Integrated Action Server

Add the action server into `agx_arm_nero_mit_controller` while leaving existing topic input intact.

### Phase 2 — Switch MoveIt Launch

Make MoveIt use:

```text
/arm_controller/follow_joint_trajectory
```

served directly by the MIT controller.

### Phase 3 — Disable Legacy Bridge

Do not launch:

```text
mit_follow_joint_trajectory
```

in normal MoveIt mode.

### Phase 4 — Restrict JointState Bridge

Keep:

```text
mit_joint_state_bridge
```

only for manual debug launch files.

### Phase 5 — Clean Documentation

Document the three allowed modes:

```text
mode: moveit_mit
mode: manual_vendor
mode: debug_soft_target
```

Each mode must have exactly one command owner.

---

## 16. Relationship to Future Stage 3

Stage 2 is not the final ROS architecture. The later fully native solution is:

```text
MoveIt
  → ros2_control joint_trajectory_controller
  → custom hardware_interface::SystemInterface
  → MIT command layer
  → agx_arm_ctrl / pyAgxArm / CAN
```

Stage 2 intentionally avoids this larger implementation while preserving the same external MoveIt contract:

```text
/arm_controller/follow_joint_trajectory
```

This means Stage 2 can later be replaced by Stage 3 without changing MoveIt task code, RViz workflows, or higher-level planners.

---

## 17. Recommended Deliverables

- `mit_controller_node.py` with integrated `FollowJointTrajectory` action server
- `moveit_controllers_mit.yaml`
- `nero_mit_moveit.launch.py`
- `execution_state` publisher
- updated README:
  - command ownership
  - launch modes
  - safety behavior
  - test procedure
- tests:
  - unit tests
  - fake feedback integration test
  - low-speed hardware test checklist
- legacy deprecation notice for:
  - `follow_joint_trajectory_action.py`
  - `joint_state_trajectory_bridge.py` in production MoveIt mode

---

## 18. Final Recommendation

Implement Stage 2 directly.

Do not continue building production behavior around:

```text
MoveIt → action bridge → JointTrajectory topic → MIT controller
```

Instead, make the MIT controller the actual MoveIt-compatible execution endpoint:

```text
MoveIt → agx_arm_nero_mit_controller:/arm_controller/follow_joint_trajectory
```

Keep `agx_arm_ctrl_single_node` as the hardware gateway and keep `/control/move_mit` as the low-level MIT command transport.

This gives the project the cleanest intermediate architecture without prematurely committing to a full `ros2_control` hardware interface.
