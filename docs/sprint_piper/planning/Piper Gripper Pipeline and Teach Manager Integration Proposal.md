# Piper Gripper Pipeline and Teach Manager Integration Proposal

## Goal

Complete the Piper gripper integration in `ROS2_Duo_System_V03` from the already implemented driver/MoveIt layer up through:

1. reliable `FollowJointTrajectory` execution,
2. Coordinator / Activity routing,
3. catalogue-level gripper actions,
4. Teach Manager control,
5. optional synchronized gripper events in taught recordings.

The implementation must preserve the current authority model and must not reintroduce direct unguarded SDK or legacy `/control/joint_states` command paths.

---

# 1. Normalized Piper Gripper Command

Expose the Piper gripper to higher-level components through a normalized closure value:

```text
closure = 0.0  -> fully open
closure = 1.0  -> fully closed
```

Intermediate values represent intermediate closure.

Do not expose the physical gripper width as the primary Teach Manager / Activity API.

Use one canonical conversion:

```python
width = width_open - closure * (width_open - width_closed)
```

where `width_open` and `width_closed` come from the gripper model/configuration rather than being duplicated as magic constants across packages.

Requirements:

- reject NaN/Inf;
- reject values outside `[0.0, 1.0]` instead of silently accepting malformed commands;
- keep force as a separate optional configuration/property;
- use the same conversion helper everywhere higher-level normalized commands are translated to physical gripper width.

This normalized representation should be the semantic representation used by the Teach Manager and catalogue. The lower driver/MoveIt layer may continue using physical joint/width units.

---

# 2. Fix Piper FJT Execution Before Using It as the Common Backend

The newly added Piper `FollowJointTrajectory` path is the correct backend for MoveIt, Coordinator and Teach Manager, but two execution semantics should be fixed before relying on it.

## 2.1 Cancellation must actually stop execution

The current action server accepts cancellation, but cancellation must remain observable while waiting for the gripper to settle.

Required behavior:

```text
FJT goal
  -> claim gripper
  -> send authorized trajectory
  -> wait for result

cancel
  -> call control/gripper/stop
  -> command/verify hold if applicable
  -> terminate FJT goal as CANCELED
  -> release claim
```

Do not return `SUCCEEDED` after a cancellation request has already been accepted.

All cleanup paths must release authority correctly.

## 2.2 Do not interpret arbitrary standstill as successful grasp

The current contact-friendly completion semantics should remain: a closing gripper may successfully stop before reaching the nominal fully closed position because an object is between the fingers.

However, stable readback alone is not sufficient.

Distinguish:

```text
target already reached
    -> success

measurable motion/progress toward target
then settles before target
    -> success/contact grasp

no meaningful progress after command
    -> failure/timeout
```

This prevents lost commands, disabled hardware or transport failures from being reported as successful grasps simply because the width never changed.

Keep tolerances configurable.

---

# 3. Integrate Piper Grippers into the Coordinator Resource Model

Add logical Piper gripper resources:

```text
left_gripper
right_gripper
```

They must become valid robot/resource IDs alongside:

```text
left_arm
right_arm
both_arms
left_hand
right_hand
```

Do not represent a Piper gripper as `left_hand/right_hand`; hands and parallel grippers have different command semantics.

Extend:

- graph validation;
- scheduler resources;
- performer routing;
- stop/cancel handling;
- Activity validation;
- diagnostics/status output.

The normal dedicated-device topology should allow the scheduler to reason about the gripper explicitly.

If concurrent arm+gripper execution has not yet been hardware validated, initially use conservative resource conflicts where required and relax them only after testing.

---

# 4. Coordinator Gripper Execution

Add a dedicated Piper gripper execution path rather than routing it through `omnihand_skill_controller`.

Recommended structure:

```text
Gripper Action
├── left_hand / right_hand
│     -> OmniHand semantic skill controller
│
└── left_gripper / right_gripper
      -> Piper FollowJointTrajectory action
```

The Coordinator should use the standard FJT surface already exposed for the Piper gripper.

It must not publish directly to the legacy `/control/joint_states` interface.

Suggested logical command:

```yaml
actiontype_id: Gripper
robot_id: right_gripper

target:
  closure: 0.75
```

The executor converts normalized `closure` into the corresponding gripper joint target and sends the FJT goal.

This keeps vendor-specific dimensions outside the Activity catalogue.

---

# 5. Catalogue Actions

Add minimal reusable Piper actions, for example:

```yaml
piper_open:
  actiontype_id: Gripper
  robot_id: right_gripper
  target:
    closure: 0.0

piper_half:
  actiontype_id: Gripper
  robot_id: right_gripper
  target:
    closure: 0.5

piper_close:
  actiontype_id: Gripper
  robot_id: right_gripper
  target:
    closure: 1.0
```

But the implementation must not be limited to presets.

Arbitrary values in `[0, 1]` must be valid so actions such as:

```yaml
target:
  closure: 0.63
```

work without creating another named preset.

If payload changes are associated with a grasp/release action, retain the existing explicit payload-update flag/mechanism. Do not infer payload attachment merely from `closure > x`.

---

# 6. Piper Gripper in the Teach Manager

The Teach Manager currently treats arm trajectories and end-effector operations separately. Preserve this design.

Do **not** append the Piper gripper as an eighth arm joint and do not include it in `--source-joints`.

The Piper is an independently commanded end effector.

Add a dedicated Piper-gripper interaction mode or extend the existing end-effector UI without changing the existing OmniHand behavior.

The manager should:

1. detect which configured arm sides have a Piper gripper;
2. allow selecting the gripper when several are available;
3. display current normalized closure from readback;
4. prompt for a new value in `[0.0, 1.0]`;
5. execute the command through the Piper FJT action;
6. wait for the FJT result;
7. clearly report success, contact completion, cancellation or failure.

Example interaction:

```text
PIPER GRIPPER
resource: right_gripper
current closure: 0.18

Target closure [0.0=open, 1.0=closed]: 0.75
```

Result:

```text
right_gripper -> closure 0.75
command accepted
gripper settled at closure 0.72
```

Do not use the legacy `/control/joint_states` ingress for Teach Manager commands.

The Teach Manager should consume exactly the same FJT interface that MoveIt and the Coordinator use.

This makes the execution chain:

```text
Teach Manager
      |
      v
Piper FJT bridge
      |
      v
Device Authority
      |
      v
Arm SDK worker
      |
      v
Piper gripper
```

rather than introducing a third execution path.

---

# 7. Teach Manager Normalized Readback

Translate measured physical width back into the same normalized representation:

```python
closure = (width_open - width) / (width_open - width_closed)
```

Clamp only the displayed readback against tiny measurement excursions:

```text
0.0 <= displayed_closure <= 1.0
```

Keep the raw width available for diagnostics.

The user-facing Teach Manager should primarily show:

```text
closure: 0.00 ... 1.00
```

rather than meters.

---

# 8. Recording Integration

## 8.1 MVP

For the first implementation, manual Piper commands from the Teach Manager do not need to become part of the arm `JointTrajectory`.

Arm recording remains unchanged:

```text
joint1 ... joint7
```

This avoids affecting:

- smoothing;
- finite-difference velocity estimation;
- resampling;
- TOTG;
- arm joint limits;
- existing recording compatibility.

This is the required MVP.

## 8.2 Recommended follow-up: event-based gripper recording

After manual control is stable, allow gripper commands issued while an arm recording is active to be stored as discrete events.

Do not sample the gripper at the arm recording rate.

Example recording extension:

```json
{
  "trajectory": {
    "...": "existing arm recording"
  },
  "end_effector_events": [
    {
      "time_from_start_sec": 1.82,
      "resource": "right_gripper",
      "closure": 0.85
    },
    {
      "time_from_start_sec": 5.41,
      "resource": "right_gripper",
      "closure": 0.0
    }
  ]
}
```

This is preferable to storing hundreds of repeated gripper samples.

### Motion-onset trimming

The current recorder trims the recording to the detected first arm motion.

Gripper events must use the same final recording time origin.

If a gripper value was commanded before the retained trajectory starts, preserve the latest preceding value as the initial end-effector state at `t=0` when required.

Events inside the retained interval are simply rebased to the same time origin.

---

# 9. Playback of Recorded Gripper Events

For playback modes with a direct time mapping:

```text
as_recorded
smooth
tempo_scale
```

apply the same time transformation to the gripper event timestamps.

Example:

```text
recorded close event: 2.0 s
tempo_scale = 2x faster
replay close event:   1.0 s
```

Do not independently replay arm and gripper clocks.

For non-uniform retiming modes such as TOTG-based `speed_scale` / `maximize_speed`, do not initially guess a timestamp scaling.

Either:

1. defer synchronized gripper-event playback for these modes initially, with an explicit message; or
2. map each recorded event to its corresponding trajectory/path progress and schedule it against the retimed trajectory.

Option 2 is the desired final implementation because a grasp command is usually associated with a motion phase, not an absolute wall-clock timestamp.

Do not silently apply uniform timestamp scaling to a non-uniformly retimed arm trajectory.

---

# 10. Recording-to-Catalogue Conversion

Extend `agx_arm_recorded_to_catalogue` only after the Coordinator supports `right_gripper/left_gripper`.

For an event-containing recording, conversion should be capable of generating either:

```text
arm action
gripper action
arm action
gripper action
...
```

for coarse action boundaries,

or retain an explicit recorded end-effector event stream when precise synchronization during arm motion is required.

Do not flatten the gripper into arm waypoint dimensions.

---

# 11. Legacy Gripper Ingress

The existing direct gripper command path through `/control/joint_states` should not be used by any new subsystem.

Keep it only where required for backward compatibility/debugging.

Preferably:

- mark it clearly as legacy/debug;
- make it disable-able through the normal execution profile;
- eventually gate it through authority or remove it from production profiles.

New production paths must be:

```text
MoveIt       -> FJT
Coordinator  -> FJT
TeachManager -> FJT
```

---

# 12. Tests

## Unit tests

### Normalized mapping

Verify:

```text
closure 0.0 -> width_open
closure 0.5 -> midpoint
closure 1.0 -> width_closed
```

and reverse mapping.

Reject:

```text
-0.01
1.01
NaN
Inf
```

### FJT

Test:

- normal open;
- normal close;
- intermediate position;
- target already reached;
- contact settling before target;
- command with zero progress;
- timeout;
- cancellation during motion;
- authority loss;
- stale generation/epoch;
- stop cleanup.

### Coordinator

Test Activity parsing for:

```text
left_gripper
right_gripper
```

and reject impossible/missing resources cleanly.

Verify correct FJT routing and cancellation.

### Teach Manager

Mock the gripper FJT server and verify:

```text
0.0
0.5
1.0
```

plus invalid input.

Verify single-side and dual-side selection.

Existing OmniHand hand mode must remain unchanged.

### Recording events

If Phase 8.2 is implemented, test:

- event before detected arm onset;
- event during recording;
- multiple events;
- single-arm recording;
- duo recording;
- tempo scaling;
- absence of gripper events;
- loading old recordings without the new field.

Backward compatibility of existing recording JSON is mandatory.

---

# 13. Hardware Validation Gate

After offline/unit tests pass, validate on the real right Piper gripper in this order:

1. readback only;
2. Teach Manager `closure=0.0`;
3. `closure=1.0`;
4. several intermediate values;
5. repeated open/close;
6. cancellation during travel;
7. physical object blocking closure;
8. intentionally prevent motion and confirm it does **not** report false success;
9. Coordinator standalone gripper action;
10. Activity:
   ```text
   arm motion -> grip -> arm motion -> release
   ```
11. restart the Activity;
12. Ctrl-C during gripper execution;
13. authority/epoch invalidation;
14. simultaneous arm hold + gripper command;
15. full arm playback with manual Teach Manager gripper commands.

If synchronized recording events are implemented, finally test:

```text
record arm motion
 -> close Piper during motion
 -> continue motion
 -> open
 -> replay
```

first under `as_recorded`/`smooth`, then `tempo_scale`.

TOTG-based synchronized event playback should only be enabled once the event-to-retimed-path mapping is verified.

---

# 14. Documentation

Update at minimum:

```text
docs/control/bringups/teach_and_run.md
docs/project/architecture.md
docs/project/control_integrity_architecture.md
agx_arm_mit_demos README
agx_arm_coordination README/config documentation
```

The old statement that end effectors are transparent/not recorded should be refined to distinguish:

```text
OmniHand semantic skills
Piper normalized gripper commands
optional event-based end-effector recording
```

Document explicitly:

```text
Piper closure:
0.0 = fully open
1.0 = fully closed
```

and that this is a semantic normalized API, not the physical gripper joint coordinate.

---

# Recommended Implementation Order

## P0 — execution correctness

- fix FJT cancellation;
- add progress-aware completion;
- add normalized conversion helper and tests.

## P1 — complete system pipeline

- add `left_gripper/right_gripper`;
- Coordinator FJT routing;
- Activity validation/resources;
- catalogue actions;
- stop/cancel integration.

## P1 — Teach Manager

- gripper discovery;
- side/resource selection;
- normalized `[0,1]` input;
- normalized feedback display;
- FJT execution;
- tests.

At this point the required integration is complete.

## P2 — synchronized teaching

- record event-based gripper commands;
- common recording timebase;
- replay under direct-time playback modes;
- catalogue conversion.

## P3 — advanced retiming

- bind gripper events to path progress;
- replay them correctly through non-uniform TOTG retiming.

---

# Acceptance Criteria

The integration is complete when all of the following are true:

- `right_gripper` is a first-class Coordinator/Activity resource;
- a catalogue action can command any normalized closure in `[0,1]`;
- the Coordinator executes it through the Piper FJT interface;
- the Teach Manager can interactively command `0.0 ... 1.0`;
- the Teach Manager does not use legacy direct gripper ingress;
- FJT cancellation physically stops/cancels the operation;
- a motionless/lost command cannot be reported as successful solely because the readback is stable;
- existing arm and OmniHand teach workflows remain functional;
- existing recording files remain loadable;
- real hardware open/close/contact/cancel tests pass.

The Piper should then be considered fully integrated into the Duo execution pipeline rather than merely integrated into the driver and MoveIt layers.