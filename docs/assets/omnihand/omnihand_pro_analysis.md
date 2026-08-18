# OmniHand Pro SDK Analysis

status: REFERENCE — deferred work package
last_updated: 2026-08-18

## Scope

This document records the capabilities and limitations discovered in the current OmniHand Pro Vendor SDK fork:

```text
robyngehler/OmniHand-Pro-2025
branch: jetson-orin-socketcan
```

The purpose is to preserve the findings for a later control/backend sprint.

These findings do **not** imply that the current runtime should adopt the more advanced control modes. The production path is `POSITION_GOAL` through `set_all_active_joint_angles`, and the ROS authority contract (§11) is deliberately richer than that backend uses, so the backend can be replaced later without touching it.

Everything in §4-§10 is **unvalidated capability**, not a plan. Nothing here is scheduled.

---

# 1. Current ROS integration

The current ROS bridge primarily uses:

```text
set_all_active_joint_angles(...)
```

This path accepts active joint angles in radians, converts them through the Vendor kinematics layer to actuator positions, and sends a position-control command to the hand.

The current FJT integration therefore behaves closer to:

```text
FollowJointTrajectory
    -> final target configuration
    -> Vendor position command
```

than to a full time-parameterized joint trajectory controller.

The current reactive controller instead produces incremental position targets in a tactile feedback loop and repeatedly forwards new target configurations.

These two semantics should remain distinct.

---

# 2. Vendor control modes

The Vendor SDK exposes more control capability than the current ROS wrapper uses.

The available control modes include:

```text
POSITION
VELOCITY
TORQUE
POSITION_TORQUE
VELOCITY_TORQUE
POSITION_VELOCITY_TORQUE
```

Therefore the hand is not fundamentally limited to static position targets.

However, availability of a low-level mode is not equivalent to having a complete ROS-joint-level controller implementation.

---

# 3. Position control

## 3.1 Active joint-angle API

The convenience path:

```text
set_all_active_joint_angles(q)
```

performs approximately:

```text
active joint angles [rad]
        |
        v
Vendor joint -> actuator conversion
        |
        v
12 actuator position targets
        |
        v
position-control CAN request
```

### Important limitation

No host-side trajectory interpolation, duration handling, velocity profile, or acceleration profile is provided by this API.

It is a target-position command.

Whether the embedded motor firmware performs additional internal trajectory shaping is not established by the inspected host-side repository and must not be assumed.

---

# 4. Velocity control

The SDK exposes dedicated velocity APIs including all-actuator velocity commands.

This means a future velocity-streaming backend is technically conceivable.

The all-joint velocity path has an attractive property:

- the SDK can send velocity targets for the complete actuator set in a common control request.

However, two major pieces are missing before this can be used as a correct ROS joint-space trajectory backend.

## 4.1 Vendor velocity units are not sufficiently documented

The low-level velocity API uses integer/raw actuator velocity values.

The inspected public API documentation does not provide a sufficiently clear physical conversion such as:

```text
raw velocity unit -> actuator rad/s
```

or an equivalent calibrated physical unit.

This must be characterized or obtained from the Vendor before ROS joint velocities can be mapped safely.

## 4.2 No joint-velocity to actuator-velocity conversion is provided

The Vendor kinematics layer provides position mappings of the form:

```text
joint positions -> actuator positions
actuator positions -> joint positions
```

but no corresponding differential mapping was found for:

```text
joint velocities -> actuator velocities
```

For a coupled hand this generally requires a configuration-dependent differential relationship:

```text
m = f(q)
dm/dt = J_mq(q) * dq/dt
```

The required Jacobian or equivalent mapping is not exposed by the current SDK.

Therefore ROS `JointTrajectoryPoint.velocities` cannot simply be copied into the Vendor actuator-velocity API.

Any such mapping would be a new control/kinematics feature requiring dedicated derivation and validation.

---

# 5. Mixed position / velocity / torque control

The SDK also exposes mixed control modes, including position + velocity + torque.

Conceptually this could fit a trajectory controller well.

However, the current implementation has a practical frame-size constraint:

- a single mixed-control request supports only a subset of the full 12-actuator hand;
- the complete hand would require multiple requests.

This means a 12-DoF mixed update is not necessarily atomically applied from the host side.

Before using this mode for coordinated trajectory execution, the following would have to be characterized:

- inter-frame skew between actuator groups;
- firmware update semantics;
- whether both requests become effective immediately or at a common cycle boundary;
- effect on coupled joints;
- worst-case command latency and jitter.

For the current refactor MVP, this complexity is intentionally deferred.

---

# 6. Synchronous request behavior

The relevant Vendor control paths use synchronous request/response handling.

Conceptually:

```text
send CAN command
    |
    v
wait for matching response / acknowledgement
    |
    v
return from SDK call
```

The transport layer has a finite request timeout.

This matters for any future command-streaming design because the sustainable command frequency is not determined by the nominal 5 Mbit/s CAN-FD data rate alone.

The relevant loop time is closer to:

```text
T_command =
    host processing
  + request transmission
  + firmware handling
  + response transmission
  + SDK synchronization overhead
```

Therefore no streaming rate such as 20 Hz, 50 Hz, or 100 Hz should be selected by assumption.

It must be measured.

---

# 7. Position streaming as a possible future backend

One possible future trajectory backend is:

```text
JointTrajectory
      |
      v
host interpolation q_d(t)
      |
      v
periodic set_all_active_joint_angles(q_d)
```

This would avoid the need for actuator-velocity conversion.

However, it is currently unknown how the embedded controller behaves when position targets are repeatedly replaced at a relatively high rate.

Possible firmware behaviors include:

- smooth retargeting;
- restart of an internal motion profile;
- command coalescing;
- increased latency;
- visible jerk;
- saturation or queueing.

Therefore position streaming is possible from an API perspective, but not yet validated as a trajectory-control strategy.

---

# 8. Recommended later characterization for position streaming

When this topic is revisited, test a fixed smooth reference trajectory at several update periods, for example:

```text
100 ms
50 ms
20 ms
10 ms
```

These values are test points, not recommended production settings.

Measure at minimum:

- SDK call RTT;
- p50 / p95 / p99 / maximum RTT;
- timeout rate;
- CAN errors/drops;
- actual joint position trajectory;
- derived joint velocity;
- tracking error;
- visible or measured jerk;
- CPU usage of the bridge/SDK worker;
- command queue behavior;
- effect of replacing an in-progress target.

The production sampling period should be chosen from these measurements.

---

# 9. Recommended later characterization for velocity control

Velocity mode should be characterized independently before integration with MoveIt or FJT.

Suggested sequence:

1. isolate a safe actuator/joint configuration;
2. command small raw Vendor velocity values;
3. measure raw actuator position change over time;
4. measure reported actuator velocity if available;
5. map the resulting active-joint motion;
6. repeat over several magnitudes and signs;
7. determine unit scale, deadband, saturation, and direction convention.

Only after the low-level units are understood should a joint-space differential mapping be introduced.

---

# 10. Differential kinematics extension

A future velocity-based backend would require an extension roughly equivalent to:

```text
joint-space desired velocity dq
        |
        v
configuration-dependent differential kinematics
        |
        v
actuator velocity dm
        |
        v
Vendor velocity control
```

The existing position conversion can potentially serve as the basis for deriving the required Jacobian, analytically or numerically.

However, this is not a trivial wrapper change.

It requires:

- mathematical derivation or robust numerical differentiation;
- singularity/conditioning analysis;
- sign and unit validation;
- rate and saturation handling;
- coupling validation;
- hardware tests for all relevant fingers/configurations.

This work belongs in a dedicated control sprint, not in the current refactor closure.

---

# 11. ROS-side architecture implication

The ROS contract should preserve information even when the current backend does not yet use all of it.

Recommended messages:

```text
DeviceCommandStamp
AuthorizedJointTrajectory
HandJointTarget
```

## `DeviceCommandStamp`

Common authority metadata:

```text
owner_id
device_epoch
unit_safety_epoch
sequence
```

## `AuthorizedJointTrajectory`

Carries:

```text
DeviceCommandStamp
trajectory_msgs/JointTrajectory
```

This preserves:

- positions;
- optional velocities;
- optional accelerations;
- time-from-start information.

The current backend may still execute only the final position target.

A future backend can implement real interpolation or velocity control without changing the upstream authority contract.

## `HandJointTarget`

Carries:

```text
DeviceCommandStamp
joint_names
positions
```

This remains the deliberately simple primitive for reactive tactile control.

It does not claim trajectory semantics and does not require the Vendor velocity interface.

---

# 12. Recommended backend model

The future OmniHand trajectory layer can support multiple explicit backends:

```text
OmniHandTrajectoryBackend
|
+-- POSITION_GOAL
|
+-- POSITION_STREAM          [future / characterized]
|
+-- VELOCITY_STREAM          [future / characterized]
|
+-- MIXED_CONTROL            [future / characterized]
```

## Production MVP

Use:

```text
POSITION_GOAL
```

with the known position-target Vendor API.

## Future development

Enable other backends only after their timing, units, kinematics, and hardware behavior are measured.

The backend selection should not change the ROS authority model.

---

# 13. Known documentation discrepancy

A discrepancy was observed between Vendor documentation and implementation regarding the number of active degrees of freedom accepted by the active-joint-angle API.

The current implementation and example usage correspond to a 12-DoF active-joint interface, while at least one API documentation location refers to a different list length.

For integration decisions, the implementation and verified hardware behavior should be treated as authoritative until the Vendor documentation is corrected.

This should be rechecked whenever the Vendor fork is updated.

---

# 14. Deferred work package

The following work should be explicitly deferred until after the refactor runtime is stable and productive:

1. characterize position-retargeting behavior;
2. measure synchronous command RTT and jitter;
3. characterize Vendor velocity units;
4. derive/validate joint-to-actuator velocity mapping;
5. evaluate position streaming;
6. evaluate velocity streaming;
7. evaluate mixed control;
8. determine whether a real hand-side trajectory controller materially improves the demo/use cases;
9. only then decide whether to extend the Vendor SDK.

---

# 15. Current recommendation

For the current refactor:

```text
standard FollowJointTrajectory
        |
        v
AuthorizedJointTrajectory
        |
        v
current position-goal backend
```

and:

```text
reactive tactile controller
        |
        v
HandJointTarget
        |
        v
current position-target backend
```

Both use:

```text
DeviceCommandStamp
```

and both are admitted through the same device-authority mechanism.

This closes the authority architecture now while avoiding a premature low-level control redesign.

The Vendor SDK clearly offers more possibilities, but exploiting them correctly requires enough kinematic, timing, and hardware validation that it should be treated as a separate engineering task rather than hidden inside the refactor MVP.
