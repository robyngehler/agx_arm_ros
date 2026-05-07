# Proposal: URDF-Based Gravity Compensation for Nero MIT Soft Trajectory Control

## Objective

Build a model-assisted MIT controller for the AgileX Nero arm that enables smooth, compliant trajectory following with reduced collision severity and lower apparent stiffness.

The controller should use the existing `pyAgxArm` MIT interface as the low-level command path:

```python
robot.move_mit(
    joint_index=i,
    p_des=q_ref[i],
    v_des=dq_ref[i],
    kp=kp[i],
    kd=kd[i],
    t_ff=tau_ff[i],
)
```

The first target is joint-space soft trajectory following. Cartesian impedance or admittance control can be considered later after the joint-space controller is stable and well-characterized.

## Current Findings

### Leader Mode

`set_leader_mode()` works smoothly on Nero firmware `1.06` with `NeroFW.DEFAULT`.

Observed behavior:

- The arm can be moved manually in a smooth, zero-force/drag-like mode.
- `leader_joint_angles` update while manually moving the robot.
- Normal `joint_angles` and `motor_states` may remain mostly static or stale during Leader Mode, so Leader Mode is useful for trajectory teaching but not yet reliable as a torque-data source.

### MIT Mode

MIT mode is usable for all seven joints through repeated calls to `move_mit()`.

Observed behavior:

- With low nonzero `kp` and moderate `kd`, the robot shows compliant, “gummy” behavior.
- `kd` has a large influence on the perceived resistance to manual motion.
- With `kp=0`, `kd=0`, and `t_ff=0`, the robot falls/sags with little resistance.

Conclusion:

- MIT mode does not appear to provide full internal gravity compensation when all explicit gains and feed-forward torque are zero.
- The compliant support observed with `kp>0` and/or `kd>0` is mainly caused by the MIT impedance behavior and drivetrain/friction effects, not by guaranteed standalone gravity compensation.
- For low-stiffness trajectory following, an explicit gravity feed-forward term is likely needed.

## Relevant Existing Scripts

Current experimental scripts:

```text
tests/test_nero_leader_mode_minimal.py
tests/test_nero_mit_minimal.py
tests/test_nero_compare_modes_torque_log.py
```

Relevant logs:

```text
logs/nero_mode_torque_compare_kp1_kd00.csv
logs/nero_mode_torque_compare_kp1_kd02.csv
logs/nero_mode_torque_compare_kp1_kd05.csv
logs/nero_mode_torque_compare_kp0_kd0.csv
```

The `kp0_kd0` log is the most important for gravity-compensation assessment because it removes explicit MIT stiffness, damping, and feed-forward torque.

## Available Model Sources

### URDF

The Nero arm has a full URDF model in `agx_arm_urdf`, including:

- joint hierarchy
- joint axes
- joint limits
- link inertial origins
- link masses
- inertia tensors
- visual and collision geometry

This should be sufficient to compute a rigid-body dynamics model with external libraries such as Pinocchio, KDL, RBDL, Drake, or MuJoCo.

The most important initial quantity is:

```text
tau_g(q)
```

where `tau_g(q)` is the joint-space gravity torque vector.

### MDH Kinematics

`pyAgxArm/utiles/mdh_kinematics` provides modified-DH forward kinematics.

This is useful for:

- validating frame conventions
- comparing SDK FK against URDF FK
- detecting joint sign or offset mismatches

It should not be treated as a dynamics model by itself.

## Reliability Assessment

The URDF should allow us to compute a rigid-body gravity model, but the result should not be trusted blindly on hardware.

Potential sources of mismatch:

- motor rotor inertia
- gearbox reflected inertia
- static and dynamic friction
- brake behavior
- cable and harness loads
- mounted tool or gripper mass
- payload mass and center of mass
- torque scaling and quantization in `NeroFW.DEFAULT`
- possible differences between URDF joint frames and SDK/MDH frames
- CAD-exported inertial values that may not match the physical robot exactly

Expected reliability:

```text
Good enough as an initial feed-forward model.
Not reliable enough for final low-stiffness control without validation and calibration.
```

## Proposed Development Plan

### Phase 1: URDF Gravity Calculator

Create a test script:

```text
tests/test_nero_urdf_gravity_compare.py
```

Responsibilities:

1. Load the Nero URDF.
2. Build a rigid-body model using Pinocchio or another dynamics library.
3. Connect to the Nero through `pyAgxArm`.
4. Read current joint positions and motor torque feedback.
5. Compute `tau_g(q)`.
6. Print and log:

```text
time
q_measured
tau_measured
tau_g_urdf
tau_error = tau_measured - tau_g_urdf
```

Recommended CSV output:

```text
logs/nero_urdf_gravity_compare.csv
```

### Phase 2: Kinematic Frame Validation

Before trusting the gravity vector, verify that the URDF joint ordering, signs, and zero offsets match the SDK convention.

Compare:

```text
pyAgxArm FK(q)
URDF/Pinocchio FK(q)
```

at several safe configurations.

Investigate and document:

- joint order
- joint signs
- joint zero offsets
- base frame convention
- flange/tool frame convention
- whether the MDH model and URDF describe the same physical frames

If needed, create a mapping:

```python
q_urdf = sign * q_sdk + offset
```

### Phase 3: Static Gravity Calibration

Collect static torque samples at multiple poses.

Possible procedure:

1. Place or move the robot into a pose.
2. Let it settle.
3. Record `q` and measured motor torques.
4. Compute `tau_g_urdf(q)`.
5. Repeat for many poses across the usable workspace.

Fit a simple per-joint correction model:

```text
tau_measured ≈ S * tau_g_urdf(q) + b
```

where:

```text
S = per-joint scale factor
b = per-joint torque bias
```

The first calibrated gravity feed-forward becomes:

```text
tau_ff = S * tau_g_urdf(q) + b
```

### Phase 4: MIT Gravity Feed-Forward Test

Create a test script:

```text
tests/test_nero_mit_gravity_hold.py
```

Test cases:

```text
A: kp=0, kd=0,   t_ff=0
B: kp=0, kd=0,   t_ff=tau_g_urdf
C: kp=0, kd=0.1, t_ff=tau_g_urdf
D: kp=0.5, kd=0.1, t_ff=tau_g_urdf
```

Success criteria:

- Reduced sag compared to `t_ff=0`.
- No unstable acceleration.
- Lower required `kp` and `kd` for the same holding behavior.
- Better manual compliance than MIT hold without gravity feed-forward.

### Phase 5: Soft Trajectory Controller

Create a controller module around repeated `move_mit()` commands for all seven joints.

Initial control law:

```text
tau_ff = gravity_scale * tau_g(q) + tau_bias
```

Command:

```python
robot.move_mit(
    joint_index=i,
    p_des=q_ref[i],
    v_des=dq_ref[i],
    kp=kp[i],
    kd=kd[i],
    t_ff=tau_ff[i],
)
```

Controller features:

- per-joint `kp`
- per-joint `kd`
- per-joint torque feed-forward
- torque saturation
- velocity limits
- position limits
- watchdog timeout
- emergency stop path
- optional gain ramping
- CSV logging

Recommended initial control strategy:

```text
low Kp
moderate Kd
gravity feed-forward enabled
trajectory smoothing enabled
```

### Phase 6: Collision-Friendly Behavior

Once gravity compensation and soft tracking work, add residual torque monitoring.

Approximate residual:

```text
tau_residual = tau_measured - tau_expected
```

where:

```text
tau_expected = Kp(q_ref - q) + Kd(dq_ref - dq) + tau_ff
```

Potential responses:

- reduce `kp`
- increase damping temporarily
- pause trajectory execution
- retreat along trajectory
- trigger controlled stop
- trigger emergency stop for large residuals

This should be treated as heuristic collision detection unless the torque feedback is validated against external force measurements.

## Required Investigation

Investigate whether the repository already contains useful code, parameters, or assets for dynamics or gravity compensation.

Specifically check:

```text
agx_arm_urdf/nero/**
pyAgxArm/utiles/mdh_kinematics.py
pyAgxArm/**/nero/**
pyAgxArm/docs/nero/**
```

Look for:

- existing inertial parameters
- existing gravity compensation functions
- existing inverse dynamics code
- existing torque limits
- joint sign conventions
- joint offset conventions
- firmware-specific torque scaling
- payload or end-effector configuration
- flange/tool mass definitions
- simulation models that include better inertials than the base URDF

Also verify whether the URDF includes the exact end-effector mounted on the current robot. If not, add a tool/payload model before relying on gravity compensation near the wrist.

## Recommended First Implementation Step

Implement:

```text
tests/test_nero_urdf_gravity_compare.py
```

This script should answer the immediate question:

```text
Does the URDF gravity model produce torques with the correct sign and approximate magnitude compared to measured static motor torques?
```

If the answer is yes, proceed with calibrated gravity feed-forward.

If the answer is no, first fix frame mapping, joint signs, offsets, or payload modeling before integrating the model into the MIT controller.

## Expected Outcome

A calibrated URDF-based gravity feed-forward should allow the MIT controller to run with lower stiffness and damping while still preventing sagging.

This should produce:

- smoother trajectory following
- softer interaction behavior
- lower collision forces
- less pose-dependent droop
- better manual deflection behavior during motion
- a practical foundation for future impedance or admittance control
