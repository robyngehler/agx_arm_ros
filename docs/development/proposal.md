# Nero MIT Soft Trajectory Controller Proposal

Status: historical design proposal kept for early MIT-controller exploration.

Use `docs/development/mit_trajectory_recording_and_playback.md` for the validated workflow and `docs/development/nero_physical_ai_progress.md` for current overall status.

## Goal

Build a custom MIT-based joint impedance controller for AgileX Nero that can follow trajectories smoothly and compliantly, with reduced collision severity compared to stiff position control.

The initial target is **soft joint-space trajectory following** using `pyAgxArm.move_mit()`:

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

The controller should eventually support:

- low-stiffness trajectory replay,
- adjustable joint impedance,
- damping-dominant compliant behavior,
- optional gravity/feed-forward compensation,
- collision/residual torque monitoring,
- trajectory recording from Leader Mode.

---

## Current Findings

### Hardware / Firmware

Tested robot:

- AgileX Nero
- Software version: `1.06`
- Required SDK config: `NeroFW.DEFAULT`
- CAN interface used in tests: `socketcan`, channel `can_nero`

Firmware `1.06` should not use `NeroFW.V111`.

---

## Leader Mode Findings

`set_leader_mode()` works and provides smooth hand-guiding behavior.

Observed behavior:

- The robot can be moved smoothly by hand in Leader Mode.
- `get_leader_joint_angles()` changes as expected.
- `get_joint_angles()` and `get_motor_states()` may not reflect the same moving state during Leader Mode.
- Status feedback may remain visually unchanged, for example still reporting `MOVE_J` or stale mode fields.

Conclusion:

- Leader Mode is usable for hand-guided trajectory recording.
- Leader Mode likely uses firmware-internal support/compensation.
- Internal Leader Mode compensation is not exposed through the public SDK API.

Recommended use:

- Use Leader Mode for manual demonstration and trajectory recording.
- Do not rely on Leader Mode motor torque logs as direct physical torque measurements unless this is verified further.

---

## MIT Mode Findings

MIT mode was tested with all seven joints using position hold commands at the current pose.

### Observed Gain Behavior

With:

```text
kp = 1.0
kd = 0.0 .. 0.5
t_ff = 0.0
```

the robot does not immediately collapse and behaves like a low-stiffness, gummy impedance system.

Main observation:

- `kd` has the strongest effect on perceived resistance.
- Near-zero `kd` feels very flexible.
- `kd > 1.0` feels significantly more resistant/stiff.
- This is likely damping-dominant behavior, not true static stiffness.

### Gravity Compensation Test

A dedicated test was run with:

```text
kp = 0.0
kd = 0.0
t_ff = 0.0
```

Result:

- The robot fell/sagged without meaningful resistance.
- Joint 4 drifted by approximately `0.397 rad` (`22.7 deg`) in the uploaded `kp0_kd0` log.
- MIT mode torque feedback dropped strongly compared to Normal Mode.

Conclusion:

- MIT mode does **not** provide full standalone gravity compensation when `kp = 0`, `kd = 0`, and `t_ff = 0`.
- Earlier support behavior with `kp > 0` and/or `kd > 0` came mainly from the MIT impedance loop, drivetrain/friction, and possibly low-level motor behavior.
- A real low-impedance trajectory controller will likely need explicit gravity/feed-forward compensation if we want very low gains without sagging.

---

## Relevant Scripts

### Existing / Tested

#### `tests/test_nero_leader_mode_minimal.py`

Purpose:

- Connect to Nero.
- Verify firmware.
- Enable robot.
- Enter Leader Mode.
- Print `leader_joint_angles`.
- Return to Normal Mode on cleanup.

Use case:

- Validate hand-guiding.
- Record manually guided joint trajectories in the next iteration.

#### `tests/test_nero_mit_minimal.py`

Purpose:

- Connect to Nero.
- Verify firmware.
- Enable robot.
- Read motor states.
- Hold selected/all joints using `move_mit()`.

Current improvements needed:

- Use `argparse` with `nargs="+"` for `--joints` instead of `type=list`.
- Ensure periodic logging is not accidentally suppressed by print-timer indentation.
- Add explicit `set_motion_mode("mit")` before the MIT loop.
- Add optional CSV logging.

#### `tests/test_nero_mit_mode_probe.py`

Purpose:

- Explicitly switch to MIT mode.
- Send a small joint step.
- Verify whether `mode_feedback` reports MIT and whether the commanded joint moves.

Use case:

- Diagnose MIT mode activation and command acceptance.

#### `tests/test_nero_compare_modes_torque_log.py`

Purpose:

- Log Normal, Leader, and MIT phases to CSV.
- Record joint positions, velocities, motor torques, currents, leader angles, status fields, and MIT gains.
- Compare behavior across modes.

Use case:

- Validate support behavior.
- Estimate whether measured torques can be used for empirical gravity compensation.

---

## Relevant Logs

Uploaded/tested logs:

- `nero_mode_torque_compare_kp1_kd00.csv`
- `nero_mode_torque_compare_kp1_kd02.csv`
- `nero_mode_torque_compare_kp1_kd05.csv`
- `nero_mode_torque_compare_kp0_kd0.csv`

Summary:

| Log | MIT gains | Key observation |
|---|---:|---|
| `kp1_kd00` | `kp=1.0`, `kd=0.0`, `t_ff=0.0` | Flexible hold, visible drift under manual interaction |
| `kp1_kd02` | `kp=1.0`, `kd=0.2`, `t_ff=0.0` | More controlled gummy behavior |
| `kp1_kd05` | `kp=1.0`, `kd=0.5`, `t_ff=0.0` | Stronger damping/resistance |
| `kp0_kd0` | `kp=0.0`, `kd=0.0`, `t_ff=0.0` | Robot sagged/fell; no full gravity compensation in MIT mode |

Important interpretation:

- Nonzero MIT gains can prevent collapse and provide compliant support.
- Zero MIT gains with zero feed-forward do not hold the robot.
- Therefore, explicit gravity compensation is likely required for very soft control.

---

## Proposed Controller Architecture

Create a new module/package for MIT-based soft trajectory control.

Suggested structure:

```text
tests/
  test_nero_leader_mode_minimal.py
  test_nero_mit_minimal.py
  test_nero_mit_mode_probe.py
  test_nero_compare_modes_torque_log.py

pyAgxArm/extensions/nero_soft_control/
  __init__.py
  trajectory_buffer.py
  mit_soft_controller.py
  gravity_compensation.py
  safety.py
  logging.py
```

Alternative: keep first implementation under `tests/` or `examples/` until stable.

---

## Controller Design

### Phase 1: Joint-Space Soft Hold

Implement all-joint MIT hold:

```text
q_ref = q_current_at_enable
dq_ref = 0
t_ff = 0
kp = low
kd = moderate
```

Initial suggested gains:

```text
joint 1: kp=1.0, kd=0.1..0.3
joint 2: kp=1.0, kd=0.2..0.5
joint 3: kp=1.0, kd=0.1..0.3
joint 4: kp=1.0, kd=0.2..0.5
joint 5: kp=0.5..1.0, kd=0.05..0.2
joint 6: kp=0.5..1.0, kd=0.05..0.2
joint 7: kp=0.3..0.8, kd=0.03..0.15
```

### Phase 2: Soft Trajectory Replay

Input:

```text
q_ref(t)
dq_ref(t)
optional ddq_ref(t)
```

Control:

```text
tau_cmd = Kp * (q_ref - q) + Kd * (dq_ref - dq) + tau_ff
```

Implemented through `move_mit()`:

```python
for i in range(7):
    robot.move_mit(
        joint_index=i + 1,
        p_des=q_ref[i],
        v_des=dq_ref[i],
        kp=kp[i],
        kd=kd[i],
        t_ff=tau_ff[i],
    )
```

Start with:

```text
tau_ff = 0.0
```

Then add gravity/feed-forward compensation once available.

### Phase 3: Gravity / Feed-Forward Compensation

Goal:

```text
tau_ff = tau_g(q)
```

Options:

1. Empirical gravity compensation:
   - collect static torque samples in Normal Mode,
   - fit/interpolate `tau_g_est(q)`,
   - validate in MIT mode with low gains.

2. URDF/dynamics-based gravity compensation:
   - use URDF inertial parameters if available,
   - compute `g(q)` using Pinocchio, KDL, RBDL, Drake, or similar.

3. Hybrid:
   - use URDF model as baseline,
   - correct with empirical residuals.

### Phase 4: Collision-Friendly Behavior

Add monitoring:

```text
position error
velocity
measured torque
estimated gravity torque
residual torque
```

Possible logic:

```text
if residual torque exceeds threshold:
    reduce Kp
    increase damping
    pause trajectory
    trigger controlled stop
```

This should be conservative at first. Motor-reported torque is not guaranteed to be a clean external torque estimate.

---

## Safety Requirements

Minimum required safety features:

- joint position limits,
- velocity limits,
- torque/feed-forward saturation,
- command watchdog,
- stale feedback detection,
- smooth gain ramp-up/ramp-down,
- emergency stop path,
- safe cleanup to Normal Mode,
- optional disable only when mechanically supported,
- logging of all commands and feedback.

MIT mode should never be entered with aggressive gains by default.

---

## Required Investigation: Gravity and Dynamics Support in the Repo

Investigate whether the repository already contains anything useful for gravity compensation or dynamic model extraction.

Look specifically for:

### 1. URDF inertials

Check whether Nero URDF files contain usable:

```text
mass
center of mass / origin
inertia tensor
joint axes
joint limits
transmission or gear information
```

If inertial parameters exist, evaluate whether they are plausible enough for gravity compensation.

### 2. Built-in kinematics

Investigate existing FK/DH utilities in `pyAgxArm`.

Questions:

- Is the built-in Nero FK based on DH parameters only?
- Are link masses or CoM values available anywhere?
- Can FK utilities be reused for gravity compensation or only for pose calculation?

### 3. Driver torque scaling

Inspect Nero DEFAULT driver implementation.

Questions:

- How is `t_ff` encoded for firmware `1.06`?
- What torque limits/ranges are used per joint?
- Is torque feedback decoded in physical Nm?
- Are there hidden offsets, current scaling, or friction compensation constants?

### 4. Firmware mode behavior

Inspect mode switching and MIT behavior.

Questions:

- Does `move_mit()` always call `_maybe_set_motion_mode("mit")`?
- When should `set_motion_mode("mit")` be called explicitly?
- Why can `mode_feedback` remain stale or not reflect Normal/Leader phases?
- Are motor states stale during Leader Mode?

### 5. Existing examples or extensions

Search for existing:

```text
gravity compensation
impedance
admittance
MIT controller
leader/follower
trajectory replay
torque control
dynamic model
```

If no dynamic model exists, implement a clean abstraction so gravity compensation can later be swapped between empirical and model-based implementations.

---

## Immediate Next Steps

1. Fix and clean `test_nero_mit_minimal.py`.
2. Add robust CSV logging to all MIT tests.
3. Create a static gravity calibration script:
   - manually place robot in several poses,
   - record Normal Mode static torque,
   - record joint angles,
   - save to CSV.
4. Build first `mit_soft_controller.py`:
   - all-joint MIT loop,
   - configurable per-joint gains,
   - trajectory input,
   - safety limits.
5. Add initial trajectory recording from Leader Mode.
6. Replay recorded Leader Mode trajectories with low-gain MIT control.
7. Investigate and implement gravity compensation:
   - empirical first,
   - URDF/dynamics-based second if feasible.

---

## Current Working Assumption

The first useful controller does not need a perfect full dynamics model.

A practical first version should use:

```text
low Kp
moderate Kd
t_ff = 0.0
safety limits
smooth trajectory interpolation
```

Then add gravity compensation if low gains cause sagging or poor tracking.

For very compliant behavior, especially with `kp` and `kd` close to zero, explicit gravity compensation will be required.
