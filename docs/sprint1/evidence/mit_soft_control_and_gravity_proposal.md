# Historical Proposal: Nero MIT Soft Control And Gravity Feedforward

Status: consolidated historical design note.

Do not use it as an operational runbook. The current runtime behavior and teach workflow now live in
`../../control/bringups/launches.md`, `../../control/bringups/teach_and_run.md`, and the stable controller docs.

## Original goal

The early Sprint 1 question was how to turn Nero's MIT command surface into a soft trajectory and
hold controller that:

- feels more compliant than stiff position control
- can replay trajectories smoothly
- can eventually use model-assisted gravity feedforward
- stays compatible with the existing `pyAgxArm.move_mit()` command path

## What the early experiments established

### Leader mode

- `set_leader_mode()` was good enough for manual demonstration and trajectory teaching
- `leader_joint_angles` reflected manual motion reliably enough for recording
- torque- or mode-side feedback during Leader Mode was not reliable enough to treat as a physical
  torque data source

### MIT mode

- low nonzero `kp` with moderate `kd` produced the desired gummy, damping-dominant behavior
- `kd` had the strongest effect on perceived resistance
- with `kp=0`, `kd=0`, and `t_ff=0`, the arm sagged or fell instead of holding itself

### Gravity implication

- MIT mode did not provide trustworthy standalone gravity compensation with zero gains
- very soft MIT control therefore needed explicit gravity feedforward rather than relying on hidden
  firmware behavior
- a URDF-backed rigid-body model looked useful as a first approximation, but it still needed
  calibration against real hardware because of friction, payload, brake behavior, and frame
  mismatches

## Design direction that survived

The early proposals converged on the following architecture, which later evolved into the current
repo runtime:

1. keep the low-level hardware path in the existing arm runtime node instead of inventing a second
   hardware gateway
2. keep trajectory playback separate from recorded torques; let the controller own feedforward
3. add a URDF-backed gravity model and calibrate it instead of assuming zero-gain MIT hold is enough
4. validate static hold first, then replay, then higher-level teaching or demos

## What later became stable repo behavior

- the MIT controller stayed repo-owned under `src/agx_arm_mit_controller`
- gravity handling moved to the repo's current URDF-backed and calibration-backed controller path
- the teach and replay flow kept the static-hold-first validation rule
- controller gains and gravity configuration became startup-YAML-owned runtime configuration rather
  than ad hoc script parameters

## Remaining historical value

Keep this file only for early design rationale:

- why the repo stopped treating zero-gain MIT as an acceptable gravity solution
- why controller-owned gravity and feedforward replaced recorded-torque replay
- why the current MIT runtime stayed inside the repo's ROS packages instead of becoming a loose SDK
  experiment