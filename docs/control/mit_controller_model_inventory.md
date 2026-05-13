# MIT Controller Model Inventory

source_document: docs/development/sprint1/control/mit_controller_model_inventory.md
promotion_date: 2026-05-12

component: Nero MIT controller model, gravity, and trajectory interfaces
repository_or_source: `src/agx_arm_mit_controller`, related docs in `docs/development/`, and Nero API docs in `pyAgxArm`
inspection_date: 2026-05-11
status: CONFIRMED
found_artifacts:
- controller package `src/agx_arm_mit_controller`
- validated workflow doc `docs/development/mit_trajectory_recording_and_playback.md`
- default params file `src/agx_arm_mit_controller/config/nero_mit_controller_defaults.yaml`
- launch file `src/agx_arm_mit_controller/launch/start_nero_mit_controller.launch.py`
- model discovery logic in `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py`
- gravity backend wrapper in `src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_model.py`
- recorder, playback, position-hold, compare-gravity, and calibration tools
- tests covering model metadata, gravity model creation, trajectory buffer/io, and feedforward calibration
missing_artifacts:
- explicit payload model extension for OmniHand and AGV-mounted variants
- explicit mount-orientation parameterization for non-default base mounting
- payload and mounting-pose metadata for future OmniHand and AGV-mounted variants
interface_notes:
- command input is `trajectory_msgs/JointTrajectory`
- expected joint ordering is `joint1` through `joint7`
- controller interfaces from the package README:
  - subscribe: `feedback/joint_states`
  - publish: `control/move_mit`
  - subscribe: `~/joint_trajectory`
  - publish: `~/reference_joint_states`
  - service: `~/enable`
  - service: `~/hold_current`
  - service: `~/cancel_trajectory`
- default gravity behavior is Pinocchio-based with `gravity_feedforward_sign: -1.0`
- when `gravity_urdf_path` and `calibration_file` are empty, the controller auto-discovers the canonical Nero URDF and `config/nero_gravity_calibration.json`
- the launch file now prefers the source YAML in a colcon workspace so parameter tuning does not require a rebuild
risks:
- the ignored root legacy description tree can still drift away from the canonical sim-backed package if both copies are edited
- controller-side model assumptions currently match standalone Nero better than future OmniHand or AGV-mounted variants
- current frame/tool naming in code is not yet aligned with the roadmap's future canonical naming
recommended_next_action:
- keep controller URDF discovery aligned with the sim-backed canonical `agx_arm_description` package
- add a future-facing extension point for payload and mounting-pose metadata before Sprint 5, 8, and 9 work begins
- keep the validated standalone Nero workflow intact while the broader physical-AI stack is still being discovered
related_sprint: 1

## Current Controller Assumptions

### Canonical Joint Space

The current controller assumes a seven-joint Nero model with joint names:

```text
joint1 joint2 joint3 joint4 joint5 joint6 joint7
```

This matches the root MoveIt package and pyAgxArm Nero definitions.

### Canonical URDF Lookup

`model_metadata.py` currently searches these categories of paths for the Nero URDF:

1. installed package share for `agx_arm_description`
2. workspace search roots under the current working directory
3. source-tree fallback paths under `src/agx_arm_sim/agx_arm_description`

That keeps the current workflow robust in practice while matching the Sprint 1 canonical package decision.

### Gravity Workflow State

Recent history already pushed the controller into a usable standalone Nero state:

- `b511952` added gravity calibration files, proposal docs, and improved model metadata.
- `f887363` added the position-hold tool and stabilized gravity-assisted playback.
- `b4976bc` documented the validated workflow.

That means Sprint 1 does not need to invent a MIT controller model path. It only needs to document the one that already exists and make its assumptions explicit.