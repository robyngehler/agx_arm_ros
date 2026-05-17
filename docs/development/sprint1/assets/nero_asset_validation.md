# Nero Asset Validation

component: Nero arm model, planning, and simulation assets
repository_or_source: agx_arm_ros (`src/agx_arm_sim/agx_arm_description`, `src/agx_arm_moveit`, `src/agx_arm_mit_controller`) and pyAgxArm (`docs/nero`, MDH tooling)
inspection_date: 2026-05-11
status: PARTIALLY_AVAILABLE
found_artifacts:
- fixed in-repo Nero/Revo2 asset tree under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf`
- Nero Xacro variants for AgileX gripper and Revo2 under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/`
- Nero DAE meshes under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/meshes/dae/`
- Unified MoveIt package `src/agx_arm_moveit` with `arm_type:=nero` support
- MIT controller gravity/model tooling under `src/agx_arm_mit_controller`
- Confirmed compiled USD asset `src/agx_arm_sim/agx_arm_description/urdf/USD/nero_gripper_d435/nero_gripper_d435.usd`
- pyAgxArm Nero API docs and MDH/FK comparison surfaces
missing_artifacts:
- a promoted Isaac asset set beyond the currently confirmed sim-backed package content
- roadmap-aligned naming for planning group (`nero_arm`) and tool frame (`nero_tool0`)
- TRAC-IK configuration; current MoveIt config uses KDL
- explicit mounting-orientation and payload variants beyond current Nero + gripper/Revo2 surfaces
interface_notes:
- `src/agx_arm_moveit` now exposes only `arm_type:=nero` with `effector_type:=none|agx_gripper|revo2`
- `src/agx_arm_ctrl` and `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py` now default to `arm_type:=nero` and no longer advertise Piper-family choices
- `src/agx_arm_moveit/config/kinematics.yaml` uses `kdl_kinematics_plugin/KDLKinematicsPlugin`
- `src/agx_arm_moveit/config/agx_arm.srdf.xacro` uses the generic planning group name `arm` and Nero-specific flange semantics based on `link7`
- `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py` now aligns Nero auto-discovery with the canonical sim-backed description package plus sim-backed workspace/source-tree fallbacks
- `src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_model.py` searches for frame candidates such as `link7`, `gripper_flange`, `tool0`, and `flange`; the roadmap's future `nero_tool0` convention is not yet enforced in the current codebase
risks:
- current MoveIt and frame naming do not yet match the roadmap's future canonical naming, which can cause later migration overhead
- Isaac support is real but uneven: the canonical sim-backed package has a confirmed USD artifact, but the broader promoted Isaac path is still incomplete
recommended_next_action:
- keep the canonical Nero URDF/Xacro ownership in `src/agx_arm_sim/agx_arm_description` and avoid reintroducing either a second discoverable `agx_arm_description` package or a separate `agx_arm_urdf` submodule dependency
- promote only the confirmed local USD artifacts into the stable documentation, and treat the rest of Isaac coverage as still open
related_sprint: 1
related_child_document: docs/development/sprint1/assets/nero_asset_validation.md

## Validation Notes

### Planning Readiness

Local Nero URDF/Xacro and mesh assets are good enough for current MoveIt use. `src/agx_arm_moveit/README.md` documents `arm_type:=nero` for both model-only and real-arm flows, and the package contains the expected `agx_arm.urdf.xacro`, `agx_arm.srdf.xacro`, controller YAMLs, and launch files.

### Controller Readiness

Local Nero assets are also good enough for the current MIT gravity and playback workflow. The validated workflow in `docs/development/sprint2/control/mit_trajectory_recording_and_playback.md` and `src/agx_arm_mit_controller/README.md` is built around the canonical Nero URDF plus `config/nero_gravity_calibration.json`.

### Simulation Readiness

Simulation readiness is only partial right now:

- the canonical `src/agx_arm_sim/agx_arm_description` package explicitly ships USD assets and camera-mount logic,
- but the workspace still does not expose a broader stable top-level Isaac asset story beyond the confirmed `nero_gripper_d435` path.

That means Sprint 1 should record Isaac assets as present but not yet complete.