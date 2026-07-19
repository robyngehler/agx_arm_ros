# Nero Asset Validation

promotion_origin: Sprint 1 repository and asset discovery pass
promotion_date: 2026-05-12

component: Nero arm model, planning, and simulation assets
repository_or_source: agx_arm_ros (`src/agx_arm_sim/agx_arm_description`, `src/agx_arm_moveit`, `src/agx_arm_mit_controller`) and `vendor/pyAgxArm` (`docs/nero`, MDH tooling)
inspection_date: 2026-05-21
status: PARTIALLY_AVAILABLE
found_artifacts:
- fixed in-repo Nero/Revo2 asset tree under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/`
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf`
- Nero Xacro variants for AgileX gripper and Revo2 under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/`
- Nero DAE meshes under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/meshes/dae/`
- Unified MoveIt package `src/agx_arm_moveit` with `arm_type:=nero` support
- MIT controller gravity/model tooling under `src/agx_arm_mit_controller`
- Confirmed compiled USD asset `src/agx_arm_sim/agx_arm_description/urdf/USD/nero_gripper_d435/nero_gripper_d435.usd`
- vendored pyAgxArm Nero API docs and MDH/FK comparison surfaces
missing_artifacts:
- a promoted Isaac asset set beyond the currently confirmed sim-backed package content
- stable promoted documentation for the Humble / Jetson TRAC-IK source-build fallback beyond the current Sprint 3 working note
- explicit mounting-orientation and payload variants beyond current Nero + gripper/Revo2 surfaces
interface_notes:
- `src/agx_arm_moveit` now exposes `arm_type:=nero` with `effector_type:=none|agx_gripper|revo2|omnihand`; the OmniHand branch currently covers the simulation, RViz, SRDF, and fake `ros2_control` path
- `src/agx_arm_ctrl` and `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py` now default to `arm_type:=nero` and no longer advertise Piper-family choices
- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf` now provides the canonical `nero_tool0` flange alias, `src/agx_arm_moveit/config/agx_arm.urdf.xacro` keeps `tcp_link` as the TCP/planning frame, and `display_control.launch.py` now always publishes the `nero_tool0` to `tcp_link` transform for the built-in RViz-compatible Nero flows
- `src/agx_arm_moveit/config/kinematics.yaml` now targets `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` for the single active `nero_arm` planning group
- `src/agx_arm_moveit/config/agx_arm.srdf.xacro` and `config/agx_arm.srdf` now keep only the monolithic `nero_arm` planning group in the active MoveIt surface
- `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py` now aligns Nero auto-discovery with the canonical sim-backed description package plus sim-backed workspace/source-tree fallbacks
- `src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_model.py` now prefers `nero_tool0` and falls back to `link7`, `gripper_flange`, `tool0`, and `flange` for existing controller-side workflows
- `scripts/moveit_profile_smoke_test.sh` now supports the external `~/workspace/trac_ik_ws/install/setup.bash` overlay and the 2026-05-21 six-profile sweep confirmed ready-state startup with no TRAC-IK plugin-load failures
risks:
- The current host still hits a `move_group` teardown crash after SIGINT on Humble/aarch64 even though TRAC-IK loads and the live `/compute_ik` path succeeds
- The Humble / Jetson TRAC-IK source-build path is documented and validated locally, but it still lives in Sprint 3 working notes rather than a promoted stable operations guide
- Isaac support is real but uneven: the canonical sim-backed package has a confirmed USD artifact, but the broader promoted Isaac path is still incomplete
recommended_next_action:
- keep the canonical Nero URDF/Xacro ownership in `src/agx_arm_sim/agx_arm_description` and avoid reintroducing either a second discoverable `agx_arm_description` package or a separate `agx_arm_urdf` submodule dependency
- validate a full pose-planning path on top of the now-working TRAC-IK baseline, then audit the MoveIt-to-MIT path
- isolate or upstream the Humble/aarch64 `move_group` shutdown crash before treating timeout-driven smoke runs as clean evidence
- promote only the confirmed local USD artifacts into the stable documentation, and treat the rest of Isaac coverage as still open
related_sprint: 1

## Validation Notes

### Planning Readiness

Local Nero URDF/Xacro and mesh assets are good enough for current MoveIt use. `src/agx_arm_moveit/README.md` documents `arm_type:=nero` for both model-only and real-arm flows, and the package contains the expected `agx_arm.urdf.xacro`, `agx_arm.srdf.xacro`, controller YAMLs, and launch files.

### Controller Readiness

Local Nero assets are also good enough for the current MIT gravity and playback workflow. The historical workflow lineage in `docs/sprint2/evidence/mit_runtime_history.md` and `src/agx_arm_mit_controller/README.md` is built around the canonical Nero URDF plus `config/nero_gravity_calibration.json`.

### Simulation Readiness

Simulation readiness is only partial right now:

- the canonical `src/agx_arm_sim/agx_arm_description` package explicitly ships USD assets and camera-mount logic,
- but the workspace still does not expose a broader stable top-level Isaac asset story beyond the confirmed `nero_gripper_d435` path.

That means Sprint 1 should record Isaac assets as present but not yet complete.