# Repository Asset Inventory

source_document: docs/development/sprint1/assets/repository_asset_inventory.md
promotion_date: 2026-05-12

## Summary Matrix

| Surface | Location | Status | Notes |
| --- | --- | --- | --- |
| Root ROS2 driver workspace | `agx_arm_ros` | CONFIRMED | Main workspace containing control, description, MoveIt, MIT, and message packages. |
| Canonical description package | `src/agx_arm_sim/agx_arm_description` | CONFIRMED | Active ROS package for bundled Nero/Revo2 URDF/Xacro assets, camera-stand assets, and USD-adjacent content after Sprint 1 canonicalization. |
| Active MoveIt package | `src/agx_arm_moveit` | CONFIRMED | Unified MoveIt package now restricted to Nero plus `none` / `agx_gripper` / `revo2`. |
| MIT controller package | `src/agx_arm_mit_controller` | CONFIRMED | Trajectory playback, hold test, gravity model, calibration, recorder, and launch surfaces. |
| Hardware bridge | `src/agx_arm_ctrl` | CONFIRMED | ROS2 control node and launch entry points used by the rest of the workspace. |
| Removed root description path | `src/agx_arm_description` | REMOVED | Legacy duplicate package removed after canonicalizing the sim-backed package and bundling the Nero/Revo2 asset tree there. |
| Python SDK workspace | `pyAgxArm` | CONFIRMED | Nero SDK, MDH kinematics, demos, effector support for AgileX gripper and Revo2. |
| OmniHand-specific repo/package | `vendor/Omnihand-2025-SDK` | PARTIALLY_AVAILABLE | Vendored SDK with C++/Python APIs, ROS2 node/message packages, and URDF/mesh assets; not yet integrated into the main stack. |
| AGV/base description or CAD | workspace-wide | MISSING | No local AGV repo, CAD export, or mount package found. |

## Local Package Map

### `agx_arm_ros/src`

| Package | Role In Sprint 1 | Reuse Guidance |
| --- | --- | --- |
| `agx_arm_ctrl` | Runtime bridge to AgileX arm control topics and launch entry points | Reuse as-is; it is the current ROS2 hardware/control surface. |
| `agx_arm_mit_controller` | Current MIT, gravity, calibration, and trajectory replay workflow | Reuse directly; it already contains the controller-side model assumptions Sprint 1 needs to document. |
| `agx_arm_moveit` | Unified MoveIt2 config for Nero, AgileX gripper, and Revo2 | Reuse directly; note that it is KDL-based today and not yet aligned to the roadmap's future `nero_arm` / TRAC-IK naming. |
| `agx_arm_msgs` | Custom message layer used by the controller stack | Reuse directly. |
| `agx_arm_sim` | Simulation/tooling tree that now owns the canonical `agx_arm_description` package | Reuse directly for bundled Nero/Revo2 description assets, the control-compatible RViz launch, camera-stand assets, and the currently confirmed USD surface. |

### `pyAgxArm`

| Surface | Role In Sprint 1 | Reuse Guidance |
| --- | --- | --- |
| `docs/nero/nero_api.md` | Nero API and firmware behavior reference | Reuse directly for controller and SDK assumptions. |
| `pyAgxArm/utiles/mdh_kinematics` | MDH FK surface referenced by MIT gravity tooling | Reuse as a comparison/debug surface, not as the sole model source. |
| `protocols/can_protocol/drivers/effector` | AgileX gripper and Revo2 effector drivers | Reuse as local examples only; they do not provide OmniHand support. |

## File Format Inventory

| Format | Status | Confirmed Examples |
| --- | --- | --- |
| URDF | CONFIRMED | `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf` |
| Xacro | CONFIRMED | `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_with_gripper_description.xacro`, `src/agx_arm_moveit/config/agx_arm.urdf.xacro` |
| SRDF | CONFIRMED | `src/agx_arm_moveit/config/agx_arm.srdf`, `src/agx_arm_moveit/config/agx_arm.srdf.xacro` |
| DAE meshes | CONFIRMED | `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/meshes/dae/*.dae` |
| YAML configs | CONFIRMED | MoveIt, controller, and calibration configs across `src/agx_arm_moveit/config/` and `src/agx_arm_mit_controller/config/` |
| Launch files | CONFIRMED | `src/agx_arm_ctrl/launch/*.launch.py`, `src/agx_arm_moveit/launch/*.launch.py`, `src/agx_arm_mit_controller/launch/*.launch.py` |
| USD | PARTIALLY_AVAILABLE | `src/agx_arm_sim/agx_arm_description/urdf/USD/nero_gripper_d435/nero_gripper_d435.usd` plus companion configuration USDs |
| STEP | MISSING | None found in workspace |
| STL | MISSING | None found in workspace |
| OBJ | MISSING | None found in workspace |
| MJCF | MISSING | None found in workspace |

## Source Of Truth

- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf` is the canonical ROS package and package-share source for planning, control-adjacent launch flows, and simulation-oriented description assets.
- The asset tree at that path is now committed directly in-repo and pruned to `nero/`, `revo2/`, and README/license material.
- The only remaining git submodule in this workspace is `vendor/Omnihand-2025-SDK`.

### Controller Model Source Candidate

- `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py` now auto-discovers the canonical Nero URDF from package-share paths and sim-backed workspace/source-tree fallbacks.

This is usable now and aligned with the Sprint 1 canonical package decision.

## Reuse Instead Of Rewrite

- Reuse `src/agx_arm_sim/agx_arm_description` as the canonical ROS package for URDF/Xacro, meshes, the bundled Nero/Revo2 asset tree, compatibility launch files, and the confirmed local USD surface.
- Reuse `src/agx_arm_moveit` as the current planning baseline, even though later sprints will likely rename groups and change kinematics plugins.
- Reuse `src/agx_arm_mit_controller` for gravity, trajectory replay, recorder, and calibration workflows.
- Reuse `pyAgxArm` for SDK access, firmware selection, MDH comparison, and effector examples.
- Do not start OmniHand or AGV implementation from scratch inside this repo until the missing upstream/vendor artifacts are actually available.