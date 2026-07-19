# Repository Asset Inventory

promotion_origin: Sprint 1 repository and asset discovery pass
promotion_date: 2026-05-12

## Summary Matrix

| Surface | Location | Status | Notes |
| --- | --- | --- | --- |
| Root ROS2 driver workspace | `agx_arm_ros` | CONFIRMED | Main workspace containing control, description, MoveIt, MIT, and message packages. |
| Canonical description package | `src/agx_arm_sim/agx_arm_description` | CONFIRMED | Active ROS package for bundled Nero/Revo2 URDF/Xacro assets, camera-stand assets, and USD-adjacent content after Sprint 1 canonicalization. |
| Duo system staging package | `src/duo_body_description` | CONFIRMED | Temporary staging package for Duo body plus configurable arm-hand system assembly and description-only bringup; stable outputs should still promote back into the canonical packages. |
| Active MoveIt package | `src/agx_arm_moveit` | CONFIRMED | Unified MoveIt package now covers the single-arm Nero baseline plus `none` / `agx_gripper` / `revo2` / `omnihand`; the first Duo-aware `right_arm` / `left_arm` / `both_arms` planning surfaces are landed, while full `duo_hand` hardware sign-off remains pending. |
| MIT controller package | `src/agx_arm_mit_controller` | CONFIRMED | Trajectory playback, hold test, gravity model, calibration, recorder, and launch surfaces. |
| Hardware bridge | `src/agx_arm_ctrl` | CONFIRMED | ROS2 control node, launch entry points, and the repo-owned OmniHand bridge surface used by the rest of the workspace. |
| Removed root description path | `src/agx_arm_description` | REMOVED | Legacy duplicate package removed after canonicalizing the sim-backed package and bundling the Nero/Revo2 asset tree there. |
| Vendored Python SDK runtime pin | `vendor/pyAgxArm` | CONFIRMED | Vendored `pyAgxArm` submodule providing the pinned Nero SDK baseline used by runtime, install, and validation flows inside this repo. |
| OmniHand-specific repo/package | `vendor/OmniHand-Pro-2025` | PARTIALLY_AVAILABLE | Vendored SDK with C++/Python APIs, ROS2 node/message packages, and URDF/mesh assets; the repo-owned bridge exists separately and the vendor ROS surface is not the public contract. |
| AGV/base description or CAD | workspace-wide | MISSING | No local AGV repo, CAD export, or mount package found. |

## Local Package Map

### `agx_arm_ros/src`

| Package | Role In Sprint 1 | Reuse Guidance |
| --- | --- | --- |
| `agx_arm_ctrl` | Runtime bridge to AgileX arm control topics, launch entry points, and the repo-owned OmniHand bridge surface | Reuse as-is; it is the current ROS2 hardware/control surface. |
| `duo_body_description` | Current Duo staging package for body-mounted Duo description assembly and bringup | Reuse only for the current Duo staging slice; do not let it become a second long-term source of truth. |
| `agx_arm_mit_controller` | Current MIT, gravity, calibration, and trajectory replay workflow | Reuse directly; it already contains the controller-side model assumptions Sprint 1 needs to document. |
| `agx_arm_moveit` | Unified MoveIt2 config for Nero, AgileX gripper, Revo2, and the repo-owned OmniHand simulation path | Reuse directly; `nero_arm` remains the baseline single-arm group, `right_arm` / `left_arm` / `both_arms` are landed planning surfaces, and the higher-level `agx_arm_ctrl` wrappers compose the current `duo_hand` runtime slice. `tcp_link` remains distinct from the canonical `nero_tool0` flange alias. |
| `agx_arm_msgs` | Custom message layer used by the controller and OmniHand bridge stack | Reuse directly. |
| `agx_arm_sim` | Simulation/tooling tree that now owns the canonical `agx_arm_description` package | Reuse directly for bundled Nero/Revo2 description assets, the control-compatible RViz launch, camera-stand assets, and the currently confirmed USD surface. |

### `vendor/pyAgxArm`

| Surface | Role In Sprint 1 | Reuse Guidance |
| --- | --- | --- |
| `docs/nero/nero_api.md` | Nero API and firmware behavior reference | Reuse directly for controller and SDK assumptions. |
| `utiles/mdh_kinematics` | MDH FK surface referenced by MIT gravity tooling | Reuse as a comparison/debug surface, not as the sole model source. |
| `protocols/can_protocol/drivers/effector` | AgileX gripper and Revo2 effector drivers | Reuse as local examples only; they do not provide OmniHand support. |

### External `pyAgxArm` development checkout (outside this repo)

| Surface | Current Role | Reuse Guidance |
| --- | --- | --- |
| sibling or external checkout such as `../pyAgxArm` | upstream-sync, local development, rebase, and tag-preparation workspace for the team fork | Not part of this repo's source tree. Pull vendor changes there, land local patches there, push and tag the fork there, then bump `vendor/pyAgxArm` in `agx_arm_ros` to the new tag or commit. |

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
| STL | CONFIRMED | `src/duo_body_description/meshes/*.stl`, `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/meshes/*.stl`, `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand/meshes/*.STL` |
| OBJ | MISSING | None found in workspace |
| MJCF | MISSING | None found in workspace |

## Source Of Truth

- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf` is the canonical ROS package and package-share source for planning, control-adjacent launch flows, and simulation-oriented description assets.
- `src/duo_body_description` is the current staging package for body-mounted Duo system assembly and description-only bringup; promote stable outputs back into the canonical packages instead of treating it as a second long-term description source.
- The asset tree at that path is now committed directly in-repo and pruned to `nero/`, `revo2/`, and README/license material.
- The tracked vendor submodules in this workspace are `vendor/pyAgxArm` and `vendor/OmniHand-Pro-2025`.
- `vendor/pyAgxArm` is the pinned runtime source inside this repo; upstream-sync and feature development happen in a separate external `pyAgxArm` checkout before that pin is advanced.

### Controller Model Source Candidate

- `src/agx_arm_mit_controller/agx_arm_mit_controller/model_metadata.py` now auto-discovers the canonical Nero URDF from package-share paths and sim-backed workspace/source-tree fallbacks.

This is usable now and aligned with the Sprint 1 canonical package decision.

## Reuse Instead Of Rewrite

- Reuse `src/agx_arm_sim/agx_arm_description` as the canonical ROS package for URDF/Xacro, meshes, the bundled Nero/Revo2 asset tree, compatibility launch files, and the confirmed local USD surface.
- Reuse `src/duo_body_description` as the current Duo system staging package while the body-mounted description layer is still being validated; keep long-term ownership in the canonical packages.
- Reuse `src/agx_arm_moveit` as the current planning baseline; it now carries the roadmap-facing `nero_arm` / `nero_tool0` semantics in a monolithic active surface plus a TRAC-IK-based MoveIt configuration.
- Reuse `src/agx_arm_mit_controller` for gravity, trajectory replay, recorder, and calibration workflows.
- Reuse `vendor/pyAgxArm` for runtime SDK access, firmware selection, MDH comparison, and effector examples.
- Use the external `pyAgxArm` checkout only when preparing upstream pulls, local SDK changes, and the next `vendor/pyAgxArm` pin update.
- Do not start OmniHand or AGV implementation from scratch inside this repo until the missing upstream/vendor artifacts are actually available.