# OmniHand Asset Validation

component: OmniHand Pro SDK, model, and integration surfaces
repository_or_source: current workspace (`agx_arm_ros`, `pyAgxArm`, `vendor/Omnihand-2025-SDK`)
inspection_date: 2026-05-11
status: PARTIALLY_AVAILABLE
found_artifacts:
- vendored SDK repository at `vendor/Omnihand-2025-SDK`
- C++ and Python SDK surfaces documented in `README.md`, `document/en/API_CPP.md`, and `document/en/API_PYTHON.md`
- ROS2 package `vendor/Omnihand-2025-SDK/node/package.xml` with package name `omnihand_node`
- ROS2 message package under `vendor/Omnihand-2025-SDK/node/node_msg`
- ROS2 topic/message documentation in `vendor/Omnihand-2025-SDK/document/zh_cn/API_ROS2.md`
- model assets in `vendor/Omnihand-2025-SDK/assets/urdf/omnihand_left.urdf`, `vendor/Omnihand-2025-SDK/assets/urdf/omnihand_right.urdf`, and `vendor/Omnihand-2025-SDK/assets/meshes/*.STL`
missing_artifacts:
- motor-index and joint-name mapping
- tactile/status data schema
- preshape library, safety limits, and calibration procedure
- a local wrapper package or integration layer aligned with the current Nero control/planning stack
- confirmed `aarch64` support or an `x86_64` bring-up environment for the vendor SDK
interface_notes:
- the vendor README describes OmniHand 2025 as `10 active + 6 passive DOF` with `400+` tactile points
- the vendor SDK documents CANFD with ZLG USBCANFD adapters as the primary supported transport
- `src/agx_arm_moveit` and the canonical `src/agx_arm_sim/agx_arm_description` package support `agx_gripper` and `revo2`, not OmniHand
- `pyAgxArm` exposes end-effector drivers for `agx_gripper` and `revo2`, not OmniHand
- the vendor ROS2 API doc exposes left/right topic families under `/agihand/omnihand/{left,right}/...`
- Revo2 support can serve as an interface reference for a future hand integration pass, but it is not a substitute for OmniHand assets
risks:
- the vendor README currently documents Ubuntu 22.04 `x86_64`, while this workspace host is `aarch64`
- vendor ROS2 packages exist, but pulling them straight into `src/` without a wrapper decision could disrupt the current workspace layout
- without a local joint-name mapping and hand integration layer, the SDK is present but not yet usable from the Nero stack
recommended_next_action:
- decide whether to wrap the vendor Python/C++ API or selectively overlay the vendor `node/` and `node_msg/` ROS2 packages into `src/`
- derive a stable 10-joint naming map and align it with the future Nero wrist adapter / OmniHand palm / grasp frame chain
- confirm Agibot `aarch64` support or shift initial SDK bring-up to an `x86_64` host before attempting build validation
open_questions:
- Should OmniHand be integrated through a thin local wrapper around the vendor SDK, or by selectively overlaying the vendor ROS2 packages into `src/`?
- Does Agibot support the current `aarch64` host for this SDK, or should initial bring-up move to an `x86_64` machine?
related_sprint: 1
related_child_document: docs/development/sprint1/assets/omnihand_asset_validation.md

## Current Local Reference Points

These are useful only as design references while OmniHand is still missing locally:

- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/revo2/urdf/` contains an articulated hand model for Revo2.
- `src/agx_arm_moveit` already handles a dexterous-hand branch through `effector_type:=revo2`.
- `pyAgxArm/protocols/can_protocol/drivers/effector/` contains end-effector driver patterns for AgileX gripper and Revo2.

Do not treat those artifacts as OmniHand validation; they are only local references alongside the newly vendored Agibot SDK.