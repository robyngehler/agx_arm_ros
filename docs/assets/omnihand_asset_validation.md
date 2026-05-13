# OmniHand Asset Validation

source_document: docs/development/sprint1/assets/omnihand_asset_validation.md
promotion_date: 2026-05-12

component: OmniHand Pro SDK, model, and integration surfaces
repository_or_source: current workspace (`agx_arm_ros`, `pyAgxArm`, `vendor/Omnihand-2025-SDK`)
inspection_date: 2026-05-13
status: PARTIALLY_AVAILABLE
integration_decision: WRAPPER_FIRST
found_artifacts:
- vendored SDK repository at `vendor/Omnihand-2025-SDK`
- C++ and Python SDK surfaces documented in `README.md`, `document/en/API_CPP.md`, and `document/en/API_PYTHON.md`
- ROS2 package `vendor/Omnihand-2025-SDK/node/package.xml` with package name `omnihand_node`
- ROS2 message package under `vendor/Omnihand-2025-SDK/node/node_msg`
- ROS2 topic/message documentation in `vendor/Omnihand-2025-SDK/document/zh_cn/API_ROS2.md`
- model assets in `vendor/Omnihand-2025-SDK/assets/urdf/omnihand_left.urdf`, `vendor/Omnihand-2025-SDK/assets/urdf/omnihand_right.urdf`, and `vendor/Omnihand-2025-SDK/assets/meshes/*.STL`
missing_artifacts:
- motor-index and joint-name mapping normalized to local naming
- local ROS adapter package or wrapper layer aligned with the Nero control/planning stack
- repo-owned hand status schema for OmniHand diagnostics and tactile data
- vendor-supported `aarch64` runtime support or a validated `x86_64` bring-up environment for first hardware access
- normalized local ROS description package for the vendor hand meshes and xacros
interface_notes:
- the vendor README describes OmniHand 2025 as `10 active + 6 passive DOF` with `400+` tactile points
- the vendor SDK documents CANFD with ZLG USBCANFD adapters as the primary supported transport
- the current workspace runtime only supports `agx_gripper` and `revo2` in `src/agx_arm_ctrl`, `src/agx_arm_moveit`, and `src/agx_arm_sim/agx_arm_description`
- the current agx_arm stack already switches planning, description, and fake-controller profiles by `effector_type`, so OmniHand can be introduced as another repo-owned effector profile without exposing vendor ROS topics as the public contract
- `pyAgxArm` exposes end-effector drivers for `agx_gripper` and `revo2`, not OmniHand
- the vendor ROS2 API doc exposes left/right topic families under `/agihand/omnihand/{left,right}/...`
- current local hand messages are Revo2-specific and should not be reused as the OmniHand long-term interface
- a local vendor patch now allows a socket-backed Python build/import path on `aarch64` for isolated testing, and the unpacked Python package can be refreshed without local wheel tooling
risks:
- the vendor README documents Ubuntu 22.04 `x86_64`, while this workspace host is `aarch64`
- the vendored `thirdParty/` tree only ships the `usbcanfd_libusb_x64` userspace bundle locally, so the stock vendor ZLG path remains x64-only even though the repo now has a socket-backed workaround for build/import
- the vendored asset set is not drop-in ready: `assets/urdf/omnihand_right.urdf` contains absolute local mesh paths, `assets/urdf/xacro/finger.xacro` contains a stray literal `y`, and multiple asset files reference `package://omnihand_description/...` even though no such ROS package exists in the vendor tree
- pulling the vendor ROS2 packages straight into `src/` would couple the workspace to a transport and topic layout that is not yet validated on this host and does not match the current wrapper-oriented repo structure
- even with the local socket-backed build path enabled, the current isolated runtime probe still returns incomplete/default runtime data when the live CAN path does not answer, so safe device enumeration still cannot be claimed
recommended_next_action:
- keep the vendor SDK vendored and continue isolated bring-up through the repo-owned Phase 1 smoke test plus the local socket-backed build path for the eventual live backend
- in parallel, define an agx_arm-native OmniHand simulation contract now: `effector_type:=omnihand`, `omnihand_type:=left|right`, normalized local joint names, repo-owned description assets, and MoveIt/mock-controller support
- keep the OmniHand adapter below ROS; let a repo-owned ROS bridge expose the hand to the agx_arm stack and later switch between mock, direct-SDK, or optional vendor-ROS backends
- use standard `sensor_msgs/JointState` plus `trajectory_msgs/JointTrajectory` and controller conventions for kinematics and motion, with new repo-owned messages only for OmniHand-specific diagnostics and tactile data
open_questions:
- Does Agibot support the current `aarch64` host for live hardware bring-up, or should first validated device access move to an `x86_64` machine?
- Should the repo keep an optional vendor-ROS backend adapter as a fallback behind the repo-owned bridge, or target direct SDK access only for the first hardware backend?
related_sprint: 1

See also: `docs/control/omnihand_ros_integration_options.md`

## Current Local Reference Points

These are useful only as design references while OmniHand is still isolated from the main runtime:

- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/revo2/urdf/` contains an articulated hand model for Revo2.
- `src/agx_arm_moveit` already handles a dexterous-hand branch through `effector_type:=revo2`.
- `pyAgxArm/protocols/can_protocol/drivers/effector/` contains end-effector driver patterns for AgileX gripper and Revo2.

Do not treat those artifacts as OmniHand validation; they are only local references alongside the vendored Agibot SDK.