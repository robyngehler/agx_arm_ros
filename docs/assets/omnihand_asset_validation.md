# OmniHand Asset Validation

source_document: docs/development/sprint1/assets/omnihand_asset_validation.md
promotion_date: 2026-05-12

component: OmniHand Pro SDK, model, and integration surfaces
repository_or_source: current workspace (`agx_arm_ros`, `pyAgxArm`, `vendor/OmniHand-Pro-2025`)
inspection_date: 2026-06-04
status: SIMULATION_READY_DEVICE_ENUM_CONFIRMED
integration_decision: WRAPPER_FIRST
found_artifacts:
- vendored SDK repository at `vendor/OmniHand-Pro-2025`
- C++ and Python SDK surfaces documented in `README.md`, `document/en/API_CPP.md`, and `document/en/API_PYTHON.md`
- ROS2 package `vendor/OmniHand-Pro-2025/node/package.xml` with package name `omnihand_node`
- ROS2 message package under `vendor/OmniHand-Pro-2025/node/node_msg`
- ROS2 topic/message documentation in `vendor/OmniHand-Pro-2025/document/zh_cn/API_ROS2.md`
- model assets in `vendor/OmniHand-Pro-2025/assets/urdf/omnihand_left.urdf`, `vendor/OmniHand-Pro-2025/assets/urdf/omnihand_right.urdf`, and `vendor/OmniHand-Pro-2025/assets/meshes/*.STL`
- normalized repo-owned OmniHand assets under `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/omnihand`
- Nero attachment xacros for `nero_with_left_omnihand_description.xacro` and `nero_with_right_omnihand_description.xacro`
- MoveIt support for `effector_type:=omnihand` and `omnihand_type:=left|right` under `src/agx_arm_moveit`
- repo-owned OmniHand bridge node and launch surface under `src/agx_arm_ctrl`
- repo-owned `agx_arm_msgs/OmniHandStatus` and `agx_arm_msgs/OmniHandTactileRaw`
- validated mock-hardware launch path through `ros2 launch agx_arm_moveit demo.launch.py effector_type:=omnihand omnihand_type:=left use_rviz:=false db:=false`
- validated Jetson `aarch64` hardware-info probe through the socket-backed vendor SDK using `OMNIHAND_SOCKETCAN_IFACE=can0`
missing_artifacts:
- non-mock backend support behind the repo-owned OmniHand bridge
- validated safe active-joint command and readback loop on the current live device path
- validated runtime evidence for the combined arm-plus-hand path beyond the current mock bridge surface
- a documented upstream-sync and patch-submission workflow for the workspace-owned GitHub fork
interface_notes:
- the vendor README describes OmniHand 2025 as `10 active + 6 passive DOF` with `400+` tactile points
- the vendor SDK documents CANFD with ZLG USBCANFD adapters as the primary supported transport
- the current non-mock hardware-backed effector path in `src/agx_arm_ctrl` has validated arm-side support for `agx_gripper` and `revo2`; the repo-owned OmniHand bridge now also has a validated `backend_type:=sdk` path against the live OmniHand Pro (O12) over native SocketCAN (`backend_type=vendor_sdk`, command + status + tactile readback on the Jetson right hand), with the mock backend retained for off-hardware development
- the current agx_arm stack already switches planning, description, and fake-controller profiles by `effector_type`, so OmniHand can be introduced as another repo-owned effector profile without exposing vendor ROS topics as the public contract
- the local launch and naming contract is now frozen for the simulation slice: `effector_type:=omnihand`, `omnihand_type:=left|right`, and normalized `left_*` / `right_*` joint names
- the repo-owned mock bridge already publishes `feedback/omnihand/joint_states`, `feedback/omnihand/status`, and `feedback/omnihand/tactile_raw`, and it exposes `control/omnihand/stop` plus the compatibility `control/omnihand/joint_trajectory` input
- `pyAgxArm` exposes end-effector drivers for `agx_gripper` and `revo2`, not OmniHand
- the vendor ROS2 API doc exposes left/right topic families under `/agihand/omnihand/{left,right}/...`
- current local hand messages are Revo2-specific and should not be reused as the OmniHand long-term interface
- a local vendor patch now allows a socket-backed Python build/import path on `aarch64` for isolated testing, and the unpacked Python package can be refreshed without local wheel tooling
- the current host now also validates live OmniHand hardware-info retrieval over SocketCAN on `can0`, including model, serial, firmware, supply voltage, and `1 Mbps / 5 Mbps` CAN FD timing
risks:
- the vendor README documents Ubuntu 22.04 `x86_64`, while this workspace host is `aarch64`
- the vendored `thirdParty/` tree only ships the `usbcanfd_libusb_x64` userspace bundle locally, so the stock vendor ZLG path remains x64-only even though the repo now has a socket-backed workaround for build/import
- the vendor asset set itself is still not drop-in ready: `assets/urdf/omnihand_right.urdf` contains absolute local mesh paths, `assets/urdf/xacro/finger.xacro` contains a stray literal `y`, and multiple asset files reference `package://omnihand_description/...`; the repo-owned description package avoids those defects for simulation, but the vendor tree still requires normalization
- pulling the vendor ROS2 packages straight into `src/` would couple the workspace to a transport and topic layout that is not yet validated on this host and does not match the current wrapper-oriented repo structure
- device enumeration and the first safe SDK command/readback loop are now validated on the current host (`backend_type:=sdk` against the live OmniHand Pro right hand over native SocketCAN); the remaining gaps are the coordinated arm-plus-hand runtime path and per-object tactile grasp calibration
- the workspace fork now exists at `https://github.com/robyngehler/OmniHand-Pro-2025.git` and `.gitmodules` can track it, but upstream-sync discipline is still required to avoid long-lived vendor drift
recommended_next_action:
- keep using the validated vendor SDK hardware-info probe as the first live-hand preflight step on Jetson
- validate the first safe active-joint command and readback loop on the same SocketCAN path before promoting ROS-side runtime claims
- keep using the landed agx_arm-native simulation contract: `effector_type:=omnihand`, `omnihand_type:=left|right`, normalized local joint names, repo-owned description assets, and MoveIt/mock-controller support
- keep the OmniHand adapter below ROS; let a repo-owned ROS bridge expose the hand to the agx_arm stack and later switch between mock, direct-SDK, or optional vendor-ROS backends
- use standard `sensor_msgs/JointState` plus `trajectory_msgs/JointTrajectory` and controller conventions for kinematics and motion, with new repo-owned messages only for OmniHand-specific diagnostics and tactile data
- keep `.gitmodules` pointed at the workspace fork and keep the upstream Agibot repository available as the sync and review source
open_questions:
- What is the first safe joint-space command/readback probe that should be captured as the Phase 1 motion baseline on the now-working `can0` path?
- Should the repo keep an optional vendor-ROS backend adapter as a fallback behind the repo-owned bridge, or target direct SDK access only for the first hardware backend?
- Should `vendor/OmniHand-Pro-2025` permanently keep the workspace fork as the default submodule URL while maintaining `AgibotTech/OmniHand-Pro-2025` as an explicit `upstream` remote?
related_sprint: 1

See also: `docs/assets/omnihand/omnihand_ros_integration_options.md`

## Current Local Reference Points

These are useful only as design references while OmniHand is still isolated from the real runtime backend:

- `src/agx_arm_sim/agx_arm_description/agx_arm_urdf/revo2/urdf/` contains an articulated hand model for Revo2.
- `src/agx_arm_moveit` already handles a dexterous-hand branch through `effector_type:=revo2`.
- `pyAgxArm/protocols/can_protocol/drivers/effector/` contains end-effector driver patterns for AgileX gripper and Revo2.

Do not treat those artifacts as OmniHand validation; they are only local references alongside the vendored Agibot SDK.
