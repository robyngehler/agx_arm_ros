# OmniHand SDK Integration Start

component: OmniHand 2025 vendor SDK onboarding
repository_or_source: `vendor/OmniHand-Pro-2025`
inspection_date: 2026-05-13
status: PARTIALLY_AVAILABLE
integration_decision: WRAPPER_FIRST
found_artifacts:
- vendored git submodule at `vendor/OmniHand-Pro-2025`
- root SDK build system: `CMakeLists.txt`, `build.sh`, `src/`, `python/`, `examples/`
- Python binding package scaffold under `python/agibot_hand/`
- ROS2 package `node/package.xml` with package name `omnihand_node`
- ROS2 message package under `node/node_msg/`
- ROS2 API documentation in `document/zh_cn/API_ROS2.md`
- Python API documentation in `document/en/API_PYTHON.md`
- hand model assets in `assets/urdf/omnihand_left.urdf`, `assets/urdf/omnihand_right.urdf`, and `assets/meshes/*.STL`
interface_notes:
- vendor README describes OmniHand 2025 as `10 active + 6 passive DOF`
- vendor README documents CANFD transport with ZLG USBCANFD adapters as the primary supported interface
- ROS2 API doc exposes left/right topic families under `/agihand/omnihand/{left,right}/...`
- Python API exposes `AgibotHandO12.create_hand(...)`, active-joint angle APIs, tactile sensor reads, and control-mode APIs
constraints:
- vendor README documents Ubuntu 22.04 `x86_64`, gcc 11.4+, and Python 3.10+
- current workspace host is `aarch64`, so vendor-supported local build/run validation is not yet confirmed here
- the vendored SDK lives outside `src/` on purpose so its vendor ROS2 packages do not get pulled into the main colcon workspace accidentally
 - the vendored `thirdParty/` tree only ships `usbcanfd_libusb_x64_1.0.10_250328` as the obvious userspace CAN bundle, and `python/CMakeLists.txt` copies that library into the Python package build
 - the asset bundle is not yet drop-in ready for a local ROS description package: `assets/urdf/omnihand_right.urdf` contains absolute local mesh paths, `assets/urdf/xacro/finger.xacro` contains a stray literal `y`, and multiple assets assume `package://omnihand_description/...` even though no matching ROS package is present in the vendor tree
- a local vendor patch now enables a socket-backed Python build/import path on `aarch64`, and the unpacked Python package can be refreshed without local wheel tooling, but the stock Python-facing vendor path still centers on the ZLG-linked build and successful runtime validation still requires a verified responding hand
open_questions:
- Is there a vendor-supported `aarch64` path for live hardware bring-up, or should the first validated device access happen on an `x86_64` host?
- Which repo-owned ROS interface should sit above the future adapter once isolated bring-up succeeds?
recommended_next_action:
- validate the vendor Python API on hardware as a standalone bring-up path before adding any repo-local ROS layer
- derive a stable joint-name mapping for the 10 active joints before attempting MoveIt or controller-side integration
- confirm whether Agibot supports `aarch64` for the SDK or whether OmniHand bring-up should happen first on an `x86_64` host
- once standalone access works, implement a thin local adapter and then add a repo-owned ROS bridge above it
- if the runtime probe still returns incomplete/default data after transport init succeeds, treat that as a device/runtime blocker and avoid treating the current host path as validated hardware access
related_sprint: 1
related_child_document: docs/development/sprint1/hand/omnihand_sdk_integration.md

## Immediate Recommendation

Do not pull the vendor ROS2 node directly into the main workspace yet. The safer next step is to keep the SDK vendored, validate isolated bring-up through the vendor Python API, and then create a thin local wrapper package once the hardware bring-up path (`x86_64` vendor path vs local `aarch64` socket path) is explicit.

## Near-Term Plan

1. Treat platform support as a gate separate from workspace integration.
2. Run the first working control loop outside the main ROS workspace using the vendor Python API.
3. Capture a stable 10-joint naming map, device info, and transport setup from the validated run.
4. Add a repo-owned adapter layer with no ROS dependencies.
5. Add a small ROS bridge above the adapter without reusing the current Revo2-specific hand messages.
6. Normalize the description assets and MoveIt integration only after the direct control path is stable.
