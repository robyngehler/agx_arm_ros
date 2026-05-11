# OmniHand SDK Integration Start

component: OmniHand 2025 vendor SDK onboarding
repository_or_source: `vendor/Omnihand-2025-SDK`
inspection_date: 2026-05-11
status: PARTIALLY_AVAILABLE
found_artifacts:
- vendored git submodule at `vendor/Omnihand-2025-SDK`
- root SDK build system: `CMakeLists.txt`, `build.sh`, `src/`, `python/`, `examples/`
- Python binding package scaffold under `python/omnihand_2025/`
- ROS2 package `node/package.xml` with package name `omnihand_node`
- ROS2 message package under `node/node_msg/`
- ROS2 API documentation in `document/zh_cn/API_ROS2.md`
- Python API documentation in `document/en/API_PYTHON.md`
- hand model assets in `assets/urdf/omnihand_left.urdf`, `assets/urdf/omnihand_right.urdf`, and `assets/meshes/*.STL`
interface_notes:
- vendor README describes OmniHand 2025 as `10 active + 6 passive DOF`
- vendor README documents CANFD transport with ZLG USBCANFD adapters as the primary supported interface
- ROS2 API doc exposes left/right topic families under `/agihand/omnihand/{left,right}/...`
- Python API exposes `AgibotHandO10.create_hand(...)`, active-joint angle APIs, tactile sensor reads, and control-mode APIs
constraints:
- vendor README documents Ubuntu 22.04 `x86_64`, gcc 11.4+, and Python 3.10+
- current workspace host is `aarch64`, so vendor-supported local build/run validation is not yet confirmed here
- the vendored SDK lives outside `src/` on purpose so its vendor ROS2 packages do not get pulled into the main colcon workspace accidentally
open_questions:
- Should the first local integration pass wrap the vendor SDK, or selectively overlay the vendor ROS2 packages into `src/`?
- Is there a supported `aarch64` path for this SDK, or should the first validated bring-up happen on an `x86_64` host?
recommended_next_action:
- decide whether OmniHand support will be wrapped from the vendor Python/C++ API or by selectively overlaying the vendor ROS2 `node/` and `node_msg/` packages into `src/`
- inspect the URDF assets and define the future Nero wrist adapter / OmniHand palm / grasp frame chain
- derive a stable joint-name mapping for the 10 active joints before attempting MoveIt or controller-side integration
- confirm whether Agibot supports `aarch64` for the SDK or whether OmniHand bring-up should happen first on an `x86_64` host
related_sprint: 1
related_child_document: docs/development/sprint1/hand/omnihand_sdk_integration.md

## Immediate Recommendation

Do not pull the vendor ROS2 node directly into the main workspace yet. The safer next step is to keep the SDK vendored, inspect the ROS2 messages and Python/C++ control surfaces, and then create a thin local wrapper package once the architecture plan (`x86_64` vs `aarch64`) is explicit.

## Open Questions

- Wrapper vs overlay remains open.
- `aarch64` vs `x86_64` bring-up remains open.
