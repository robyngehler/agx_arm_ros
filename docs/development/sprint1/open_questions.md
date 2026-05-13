# Sprint 1 Decisions And Open Questions

## Resolved Decisions

### OmniHand Integration Strategy

- The first local OmniHand integration pass will use isolated vendor-SDK bring-up plus a thin local wrapper.
- The vendor ROS2 `node/` and `node_msg/` packages remain vendored for reference and should not be selectively overlaid into `src/` yet.
- The first ROS-side integration should sit above the wrapper, not directly above the vendor node.

## Remaining Open Questions

### OmniHand Bring-Up Platform

- The repo now has a local socket-backed `aarch64` build/import path for isolated probing.
- Does Agibot support the current `aarch64` host for live hardware bring-up as well?
- If not, should the first validated device-access pass move to an `x86_64` host while this repo keeps the SDK vendored for inspection and wrapper work?

### OmniHand ROS Surface After Standalone Validation

- Should the first repo-owned ROS layer be limited to `sensor_msgs/JointState` plus raw status topics?
- Or is a new custom message family needed immediately for tactile data and richer diagnostics?