# Sprint 1 Open Questions

These items are intentionally left open after the current Sprint 1 implementation pass.

## OmniHand Integration Strategy

- Should the first local integration pass wrap the vendor Python/C++ SDK with a thin local package?
- Or should the repo selectively overlay the vendor ROS2 `node/` and `node_msg/` packages into `src/`?

## OmniHand Bring-Up Platform

- Does Agibot support the current `aarch64` host for the vendored SDK?
- If not, should the first validated bring-up move to an `x86_64` host while this repo keeps the SDK vendored for inspection only?