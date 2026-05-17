# Sprint 2 Checklist

Sprint 2 is still active. The shared ROS2 contract, package baseline, and the first stable interaction diagrams are now in place.

## Documentation And Visualization

- [x] Create `docs/development/sprint2/`.
- [x] Create the Sprint 2 overview, checklist, and open-issues docs.
- [x] Capture the current MoveIt launch chain from `start_single_agx_arm_moveit.launch.py`.
- [x] Capture the current runtime node interfaces for `agx_arm_ctrl_single_node` and `omnihand_bridge_node`.
- [x] Capture the current file-composition path for MoveIt and RViz launches.
- [x] Promote stable Mermaid diagrams into `docs/project/repo_interaction_diagrams.md`.
- [x] Re-check Sprint 2 progress and align the roadmap and progress docs with the current repo state.

## Shared Runtime Baseline

- [x] Freeze the shared launch arguments around `effector_type:=omnihand` and `omnihand_type:=left|right`.
- [x] Keep the OmniHand bridge inside `agx_arm_ctrl` during Sprint 2.
- [x] Keep OmniHand-specific messages in `agx_arm_msgs`.
- [x] Keep the mock-backed OmniHand bridge integrated with the shared `control/joint_states` path.

## Remaining Work Before Sprint 2 Can Be Called Complete

- [ ] Validate the runtime graph from a live running launch in this environment rather than only from file inspection.
- [ ] Land the first non-mock OmniHand backend behind the repo-owned bridge.
- [ ] Validate the shared arm-plus-hand runtime contract against real device behavior.
- [ ] Decide whether `control/omnihand/joint_trajectory` remains compatibility-only or should evolve into a longer-term action or controller surface.
- [ ] Confirm whether the bridge still belongs in `agx_arm_ctrl` once the first real backend is proven.
- [ ] Capture the handoff boundary from Sprint 2 into Sprint 3 without reopening package-placement decisions.