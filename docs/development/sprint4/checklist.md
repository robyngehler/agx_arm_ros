# Sprint 4 Checklist

Sprint 4 is now active for the first body-mounted Duo system baseline.

## Kickoff And Handoff

- [x] Create `docs/development/sprint4/`.
- [x] Reframe the sprint target as `body + right arm + right OmniHand` first, then mirror to the left side.
- [x] Document `src/duo_body_description` as the current staging package instead of leaving it as an undocumented exception.

## Landed First Steps

- [x] Add a prefix-safe Nero arm composition macro for body-mounted multi-arm assembly.
- [x] Add side-selectable OmniHand attachment macros for the staged Duo system chain.
- [x] Make `src/duo_body_description/urdf/duo_system.urdf.xacro` configurable via `use_left_arm`, `use_left_hand`, `use_right_arm`, and `use_right_hand`.
- [x] Keep the current right-side system slice as the default description bringup target.
- [x] Add a minimal `display_duo_system.launch.py` path for description-only validation.
- [x] Capture the current direction change in the stable policy docs and global development docs.

## Next Validation And Integration Steps

- [x] Build `duo_body_description` in the active ROS overlay and clear the package-local blocker that prevented ROS-native validation.
- [x] Run `xacro` and `check_urdf` for the right-side Duo system slice in a ROS-capable shell.
- [x] Start `display_duo_system.launch.py` headlessly for the right and left slices in a ROS-capable shell.
- [ ] Validate the right-side mount frames, base frame, flange frame, and OmniHand base frame in RViz.
- [ ] Confirm whether the current zero mount-to-base transform is sufficient on the physical body or needs a small adjustment.
- [x] Add the left arm and left OmniHand chain and validate the mirrored body-mount behavior.
- [x] Decide the first stable frame and planning-group names for the full two-arm body system.

## Runtime And Planning Generalization

- [ ] Generalize `agx_arm_ctrl` launch surfaces away from implicit single-arm assumptions while keeping one MIT controller per arm.
- [x] Land the first control-topic Duo-aware MIT-controller RViz debug path by forwarding custom Duo Xacros through `display_control.launch.py` and stripping per-arm input prefixes in the MIT debug bridge.
- [ ] Add feedback-side prefix adaptation or another shared-state adapter before treating the Duo custom-model debug path as `follow:=true` capable.
- [ ] Split the MIT-controller RViz path into a reusable Duo-aware description selector plus per-arm namespace-scoped controller/debug instances.
- [ ] Generalize `agx_arm_moveit` in place via prefix-aligned multi-profile outputs instead of creating a second multi-arm MoveIt package.
- [ ] Implement the initial `right_arm`, `left_arm`, and `both_arms` planning groups and the first hand-aware variants in the generated SRDF/config surfaces.
- [ ] Capture the remaining execution-safety, collision, and controller-ownership gaps for coordinated dual-arm tasks.

## Demo-Oriented Exit Criteria

- [x] Record one first coordinated dual-arm benchmark target in concrete planning terms.
- [x] Use the coordinated Hefeweizen pouring workflow as the reference benchmark unless a narrower first demo proves more practical.

The 2026-05-28 ROS-native validation pass built `duo_body_description`, then passed `xacro` + `check_urdf` for the `right`, `left`, and `both` profiles, and started the headless display launch for the right and left slices. A follow-up runtime pass also landed the first Duo-aware MIT RViz debug path through `start_single_agx_arm_rviz.launch.py`, `display_control.launch.py`, and the MIT joint-state bridge on the control-topic side. The remaining validation gates now depend on RViz review, physical mount measurements, feedback-side prefix handling, wider runtime generalization, and shared multi-arm planning surfaces rather than package discovery or URDF parsing.