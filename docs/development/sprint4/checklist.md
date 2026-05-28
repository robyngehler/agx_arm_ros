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

- [ ] Run `xacro` and `check_urdf` for the right-side Duo system slice in a ROS-capable shell.
- [ ] Validate the right-side mount frames, base frame, flange frame, and OmniHand base frame in RViz.
- [ ] Confirm whether the current zero mount-to-base transform is sufficient on the physical body or needs a small adjustment.
- [ ] Add the left arm and left OmniHand chain and validate the mirrored body-mount behavior.
- [ ] Decide the first stable frame and planning-group names for the full two-arm body system.

## Runtime And Planning Generalization

- [ ] Generalize `agx_arm_ctrl` launch surfaces away from implicit single-arm assumptions.
- [ ] Generalize the MIT-controller RViz path away from the current single-arm control assumptions.
- [ ] Generalize `agx_arm_moveit` in place instead of creating a second multi-arm MoveIt package.
- [ ] Define the initial `right_arm`, `left_arm`, and `both_arms` planning groups and the first hand-aware variants.
- [ ] Capture the remaining execution-safety, collision, and controller-ownership gaps for coordinated dual-arm tasks.

## Demo-Oriented Exit Criteria

- [ ] Record one first coordinated dual-arm benchmark target in concrete planning terms.
- [ ] Use the coordinated Hefeweizen pouring workflow as the reference benchmark unless a narrower first demo proves more practical.