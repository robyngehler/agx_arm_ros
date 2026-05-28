# Duo System Integration Direction

## Direction Change

The current priority is no longer a single-arm path followed by a later ad hoc body merge.

The repo now treats the body-mounted multi-arm system as the main integration direction, but stages it in a pragmatic order:

1. `body + right arm + right OmniHand`
2. mirror the left side into the same top-level description and bringup surfaces
3. generalize the current single-arm RViz, MoveIt, and controller-facing surfaces in place
4. only then widen further into Isaac and later AGV/mobile variants

## Landed First Steps

- `src/duo_body_description` now exists as a documented Sprint 3 and Sprint 4 staging package.
- `src/duo_body_description/urdf/nero_arm_macro.xacro` now provides:
  - a prefix-safe Nero arm chain
  - a reusable OmniHand flange macro
  - side-selectable OmniHand attachment macros
- `src/duo_body_description/urdf/duo_system.urdf.xacro` now supports:
  - `use_left_arm`
  - `use_left_hand`
  - `use_right_arm`
  - `use_right_hand`
- `src/duo_body_description/launch/display_duo_system.launch.py` provides a minimal description-only bringup path for RViz and `robot_state_publisher` validation.
- Stable policy and development docs now acknowledge the staging package instead of leaving it in conflict with the documented package structure.

## First Practical Validation Slice

The next executable validation target is the right-side chain only:

```text
body_base_link
-> right_arm_mount_link
-> right_arm_base_link
-> right_arm_nero_tool0
-> right_arm_omnihand_flange
-> right_base_link
```

This keeps the immediate geometry and frame audit small while still proving that the new top-level structure is viable.

## Immediate Next Steps

1. Run `xacro` and `check_urdf` for the right-side system slice in a ROS-capable shell.
2. Validate the right-side mount, base, flange, and hand frames in RViz.
3. Decide whether the current zero mount-to-base offset is sufficient or needs a measured correction.
4. Mirror the left side only after the right-side chain is visually and structurally correct.
5. Start the in-place generalization of `agx_arm_ctrl`, the MIT-controller RViz path, and `agx_arm_moveit` away from their current single-arm assumptions.

## Near-Term Benchmark Target

The representative coordinated system task is a shared pouring workflow with both arms. The current planning intention is:

- one arm manages the bottle
- the other arm manages the glass or support pose
- the body-mounted frame semantics and the multi-arm-safe planning groups should make that benchmark possible without another description-layer redesign