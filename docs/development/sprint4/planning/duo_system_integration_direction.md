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
- `src/duo_body_description/CMakeLists.txt` no longer installs a nonexistent `rviz/` directory, so the staging package now builds cleanly in the active ROS overlay again.
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

## Validated On 2026-05-28

- `colcon build --packages-select duo_body_description` succeeded in a ROS-capable shell after forcing the system Python for that package build.
- `ros2 run xacro xacro ...` plus `check_urdf` succeeded for the `right`, `left`, and `both` Duo profiles.
- `ros2 launch duo_body_description display_duo_system.launch.py gui:=false use_rviz:=false ...` started successfully for the `right` and `left` slices.
- The Duo body STL was confirmed to be exported in millimeters and is now scaled into meters in the staging URDF, while the body mesh origin and the initial mount-to-base correction are exposed as top-level arguments for RViz-side alignment.
- `start_single_agx_arm_rviz.launch.py` now forwards `custom_model`, `custom_model_xacro_args`, and `input_joint_prefix`, so the existing MIT RViz debug path can target staged Duo Xacros without renaming the controller-facing joint contract.
- The remaining geometry work is now a visual RViz audit and a physical mount measurement, not a package-discovery or URDF-parsing problem.

## Immediate Next Steps

1. Run the RViz frame audit for the right-side mount, base, flange, and hand frames with `Fixed Frame := body_base_link`.
2. Refine the current staging mount-to-base correction and body-mesh origin against the physical body and CAD reference until the plate holes, base-link axis, and real joint-center alignment agree.
3. Validate the new Duo-aware MIT RViz debug path in a graphical shell and decide whether `follow:=false` is sufficient for the first per-arm debug slice or whether a feedback-side joint-name prefix adapter is needed next.
4. Extend the current `agx_arm_moveit` namespace/profile builder into prefix-aligned multi-profile `right_arm`, `left_arm`, and `both_arms` outputs.
5. Generalize `agx_arm_ctrl` launch surfaces away from implicit single-arm assumptions while keeping one MIT controller per arm and shared planning above them.
6. Capture the remaining execution-safety and collision gaps for coordinated dual-arm tasks.

## Near-Term Benchmark Target

The representative coordinated system task is a shared pouring workflow with both arms. The current planning intention is:

- one arm manages the bottle
- the other arm manages the glass or support pose
- the body-mounted frame semantics and the multi-arm-safe planning groups should make that benchmark possible without another description-layer redesign
- the first executable orchestration slice may use synchronized or sequential recorded per-arm trajectories, but only after a shared planning and collision check pass