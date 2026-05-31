# Sprint 4 Open Questions

## Resolved Decisions

- Keep `body_base_link` as the shared base frame. Use `left_arm_*` and `right_arm_*` prefixes for arm frames, keep the OmniHand base links as `left_base_link` and `right_base_link`, and use `right_arm`, `left_arm`, and `both_arms` as the initial planning-group names with hand-aware variants layered on top.
- Use prefixed arm and link names inside one shared robot-level TF and planning structure for a single Duo body system. Keep per-arm namespaces available for multiple body-level robots rather than splitting one body-mounted two-arm robot across separate namespaces by default.
- Treat one base with at most two arms as one robot. A future `>2` arm layout should be modeled as another body-level two-arm robot in its own namespace rather than stretching the current Duo system contract.
- Grow `agx_arm_moveit` in place via prefix-aligned multi-profile outputs. Keep the future `left_*`, `right_*`, and `duo_*` SRDF/config surfaces synchronized instead of forking a second MoveIt package.
- Keep one MIT controller instance per arm because gravity, tools, and local control limits remain arm-specific. Keep planning, collision checking, and shared tooling such as trajectory playback at the shared robot level.
- Use shared planning and collision checks as the minimum stable contract for the first coordinated dual-arm task.
- Use the coordinated Hefeweizen pouring workflow as the reference benchmark. Model it as one coupled planning group with orchestration above per-arm execution; the first executable slice may start with synchronized or sequential recorded per-arm trajectories after merged planning.
- The first landed change for the MIT-controller RViz path is to preserve the current single-arm launch as a per-arm debug surface, forward `custom_model` and `custom_model_xacro_args` into `display_control.launch.py`, and let the MIT debug bridge strip a per-arm input joint prefix so the staged Duo description can drive the existing controller-facing debug path without moving controller ownership out of the current packages.
- Keep `src/duo_body_description` as a Sprint 3 and Sprint 4 staging package only. Stable Duo outputs should be promoted back into `src/agx_arm_sim/agx_arm_description`, `src/agx_arm_moveit`, and the owning runtime packages instead of leaving a second long-term source of truth.
- Treat the current Duo-aware MIT RViz debug slice as a control-topic path with `follow:=false` until a feedback-side prefix adapter or equivalent shared-state solution exists.

## Remaining Open Questions

- What is the exact shared-vs-per-arm ROS topic, action, and launch split once a second live arm is added to `agx_arm_ctrl`, `agx_arm_mit_controller`, and `agx_arm_moveit`?
- When should the feedback-side joint-name prefix adapter, or another shared-state adaptation layer, be added so the Duo custom-model RViz path can safely support `follow:=true`?
-> answer: as next step after visual check of the assembly in rviz.