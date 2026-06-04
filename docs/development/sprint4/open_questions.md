# Sprint 4 Open Questions

## Resolved Decisions

- Keep `body_base_link` as the shared base frame. Use `left_arm_*` and `right_arm_*` prefixes for arm frames, keep the OmniHand base links as `left_base_link` and `right_base_link`, and use `right_arm`, `left_arm`, and `both_arms` as the initial planning-group names with hand-aware variants layered on top.
- Use prefixed arm and link names inside one shared robot-level TF and planning structure for a single Duo body system. Keep per-arm namespaces available for multiple body-level robots rather than splitting one body-mounted two-arm robot across separate namespaces by default.
- Treat one base with at most two arms as one robot. A future `>2` arm layout should be modeled as another body-level two-arm robot in its own namespace rather than stretching the current Duo system contract.
- Grow `agx_arm_moveit` in place via prefix-aligned multi-profile outputs. Keep the future `left_*`, `right_*`, and `duo_*` SRDF/config surfaces synchronized instead of forking a second MoveIt package.
- Keep one MIT controller instance per arm because gravity, tools, and local control limits remain arm-specific. Keep planning, collision checking, and shared tooling such as trajectory playback at the shared robot level.
- Keep the first shared Duo RViz debug surface as one robot-level Duo description plus one shared soft-target JointState topic, and fan that topic out into one namespace-scoped MIT controller plus one debug bridge per arm.
- Keep the current `both_arms` execution contract arm-only. Reject dual-arm hand effectors on this surface until the hand-aware SRDF, self-collision matrix, and controller-ownership semantics are ready.
- Keep coordinated execution decomposed into the two namespace-scoped `arm_controller/follow_joint_trajectory` endpoints. Do not add a robot-level `both_arms/follow_joint_trajectory` action until fault handling and safety rules are explicit.
- Use shared planning and collision checks as the minimum stable contract for the first coordinated dual-arm task.
- Keep the tightened Duo self-collision contract strict by default. Only directly adjacent arm-link pairs and the body mount contact are exempted from collision checking until live staged-scene evidence says otherwise.
- Use `agx_arm_duo_soft_estop` as the current shared dual-arm soft-stop contract. `/emergency_stop` fans `cancel_trajectory` plus `hold_current` into every configured MIT namespace, while the per-arm `hold_<namespace>` services keep the launch logic ready for a later selective-fix policy.
- Use the coordinated Hefeweizen pouring workflow as the reference benchmark. Model it as one coupled planning group with orchestration above per-arm execution; the first executable slice may start with synchronized or sequential recorded per-arm trajectories after merged planning.
- The first landed change for the MIT-controller RViz path is to preserve the current single-arm launch as a per-arm debug surface, forward `custom_model` and `custom_model_xacro_args` into `display_control.launch.py`, and let the MIT debug bridge strip a per-arm input joint prefix so the staged Duo description can drive the existing controller-facing debug path without moving controller ownership out of the current packages.
- Keep `src/duo_body_description` as a Sprint 3 and Sprint 4 staging package only. Stable Duo outputs should be promoted back into `src/agx_arm_sim/agx_arm_description`, `src/agx_arm_moveit`, and the owning runtime packages instead of leaving a second long-term source of truth.
- Treat the newly landed feedback-side JointState prefix adapter plus custom-model TCP-parent hook as the next minimal `follow:=true` slice; the remaining question is whether graphical RViz and MoveIt validation confirms that this is sufficient, or whether a broader shared-state adapter is still needed.
- Keep the first hand-aware config-based launch path per-arm. `execution_profile:=left_hand|right_hand` is now the landed contract; `both_arms` and `duo_arm` stay arm-only until the shared hand-aware execution semantics are explicit.

## Remaining Open Questions

- What staged-body clearance evidence and live-scene validation are still required before the tightened self-collision matrix can be treated as production-safe for coordinated tasks?
- When coordinated tasks move beyond the current central soft-stop contract, should one-arm faults still fan out to both arms unconditionally, or should the per-arm `hold_<namespace>` surfaces become the user-facing selective-fix mechanism?
- What additional SRDF, controller-ownership, and safety semantics are required before a hand-aware dual-arm surface can graduate beyond the landed per-arm `left_hand` and `right_hand` profiles?
- Does the shared Duo RViz debug surface still need a broader shared-state adapter after the first real graphical and live dual-hardware validation, or is the landed merged-feedback path sufficient?