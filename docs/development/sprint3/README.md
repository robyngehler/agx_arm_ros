# Sprint 3 Working Notes

## Purpose

This folder tracks Sprint 3 implementation details for the "Nero Planning and Control Baseline Hardening" objective in the physical AI roadmap.

It captures arm-only MoveIt and MIT-controller validation work that can proceed without a live OmniHand backend, plus the minimum naming and description groundwork needed so Sprint 4 can assemble the first Duo body system slice without reopening runtime ownership.

Program-level coordination lives in:

- `docs/development/nero_physical_ai_roadmap.md`
- `docs/development/nero_physical_ai_progress.md`
- `docs/development/component_implementation_map.md`

Do not use this Sprint 3 folder as the cross-sprint source of truth.

## Current Working Location During Implementation

- `docs/development/sprint3/checklist.md`
- `docs/development/sprint3/errors_and_fixes.md`
- `docs/development/sprint3/open_questions.md`
- `docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md`

## Current Snapshot

| Area | Status | Summary |
| --- | --- | --- |
| MoveIt naming baseline | CONFIRMED | `nero_arm` is now the only active planning group, `nero_tool0` now originates in the canonical Nero description package, and `tcp_link` stays distinct as the TCP/planning target frame. |
| `JointTrajectory` execution path | CONFIRMED | `src/agx_arm_mit_controller` already accepts `trajectory_msgs/JointTrajectory` and includes local validation for interpolation and joint-name handling; a 2026-05-28 package-scoped test pass revalidated that path on the current host. |
| MoveIt IK baseline | CONFIRMED | `agx_arm_moveit` now runs against TRAC-IK, and Humble / Jetson hosts can use the documented `~/workspace/trac_ik_ws` source-built overlay when the apt package is unavailable. |
| Representative OMPL pose planning | CONFIRMED | `src/agx_arm_moveit/scripts/plan_pose_smoke_test.py` now provides a repo-owned near-home OMPL pose-planning check; on 2026-05-28 it succeeded locally against `nero_arm` with a `tcp_link` target offset by `+3 cm` in `x`. |
| Simulation-first integration path | CONFIRMED | Sim-only MoveIt bringup across the current effector profiles is a valid Sprint 3 hardening path before collision-checked sim-plus-real execution. |
| Duo system staging handoff | STARTED | `src/duo_body_description` now contains prefix-safe arm composition, side-selectable hand assembly, and a right-side default description bringup path; the remaining RViz, MoveIt, and controller generalization belongs to Sprint 4. |
| Live validation evidence | PARTIAL | On 2026-05-21, a six-profile sim-only sweep reached the MoveIt ready state for `none`, `agx_gripper`, `revo2` left/right, and `omnihand` left/right with no TRAC-IK plugin-load errors after sourcing `~/workspace/trac_ik_ws/install/setup.bash`, and a live `/compute_ik` call on `nero_arm` returned `MoveItErrorCodes.SUCCESS`. On 2026-05-28, a targeted `agx_arm_mit_controller` test pass confirmed the current non-hardware audit path for trajectory ordering and timing, `plan_pose_smoke_test.py` succeeded on the `ompl` pipeline, and an OMPL-only timeout run still reproduced the `move_group` teardown crash. The remaining shared gates are broader full-profile planning evidence and a smaller reproducible crash isolation path. |

## Scope Adjustments From The Roadmap

- Sprint 3 can run in parallel with Sprint 2 only when the work stays arm-only and does not reopen the shared ROS2 contract or package-boundary decisions.
- Sprint 3 may also land the minimum prefix-safe and side-selectable description groundwork needed by the first Duo body system slice, provided that long-term package ownership and runtime contracts stay with the current canonical packages.
- Sim-only MoveIt integration across effector profiles is a valid hardening path and should be used before widening into sim-plus-real collision-checked execution.
- TRAC-IK is now the selected MoveIt IK baseline for Nero rather than a deferred comparison item.

## Document Map

- `checklist.md`: Sprint 3 task list and completion state.
- `errors_and_fixes.md`: local issues encountered while hardening the Nero planning baseline.
- `open_questions.md`: unresolved planning, validation, and naming questions that still affect Sprint 3 exit criteria.
- `planning/trac_ik_humble_jetson_repro.md`: reproducible TRAC-IK source-build fallback for Humble / Jetson plus the verified overlay source order used by this workspace.
- `docs/development/sprint4/README.md`: the handoff target for the first Duo body system baseline once the description groundwork is in place.

## Inputs Used For This Pass

- `src/agx_arm_moveit/config/agx_arm.urdf.xacro`
- `src/agx_arm_moveit/config/agx_arm.srdf.xacro`
- `src/agx_arm_moveit/config/agx_arm.srdf`
- `src/agx_arm_moveit/config/kinematics.yaml`
- `src/agx_arm_moveit/scripts/plan_pose_smoke_test.py`
- `src/agx_arm_moveit/launch/demo.launch.py`
- `src/agx_arm_moveit/launch/move_group.launch.py`
- `src/agx_arm_moveit/package.xml`
- `src/agx_arm_moveit/README.md`
- `src/agx_arm_moveit/README_EN.md`
- `scripts/agx_arm_install_deps.sh`
- `scripts/moveit_profile_smoke_test.sh`
- `src/agx_arm_mit_controller/README.md`
- `src/agx_arm_mit_controller/config/nero_mit_controller_defaults.yaml`
- `src/agx_arm_mit_controller/agx_arm_mit_controller/mit_controller_node.py`
- `src/agx_arm_mit_controller/test/test_follow_joint_trajectory_validation.py`
- `src/agx_arm_mit_controller/test/test_trajectory_buffer.py`
- `src/agx_arm_mit_controller/test/test_trajectory_io.py`
- `docs/assets/tcp_offset/TCP_OFFSET.md`
- `docs/assets/tcp_offset/TCP_OFFSET_EN.md`
- `docs/development/sprint2/README.md`
- `docs/development/sprint2/checklist.md`
- `docs/development/sprint3/planning/trac_ik_humble_jetson_repro.md`
- `src/duo_body_description/urdf/duo_system.urdf.xacro`
- `src/duo_body_description/urdf/nero_arm_macro.xacro`
- `src/duo_body_description/launch/display_duo_system.launch.py`