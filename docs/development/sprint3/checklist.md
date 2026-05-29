# Sprint 3 Checklist

Sprint 3 is now active for Nero arm MoveIt and MIT-controller hardening that does not depend on live OmniHand runtime behavior.

## Kickoff And Handoff

- [x] Create `docs/development/sprint3/`.
- [x] Capture the Sprint 2 to Sprint 3 boundary without reopening package-placement decisions.
- [x] Identify the first non-OmniHand-dependent Sprint 3 slice.

## Current Hardening Work

- [x] Promote `nero_arm` as the canonical planning group and remove the temporary `arm` compatibility alias from the active MoveIt surface.
- [x] Keep `nero_tool0` as the Nero flange alias and `tcp_link` as the TCP/planning frame instead of collapsing the two semantics.
- [x] Switch the MoveIt kinematics configuration to TRAC-IK.
- [x] Record that `src/agx_arm_mit_controller` already accepts `trajectory_msgs/JointTrajectory`.
- [x] Run a package-scoped MoveIt smoke test in this environment.
- [x] Add a sim-only profile sweep helper for the current `none` / `agx_gripper` / `revo2` / `omnihand` MoveIt combinations.
- [x] Run the sim-only MoveIt profile sweep on the current host and capture the common runtime gates.
- [x] Run the sim-only MoveIt profile sweep on a host with `trac_ik_kinematics_plugin` available.
- [x] Validate one representative live `/compute_ik` request on the TRAC-IK baseline for `nero_arm`.
- [x] Land the minimum prefix-safe description groundwork needed for the Duo body system slice without forking canonical package ownership.
- [x] Create the `src/duo_body_description` staging package surfaces needed for right-first description bringup.
- [x] Capture the Sprint 3 to Sprint 4 handoff around Duo body system integration.
- [x] Validate at least one representative pose-planning task on the TRAC-IK + OMPL baseline.
- [x] Audit joint ordering, timing, and unit assumptions from MoveIt output into the MIT controller.
- [ ] Iterate the full planning path across all current effector profiles and capture the remaining collision and execution-safety gaps.
- [ ] Isolate the timeout-driven `move_group` shutdown crash well enough to tell whether it is host-local or workspace-local.

The 2026-05-21 profile sweep reached "You can start planning now!" for all current effector profiles with the external `~/workspace/trac_ik_ws` overlay, and a live `/compute_ik` request returned `MoveItErrorCodes.SUCCESS` for `nero_arm`.

On 2026-05-28, a package-scoped `colcon test` pass on `agx_arm_mit_controller` also confirmed the current non-hardware audit path for joint-name validation, joint reordering, and trajectory interpolation on the MIT action surface.

On 2026-05-28, `/usr/bin/python3 src/agx_arm_moveit/scripts/plan_pose_smoke_test.py` succeeded against the active headless MoveIt stack with `candidate=x_plus_3cm`, `points=21`, and `pipeline=ompl`, so the first representative TRAC-IK + OMPL pose-planning task is now verified locally. A follow-up `timeout --signal=INT --kill-after=10s 25s ros2 launch agx_arm_moveit demo.launch.py ... planning_pipelines:=ompl use_rviz:=false` run still hit the same `move_group` teardown segmentation fault, which means the current crash does not require the extra Pilz or CHOMP pipelines to reproduce. The remaining shared gates are full-profile planning-path evidence and isolating whether that OMPL-only teardown failure is workspace-local or upstream host-local.

The Duo body staging work that has already landed is an input to Sprint 4, not a replacement for the remaining Sprint 3 arm-hardening checks.