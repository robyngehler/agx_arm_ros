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
- [ ] Validate at least one representative pose-planning task on the TRAC-IK + OMPL baseline.
- [ ] Audit joint ordering, timing, and unit assumptions from MoveIt output into the MIT controller.
- [ ] Iterate the full planning path across all current effector profiles and capture the remaining collision and execution-safety gaps.
- [ ] Isolate the timeout-driven `move_group` shutdown crash well enough to tell whether it is host-local or workspace-local.

The 2026-05-21 profile sweep reached "You can start planning now!" for all current effector profiles with the external `~/workspace/trac_ik_ws` overlay, and a live `/compute_ik` request returned `MoveItErrorCodes.SUCCESS` for `nero_arm`; the remaining shared gate is the timeout-driven `move_group` teardown crash on this host.

The Duo body staging work that has already landed is an input to Sprint 4, not a replacement for the remaining Sprint 3 arm-hardening checks.