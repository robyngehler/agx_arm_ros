# Sprint Refactor - Open Questions

## Resolved during the initial code cross-check

- Active Duo MoveIt controller config is generated in
  `src/agx_arm_moveit/launch/_moveit_config_builder.py` from the selected
  `arm_instances`; it already creates namespaced arm controllers and, when an
  OmniHand is present, namespaced hand FJT controllers.
- The only standalone unprefixed `moveit_controllers.yaml` currently in the
  workspace is
  `src/agx_arm_sim/Moveit2/nero_gripper_moveit_config/config/moveit_controllers.yaml`.
  It belongs to the legacy standalone MoveIt package and should be treated as a
  legacy surface, not Duo runtime truth.
- `execution_profiles.yaml` already defers some frame and prefix values to
  `duo_motion_registry.yaml`, so the refactor should extend the existing
  registry-resolution path instead of replacing it wholesale.

## Contract decisions to freeze before implementation fans out

- Should `SideControlState` and the lease acquire/release contracts live as new
  repo-owned interfaces in `src/agx_arm_msgs`, or is a smaller extension of the
  current message surface sufficient?
- Should the control epoch be added to the current MIT command message, or
  should a dedicated `ArmMitCommand` be introduced to keep legacy ingress
  isolated?
- Should `prepare_hand_window` and `resume_arm_control` remain as compatibility
  wrappers around the new lease flow during migration, or be removed in one cut?
- Should the MoveIt hand FJT path remain available only for explicit debug or
  development profiles once the semantic hand-skill path is the production
  contract?

## Hardware validation dependencies

- Which hardware session will own Phase 0 and Phase 2 validation on the V02
  branch?
- Can we capture per-side SocketCAN frame counts, loop jitter, and per-thread
  CPU without disturbing timing on the target Jetson?
- Does the O12 Pro backend on the current firmware hold the last verified
  setpoint autonomously on both hands after the host stops sending commands?
- If real velocity must be derived from timestamped positions first, what update
  rate and filtering are acceptable for e-stop verification without masking
  motion?

## Packaging and rollout questions

- Does `sprint_refactor` stay as a dedicated migration surface until the full
  V02 program closes, or does it fold into a numbered sprint once code changes
  start landing?
- At which phase should stable docs in `docs/assets/`, `docs/control/`, and
  `docs/project/` start absorbing the new contracts rather than pointing at this
  sprint surface?