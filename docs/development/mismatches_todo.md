# Repo Docs And Source Mismatches

Working tracker for the current documentation cleanup. This file now tracks the remaining structural
follow-ups after the 2026-07-02 cleanup pass.

## Resolved in the 2026-07-02 pass

- top-level README and README_EN native CAN naming and bringup examples
- public launch defaults and runtime-facing helper defaults now use `can_nero_right` instead of deprecated `can0` or `can_nero`
- vendored `pyAgxArm` runtime preference in `scripts/setup_agx_arm_runtime_env.sh`
- top-level CAN guide prioritization between native `mttcan` and USB role-based setup
- `agx_arm_moveit` and `agx_arm_mit_controller` README launch examples
- missing `src/agx_arm_sim/agx_arm_description/README_EN.md`
- active MIT workflow references that still pointed at the old Sprint 2 note
- active repo-level MoveIt quickstart references now point at `start_agx_arm_components.launch.py` / `docs/control/bringup.md` instead of promoting `start_single_agx_arm_moveit.launch.py` as the primary path
- active OmniHand bringup docs now match the bridge's SDK auto-discovery path instead of implying manual ROS-launch env exports
- top-level `docs/development/` now has its own cross-sprint `checklist.md`, `errors_and_fixes.md`, and `open_questions.md`
- `src/agx_arm_sim/agx_arm_description/README.md` now scopes OmniHand support to description/visualization and points live runtime ownership at `agx_arm_ctrl`
- active development overview now distinguishes the native CAN baseline from the workflow-specific shared arm+hand bus caveat
- historical OmniHand asset docs were removed from `docs/assets/omnihand/`; surviving active references now point at `omnihand_vendor_sdk_aarch64.md`, `omnihand_active_joint_map.md`, and the current validation docs

## Remaining open mismatches or follow-ups

### Historical docs that still look runnable

- `docs/development/sprint5/planning/arm_plus_hand_shared_can_proposal.md` still contains old
  `can_port:=can_nero` examples, but it is now explicitly bannered as historical. Leave the commands
  only as transport-history evidence unless the repo later decides to normalize archived command text.

### Documentation boundary questions

- Decide how much operational launch detail should remain in package-local READMEs versus moving
  entirely into `docs/control/`.