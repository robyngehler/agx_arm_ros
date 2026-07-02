# Global Checklist

Repo-wide documentation and consistency checklist.

## Done in this pass

- [x] Added a central docs hub at `docs/README.md`.
- [x] Aligned top-level `README.md` and `README_EN.md` with the current native CAN bringup and vendored `pyAgxArm` runtime path.
- [x] Repointed repo-level MoveIt quickstart examples and stable diagrams to the current canonical wrapper path (`start_agx_arm_components.launch.py` for the operational matrix, `start_agx_arm_moveit.launch.py` as the lower-level wrapper).
- [x] Updated `scripts/setup_agx_arm_runtime_env.sh` to prefer `vendor/pyAgxArm` and only fall back to `../pyAgxArm`.
- [x] Reprioritized `docs/CAN_USER.md` and `docs/CAN_USER_EN.md` so native `mttcan` is the current first path and USB role-based setup is clearly fallback.
- [x] Normalized `agx_arm_moveit` and `agx_arm_mit_controller` README examples to explicit execution profiles and current CAN naming.
- [x] Added `src/agx_arm_sim/agx_arm_description/README_EN.md` to restore the English documentation path.
- [x] Marked the Sprint 2 MIT workflow note as historical and redirected active workflow references to `docs/control/teach_and_run.md`.
- [x] Added cross-sprint `docs/development/checklist.md`, `docs/development/errors_and_fixes.md`, and `docs/development/open_questions.md` so development-layer follow-ups no longer live only inside sprint folders.
- [x] Aligned active OmniHand bringup docs and runtime error text with the bridge's SDK auto-discovery path.
- [x] Re-scoped the Chinese `agx_arm_description` package README so live OmniHand bridge ownership points at `agx_arm_ctrl` instead of future-work language.
- [x] Removed historical OmniHand asset docs from `docs/assets/omnihand/`, promoted the surviving active-joint mapping under a stable filename, and redirected active references to the stable validation docs.

## Still open

- [x] Deprecated launch defaults and public runtime names such as `can0` and `can_nero` in favor of `can_nero_right` and `can_nero_left`.
- [ ] Decide how much operational launch detail should stay in package-local READMEs versus moving entirely into `docs/control/`.