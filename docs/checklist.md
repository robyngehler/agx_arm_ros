# Global Checklist

Repo-wide documentation and consistency checklist.

## Current migration pass

- [x] Re-audited the repo against the target structure and refreshed `docs/target/README.md` as the active control document.
- [x] Created `docs/control/environment.md` as the new canonical page for Python environments, ROS overlays, and build or test wrappers.
- [x] Started promoting the latest shared arm-plus-hand CAN findings into `docs/errors_and_fixes.md`.
- [x] Split `docs/control/bringup.md` into `docs/control/bringups/launches.md` and kept `bringup.md` as a compatibility pointer.
- [x] Rewrite `README.md` and `README_EN.md` into short repo entrypoints that route into `docs/`.
- [x] Created `docs/project/architecture.md` and the stable component index under `docs/project/`.
- [x] Created first-class `docs/sprint1/` through `docs/sprint6/` migration entrypoints while detailed evidence still lives under `docs/development/sprintX/`.
- [x] Align `AGENTS.md`, `CLAUDE.md`, `.claude/`, and `.github/` with the migrated docs surfaces, hardware gate, and platform split.
- [x] Added `docs/target/legacy_doc_inventory.md` as the repo-wide cleanup inventory for old docs and compatibility shims.
- [x] Retired the duplicate top-level `docs/development` checklist, error, question, and mismatch trackers.
- [ ] Consolidate overlapping historical proposal and investigation families before deleting their source files.

## Earlier cleanup baseline

- [x] Added a central docs hub at `docs/README.md`.
- [x] Aligned top-level `README.md` and `README_EN.md` with the current native CAN bringup and vendored `pyAgxArm` runtime path.
- [x] Repointed repo-level MoveIt quickstart examples and stable diagrams to the current canonical wrapper path (`start_agx_arm_components.launch.py` for the operational matrix, `start_agx_arm_moveit.launch.py` as the lower-level wrapper).
- [x] Updated `scripts/setup_agx_arm_runtime_env.sh` to prefer `vendor/pyAgxArm` and only fall back to `../pyAgxArm`.
- [x] Reprioritized `docs/CAN_USER.md` and `docs/CAN_USER_EN.md` so native `mttcan` is the current first path and USB role-based setup is clearly fallback.
- [x] Normalized `agx_arm_moveit` and `agx_arm_mit_controller` README examples to explicit execution profiles and current CAN naming.
- [x] Added `src/agx_arm_sim/agx_arm_description/README_EN.md` to restore the English documentation path.
- [x] Marked the Sprint 2 MIT workflow note as historical and redirected active workflow references to `docs/control/teach_and_run.md`.
- [x] Temporarily added cross-sprint `docs/development/checklist.md`, `docs/development/errors_and_fixes.md`, and `docs/development/open_questions.md` during the earlier cleanup pass; these duplicate trackers are now retired again.
- [x] Aligned active OmniHand bringup docs and runtime error text with the bridge's SDK auto-discovery path.
- [x] Re-scoped the Chinese `agx_arm_description` package README so live OmniHand bridge ownership points at `agx_arm_ctrl` instead of future-work language.
- [x] Removed historical OmniHand asset docs from `docs/assets/omnihand/`, promoted the surviving active-joint mapping under a stable filename, and redirected active references to the stable validation docs.

## Still open

- [x] Deprecated launch defaults and public runtime names such as `can0` and `can_nero` in favor of `can_nero_right` and `can_nero_left`.
- [ ] Decide how much operational launch detail should stay in package-local READMEs versus moving entirely into `docs/control/`.