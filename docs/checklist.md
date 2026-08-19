# Global Checklist

Repo-wide documentation and consistency checklist.

## Execution Sprint Snapshot

| Sprint | Area | Status | Summary |
| --- | --- | --- | --- |
| 1 | Asset audit and model baseline | complete | Stable asset inventories and validation docs are promoted; missing AGV CAD and broader USD coverage remain external gaps. |
| 2 | Common environment and OmniHand bridge direction | complete | Shared ROS semantics, package boundaries, and the repo-owned OmniHand bridge direction are established. |
| 3 | Nero planning and control hardening | complete | TRAC-IK + OMPL planning, `/compute_ik`, and the MIT execution path are validated. |
| 4 | Duo body + OmniHand system baseline | complete | Body-mounted Duo staging, prefixed multi-arm MoveIt, and per-arm hand-aware profiles are landed. |
| 5 | CAN transport + arm-plus-hand | complete | Native `mttcan` CAN FD side buses and pinned `pyAgxArm` runtime are the baseline; shared-bus caveats remain operational guidance. |
| 6 | Coordinated tasks + skill layer | active | Coordinator, dual-arm teach flow, and semantic OmniHand skill abstraction are landed. `tea_pour_left_v1` completed end to end on hardware 2026-08-17, twice in one stack. Hefeweizen still needs calibrated tactile thresholds. |
| refactor | V02 runtime refactor (device authority, parallel operation, contract consolidation) | active | Safety, CPU relief, and parallel arm-plus-hand operation. Canonical plan: `docs/sprint_refactor/planning/integration_plan.md`, binding constraints C1-C8. |

## Current System Focus

- **The Refactor Runtime RC closed 2026-08-17.** Coordinated demo work resumes; the remaining Phase-4/5/6 items are follow-up engineering, not prerequisites for productive motion development. Pull a Phase-4 item forward only when it blocks an interface the demo actually uses
- `sprint_refactor` remains the implementation sprint for that follow-up work; `docs/sprint_refactor/planning/integration_plan.md` is canonical over any older plan
- Sprint 6 resumes against the refactored runtime contracts; its step-and-settle and shared-bus notes stay superseded by the per-device CAN topology
- the normal topology is one CAN bus per device (arms `can_nero_left` / `can_nero_right` native, hands `hand_left` / `hand_right` on USB-CAN FD adapters); `shared_per_side` is a selectable degraded compatibility mode, not a production focus
- keep the public ROS surface agx_arm-centric and keep arm execution per-arm at the MIT action boundary while coordination grows above it
- L1 and L2 are the standing per-phase software regression gates; L3 evidence is required wherever hardware behaviour is claimed
- **`tea_pour_left_v1` completed on hardware 2026-08-17**, twice in one stack, on the post-refactor contracts and on the *existing* taught data — the re-teach that was expected to be needed first was not. Evidence: `docs/sprint6/evidence/tea_pour_left_v1_2026-08-17.md`
- the Hefeweizen ladder is the remaining demo target and is blocked on a calibrated `contact_threshold`, not on the runtime

### Previous state (superseded 2026-08-16)

Until this date the snapshot listed Sprint 6 as active, with its hardware validation of tactile
thresholds, grasp presets, shared-bus timing under sustained motion, and the Hefeweizen demo ladder
as the current focus, and kept the shared arm-plus-hand CAN caveat as standing operational guidance.
The V02 refactor took priority over that work, and per-device CAN buses replaced the shared-bus
assumption underneath it.

## Documentation cleanup closure

- [x] Re-audited the repo against the target structure and reduced `docs/target/README.md` to the stable repo target and documentation-ownership surface.
- [x] Created `docs/control/environment.md` as the new canonical page for Python environments, ROS overlays, and build or test wrappers.
- [x] Promoted the latest shared arm-plus-hand CAN findings into `docs/errors_and_fixes.md` and aligned script comments with the stable safety guidance.
- [x] Retired the old control launch shim in favor of `docs/control/bringups/launches.md`.
- [x] Rewrite `README.md` and `README_EN.md` into short repo entrypoints that route into `docs/`.
- [x] Created `docs/project/architecture.md` and the stable component index under `docs/project/`.
- [x] Created first-class `docs/sprint1/` through `docs/sprint6/` sprint surfaces and migrated their retained evidence out of the old development tree.
- [x] Align `AGENTS.md`, `CLAUDE.md`, `.claude/`, and `.github/` with the migrated docs surfaces, hardware gate, and platform split.
- [x] Removed the separate legacy-doc inventory once repo-wide references were clean; git history and sprint evidence are now the audit trail.
- [x] Clarified `vendor/pyAgxArm` as the in-repo runtime pin and the external `pyAgxArm` checkout as the upstream-sync and pin-preparation workflow.
- [x] Retired the duplicate top-level `docs/development` checklist, error, question, and mismatch trackers.
- [x] Consolidated the main overlapping historical proposal and investigation families before deleting their source files.
- [x] Moved the thematic Physical AI roadmap into `docs/project/roadmap_and_phases.md` and folded the old progress tracker into this checklist.
- [x] Moved the pre-sprint Physical AI brainstorm into `docs/sprint_physAI/brainstorm.md`.

## Earlier cleanup baseline

- [x] Added a central docs hub at `docs/README.md`.
- [x] Aligned top-level `README.md` and `README_EN.md` with the current native CAN bringup and vendored `pyAgxArm` runtime path.
- [x] Repointed repo-level MoveIt quickstart examples and stable diagrams to the current canonical wrapper path (`start_agx_arm_components.launch.py` for the operational matrix, `start_agx_arm_moveit.launch.py` as the lower-level wrapper).
- [x] Updated `scripts/setup_agx_arm_runtime_env.sh` to prefer `vendor/pyAgxArm` and only fall back to `../pyAgxArm`.
- [x] Reprioritized `docs/CAN_USER.md` and `docs/CAN_USER_EN.md` so native `mttcan` is the current first path and USB role-based setup is clearly fallback.
- [x] Normalized `agx_arm_moveit` and `agx_arm_mit_controller` README examples to explicit execution profiles and current CAN naming.
- [x] Added `src/agx_arm_sim/agx_arm_description/README_EN.md` to restore the English documentation path.
- [x] Marked the Sprint 2 MIT workflow note as historical and redirected active workflow references to `docs/control/bringups/teach_and_run.md`.
- [x] Temporarily added duplicate cross-sprint top-level trackers during the earlier cleanup pass; those compatibility surfaces are now retired again.
- [x] Aligned active OmniHand bringup docs and runtime error text with the bridge's SDK auto-discovery path.
- [x] Re-scoped the Chinese `agx_arm_description` package README so live OmniHand bridge ownership points at `agx_arm_ctrl` instead of future-work language.
- [x] Removed historical OmniHand asset docs from `docs/assets/omnihand/`, promoted the surviving active-joint mapping under a stable filename, and redirected active references to the stable validation docs.

## Resolved repo-wide decisions

- [x] Deprecated launch defaults and public runtime names such as `can0` and `can_nero` in favor of `can_nero_right` and `can_nero_left`.
- [x] Package-local READMEs may keep package-local behavior, parameters, and focused examples, but the canonical system bringup matrix stays in `docs/control/`.