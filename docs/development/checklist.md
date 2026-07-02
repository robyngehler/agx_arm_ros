# Development Checklist

Cross-sprint checklist for documentation cleanup, promotion, and development-layer consistency.

## Done in this pass

- [x] Added top-level development base docs (`README.md`, `checklist.md`, `errors_and_fixes.md`, `open_questions.md`) so cross-sprint follow-ups no longer live only inside sprint folders.
- [x] Repointed active repo-level launch guidance away from compatibility wrappers and toward `docs/control/bringup.md` plus `start_agx_arm_components.launch.py`.
- [x] Aligned active OmniHand bringup docs with the bridge's SDK auto-discovery behavior.
- [x] Updated the development overview so the native CAN baseline and the shared arm+hand bus caveat are no longer conflated.
- [x] Removed historical OmniHand document artifacts from `docs/assets/omnihand/` and kept only the promoted stable runtime notes outside `docs/development/`.

## Still open

- [ ] Keep promoting stable outcomes from sprint folders into `docs/control/`, `docs/assets/`, and `docs/project/` so sprint folders remain evidence, not operational source of truth.
- [ ] Recheck package-local READMEs after future launch-surface changes so they do not drift back into maintaining their own launch matrix.