# Errors And Fixes

## 2026-05-17

### Launch and runtime understanding was spread across code and docs

- Symptom: launch order, ROS graph behavior, file composition, and config dataflow had to be reconstructed from several launch files, xacros, and control docs.
- Impact: Sprint 2 context recovery stayed slower than it should be for developers and agents.
- Fix: promoted a stable diagram set into `docs/project/repo_interaction_diagrams.md` and created the Sprint 2 working-note folder for the remaining runtime-baseline work.

### Sprint 2 had no working-note folder yet

- Symptom: the new two-tier docs layout had Sprint 1 working notes, but no matching Sprint 2 folder for checklist and issue tracking.
- Impact: Sprint 2 progress existed only in stable docs and runtime/control docs, which made it harder to record in-flight questions without adding another top-level source.
- Fix: created `docs/development/sprint2/` with `README.md`, `checklist.md`, `errors_and_fixes.md`, and `open_questions.md`.