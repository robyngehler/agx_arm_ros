# Development — Roadmap & Historical Working Notes

This tree now holds the program-level roadmap and progress notes, component routing, and the
historical working evidence that has not yet been fully migrated into first-class `docs/sprintX/`
surfaces.

## Current role

- keep roadmap and progress documents that do not belong to one sprint folder
- keep component routing notes that help promotion into stable docs
- preserve historical sprint-local evidence until it is either promoted or moved

The active top-level sprint entrypoint is now `../sprint6/`. Earlier sprint entrypoints exist under
`../sprint1/` through `../sprint5/`.

## Layout

- `nero_physical_ai_roadmap.md`, `nero_physical_ai_progress.md` — program roadmap and status.
- `component_implementation_map.md` — component → owning package → docs routing.
- `sprintN/` — existing historical sprint working folders and attached evidence.
- `sprint_physAI/` — separate physical-AI track notes.

## Where durable docs live (not here)

- How to run the system (launch + arguments, teach loop) → `../control/`
- Component / runtime / OmniHand / control architecture → `../assets/` and `../project/architecture.md`
- Human repository structure & architecture → `../project/`
- Cross-repo checklist / fixes / open questions → `../checklist.md`, `../errors_and_fixes.md`, `../open_questions.md`
- Agent workflow, naming, and ROS2-practice rules → `.claude/rules/` and `.github/instructions/`

## Migration note

Treat `docs/sprintX/` as the user-facing sprint entrypoints. Use this tree when you need the older
working notes, planning detail, or evidence that has not yet been promoted.

The authoritative cleanup inventory for legacy docs now lives in `../target/legacy_doc_inventory.md`.

The temporary top-level `docs/development/checklist.md`, `errors_and_fixes.md`, `open_questions.md`,
and `mismatches_todo.md` were migration scaffolding and are retired.
