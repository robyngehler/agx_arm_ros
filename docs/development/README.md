# Development — Target & Index

This tree tracks the active development of the Duo Nero system. It holds the program-level
roadmap/progress plus one folder per sprint.

## Current target

A stable, reproducible Duo runtime moving toward coordinated dual-arm (and dual-hand) tasks:

- **Transport stable** — arms on native `mttcan` (`can0`/`can1`) with `one-shot on`; one bus per
  arm; arm+hand-per-bus under evaluation. See `sprint5/`.
- **Control layer pinned** — `pyAgxArm` vendored as the `vendor/pyAgxArm` submodule; runtime no
  longer drifts. See `../project/control_layer_and_dependencies.md`.
- **Duo body baseline** — prefixed MoveIt + per-arm MIT wrappers, hand-aware config profiles.
  See `sprint4/`.

## Layout

- `nero_physical_ai_roadmap.md`, `nero_physical_ai_progress.md` — program roadmap and status.
- `component_implementation_map.md` — component → owning package → docs routing.
- `sprintN/` — each sprint: `README.md` (target), `checklist.md`, `errors_and_fixes.md`,
  `open_questions.md`, and `planning/` for the valuable insight docs.
- **Current sprint:** `sprint6/` (Activity-DAG coordinator, dual-arm teach, duo trajectory sync).

## Where durable docs live (not here)

- How to run the system (launch + arguments, teach loop) → `../control/`
- Component / runtime / OmniHand / control architecture → `../assets/`
- Human repository structure & architecture → `../project/`
- Agent workflow, naming, and ROS2-practice rules → `.claude/rules/` and `.github/instructions/`
