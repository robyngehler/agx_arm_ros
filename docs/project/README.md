# Project Docs

Stable repository structure, architecture decisions, and component ownership.

## Canonical docs

- `repository_structure.md`: package ownership, staging boundaries, and documentation split
- `architecture.md`: stable runtime, launch, and configuration interaction diagrams
- `components/README.md`: stable component index and owning surfaces
- `control_layer_and_dependencies.md`: pyAgxArm control-layer pin and dependency ownership
- `python_environment_workflow.md`: compatibility pointer to the canonical control environment doc
- `repo_interaction_diagrams.md`: compatibility pointer to `architecture.md`

## Scope

Keep durable repo-level structure and architecture decisions here. Environment and launch usage live
under `../control/`. Sprint-local discovery and historical evidence stay under `../development/`
until promoted or migrated into first-class `../sprintX/` surfaces.