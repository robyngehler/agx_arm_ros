# Project Docs

Stable repository structure, architecture decisions, and component ownership.

## Canonical docs

- `repository_structure.md`: package ownership, staging boundaries, and documentation split
- `architecture.md`: stable runtime, launch, and configuration interaction diagrams
- `components/README.md`: stable component index and owning surfaces
- `components/implementation_map.md`: cross-component ownership and doc routing map
- `roadmap_and_phases.md`: long-term Physical AI roadmap and thematic phase sequence
- `control_layer_and_dependencies.md`: pyAgxArm control-layer pin and dependency ownership

## Scope

Keep durable repo-level structure and architecture decisions here. Environment and launch usage live
under `../control/`. Sprint-local evidence now lives directly in the matching `../sprintX/`
surfaces.