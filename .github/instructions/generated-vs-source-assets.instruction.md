---
description: "Use when deciding whether a file in agx_arm_ros is canonical source, generated output, or a promoted artifact that should move into a stable source-managed location."
---

# Generated Vs Source Assets

Keep Sprint 2 changes grounded in source-managed files and avoid treating runtime outputs as canonical input.

## Source-Managed

- `src/`
- `vendor/`
- `docs/`
- `config/`
- `scripts/`
- `.github/`

## Generated Or Runtime-Managed

- `build/`
- `install/`
- `log/`
- transient generated URDF, SRDF, RViz, or analysis outputs
- ad hoc run outputs in `logs/` unless deliberately promoted

## Promotion Rules

- promote only reproducible or intentionally measured outputs
- move promoted artifacts into a source-managed location
- document the origin when promoting a generated artifact into source
- patch vendor source in the tracked vendor fork or normalized repo-owned copies, not in unpacked build output