# Generated Vs Source Assets

status: ACTIVE_SPRINT2_BASELINE
last_updated: 2026-05-14

## Purpose

This document defines which files are treated as source-managed and which are treated as generated or runtime outputs.

The goal is to stop Sprint 2 from mixing canonical robot assets with ephemeral build outputs.

## Source-Managed Assets

Treat these as source assets that should be edited deliberately and reviewed as code or curated documentation:

- URDF, Xacro, meshes, and related description assets under `src/agx_arm_sim/agx_arm_description`
- MoveIt configs and launch files under `src/agx_arm_moveit`
- runtime launch files and nodes under `src/agx_arm_ctrl`
- MIT-controller code and curated configs under `src/agx_arm_mit_controller`
- ROS message definitions under `src/agx_arm_msgs/msg`
- Copilot-native repo guidance under `.github/` and `AGENTS.md`
- reusable local tool surfaces under `tools/`
- curated calibration/config data such as `config/nero_gravity_calibration.json`
- promoted stable docs under `docs/assets`, `docs/control`, `docs/project`, and the fixed coordination docs in `docs/development`
- vendored third-party source tracked intentionally under `vendor/`

## Generated Or Runtime Assets

Treat these as generated outputs that should not be hand-edited or treated as canonical source:

- `build/`
- `install/`
- `log/`
- transient exported URDF or SRDF products created during tool runs
- temporary RViz configs or derived simulation artifacts created only for local runs
- ad hoc run outputs in `logs/` unless explicitly promoted into a curated reference document

## Promotion Rule

A generated artifact can be promoted into source only when all of the following are true:

1. it is reproducible or intentionally measured,
2. the team wants to treat the reviewed output as canonical input going forward,
3. it is stored in a source-managed location rather than left inside build or runtime directories,
4. its origin is documented.

Examples:

- a reviewed SRDF or self-collision matrix can become source once committed in `src/agx_arm_moveit/config/`
- a measured payload or calibration file can become source once committed under `config/`
- a one-off generated file under `build/` or `install/` should remain generated

## Vendor Asset Rule

Vendor-provided source belongs in the tracked vendor repository or in normalized repo-owned copies derived from it.

Do not patch unpacked build outputs when the real change belongs in the workspace fork or in the normalized repo-owned asset tree.

## Documentation Rule

Use this split for documentation:

- roadmap, progress, and component routing in the fixed top-level `docs/development/` docs
- working logs, experiments, and sprint notes in `docs/development/sprintN/`
- promoted stable state and policy docs in the top-level `docs/` tree

Once a Sprint 2 decision is settled, prefer promotion into a stable top-level document over leaving the canonical answer buried in a sprint note.