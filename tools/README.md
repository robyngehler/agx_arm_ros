# tools

This directory is reserved for reusable repo-local developer or integration tools that are not ROS packages under `src/` and not one-off scripts under `scripts/`.

Use `tools/` when a helper surface needs its own small project structure, assets, or documentation.

Use `scripts/` for single-purpose automation, setup, or transient workflow helpers.

Current state:

- the directory exists to mirror the structure used by the local Copilot template without inventing placeholder tooling
- add concrete tool projects here only when they provide durable value to the repo workflow