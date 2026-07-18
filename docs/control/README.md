# Control Docs

Operational source of truth for bringing up and using the current system.

## Canonical docs

- `environment.md`: Python environment split, ROS overlay handling, build/test wrappers, and platform caveats
- `bringups/launches.md`: baseline, tool, and demo launch map for native runtime, MoveIt, MIT, OmniHand, and coordinator slices
- `teach_and_run.md`: current teach, record, replay, anchor capture, and coordination-facing motion workflow

## Scope

Keep stable runnable workflows here. Package-local READMEs may summarize local behavior, but the canonical launch combinations should live in this directory.

`environment.md` owns the system-Python versus Conda split, while `bringups/launches.md` owns the
baseline launch taxonomy and the rule that `start_agx_arm_components.launch.py` should normally be
driven through `execution_profile` presets rather than rebuilt from ad hoc per-command model or
effector overrides.