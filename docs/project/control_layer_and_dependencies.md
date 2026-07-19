# pyAgxArm Control-Layer Pin & Submodule Migration

**Status:** Submodule wired · **Date:** 2026-06-12 (submodule landed 2026-06-15)

The `agx_arm_ros` runtime drives the Nero arms through **pyAgxArm**. This file records exactly
which pyAgxArm the control layer runs, how it is installed, and the plan to vendor it as a
pinned git submodule so the source never silently drifts again.

## Why this exists

The ROS control stack runs on **system `python3.10`** (ROS Humble). For ~2 months the control
layer silently ran a frozen, non-editable pyAgxArm snapshot from 2026-04-09 in
`~/.local/lib/python3.10/site-packages`, while all newer development (Nero v112 driver, comm
error-recovery rework) lived only in a conda **base / python 3.13** editable install that
cannot even import `rclpy`. See `../assets/control/single_vs_multi_arm_control_chain.md` and
`../sprint5/evidence/can_transport_decision.md` for the bus work this surfaced during.

## Current control-layer source (the pin)

- Interpreter: **`/usr/bin/python3.10`** (ROS Humble). conda base (3.13) is **not** a ROS runtime.
- pyAgxArm: editable install (`pyAgxArm.egg-link` → `vendor/pyAgxArm` **submodule**).
- Commit: **`37d87e6`** ("Add minimal Nero validation scripts"), 1 local commit on top of
  upstream `agilexrobotics/pyAgxArm@19e28e8`.
- Tag: **`control-layer-pin-2026-06-12`** → `37d87e6` (pin only — HW validation pending).
- Submodule: `vendor/pyAgxArm` → fork `github.com/robyngehler/pyAgxArm`, gitlink pinned at the
  tag. Fork `master` is newer upstream (`97f56a6`); we intentionally pin to `37d87e6`.

### Reproduce the install (system 3.10, editable)

```bash
python3.10 -m pip uninstall -y pyAgxArm          # remove any stale non-editable copy first
python3.10 -m pip install --user -e <pyAgxArm> --no-build-isolation
```

`--no-build-isolation` is required: the system `setuptools` (59.6) predates PEP 660, so pip
falls back to `setup.py develop`; with build isolation the egg-link write fails, without it the
system setuptools links the source directly. Verify:

```bash
python3.10 -c "import pyAgxArm, os; print(os.path.dirname(pyAgxArm.__file__))"   # must be the checkout
```

## Drift-prevention rules

- The control layer is **system 3.10**. Always install/patch pyAgxArm with `python3.10 -m pip`,
  never via conda base (`python`/`python3` resolve to 3.13 when conda base is active).
- Build ROS with `scripts/colcon_build_system_python.sh` (it strips conda from `PATH`).
- Keep repo-recovery logic in `agx_arm_ctrl` (node-side, comm-model-agnostic). Patch pyAgxArm
  only when something must live below ROS; treat the SDK as upstream input, not the ROS contract.

## Submodule migration plan

Goal: vendor pyAgxArm as a pinned submodule at `vendor/pyAgxArm` (mirrors the `vendor/`
convention), tracking a team-owned fork with `upstream` = agilexrobotics.

Done (2026-06-15):

1. ✅ Fork created: `github.com/robyngehler/pyAgxArm`; `origin`→`upstream`, `fork` added in
   `/home/user/workspace/pyAgxArm`. Tag `control-layer-pin-2026-06-12` (commit `37d87e6`) pushed
   to the fork. (Fork `master` push was rejected as non-fast-forward — it carries newer upstream;
   the tag carries our commit, which is all the submodule pin needs.)
2. ✅ Submodule wired: `git submodule add github.com/robyngehler/pyAgxArm vendor/pyAgxArm`,
   checked out at the tag; gitlink pinned at `37d87e6`.
3. ✅ Editable install re-pointed: `python3.10 -m pip install --user -e vendor/pyAgxArm
   --no-build-isolation`; verified `python3.10` imports from `vendor/pyAgxArm` with v112 present.

Remaining:

- Optional: push our baseline to a named fork branch (e.g. `control-layer`) for discoverability
  (`git push fork master:refs/heads/control-layer` — needs GitHub credentials).
- After hardware validation, tag `hw-validated-<date>` and bump the submodule pin to it.
- The old loose checkout at `/home/user/workspace/pyAgxArm` is no longer used by the runtime; it
  still holds the `upstream`/`fork` remotes for rebasing. Develop there, push to the fork, then
  bump `vendor/pyAgxArm`.
