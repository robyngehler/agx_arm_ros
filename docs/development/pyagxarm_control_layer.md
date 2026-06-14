# pyAgxArm Control-Layer Pin & Submodule Migration

**Status:** In progress · **Date:** 2026-06-12

The `agx_arm_ros` runtime drives the Nero arms through **pyAgxArm**. This file records exactly
which pyAgxArm the control layer runs, how it is installed, and the plan to vendor it as a
pinned git submodule so the source never silently drifts again.

## Why this exists

The ROS control stack runs on **system `python3.10`** (ROS Humble). For ~2 months the control
layer silently ran a frozen, non-editable pyAgxArm snapshot from 2026-04-09 in
`~/.local/lib/python3.10/site-packages`, while all newer development (Nero v112 driver, comm
error-recovery rework) lived only in a conda **base / python 3.13** editable install that
cannot even import `rclpy`. See `single_vs_multi_arm_control_chain.md` and
`nero_bus_problem_proposal.md` for the bus work this surfaced during.

## Current control-layer source (the pin)

- Interpreter: **`/usr/bin/python3.10`** (ROS Humble). conda base (3.13) is **not** a ROS runtime.
- pyAgxArm: editable install (`pyAgxArm.egg-link` → `/home/user/workspace/pyAgxArm`).
- Commit: **`37d87e6`** ("Add minimal Nero validation scripts"), 1 local commit on top of
  upstream `agilexrobotics/pyAgxArm@19e28e8`.
- Tag in the pyAgxArm repo: **`control-layer-pin-2026-06-12`** (pin only — HW validation pending).

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

**Prerequisite (blocks the submodule):** a team-owned fork remote must exist and contain the
pinned commit. `origin` today is the read-only upstream `agilexrobotics/pyAgxArm`; our pin
`37d87e6` is local-only, so a submodule cannot resolve it elsewhere yet.

1. Create the fork (team org or GitHub fork), e.g. `git@<host>:<team>/pyAgxArm.git`.
2. In the existing checkout, push the pin and tag to the fork:
   ```bash
   cd /home/user/workspace/pyAgxArm
   git remote add fork <fork-url>
   git remote rename origin upstream        # keep upstream for rebases
   git push fork master --tags
   ```
3. Wire the submodule in agx_arm_ros (pinned at the validated commit):
   ```bash
   cd <agx_arm_ros>
   git submodule add <fork-url> vendor/pyAgxArm
   git -C vendor/pyAgxArm checkout control-layer-pin-2026-06-12
   git add .gitmodules vendor/pyAgxArm && git commit -m "vendor: pin pyAgxArm as submodule"
   ```
4. Re-point the editable install at the submodule path and re-verify:
   ```bash
   python3.10 -m pip uninstall -y pyAgxArm
   python3.10 -m pip install --user -e <agx_arm_ros>/vendor/pyAgxArm --no-build-isolation
   ```
5. After hardware validation, add a `hw-validated-<date>` tag and bump the submodule pin to it.

Until step 1 is done, the control layer stays on the local checkout + the
`control-layer-pin-2026-06-12` tag recorded above.
