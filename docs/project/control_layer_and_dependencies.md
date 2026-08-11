# pyAgxArm Control-Layer Pin & Submodule Migration

**Status:** Submodule wired · **Date:** 2026-06-12 (submodule landed 2026-06-15;
pin advanced 2026-07-24; reviewed 2026-08-11)

This file is the canonical record of the two-tier vendor workflow that the V02
refactor references as constraint C3
(`docs/sprint_refactor/planning/integration_plan.md`): the pinned submodule is
the execution path, and vendor development happens in a separate checkout.

The `agx_arm_ros` runtime drives the Nero arms through **pyAgxArm**. This file records exactly
which pyAgxArm the control layer runs, how it is installed, and how the pinned in-repo runtime
source relates to the separate external development checkout used to prepare new pins.

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
- Commit: **`4f52610`** ("feat(comm): make silent TX loss observable"), on top of `37d87e6`
  ("Add minimal Nero validation scripts"), which sits on upstream
  `agilexrobotics/pyAgxArm@19e28e8`.
- Tag: **`control-layer-pin-2026-07-24`** → `4f52610`. The earlier
  `control-layer-pin-2026-06-12` → `37d87e6` is history.
- Submodule: `vendor/pyAgxArm` → fork `github.com/robyngehler/pyAgxArm`, gitlink pinned at the
  tag. Fork `master` is newer upstream; we intentionally pin to the tag.

### Reproduce the install (system 3.10, editable)

Runtime pin inside this repo:

```bash
python3.10 -m pip uninstall -y pyAgxArm
python3.10 -m pip install --user -e vendor/pyAgxArm --no-build-isolation
```

Intentional development verification against an external checkout:

```bash
python3.10 -m pip uninstall -y pyAgxArm          # remove any stale non-editable copy first
python3.10 -m pip install --user -e /path/to/pyAgxArm --no-build-isolation
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
- Treat `vendor/pyAgxArm` as the runtime and install pin inside this repo. Do not treat the
  external development checkout as an implicit runtime baseline.

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

Ongoing workflow:

- Keep the external checkout (for example `/home/user/workspace/pyAgxArm` or a sibling
  `../pyAgxArm`) as the place where upstream pulls, rebases, and SDK feature work happen.
- Pull vendor changes there, land local commits there, push them to the fork, and tag the chosen
  commit there.
- After validation, bump `vendor/pyAgxArm` in `agx_arm_ros` to that new tag or commit so runtime,
  install, and repo-local validation stay pinned to the reviewed baseline.
- Optional: keep a named fork branch such as `control-layer` for discoverability in addition to the
  tags that the submodule pin follows.

### Recreating the external checkout

As of 2026-08-11 the development checkout is **not present on this host**; only the
pinned submodule exists. Any vendor-side work — starting with the V02 velocity
fix — needs it back, set up so upstream vendor updates keep flowing in:

```bash
git clone git@github.com:robyngehler/pyAgxArm.git /home/user/workspace/pyAgxArm
cd /home/user/workspace/pyAgxArm
git remote add upstream https://github.com/agilexrobotics/pyAgxArm.git
git fetch upstream --tags
git checkout -b control-layer control-layer-pin-2026-07-24   # same commit the repo runs
```

Do not point the editable install at this checkout while validating repo
behaviour: the runtime pin stays `vendor/pyAgxArm` unless a development
verification is explicitly intended (see the two install recipes above).
