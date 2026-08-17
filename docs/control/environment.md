# Environment

status: ACTIVE_BASELINE
last_updated: 2026-07-19

## Purpose

This is the operational source of truth for Python environments, ROS overlays, build wrappers, and
test wrappers.

Launch combinations now live in `bringups/launches.md`.

## Environment classes

### Jetson or other `aarch64` ROS plus hardware environment

Use this class when you need live CAN bringup, real arm or OmniHand launches, or vendor SDK probes
against connected devices.

- `sudo` may be required for CAN bringup scripts
- live hardware validation belongs here
- **after any kernel update, check the 40-pin header before anything else:**
  `sudo /opt/nvidia/jetson-io/jetson-io.py`. A kernel update discards the header
  pinmux, and the native arm interfaces then come up UP and ERROR-ACTIVE while
  nothing can be transmitted (`RX=0 TX=0`, sends failing `ENOBUFS`). The hands
  are on USB-CAN FD adapters and are unaffected, so a working hand bus does not
  rule this out. See `docs/errors_and_fixes.md`.

### x86 or editor-only environment without ROS or hardware

Use this class for documentation work, code changes, file-level checks, and offline validation.

- do not claim live hardware validation from this environment
- do not treat x86 timing behavior as a substitute for CAN or real-device validation

## Golden rules

- the production runtime interpreter is **system `/usr/bin/python3.10`**; Conda is optional and not
  part of the validated runtime path
- use `scripts/agx_arm_install_deps.sh` for the apt layer plus the pinned pip layer in
  `requirements.txt`
- use `scripts/colcon_build_system_python.sh` for workspace builds
- use `scripts/setup_trac_ik_overlay.sh` for the TRAC-IK overlay that `agx_arm_moveit` requires
- use `scripts/setup_omnihand_sdk.sh` to build the vendored OmniHand SDK the bridge imports
- do not mix manual `conda activate` with `source install/setup.bash` in the same shell flow
- treat `vendor/OmniHand-Pro-2025` as upstream input, not as a normal workspace package for the
  default repo-wide `colcon build`

## Dependency layers

A provisioned host is defined by four layers. Missing any of them produces a workspace that builds but
fails at runtime:

| Layer | Source of truth | Provisioned by |
|---|---|---|
| apt (system + ROS) | `scripts/agx_arm_install_deps.sh` | that script, steps 1–3 |
| pip (system `python3.10` user site) | `requirements.txt` | that script, step 4 |
| TRAC-IK overlay (source build) | `config/trac_ik_overlay.repos` + `scripts/patches/` | `scripts/setup_trac_ik_overlay.sh` |
| OmniHand vendor SDK (source build) | `vendor/OmniHand-Pro-2025` submodule | `scripts/setup_omnihand_sdk.sh` |

> **A vendor SDK change does not arrive by pulling.** `build/` is gitignored inside the
> `OmniHand-Pro-2025` submodule, so a pull delivers patched *source* while the `.so` the bridge
> actually loads stays whatever you built last. After any submodule update that touches the SDK,
> re-run `scripts/setup_omnihand_sdk.sh` — otherwise the code says one thing and the running system
> does another, and the difference only shows up as a puzzling measurement. The receive-thread fix of
> 2026-08-14 (worth ~100 % of a core per hand) is delivered exactly this way.

`requirements.txt` exists because one pip dependency is load-bearing and cannot come from apt:
Ubuntu 22.04 ships python-can 3.3.2, but the arm's CAN error-recovery path needs the
python-can ≥ 4.0 exception types. See the comments in `requirements.txt`.

For a blank host, follow `../project/jetson_migration.md` rather than assembling these by hand.

## System build path

Install dependencies:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash ./agx_arm_install_deps.sh
```

Build with system Python through the repo wrapper:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl
```

Run package tests from a system-Python ROS shell, not from Conda:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl
colcon test --packages-select agx_arm_ctrl
```

Notes:

- if you use a separate TRAC-IK overlay, set `AGX_ARM_TRAC_IK_OVERLAY=/path/to/install/setup.bash`
  before running the wrapper
- after `rm -rf build/ install/ log/`, the wrapper filters stale local prefix entries under this
  workspace's `install/` path before `colcon` starts
- the wrapper skips the vendor OmniHand SDK by default during repo-wide builds unless you select the
  vendor package explicitly

Full clean rebuild:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
rm -rf build/ install/ log/
bash ./scripts/colcon_build_system_python.sh
```

## Optional Conda runtime path

status: OPTIONAL / NOT IN THE VALIDATED RUNTIME PATH

This path is a development convenience, not the production runtime. The reference Jetson runs no
`agx-arm-runtime` environment at all (`conda env list` shows only `base`), and every dependency the
env would provide is already satisfied under system `python3.10` — including `pinocchio`, which comes
from apt `ros-humble-pinocchio`. Skip this section when provisioning a new host.

Create or update the runtime environment:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/setup_agx_arm_runtime_env.sh
```

The environment script installs `vendor/pyAgxArm` first and falls back to `../pyAgxArm` only when
the vendored copy is unavailable. The vendored copy is the normal runtime baseline inside this repo;
the sibling checkout exists to support the external pyAgxArm development workflow that prepares new
tags or commits before `vendor/pyAgxArm` is bumped.

Run ROS commands through the runtime wrapper instead of activating Conda manually:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_hand
```

The runtime wrapper sources, in order:

1. `/opt/ros/$ROS_DISTRO/setup.bash`
2. the optional `AGX_ARM_TRAC_IK_OVERLAY`
3. the local workspace `install/setup.bash` when it exists

and only then executes the requested command through `conda run`.

## Testing guidance

- prefer `scripts/colcon_build_system_python.sh --packages-select <pkg>` for package builds
- prefer `colcon test --packages-select <pkg>` from a system-Python ROS shell for ROS package tests
- prefer `/usr/bin/python3 -m pytest ...` when the test targets source-only Python helpers in the ROS
  build path
- use `scripts/run_in_ros_conda.sh -- python3 -m pytest ...` when the test depends on Conda-managed
  runtime libraries such as `pinocchio`
- call out explicitly when hardware validation could not be run because the active environment is
  editor-only or lacks live devices

## Common failure patterns

If ROS imports such as `xacro` or `yaml` disappear after switching into Conda, the usual causes are:

1. `PYTHONPATH` was replaced instead of appended by sourced overlays
2. the Conda environment Python version does not match the ROS distro
3. the command was run in a manually activated Conda shell instead of through `scripts/run_in_ros_conda.sh`

If a repo-wide build unexpectedly tries to compile `omni_hand_pro_2025`, use the repo wrapper instead
of raw `colcon build`.

## Related operational docs

- `bringups/launches.md`: current launch matrix
- `bringups/teach_and_run.md`: teach, replay, and coordination-facing workflow guidance
- `../project/jetson_migration.md`: provisioning a new Jetson host from a blank Ubuntu 22.04 install
  (this file assumes the host is already provisioned)