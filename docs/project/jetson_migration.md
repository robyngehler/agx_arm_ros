# Jetson Migration Guide — Setting Up The Software Stack On A Second AGX Orin

status: ACTIVE_BASELINE
last_updated: 2026-07-30
audited_against: `ROS2_Duo_System_V02` @ `edcc5d7`

## Purpose

Reproduce the current `agx_arm_ros` software stack 1:1 on a **second Jetson AGX Orin** that starts
from a blank Ubuntu 22.04 / L4T install, by cloning this repository at its current state.

This document is the migration counterpart to `../control/environment.md`: that file describes how to
*operate* an already-provisioned host, this one describes how to *create* one.

## Reference host (the machine being migrated from)

| Property | Value |
|---|---|
| Board | Jetson AGX Orin (`p3737-0000` + `p3701-0005`) |
| OS | Ubuntu 22.04.5 LTS, `aarch64` |
| L4T | R36.2.0 (`/etc/nv_tegra_release`), kernel `5.15.122-tegra` |
| ROS | ROS 2 Humble, `ros-humble-desktop` |
| System Python | `/usr/bin/python3.10` — **this is the ROS runtime interpreter** |
| Conda | Miniforge3 at `~/miniforge3`, base = Python 3.13 — **not a ROS runtime** |
| CAN transport | Jetson native `mttcan`, 40-pin header, CAN FD 1M/5M |
| CUDA | 12.2 (on `PATH`/`LD_LIBRARY_PATH`, not required by this stack) |

Verify the same values on the target with:

```bash
cat /etc/nv_tegra_release; uname -m -r; lsb_release -d
```

### On updating L4T / JetPack

The stack itself is not L4T-pinned; it depends on L4T only through two things:

1. the `mttcan` native CAN driver exposing a writable `tdc_offset` sysfs attribute, and
2. Jetson-IO being able to pin CAN1/CAN2 onto the 40-pin header.

Both are present on R36.2.0. If the target ships a newer L4T (R36.3/R36.4, JetPack 6.x), **the ROS
side is unaffected** — Ubuntu 22.04 / `aarch64` / Humble stay valid. What must be re-verified after
any L4T change is Step 3 below (`tdc_offset` present, `can0`/`can1` come up in FD mode at 5 Mbit with
BRS). Do not assume the TDCR value carries over to a different transceiver or a different L4T.

---

## Audit findings that shape this guide

The audit behind this guide (2026-07-30) found that the setup surface could not reproduce a working
host. Findings 1–4 and 6 were **fixed in the repo**; the rest are inherent to the platform and are
handled by the steps below.

| # | Finding | Status |
|---|---|---|
| 1 | **No `requirements.txt` existed.** Python dependencies were split across `scripts/agx_arm_install_deps.sh` (apt), `config/agx_arm_runtime.conda.yaml` (conda), and an undocumented set of `pip --user` installs. | **Fixed** — `requirements.txt` added and installed by the deps script. |
| 2 | `agx_arm_install_deps.sh` installed **apt `python3-can` (Jammy candidate 3.3.2)**, but the control layer uses `can.CanOperationError` / `can.CanInitializationError` (`vendor/pyAgxArm/pyAgxArm/protocols/can_protocol/comms/can_comm.py:129,178`) — **python-can ≥ 4.0** APIs. The workspace built fine and failed at runtime. | **Fixed** — apt `python3-can` dropped; the pinned pip version is installed and the version is asserted. |
| 3 | **TRAC-IK was an unmanaged source overlay** (`~/workspace/trac_ik_ws`), required by `agx_arm_moveit`, with no Humble/arm64 apt package — carrying a header patch that existed **only as uncommitted local state** on the reference host. | **Fixed** — pin in `config/trac_ik_overlay.repos`, patch in `scripts/patches/`, build via `scripts/setup_trac_ik_overlay.sh`. |
| 4 | The OmniHand SDK is consumed from `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`, but **`build/` is gitignored inside the submodule**, so a fresh clone has no compiled SDK — and rebuilding it silently picks conda's Python 3.13. | **Fixed** — `scripts/setup_omnihand_sdk.sh` pins the interpreter and verifies the import. |
| 5 | The OmniHand submodule uses an **SSH remote** (`git@github.com:robyngehler/OmniHand-Pro-2025.git`). | Inherent — Step 4 sets up key access. |
| 6 | The documented conda env `agx-arm-runtime` **does not exist on the reference host**, and everything it provides (`pinocchio` included) comes from apt under `/usr/bin/python3.10`. | **Fixed** — demoted to optional in `../control/environment.md`; Step 12 marks it skippable. |
| 7 | Native CAN on the 40-pin header requires a **Jetson-IO pinmux overlay** — a boot-level change no repo script can perform. | Inherent — Step 3. |
| 8 | The reference host's `extlinux.conf` carries eight `JetsonIO-CAN-TDCR-*` boot entries and `/boot/dtb_can_tdcr/*.dtb`. **All eight DTBs are byte-identical** (`md5sum`), confirming the boot-DTB TDCR approach was a no-op. | **Deliberately not reproduced** — only the sysfs TDCR path is documented. |

Good news, verified: all robot meshes, URDF/xacro, MoveIt configs, the activity catalogue, taught
poses and anchors (`src/agx_arm_coordination/config/`, `src/agx_arm_mit_demos/config/`) **are tracked
in git**. The `nero_meshes.zip` that `.gitignore` excludes is a redundant archive of already-tracked
`.stl` files — nothing is lost. There is **no out-of-tree runtime data to migrate.**

---

## Step 1 — Base OS and system packages

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential cmake git curl gnupg lsb-release \
    python3-dev python3-pip \
    can-utils ethtool \
    libeigen3-dev libnlopt-dev libnlopt-cxx-dev liborocos-kdl-dev
```

`libnlopt*` and `liborocos-kdl-dev` are TRAC-IK build dependencies (Step 8); the rest is the shared
base. System `cmake` 3.22.1 and `gcc` 11.4 are sufficient (the vendor SDK needs ≥ 3.16 and C++20).

Locale — MoveIt misparses floats under a non-`en_US` numeric locale:

```bash
echo 'export LC_NUMERIC=en_US.UTF-8' >> ~/.bashrc
```

## Step 2 — ROS 2 Humble

```bash
sudo apt install -y software-properties-common && sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=1.2.0
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep python3-vcstool
sudo rosdep init && rosdep update
```

The reference host runs `ros-humble-desktop` (not `ros-base`): RViz and the rqt tooling are part of
the working setup. Then:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## Step 3 — Native CAN on the 40-pin header (boot-level, needs reboot)

This is the one step with no repo script, and the most common migration blocker. The Duo arms **and**
both OmniHands run on the Jetson native `mttcan` controllers via the 40-pin header — not on USB
adapters.

### 3a. Pin CAN onto the header with Jetson-IO

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

`Configure Jetson 40pin Header` → `Configure header pins manually` → enable **`can0`** and **`can1`**
→ `Save pin changes` → `Save and reboot to reconfigure pins`.

This writes `/boot/kernel_tegra234-<board>-nv-hdr40-user-custom.dtbo` and adds a `JetsonIO` entry to
`/boot/extlinux/extlinux.conf`. Make that entry the `DEFAULT`.

> **Do not** recreate the `JetsonIO-CAN-TDCR-*` boot entries or `/boot/dtb_can_tdcr/` from the
> reference host. That approach was investigated and **did not work** — all eight generated DTBs are
> byte-identical, so the boot-time TDCR value never differed and never took effect. The TDCR is set at
> runtime via sysfs instead (3c), which `scripts/activate_native_can.sh` does for you. Same verdict for
> the `devmem`/`0xC310048` register approach. See `../assets/omnihand/omnihand_canfd_setup.md`.

### 3b. Verify after reboot

```bash
lsmod | grep -E 'mttcan|can_dev'      # expect mttcan + can_dev
ip -br link show type can              # expect can0 and can1
```

If the modules are missing: `sudo modprobe can can_raw mttcan`. To make that permanent,
`echo -e "can\ncan_raw\nmttcan" | sudo tee /etc/modules-load.d/can.conf`. On the reference host the
overlay loads them automatically and no such file is needed.

### 3c. Verify the TDCR sysfs attribute exists

CAN FD with BRS at 5 Mbit through the TJA1051T/3 transceiver requires the Transmitter Delay
Compensation offset, written **while the interface is down**:

```bash
find /sys/devices/platform/bus@0 -name tdc_offset
# expect:
#   /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset
#   /sys/devices/platform/bus@0/c320000.mttcan/net/can1/tdc_offset
```

If these paths are absent after an L4T update, stop and re-evaluate — CAN FD to the OmniHand will not
work and no repo-side workaround exists. The validated value is `0x800` for the **Adafruit CAN Pal
(TJA1051T/3)**; `scripts/activate_native_can.sh` applies it automatically and accepts
`TDCR_VALUE=0x...` for a different transceiver.

### 3d. Wiring

Same transceiver carries both devices per side. `can0` → `can_nero_right` (right arm classic 1M +
right OmniHand FD/BRS 5M), `can1` → `can_nero_left`. Details and the bus-sharing caveats:
`../assets/omnihand/omnihand_canfd_setup.md`.

## Step 4 — SSH access for the private submodule

`vendor/OmniHand-Pro-2025` is fetched over SSH. Before cloning, put a key with read access to
`robyngehler/OmniHand-Pro-2025` on the target and confirm:

```bash
ssh -T git@github.com     # expect "Hi <user>! You've successfully authenticated"
```

Alternative if you prefer HTTPS on the target:

```bash
git config --global url."https://github.com/".insteadOf git@github.com:
```

## Step 5 — Clone the workspace

The reference layout is a flat `~/workspace` (not a nested colcon `src/`), and several docs use
`~/workspace/agx_arm_ros` paths. Keep it identical:

```bash
mkdir -p ~/workspace && cd ~/workspace
git clone --recurse-submodules git@github.com:<org>/agx_arm_ros.git
cd agx_arm_ros
git checkout ROS2_Duo_System_V02
git submodule update --init --recursive
git submodule status
```

Expected pins (verify — the gitlinks are deliberate, do not `git submodule update --remote`):

```text
 4f52610…  vendor/pyAgxArm              (control-layer-pin-2026-07-24)
 06dbbff…  vendor/OmniHand-Pro-2025     (heads/jetson-orin-socketcan)
```

`.gitignore` excludes `build/`, `install/`, `log/`, `__pycache__/`, `*.zip` and
`.claude/settings.local.json`. All of it is regenerated or irrelevant — see the "Audit findings" note
on assets.

## Step 6 — Repo dependency script

```bash
cd ~/workspace/agx_arm_ros
bash ./scripts/agx_arm_install_deps.sh
```

This installs the CAN tooling, `ros2-control`/`controllers`, MoveIt 2, `xacro`,
`joint-state-publisher-gui`, `robot-state-publisher`, `topic-tools` and
`python3-colcon-common-extensions`. It **requires `ROS_DISTRO` to be set** — source ROS first.

It will warn that `ros-humble-trac-ik-kinematics-plugin` is unavailable. That is expected; Step 8
handles it.

Two packages the script does not cover but the stack uses — install them explicitly:

```bash
sudo apt install -y ros-humble-pinocchio ros-humble-eigenpy
```

`agx_arm_mit_controller` imports `pinocchio` for the gravity model
(`src/agx_arm_mit_controller/agx_arm_mit_controller/gravity_model.py`). On the reference host this
resolves to apt `ros-humble-pinocchio` 4.0.0 under `/usr/bin/python3.10` — **not** to a conda copy.

## Step 7 — Python (pip) layer

**Step 6 already did this** (its step 4/4 installs `requirements.txt` into the system `python3.10`
user site and then asserts `python-can >= 4.0`). This section documents the layer for reference and
for manual recovery.

```bash
/usr/bin/python3.10 -m pip install --user --upgrade -r requirements.txt
```

Reference-host versions, for a byte-comparable target: `python-can 4.6.1`, `build 1.5.0`,
`setuptools 82.0.1`, `wheel 0.47.0`, `pybind11 3.0.4`.

> **Why `python-can` cannot come from apt:** the Jammy `python3-can` candidate is **3.3.2**, which
> predates `can.CanOperationError` and `can.CanInitializationError`. Those are used unconditionally in
> the arm's CAN error-recovery path (`vendor/pyAgxArm/.../comms/can_comm.py:129,178`), so a 3.x install
> builds fine and fails at runtime. `agx_arm_install_deps.sh` therefore no longer installs apt
> `python3-can` and fails loudly if a 3.x copy shadows the pip one.

Verify the interpreter actually sees version 4:

```bash
/usr/bin/python3.10 -c "import can; print(can.__version__)"   # must be >= 4.0
```

## Step 8 — TRAC-IK overlay workspace

`agx_arm_moveit` depends on `trac_ik_kinematics_plugin`, which has no Humble/arm64 apt package. It is
built from source in its **own** workspace, outside `agx_arm_ros`, so it stays out of this repo's
colcon graph.

```bash
cd ~/workspace/agx_arm_ros
bash ./scripts/setup_trac_ik_overlay.sh
```

The script is idempotent and does the four things this build needs:

1. imports the pinned sources from `config/trac_ik_overlay.repos` (2.0.2 / `d8d54ab`),
2. drops `COLCON_IGNORE` into `trac_ik_examples` and `trac_ik_python` (unused, and they do not build
   cleanly here),
3. applies `scripts/patches/trac_ik_moveit_humble_headers.patch` — TRAC-IK 2.0.2 includes
   `<moveit/.../*.hpp>`, which only exists post-Humble; Humble ships the same headers as `*.h`,
4. builds conda-free with `-DPython3_EXECUTABLE=/usr/bin/python3`.

Default location is `~/workspace/trac_ik_ws`; override with `TRAC_IK_WS=...`. Expected result:
`install/trac_ik`, `install/trac_ik_kinematics_plugin`, `install/trac_ik_lib`.

> The header patch previously existed **only as uncommitted local state on the reference host**. It is
> now source-managed under `scripts/patches/`, which is what makes this step reproducible at all.

## Step 9 — OmniHand vendor SDK

`build/` is gitignored inside the submodule, so the compiled SDK must be produced on the target. The
bridge auto-discovers it at `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`
(`src/agx_arm_ctrl/agx_arm_ctrl/omnihand/sdk_o12_pro.py:47`), so build it in place and do not relocate
it.

```bash
cd ~/workspace/agx_arm_ros
bash ./scripts/setup_omnihand_sdk.sh
```

The script strips conda from the environment, pins the interpreter, checks that `build`/`setuptools`/
`wheel` are importable by it, runs the vendor `build.sh`, and then verifies that the produced
extension module actually imports. Re-check an existing host with `--verify`.

The vendor build configures `OMNIHAND_PRO_CAN_BACKEND=SOCKETCAN` (the Jetson native path; the `ZLG`
USB backend is x86-only and unused) and `BUILD_PYTHON_BINDING=ON`. The vendor ROS node under `node/`
stays off — `agx_arm_ctrl` owns the hardware session.

> **Why the wrapper exists.** The vendor CMake does
> `find_package(Python3 COMPONENTS Interpreter Development REQUIRED)`. With Miniforge on `PATH` it
> picks conda's Python 3.13 and emits `agibot_hand_core.cpython-313-*.so`, which the ROS runtime
> (`python3.10`) cannot import — and that failure only surfaces later, at bridge startup. The
> reference host has `agibot_hand_core.cpython-310-aarch64-linux-gnu.so`.

If system `pybind11` is missing, CMake falls back to `FetchContent` and downloads it — so either keep
Step 7's `pybind11` installed or allow network access during this build.

Manual verification, if you want it independently of the script:

```bash
ls vendor/OmniHand-Pro-2025/build/agibot_hand_pkg/agibot_hand/   # expect cpython-310 .so + libomniHandPro25Can.so
PYTHONPATH=vendor/OmniHand-Pro-2025/build/agibot_hand_pkg \
  /usr/bin/python3.10 -c "import agibot_hand; print('ok')"
```

The `.so` has `RUNPATH=$ORIGIN`, so no `LD_LIBRARY_PATH` is needed for the import.

## Step 10 — Build the ROS workspace

```bash
cd ~/workspace/agx_arm_ros
export AGX_ARM_TRAC_IK_OVERLAY=~/workspace/trac_ik_ws/install/setup.bash
bash ./scripts/colcon_build_system_python.sh
```

The wrapper does the environment discipline for you: strips conda and `~/.local/bin` from `PATH`, sets
`PYTHONNOUSERSITE=1`, asserts `/usr/bin/python3`, sources ROS plus the TRAC-IK overlay, drops stale
`install/` prefixes, and **skips `omni_hand_pro_2025`** so the vendor SDK is not dragged into the
colcon graph.

Expected packages in `install/`: `agx_arm_coordination`, `agx_arm_ctrl`, `agx_arm_description`,
`agx_arm_mit_controller`, `agx_arm_mit_demos`, `agx_arm_mit_tools`, `agx_arm_moveit`, `agx_arm_msgs`,
`duo_body_description`, `nero_gripper_moveit_config`, `realsense2_description`, **and `pyAgxArm`**.

`pyAgxArm` appears because `vendor/pyAgxArm` carries a `setup.py` and is discovered as a workspace
Python package — that is how the control layer is installed on the reference host
(`install/pyAgxArm/lib/python3.10/site-packages`). No separate pip install is needed. The
`--user -e vendor/pyAgxArm` editable route described in `control_layer_and_dependencies.md` is the
older mechanism; a stale `pyAgxArm.egg-link` from it still exists on the reference host but the
colcon-built copy wins via `PYTHONPATH`. **On a fresh target, do not create the editable install** —
just build the workspace.

## Step 11 — Shell environment

Append to `~/.bashrc`, in this order (this is the reference host's working configuration):

```bash
export LC_NUMERIC=en_US.UTF-8
source /opt/ros/humble/setup.bash
source ~/workspace/agx_arm_ros/install/setup.bash
source ~/workspace/trac_ik_ws/install/setup.bash
export AGX_ARM_TRAC_IK_OVERLAY=~/workspace/trac_ik_ws/install/setup.bash
```

Sourcing `trac_ik_ws` last puts it first in `AMENT_PREFIX_PATH`, which is what the reference host runs
and what `moveit_profile_smoke_test.sh` was validated against. The two overlays hold disjoint packages,
so the order is not load-bearing — but keep it identical to avoid re-validating.

If Miniforge is installed on the target, its `conda init` block must come **after** the ROS lines and
you must not `conda activate` in a shell that has `install/setup.bash` sourced. Use
`scripts/run_in_ros_conda.sh` instead. Simplest path for a pure runtime host: **do not install conda
at all** (see Step 12).

## Step 12 — Conda runtime environment (optional, currently unused)

```bash
# only if you actually need it
bash ./scripts/setup_agx_arm_runtime_env.sh
bash ./scripts/run_in_ros_conda.sh -- <command>
```

`config/agx_arm_runtime.conda.yaml` defines `agx-arm-runtime` (Python 3.10, numpy, scipy, pyyaml,
pytest, python-can, pinocchio) and the script additionally pip-installs `vendor/pyAgxArm` into it.

**This environment does not exist on the reference host** — `conda env list` shows only `base`. Every
dependency it provides is already satisfied by apt under `/usr/bin/python3.10`, `pinocchio` included.
Treat it as an optional development convenience; the production runtime path is system Python. Skip
this step for a clean migration.

---

## Verification

### A. Without hardware

```bash
cd ~/workspace/agx_arm_ros

# interpreter and dependency sanity
/usr/bin/python3.10 -c "import can, numpy, scipy, yaml, pinocchio, pyAgxArm; print('deps ok')"
/usr/bin/python3.10 -c "import pyAgxArm, os; print(os.path.dirname(pyAgxArm.__file__))"

# TRAC-IK is resolvable
ros2 pkg prefix trac_ik_kinematics_plugin

# vendor SDK imports under the ROS interpreter
PYTHONPATH=vendor/OmniHand-Pro-2025/build/agibot_hand_pkg \
  /usr/bin/python3.10 -c "import agibot_hand; print('sdk ok')"

# package tests, from a system-Python ROS shell (never conda)
colcon test --packages-select agx_arm_ctrl agx_arm_coordination agx_arm_mit_controller
colcon test-result --verbose

# MoveIt profiles load without plugin errors
bash ./scripts/moveit_profile_smoke_test.sh
```

`moveit_profile_smoke_test.sh` reaching `You can start planning now!` for the `none`, `agx_gripper`,
`revo2` and `omnihand` profiles is the strongest no-hardware signal that Steps 8–10 are correct.

### B. With hardware — ask before running any of this

Hardware bringup touches live arms and hands. Per `AGENTS.md`, confirm hardware access is granted for
the session first. `sudo` is available without a password in the intended hardware environment.

```bash
# 1. CAN bringup (both side buses, CAN FD + TDCR)
sudo bash ./scripts/activate_native_can.sh
ip -details link show can_nero_right | grep 'mtu 72'    # 72 = CAN FD MTU

# 2. Below-ROS hand probe
cd vendor/OmniHand-Pro-2025
PYTHONPATH=$PWD/build/agibot_hand_pkg \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
  /usr/bin/python3.10 python/example/demo_get_hardware_info.py

# 3. Full bringup
cd ~/workspace/agx_arm_ros
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
    mode:=moveit_mit execution_profile:=right_hand
```

Then follow `../control/bringups/launches.md` and `../control/bringups/teach_and_run.md`.

**Two known hardware-side traps when standing up a second machine** — read
`../sprint6/errors_and_fixes.md` before concluding the migration failed:

- **Arm firmware is not uniform.** The reference host's two arms run different firmware (1.06 and
  1.11). Check the target's arms; behavioral differences between the two machines may be firmware, not
  software.
- **A persisted push-disabled config deadlocks startup** and leaves the arm limp. If the new host's
  arm never enables, check the push configuration before suspecting this guide.

---

## What the reproducibility fixes changed

Applied 2026-07-30, in response to the audit above. Listed so a reader of the old setup docs knows what
moved:

| Change | File |
|---|---|
| Pinned pip manifest added; installed by the deps script and version-asserted | `requirements.txt` |
| Dropped apt `python3-can` (3.3.2, too old); added step 4/4 that installs `requirements.txt` and fails loudly on python-can < 4; added `ros-humble-pinocchio` / `-eigenpy`; TRAC-IK hint now names the setup script | `scripts/agx_arm_install_deps.sh` |
| TRAC-IK sources pinned (2.0.2 / `d8d54ab`) | `config/trac_ik_overlay.repos` |
| The MoveIt Humble header fix, previously uncommitted local state | `scripts/patches/trac_ik_moveit_humble_headers.patch` |
| Idempotent TRAC-IK overlay provisioning: import, ignore-markers, patch, conda-free build | `scripts/setup_trac_ik_overlay.sh` |
| OmniHand SDK build with a pinned interpreter, prerequisite check, and import verification (`--verify`) | `scripts/setup_omnihand_sdk.sh` |
| Dependency-layer table; conda demoted from "golden rules" to explicitly optional | `../control/environment.md` |
| Absolute `source_log` made repo-relative | `config/nero_gravity_calibration.json` |
| Fit tool now writes cwd-relative provenance so regeneration stops baking in a home directory | `src/agx_arm_mit_tools/.../fit_gravity_calibration.py` |
| Hardcoded `~/workspace/agx_arm_ros` in the usage docstring replaced with a repo-root lookup | `scripts/omnihand/omnihand_load_test.py` |
| Marked operationally superseded, pointing at the automated path | `../sprint3/evidence/trac_ik_humble_jetson_repro.md` |

Deliberately left alone:

- **`config/agx_arm_runtime.conda.yaml` and its two wrapper scripts** are kept, not deleted. They are
  documented as optional rather than removed, because `scripts/run_in_ros_conda.sh` is still the
  correct way to run something that genuinely needs a Conda-managed library.
- **The `nero_meshes.zip` `.gitignore` entry** stays; the archive is redundant with tracked `.stl`
  files.

## Related

- `../control/environment.md` — operating an already-provisioned host: wrappers, overlays, platform split
- `../control/bringups/launches.md` — launch matrix once the stack runs
- `../assets/omnihand/omnihand_canfd_setup.md` — CAN FD timing, TDCR, dead-end approaches
- `control_layer_and_dependencies.md` — pyAgxArm pin history and submodule workflow
- `repository_structure.md` — package ownership and staging boundaries
