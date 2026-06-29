# Proposal: Migrate OmniHand Integration from OmniHand 2025 SDK (O10) to OmniHand Pro 2025 SDK (O12)

**Date:** 2026-06-24  
**Target repository:** `AgibotTech/OmniHand-Pro-2025` → fork under `robyngehler/OmniHand-Pro-2025`  
**Current repository in stack:** `robyngehler/OmniHand-Pro-2025`  
**Target hardware:** AGIBOT OmniHand Pro 2025 on Jetson AGX Orin, ROS 2 Humble, native SocketCAN/CAN-FD side buses

---

## 1. Executive Summary

The current ROS bridge and helper tooling were built around the **OmniHand 2025 / O10 SDK**. That implementation is now functionally useful, but it targets the wrong hardware generation for our system. We actually own and must integrate the **OmniHand Pro 2025 / O12**.

This is not a small rename. The target SDK changes the Python package, main SDK class, joint/motor count, tactile payload, CMake build assumptions, and likely the supported low-level CAN path. The current stack assumes:

- Python import package: `agibot_hand`
- SDK class: `AgibotHandO12`
- active command vector: 10 values
- old joint order:
  `[thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip, middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip]`
- current bridge path: `set_all_active_joint_angles(...)`
- current feedback topics:
  - `feedback/omnihand/joint_states`
  - `feedback/omnihand/status`
  - `feedback/omnihand/tactile_raw`
- current command topics:
  - `control/joint_states`
  - `control/omnihand/joint_trajectory`
  - `control/omnihand/stop`

The target Pro SDK exposes:

- Python import package: `agibot_hand`
- Python distribution name: `omnihand_pro_2025_py`
- SDK class: `AgibotHandO12`
- hand type enum: `EHandType`
- control mode enum: `EControlMode`
- documented motor indices: 1..12
- target active angle examples using 12-element vectors
- tactile structure with per-finger normal force, tangent force, tangent force angle, channel values, and capacitive approach values
- C++/CMake build which currently defaults to the x86 ZLG USB-CANFD SDK path

The safest migration is therefore:

1. **Fork and vendor the correct Pro SDK.**
2. **Patch the Pro SDK for Jetson/aarch64 and native SocketCAN.**
3. **Introduce a model-specific SDK adapter layer in `agx_arm_ctrl`.**
4. **Keep the existing ROS surface stable where possible.**
5. **Replace all O10-specific joint limits, gestures, conversion polynomials, and tests with O12-Pro equivalents.**
6. **Validate with read-only, then low-rate controlled sweeps, then skill-level tactile grasps.**

Tiny detail, but one that likes to ruin afternoons: do **not** let two processes open the same hand/CAN session. The Pro SDK includes its own ROS node directory, but our MVP should keep our custom ROS bridge as the single owner of the hardware session and use the vendor node only as a reference.

---

## 2. Confirmed Findings

### 2.1 Current stack is O10-specific

The current bridge hardcodes the O10 shape in several places:

- 10 active joint suffixes.
- O10-specific right/left joint limits.
- O10 actuator-to-active-angle conversion polynomials.
- import package `agibot_hand`.
- required SDK symbols `AgibotHandO12`, `EFinger`, `EHandType`.
- built vendor path: `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`.
- O10-specific workaround for padded 12-value readback vectors trimmed to 10 active channels.
- `OMNIHAND_SOCKETCAN_IFACE` environment variable is already used in our patched bridge to select `can_nero_right` / `can_nero_left`.

This work should be preserved conceptually, not copy-pasted blindly. The O10 compatibility logic is valuable as migration history, but some of it becomes actively dangerous for O12 Pro, especially old limits, old vector trimming, and old motor-to-angle conversion.

### 2.2 Current skill abstraction should survive

The skill controller design is already correct at the architecture level:

- public task/action layer carries `skill_name`, not vendor gesture IDs;
- vendor mapping stays in the backend;
- tactile-confirmed close is the core grasp mechanism;
- `completion_policy` and `fallback_policy` describe behavior, not vendor commands;
- hold is internal after a successful grasp;
- release remains a normal action.

This should remain unchanged. Only the backend mapping and tactile interpretation need to become O12-Pro-aware.

### 2.3 Current gesture mapping is O10-only

The existing gesture file documents 18 vendor O10 gestures as 10-element vectors. Those values must not be reused for Pro except as conceptual labels. The Pro example `demo_set_angle.py` uses 12-element vectors, for example a reset vector and a FIST vector. That should become the first bootstrap for `omnihand_pro_gestures.yaml`, followed by hardware calibration.

### 2.4 Pro SDK surface differs materially

The target repository README states that OmniHand Pro 2025 is a 12-DOF professional dexterous hand with tactile sensors and multiple control modes. Its Python docs and `.pyi` expose:

- `AgibotHandO12`
- `EFinger`
- `EControlMode`
- `EHandType`
- `set_all_joint_positions(List[int])`
- `get_all_joint_positions()`
- `set_all_active_joint_angles(List[float])`
- `get_all_active_joint_angles()`
- `get_all_joint_angles()`
- `get_tactile_sensor_data(EFinger)`
- `get_all_error_reports()`
- temperature/current reporting APIs
- current threshold APIs
- mixed control via `MixCtrl`

The Pro tactile object is richer than the current flat float stream:

```text
online_state
normal_force
tangent_force
tangent_force_angle
channel_values[9]
capacitive_approach[4]
```

### 2.5 Pro SDK Jetson risk: x86 defaults and hardcoded CAN backend

The target repo is not Jetson-ready out of the box.

Observed upstream patterns to fix:

1. README lists Ubuntu 22.04 `x86_64`.
2. The Python wheel example is `linux_x86_64`.
3. `src/CMakeLists.txt` currently hardcodes:
   ```cmake
   set(CanfdDevice 1)
   ```
   where `1` means ZLG USB-CANFD SDK and `2` means SocketCAN.
4. The same CMake file points to:
   ```text
   thirdParty/usbcanfd_libusb_x64_1.0.10_250328
   ```
5. `python/CMakeLists.txt` copies `libusbcanfd.so.1.0.10` from that x64 third-party directory into the Python package.
6. The SocketCAN source currently hardcodes `can0` via:
   ```cpp
   strcpy(ifr.ifr_name, "can0");
   ```

Therefore, for Jetson native CAN-FD, the target fork must expose a build option for SocketCAN and avoid linking/copying x86 USB-CANFD libraries.

---

## 3. Proposed Repository and Git Migration

### 3.1 Fork target repo

Fork:

```text
https://github.com/AgibotTech/OmniHand-Pro-2025
```

to:

```text
https://github.com/robyngehler/OmniHand-Pro-2025
```

Then initialize a Jetson integration branch:

```bash
git clone git@github.com:robyngehler/OmniHand-Pro-2025.git
cd OmniHand-Pro-2025

git remote add upstream https://github.com/AgibotTech/OmniHand-Pro-2025.git
git fetch upstream --tags

git switch -c jetson-orin-socketcan upstream/main
```

Recommended branch naming:

```text
jetson-orin-socketcan
```

Reason: this is not only an AGX build branch; it changes the CAN backend behavior and packaging.

### 3.2 Replace vendor SDK in the main stack

First discover how the current vendor repo is integrated:

```bash
cd <agx_arm_ctrl_repo>

git submodule status || true
git ls-files vendor | sed -n '1,120p'
git remote -v
git grep -n "OmniHand-Pro-2025\|agibot_hand\|AgibotHandO12"
```

#### If current SDK is a Git submodule

```bash
git submodule deinit -f vendor/OmniHand-Pro-2025
git rm -f vendor/OmniHand-Pro-2025

git submodule add -b jetson-orin-socketcan \
  git@github.com:robyngehler/OmniHand-Pro-2025.git \
  vendor/OmniHand-Pro-2025

git submodule update --init --recursive
```

#### If current SDK is vendored as normal files

```bash
git rm -r vendor/OmniHand-Pro-2025

git clone git@github.com:robyngehler/OmniHand-Pro-2025.git \
  vendor/OmniHand-Pro-2025

cd vendor/OmniHand-Pro-2025
git switch jetson-orin-socketcan
cd ../..

git add vendor/OmniHand-Pro-2025
```

### 3.3 Keep a migration tag before removal

Before replacing anything, tag the last working O10 state:

```bash
git tag omnihand-o10-bridge-working-2026-06-24
git push origin omnihand-o10-bridge-working-2026-06-24
```

This preserves the known-good bring-up state. Yes, it is boring. Boring is what we call “debuggable” when it saves us two days.

---

## 4. Required Fork Patches in `OmniHand-Pro-2025`

### 4.1 Add a CMake option for CAN backend

Patch `src/CMakeLists.txt`.

Current upstream behavior:

```cmake
set(CanfdDevice 1)
```

Proposed replacement:

```cmake
set(OMNIHAND_PRO_CAN_BACKEND "SOCKETCAN" CACHE STRING "CAN backend: SOCKETCAN or ZLG")
set_property(CACHE OMNIHAND_PRO_CAN_BACKEND PROPERTY STRINGS SOCKETCAN ZLG)

if(OMNIHAND_PRO_CAN_BACKEND STREQUAL "ZLG")
  set(CanfdDevice 1)
elseif(OMNIHAND_PRO_CAN_BACKEND STREQUAL "SOCKETCAN")
  set(CanfdDevice 2)
else()
  message(FATAL_ERROR "Unsupported OMNIHAND_PRO_CAN_BACKEND=${OMNIHAND_PRO_CAN_BACKEND}")
endif()
```

Then guard all ZLG-specific include/link/install statements:

```cmake
if(CanfdDevice EQUAL 1)
  # include/link/copy x64 ZLG library only here
endif()
```

Do **not** install or copy x64 `libusbcanfd.so` when building SocketCAN.

### 4.2 Patch Python packaging for SocketCAN

Current `python/CMakeLists.txt` copies the x64 ZLG library into the Python package unconditionally.

Patch it so this only happens for `CanfdDevice == 1`:

```cmake
if(CanfdDevice EQUAL 1)
  add_custom_command(
    TARGET ${CUR_TARGET_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10
            ${PYTHON_PKG_DIR}/agibot_hand/
    # symlink commands...
  )
endif()
```

For SocketCAN, the package should contain:

```text
agibot_hand/
  __init__.py
  __init__.pyi
  agibot_hand_core*.so
  libomniHandPro25Can.so or equivalent linked runtime dependency
```

Then verify:

```bash
cd vendor/OmniHand-Pro-2025

file build/agibot_hand_pkg/agibot_hand/*.so
ldd  build/agibot_hand_pkg/agibot_hand/agibot_hand_core*.so
readelf -d build/agibot_hand_pkg/agibot_hand/agibot_hand_core*.so | grep -E "RPATH|RUNPATH" || true
```

Acceptance: no `x86-64` object is pulled into an `aarch64` runtime.

### 4.3 Patch SocketCAN interface selection

Current upstream SocketCAN source hardcodes `can0`. Patch it to follow our existing bridge convention:

```cpp
#include <cstdlib>
#include <cstring>

const char* env_iface = std::getenv("OMNIHAND_SOCKETCAN_IFACE");
const char* iface = (env_iface && env_iface[0] != '\0') ? env_iface : "can0";

std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
ifr.ifr_name[IFNAMSIZ - 1] = '\0';
```

This preserves compatibility with default `can0` but lets our ROS launch select:

```bash
OMNIHAND_SOCKETCAN_IFACE=can_nero_right
```

or

```bash
OMNIHAND_SOCKETCAN_IFACE=can_nero_left
```

### 4.4 Build command for Jetson

Recommended first attempt:

```bash
cd vendor/OmniHand-Pro-2025

python3 -m pip install --upgrade build setuptools wheel pybind11

./build.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/install" \
  -DBUILD_PYTHON_BINDING=ON \
  -DBUILD_CPP_EXAMPLES=OFF \
  -DOMNIHAND_PRO_CAN_BACKEND=SOCKETCAN
```

If CMake version fails, check actual version:

```bash
cmake --version
```

The repo top-level `CMakeLists.txt` currently declares `cmake_minimum_required(VERSION 3.16)`, while the README says CMake 3.24 or higher. If Ubuntu 22.04's default CMake works, do not upgrade just because the README says so. If it fails due a specific CMake feature, install a newer CMake via Kitware or a controlled package source.

---

## 5. Required Changes in `agx_arm_ctrl`

### 5.1 Introduce a model-aware SDK adapter layer

Do not mutate `SdkOmniHandBackend` into a giant `if O10 else O12` monster. That beast will bite.

Proposed structure:

```text
agx_arm_ctrl/
  omnihand/
    __init__.py
    models.py
    sdk_adapter_base.py
    sdk_adapter_o10.py
    sdk_adapter_o12_pro.py
    joint_models.py
    tactile_models.py
```

Minimal interface:

```python
class HandSdkAdapter(Protocol):
    backend_name: str
    model_name: str
    active_joint_names: list[str]

    def apply_active_joint_targets(self, target_map: dict[str, float], control_mode: str) -> int: ...
    def apply_trajectory(self, msg: JointTrajectory) -> None: ...
    def stop_hold_current_pose(self) -> None: ...
    def read_joint_state(self) -> list[float]: ...
    def read_status(self) -> OmniHandStatusSnapshot: ...
    def read_tactile(self) -> OmniHandTactileSnapshot: ...
```

Then the ROS node only talks to this adapter. The adapter owns all vendor weirdness. A small bureaucratic border, yes, but borders are useful when SDKs change shape like startled octopuses.

### 5.2 Add parameters

Add/rename bridge parameters:

```yaml
hand_model: "o12_pro"        # o10 | o12_pro
backend_type: "sdk"          # mock | sdk
omnihand_type: "right"       # right | left
sdk_python_dir: ""           # optional explicit package dir
can_interface: ""            # optional; fallback from config
control_mode: "position"     # position | velocity | torque | position_torque
```

Recommended launch naming:

```bash
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
  hand_model:=o12_pro \
  backend_type:=sdk \
  omnihand_type:=right
```

### 5.3 Update SDK import resolution

Current O10 logic searches for:

```text
vendor/OmniHand-Pro-2025/build/agibot_hand_pkg
```

Target O12-Pro logic should search for:

```text
vendor/OmniHand-Pro-2025/build/agibot_hand_pkg
```

and import:

```python
from agibot_hand import AgibotHandO12, EFinger, EHandType, EControlMode
```

Proposed discovery logic:

```python
_VENDOR_O12_PRO_PKG_REL = Path("vendor") / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"

def _ensure_o12_pro_importable(sdk_python_dir: str = "") -> None:
    candidates = [
        sdk_python_dir,
        os.environ.get("AGX_ARM_OMNIHAND_PRO_SDK_DIR", ""),
        _locate_builtin_o12_pro_pkg() or "",
    ]
    # try import agibot_hand
```

Do not reuse the `agibot_hand` package name.

### 5.4 Replace active joint order and limits

Use the Pro Python API docs as the first candidate order, but verify against the vendor assets and live hardware.

Candidate O12-Pro joint order from current docs/example:

```python
O12_PRO_JOINT_SUFFIXES = [
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "thumb_pip_joint",
    "index_abad_joint",
    "index_mcp_joint",
    "index_pip_joint",
    "middle_abad_joint",
    "middle_mcp_joint",
    "middle_pip_joint",
    "ring_mcp_joint",
    "pinky_mcp_joint",
]
```

Important local validation:

```bash
cd <agx_arm_ctrl_repo>
git grep -n "thumb_pip_joint\|index_mcp_joint\|middle_abad_joint\|ring_mcp_joint\|pinky_mcp_joint" \
  src config launch urdf xacro
```

If the robot description uses vendor `R_...`/`L_...` names, add a mapping layer rather than renaming public ROS interfaces mid-flight:

```yaml
omnihand_pro_joint_order:
  ros:
    - right_thumb_roll_joint
    - right_thumb_abad_joint
    - right_thumb_mcp_joint
    - right_thumb_pip_joint
    - right_index_abad_joint
    - right_index_mcp_joint
    - right_index_pip_joint
    - right_middle_abad_joint
    - right_middle_mcp_joint
    - right_middle_pip_joint
    - right_ring_mcp_joint
    - right_pinky_mcp_joint
  vendor_index_1_based:
    - 1
    - 2
    - 3
    - 4
    - 5
    - 6
    - 7
    - 8
    - 9
    - 10
    - 11
    - 12
```

### 5.5 Remove O10 motor conversion for O12

The O10 adapter currently contains motor-to-angle conversion via actuator min/max and polynomials. Do **not** reuse that for Pro.

For O12-Pro:

1. Prefer direct angle readback:
   ```python
   hand.get_all_active_joint_angles()
   ```
2. Validate length is 12.
3. Clamp using O12-Pro limits.
4. Only use `get_all_joint_positions()` for raw motor diagnostics, not as the main ROS joint state source unless angle readback is proven unreliable.
5. If angle readback is broken on hardware, create a **new** O12 calibration conversion document. Do not borrow O10 polynomials.

### 5.6 Tactile bridge update

The existing `OmniHandTactileRaw` can remain for MVP if it is just a flat vector. But the flattening must be explicit and documented.

Recommended MVP flattening per finger:

```text
finger order: thumb, index, middle, ring, little

per finger:
  online_state
  normal_force
  tangent_force
  tangent_force_angle
  channel_values[0..8]
  capacitive_approach[0..3]
```

That gives:

```text
5 fingers × (1 + 1 + 1 + 1 + 9 + 4) = 85 scalar values
```

Recommended layout name:

```text
o12_pro:v1:thumb,index,middle,ring,little:online,normal,tangent,tangent_angle,channels9,capacitive4
```

Near-future cleaner message:

```text
OmniHandTactileFinger.msg
  string finger
  bool online
  float32 normal_force_n
  float32 tangent_force_n
  float32 tangent_force_angle_deg
  float32[] channel_values
  float32[] capacitive_approach

OmniHandTactileArray.msg
  std_msgs/Header header
  string hand_side
  string backend_name
  OmniHandTactileFinger[] fingers
```

For the skill controller, use `normal_force` first for `contact_score`. Add shear/tangent force later for slip detection.

### 5.7 Status bridge update

For O12-Pro, status arrays must become length 12:

```text
active_joint_temperatures_c: 12
active_joint_currents_a: 12
active_joint_stalled: 12
active_joint_over_temperature: 12
active_joint_over_current: 12
```

Also map the additional O12 error fields if useful:

```text
motor_except
commu_except
```

If the existing `OmniHandStatus` message cannot carry those fields, include them in `status_text` for MVP and schedule a message update later.

---

## 6. FollowJointTrajectory Bridge Changes

The current `omnihand_follow_joint_trajectory.py` duplicates `JOINT_SUFFIXES` locally. That must stop.

Required changes:

1. Import joint model from one shared source:
   ```python
   from agx_arm_ctrl.omnihand.joint_models import build_joint_names
   ```
2. Add parameter:
   ```python
   hand_model: "o12_pro"
   ```
3. Validate against model-specific joint names.
4. Keep the action name stable only if MoveIt config expects it:
   ```text
   right_omnihand_controller/follow_joint_trajectory
   ```
5. On cancel, call the bridge stop service or publish a hold command. The current action accepts cancel but does not actively stop motion; for O12-Pro with stronger fingers, passive cancel is too polite.

MVP behavior can still be final-point command only, because the bridge currently publishes a `JointTrajectory` and the vendor command path applies the final point. Later, if we need smoother hand motion, the bridge can interpolate internally at 20–50 Hz.

---

## 7. Exerciser, Control Scripts, Load Tests

### 7.1 Replace O10 exerciser presets

Create:

```text
config/omnihand_pro_gestures.yaml
```

Bootstrap with safe Pro vectors:

```yaml
omnihand_model: o12_pro
omnihand_active_joint_order:
  - thumb_roll_joint
  - thumb_abad_joint
  - thumb_mcp_joint
  - thumb_pip_joint
  - index_abad_joint
  - index_mcp_joint
  - index_pip_joint
  - middle_abad_joint
  - middle_mcp_joint
  - middle_pip_joint
  - ring_mcp_joint
  - pinky_mcp_joint

omnihand_gestures:
  zero: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  fist_vendor_demo: [0.5, -0.2, 0.0, -1.2, 0.0, 1.35, 1.53, 0.0, 1.36, 1.82, 1.55, 1.54]
```

Do **not** call this `grasp_glass` yet. The Pro fist vector is only a vendor demo. Calibrate real grasp poses on the glass/bottle task.

### 7.2 New scripts

Recommended scripts:

```text
scripts/omnihand_pro/
  pro_import_check.py
  pro_hardware_probe.py
  pro_readonly_load_test.py
  pro_tactile_probe.py
  pro_safe_joint_sweep.py
  pro_ros_bridge_smoke_test.md
```

#### `pro_import_check.py`

Purpose: verify import and architecture before touching hardware.

```python
from agibot_hand import AgibotHandO12, EFinger, EHandType, EControlMode
print("import ok")
print(AgibotHandO12, EFinger, EHandType, EControlMode)
```

#### `pro_hardware_probe.py`

Read-only hardware facts:

```python
from agibot_hand import AgibotHandO12, EHandType

h = AgibotHandO12(hand_type=EHandType.RIGHT)
h.show_data_details(False)

print("vendor:", h.get_vendor_info())
print("device:", h.get_device_info())
print("joint_positions:", len(h.get_all_joint_positions()), h.get_all_joint_positions())
print("active_angles:", len(h.get_all_active_joint_angles()), h.get_all_active_joint_angles())
print("all_angles:", len(h.get_all_joint_angles()), h.get_all_joint_angles())
print("errors:", len(h.get_all_error_reports()))
```

#### `pro_tactile_probe.py`

Per-finger tactile shape discovery:

```python
from agibot_hand import AgibotHandO12, EFinger, EHandType

h = AgibotHandO12(hand_type=EHandType.RIGHT)

for finger in [EFinger.THUMB, EFinger.INDEX, EFinger.MIDDLE, EFinger.RING, EFinger.LITTLE]:
    d = h.get_tactile_sensor_data(finger)
    print(finger)
    print("online_state:", d.online_state)
    print("normal_force:", d.normal_force)
    print("tangent_force:", d.tangent_force)
    print("tangent_force_angle:", d.tangent_force_angle)
    print("channel_values:", len(d.channel_values), d.channel_values)
    print("capacitive_approach:", len(d.capacitive_approach), d.capacitive_approach)
```

#### `pro_readonly_load_test.py`

Measure:

- 50 Hz active angle readback
- 5–20 Hz tactile readback
- 1 Hz status/error/temperature/current
- CAN bus error counters before/after

Acceptance:

```text
0 SDK exceptions
0 CAN bus-off events
no steadily increasing rx/tx errors
mean active-angle read < 2 ms
mean tactile read per all fingers < 20 ms
```

The exact thresholds may be adjusted after first hardware measurements.

---

## 8. Local Discovery Checklist

Run this before implementation, because the local stack may contain launch/config/URDF dependencies not visible from the attached files.

### 8.1 Find old SDK references

```bash
cd <agx_arm_ctrl_repo>

git grep -n \
  "OmniHand-Pro-2025\|agibot_hand\|AgibotHandO12\|agibot_hand_pkg\|SDK_ACTIVE_JOINT_COUNT\|JOINT_SUFFIXES\|omnihand_gestures"
```

Every hit must be classified:

```text
delete      old O10-only code no longer needed
adapt       useful bridge/adapter pattern, but O12-specific data changes
keep        generic ROS surface or mock backend
archive     docs/history only
```

### 8.2 Find ROS launch and config assumptions

```bash
find . -type f \( -name "*.py" -o -name "*.yaml" -o -name "*.xml" -o -name "*.xacro" -o -name "*.urdf" -o -name "*.md" \) \
  -print0 | xargs -0 grep -n \
  "omnihand\|OmniHand\|right_omnihand\|left_omnihand\|control/omnihand\|feedback/omnihand"
```

Special attention:

- launch files
- MoveIt controller config
- robot description joint names
- `agx_arm_msgs` message definitions
- performer routing / skill controller
- CAN activation scripts
- systemd units if used

### 8.3 Verify Pro package output

```bash
cd vendor/OmniHand-Pro-2025

find build -maxdepth 4 -type f | sort | sed -n '1,200p'
find build -maxdepth 5 -type f -name "*agibot*so*" -o -name "*.whl"
python3 - <<'PY'
import sys, pathlib
sys.path.insert(0, "build/agibot_hand_pkg")
import agibot_hand
print(agibot_hand)
print(dir(agibot_hand))
PY
```

### 8.4 Verify no x86 library contamination

```bash
find vendor/OmniHand-Pro-2025/build -type f -name "*.so*" -exec file {} \;
```

Acceptance on Jetson:

```text
ELF 64-bit LSB shared object, ARM aarch64
```

Anything saying `x86-64` in the runtime package is a bug unless it is an unused source artifact.

### 8.5 Verify native CAN-FD state

```bash
ip -details -statistics link show can_nero_right
ip -details -statistics link show can_nero_left

# optional, only if can-utils installed
candump -tz -x can_nero_right
```

Expected for our bus:

```text
CAN FD enabled
bitrate nominal/data rates match the hand setup
no bus-off
error counters stable during read-only test
```

---

## 9. Skill Layer Migration

### 9.1 Keep public skill names

Keep existing public names:

```text
open_hand
grasp_glass_until_contact
grasp_bottle_until_contact
release_glass
release_bottle
stop_hand
```

Do **not** expose vendor gesture IDs in the activity graph.

### 9.2 Replace backend mapping

O10 mapping:

```text
open       -> old O10 PAPER/open
glass      -> old O10 FIST1
bottle     -> old O10 FIST2
```

O12-Pro mapping should initially be:

```text
open_hand                  -> calibrated_o12_open
grasp_glass_until_contact  -> calibrated_o12_glass_close_direction + tactile stop
grasp_bottle_until_contact -> calibrated_o12_bottle_close_direction + tactile stop
release_glass              -> calibrated_o12_open
release_bottle             -> calibrated_o12_open
stop_hand                  -> hold current active angles
```

The first Pro calibration must produce:

```yaml
calibrated_o12_open:
  positions: [...]
  tolerance_rad: [...]

calibrated_o12_glass_pregrasp:
  positions: [...]

calibrated_o12_glass_close_target:
  positions: [...]
  max_step_rad: [...]
  max_current_threshold: [...]

calibrated_o12_bottle_pregrasp:
  positions: [...]

calibrated_o12_bottle_close_target:
  positions: [...]
  max_step_rad: [...]
  max_current_threshold: [...]
```

### 9.3 Contact scoring for Pro

Start with:

```python
contact_score = max(normal_force_n over selected fingers)
```

Then compare with:

```python
contact_score = mean(normal_force_n over selected fingers)
```

Recommended first contact sensors:

```yaml
glass:
  fingers: [thumb, index, middle]
  metric: normal_force_max
  stable_samples: 3

bottle:
  fingers: [thumb, index, middle, ring]
  metric: normal_force_mean_or_max
  stable_samples: 3
```

These are placeholders. Actual thresholds must be measured with the Hefeweizen glass and bottle.

### 9.4 Slip monitoring

For O12-Pro, tangent force and tangent angle become useful for slip detection. MVP:

```text
warn: normal_force drops below warn threshold for N samples
critical: normal_force drops below critical threshold for N samples
```

Later:

```text
slip candidate: tangent_force spike + normal_force drop + object motion
```

Do not block the coordinator with a long-running hold action. Keep hold internal, publish status/events.

---

## 10. Proposed Implementation Plan

### Phase 0 — Baseline and freeze old working state

Deliverables:

- tag last O10 working state
- export current O10 load test results
- document current CAN settings
- record current bridge launch command

Commands:

```bash
git status
git tag omnihand-o10-bridge-working-2026-06-24
git push origin omnihand-o10-bridge-working-2026-06-24

ip -details -statistics link show can_nero_right > docs/omnihand_o10_can_right_before_migration.txt
ip -details -statistics link show can_nero_left  > docs/omnihand_o10_can_left_before_migration.txt
```

### Phase 1 — Fork and Jetson-build Pro SDK

Deliverables:

- `robyngehler/OmniHand-Pro-2025`
- branch `jetson-orin-socketcan`
- CMake backend option
- no unconditional x64 lib copy
- SocketCAN interface env var
- `pro_import_check.py` passes on Jetson
- read-only hardware probe passes

Acceptance:

```bash
python3 scripts/omnihand_pro/pro_import_check.py
python3 scripts/omnihand_pro/pro_hardware_probe.py --side right --iface can_nero_right
```

### Phase 2 — Adapter layer in ROS bridge

Deliverables:

- model-aware adapter classes
- `hand_model:=o12_pro`
- old O10 code either isolated or archived
- bridge starts in mock mode with 12 joints
- bridge starts in SDK mode with live Pro hand
- `feedback/omnihand/joint_states` publishes 12 Pro joints
- `feedback/omnihand/status` publishes 12 status entries
- `feedback/omnihand/tactile_raw` publishes flattened Pro tactile layout

Acceptance:

```bash
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
  hand_model:=o12_pro backend_type:=mock omnihand_type:=right

ros2 topic echo /feedback/omnihand/joint_states --once
```

then:

```bash
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
  hand_model:=o12_pro backend_type:=sdk omnihand_type:=right can_interface:=can_nero_right
```

### Phase 3 — Pro exerciser and safe motion

Deliverables:

- `omnihand_exerciser --model o12_pro`
- `omnihand_pro_gestures.yaml`
- reset/open command tested
- single-joint safe sweep tested
- stop service holds current pose

Acceptance:

```bash
ros2 run agx_arm_ctrl omnihand_exerciser \
  --model o12_pro --side right --gesture zero --stop
```

Then a single low-amplitude joint sweep with hand safely mounted and no object.

### Phase 4 — FollowJointTrajectory and MoveIt controller config

Deliverables:

- shared model-specific joint names
- controller validates 12 Pro joints
- MoveIt/controller config updated to Pro joint list
- cancel calls stop/hold

Acceptance:

```bash
ros2 action list | grep follow_joint_trajectory
ros2 action send_goal /right_omnihand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...12-joint goal...}"
```

### Phase 5 — Skill controller recalibration

Deliverables:

- Pro tactile parser
- Pro calibrated open/pregrasp/close target vectors
- glass and bottle contact thresholds
- passive contact monitor
- Hefeweizen validation notes

Acceptance:

```text
10/10 open_hand success
10/10 release success
8/10 glass contact detected without overclosing
8/10 bottle contact detected without overclosing
no thermal/current fault during repeated low-speed calibration
```

### Phase 6 — Docs cleanup

Deliverables:

```text
docs/omnihand_pro_migration.md
docs/omnihand_pro_jetson_build.md
docs/omnihand_pro_tactile_layout.md
docs/omnihand_pro_calibration_log.md
docs/errors_and_fixes.md
config/omnihand_pro_gestures.yaml
config/omnihand_pro_joint_order.yaml
```

---

## 11. Expected File-Level Changes

### 11.1 `omnihand_bridge_node.py`

Replace monolithic O10 SDK backend with adapter-backed backend.

Change:

```python
declare_parameter("hand_model", "o12_pro")
```

Move these to model files:

```text
JOINT_SUFFIXES
TACTILE_FINGERS
SDK_ACTIVE_JOINT_MAX_RIGHT
SDK_ACTIVE_JOINT_MIN_RIGHT
SDK_MOTOR_MAX_RIGHT
SDK_MOTOR_MIN_RIGHT
SDK_ACTUATOR_*
polynomial conversion
gesture loading/mirroring
```

Add O12-Pro logic:

```python
if hand_model == "o12_pro":
    adapter = O12ProSdkAdapter(...)
elif hand_model == "o10":
    adapter = O10SdkAdapter(...)
else:
    raise ValueError(...)
```

### 11.2 `omnihand_exerciser_node.py`

Add:

```text
--model o12_pro
```

Load:

```text
config/omnihand_pro_gestures.yaml
```

instead of O10 presets.

### 11.3 `omnihand_follow_joint_trajectory.py`

Remove local duplicated joint list.

Add:

```text
hand_model parameter
shared build_joint_names(hand_model, side)
active stop/hold on cancel
```

### 11.4 `hand_skill_backend_mapping.md`

Keep architecture, update hardware assumptions:

```text
10-joint targets -> model-specific active target vector
O10 tactile raw -> O12-Pro tactile schema
glass/bottle presets -> calibrated O12-Pro presets
```

### 11.5 `omnihand_gesture_mapping.md`

Archive old O10 gesture inventory or rename it:

```text
omnihand_o10_gesture_mapping.md
```

Create:

```text
omnihand_pro_gesture_mapping.md
```

with:

- O12-Pro active joint order
- vendor demo vectors
- calibrated vectors
- safety notes
- object-specific grasp mappings

### 11.6 `errors_and_fixes.md`

Add new Pro migration section:

```text
## OmniHand Pro 2025 Migration

### x64 libusbcanfd copied into Jetson package
### SocketCAN hardcoded can0
### O12 active angle docs/demo mismatch
### Pro tactile flattening changed layout
### FollowJointTrajectory joint list stale after O10 removal
```

---

## 12. Known Unknowns and Required Local Validation

I cannot confirm the following from the uploaded files alone:

1. Whether the current vendor SDK is a submodule or normal vendored directory.
2. The exact local launch file names and installed config paths.
3. Whether the local URDF/Xacro already contains Pro joint names.
4. Whether `agx_arm_msgs/OmniHandTactileRaw` is acceptable long-term or should be replaced immediately.
5. Whether the Pro firmware returns 12 values for `get_all_active_joint_angles()` on our hardware. The vendor docs contain conflicting hints: some API text still mentions length 10, while Pro examples and product description point to 12.
6. Whether the target Pro SDK works reliably over native SocketCAN after patching, because upstream defaults to ZLG USB-CANFD and hardcoded `can0`.
7. Whether the Pro SDK class constructor needs only `device_id`/`hand_type` or also hidden communication parameters in our specific build.

Discovery commands for these unknowns are listed in Section 8 and should be run before code changes are finalized.

---

## 13. Acceptance Criteria for the Migration

### Build acceptance

- Pro SDK builds on Jetson AGX Orin.
- Python import works without manually replacing ROS `PYTHONPATH`.
- No x86 shared library is loaded into the Jetson runtime.
- SocketCAN interface can be selected via `OMNIHAND_SOCKETCAN_IFACE`.

### Hardware acceptance

- Bridge starts against right Pro hand on `can_nero_right`.
- Bridge starts against left Pro hand on `can_nero_left`, if connected.
- Read-only load test shows stable CAN error counters.
- Status readback works at 1 Hz.
- Active angle readback works at 50 Hz or chosen bridge feedback rate.
- Tactile readback works at a documented rate.

### ROS acceptance

- `feedback/omnihand/joint_states` publishes 12 correct Pro joints.
- `control/joint_states` commands only recognized hand joints and ignores arm-only updates.
- `control/omnihand/joint_trajectory` accepts 12-joint trajectories.
- `control/omnihand/stop` holds current pose.
- `FollowJointTrajectory` accepts valid Pro goals and rejects stale O10 goals.

### Skill acceptance

- `open_hand` succeeds repeatedly.
- `release_*` succeeds repeatedly.
- `grasp_glass_until_contact` stops on tactile contact, not only on final joint pose.
- `grasp_bottle_until_contact` stops on tactile contact, not only on final joint pose.
- contact loss produces warning/failure according to `fallback_policy`.
- no public task/action graph depends on vendor gesture IDs.

---

## 14. Recommended First Commit Breakdown

1. `vendor: add OmniHand-Pro-2025 fork`
2. `vendor: add Jetson SocketCAN build option for OmniHand Pro`
3. `omnihand: add model-aware joint definitions`
4. `omnihand: add O12 Pro SDK adapter`
5. `bridge: select hand model via launch parameter`
6. `tools: add OmniHand Pro import and hardware probes`
7. `tools: add OmniHand Pro exerciser presets`
8. `trajectory: use shared OmniHand joint model`
9. `docs: add OmniHand Pro migration and calibration notes`
10. `skill: update backend mapping for O12 Pro tactile contact`

This order keeps every step testable. Revolutionary, I know: a migration where each commit can actually be bisected. Civilization advances.

---

## 15. Source Notes Used for This Proposal

Online sources checked on 2026-06-24:

- `https://github.com/AgibotTech/OmniHand-Pro-2025`
- `https://github.com/AgibotTech/OmniHand-Pro-2025/blob/main/document/API_PYTHON.md`
- `https://raw.githubusercontent.com/AgibotTech/OmniHand-Pro-2025/main/python/agibot_hand/__init__.pyi`
- `https://raw.githubusercontent.com/AgibotTech/OmniHand-Pro-2025/main/python/example/demo_set_angle.py`
- `https://raw.githubusercontent.com/AgibotTech/OmniHand-Pro-2025/main/src/CMakeLists.txt`
- `https://raw.githubusercontent.com/AgibotTech/OmniHand-Pro-2025/main/python/CMakeLists.txt`
- `https://raw.githubusercontent.com/AgibotTech/OmniHand-Pro-2025/main/src/can_bus_device/socket_can/c_can_bus_device_socket_can.cc`
- `https://github.com/AgibotTech/OmniHand-Pro-2025`

Uploaded local context used:

- `omnihand_bridge_node.py`
- `omnihand_exerciser_node.py`
- `omnihand_follow_joint_trajectory.py`
- `hand_skill_backend_mapping.md`
- `omnihand_gesture_mapping.md`
- `errors_and_fixes.md`
