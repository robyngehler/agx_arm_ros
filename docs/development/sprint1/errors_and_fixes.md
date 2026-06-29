# Errors And Fixes

## 2026-05-11

### `rg` not installed in terminal

- Symptom: `bash: rg: command not found`
- Impact: fast terminal-side repository enumeration failed.
- Fix: switched to workspace-native tools (`file_search`, `grep_search`, `list_dir`) plus standard git commands.

### `src/agx_arm_sim/.gitmodules` not present

- Symptom: a direct read of `src/agx_arm_sim/.gitmodules` failed.
- Impact: the sim README references submodule management, but the nested file is not present in this checkout.
- Fix: inspected the actual directory layout under `src/agx_arm_sim/` and used repo-level `git submodule status` instead.

### Binary USD asset not returned by `file_search`

- Symptom: glob search for `*.usd` returned no results even though a USD asset exists in `src/agx_arm_sim/agx_arm_description/urdf/USD/nero_gripper_d435/`.
- Impact: Isaac asset discovery looked empty until verified manually.
- Fix: used `list_dir` and a direct `read_file` on the known path to confirm `nero_gripper_d435.usd` plus companion configuration files.

### Duplicate `agx_arm_description` packages in the same workspace

- Symptom: `colcon list --base-paths src` reported both `src/agx_arm_description` and `src/agx_arm_sim/agx_arm_description` with the same package name.
- Impact: package-share resolution and future builds would stay ambiguous while the root and sim trees coexisted as discoverable packages.
- Fix: kept `src/agx_arm_sim/agx_arm_description` as the canonical package, added a sim-backed `launch/display_control.launch.py` for the current control/RViz workflow, moved the tracked `agx_arm_urdf` submodule into that package, and removed the legacy `src/agx_arm_description` tree.

### Root and sim `display.launch.py` interfaces were not compatible

- Symptom: `agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py` passed arguments such as `namespace`, `effector_type`, `follow`, and `tcp_offset` that the sim package's `display.launch.py` does not accept.
- Impact: simply switching package discovery to the sim description package would have broken the existing RViz/control launch path.
- Fix: added `src/agx_arm_sim/agx_arm_description/launch/display_control.launch.py` with the required compatibility interface and repointed `agx_arm_ctrl` to use it.

### Runtime validation is still partial

- Symptom: Sprint 1 discovery still lacks real hardware motion validation and simulator execution coverage.
- Impact: current confidence is based on package build/test/launch smoke checks, not on arm motion or Isaac scene execution.
- Fix: a focused `colcon build` passed for `agx_arm_description`, `agx_arm_ctrl`, `agx_arm_moveit`, and `agx_arm_mit_controller`; `agx_arm_mit_controller` unit tests passed (`20 passed`); and both `agx_arm_description/display_control.launch.py` and `agx_arm_ctrl/start_single_agx_arm_rviz.launch.py` resolved successfully with `--show-args`.

### OmniHand vendor SDK is documented for `x86_64`, but this host is `aarch64`

- Symptom: `vendor/OmniHand-Pro-2025/README.md` documents Ubuntu 22.04 `x86_64`, while `uname -m` on this machine reports `aarch64`.
- Impact: the vendor SDK can be vendored and inspected locally, but a supported local build/run result cannot be claimed yet on this host.
- Fix: added the SDK as a git submodule for inspection and planning, but deferred build validation until Agibot's `aarch64` support is confirmed or an `x86_64` bring-up host is chosen.

### `agx_arm_urdf` still acted like a submodule after the canonical-package cleanup

- Symptom: even after the duplicate root package was removed, the canonical description package still depended on a tracked `agx_arm_urdf` git submodule and still carried Piper-family asset overhead.
- Impact: the workspace still required external submodule state for description assets, and active launch/config/docs continued to advertise models no longer intended for the Nero-focused repo state.
- Fix: detached `src/agx_arm_sim/agx_arm_description/agx_arm_urdf` from submodule management, pruned it to `nero/`, `revo2/`, and README/license files, and then updated the active launch/config/MoveIt surfaces to default to Nero only.

### Removed submodule left stale local git config behind

- Symptom: `git config --get-regexp '^submodule\.'` still returned `submodule.src/agx_arm_sim/agx_arm_description/agx_arm_urdf.*` after the submodule entry was removed from `.gitmodules`.
- Impact: local repository metadata still suggested a submodule dependency that no longer exists in the working tree.
- Fix: remove the stale local git config section with `git config --remove-section submodule.src/agx_arm_sim/agx_arm_description/agx_arm_urdf`.

### OmniHand asset bundle is not a drop-in ROS description package yet

- Symptom: the vendored OmniHand assets mix `package://omnihand_description/...` references with no matching ROS package, `assets/urdf/omnihand_right.urdf` contains absolute mesh paths from a local desktop checkout, and `assets/urdf/xacro/finger.xacro` contains a stray literal `y` before one joint declaration.
- Impact: the asset bundle cannot be trusted as a clean in-workspace description source for RViz or MoveIt integration without normalization.
- Fix: keep the SDK vendored for isolated device testing first, record the asset defects in Sprint 1, and defer creation of a repo-owned OmniHand description package until after wrapper-side bring-up is validated.

### OmniHand userspace transport bundle appears x64-only in the vendored tree

- Symptom: the vendored `thirdParty/` tree only ships `usbcanfd_libusb_x64_1.0.10_250328` as the obvious userspace CAN bundle, and the Python packaging copies that library into the built package.
- Impact: local `aarch64` bring-up may fail before any repo integration issue is even exercised.
- Fix: treat platform validation as a separate gate from workspace integration, and move first runtime validation to `x86_64` if Agibot does not provide an `aarch64` path.

## 2026-05-13

### OmniHand CAN readbacks could crash after request-send failures

- Symptom: on the socket-backed `aarch64` probe path, failed CAN requests left `GetAllJointMotorPosi()` returning an empty vector while higher-level read methods still pushed that vector into kinematics conversions.
- Impact: the isolated Phase 1 probe could report a process crash instead of a clean incomplete-runtime result when no responsive hand was present on the CAN path.
- Fix: guard the vendored CAN readback conversions against incomplete actuator vectors and return empty higher-level readbacks instead of continuing into invalid conversions.

### OmniHand Phase 1 probe could not write JSON results for pybind objects

- Symptom: once the runtime crash was removed, the repo-owned smoke test failed while serializing vendor return types such as `DeviceInfo` and `VendorInfo` into JSON.
- Impact: structured probe evidence was lost even though the child process now had useful runtime data to report.
- Fix: add recursive JSON normalization for pybind objects and classify incomplete joint-vector readbacks as `runtime_probe_incomplete` instead of treating them as successful enumeration.

### Socket-backed vendor rebuild still failed when Python wheel tooling was absent

- Symptom: the root vendor CMake warned that `build`, `setuptools`, or `wheel` were missing, but `python/CMakeLists.txt` still unconditionally ran `python -m build --wheel`.
- Impact: the socket-backed rebuild failed after the native module linked even though the repo only needed the unpacked `agibot_hand_pkg` directory for the Phase 1 probe.
- Fix: keep the unpacked package refresh target, but make wheel generation conditional on those Python packaging modules actually being available.

### MIT controller defaults drifted across code and config

- Symptom: `mit_controller_node.py` and `config/nero_mit_controller_defaults.yaml` no longer agreed on the default `kp` vector, and an accidental copied YAML file introduced a third unchecked profile.
- Impact: direct node execution and launch-based runs could diverge, and the repo carried an extra config file with ambiguous tuning intent.
- Fix: align the node defaults with the checked-in YAML profile and remove the stray copied config file.