# Sprint 6 — Errors & Fixes

Implementation has not started yet (planning sprint). Record concrete errors and their fixes here as
the hand skill controller, performer routing, and coordinator come up — e.g. tactile stream gaps,
grasp-threshold miscalibration, `both_arms` joint-ordering issues, coordinator resource deadlocks, or
sync-group dispatch problems.

## 2026-06-24

### OmniHand vendor load test failed to import the SDK (PYTHONPATH shadowing)

- Symptom: `python3.10 scripts/omnihand/omnihand_load_test.py` failed with
  `ModuleNotFoundError: No module named 'omnihand_2025.omnihand_2025_core'`, even though
  `PYTHONPATH` was correctly set to the built package `build_phase1_socket/omnihand_2025_pkg`.
- Cause: the script defaulted `--sdk-python-dir` to the **source** tree `vendor/Omnihand-2025-SDK/python`
  and did `sys.path.insert(0, ...)`, prepending the source `omnihand_2025` package ahead of the
  user's `PYTHONPATH`. The source tree has `__init__.py` but no compiled `omnihand_2025_core` .so, so
  the import resolved to a core-less package. Same class as the `python_environment_workflow.md`
  "PYTHONPATH replaced instead of appended" pattern.
- Fix: the load test now respects an already-set `PYTHONPATH` (no default source-tree insert) and only
  prepends an explicit `--sdk-python-dir`. If the import still fails because the package is absent or
  points at the source tree, it self-heals by appending the built package
  (`build_phase1_socket/omnihand_2025_pkg`) and retrying. `LD_LIBRARY_PATH` must still be exported
  before launch for the native library to load.
- Validated on the Jetson against the live right hand: 50 Hz `get_joint_positions` (~0.4 ms/call),
  1 Hz `get_all_error_reports` (~10 ms/call).

### OmniHand ROS bridge `backend_type:=sdk` could not find the vendor SDK on launch

- Symptom: `ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py backend_type:=sdk` died with
  `ModuleNotFoundError: No module named 'omnihand_2025'` →
  `RuntimeError: backend_type=sdk requires omnihand_2025 on PYTHONPATH`. Prefixing the launch with
  `PYTHONPATH=.../omnihand_2025_pkg ros2 launch ...` then broke `ros2` itself with
  `PackageNotFoundError: No package metadata was found for ros2cli`, because the inline `PYTHONPATH=`
  *replaced* ROS's own `PYTHONPATH` (which carries `ros2cli`) instead of appending.
- Cause: the bridge relied on an ambient `PYTHONPATH` for the vendor SDK, which is incompatible with
  `ros2 launch` (you cannot set a launch-only PYTHONPATH without clobbering ROS's).
- Fix: the bridge now self-locates the built vendor package. `_ensure_omnihand_importable()` tries the
  ambient import first, then an explicit `sdk_python_dir` param, then `AGX_ARM_OMNIHAND_SDK_DIR`, then
  an upward search for `vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg`, and adds it to
  `sys.path`. No `LD_LIBRARY_PATH` is needed: the compiled `omnihand_2025_core.so` has `RUNPATH=$ORIGIN`,
  so it finds `libomnihand2025.so` next to it. `ros2 launch ... backend_type:=sdk` now works with no
  manual env. Validated on the Jetson: bridge starts with `backend_type=vendor_sdk` against the live
  right hand.

### OmniHand load test `--with-commands` crashed with IndexError on the baseline read

- Symptom: `omnihand_load_test.py --with-commands` printed
  `Unexpected actuator vector size: expected 10, got 12` /
  `Failed to read a complete actuator vector for active-joint conversion.` and then crashed with
  `IndexError: list index out of range` at `target[4]`, plus the SDK error
  `无效参数，需与主动自由度数量 10 相匹配` (invalid parameter, must match 10 active DOF).
- Cause: the command path read its sweep baseline via `hand.get_all_active_joint_angles()`. On this
  firmware that readback fails its internal actuator-vector conversion (12 vs 10) and returns a
  short/empty list, so `target[4]` indexed past the end and `set_all_active_joint_angles([])` was
  rejected. The bridge avoids this by trimming padded 12→10 vectors and never blind-indexing.
- Fix: `acquire_reference_pose()` now trims a padded vector to 10 and validates it; if the readback is
  incomplete it falls back to the safe open pose (warning that the hand will move to open first). The
  sweep is length-guarded, one-directional, and clamps the swept joint to a safe range. Validated on
  the Jetson with `--with-commands --tactile`: 500 reads + 500 commands + 10 status + 50 tactile in
  10 s, zero errors.

### OmniHand SDK bridge opened `can0` instead of the native `can_nero_right`/`can_nero_left`

- Symptom: `ros2 launch ... start_omnihand_bridge.launch.py backend_type:=sdk omnihand_type:=right`
  died with `Failed to resolve SocketCAN interface 'can0'`, even though the native side bus is named
  `can_nero_right` (from `scripts/activate_native_can.sh`).
- Cause: the vendor SocketCAN backend selects its interface ONLY from the `OMNIHAND_SOCKETCAN_IFACE`
  env var, defaulting to `can0` (confirmed in
  `vendor/Omnihand-2025-SDK/src/can_bus_device/socket_can/c_can_bus_device_socket_can.cc`). The numeric
  `canfd_id` only applies to the ZLG USB path. The bridge passed `canfd_id` but never set the env var,
  so the SDK always tried `can0`.
- Fix: added a visible side->interface mapping in `agx_arm_ctrl/config/omnihand_can_interfaces.yaml`
  (`right: can_nero_right`, `left: can_nero_left`). The bridge resolves the interface from that config
  by `omnihand_type` (overridable with the `can_interface` parameter / launch arg) and exports
  `OMNIHAND_SOCKETCAN_IFACE` before `create_hand`. It logs the resolved interface and its source.
  Validated on the Jetson: bridge opens `can_nero_right` and starts with `backend_type=vendor_sdk`.
