# Sprint 6 — Errors & Fixes

Record concrete errors and their fixes here as the hand skill controller, performer routing, and
coordinator come up — e.g. tactile stream gaps, grasp-threshold miscalibration, `both_arms`
joint-ordering issues, coordinator resource deadlocks, or sync-group dispatch problems.

## 2026-06-29 (O12 MoveIt execution + build env)

### `colcon build` wrapper still failed `agx_arm_msgs` with "No module named 'cmake'"

- Symptom: even via `scripts/colcon_build_system_python.sh`, `agx_arm_msgs` died at
  `/home/user/.local/bin/cmake` → `ModuleNotFoundError: No module named 'cmake'`. A bare
  `colcon build` instead failed the *python* packages with `option --editable not recognized`.
- Cause: one root, two faces. The wrapper sets `PYTHONNOUSERSITE=1` (fixes the too-new `~/.local`
  setuptools that breaks `develop --editable`), but a pip `cmake` shim in `~/.local/bin` was still
  first on PATH and, with user-site disabled, could no longer import its own `cmake` module → it
  shadowed `/usr/bin/cmake` and crashed. Bare colcon had the opposite halves (cmake shim worked,
  setuptools broke the python packages).
- Fix: the wrapper now also drops `$HOME/.local/bin` from PATH (alongside conda/miniforge), so
  `/usr/bin/cmake` wins. All of msgs/ctrl/coordination/mit_controller/mit_demos build clean.

### MoveIt hand execution: "Unable to identify any set of controllers that can actuate ... [right_*_mcp_joint]"

- Symptom (duo, after the O12 description migration): planning the hand group succeeds, but
  execution aborts — move_group's only known controller is `arm_controller` (right_arm_joint1..7).
- Cause: the MIT/duo trajectory-execution config (`_build_mit_trajectory_execution`) registered
  only the per-arm controllers; no controller covered the 12 hand joints.
- Fix: register a `<ns>/<side>_omnihand_controller` FollowJointTrajectory controller (12 active O12
  joints) for any arm_instance carrying an omnihand, namespaced like its arm controller. Shared
  `OMNIHAND_O12_ACTIVE_JOINT_SUFFIXES` + helpers in `_multi_arm_runtime`; also replaced the stale
  O10 list in `start_moveit`'s generated hand JointTrajectoryController.
- **Real hand from MoveIt (chosen path, works with the fix above):** the per-arm driver chain in the
  MIT/duo path (`start_nero_mit_controller` → `start_single_agx_arm`) already launches the
  `omnihand_follow_joint_trajectory` FJT→bridge adapter at
  `<ns>/<side>_omnihand_controller/follow_joint_trajectory` when `effector_type:=omnihand` and
  `launch_omnihand_bridge:=true`. The adapter defaults to `hand_model=o12_pro` and accepts the 12
  active joints. Because the trajectory-execution controller name is derived from the same arm-instance
  namespace, it matches the adapter by construction — so once move_group knows the controller (the fix
  above), MoveIt finger plans drive the **physical** hand via the bridge. **Two runtime conditions:**
  (1) launch with the bridge on (`launch_omnihand_bridge:=true omnihand_backend_type:=sdk`), and
  (2) keep fake execution OFF — otherwise `start_moveit`'s mock `mock_components/GenericSystem` JTC
  also claims `<side>_omnihand_controller/follow_joint_trajectory` and competes with the adapter.
  The `omnihand_skill_controller` (semantic grasp/open/release) remains the higher-level real-hand path
  used by the coordinator; both share the same bridge.

## 2026-06-29 (arm64 host validation)

### `colcon build` fails with `option --uninstall not recognized`

- Symptom (arm64 Duo host): a bare `colcon build --packages-select agx_arm_ctrl` aborts with
  `error: option --uninstall not recognized`, and a setuptools `Unknown distribution option:
  'tests_require'` warning, sourced from `/home/user/.local/.../setuptools`.
- Cause: Python-env drift — the bare build picks up the user-site (`~/.local`) setuptools, whose
  `setup.py develop --uninstall` cleanup step (run by colcon on a prior install) is unsupported.
  Same class as `docs/control/environment.md` "Common failure patterns".
- Fix: build through the repo wrapper, which filters conda/miniforge from PATH, sets
  `PYTHONNOUSERSITE=1`, and forces `/usr/bin/python3`:
  `bash ./scripts/colcon_build_system_python.sh --packages-select agx_arm_ctrl agx_arm_coordination`.
  Validated: both Python packages build clean; `agx_arm_coordination` tests 32/32 pass.
- Side note: the C++ `agx_arm_msgs` build additionally needs a working cmake; a broken
  `~/.local/bin/cmake` shim (`ModuleNotFoundError: No module named 'cmake'`) can shadow the
  system cmake. Only relevant when the messages change (it is already installed clean).

### O10/O12 description drift is the `components.launch` MoveIt error

- Symptom: `move_group` loops `Joint 'right_index_mcp_joint' not found in model 'duo_nero_system'`
  (also middle/ring/pinky). Arm planning still succeeds.
- Cause: hand URDF + SRDF group + MoveIt controllers + initial_positions are still the O10 layout
  (`*_abad`/`*_pip`), while `models.py`/skills/SDK are O12 Pro (`*_mcp`, `thumb_pip` active). The
  exact error came from the crashed attempt's partial O12 SRDF over an O10 URDF (state not on disk
  now). Not a coordinator bug; the hand is skill-controlled, not MoveIt-planned, so it is log noise
  that does not block `both_arms` planning.
- Fix (planned next session, now vendor-grounded): the official O12 URDF is available at
  `vendor/OmniHand-Pro-2025/description/urdf/o12_hand_description-o12_t3/` (visual+collision meshes,
  both sides, mimic ratios that match `models.py`). Integrate it (reprefix `R_`/`L_`→`right_`/`left_`)
  and update SRDF/controllers/initial_positions/ros2_control to the 12 active joints. See
  `planning/session_handoff_2026-06-29.md`.

## 2026-06-29

### Sprint-6 orchestration + hand-skill layer landed (development host only)

- Implemented the full sprint-6 functionality slice: `agx_arm_msgs`
  PerformAction/PerformActivity/RobotEvent; `omnihand_skill_controller` (+ `omnihand/skills.py`,
  `config/omnihand_skills.yaml`) in `agx_arm_ctrl`; and the new `agx_arm_coordination` package
  (graph model, scheduler, resource model, YAML loader, performer routing, arm executor, coordinator
  node, configs, launch, tests).
- Validation boundary (no ROS / no arm64 host here): `py_compile` clean on all new files; the
  graph/scheduler/resource/sync logic was exercised directly on the full `hefeweizen_pour_v1` graph
  (drains in 15 ticks, sync pairs dispatch together, no in-batch resource clash). `colcon
  build/test` and the `pytest` suites must still run on the Jetson/Duo host (PyYAML + pytest are
  absent on the Windows dev host). Tracked in `planning/hefeweizen_validation_log.md`.
- Gotcha for the next person — **mock backend tactile is all zeros**, so a tactile-confirmed grasp
  (`grasp_*_until_contact`) cannot reach its threshold and will time out under `backend_type:=mock`.
  For dev/CI without hardware, use `hands_open_release_v1` (no grasp) and `arm_dry_run:=true` to
  exercise the coordinator end to end; real grasps need the SDK backend on the Pro hand.
- Determinism choices (per the sprint direction): skills map to the only calibrated O12 presets today
  (`open`/release → `zero`, grasps close toward `fist_vendor_demo` until contact); arm transitions
  command the named anchor-pose endpoint and let the controller interpolate (collision-aware MoveIt
  between anchors deferred); functional trajectories are recorded and start only from their designated
  anchor entry pose. Presets, thresholds, anchor poses, and recorded waypoints are placeholders to
  calibrate/teach on hardware.

## 2026-06-25

### OmniHand Pro (O12) migration left the exerciser + gesture/trajectory helpers on the O10 layout

- Symptom (live Pro hand, `hand_model:=o12_pro`): `omnihand_exerciser --model o12_pro …` failed with
  `unrecognized arguments: --model`; without `--model`, `fist` curled only index + middle, the thumb
  did not move, `zero` opened the hand and `open` did nothing. `feedback/omnihand/joint_states` looked
  mostly dead (only index_pip/middle_pip ever changed).
- Cause: the Pro migration (proposal §5–§7) was only carried through Phase 2 (model-aware `models.py`,
  `O12ProSdkBackend`, `hand_model` bridge/launch param). Phase 3 was never wired: the exerciser and the
  shared helpers `build_joint_names` / `load_gesture_presets` / `resolve_gesture_presets` in
  `omnihand_bridge_node.py` were hardcoded to the O10 10-joint layout and `omnihand_gestures.yaml`. The
  new `omnihand_pro_gestures.yaml` existed but **no code loaded it** (0 references). So the exerciser
  published O10 joint names; only `index_pip`/`middle_pip` overlap the O12 set, and O10 thumb values
  (`thumb_roll≈0.43`, `thumb_mcp≈0.66`) fall outside the O12 limits (`[-0.73,0]` / `[-0.86,0]`) and were
  clamped to 0 — hence "only two fingers, no thumb". `zero`/`open` only nudged those two overlapping
  joints, explaining the confusing open/no-op behavior.
- Fix: made the helpers model-aware (`build_joint_names(side, model)`,
  `load_gesture_presets(model)`, `resolve_gesture_presets(side, model)`, `mirror_active_joint_vector(v,
  model)`) keyed off a new `HandModel.gesture_config_file`; added `--model` to the exerciser (default
  `o12_pro`, validates the pose against the model's set instead of crashing in argparse); and removed
  the duplicate O10 `JOINT_SUFFIXES` from `omnihand_follow_joint_trajectory.py` in favor of the shared
  model-aware `build_joint_names` plus a `hand_model` parameter. `model=None` keeps the legacy O10
  behavior so the O10 mock/SDK path is unchanged. Validated with `colcon build` + a mock load of both
  models: o12_pro → 12 joints + `omnihand_pro_gestures.yaml`, o10 → 10 joints + legacy file; left mirror
  flips only thumb_roll/thumb_abad on the Pro.
- Not bugs (verified, documented so they are not chased): `tactile_raw` reading 0 except a leading 1 is
  correct — the 1 is `online_state`, the 0s are `normal_force`/`tangent_force` with no contact; status
  `active_joint_temperatures_c`/`active_joint_currents_a` are `[]` because the Pro SDK returns empty
  temperature/current report lists on this firmware.
- Follow-ups: `omnihand_pro_gestures.yaml` still only carries the vendor bootstrap (`zero`,
  `fist_vendor_demo`); calibrated `open`/grasp poses must be measured on the Pro hardware (proposal
  §7.1, §9.2) before the skill controller names grasp poses. Doc drift corrected alongside this fix:
  `sprint6/planning/omnihand_gesture_mapping.md` (O10-era banner), `sprint6/README.md` (12-joint Pro),
  `config/omnihand_gestures.yaml` header (legacy/mock-only). The earlier stable-doc drift around
  `docs/assets/omnihand_asset_validation.md` and `docs/assets/control/basic_control_scripts.md` has
  since been corrected; remaining work is hardware sign-off, not naming cleanup.

## 2026-06-24

### OmniHand vendor load test failed to import the SDK (PYTHONPATH shadowing)

- Symptom: `python3.10 scripts/omnihand/omnihand_load_test.py` failed with
  `ModuleNotFoundError: No module named 'agibot_hand.agibot_hand_core'`, even though
  `PYTHONPATH` was correctly set to the built package `build/agibot_hand_pkg`.
- Cause: the script defaulted `--sdk-python-dir` to the **source** tree `vendor/OmniHand-Pro-2025/python`
  and did `sys.path.insert(0, ...)`, prepending the source `agibot_hand` package ahead of the
  user's `PYTHONPATH`. The source tree has `__init__.py` but no compiled `agibot_hand_core` .so, so
  the import resolved to a core-less package. Same class as the `docs/control/environment.md`
  "PYTHONPATH replaced instead of appended" pattern.
- Fix: the load test now respects an already-set `PYTHONPATH` (no default source-tree insert) and only
  prepends an explicit `--sdk-python-dir`. If the import still fails because the package is absent or
  points at the source tree, it self-heals by appending the built package
  (`build/agibot_hand_pkg`) and retrying. `LD_LIBRARY_PATH` must still be exported
  before launch for the native library to load.
- Validated on the Jetson against the live right hand: 50 Hz `get_joint_positions` (~0.4 ms/call),
  1 Hz `get_all_error_reports` (~10 ms/call).

### OmniHand ROS bridge `backend_type:=sdk` could not find the vendor SDK on launch

- Symptom: `ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py backend_type:=sdk` died with
  `ModuleNotFoundError: No module named 'agibot_hand'` →
  `RuntimeError: backend_type=sdk requires agibot_hand on PYTHONPATH`. Prefixing the launch with
  `PYTHONPATH=.../agibot_hand_pkg ros2 launch ...` then broke `ros2` itself with
  `PackageNotFoundError: No package metadata was found for ros2cli`, because the inline `PYTHONPATH=`
  *replaced* ROS's own `PYTHONPATH` (which carries `ros2cli`) instead of appending.
- Cause: the bridge relied on an ambient `PYTHONPATH` for the vendor SDK, which is incompatible with
  `ros2 launch` (you cannot set a launch-only PYTHONPATH without clobbering ROS's).
- Fix: the bridge now self-locates the built vendor package. `_ensure_omnihand_importable()` tries the
  ambient import first, then an explicit `sdk_python_dir` param, then `AGX_ARM_OMNIHAND_SDK_DIR`, then
  an upward search for `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`, and adds it to
  `sys.path`. No `LD_LIBRARY_PATH` is needed: the compiled `agibot_hand_core.so` has `RUNPATH=$ORIGIN`,
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
  `vendor/OmniHand-Pro-2025/src/can_bus_device/socket_can/c_can_bus_device_socket_can.cc`). The numeric
  `canfd_id` only applies to the ZLG USB path. The bridge passed `canfd_id` but never set the env var,
  so the SDK always tried `can0`.
- Fix: added a visible side->interface mapping in `agx_arm_ctrl/config/omnihand_can_interfaces.yaml`
  (`right: can_nero_right`, `left: can_nero_left`). The bridge resolves the interface from that config
  by `omnihand_type` (overridable with the `can_interface` parameter / launch arg) and exports
  `OMNIHAND_SOCKETCAN_IFACE` before `create_hand`. It logs the resolved interface and its source.
  Validated on the Jetson: bridge opens `can_nero_right` and starts with `backend_type=vendor_sdk`.

## 2026-07-16 (CANFD TDC on the new transceiver + "bridge doesn't spawn")

### Constant `请求超时` on all OmniHand CANFD requests after swapping the bus transceiver

- Symptom: with the new (unknown-delay) transceiver and `tdc_offset=0x200`, every SDK
  request times out (`[ERROR]: CANFD ID: 0x00130101 请求超时`, `motor Input size does not
  match expected motor count`); with the old TJA1051T/3 and `0x800`, ~8/10 commands landed.
- Cause: `tdc_offset` is the raw M_CAN TDCR register; TDCO sits in bits [14:8] in CAN-clock
  ticks (20 ns at the 50 MHz mttcan clock). One 5 Mbit data bit = 10 ticks, so TDCO=8
  (`0x800`) puts the secondary sample point at 80 % of the bit — consistent with
  `dsample-point 0.8`. `0x200` (TDCO=2) samples at 20 %, inside the transceiver-delayed
  edge, producing bit errors on every BRS frame. Verified live: a read-only SDK worker at
  `0x200` scores 0/5 successful joint readbacks on `can_nero_left`.
- Fix/tooling: `scripts/can_tdc_sweep.py` sweeps `tdc_offset` per side bus (down →
  sysfs write → validated 1M/5M FD timing → up), measures SDK readback success rate,
  latency, and `bus_error` counter deltas per value, optionally verifies a fist→zero
  motion cycle, ranks the values, and restores (or `--apply-best` applies) the winner.
  Persist the winner via `TDCR_VALUE=0x... sudo bash scripts/activate_native_can.sh`.
- First idle-bus sweep (2026-07-16, new transceiver): TDCO 0 fails hard, TDCO 1 is
  flaky (left 10/10, right 5/10, and 0x0/0x100 flipped between runs), TDCO 2..15 all
  pass 100 % — so `0x200` is not broken per se on an idle bus; the permanent `请求超时`
  the bridge sees must involve arm bus load. An idle bus does NOT discriminate inside
  the plateau; two-stage method: (1) idle sweep maps the raw window, (2) re-sweep the
  plateau with `--arm-load` (arm bringup without hand bridges, MIT controller actively
  commanding — idle-hold is not load) and `--passes 3+`; per-trial bus pkt/s is logged
  and low-load trials are flagged. The recommendation is the CENTER of the longest
  clean TDCO window (edge values are marginal), and the motion check judges delivery
  by progress-from-baseline toward the target (absolute command==readback tolerance is
  not calibrated on the Pro and fails even when the hand visibly moves).
- Second sweep under "arm stack running" (2026-07-16): still ~96-100 % everywhere with
  no TDCO-correlated pattern — the ~2200 pkt/s were almost pure RX (firmware feedback
  push at 200 Hz). Merely launching the bringup is NOT the failure regime: the MIT
  controller starts DISABLED and transmits nothing. The real load regime needs, per
  arm namespace (`enable_agx_arm` is already auto; NO manual `set_motion_mode` needed —
  the driver handshakes `set_motion_mode('mit')` on the first `move_mit` command):
  `ros2 service call /<ns>/mit_controller/enable std_srvs/srv/SetBool "{data: true}"`
  then `ros2 service call /<ns>/mit_controller/hold_current std_srvs/srv/Empty` — the
  controller then streams 7 move_mit frames per tick at control_rate_hz (~700 tx/s per
  arm). `agx_arm_test_position_hold --leave-mit-enabled --no-wait` runs the full
  sequence (now accepts `--ros-args -r __ns:=/left_arm`; previously argparse rejected
  ROS args). The sweep now logs per-trial TX pkt/s and flags trials below ~150 tx/s as
  `low-tx-load(mit-not-streaming?)`.
- Observation during sweeps with the arm stack up: `agx_arm_ctrl_single` fires
  "CAN bus stall detected (tx_stall_count=0, is_ok=True); starting recovery" every few
  seconds (trigger: >0.5 s without good feedback — `feedback_timeout`), recovering on
  attempt 1 each time. Each per-value interface down/up causes at least one such gap;
  the observed frequency was higher than the trial cadence, so either the hand
  worker's FD traffic or the recovery cycle itself also stalls the vendor feedback
  parser — unverified. The recovery re-enable spam itself perturbs the bus, so for
  sweep runs consider `bus_recovery_enabled:=false` (or a larger `feedback_timeout`)
  on the arm driver and re-enable it for production.

### "CAN bus stall" storm under active MIT streaming is a false positive — the bus is clean

- Reproduction (2026-07-16, duo_arm, NO hand attached to the session): enable both MIT
  controllers (`mit_controller/enable` + `hold_current`); after ~30-50 s each
  `agx_arm_ctrl_single` enters a strictly periodic (~7.4 s) recovery storm:
  "CAN bus stall detected (tx_stall_count=0, is_ok=True)".
- Measured evidence that the CAN side is innocent: 65 s passive `candump` on
  `can_nero_left` during the storm regime shows 139 844 frames (~2 150/s, IDs
  0x251-257/0x261-267/0x2A1-2A9) with a **maximum inter-frame gap of 26.5 ms** on any
  feedback ID (bus-wide max 6.7 ms). The stall trigger requires >500 ms of parsed-
  feedback silence — the frames never stopped; the *parsing/accounting* did. Both
  driver processes sit at ~100 % of a core while MIT streams; the vendor-lib readiness
  check is `get_joint_angles().hz > 0`, where `hz` is pyAgxArm's FPSManager 0.1 s
  window delta — one starved 0.1 s window in a GIL-saturated Python process reads as
  hz==0, and 0.5 s of that trips `feedback_timeout`. Each recovery then does a full
  disconnect/reconnect/re-enable, which is itself the main disturbance.
- Follow-up observation in the same session: later BOTH drivers silently stopped
  forwarding MIT commands entirely — `control/move_mit` flows at 100 Hz on both arms
  (ROS side) but candump shows **zero** 0x15A-0x160 TX frames on either bus, while
  feedback keeps publishing with live payloads. The arms are then NOT actively
  regulated although the MIT controllers stream. Verify real TX load with
  `timeout 5 candump can_nero_left,15A:7F0 | wc -l` (~3 500 expected at 100 Hz).
- The user's console (driver stdout, not captured in launch.log) confirmed the
  cascade: three freshness checks interact — (1) the vendor 0.1 s fps window feeds
  `_check_arm_ready` (`hz > 0`), so a starved window rejects every move_mit callback
  with "Agx_arm is not connected, cannot control" at exactly the 100 Hz command rate;
  (2) the driver watchdog (`feedback_timeout` 0.5 s of hz==0) then runs the
  disconnect/reconnect recovery; (3) the MIT controller separately logs "Paused MIT
  command publishing because feedback is stale" when the driver's publish loop gaps
  long enough. Under CPU saturation the system oscillates between storm phases and
  quiet-but-dead phases (controllers streaming, drivers rejecting silently). All
  three trip while candump shows uninterrupted 130-200/s feedback frames per ID.
### Hand CANFD requests fail 100 % while the arm MIT stream is active — NOT a TDC problem

- Setup: driver-fix build (timestamp-based readiness), MIT controllers actively
  streaming (verified 700 tx/s per bus via kernel counters — NOTE: neither pyAgxArm
  (`local_loopback: False` in can_comm.py) nor the OmniHand SDK enable SocketCAN
  local loopback, so candump shows NEITHER of our own TX streams; use
  `/sys/class/net/<if>/statistics/tx_packets` to verify TX, never candump.
- Result: TDC sweep under load scores **0/1050 readbacks across TDCO 2..15** on both
  buses, ~+1 `bus_error` per request. Error-frame capture
  (`candump -e "can_nero_left,0:0,#FFFFFFFF"`) shows exactly one
  `20000088` frame per attempt: protocol-violation **tx-recessive-bit-error** +
  bus-error — during our FD-BRS transmission another node pulled the bus dominant
  (typical for a receiver that is not FD-tolerant issuing an error flag).
- Differential facts: (1) pure MIT classic streaming, 15 s, both buses: **zero** bus
  errors — classic TX/RX is flawless; (2) hand FD requests with the arm connected and
  pushing feedback but NO Jetson TX: **100 %** success at TDCO 2..15; (3) hand FD
  requests while MIT streams: **0 %**, TDCO-independent, both arms (fw 1.06 + 1.11).
- Conclusion: TDC/transceiver timing is NOT the lever (sweep now prints "no
  recommendation" when everything scores 0 %). Two live hypotheses: (A) the Nero
  firmware's CAN controller is not FD-tolerant while in MIT/CAN_CTRL mode and error-
  flags every FD frame (digital, deterministic); (B) signal-integrity of the new
  transceiver under dense back-to-back traffic (analog — would explain why the old
  TJA1051T/3 got ~8/10 with app-level retries and the new transceiver gets 0).
  Discriminating experiments: sweep one value with `--one-shot off` (does controller
  retransmission ever get a frame through?); lower `mit_control_rate_hz` (10, then
  50 — density vs. mode); scope the FD frame under load vs. idle. If FD-under-MIT
  stays structurally dead, move the hands to their own bus
  (`scripts/omnihand_canfd_activate.sh`, USB CAN-FD adapter path) or take the FD
  tolerance question to the arm vendor.

- Fix (implemented 2026-07-16): `_check_arm_ready` now treats the kernel RX timestamp
  of the last parsed feedback frame as the authoritative liveness signal — the arm is
  ready as long as that timestamp keeps advancing within `feedback_timeout` (advance
  is dated with the local monotonic clock, so no cross-clock comparison); the
  instantaneous `hz > 0` window remains only as a fallback for transports that never
  populate frame timestamps. `feedback_timeout` default raised 0.5 -> 2.0 s. A
  recovery expires the advance window first so `_wait_for_feedback` demands a
  genuinely new frame after the reconnect. Note the bringup default is
  `mit_control_rate_hz` 100 (the components launch overrides the node/config 50);
  lowering it to 50 remains the quickest CPU-load lever if the driver still pegs a
  core. Longer term the driver hot path belongs out of Python.

### `duo_hand` bringup "does not spawn" the OmniHand bridge

- Symptom: after `start_agx_arm_components.launch.py ... execution_profile:=duo_hand
  omnihand_backend_type:=sdk`, hand commands do nothing; a manually launched
  `start_omnihand_bridge.launch.py` works. The hand only reacts once the teach manager runs.
- Cause: the bridge IS spawned — launch logs show two `omnihand_bridge` processes — but one
  per arm instance inside the `/left_arm` and `/right_arm` namespaces, subscribing to
  `/left_arm/control/joint_states` + `/left_arm/control/omnihand/joint_trajectory` (same for
  right). Commands published to the root-level topics (which the manual root-namespace bridge
  serves) never reach them; the teach manager works because it auto-discovers the prefixed
  topics. Also: the profile-spawned Python nodes buffer stdout, so the bridge init line and
  SDK errors may not appear in `launch.log` — check `ros2 node list | grep omnihand` and
  `ros2 param get /left_arm/omnihand_bridge_node backend_type` instead.
- Note: `enable_debug_joint_trajectory_topic:=true` is silently dropped by
  `start_agx_arm_components.launch.py` — the argument is not declared/forwarded there; set it
  on the MIT controller launch level if needed.

## 2026-07-17 (one-shot off default, exerciser vs MoveIt, freedrive start state)

### `one-shot off` promoted to the default bus configuration

- With `ONE_SHOT=off` (now the default in `scripts/activate_native_can.sh` and
  `scripts/omnihand_canfd_activate.sh`) the hand error flood under arm load drops to a
  periodic `CANFD ID: 0x00200101 请求超时` roughly every 2 s — retransmission gets most hand
  frames through the arbitration pressure that one-shot silently dropped. TDCR sweeps
  (`scripts/can_tdc_sweep.py`, promoted to `scripts/`) confirmed TDC timing is NOT the lever:
  wide flat TDCO window when idle, 0 % under MIT load at every TDCO. Stable docs updated:
  `docs/assets/omnihand/omnihand_canfd_setup.md`, `docs/control/bringup.md`,
  `docs/control/teach_and_run.md`.

### Exerciser command never reaches the hand while MoveIt trajectories move it

- Symptom: with the Duo bringup, `omnihand_exerciser --gesture ...` never moves the hand no
  matter how often it is sent, while RViz/MoveIt-planned hand trajectories execute promptly.
- Cause: not a bus problem. Both paths converge in the bridge's `_submit_command` (identical
  CAN behavior, including readback-verified retry). The difference is ROS-side routing: the
  Duo bringup namespaces each bridge (`/left_arm`, `/right_arm` from the motion registry), and
  MoveIt reaches it via the namespaced `<side>_omnihand_controller/follow_joint_trajectory`
  action, while the exerciser published `JointState` to root-level `control/joint_states`,
  which nothing subscribes to.
- Fix: `omnihand_exerciser` now uses the MoveIt path by default — it resolves the side
  namespace from the motion registry and sends a `FollowJointTrajectory` goal (falling back
  to the namespaced `control/omnihand/joint_trajectory` topic when the action server is not
  up, e.g. a standalone bridge; pass `--namespace ''` for the root-namespace solo bringup).
  The old shared-topic publish remains behind an explicit `--topic`.

### MoveIt plans from the pre-freedrive state after dragging the arm

- Symptom: after moving the arm in freedrive/drag mode, RViz `<current>` start state and
  planning still use the old pose.
- Cause: in leader/drag mode the firmware disables the normal joint-state CAN push
  (`enable_can_push=DISABLE`), so `agx_arm_ctrl_single_node._publish_joint_states` returned
  early (`get_joint_angles().hz <= 0`) and `feedback/joint_states` froze at the pre-freedrive
  pose; the joint-state merger kept republishing that frozen message, so move_group's current
  state monitor never saw the dragged pose. The live signal during drag is
  `feedback/leader_joint_angles`, which MoveIt never consumes.
- Fix: `_publish_joint_states` now uses the same frame-advance liveness rule as
  `_check_arm_ready` (instantaneous `hz` starves under GIL pressure) and, when the normal
  stream is silent in leader mode, republishes the leader-angle stream (plus
  gripper/hand/omnihand joints) onto `feedback/joint_states` so MoveIt keeps tracking. The
  merger additionally stamps the merged message with the newest source stamp instead of the
  last source in the list (one stalled source no longer freezes the merged header stamp).
- Hardware validation still pending: verify in RViz that the `<current>` start state follows
  the arm during drag mode and that planning starts from the dragged pose.

### `mit_control_rate_hz` bringup default lowered 100 -> 50

- `start_agx_arm_components.launch.py` now defaults to 50 Hz (matching the MIT controller's
  own default), halving per-bus MIT frame load and the driver CPU load that fed the false
  bus-stall storms. `docs/assets/control/single_vs_multi_arm_control_chain.md` updated.

### Bus meltdown when the teach manager enables MIT with hands on the shared bus (one-shot off)

- Symptom (2026-07-17): components.launch (duo_hand, sdk backend) runs clean until the teach
  manager enables the MIT controllers; then a cascading storm: both bridges spam `请求超时` +
  "motor Input size does not match expected motor count", both drivers loop "CAN bus stall
  detected (tx_stall_count=0, is_ok=True)" -> recovery every ~0.5 s, MIT controllers pause on
  stale feedback, the teach manager's `mit_controller/enable` call times out. Restarting
  components.launch resumes the storm IMMEDIATELY; only `ip link down/up` clears it.
- Bus statistics after the storm (both side buses, symmetric): ~25 000 bus-errors, ~170 000
  RX dropped, error-warn/error-passive counters set, **bus-off 2 / re-started 2** — the bus was
  physically in an error storm, not just busy.
- Mechanism: the sprint-6 finding "hand FD frames are error-flagged while the arm streams MIT"
  (0 % FD delivery under MIT, TDCO-independent) combines with `one-shot off`: every failed hand
  FD frame is now **retransmitted by the M_CAN controller indefinitely**, each attempt drawing
  another error flag (~+1 bus_error per attempt) — an unbounded error flood that starves the
  arm's classic traffic (driver sends fail -> comm-error recovery path, hence the 0.5 s cadence
  independent of feedback_timeout). A stuck FD frame keeps retransmitting from the controller
  TX FIFO even after the ROS processes die, which is why only a link down/up (FIFO flush)
  recovers and why a relaunch re-storms instantly (the bridges' first 20 Hz readback probes
  refill the loop).
- Trade-off now explicit: on the shared bus with an MIT-streaming arm the hand is broken either
  way — `one-shot on` = silent drops (hand dead under load, bus stays healthy), `one-shot off`
  = eventual delivery when the arm pauses BUT unbounded error storm while it streams.
- Mitigation (implemented): the bridge now backs off all periodic SDK polling to a single probe
  per `fault_poll_interval_s` (default 2 s) while the backend reports a communication fault,
  and skips status/tactile reads entirely (cached snapshots republished) until one readback
  succeeds. This stops the 20 Hz+ request stream from feeding the flood but cannot fix FD
  delivery under MIT streaming.
- Open decision (needs hardware/vendor input): (a) move the hands to their own CANFD bus
  (USB adapter path, `scripts/omnihand_canfd_activate.sh`), (b) bus time-sharing (pause the
  MIT stream around hand commands), or (c) vendor escalation of the arm firmware's FD
  tolerance in MIT/CAN_CTRL mode. Until decided: with hands on the shared bus, do not run
  hand SDK traffic while MIT is enabled, or bring the bus up with `ONE_SHOT=on` for
  arm-only sessions.

### The driver recovery loop is a storm amplifier: latched comm error + no cooldown

- Counter-observation to the pure bus story (2026-07-17, second session): after a restart the
  stack runs quiet, stays quiet for a while after the teach manager starts (hand faults only
  every few seconds, bridge backoff engaging/releasing), then "tips" into the storm. During
  the storm the LEFT driver recovered every 60-100 ms ("recovery succeeded on attempt 1" ->
  1-20 ms later "stall detected"), far below any feedback_timeout — so a software loop, not a
  bus timeout, drives the recovery cadence.
- Mechanism (pyAgxArm `can_comm.py` + `driver_context.py`): send/recv ENOBUFS is swallowed but
  **latched** in `comm.last_error`; it is only cleared by a clean **data** frame in recv —
  **error frames never clear it** (`if not msg.is_error_frame`). The node's watchdog path B
  (`has_comm_error()` + ENOBUFS classify) therefore stays armed during an error-frame-dominated
  phase. Reconnect clears the latch, but the recovery's OWN sends (enable x7, set_speed,
  set_tcp) hit the congested bus and re-latch a fresh ENOBUFS before the next publish-loop
  iteration -> immediate re-trigger -> self-sustaining 60-100 ms disconnect/enable loop that
  itself floods the bus. This is the amplifier that turns "hand FD frames failing" into the
  full cascade; it predates the 2026-07-16/17 driver changes but was previously masked by the
  0.5 s feedback_timeout storms having a different rhythm.
- Fixes (implemented):
  - driver: `bus_recovery_cooldown_s` (default 5 s) — after a completed recovery the watchdog
    holds off; one WARN when a re-trigger is suppressed. Plus `_clear_comm_error()` at the end
    of recovery so errors latched by the recovery's own sends never count as a fresh trigger.
  - bridge: fault backoff now has hysteresis — full-rate polling resumes only after 3
    consecutive clean readbacks at the slow cadence (one lucky probe no longer flips the
    bridge back into a 20 Hz+ request burst on a still-congested bus).
- What is still NOT explained by code alone: why several minutes of MoveIt hand testing
  (2026-07-16 evening, one-shot off) stayed clean. Most plausible: the MIT controllers were
  never enabled in that session (they start DISABLED; RViz hand planning does not enable
  them), so the arms never streamed and hand FD delivery worked. Needs confirmation.
- Discriminating A/B for the next hardware run (order matters):
  1. Baseline: teach-manager scenario with the new cooldown build; watch
     `ip -statistics link show can_nero_left` bus-errors slope BEFORE vs AFTER the tipping
     point (if it still tips).
  2. Same scenario with `bus_recovery_enabled:=false` on both drivers: if the bus-error slope
     and the tipping vanish, the recovery loop was the main driver; if bus-errors still climb
     (~1 per hand request) and the stack tips, the FD-retransmit flood is primary.
  3. Same scenario with the hand bridges NOT launched (`launch_omnihand_bridge:=false`): zero
     FD traffic; if everything stays clean for minutes, hand FD under MIT remains the root
     trigger, and the shared-bus decision (own hand bus / time-sharing / vendor) stands.
