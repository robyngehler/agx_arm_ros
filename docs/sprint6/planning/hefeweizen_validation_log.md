# Hefeweizen Validation Log

Running record of what was validated for the coordinated pour, where, and with
what measured values. Development-host entries (no ROS/hardware) and Jetson/Duo
hardware entries are kept separate so calibration placeholders are obvious.

> Hardware calibration is still pending for all tactile thresholds, grasp/open
> presets, anchor poses, and recorded trajectories. Until those are measured,
> the demo runs only with `arm_dry_run:=true` and the open/release hand smoke
> test; grasps need real tactile contact.

## 2026-06-29 — implementation slice (development host, no ROS/hardware)

**Scope.** Built the sprint-6 orchestration + hand-skill layer: `agx_arm_msgs`
PerformAction/PerformActivity/RobotEvent, the `omnihand_skill_controller`
(`agx_arm_ctrl`), and the new `agx_arm_coordination` package (coordinator,
performer routing, arm executor, YAML graph/catalogue loader, mini graphs).

**Validated here (no ROS):**

- `python -m py_compile` clean on every new node/module/launch/test file.
- Graph/scheduler/resource/sync logic exercised directly on the full
  `hefeweizen_pour_v1` graph (rebuilt as in-memory dicts): validates clean,
  drains in 15 scheduler ticks over all 18 nodes with no deadlock, the three
  `sync_flag` pairs (20/21, 40/41, 120/121) dispatch together, and no two
  actions in any batch share physical units (both_arms vs per-arm serialization
  holds).
- Routing: `Gripper+{left,right}_hand` → hand, `Trajectory+{both_arms,*_arm}` →
  arm; mismatches raise `RoutingError`.
- Tactile/contact and skill-catalogue helpers behave (mean/max/min aggregation,
  finger aliasing ring→ring_tip / pinky→little_tip, fallback skill map
  open→`zero`, grasp→`fist_vendor_demo`).

**Could NOT run here (call out explicitly):** `colcon build` / `colcon test`
(no ROS / arm64 host), the `pytest` unit suites (PyYAML + pytest absent on the
Windows dev host), and any live hand/arm motion. Run on the Jetson/Duo host:
`colcon build --packages-select agx_arm_msgs agx_arm_ctrl agx_arm_coordination`
then `colcon test --packages-select agx_arm_ctrl agx_arm_coordination`.

## 2026-06-29 — arm64 Duo host, toolchain validation (right side live)

First run of the real toolchain on the hardware host (previous entry was a Windows
`py_compile` slice only). Right side live: `can_nero_right` UP, hand + right arm via
`nero_right_arm`; left side / `can_nero_left` not connected.

**Validated here:**

- `colcon build` of `agx_arm_msgs`, `agx_arm_ctrl`, `agx_arm_coordination` — clean, via
  `scripts/colcon_build_system_python.sh` (a bare `colcon build` fails with
  `option --uninstall not recognized` due to `~/.local` setuptools — see errors_and_fixes).
- `agx_arm_coordination` unit tests — 32 passed (graph_model, graph_loader, arm_executor,
  performer) under system Python.
- Vendor SDK import OK: `agibot_hand.AgibotHandO12` present from
  `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`.
- Sprint-6 catalogue/skill/pose names cross-checked consistent.

**Could NOT run here yet:** any live hand/arm motion, the skill controller against the Pro
hand, the coordinator end-to-end. The SDK-path migration (bridge + load/smoke scripts) is in
the working tree, builds clean, but is **not committed** and **not yet exercised on hardware**.
The O12 description migration (now vendor-grounded — official URDF available) is the next blocker
to a clean `components.launch`. See `session_handoff_2026-06-29.md`.

## 2026-06-29 — O12 description migration + teach tooling (arm64 host)

- **OmniHand description migrated O10 → O12 Pro** using the official vendor URDF
  (`o12_hand_description-o12_t3`), imported into `agx_arm_description`
  (`omnihand/urdf/xacro_pro` + `meshes_pro`). MoveIt surfaces updated to the 12
  active joints: SRDF `omnihand_group`, `moveit_controllers_omnihand_{right,left}`,
  `initial_positions`, `ros2_control`. **Validated:** description + moveit build
  clean; standalone and duo URDFs expand; offline SRDF-vs-URDF cross-check reports
  0 missing joints/links → the `components.launch` "Joint 'right_*_mcp_joint' not
  found in model 'duo_nero_system'" error is resolved. Live RViz render + the duo
  `components.launch` clean-log confirmation still owed on hardware.
- **Pose-capture tool added** (`agx_arm_capture_anchor_pose`, `agx_arm_mit_demos`):
  snapshots the live arm joint vector into a named `arm_config.yaml` anchor pose,
  comment-preserving. Offline-tested (update + insert). Completes the teach loop with
  the existing leader recorder + performer. See `../../../control/bringups/teach_and_run.md`.
- Still NOT run on hardware here: live arm motion, the skill controller against the
  Pro hand, RViz. Those are the right-side bring-up steps in `../../../control/bringups/teach_and_run.md`.

## 2026-06-30 — central teach tool + cleanup (development host)

- **Central teach manager added** (`agx_arm_teach_manager`, `agx_arm_mit_demos`): one keyboard UI
  for the whole teach loop — idle/freedrive, record (leader), playback (MIT), anchor-pose capture
  into `arm_config.yaml` (`a`), and recorded→catalogue waypoint conversion (`w`). Reuses the leader
  recorder, saved-trajectory executor, `capture_anchor_pose.update_pose_in_config`, and the new
  converter; no new motion code. Modelled on `wakeword_motion_manager`.
- **Waypoint converter added** (`agx_arm_recorded_to_catalogue`): downsamples a `RecordedTrajectory`
  JSON to N timed waypoints and emits a paste-ready `waypoints:` block (sidecar + stdout). Closes
  the manual-transcription follow-up. Does not auto-rewrite the comment/flow-style `catalogue.yaml`.
- **Dead code dropped:** removed the unused `sdk_import_package` / `sdk_class_name` registry fields
  from `omnihand/models.py` (the bridge hardcodes `agibot_hand`/`AgibotHandO12`); `o10` stays
  mock-only.
- **Stable-doc drift reconciled:** `omnihand_asset_validation.md` and `basic_control_scripts.md` no
  longer say "mock-only" / "10 active joints" (the `backend_type:=sdk` O12 path is validated on the
  Jetson); vendor repo-rename note (`agillink_omnihand_sdk`) added to `omnihand_vendor_sdk_aarch64.md`;
  `../../../control/bringups/teach_and_run.md` now states the arm-execution seam honestly (per-arm via `action_name`
  override; `both_arms` is planning-only with no execution controller yet).
- **Validated here (no ROS/hardware):** `py_compile` clean on the new modules + `models.py`; the
  converter downsampling exercised directly (endpoints preserved, monotone, ≤ max_points).
- **Could NOT run here:** `colcon build`/`colcon test`, the teach manager against live arm/hand
  (needs leader mode + MIT services + Duo host). Run on the Jetson/Duo via
  `scripts/colcon_build_system_python.sh --packages-select agx_arm_mit_demos`.

## 2026-06-30 — both_arms execution seam + profile resolution (development host)

- **`both_arms` execution path wired.** New fan-out FollowJointTrajectory bridge
  `agx_arm_both_arms_trajectory_bridge` (`agx_arm_mit_tools`) owns
  `both_arms_controller/follow_joint_trajectory`, splits the combined 14-joint goal by
  `left_arm_`/`right_arm_` prefix, and forwards a sub-goal to each namespaced per-arm controller
  (`/<side>_arm/arm_controller/follow_joint_trajectory`), aggregating results (both must succeed;
  one failure cancels the other). New bring-up launch
  `agx_arm_mit_controller/launch/start_both_arms_execution.launch.py` starts both per-arm MIT
  controllers + the bridge. `arm_config.yaml` now points its three groups at these real providers.
- **Profile resolution corrected (chosen over editing the test):** `duo_arm` is back to a
  planning-only profile (arm instances `launch_driver: false`, no `can_port`) — driver/controller
  bring-up moved to `start_both_arms_execution.launch.py`, keeping planning and execution separate.
  `test_resolve_duo_arm_profile_provides_planning_only_instances` passes again (4/4 in
  `test_execution_profiles.py`). **Behaviour change:** `start_multi_agx_arm_rviz` with `duo_arm` is
  now visualization/planning-only and no longer auto-launches the arm drivers (no crash —
  `resolve_arm_instances` defaults the missing `can_port`); bring drivers up via the execution launch.
- **Validated here (no ROS/hardware):** `agx_arm_mit_tools`, `agx_arm_mit_controller`,
  `agx_arm_ctrl` build clean via the system-python wrapper; the trajectory-split unit tests pass
  (3/3, `test_both_arms_trajectory_bridge.py`); `start_both_arms_execution.launch.py` generates
  cleanly (2 per-arm includes + 1 bridge node); `test_execution_profiles.py` 4/4.
- **Could NOT run here:** the live both_arms split against two real per-arm controllers, multi-cycle
  execution, and cancel/abort propagation on hardware — Duo-host bring-up steps.

## 2026-07-01 — coordinator converged onto the MoveIt slice (development host)

- **Single arm-execution slice.** The coordinator no longer owns a parallel both_arms controller.
  `arm_executor.plan()` now returns a `MoveGroupPlan` (anchor `to_pose`) or `RecordedTrajectoryPlan`
  (recorded `waypoints`); the coordinator dispatches `moveit_msgs/MoveGroup` (collision-aware plan +
  execute) or `moveit_msgs/ExecuteTrajectory`. MoveIt's `MoveItSimpleControllerManager` fans a
  both_arms plan out to the per-arm `/<side>_arm/arm_controller` servers natively — the fan-out was
  duplicated by the retired bridge. **Retired** (supersedes the 2026-06-30 entry):
  `both_arms_trajectory_bridge.py` + its test + entry point, and `start_both_arms_execution.launch.py`.
- **Config.** `arm_config.yaml` shrank to the group list + `move_group_action` /
  `execute_trajectory_action` + poses + defaults; planning group + joint names come from the registry.
  `agx_arm_coordination` package.xml: `control_msgs` → `moveit_msgs`.
- **Bring-up** is the one MoveIt slice: `start_agx_arm_components.launch.py
  execution_profile:=duo_arm mode:=moveit_mit` (per-arm MIT controllers + move_group together).
- **Validated here (no ROS/hardware):** coordination builds; 32 unit tests pass (`test_arm_executor`
  rewritten for the new plan types); the real `arm_config` yields a both_arms `MoveGroupPlan` with 14
  registry-derived joints; `coordinator_node` imports resolve (`moveit_msgs`). `test_pep257` stays
  red on the pre-existing package-wide D213 docstring style.
- **Could NOT run here:** live MoveGroup planning/execution against move_group + the per-arm
  controllers on the Duo; recorded `ExecuteTrajectory` replay (waypoints still placeholders).

## Hardware bring-up checklist (to fill on the Jetson/Duo)

### Step 1 — skill controller standalone (proposal §9 Step 1)

- [ ] `open_hand` reaches the open preset on both hands (mechanical limits OK).
- [ ] `grasp_glass_until_contact` / `grasp_bottle_until_contact` confirm contact
      and hold; record working `contact_sensors`, `contact_threshold`,
      `stable_samples`, `timeout_sec` per object below.
- [ ] grasp timeout fails cleanly; release returns to open; hold persists.
- [ ] passive slip warning/critical events fire at the configured factors.

### Step 2 — performer routing (proposal §9 Step 2)

- [ ] `hands_open_release_v1` completes via the coordinator (mock + sdk).
- [ ] structured failures propagate (e.g. force a stale-tactile fault).

### Step 3 — arm trajectories (proposal §9 Step 3)

- [ ] measure + record every anchor pose in `agx_arm_coordination/config/arm_config.yaml`.
- [ ] teach the four recorded trajectories and add their `waypoints`.
- [ ] confirm `both_arms_controller` / per-arm FJT action server names.

### Step 4 — mini graphs (proposal §9 Step 4)

- [ ] `hands_open_close_release_v1`, `both_arms_pregrasp_grasp_retract_v1`,
      `both_arms_lift_pour_return_v1` run; resource conflicts + child-failure
      abort verified.

### Step 5 — full demo escalation (proposal §9 Step 5)

- [ ] no objects → dummy → empty → water → real Hefeweizen.

## Measured values (fill on hardware)

| Parameter | Object | Value | Notes |
|---|---|---|---|
| contact_sensors | glass | _tbd_ | |
| contact_threshold | glass | _tbd_ | raw normal-force units |
| stable_samples | glass | _tbd_ | |
| contact_sensors | bottle | _tbd_ | |
| contact_threshold | bottle | _tbd_ | |
| open preset | both | _tbd_ | replace `zero` with a flat-palm `open` |
| glass grasp preset | left | _tbd_ | replace `fist_vendor_demo` |
| bottle grasp preset | right | _tbd_ | replace `fist_vendor_demo` |
| pour angle / duration | — | _tbd_ | low-risk first demo |
