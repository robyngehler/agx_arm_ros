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
