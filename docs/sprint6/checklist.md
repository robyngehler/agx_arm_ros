# Sprint 6 — Checklist

Build order from `planning/architecture_and_repo_integration.md` §9 (decisions: YAML graph,
PerformAction-for-hand-skills, coordinator-internal performer, package `agx_arm_coordination`).
Decisions and their rationale: `planning/decision_record.md`.

> **Resuming on the V02 contracts (2026-08-17).** Every unchecked item below is a
> *hardware-pending* item, and the runtime under it changed while the sprint was
> paused — hand commands now carry authority, an unclaimed hand executes nothing,
> and the hand window is off by default on the four-bus topology. Read
> `planning/decision_record.md` §6 before ticking anything. The taught data
> predates the new command contracts, so a re-teach comes first.

## Planning

- [x] Adapt the cetibar coordinator pattern to this repo (architecture & repo integration doc).
- [x] Resolve MVP architecture decisions (storage, hand-skill transport, performer, package, tokens).
- [x] Define the semantic hand-skill set and backend mapping (`hand_skill_backend_mapping.md`).
- [x] Define the canonical `hefeweizen_pour_v1` graph + catalogue (`hefeweizen_activity_graph.md`).

## Step 1 — OmniHand skill controller (standalone)

- [x] `omnihand_skill_controller` node in `src/agx_arm_ctrl` (one per side).
- [x] `skill_name → backend` map (open / glass grasp / bottle grasp / release / stop)
	in `config/omnihand_skills.yaml` + `agx_arm_ctrl/omnihand/skills.py`.
- [x] tactile-confirmed close: `contact_score`, threshold, `stable_samples`, timeout.
- [x] state machine `IDLE/OPENING/CLOSING_UNTIL_CONTACT/GRASP_HOLDING/RELEASING/FAILED`.
- [x] `completion_policy` / `fallback_policy` handling; internal hold; passive slip monitoring.
- [ ] standalone validation (no coordinator) against proposal §9 Step 1 success criteria.
	*(hardware-pending — see `planning/hefeweizen_validation_log.md`; presets/thresholds
	are placeholders until measured on the Pro hand.)*

## Step 2 — Performer routing + messages

- [x] `agx_arm_msgs`: `PerformActivity.action`, `PerformAction.action`, `RobotEvent.msg`.
- [x] coordinator-internal performer routes `Gripper+{left,right}_hand` → skill controller,
	  `Trajectory+both_arms` → existing both_arms FJT path (`performer.py` + coordinator dispatch).
- [x] hand-skill metadata (skill_name, contact_*, *_policy) carried on `PerformAction.metadata_json`.
- [x] result/feedback propagate; failures are structured (`_HandChild`/`_ArmChild`, abort path).

## Step 3 — Combined arm trajectories

- [x] catalogue + arm executor for the `both_arms_*` trajectories (anchor-pose endpoints now;
	recorded waypoints teach-on-hardware via `arm_executor.py` + `config/arm_config.yaml`).
- [x] teach tooling for anchor poses: `agx_arm_capture_anchor_pose` (`agx_arm_mit_demos`) snapshots
	the live joint vector into `arm_config.yaml`, reusing the leader recorder + performer.
	Right-side runbook in `../../control/bringups/teach_and_run.md`.
- [x] central teach tool `agx_arm_teach_manager` (`agx_arm_mit_demos`): one keyboard UI for
	freedrive/record/playback + anchor capture (`a`) + recorded→catalogue waypoint conversion
	(`w`), modelled on the wakeword motion manager. Standalone converter:
	`agx_arm_recorded_to_catalogue` (downsamples a recording to a paste-ready `waypoints:` block).
- [x] OmniHand MoveIt description migrated O10 → **O12 Pro** (vendor URDF) so `components.launch`
	no longer errors on missing `*_mcp_joint`; SRDF/controllers/initial_positions/ros2_control
	all on the 12 active joints (offline SRDF-vs-URDF cross-check clean).
- [x] `both_arms` execution converged onto the MoveIt slice: the coordinator dispatches anchor moves
	via `moveit_msgs/MoveGroup` (collision-aware) and recorded replay via `ExecuteTrajectory`;
	MoveIt fans a both_arms plan out to the per-arm controllers natively. The earlier parallel
	fan-out bridge + `start_both_arms_execution.launch.py` were retired. Bring-up = components
	`execution_profile:=duo_arm mode:=moveit_mit` (per-arm controllers + move_group together).
- [ ] each executes independently; correct joint ordering; multiple cycles without launch restart;
	slow scaling respected; safe cancel/stop. *(hardware-pending — anchor poses + recorded
	waypoints are placeholders; confirm live MoveGroup planning + the native fan-out on the Duo.)*

## Step 4 — Coordinator + mini graphs

- [x] `src/agx_arm_coordination`: coordinator node, `execute_activity`, YAML graph/catalogue
	loader behind `get_activity_plan`/`validate_activity`/`get_action_detail`.
- [x] resource model + `sync_flag` barrier groups; `R_BOTH_ARMS` conflicts with both per-arm
	tokens (validated on the full graph: drains, sync pairs together, no batch resource clash).
- [ ] run `hands_open_close_release_v1`, `both_arms_pregrasp_grasp_retract_v1`,
	`both_arms_lift_pour_return_v1`; parallel hand actions where allowed; child failure aborts.
	*(graphs + `arm_dry_run` runnable; live runs hardware-pending.)*

## Step 5 — Full demo

- [x] **first MVP demo assembled: `tea_pour_left_v1`** (2026-07-28) — left arm + left hand pour a
	teapot; both sides brought up, only the left addressed. 17-node linear graph over 8 anchor
	moves, 3 taught replays (`Grip_Can_L`, `CanFill02`, `Can_Release_L_V02`) and 5 hand poses.
	Runbook: `../../control/bringups/tea_demo.md`. Chosen ahead of the Hefeweizen MVP as the
	simpler single-side first test.
	- new `pose` hand motion (deterministic ramp to a taught preset, no tactile gating) — the
	  Hefeweizen `close_until_contact` path stays available but needs a calibrated threshold.
	- recorded replay now dispatches as a planned `MoveGroup` approach to waypoint 0 **then**
	  `ExecuteTrajectory`, so a taught segment no longer has to start bit-exact on its anchor.
	- `trajectory_execution.allowed_start_tolerance` raised 0.01 → 0.05 (all three recordings
	  exceeded the MoveIt default against their anchors).
	- catalogue may now be split across `config/catalogue.d/*.yaml` fragments.
- [ ] run `tea_pour_left_v1` on hardware, escalation ladder: no teapot → empty → water.
	*(hardware-pending — nothing in this chain has been executed live.)*
- [ ] run `hefeweizen_pour_v1` on the escalation ladder: no objects → dummy → empty → water → beer.
	*(graph + `start_hefeweizen_demo.launch.py` ready; hardware-pending.)*

## Step 5b — Dynamic payload adjustment (2026-08-17)

Decisions and the flange-axis correction in `planning/decision_record.md` §4;
evidence in `reference/payload_gravity_model.md`.

- [x] MIT controller preloads a base and a loaded gravity model; `~/payload_attached`
	(`std_srvs/SetBool`) swaps the active reference under `state_lock`. Idempotent,
	non-motion-generating, and refused when no loaded model exists.
- [x] `gravity_launch_utils.derive_fixed_payload_urdf` appends one fixed payload link
	to the already-resolved gravity URDF; parent link resolved from the URDF
	(`*nero_tool0`, narrowed by joint prefix), never guessed.
- [x] action-level `payload_update: attach|detach` in the catalogue, validated at load
	so a typo fails before the robot moves. Deliberately **not** derived from the hand
	preset: `pre_grip` and `release` both run `can_pre_grip`.
- [x] coordinator applies the transition after a child succeeds and **before** the node
	counts as completed; a failed transition aborts the activity.
- [x] `tea_pour_left_v1`: attach on action 70, detach on action 150.
- [x] payload launch arguments forwarded through the production `moveit_mit` bringup.
	*(Not through the RViz debug launches — deliberate.)*
- [ ] L3 static payload check: hold the grip pose, toggle `payload_attached`, confirm
	`~/gravity_feedforward` moves in the expected direction and the arm does not sag.
	*(hardware-pending; mass 1.0 kg and the 0.15 m lever are unmeasured estimates.)*
- [ ] L3 first `tea_pour_left_v1` with the payload active: exactly two transitions, no
	visible sag after lift, no jump at attach/detach, no torque-limit rejection.
	*(hardware-pending.)*
- [x] `planning/hefeweizen_validation_log.md` created to capture runs (dev slice logged).

## Step 6 — Stop / interrupt safety

- [x] `Ctrl+C` on the coordinator no longer strands a running MoveIt goal: the node takes SIGINT
	itself, the activity unwinds (cancel children → reopen hand windows → `cancel_trajectory` +
	`hold_current` on the moving side), and a second interrupt escalates to `emergency_stop`.
- [x] `Ctrl+C` on `run_activity` cancels the activity goal and waits for the result instead of
	exiting while the robot keeps moving.
- [x] the activity loop no longer reports **success** when it exits because rclpy went down with
	nodes still pending.
- [ ] verify the stop ladder on hardware, mid-replay and mid-hand-window.
	*(hardware-pending; unit-tested only.)*
- [ ] coordinator **crash** (as opposed to interrupt) is still uncovered — the MoveIt goal survives
	the process. Needs either a MoveIt-side watchdog or a supervisor.

## Repo hygiene

- [x] record `agx_arm_coordination` in `docs/project/repository_structure.md`, `.claude/rules/`,
	  and the `.github/instructions/` mirror.
- [x] keep the OmniHand bridge + skill controller in `agx_arm_ctrl` (no premature package split).