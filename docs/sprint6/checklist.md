# Sprint 6 — Checklist

Build order from `planning/architecture_and_repo_integration.md` §9 (decisions: YAML graph,
PerformAction-for-hand-skills, coordinator-internal performer, package `agx_arm_coordination`).
Decisions and their rationale: `planning/decision_record.md`.

> **Resumed on the V02 contracts, and the demo has run (2026-08-17).**
> `tea_pour_left_v1` completed end to end on hardware **three times** across two
> bring-ups — 16:49, 17:02 and 17:12 — with one further run cancelled two nodes
> from the end and one aborted before the fix that enabled the rest. Full evidence
> in [`evidence/tea_pour_left_v1_2026-08-17.md`](evidence/tea_pour_left_v1_2026-08-17.md).
> The re-teach this sprint was waiting for turned out not to be needed: the taught
> data ran unchanged against the new command contracts.
>
> **Read the remaining unchecked items individually.** They are no longer one
> category. Some are still hardware or calibration work (the Hefeweizen demo,
> tactile thresholds, the payload mass); some are architecture or runtime
> resilience that a successful supervised demo does not touch (coordinator crash
> containment, the stop ladder mid-motion). Do not read a checked demo as
> covering either.
>
> Decisions and rationale: `planning/decision_record.md`; what the refactor
> changed underneath this sprint is §6 there.

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
- [x] **run `tea_pour_left_v1` on hardware — done 2026-08-17, three complete runs.** 90.0 s,
	92.7 s and 93.8 s across two independent bring-ups; the last two ran back to back in one stack
	without a restart or an operator step, and within a stack no action differed by more than 0.9 s.
	Both payload transitions fired in every completed run; 19 hand claim/release pairs against 19
	skill-controller performs, so no hand action skipped its claim; the two arms came up on their
	different protocol tiers and each fitted its own torque envelope. Every arm motion went through
	`FollowJointTrajectory`, 16 goals per run, none rejected or replanned. Commit `31c0350`,
	profile `duo_hand_external_bridge`.
	- The **first live attempt aborted** (16:41) with MoveIt `CONTROL_FAILED`, because the MIT
	  controller's own claim bumped the device epoch and its authority callback aborted the goal it
	  had just accepted. Fixed by `31c0350` four minutes later; the first success came eight
	  minutes after that. The failure appears before that commit and never after it.
	- A **fourth live run was cancelled** two nodes from the end (16:58). See Step 6.
	- Three dry runs preceded the live ones; the first found `payload_attach[left]: service
	  unavailable` before any hardware was at risk.
	Evidence, including the five anomalies that did *not* fail a run:
	`evidence/tea_pour_left_v1_2026-08-17.md`.
	- The escalation ladder (no teapot → empty → water) is not separately recorded; the logs do
	  not say what the teapot held, so treat the object state as unrecorded rather than assumed.
	- Two taught replays dominate the runtime: `left_arm_pour_tea` at 21.3 s and
	  `left_arm_teapot_handle_release` at 14.3 s, 38 % of the activity between them. Both are the
	  recorded segment's own declared duration, so shortening the demo means re-timing the
	  recordings, not the runtime.
- [ ] run `hefeweizen_pour_v1` on the escalation ladder: no objects → dummy → empty → water → beer.
	*(graph + `start_hefeweizen_demo.launch.py` ready; hardware-pending. The tea result does not
	transfer — Hefeweizen is dual-arm and tactile-gated, and needs a calibrated
	`contact_threshold` first.)*

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
- [x] L3 first `tea_pour_left_v1` with the payload active — **done 2026-08-17**, and twice.
	**Exactly two transitions per run**, in the intended places and order: attach after
	`left_hand_grip_handle` (action 70), detach after `left_hand_release_handle` (action 150).
	Each applied in ~0.51 s and each named the gravity model it switched to, so the loaded model
	was active for the whole carried-and-poured section and the base model for the rest. No
	torque-limit rejection, no authority refusal, no aborted goal anywhere in either run.
	*Sag and jump are operator-observed only — nothing sampled `~/gravity_feedforward` or joint
	effort during the run, so "no visible sag" is what the operator saw, not a measurement.*
- [ ] Measure whether 1.0 kg at `[0.15, 0.0, 0.0]` is actually right. The run proves the
	transition mechanism works on hardware; it says nothing about the number, which is still an
	unmeasured estimate. This is what the static check above is for.
- [x] `planning/hefeweizen_validation_log.md` created to capture runs (dev slice logged).

## Step 6 — Stop / interrupt safety

- [x] `Ctrl+C` on the coordinator no longer strands a running MoveIt goal: the node takes SIGINT
	itself, the activity unwinds (cancel children → reopen hand windows → `cancel_trajectory` +
	`hold_current` on the moving side), and a second interrupt escalates to `emergency_stop`.
- [x] `Ctrl+C` on `run_activity` cancels the activity goal and waits for the result instead of
	exiting while the robot keeps moving.
- [x] the activity loop no longer reports **success** when it exits because rclpy went down with
	nodes still pending.
- [x] `Ctrl+C` on an **idle** coordinator exits instead of spinning. Hardware, 2026-08-17: the
	interrupt on the final stack arrived with nothing running, the coordinator logged
	`stop requested (interrupt (Ctrl+C)); no activity running`, and all five tea-stack processes
	were reported finished cleanly within 0.66 s — the coordinator itself in 0.28 s. This closes
	the Phase-0B finding that an idle coordinator survived the first SIGINT.
- [x] **cancel an activity in flight.** Hardware, 2026-08-17 16:58: a run was cancelled at action
	150 of 170, two nodes from the end. The activity terminated cleanly and both launches then
	shut down normally. Read the limit with it — see the next item.
- [ ] verify the stop ladder on hardware, **mid-replay and mid-hand-window**.
	*(still hardware-pending. The 16:58 cancel landed in the ~1 s window between the hand child
	finishing and the coordinator recording it complete, so **nothing was in flight**: no arm goal
	was open, the hand goal had already succeeded, and `_cancel_children` had no moving child to
	stop. Cancelling an activity and cancelling a moving arm are different claims, and only the
	first has hardware evidence.)*
- [ ] **make a cancelled activity visible in the log.** Found by reconstructing the 16:58 run:
	`_abort()` logs `ERROR: aborting '<activity>': <reason>`, but the cancel branch logs nothing
	at all — it emits a `failed` event on `~/events`, sets the action result, and returns. A
	cancelled run therefore leaves a log that stops mid-dispatch with no terminal line, which is
	indistinguishable at a glance from a coordinator that hung. Establishing which had happened
	required reading the source, not the log.
- [ ] **decide what a cancelled activity should leave the payload state as.** The 16:58 cancel
	arrived before the detach transition, so the activity ended with the loaded 1 kg gravity model
	still active on a hand that had physically released the teapot. `tea_demo.md` documents this
	as the deliberate safer approximation (over- rather than under-compensating), and the attach
	is idempotent so the next run recovers — but this is its first hardware occurrence and it is
	worth confirming that is still the intended answer.
- [ ] coordinator **crash** (as opposed to interrupt) is still uncovered — the MoveIt goal survives
	the process. Needs either a MoveIt-side watchdog or a supervisor.
	*(Known runtime-resilience limitation, not a demo gap. It does not make the 2026-08-17 run
	incomplete: that run was supervised, and a graceful interrupt exercises a different path from
	an abrupt process death.)*

## Repo hygiene

- [x] record `agx_arm_coordination` in `docs/project/repository_structure.md`, `.claude/rules/`,
	  and the `.github/instructions/` mirror.
- [x] keep the OmniHand bridge + skill controller in `agx_arm_ctrl` (no premature package split).