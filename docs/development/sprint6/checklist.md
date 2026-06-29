# Sprint 6 — Checklist

Build order from `planning/architecture_and_repo_integration.md` §9 (decisions: YAML graph,
PerformAction-for-hand-skills, coordinator-internal performer, package `agx_arm_coordination`).

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
      Right-side runbook in `planning/teach_and_run_right_side.md`.
- [x] OmniHand MoveIt description migrated O10 → **O12 Pro** (vendor URDF) so `components.launch`
      no longer errors on missing `*_mcp_joint`; SRDF/controllers/initial_positions/ros2_control
      all on the 12 active joints (offline SRDF-vs-URDF cross-check clean).
- [ ] each executes independently; correct joint ordering; multiple cycles without launch restart;
      slow scaling respected; safe cancel/stop. *(hardware-pending — anchor poses + recorded
      waypoints are placeholders; confirm controller/action-server names on the Duo runtime.)*

## Step 4 — Coordinator + mini graphs

- [x] `src/agx_arm_coordination`: coordinator node, `execute_activity`, YAML graph/catalogue
      loader behind `get_activity_plan`/`validate_activity`/`get_action_detail`.
- [x] resource model + `sync_flag` barrier groups; `R_BOTH_ARMS` conflicts with both per-arm
      tokens (validated on the full graph: drains, sync pairs together, no batch resource clash).
- [ ] run `hands_open_close_release_v1`, `both_arms_pregrasp_grasp_retract_v1`,
      `both_arms_lift_pour_return_v1`; parallel hand actions where allowed; child failure aborts.
      *(graphs + `arm_dry_run` runnable; live runs hardware-pending.)*

## Step 5 — Full demo

- [ ] run `hefeweizen_pour_v1` on the escalation ladder: no objects → dummy → empty → water → beer.
      *(graph + `start_hefeweizen_demo.launch.py` ready; hardware-pending.)*
- [x] `planning/hefeweizen_validation_log.md` created to capture runs (dev slice logged).

## Repo hygiene

- [x] record `agx_arm_coordination` in `docs/project/repository_structure.md`, `.claude/rules/`,
      and the `.github/instructions/` mirror.
- [x] keep the OmniHand bridge + skill controller in `agx_arm_ctrl` (no premature package split).
