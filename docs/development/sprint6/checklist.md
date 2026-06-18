# Sprint 6 — Checklist

Build order from `planning/architecture_and_repo_integration.md` §9 (decisions: YAML graph,
PerformAction-for-hand-skills, coordinator-internal performer, package `agx_arm_coordination`).

## Planning

- [x] Adapt the cetibar coordinator pattern to this repo (architecture & repo integration doc).
- [x] Resolve MVP architecture decisions (storage, hand-skill transport, performer, package, tokens).
- [x] Define the semantic hand-skill set and backend mapping (`hand_skill_backend_mapping.md`).
- [x] Define the canonical `hefeweizen_pour_v1` graph + catalogue (`hefeweizen_activity_graph.md`).

## Step 1 — OmniHand skill controller (standalone)

- [ ] `omnihand_skill_controller` node in `src/agx_arm_ctrl` (one per side).
- [ ] `skill_name → backend` map (open / glass grasp / bottle grasp / release / stop).
- [ ] tactile-confirmed close: `contact_score`, threshold, `stable_samples`, timeout.
- [ ] state machine `IDLE/OPENING/CLOSING_UNTIL_CONTACT/GRASP_HOLDING/RELEASING/FAILED`.
- [ ] `completion_policy` / `fallback_policy` handling; internal hold; passive slip monitoring.
- [ ] standalone validation (no coordinator) against proposal §9 Step 1 success criteria.

## Step 2 — Performer routing + messages

- [ ] `agx_arm_msgs`: `PerformActivity.action`, `PerformAction.action`, `RobotEvent.msg`.
- [ ] coordinator-internal performer routes `Gripper+{left,right}_hand` → skill controller,
      `Trajectory+both_arms` → existing both_arms FJT/MIT path.
- [ ] hand-skill metadata (skill_name, contact_*, *_policy) carried on `PerformAction`.
- [ ] result/feedback propagate; failures are structured.

## Step 3 — Combined arm trajectories

- [ ] record/plan the six `both_arms_*` catalogue trajectories.
- [ ] each executes independently; correct joint ordering; multiple cycles without launch restart;
      slow scaling respected; safe cancel/stop.

## Step 4 — Coordinator + mini graphs

- [ ] `src/agx_arm_coordination`: coordinator node, `~/execute_activity`, YAML graph/catalogue
      loader behind `get_activity_plan`/`validate_activity`/`get_action_detail`.
- [ ] resource model + `sync_flag` barrier groups; `R_BOTH_ARMS` conflicts with both per-arm tokens.
- [ ] run `hands_open_close_release_v1`, `both_arms_pregrasp_grasp_retract_v1`,
      `both_arms_lift_pour_return_v1`; parallel hand actions where allowed; child failure aborts.

## Step 5 — Full demo

- [ ] run `hefeweizen_pour_v1` on the escalation ladder: no objects → dummy → empty → water → beer.
- [ ] capture runs in `planning/hefeweizen_validation_log.md`.

## Repo hygiene

- [ ] record `agx_arm_coordination` in `docs/project/repository_structure.md` + `.claude/rules/`.
- [ ] keep the OmniHand bridge + skill controller in `agx_arm_ctrl` (no premature package split).
