# Sprint 6 — Open Questions

## Resolved (MVP architecture — see planning/architecture_and_repo_integration.md §8)

- Activity-graph storage → **YAML for the MVP**, behind the same service contract; DB later.
- Hand-skill transport → **`PerformAction` with metadata**, no dedicated `HandSkill.action` yet.
- Performer → **coordinator-internal** router for the MVP.
- Orchestration package → **`agx_arm_coordination`**.
- CAN-bus resource tokens → **deferred** until contention is observed.

## Opened by the tea demo (2026-07-28)

- Anchor `Can_Grip_L` sits ~0.22 rad (j5) / 0.20 rad (j7) past the end of the `Grip_Can_L`
	recording. Confirmed intentional — that twist seats the hand in the teapot handle — but it has
	not been re-verified since the recording was taught. Re-capture if the seat looks wrong.
- Is `allowed_start_tolerance` 0.05 rad enough for the MIT controller's standing error under the
	teapot payload, or does it need to go higher? Too low aborts the replay before it moves (safe
	failure); too high lets a replay start further from its taught path than intended.
- Is the `pose` hand motion good enough for the demo, or does the grip need tactile confirmation?
	`pose` is deterministic but blind — it closes on empty air if the handle is not where the anchor
	says. Blocked on a calibrated `contact_threshold` (the 0.35 placeholder is orders of magnitude
	below the Pro's raw normal-force values).
- What is the real CPU headroom during the demo? The lowered hand rates in `start_tea_demo.launch.py`
	are reasoned, not measured, against `sprint_refactor/reference/critical_cpu_paths.md`.
- How should a coordinator **crash** be covered? The interrupt path is handled, but a hard crash
	leaves the MoveIt goal executing. Candidates: a MoveIt-side execution watchdog, or a supervisor
	that pins the arms when the coordinator disappears.

## Needs hardware validation / calibration

- Which backend gestures/presets work best for the glass and the bottle grasp?
- Which tactile sensors are reliable for stable contact per object?
- Robust `contact_threshold` and `stable_samples` across repeated grasps?
- Does arm + hand on one native side bus stay stable during sustained coordinated motion
	(depends on sprint-5 bus-load validation)?
- Should `one-shot` CAN mode stay on for the hand under all demo conditions, or off so hand
	commands retransmit? (sprint-5 caveat)
- Safest fallback if a hand loses contact during the pour (warn vs abort)?
- Pour angle and duration for a visually successful but low-risk first demo?

## Design questions settled during implementation (2026-06-29)

- `contact_score` aggregation → **configurable, default `mean`** over the matched `contact_sensors`
	(`mean | max | min`), set via `defaults.contact_aggregation` in `config/omnihand_skills.yaml` or
	per-action `metadata.contact_aggregation`. `min` ("all sensors must touch") is available for a
	stricter grasp once calibrated.
- `both_arms` executor → **dispatches through the MoveIt multi-arm slice** (updated 2026-07-01,
	supersedes the earlier "thin FJT adapter" decision): `arm_executor.ArmTrajectoryPlanner` builds a
	`MoveGroupPlan` (anchor `to_pose`) or `RecordedTrajectoryPlan` (recorded `waypoints`); the
	coordinator sends `moveit_msgs/MoveGroup` (collision-aware plan + execute) or
	`moveit_msgs/ExecuteTrajectory`. MoveIt fans a both_arms plan out to the per-arm controllers
	natively, so there is no second arm-execution path (the fan-out bridge was retired).
- Event schema → **one shared `RobotEvent`** for the coordinator and every executor (skill
	controller, arm path), streamed on each node's `~/events`.

### Still open / deferred

- **Resolved (2026-07-01):** collision-aware anchor-to-anchor planning now happens via MoveGroup.
- Recorded replay uses `ExecuteTrajectory` (executes the taught trajectory as-is through MoveIt's
	controller manager; the path itself is not re-collision-checked). Full MoveIt Cartesian/retime of
	recorded joint trajectories is a later refinement — not exercisable until waypoints are taught.