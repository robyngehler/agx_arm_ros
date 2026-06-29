# Sprint 6 — Open Questions

## Resolved (MVP architecture — see planning/architecture_and_repo_integration.md §8)

- Activity-graph storage → **YAML for the MVP**, behind the same service contract; DB later.
- Hand-skill transport → **`PerformAction` with metadata**, no dedicated `HandSkill.action` yet.
- Performer → **coordinator-internal** router for the MVP.
- Orchestration package → **`agx_arm_coordination`**.
- CAN-bus resource tokens → **deferred** until contention is observed.

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
- `both_arms` executor → **reuses the existing FollowJointTrajectory path via a thin adapter**
  (`arm_executor.ArmTrajectoryPlanner`): catalogue metadata (anchor `to_pose` or recorded
  `waypoints`, `velocity_scaling`) is turned into the FJT goal; no second arm execution path.
- Event schema → **one shared `RobotEvent`** for the coordinator and every executor (skill
  controller, arm path), streamed on each node's `~/events`.

### Still open / deferred

- MoveIt collision-aware planning between anchor poses (the MVP commands the endpoint joint vector
  and lets the controller interpolate; collision-aware anchor-to-anchor planning is a later slice).
