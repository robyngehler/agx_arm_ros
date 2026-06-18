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

## Design questions to settle during implementation

- `contact_score` aggregation: mean vs max vs per-sensor min over `contact_sensors`?
- Does the `both_arms` executor reuse the existing FJT action as-is, or does the performer need a
  thin adapter for catalogue metadata (velocity/acceleration scaling)?
- Event schema: reuse one `RobotEvent` for coordinator + executors, or per-layer event types?
