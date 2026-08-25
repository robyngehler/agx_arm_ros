# Sprint 6 — Open Questions

## Resolved (MVP architecture — see planning/architecture_and_repo_integration.md §8)

- Activity-graph storage → **YAML for the MVP**, behind the same service contract; DB later.
- Hand-skill transport → **`PerformAction` with metadata**, no dedicated `HandSkill.action` yet.
- Performer → **coordinator-internal** router for the MVP.
- Orchestration package → **`agx_arm_coordination`**.
- CAN-bus resource tokens → **deferred** until contention is observed.
	*Superseded 2026-08-17.* Contention was observed, which produced the shared-bus token and
	step-and-settle; the four-bus topology then removed the contention. The resource relation is
	now **derived from `bus_topology`** rather than fixed: under `dedicated_per_device` a side's
	arm and hand hold independent tokens and may run concurrently, under `shared_per_side` they
	share one, and an unrecognised value reads as shared. Validation and scheduling take the same
	table, so they cannot disagree about the same machine.

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
- How should a coordinator **crash** be covered? The interrupt path is handled, but a hard crash
	leaves the MoveIt goal executing. Candidates: a MoveIt-side execution watchdog, or a supervisor
	that pins the arms when the coordinator disappears.

## Closed by the V02 refactor (2026-08-17)

- **Does arm + hand on one native side bus stay stable during sustained coordinated
	motion?** Moot. Each device owns its own interface (C1). Same-side arm and hand
	motion was run in parallel on both sides and both sides at once with zero errors
	and zero drops on every bus. The shared side bus survives only as the selectable
	`shared_per_side` topology.
- **Should `one-shot` stay on for the hand?** It was a shared-bus arbitration
	question. On a dedicated hand adapter nothing else transmits, so the setting no
	longer trades hand delivery against arm retransmission buildup.
- **What is the real CPU headroom during the demo?** Measured, and the answer moved
	twice: the stack fell from 814.5 % of a core to 399.7 % idle and from 882.9 % to
	431.3 % under dual MIT at 100 Hz, after a pub-rate fix and a vendor receive-loop
	patch. The lowered hand rates in `start_tea_demo.launch.py` are now redundant
	rather than wrong — `pub_rate` is a ceiling that cannot drive publication.

## Needs hardware validation / calibration

- Which backend gestures/presets work best for the glass and the bottle grasp?
- Which tactile sensors are reliable for stable contact per object?
- Robust `contact_threshold` and `stable_samples` across repeated grasps? The 0.35
	placeholder is orders of magnitude below the Pro's raw normal-force values, and it
	blocks `close_until_contact` for the demo.
- Safest fallback if a hand loses contact during the pour (warn vs abort)?
- Pour angle and duration for a visually successful but low-risk first demo?
- Does the payload gravity swap behave? Mass 1.0 kg and the 0.15 m lever are
	unmeasured estimates (`planning/decision_record.md` §4).

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
- **Partly addressed 2026-08-25.** "As-is" was doing more damage than the phrasing suggests: the
	dispatch emitted positions and times only, and the MIT trajectory buffer reads a missing velocity
	as a *commanded zero*, so the kd term braked against the position command (`|v_des - dp/dt|` 0.224
	rad/s against the teach path's 0.004). It now supplies central differences, and catalogue waypoints
	are selected by chord error rather than by even sample index.
- **Still open: the activity path has no retiming modes and no access to the taught density.** The
	teach loop gained `as_recorded`/`smooth`/`tempo_scale`/`speed_scale`/`maximize_speed` over a
	uniformly resampled trajectory; a catalogue action has only `velocity_scaling` over an inlined
	~10:1 decimation. No downstream retiming recovers what decimation removed. The shape of the fix is
	for a catalogue action to *reference* its recording rather than inline waypoints, which would give
	an assembled activity every mode the teach path has. See
	`docs/sprint_refactor/reference/teach_replay_timebase.md`.