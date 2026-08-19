# Sprint 6 — Decision Record

status: CONSOLIDATED
covers: 2026-06-24 … 2026-08-17
sprint state: **resuming** against the V02 refactor contracts

Sprint 6 built the coordination layer: an Activity-DAG coordinator, a semantic
hand-skill controller, the teach toolchain, and the first end-to-end demo. It was
paused mid-flight for the V02 runtime refactor, which changed several of the
assumptions underneath it. This file records what was decided and why, marks what
the refactor invalidated, and says what the sprint needs on resumption.

It consolidates six planning documents whose decisions had landed: the two
step-and-settle documents, the dual-hand MoveIt gap analysis, the dynamic payload
proposal, and the session handoff. Their content is here; the originals are in
git history.

## What is still live in `planning/`

| File | Why it stays |
| --- | --- |
| `architecture_and_repo_integration.md` | The coordinator's architecture decisions (§8) and build order (§9); the checklist indexes into it |
| `hefeweizen_pour_proposal.md` | The demo specification and its escalation ladder — not yet run |
| `hefeweizen_activity_graph.md` | The canonical graph and catalogue for that demo |
| `hand_skill_backend_mapping.md` | The semantic skill → backend design the skill controller implements |
| `omnihand_gesture_mapping.md` | Gesture inventory and preset selection, still uncalibrated |
| `gravity_payload_api_plan.md` | The **general** payload service is still wanted and unbuilt; the MVP below is narrower |
| `hefeweizen_validation_log.md` | The running record of what has actually been validated where |

Raw session captures live in `../evidence/`.

---

# 1. Coordination architecture

The MVP decisions, all taken 2026-06-29 and all still in force
(`architecture_and_repo_integration.md` §8):

| Question | Decision | Why |
| --- | --- | --- |
| Activity-graph storage | YAML, behind the service contract | A DB adds an operational dependency before the graph shape is settled. The contract (`get_activity_plan` / `get_action_detail` / `validate_activity`) is the same either way, so the swap is later and cheap |
| Hand-skill transport | `PerformAction` with `metadata_json` | A dedicated `HandSkill.action` would freeze a skill vocabulary that is still being discovered |
| Performer | Coordinator-internal router | One process, one activity; a separate performer node buys nothing at this scale |
| Package | `agx_arm_coordination` | Orchestration is not arm runtime; keeping it out of `agx_arm_ctrl` keeps the bridge's boundary clean |
| CAN-bus resource tokens | Deferred until contention is observed | Contention *was* then observed, which produced step-and-settle (§2), which the four-bus topology then removed |
| `both_arms` execution | Dispatch through the MoveIt multi-arm slice | Supersedes the earlier "thin FJT adapter" decision (2026-07-01). MoveIt fans a `both_arms` plan out to the per-arm controllers natively, so there is no second arm-execution path. The parallel fan-out bridge was retired |
| Event schema | One shared `RobotEvent` for coordinator and every executor | Streamed on each node's `~/events`; one schema means one consumer |
| `contact_score` aggregation | Configurable, default `mean` over the matched sensors | `min` ("all sensors must touch") is available for a stricter grasp once thresholds are calibrated |

Recorded replay uses `ExecuteTrajectory` — the taught trajectory executes as-is
through MoveIt's controller manager and the path itself is not re-collision-checked.
Anchor-to-anchor motion goes through `MoveGroup` and *is* collision-aware. A
recorded segment is dispatched as a planned approach to waypoint 0 **then**
`ExecuteTrajectory`, so a taught segment no longer has to start bit-exact on its
anchor; `trajectory_execution.allowed_start_tolerance` was raised 0.01 → 0.05
because all three recordings exceeded the MoveIt default against their anchors.

---

# 2. Step-and-settle — dead as a model, alive as mechanisms

**What it was.** The arm and its hand shared one side CAN bus. The arm's *feedback
push* — not its command stream — saturated that bus at ~2150 frames/s while merely
holding, and the hand's high-ID CAN FD frames lost every arbitration under the
one-shot baseline. Step-and-settle serialized them: the arm settles into a verified
firmware hold, its feedback push is silenced, the hand acts, then the arm resumes.

**Why it is dead.** C1 of the refactor: each device now owns its own interface.
Same-side arm and hand motion runs in parallel, and `shared_per_side` is a
selectable degraded topology derived from one declared `bus_topology` value. The
scheduler no longer shares a bus token per side, and `prepare_hand_window` /
`resume_arm_control` survive only as that degraded mode's implementation.

**Why it still matters.** Much of the current safety machinery was built here and
is unchanged by the topology:

- **The Nero firmware has no MIT command watchdog.** It executes the last setpoint
  forever, so going silent leaves a *moving* arm. A runaway was observed live during
  a teach recording. This is why stale feedback streams a damped zero rather than
  pausing, and why a refusal on an active stream still needs a defined stop or hold.
- **Never resume a trajectory whose clock ran through an outage.** The stale-feedback
  abort used to leave `active_trajectory` and its monotonic clock armed, so when
  feedback returned the loop sampled a far-ahead point and snapped under MIT gains.
- **Recover on real loss, not local starvation.** Feedback went "stale" under GIL/CPU
  saturation while the bus was alive, triggering heavyweight reconnects mid-trajectory.
  Liveness is taken from the **kernel RX timestamp**, which is ground truth; the FPS
  window and the node clock both starve locally. This is Level 0 of the recovery
  ladder in `../../project/control_integrity_architecture.md`.
- **A stop command proves nothing; trust the observed effect.** `emergency_stop`
  logged success unconditionally, and under `ENOBUFS` the SDK drops the command
  silently — so a "stopped" arm could still be moving. It became a `Trigger` whose
  `success` is true only when the arm is confirmed stopped, and reports
  `fault_lockout=latched` when the last resort forced a recovery.
- **Recovery is a fault; clearing the lockout is a separate deliberate act.** No
  recovery path clears its own lockout — re-arming stays the initiator's decision.
- **`_assert_firmware_hold` exists because one MOVE-J frame is not enough.** On a
  saturated one-shot bus the single mode frame was dropped and the firmware stayed
  in MIT. The bounded re-assertion built here is now the *shared* safety hold used by
  the emergency stop and by pre-recovery teardown (refactor decision record §4).
- **`nero_can_push.py` exists because the stock APIs bundle "quiet bus" with a mode
  switch.** Only `set_leader_mode`/`set_follower_mode` silence the push, and leader
  mode is zero-force drag: with no firmware gravity model for this mounting pose the
  arm would sag. The repo-owned helper sends only the mode frame's push bit
  (`move_mode = 255` = no change), so the window gets a *quiet* arm rather than a
  *limp* one. The vendor SDK stays untouched under C3.
- **The window used to turn the flood back on by construction.** `prepare_hand_window`
  called `set_normal_mode()`, and the Nero driver's `set_normal_mode()` sets
  `enable_can_push = ENABLE`. It could not free the bus, and the diagnosis took two
  hardware runs.
- **A deliberate silence must never read as a stall.** The recovery watchdog treats a
  requested silence as healthy, bounded by `hand_window_max_silence_s`.
- **Concurrent grasp-and-carry was out of scope** for the MVP: the flow is strictly
  sequential and a held grasp is assumed to need no active connection. The four-bus
  topology removes the reason for that restriction; it has not been re-exercised.

**The finding that outlived the model.** Interface statistics over a full teach
session showed `arbit-lost` at 9–10 of ~2.5 M TX but **~108 k RX frames dropped per
bus**. The bottleneck the window could not fix was the kernel CAN **RX socket
buffer** overflowing during 200 ms+ publish-loop stalls under CPU starvation —
dropping the hand's *response* frames even inside an open window. That is host CPU,
not bus arbitration, and it is why the refactor's priority list put CPU relief
beside safety. **Parallel operation makes it more likely, not less.**

Still unresolved from it: `activate_duo_can.sh` raises `net.core.rmem_max`, but that
is only a ceiling an application must then request, and `python-can` never calls
`setsockopt(SO_RCVBUF)` — so the intended buffer raise never took effect. Tracked in
the refactor checklist under 1B.

---

# 3. Dual-arm plus both hands in MoveIt

**Decision.** `execution_profile:=duo_hand` (2026-07-15). Three gates closed:

1. the registry's `allowed_effector_types` extended to `[none, omnihand]`;
2. `omnihand_group` in `agx_arm.srdf.xacro` made per-side instantiable
   (`group_name`/`parent_group`/`parent_link`/`link_prefix`), with legacy defaults
   keeping the single-side profiles byte-identical;
3. `_moveit_config_builder` derives the per-side xacro args from `arm_instances`, so
   the duo slice no longer loses the intra-hand collision ACM.

**Why it mattered.** Without gate 2 the dual-arm branch silently dropped 2×325
intra-hand ACM entries — the planner would have been checking collisions the hand
cannot have and missing ones it can.

**Validation.** Offline (`validate_duo_hand.py`): no duplicate SRDF groups, per-side
eef parents, per-side flange pairs, and pinocchio+FCL on the full
both-arms-both-hands URDF showing 0 self-collisions at open / right fist / left fist /
both fists across 1626 checked pairs. **Live hardware validation of `duo_hand` closed
2026-08-11** during refactor Phase 2A: both bridges resolve to their own interface,
zero CAN FD timeouts, each device on its own bus.

The O10 → O12 Pro description migration belongs to the same slice: SRDF, controllers,
initial positions and `ros2_control` all moved to the 12 active joints, which is what
stopped `components.launch` erroring on missing `*_mcp_joint`.

---

# 4. Carried-payload gravity

**Decision (shipped 2026-08-17).** The MIT controller preloads **two** Pinocchio
models — base and loaded — and `~/payload_attached` (`std_srvs/SetBool`) swaps the
active reference under `state_lock`. Idempotent, non-motion-generating, and refused
when no loaded model exists. `gravity_launch_utils.derive_fixed_payload_urdf` appends
one fixed payload link to the already-resolved gravity URDF, with the parent link
**resolved from the URDF** (`*nero_tool0`, narrowed by joint prefix) rather than
guessed.

**Why a swap rather than the general runtime service.** The general
`SetGravityPayload.srv` — runtime mass/COM/tensor, `reference: flange|palm`
resolution, per-action payload parameters, clear-on-release tracking — is designed
and still wanted (`gravity_payload_api_plan.md`). The MVP hardcodes one payload per
bringup and knows only attached/detached, which is exactly what the tea demo needs
and nothing more.

**Why the payload transition is a property of the *action*, never of the hand
preset.** `pre_grip` and `release` both run the same `can_pre_grip` gesture, and only
one of them means the object is gone. A reusable gesture must not carry the task's
physical consequence. The catalogue declares `payload_update: attach|detach`,
validated at load so a typo fails before the robot moves; the coordinator applies the
transition after a child succeeds and **before** the node counts as completed, and a
failed transition aborts the activity.

**The correction worth keeping.** The proposal specified a flange **+z** offset. The
OmniHand mounts through a rotated flange joint and reaches along the tool0 frame's
**+x**, so the real offset is `[0.15, 0.0, 0.0]`. The deferred plan's own warning —
*verify the frame axis direction via FK on the generated URDF, do not guess* — was
the one instruction that mattered, and it was written before the mistake was made.

**Validated on hardware 2026-08-17**, in the mechanism only: both transitions fired
in both tea-demo runs, in the intended places, each applying in ~0.51 s and each
naming the gravity model it switched to. No torque-limit rejection followed either.

**Still unvalidated:** the mass. 1.0 kg at a 0.15 m lever remain estimates, and
nothing sampled `~/gravity_feedforward` or joint effort during the run — "no visible
sag" is an operator observation, not a measurement. The L3 static check (hold the
grip pose, toggle `payload_attached`, confirm the feedforward moves in the expected
direction) has not run, and it is what would settle the number.

---

# 5. Demo sequencing

**Decision.** `tea_pour_left_v1` was assembled first (2026-07-28) ahead of the
Hefeweizen MVP: one arm and one hand, 17 linear nodes over 8 anchor moves, 3 taught
replays and 5 hand poses. Both sides are brought up and only the left is addressed.

**Why.** A single-side linear graph exercises the whole chain — coordinator, MoveIt
dispatch, taught replay, hand skills, payload transitions — with one arm's worth of
failure modes. The Hefeweizen graph adds dual-arm synchronization on top of a chain
that had never run.

**Outcome, 2026-08-17.** The sequencing paid off: the chain ran end to end on the
first supervised session, twice in one stack, and the four anomalies the logs
recorded were all already-known open items rather than new defects
(`../evidence/tea_pour_left_v1_2026-08-17.md`). Two taught replays account for 38 %
of the 93 s runtime, which is the first concrete argument for re-timing recordings
rather than optimising the runtime.

**The `pose` hand motion** was added for it: a deterministic ramp to a taught preset
with no tactile gating. The `close_until_contact` path stays available but needs a
calibrated `contact_threshold` — the 0.35 placeholder is orders of magnitude below
the Pro's raw normal-force values. `pose` is deterministic but blind: it closes on
empty air if the handle is not where the anchor says.

**Naming.** Anchors and gestures were captured under `Can_*` names before the object
was settled. The object is a **teapot**; `Can_*` is a capture-time misnomer kept
verbatim wherever it names stored data so nothing has to be re-measured. The public
`action_id`s say teapot.

---

# 6. What the refactor changed underneath this sprint

Read before resuming.

| Sprint-6 assumption | Now |
| --- | --- |
| Same-side arm and hand contend for one bus; every hand action needs a window | Four buses; parallel operation is normal. The handshake defaults **off**, derived from `bus_topology` |
| A hand command is a bare `JointState` / `JointTrajectory` on a shared topic | A hand command carries `DeviceCommandStamp` inside `AuthorizedJointTrajectory` or `HandJointTarget`. The bare surfaces exist only under `allow_legacy_hand_command_ingress` (default false) |
| Anything may command a hand | Both primitives claim `control/omnihand/claim_device` first. **The bridge is fail-closed: an unclaimed hand executes nothing** |
| The skill controller republishes its grasp target at 20 Hz while holding | It monitors contact only. A confirmed grasp keeps the claim and sends nothing |
| The MIT controller stands down on `feedback/hand_window_active` | It consumes `AgxDeviceAuthority` and aborts on authority loss or any epoch change. The boolean is still published and nothing subscribes to it |
| `emergency_stop` on one arm stops that arm | A unit-safety generation stops every device; a device can still stop itself unilaterally if the writer is absent |
| The coordinator accepts every activity goal | One activity at a time, rejected at the door with a structured reason |
| A sync group is a barrier the scheduler tries to honour | A sync group is admitted whole or not at all, and a batch that cannot be merged aborts the activity rather than falling back to independent dispatch |
| Hand feedback publishes at the arm's `pub_rate` | Bringups pass `hand_pub_rate` / `hand_joint_read_rate`; publication is driven by new readbacks |

**Resumption happened on 2026-08-17, and the expected blocker did not
materialise.** A re-teach against the new command contracts was assumed to be the
first thing needed, because the taught data predates them. It was not: the
existing anchors and recordings ran unchanged, twice, and the activity completed
end to end (`../evidence/tea_pour_left_v1_2026-08-17.md`). The command contract
changed what crosses the bridge boundary, not what a taught pose means.

What remains, in order:

1. **Calibrate `contact_threshold` and `stable_samples`** on the Pro hand. The tea
   demo sidesteps this with the deterministic `pose` motion; the Hefeweizen grasps
   cannot.
2. **Measure the payload** (§4). The transition mechanism now has hardware
   evidence; the 1.0 kg and the 0.15 m lever still do not.
3. **The stop ladder on hardware**, mid-replay and mid-hand-window. The 2026-08-17
   interrupt landed on an idle stack and proves only the idle exit.
4. **Give a blocked hand pose an honest completion.** A closing gesture into a
   physical stop exhausts the bridge's 8-attempt verification every time — five
   occurrences in that one session. Harmless here, but it is the concrete case for
   distinguishing `commanded`, `delivery_verified` and `contact_confirmed`.

---

# 7. Still open from this sprint

- **A coordinator *crash* is uncovered.** The interrupt path unwinds; a hard crash
  leaves the MoveIt goal executing. Candidates: a MoveIt-side execution watchdog, or
  a supervisor that pins the arms when the coordinator disappears. Related but not
  the same as the external hardware watchdog in
  `../../project/control_integrity_architecture.md`.
- **Anchor `Can_Grip_L`** sits ~0.22 rad (j5) / 0.20 rad (j7) past the end of the
  `Grip_Can_L` recording. Confirmed intentional — that twist seats the hand in the
  teapot handle — but not re-verified since the recording was taught.
- **Is `allowed_start_tolerance` 0.05 rad enough** for the MIT controller's standing
  error under the teapot payload? Too low aborts the replay before it moves (safe
  failure); too high lets a replay start further from its taught path than intended.
- **Which backend gestures and tactile sensors** work for the glass and the bottle,
  and what the safest fallback is if a hand loses contact during a pour.
- **Pour angle and duration** for a visually successful but low-risk first demo.
- **The general payload service** (§4) and the optional dynamic `grip_center`.
