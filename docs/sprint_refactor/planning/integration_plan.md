# Sprint Refactor - Integration Plan

status: CANONICAL_PLAN
last_updated: 2026-08-11
branch: ROS2_Duo_System_V02_refactor

## Goal

Turn the coordination refactor proposal into a migration sequence that can be
implemented on branch `ROS2_Duo_System_V02_refactor` without building later work on top
of unsafe ownership assumptions.

This plan is the canonical migration surface. `../coordination_architecture_refactor_proposal.md`
is the architectural input; where the two disagree, this plan wins. Proposal
items that are still relevant but were missing here have been consolidated into
the phases below (see `Consolidated from the proposal`).

## Priority

The refactor takes priority over demo work on three axes: **safety**, **CPU
relief**, and **parallel operation**. The demo is not meaningful before those
land. `docs/sprint6/` adapts to the resulting contracts afterwards rather than
running in parallel for the same hardware and the same files.

## Working practice

- follow the `commit-quality` skill before every commit: the message states the
  system-level change, why it was needed, and its consequence, and names the
  level the evidence came from when the change touches CAN, timing, or motion
- run the `docs-keeper` agent at each phase boundary to reconcile the docs tree
  and the agent layer against the actual code state
- when a statement in this surface is superseded, rewrite it and mark the old
  reading superseded with its date; never append a contradicting "Update:" block
- a rule, skill, or agent changed in `.claude/` but not in `.github/` is a defect

## Planning rules for this branch

- fix release blockers and ownership bugs before doing cleanup or performance work
- keep package boundaries intact unless a new repo-owned ROS contract requires a
  new interface in `src/agx_arm_msgs`
- validate along the test ladder (C4), not just "the narrowest package"
- treat hardware validation as a gated activity, not as something implied by
  editor-only code review
- the **L2 activity harness is the mandatory regression after every phase**.
  `tea_pour_left_v1` is deferred by decision until the demo is re-taught
  against the new contracts, so it is the eventual end-to-end benchmark, not a
  current gate
- once re-taught, keep `tea_pour_left_v1` runnable after every phase as the end-to-end
  regression benchmark, accepting that it will be re-taught once the contracts
  settle

---

## Binding constraints

Fixed inputs, not tuning levers. Proposal text that contradicts them is
superseded.

### C1. Separate CAN bus per device (supersedes shared-bus step-and-settle)

```text
can0  ->  can_nero_right     right arm      (native mttcan)
can1  ->  can_nero_left      left  arm      (native mttcan)
can2  ->  hand_right         right OmniHand (USB-CAN FD adapter, peak_usb)
can3  ->  hand_left          left  OmniHand (USB-CAN FD adapter, peak_usb)
```

Like the arm buses, the hand adapters are used under stable renamed interfaces
rather than raw `canN`; the bridge is launched with `can_interface:=hand_left` /
`hand_right`. Deterministic naming is a 2A requirement precisely because two
identical USB adapters can otherwise swap enumeration order.

The interface names are `hand_left`/`hand_right`, not `left_hand`/`right_hand`;
they were renamed on 2026-08-11 and an earlier revision of this plan had them
the other way round. `left_hand` remains the *scheduler resource* name in
`graph_model.py`, so the two spellings are not interchangeable — 2A must not
derive one from the other by string surgery.

The arm buses also need the Jetson 40-pin header configured before `mttcan`
carries anything; an unconfigured header presents as an interface that is UP and
completely silent.

Consequences:

- **same-side arm and hand motion may run in parallel**; they are no longer
  mutually exclusive resources
- parallel operation is the new normal mode; step-and-settle survives only as a
  selectable degraded-topology mode
- the hand lease is no longer needed as *bus arbitration*; what remains is
  single-commander arbitration per hand device
- `prepare_hand_window` / `resume_arm_control`, arm feedback-push silencing, and
  the MIT stand-down on `feedback/hand_window_active` lose their rationale
- **Wire-level CAN contention is removed; host runtime resources are now the
  observed shared bottleneck.** The RX-overflow defect in
  `docs/sprint6/errors_and_fixes.md` was host-side socket overflow from CPU
  starvation, not bus arbitration. Parallel operation makes that failure mode
  *more* likely, not less — and the shared resources are plural: CPU, the GIL,
  executor threads and the kernel socket buffers, which the Phase 0 fault test
  showed can couple one bus's stall to another bus's frames.

A later consolidation to one shared bus for both hands is anticipated; keep the
bus mapping data-driven so that change is a registry edit.

### C7. Bus topology is one declared fact, not two switches

The topology is declared once in the registry and everything else derives from
it:

```yaml
bus_topology: dedicated_per_device   # or: shared_per_side
```

From that single value follow **both** the scheduler's resource claims (whether
`<side>_arm` and `<side>_hand` share a bus token) **and** whether the arm/hand
handoff runs at all. The existing `handoff_enabled` coordinator parameter becomes
a *derived* value, not an independent one.

This closes a dependency gap that would otherwise be built in: `handoff_enabled`
and `ROBOT_UNITS` describe the same physical fact, so leaving them separately
configurable gives the coordinator and the resource scheduler two independent
truths about one wiring loom. A run with `handoff_enabled=false` and a still-
shared bus token would serialise motions that need no serialising; the reverse
would drop the handoff on a genuinely shared bus. Neither failure is visible
until it matters.

Consequence for the phases: 2A introduces the declaration, 2B derives the
handoff from it, and 2C derives the resource model from it. Nothing reads
`handoff_enabled` directly after that.

### C8. The two arms speak different protocol tiers, permanently

The arms were bought as different versions and cannot be flashed. The right arm
runs firmware 1.06 (default Nero protocol tier), the left runs 1.11
(`NeroFW.V111`). This is a standing property of the unit.

The tiers are not two revisions of one protocol. The 1.11 driver encodes MIT
frames with a 12-bit feed-forward torque field and no CRC, and overrides
`set_motion_mode`, `move_mit`, `get_flange_pose` and `get_motor_states`; it also
carries its own status enum. The vendor SDK dispatches per tier, so the split is
handled *if and only if* the tier is resolved per arm and nothing above the SDK
assumes symmetry.

What follows from it:

- **Anything derived from the protocol is per tier, not per robot model.** The
  first casualty was the MIT feed-forward torque bound: 24/16/8 N·m per joint on
  the default tier, a flat 16 N·m on 1.11. One table for both arms refuses
  legitimate commands on one arm and admits impossible ones on the other.
- **A config that is valid on one arm may not be on the other.** The MIT
  controller's `torque_limit` is shared configuration; above 16 N·m it is
  accepted by the right arm and refused by the left, which under a dual-arm
  activity leaves one arm executing and one frozen at its last setpoint. The
  driver refuses loudly and names the bound, which is the current handling.
- **Every measurement names the arm it came from.** A number taken on one arm
  is not a number about "the Nero".
- Audited 2026-08-12 and *not* affected: the status and mode comparisons all go
  through `self.agx_arm.ARM_STATUS`, which is tier-dispatched; and the flange
  pose, where the default tier applies a posture correction that 1.11 does not
  need, reaches only `feedback/tcp_pose`, which nothing in the workspace
  consumes. If a consumer for that topic ever appears, the correction has to be
  re-checked against both tiers first.

### C2. MIT control rate is a requirement

100 Hz today (`mit_controller_node.py` `control_rate_hz`, raised from 50 Hz in
`15ac809`) is the **minimum** for a stable controller; the target is 200–250 Hz.

The refactor may not lower the control rate to save CPU. Gravity compensation
costs one RNEA over 19 articulated payload joints per tick per arm, so it scales
with the rate: ~200 RNEA/s today, ~400–500/s at target. The lever is **per-call
cost**: cache the unchanged-configuration term, reduce articulated payload DoF,
share one model instance per process, or compile the gravity path.

### C3. Vendor SDK: pinned execution path, separate development checkout

`vendor/pyAgxArm` is a submodule pinned at `control-layer-pin-2026-07-24`
(commit `4f52610`, fork `github.com/robyngehler/pyAgxArm`). The pinned submodule
is the execution path and stays unchanged during a phase. Vendor development
happens in a separate checkout of the same fork with the upstream vendor remote
configured so upstream updates stay mergeable; relevant work is pushed, tagged,
and lands here as an explicit pin bump followed by a workspace rebuild. No phase
gate may depend on an unpinned or dirty submodule state.

`docs/project/control_layer_and_dependencies.md` is the canonical record of this
workflow, including the editable-install recipes and the drift-prevention rules;
do not restate it here. **The development checkout is currently absent on this
host**, so recreating it per that document is a prerequisite for 0C.

### C4. Test ladder

| Level | Scope | Where it runs | When it is required |
| --- | --- | --- | --- |
| L1 unit | pure logic, no ROS spin | any platform | every commit |
| L2 mock/integration | ROS graph with mock backends, no hardware | any platform; primary gate on non-`aarch64` | before any hardware run |
| L3 hardware e2e | real arms, hands, CAN | `aarch64` with granted hardware access | every phase gate |

The platform decides what is *possible*; the ladder decides what is *required*.
L1 and L2 must pass before hardware is touched, on every platform. L2 is not a
substitute for L3 for timing, CAN, or safety claims. A phase gate that could not
reach L3 must say so explicitly. Encoded as a `.claude/skills/` workflow in 0B.

### C5. Message policy

- native ROS interfaces where they already carry the meaning
  (`sensor_msgs/JointState`, `std_msgs/Header`, `builtin_interfaces/Time`,
  `action_msgs`); repo-owned interfaces only for what ROS lacks
- statically defined fields; no runtime-variable structure in control paths
- hand interfaces abstract enough for any hand, not OmniHand-shaped
- the hand message surface is **consolidated, not extended**: `HandCmd`,
  `HandPositionTimeCmd`, `HandStatus`, `GripperStatus`, and `OmniHandStatus`
  merge into one abstract hand contract with a documented caller migration
- `src/agx_arm_msgs` has no `srv/` directory yet; adding one is part of Phase 1

**Extend the existing hand-model abstraction; do not invent a second one.** The
repo already models hand variants as data, and that layer is the reference for
what "abstract enough" means:

| Model | Device | Active joints | Limit unit | SDK class |
| --- | --- | --- | --- | --- |
| `o10` | OmniHand 2025 | 10 | rad | `AgibotHandO10` |
| `o12_pro` | OmniHand Pro 2025 (current) | 12 | deg | `AgibotHandO12` |

Joint suffixes, per-joint limits, left-hand mirror directions, and tactile finger
maps all come from `duo_motion_registry.yaml` through
`agx_arm_ctrl/omnihand/models.py`; the bridge selects a model with the
`hand_model` parameter (`DEFAULT_HAND_MODEL` is `o12_pro`). The two hardware
variants differ exactly in **actuated joint count and tactile feedback**, which
is what the consolidated message contract must carry as data rather than encode
in the message shape. Joint count, joint naming, and tactile layout must
therefore never be fixed-width fields.

### C6. Instrumentation form (MVP)

Log counters in the node plus external measurement tooling (`tegrastats`,
`pidstat`, `ip -s -d link`, `ros2 topic hz`/`bw`). No new public ROS metrics
contract in Phase 0, and no in-band publication that loads the node being
measured. Revisit only if the baseline proves counters insufficient.

---

## Cross-checked starting point

Verified against the working tree on 2026-08-11; all findings still hold.

- Velocity-based soft e-stop verification is not trustworthy (`driver.py:540`
  forces `velocity = 0.0`; `:541` also flips the sign of `current`, unaudited).
- The arm driver still has multiple SDK callers and mode mutations.
- The skill controller still republishes hold commands after a grasp succeeds
  (`omnihand_skill_controller_node.py:590`).
- The bridge still polls and retries outside explicit ownership state
  (`omnihand_bridge_node.py:971-973`).
- The coordinator still accepts every activity goal (`coordinator_node.py:320`)
  and advances children through `time.sleep`-based polling (`:393`, `:466`,
  `:850`).
- The coordinator has since grown a Ctrl+C stop ladder and replay planning
  (`8e8fc44`, `4762114`, now 963 lines); that logic must be **migrated** by
  Phase 3, not deleted with the polling loop.
- The registry has **no hand bus entry**; the bridge derives its interface from
  the arm's `can_port` and falls back to `can_nero_right`.
- `OMNIHAND_SOCKETCAN_IFACE` is set as a process-global environment variable by
  the backend constructor — a correctness hazard once the hands are on two
  distinct adapters.
- **A partial mode switch already exists**: the coordinator parameter
  `handoff_enabled` (default `True`, `coordinator_node.py:212`) skips
  `prepare_hand_window` / `resume_arm_control` when disabled. It is **not
  sufficient on its own**: the scheduler still serializes same-side arm and hand
  through the shared `*_can_bus` token in `graph_model.py` `ROBOT_UNITS`.
  Parallel operation needs both halves (2B and 2C).
- The active Duo MoveIt controller surface is already generated in
  `_moveit_config_builder.py`; the legacy standalone unprefixed
  `moveit_controllers.yaml` is a quarantined legacy artifact.

## Workstream map

| Workstream | Primary packages | Why it belongs early |
| --- | --- | --- |
| Guidance hygiene | `docs/`, `.claude/`, `.github/`, `AGENTS.md`, `CLAUDE.md` | Stale guidance makes every later slice chase a closed path |
| Regression harness | `agx_arm_ctrl`, `agx_arm_coordination`, `.claude/skills` | Nothing else can be refactored safely without it |
| Velocity truth and stop semantics | `vendor/pyAgxArm`, `agx_arm_ctrl`, `agx_arm_coordination` | Current safety wording is unsound until feedback is honest |
| Side authority, epoch, feedback budget | `agx_arm_msgs`, `agx_arm_ctrl`, `agx_arm_mit_controller` | One authoritative owner, and the vehicle for the first CPU relief |
| Parallel operation | `agx_arm_description`, `agx_arm_ctrl`, `agx_arm_coordination`, `scripts/` | The new normal mode; replaces the closed hand-window path |
| Coordinator exclusivity | `agx_arm_coordination` | Parallel execution raises concurrency, so unit rules must be explicit |
| Registry and contract consolidation | `agx_arm_description`, `agx_arm_ctrl`, `agx_arm_moveit`, `agx_arm_msgs` | Four buses and per-device mapping make drift expensive |
| Runtime work reduction | `agx_arm_ctrl`, `agx_arm_mit_controller` | CPU is the binding constraint (C1, C2) |

---

## Phase 0 - Hygiene, harness, safety baseline

### 0A. Guidance and topology hygiene

Nothing else starts until the guidance layer stops describing a closed path.
Known targets, from the 2026-08-11 sweep:

- sprint pointers still naming `docs/sprint6/` as the current entrypoint:
  `CLAUDE.md:51-52`, `.claude/rules/context-routing.md:50`,
  `.github/copilot-instructions.md:38-39`
- ROS contract rules that now point the wrong way:
  `AGENTS.md:41` and `CLAUDE.md:77` (shared `control/joint_states` as the
  coordinated arm-plus-hand command surface), `AGENTS.md:44` (do not map
  OmniHand onto the Revo2 messages, versus the C5 consolidation)
- `.claude/rules/omnihand-bridge.md`: command surface and message rules
- operational docs describing the shared side bus and hand window as normal
  operation: `docs/control/bringups/teach_and_run.md`,
  `docs/control/bringups/tea_demo.md`,
  `docs/assets/omnihand/omnihand_solo_bringup_and_load_test.md`
- `scripts/activate_native_can.sh`, whose header still documents can0/can1 as
  shared side buses
- `docs/sprint6/` step-and-settle planning notes, to be marked superseded
- all `pyAgxArm` paths, which must read `vendor/pyAgxArm`
- `vendor/Omnihand-2025-SDK/` is the **legacy SDK for the non-Pro OmniHand
  (`o10`) and is no longer used**. The runtime targets the OmniHand Pro through
  `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg`. The stray checkout is left in
  the working tree untracked; removing it is deliberate cleanup, not part of a
  code slice. Phase 2A targets the Pro SDK.

Rule for operational docs: the *hardware* has four buses, but the *code* still
resolves the hand interface from the arm bus. Operational docs therefore get a
supersession banner describing what is current and what replaces it — they are
not rewritten to describe behaviour that does not exist yet. They are rewritten
in 2A when the code catches up.

### 0B. Regression harness and test ladder

- add an L2 mock integration test driving coordinator to arm driver to hand
  bridge through one activity including a hand action, on mock backends **(done)**
- encode the C4 ladder as a `.claude/skills/` workflow with a `.github/skills/`
  mirror, following the shape of the existing `commit-quality` skill **(done)**
- define the `tea_pour_left_v1` regression criteria (deferred: the L2 harness
  is the standing regression until the demo is re-taught)

Notes from building it:

- the production arm driver has **no mock backend**, so the harness uses an arm
  test double (`test/l2_arm_double.py`) offering the driver's service surface.
  Phase 1 changes that surface, and the double is where the new contract gets
  pinned first.
- the harness runs on its own `ROS_DOMAIN_ID` and refuses a non-empty domain;
  this machine carries live hardware on the default domain.

Exit gate: an L2 run reproduces an activity end to end without hardware.

### 0C. Honest velocity and stop semantics

- derive velocity from timestamped joint positions in the repo path, with an
  explicit validity flag rather than a silent zero
- **done, and the protocol comparison is closed**: captured traffic shows the
  firmware's field flat at zero while the joints move, so there is no protocol
  value to compare against. What remains is validating the *derived* signal
  against a controlled position trajectory, and defining its filter, maximum
  sample age, and validity rule
- separate `commanded` from `feedback_verified` in stop reporting
- define and implement the coordinator response to a `commanded`-only stop,
  including the Ctrl+C stop ladder from `8e8fc44`
- keep the emergency path fail-closed when velocity evidence is missing
- audit the undocumented `current *= -1` mutation while in that code

Primary files: `vendor/pyAgxArm/.../nero/default/driver.py`,
`agx_arm_ctrl_single_node.py`, `coordinator_node.py`,
`test/test_emergency_stop_verify.py`.

### 0D. Baseline instrumentation

Per C6: loop duration, callback duration, SDK call origin with thread id, and
per-interface CAN traffic, as in-node log counters plus external tooling.

### 0E. Hardware baseline (L3)

This is the hardware slot that gates the safety work. Capture at the current
100 Hz MIT rate on the four-bus topology:

- idle stack; dual-arm hold; one MIT arm; two MIT arms; one hand action
- **same-side arm and hand in parallel** (new, only possible under C1)
- both sides arm-plus-hand in parallel
- bus-fault and recovery
- per-interface CAN counters and RX drop counts for every scenario

Exit gate:

- no stop path relies on known synthetic zero velocities
- a baseline report exists per scenario
- the parallel scenarios are characterised before code enables them by default

---

## Authority and epoch model

The proposal's "side hardware authority" is one grain too coarse. A side is no
longer a resource — the devices on it have separate buses, separate failure
modes and separate recovery paths. Four authorities, one per device:

```text
LeftArmAuthority            RightArmAuthority
LeftHandTransportAuthority  RightHandTransportAuthority
```

The hand authorities are named *transport* deliberately: what they own is the
vendor SDK session and the CAN transport for one hand, not the semantics of a
grasp, which stays with the skill controller.

### Epochs

Two levels, because they answer different questions:

| Epoch | Scope | Increments on |
| --- | --- | --- |
| `device_epoch` | one device | that device's ownership transition, recovery, re-enable |
| `unit_safety_epoch` | the whole unit | e-stop, unit fault, anything that invalidates every device at once |

A command carries the `device_epoch` of the device it addresses plus the
`unit_safety_epoch` it was issued under, and is rejected if either is stale.

**Why per device rather than per side.** With one epoch per side, a left arm
recovery would invalidate in-flight left *hand* commands, because both are
"left" — a hand grasp aborted by an unrelated arm fault on a bus the hand does
not share. That is exactly the coupling C1 removed at the wire level, and it
would be reintroduced in software. The unit-level epoch keeps the case that
*should* invalidate everything — an emergency stop — genuinely global.

## Phase 1 - Device authority, serialized SDK access, unit guard

Unaffected in substance by the topology change: the SDK time-of-check/
time-of-use race is independent of bus layout. With the hand-window path closed,
this is the highest-value phase, and it carries the first CPU relief because the
serialized worker is the vehicle for it.

### 1A. Device authority, epoch, serialized SDK access

The authority and epoch model above, plus the worker that makes it real:

- `LeftArmAuthority` / `RightArmAuthority` each publish one authoritative device
  state carrying `device_epoch` and the current `unit_safety_epoch`
- all SDK calls for one arm run on one worker or queue, with emergency stop on a
  priority lane ahead of queued motion
- old-epoch queued commands are dropped after ownership transitions or recovery
- extend `MoveMITMsg` with the frozen command stamp — `owner_id`,
  `device_epoch`, `unit_safety_epoch`, `sequence` — rather than adding a
  parallel `ArmMitCommand` (frozen decision); this is an ABI change needing a
  coordinated workspace rebuild, and producer, consumer, docs and tests migrate
  in the same change set
- MIT consumes the authoritative device state instead of the
  `feedback/hand_window_active` boolean, and aborts on authority loss
- reject at the hardware boundary: wrong epoch, duplicate or missing joint
  indexes, empty commands, non-finite values, out-of-range positions, gains and
  torque, stale sequences. SDK clamping stays a last protection, not the input
  contract
- separate fault acknowledgement from verified rearm, and make recovery report
  what it *verified* rather than what happened to be true afterwards — the 0E
  fault test showed "recovery succeeded" reported for a bus that came back on
  its own
- replace the unassigned `AgxArmStatus.err_status` with a documented structured
  error representation
- fix enable readback and firmware-version parsing (the node has no `NeroFW.V112`
  branch and compares versions as strings); make forced e-stop recovery
  independent of the optional normal-recovery setting
- disable or quarantine direct legacy arm motion ingress for coordinated
  hardware profiles
- add the `srv/` directory to `src/agx_arm_msgs` for the new contracts

Exit gate: no arm SDK call outside the worker on the migrated path; stale-epoch
commands rejected in tests; MIT aborts immediately on authority loss; the
SDK-call counter under a *full* stack still shows one thread per arm.

### 1B. Feedback snapshot and driver CPU reduction

The worker from 1A is the vehicle, and the 0E baseline says where the cost
actually is:

- separate acquisition cadence from publication cadence
- one immutable feedback snapshot per acquisition cycle; no callback calls SDK
  getters directly
- **target the whole publish batch, not just the per-joint reads.** Measured,
  those reads are ~0.11 ms of a 1.10 ms batch; the other 90 % is pose, arm
  status, effector status, leader publication and message
  construction/serialisation. Batching the reads alone recovers about a tenth
- keep only rates justified by control deadlines or consumers; `pub_rate` is a
  republish rate and not a bus lever
- close the 10-second publish-loop hole seen during the 0E fault test: a stalled
  loop must not stop draining the RX socket

Exit gate: measured CPU improvement on the arm driver against the 0E baseline
(71.6 % of a core at rest, 1.10 ms mean batch against a 5 ms period).

### 1C. One active unit activity — the small guard

Pulled forward from Phase 3 on review. Parallel operation multiplies the ways
two activities can interleave, so the exclusivity rule has to exist *before*
Phase 2 enables the parallelism, not after.

Deliberately small — this is the guard, not the event-driven rewrite:

```text
READY      -> accept one activity
EXECUTING  -> reject every further goal, with a structured reason
```

- one authoritative unit activity state and failure reason
- no change to the polling loop, no event queue, no cleanup-deadline work; all
  of that stays in Phase 3

Exit gate: a second concurrent activity goal is rejected while one runs.

---

## Phase 2 - Parallel operation

Replaces the struck "leased hand control" phase. The hand-window path is closed,
not migrated; this phase builds its parallel-operation equivalent.

### 2A. Declare the topology and model the four buses

- add `bus_topology` to the registry as the single declared fact (C7), plus a
  per-device bus entry `omnihand.sides.<side>.can_port`; bump the schema version
- remove the bridge's derivation of its interface from the arm's `can_port`;
  fail closed for hardware profiles instead of falling back to `can_nero_right`
- replace process-global `OMNIHAND_SOCKETCAN_IFACE` selection with explicit
  backend construction so two hand backends cannot fight over one variable
- adopt slot-anchored interface bring-up (`scripts/activate_duo_can.sh`) as the
  supported path and retire `activate_native_can.sh` /
  `omnihand_canfd_activate.sh`; identical USB adapters must not be addressed by
  enumeration order
- rewrite the operational docs banner-marked in 0A for the four-bus reality

### 2B. Parallel resource model, handoff derived not configured

- derive the scheduler's claims from `bus_topology`: under
  `dedicated_per_device`, `<side>_arm` and `<side>_hand` no longer share a bus
  token in `graph_model.py` and may be scheduled concurrently
- derive `handoff_enabled` from the same value; nothing reads it directly
- remove the MIT stand-down on `feedback/hand_window_active` from the parallel
  path
- keep `prepare_hand_window` / `resume_arm_control` only as the
  `shared_per_side` implementation, documented as such, with no new work invested
- fix the stale warning text that still explains TX loss as "hand-frame
  arbitration loss on the shared bus"
- add tests for the newly reachable interleavings

### 2C. Hand arbitration and transport efficiency

Substantially expanded on review, and moved ahead of the coordinator rewrite,
because the 0E baseline made it the largest single CPU consumer in the system:
**115 % of one core for 27 CAN frames per second**, against the arm driver's
73 % for ~2800 f/s. The cost is in how the transport is driven, not in frame
volume, so this is a correctness-shaped performance problem rather than tuning.

Arbitration:

- implement the frozen single-goal contract: one commander per hand device,
  enforced by rejecting a second active goal, with `owner_id`, `device_epoch`
  and `sequence` on the command. No separate lease contract
- reject stale-epoch and out-of-order hand commands at the bridge boundary

Transport efficiency — profile first, then cut:

- **profile the O12 Pro backend at SDK-call level**: which calls, how many per
  setpoint, how long each blocks. The 115 % figure is process-level; the
  distribution inside it is unmeasured and the fix depends on it
- eliminate the read-before-write round trip in the full-joint command path
- decouple command verification, joint readback, tactile and status into
  separate schedules instead of one shared cadence
- stop polling entirely while no hand action is active
- remove the recurring post-grasp hold traffic
- bound the SDK round trips per commanded setpoint, and record the bound

Exit gate:

- a hardware profile with a missing or wrong hand bus fails at configuration time
- hand traffic appears only on the hand interfaces
- same-side arm and hand motion run in parallel in an L3 run without CAN RX drops
- **measured** hand-bridge CPU reduction against the 115 % baseline, with the
  per-call profile recorded alongside it
- the `shared_per_side` topology still executes an activity when selected

---

## Phase 3 - Event-driven coordinator and strict synchronization

The exclusivity guard landed in 1C, ahead of the parallelism that makes it
matter. What remains here is the conversion itself.

### 3A. Event-driven child management

- replace future polling and `time.sleep` with done-callbacks and an internal
  event queue
- keep only a low-rate watchdog for deadlines and timeouts
- add bounded child-cancel and cleanup handling with structured failure reasons
- migrate the Ctrl+C stop ladder and replay planning onto the event model
  without weakening them
- make SIGINT with no activity in flight exit rather than spin (0B finding)

### 3B. Strict synchronization behavior

- require `sync_flag` arm pairs to merge into the proper combined execution path
- remove any independent-dispatch fallback for strict synchronization

### 3C. Complete the unit activity state

- extend the 1C guard from "reject while executing" to the full state machine
  with cleanup as part of completion
- an activity cannot succeed if a required final state, child cancel, or arm
  resume fails

Exit gate:

- child completion, cancellation, and cleanup do not rely on a polling loop
- parallel same-side activities from 2B still execute correctly
- concurrent goals are still rejected (regression on the 1C guard)

---

## Phase 4 - Registry, manifest, and contract consolidation

### 4A. Resolver and manifest

- one resolved manifest combining registry, execution profile, and explicit
  launch overrides, with source hashes and a manifest hash

### 4B. Reduce execution profiles to selection-only composition

- stop repeating side namespaces, prefixes, controller names, and CAN ports that
  already exist in the registry
- build on the existing `execution_profiles.py` resolver

### 4C. Generate runtime artifacts

- generate MoveIt simple-controller-manager config, resource claims,
  joint-state merger inputs, and launch parameter dictionaries from the manifest
- quarantine the legacy standalone `moveit_controllers.yaml`
- keep the MoveIt hand FJT path non-default in coordinated production profiles

### 4D. Hand contract consolidation

Moved here from the struck Phase 2 because it is source-of-truth work, not
parallel-operation work. Note that it touches the bridge a second time after 2A;
sequence the bridge edits so the two do not fight.

- consolidate `HandCmd`, `HandPositionTimeCmd`, `HandStatus`, `GripperStatus`,
  and `OmniHandStatus` into one abstract hand contract per C5
- carry owner identity, control epoch, and sequence in hand commands
- keep joint count, joint naming, and tactile layout as data, never as
  fixed-width fields, so `o10`, `o12_pro`, and the 1-DoF AGX gripper all fit
- migrate every in-repo caller and remove the retired messages inside this phase;
  there are no external consumers
- validate joint values at the bridge; reject unknown, duplicate, non-finite,
  out-of-limit entries
- remove SDK read-before-write for full-joint commands; reject partial commands
  without a valid cache
- distinguish `commanded`, `delivery_verified`, and `contact_confirmed`
  completion; a timed-out grasp without contact is not a success
- document the caller migration for every retired message

### 4E. Fail-closed configuration

- production launch chooses exactly one configuration origin, never a mix
- silent fallback to built-in CAN interfaces only in mock profiles
- repository check for duplicated authoritative values

Exit gate:

- changing a side namespace, prefix, or CAN port requires one authoritative edit
- all migrated runtime nodes report the same manifest hash

---

## Phase 5 - Runtime consolidation and close-out measurements

CPU is the binding constraint (C1, C2). 5A moved into Phase 1E.

### 5B. MIT tick cost — decompose before optimising

The 0E baseline measured a MIT controller at ~60 % of a core *while merely
holding*, which at the 200–250 Hz target of C2 roughly doubles. It did **not**
measure where inside the tick that time goes.

"Gravity compensation dominates" is a **hypothesis**, and the same measurement
already falsified the analogous assumption about the arm driver — the per-joint
SDK reads that `critical_cpu_paths.md` called dominant turned out to be ~10 % of
their batch. Decompose first, with the same counters:

| Segment | What it covers |
| --- | --- |
| trajectory sampling | interpolation, buffer access |
| feedback snapshot | reading and assembling current state |
| gravity / RNEA | the pinocchio call itself |
| command construction | building the MIT setpoint |
| ROS publish | serialisation and transport |
| action feedback / tolerance | goal and path tolerance checks, feedback emission |
| locking / executor | contention, callback-group waits |

Only then optimise, and only what the decomposition names. Structural work that
holds regardless:

- one timer as the sole trajectory evaluator
- event-driven action completion and feedback emission off execution snapshots
- velocity tolerances stay unavailable until fed from the derived source — the
  firmware value they used is flat zero (see `reference/phase0_baseline.md`)

### 5C. Bridge timer split

- split command verification, tactile, status, and heartbeat work by semantics
- active polling only during hand ownership; cached or heartbeat status otherwise

### 5D. Executor and process policy

- single-threaded executors where concurrent blocking work is not required
- explicit callback groups for coordinator and MIT controller
- bounded executor thread counts
- each vendor SDK session in its own process

### 5E. Feedback duplication

- arm driver publishes arm joints only; hand bridge publishes hand joints only
- one unit-level aggregator produces the combined MoveIt state at a bounded rate

### 5F. Before/after measurement close-out

- re-run the 0E scenarios including the parallel ones
- compare CPU, callback duration, timer jitter, and per-interface CAN traffic
- record remaining hardware-only gaps before broader rollout

Exit gate:

- no unconditional high-rate loop performs non-control work
- post-refactor measurements captured and compared against 0E
- the MIT control rate holds at or above 100 Hz with headroom toward the target

---

## Phase 6 - Unit contract skeleton

- freeze the unit-level public contract for later multi-unit work
- namespace public unit interfaces; separate internal side topics from public
  unit topics
- no dynamic leader election, no distributed consensus in this sprint

---

## Struck from the plan

Deleted with the old Phase 2 "Leased hand control", so it does not get
resurrected. All of it existed to arbitrate a bus that is no longer shared:

- hand lease acquire/release action and service as a bus-arbitration contract
- bus-quiet verification and CAN feedback-push silencing as lease preconditions
- "zero hand TX during arm ownership" as an acceptance criterion
- lease identity threaded through coordinator, skill controller, and bridge for
  the purpose of bus ownership
- step-and-settle as the normal operating model

Retained from that phase, redistributed: bus topology modelling (2A), window-path
closure (2B), resource split (2C), single-commander arbitration and traffic
reduction on CPU grounds (2C), hand contract consolidation (4D).

## Consolidated from the proposal

| Proposal section | Landed in |
| --- | --- |
| §12.2 arm feedback snapshot and publication budget | 1E |
| §12.10 executor and process policy, per-process SDK sessions | 5D |
| §13.1 command validation at the hardware boundary | 1D |
| §13.2 `err_status` fault contract | 1D |
| §14.1 enable readback, firmware version parsing, forced e-stop recovery | 1D |
| §14.5 commanded / delivery_verified / contact_confirmed distinction | 4D |
| §14.6 read-before-write, partial-command cache, fail-hard CAN mapping | 2A, 4D |
| §11 MoveIt hand FJT non-default in production | 4C |
| §16.9 registry drift check | 4E |
| §16 failure-injection and state-conformance suite | 0B and per-phase gates |

Superseded by this plan:

| Proposal section | Superseded by |
| --- | --- |
| §3.3, §3.4 shared-bus hand window rationale | C1 |
| §7 lease as bus arbitration, bus-quiet verification | C1, Struck |
| §9 shared `*_can_bus` resource claims | C1, 2C |
| §12.6, §12.8 ownership-gated polling as a bus measure | 2C, on CPU grounds |
| §16.3 "hand TX during ARM_CONTROL = 0" acceptance | 2 exit gate |
| §12.2 lowering rates as the CPU lever | C2 |

---

## Implementation order

Revised on external review (2026-08-11). Two changes from the first ordering:
the unit-exclusivity guard moves ahead of the parallelism it has to constrain,
and the hand transport work moves ahead of the coordinator rewrite because the
baseline made it the largest CPU consumer in the system.

```text
Phase 0   baseline / velocity / harness                    done
   |
Phase 1A  device authority + epochs + SDK worker
   |
Phase 1B  feedback snapshot + driver CPU reduction
   |
Phase 1C  ONE active unit activity guard          [pulled forward]
   |
Phase 2A  bus_topology declaration + four-bus registry + slot-anchored bring-up
   |
Phase 2B  parallel resource model + handoff derived, not configured
   |
Phase 2C  hand single-commander + O12 transport CPU refactor   [expanded]
   |
Phase 3   event-driven coordinator + strict sync
   |
Phase 4   manifest + profiles + generated config + hand-contract migration
   |
Phase 5   MIT tick decomposition + executor/process cleanup + final measurement
   |
Phase 6   unit API
```

Why the guard moves first: parallel operation multiplies the interleavings two
activities can produce, so "only one activity at a time" has to hold before the
parallelism exists, not after. It is a small guard in 1C; the full event-driven
conversion stays in Phase 3.

Why the hand work moves up: 115 % of a core for 27 frames/s is the single
largest measured cost, and it sits in the device the parallel mode is meant to
exercise.

## Validation commands to prefer

- `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg_names>`
- `colcon test --packages-select <pkg_names>` from a system-Python ROS shell
- the L2 mock harness before any hardware run
- targeted unit tests for the touched slice before broader integration checks

## Promotion path after implementation starts

- promote stable runtime contracts into `docs/assets/`
- promote stable launch and bring-up changes into `docs/control/`
- promote stable package-boundary or generated-artifact policy into
  `docs/project/`
- keep intermediate evidence, unresolved measurements, and branch-local rollout
  notes inside this sprint surface until the V02 migration closes
