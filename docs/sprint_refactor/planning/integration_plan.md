# Sprint Refactor - Integration Plan

status: CANONICAL_PLAN
last_updated: 2026-08-11
branch: ROS2_Duo_System_V02

## Goal

Turn the coordination refactor proposal into a migration sequence that can be
implemented on branch `ROS2_Duo_System_V02` without building later work on top
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

## Planning rules for this branch

- fix release blockers and ownership bugs before doing cleanup or performance work
- keep package boundaries intact unless a new repo-owned ROS contract requires a
  new interface in `src/agx_arm_msgs`
- validate along the test ladder (C4), not just "the narrowest package"
- treat hardware validation as a gated activity, not as something implied by
  editor-only code review
- keep `tea_pour_left_v1` runnable after every phase as the end-to-end
  regression benchmark, accepting that it will be re-taught once the contracts
  settle

---

## Binding constraints

Fixed inputs, not tuning levers. Proposal text that contradicts them is
superseded.

### C1. Separate CAN bus per device (supersedes shared-bus step-and-settle)

```text
can0  ->  can_nero_right     right arm    (native mttcan)
can1  ->  can_nero_left      left  arm    (native mttcan)
can2  ->  right hand         right OmniHand (USB-CAN FD adapter, peak_usb)
can3  ->  left  hand         left  OmniHand (USB-CAN FD adapter, peak_usb)
```

Consequences:

- **same-side arm and hand motion may run in parallel**; they are no longer
  mutually exclusive resources
- parallel operation is the new normal mode; step-and-settle survives only as a
  selectable degraded-topology mode
- the hand lease is no longer needed as *bus arbitration*; what remains is
  single-commander arbitration per hand device
- `prepare_hand_window` / `resume_arm_control`, arm feedback-push silencing, and
  the MIT stand-down on `feedback/hand_window_active` lose their rationale
- **CPU becomes the only remaining contention point.** The RX-overflow defect in
  `docs/sprint6/errors_and_fixes.md` was host-side socket overflow from CPU
  starvation, not bus arbitration. Parallel operation makes that failure mode
  *more* likely, not less.

A later consolidation to one shared bus for both hands is anticipated; keep the
bus mapping data-driven so that change is a registry edit.

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
(fork: `github.com/robyngehler/pyAgxArm`). The pinned submodule is the execution
path and stays unchanged during a phase. Vendor development happens in a
separate checkout of the same fork with the upstream vendor remote configured so
upstream updates stay mergeable; relevant work is pushed, tagged, and lands here
as an explicit pin bump followed by a workspace rebuild. No phase gate may depend
on an unpinned or dirty submodule state.

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
  bridge through one activity including a hand action, on mock backends
- encode the C4 ladder as a `.claude/skills/` workflow
- define the `tea_pour_left_v1` regression criteria enforced after every phase

Exit gate: an L2 run reproduces an activity end to end without hardware.

### 0C. Honest velocity and stop semantics

- derive velocity from timestamped joint positions in the repo path, with an
  explicit validity flag rather than a silent zero
- verify it against the protocol value using the C3 development checkout
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

## Phase 1 - Side authority, serialized SDK access, and arm feedback budget

Unaffected in substance by the topology change: the SDK time-of-check/
time-of-use race is independent of bus layout. With the hand-window path closed,
this is the highest-value phase, and it carries the first CPU relief because the
serialized worker is the vehicle for it.

### 1A. Authoritative side state

- per-side state contract with a control epoch, published by the arm side
  authority
- extend `MoveMITMsg` with `control_epoch` and `sequence` rather than adding a
  parallel `ArmMitCommand` (frozen decision); this is an ABI change and needs a
  coordinated workspace rebuild
- MIT consumes the authoritative side state instead of the
  `feedback/hand_window_active` boolean

### 1B. Serialized hardware worker

- all arm SDK calls for one side on one worker or queue
- emergency stop on a priority path within that worker
- old-epoch queued commands dropped after ownership transitions or recovery

### 1C. Recovery and rearm split

- separate fault acknowledgement from verified rearm
- require fresh feedback, comm health, and hold capture before returning to arm
  control

### 1D. Hardware-boundary validation and status contract

- reject wrong epoch, duplicate or missing joint indexes, empty commands,
  non-finite values, out-of-range positions, gains and torque, stale sequences
- keep SDK clamping as final protection, not as the input contract
- replace the unassigned `AgxArmStatus.err_status` with a documented structured
  error representation
- fix enable readback and firmware-version parsing; make forced e-stop recovery
  independent of the optional normal-recovery setting
- disable or quarantine direct legacy arm motion ingress for coordinated
  hardware profiles

### 1E. Arm feedback acquisition and publication budget

Pulled forward from the original Phase 5 because the worker introduced in 1B is
the natural vehicle and CPU relief is a priority axis:

- separate acquisition cadence from publication cadence
- one immutable feedback snapshot per acquisition cycle; no callback calls SDK
  getters directly
- remove repeated SDK getter work from the unconditional 200 Hz loop, which is a
  republish rate and not a bus lever
- keep only rates justified by control deadlines or consumers

Exit gate:

- no arm SDK call happens outside the worker on the migrated path
- stale-epoch commands are rejected in tests
- MIT aborts immediately when the side authority leaves arm control
- measured CPU improvement on the arm driver against the 0E baseline

---

## Phase 2 - Parallel operation

Replaces the struck "leased hand control" phase. The hand-window path is closed,
not migrated; this phase builds its parallel-operation equivalent.

### 2A. Model the four-bus topology

- add `omnihand.sides.<side>.can_port` to the registry and bump the schema
  version
- remove the bridge's derivation of its interface from the arm's `can_port`;
  fail closed for hardware profiles instead of falling back to `can_nero_right`
- replace process-global `OMNIHAND_SOCKETCAN_IFACE` selection with explicit
  backend construction so two hand backends cannot fight over one variable
- update CAN bring-up for four interfaces with deterministic adapter naming, so
  the two USB adapters cannot swap identities
- rewrite the operational docs banner-marked in 0A to describe the four-bus
  reality

### 2B. Close the hand-window path

- make parallel the default operating mode and step-and-settle a selectable
  degraded-topology mode, keeping the existing `handoff_enabled` coordinator
  parameter as the switch
- remove the MIT stand-down on `feedback/hand_window_active` from the parallel
  path
- keep `prepare_hand_window` / `resume_arm_control` only as the degraded-mode
  implementation, documented as such, with no new work invested in them
- keep arm feedback-push silencing only where it still serves a purpose outside
  hand arbitration

### 2C. Parallel resource model

- split `ROBOT_UNITS` in `graph_model.py` so `<side>_arm` and `<side>_hand` no
  longer share one bus token
- allow same-side arm and hand actions to be scheduled concurrently
- make the degraded mode re-couple them through configuration, not code
- add tests for the newly reachable interleavings

### 2D. Hand commander arbitration

- enforce exactly one commander per hand device through the existing single-goal
  action semantics of the skill controller plus `owner_id`, `control_epoch`, and
  `sequence` on the hand command; no separate lease contract (frozen decision)
- stop bridge polling and retries outside active hand ownership, on CPU grounds
- remove recurring host-side hold traffic after a grasp completes, on CPU and
  latency grounds

Exit gate:

- a hardware profile with a missing or wrong hand bus fails at configuration time
- hand traffic appears only on the hand interfaces; the arm interfaces carry no
  hand frames
- same-side arm and hand motion run in parallel in an L3 run without CAN RX drops
- the degraded mode still executes an activity when selected

---

## Phase 3 - Coordinator exclusivity and event-driven execution

### 3A. One active unit activity

- reject new activity goals unless the unit is `READY`
- track one authoritative unit activity state and failure reason

### 3B. Event-driven child management

- replace future polling and `time.sleep` with done-callbacks and an internal
  event queue
- keep only a low-rate watchdog for deadlines and timeouts
- add bounded child-cancel and cleanup handling
- migrate the Ctrl+C stop ladder and replay planning onto the event model
  without weakening them

### 3C. Strict synchronization behavior

- require `sync_flag` arm pairs to merge into the proper combined execution path
- remove any independent-dispatch fallback for strict synchronization

Exit gate:

- concurrent activity goals are rejected
- child completion, cancellation, and cleanup do not rely on a polling loop
- parallel same-side activities from 2C still execute correctly

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

### 5B. MIT execution loop and gravity cost

- one timer as the sole trajectory evaluator
- event-driven action completion and feedback emission off execution snapshots
- reduce per-tick gravity cost per C2 so the control rate can rise toward
  200–250 Hz rather than fall

This item is dependency-free and may be pulled forward if the 0E baseline shows
gravity compensation dominating.

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
reduction on CPU grounds (2D), hand contract consolidation (4D).

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
| §12.6, §12.8 ownership-gated polling as a bus measure | 2D, on CPU grounds |
| §16.3 "hand TX during ARM_CONTROL = 0" acceptance | 2 exit gate |
| §12.2 lowering rates as the CPU lever | C2 |

---

## Suggested implementation increments

1. Guidance and topology hygiene sweep (0A).
2. L2 mock harness and the test-ladder skill (0B).
3. `vendor/pyAgxArm` plus `agx_arm_ctrl`: velocity truth and stop semantics (0C).
4. Instrumentation, then the hardware baseline including parallel scenarios
   (0D, 0E) — the hardware slot for the safety checks.
5. Side state, epoch, serialized worker, boundary validation (1A–1D).
6. Arm feedback acquisition and publication budget (1E).
7. Registry bus modelling, explicit backend interface selection, CAN bring-up
   (2A).
8. Close the hand-window path and split the resource model (2B, 2C).
9. Hand commander arbitration and traffic reduction (2D).
10. Coordinator exclusivity and event-driven children (Phase 3).
11. Resolved manifest, generated artifacts, hand contract consolidation
    (Phase 4).
12. Runtime consolidation, gravity cost, measurement close-out (Phase 5).

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
