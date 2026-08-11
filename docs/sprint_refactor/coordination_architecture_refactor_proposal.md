# Proposal: Deterministic Coordination, Resource Ownership, and Runtime Consolidation for the Duo Nero System

> **Amended 2026-08-11 — read `planning/integration_plan.md` first.**
> This document is the architectural input, not the migration plan. The plan is
> canonical; where they disagree, the plan wins.
>
> Two premises of this proposal have since changed:
>
> 1. **The side CAN bus is no longer shared.** Each device has its own interface
>    (arms native `can0`/`can1`, hands on FD-capable USB adapters `can2`/`can3`).
>    Same-side arm and hand motion may run in parallel. Everything here that
>    treats the hand lease, bus-quiet verification, feedback-push silencing, or
>    the shared `*_can_bus` resource claim as a *safety* mechanism is superseded
>    (§3.3, §3.4, §7, §9, §12.6, §12.8, §16.3). The CPU arguments for bounding
>    hand traffic survive; the bus arguments do not.
> 2. **The MIT control rate is a requirement, not a lever.** It runs at 100 Hz
>    (minimum for stability) with a 200-250 Hz target, so §12.2's "lower the
>    rate" reasoning is replaced by reducing per-tick cost.
>
> Line references in Appendix A point at the reviewed snapshot and no longer
> match the working tree; the plan carries current anchors.

**Status:** Proposed, partially superseded  
**Date:** 2026-07-27  
**Scope:** One Duo unit consisting of two Nero arms, two OmniHands, one MoveIt instance, one Jetson, and one activity coordinator  
**Primary goal:** Establish a deterministic and resource-efficient control architecture with explicit ownership, closed state machines, fail-closed handovers, and enforceable single sources of truth.  
**Secondary goal:** Preserve a clean migration path toward multiple independently deployed Duo units coordinated by a higher-level system.

---

## 1. Executive Summary

The current stack already contains many valuable local safeguards: MIT stale-feedback handling, command verification and retry for OmniHand, firmware-hold verification before a hand window, bus-recovery logic, MoveIt-based dual-arm execution, resource-aware DAG scheduling, and registry-backed joint definitions. The primary problem is no longer the absence of safety logic. The problem is that this logic is distributed across several nodes that each maintain a partial and sometimes contradictory interpretation of the system state.

At present:

- the coordinator owns activity scheduling but does not exclusively own unit-level execution;
- the MIT controller owns trajectory and hold state but does not consume a complete authoritative driver state;
- the arm driver owns the CAN gateway but exposes multiple command paths and maintains several overlapping Boolean modes;
- the OmniHand skill controller reports a grasp action as complete while continuing to publish hold commands indefinitely;
- the OmniHand bridge continues periodic CAN reads outside explicit hand-control windows;
- execution profiles and the motion registry overlap rather than forming a strict selection-plus-resolution pipeline;
- high-rate polling, repeated message construction, duplicated feedback aggregation, and Python-side busy waiting consume scarce Jetson CPU capacity.

The proposed refactor introduces five architectural rules:

1. **One authority per decision.** The unit coordinator owns activities and resources; MoveIt owns geometric arm planning; the MIT controller owns arm setpoint generation; the side authority owns hardware mode and CAN access; the hand skill controller owns semantic hand behavior; the bridge owns vendor translation only.
2. **One closed state contract per layer.** Nodes must not infer safety-critical global state from unrelated flags. A single authoritative side-control state is published for each arm/hand side.
3. **Leased hand control.** A hand action may transmit or poll the shared CAN bus only while holding a verified, expiring side lease. The lease is deliberately small and pragmatic, not a distributed-consensus subsystem.
4. **Registry selection, then immutable resolution.** The system registry stores physical and geometric truth; execution profiles only select a deployment composition; a resolver produces one immutable manifest consumed by every node and generated MoveIt configuration.
5. **Event-driven execution with bounded periodic work.** High-rate loops are limited to control-critical functions. Status, diagnostics, retries, scheduling, and feedback publication are decimated, event-driven, or performed only while the associated resource is active.

The first MVP should not attempt multi-Jetson consensus, dynamic leader election, or a rewrite into `ros2_control`. It should make the existing single-unit system coherent, measurable, and safe enough to become the local building block of a later hierarchical multi-unit architecture.

---

## 2. Current System Boundary

The current deployment is treated as one robotic unit:

```text
Duo Unit
├── Unit Coordinator
├── MoveIt / move_group
├── Left side
│   ├── Nero arm driver
│   ├── MIT FollowJointTrajectory controller
│   ├── OmniHand bridge
│   └── OmniHand skill controller
└── Right side
    ├── Nero arm driver
    ├── MIT FollowJointTrajectory controller
    ├── OmniHand bridge
    └── OmniHand skill controller
```

MoveIt uses `moveit_simple_controller_manager/MoveItSimpleControllerManager`. This remains acceptable for the MVP because the arm controllers expose `FollowJointTrajectory` actions directly and are not implemented as `ros2_control` controllers. MoveIt should remain a planning and trajectory-dispatch layer, not become the owner of activity resources, CAN handovers, driver recovery, or semantic hand skills.

The checked-in `moveit_controllers.yaml` is a standalone configuration with one unprefixed `arm_controller` and a legacy `gripper_controller`. It must not be treated as the authoritative Duo configuration. The Duo controller configuration should be generated from the resolved system manifest. The standalone file should either be renamed accordingly or removed from Duo launch paths.

---

## 3. Confirmed High-Priority Findings

### 3.1 Emergency-stop velocity verification is currently invalid

The vendor Nero driver overwrites every reported motor velocity with `0.0` in `get_motor_states()`. The ROS arm driver then uses those values in `_arm_velocities_settled()` to classify an emergency stop as feedback-verified. The MIT controller also receives these zero velocities through `feedback/joint_states` and uses them for goal-velocity tolerance checks.

Consequences:

- a moving arm can be reported as stopped;
- goal velocity tolerances can pass without real velocity evidence;
- recorded diagnostics are misleading;
- any safety argument based on the current velocity field is invalid.

This is a release blocker. Before the rest of the coordination refactor is considered operational, velocity must be restored from the protocol or estimated from timestamped joint positions. Until then, software stop results must be reported as **commanded but not velocity-verified**.

### 3.2 The arm SDK has no exclusive caller

The arm SDK object is accessed from:

- the dedicated publisher/recovery thread;
- ROS subscription callbacks;
- service callbacks;
- emergency-stop paths;
- hand-window transitions;
- effector handlers.

The SDK maintains a mutable cached mode message (`_msg_mode`) that is modified by motion-mode changes, speed changes, and CAN-push changes. The repository-owned `nero_can_push` helper also modifies this object temporarily. Without serialization, a mode transition can interleave with a motion callback or recovery operation.

A simple precondition check such as `_check_can_control()` is not sufficient because the state can change between the check and the SDK call. This is a classic time-of-check/time-of-use race.

### 3.3 The current hand window is not a complete ownership transfer

`prepare_hand_window` gates arm commands and verifies a firmware hold. However, it can return `success=True` even when CAN feedback push silencing was not verified. The coordinator interprets any successful response as a valid hand window and dispatches the hand action.

The contract therefore currently conflates:

- “the arm is held”; and
- “the hand has usable shared-bus ownership.”

For deterministic behavior, a hand lease must require both.

### 3.4 Hand control continues after the coordinator releases the window

After a confirmed grasp, `OmniHandSkillController` marks the action successful and enables an internal hold timer. That timer republishes the same hand target at the configured skill-control rate and continues tactile monitoring. The coordinator, meanwhile, receives the successful action result and immediately calls `resume_arm_control`.

The bridge sends every recognized hand target to the vendor backend and maintains a retry/readback process. There is no lease check at the bridge. Therefore, after the coordinator believes the hand window is closed, the hand stack can continue producing CAN traffic.

This violates the intended step-and-settle assumption.

For the MVP, the system must make a deliberate tradeoff:

- after grasp confirmation, the vendor hand controller holds the last setpoint autonomously;
- no periodic hand command is sent while the arm owns the side bus;
- no continuous tactile monitoring is attempted while the same-side arm moves;
- a release or further hand adjustment must reacquire a hand lease;
- optional slip checks may later use explicit diagnostic windows, not invisible background polling.

### 3.5 The OmniHand bridge polls and republishes independently of resource ownership

The bridge currently has:

- one feedback timer at `pub_rate`, default 50 Hz;
- joint SDK readback at `joint_read_rate`, default 20 Hz;
- status SDK reads internally rate-limited to 1 Hz;
- tactile SDK reads internally rate-limited to 1 Hz;
- a separate command-retry timer;
- fault backoff logic.

The fault backoff is sensible, but the normal operating model still performs hand CAN requests while the arm is supposed to own the shared bus. It also allocates and publishes joint-state, status, and tactile messages at 50 Hz even when the underlying data has not changed.

The correct optimization is not merely to lower all rates. Polling should be **state-dependent**:

- no normal hand SDK polling during arm ownership;
- active hand joint/tactile polling only during a hand lease;
- status heartbeat from cached data at a low rate;
- explicit diagnostic probing through a planned resource window.

### 3.6 Coordinator execution is poll-driven and not globally exclusive

The coordinator:

- accepts every activity goal;
- uses a `ReentrantCallbackGroup` and `MultiThreadedExecutor`;
- maintains one global `_open_hand_windows` set;
- runs a 20 Hz polling loop by default;
- polls child futures;
- calls `scheduler.next_batch()` more than once per loop;
- synchronously waits for service futures using `time.sleep()`.

Two activity goals can therefore execute concurrently and mutate shared coordinator state. Resource exclusion is local to each activity scheduler, not global to the unit.

The MVP must allow exactly one active unit activity. Later concurrency can be added only through one global unit resource manager.

### 3.7 MIT trajectory state and driver state are not one contract

The MIT controller listens to `feedback/hand_window_active`, but that topic currently reflects only whether the arm feedback push was silenced, not whether the driver command gate is active. If push silencing fails, the driver can gate MIT commands while the MIT controller continues to believe it is executing normally.

The controller also does not consume the driver's latched fault lockout. A quick recovery can therefore leave the controller's action state inconsistent with the hardware gateway.

### 3.8 Registry and execution profiles still duplicate truth

The motion registry already contains:

- canonical arm joints;
- side prefixes, namespaces, CAN ports, and frames;
- controller name;
- MoveIt groups;
- OmniHand models and active joints.

Execution profiles still repeat CAN ports, namespaces, prefixes, hand types, and launch details in `arm_instances`. Additional duplication exists in resource maps, MoveIt controller YAML, arm configuration, launch helpers, and fallback constants.

The result is a nominal single source of truth with several emergency cousins living nearby.

---

## 4. Design Principles

### 4.1 Fail closed

If ownership, feedback freshness, registry resolution, mode readback, or lease identity is ambiguous, commands are rejected and the system enters a recoverable fault state. The system must never infer permission from the absence of a known error.

### 4.2 States describe authority, not implementation detail

A state such as `HAND_CONTROL` must mean that the hand can actually use the bus, not merely that one Boolean was set or one service returned.

### 4.3 Every resource has one owner

At runtime, exactly one component decides who owns:

- a unit activity slot;
- an arm motion resource;
- a hand motion resource;
- a side CAN bus;
- the vendor SDK session;
- the MoveIt execution channel.

### 4.4 Separate planning, setpoint generation, and hardware authority

- MoveIt plans geometry and dispatches FJT goals.
- The MIT controller turns trajectories into bounded MIT setpoints.
- The arm side authority validates the current control epoch and performs serialized SDK calls.
- The coordinator schedules semantic actions and resources.

No layer silently assumes the responsibilities of another.

### 4.5 Periodic work must justify its rate

Every timer or polling loop must document:

- why it cannot be event-driven;
- what data or control deadline it serves;
- its active and idle rate;
- its worst-case SDK/CAN calls per tick;
- how it backs off on faults;
- which state enables it.

### 4.6 Configuration errors are startup errors

A missing registry entry, unknown motion profile, inconsistent joint order, duplicated controller, or source/install mismatch must abort configuration. Production hardware must not fall back silently to guessed CAN interfaces or legacy joint sets.

---

## 5. Target Layered Architecture

```text
Global Coordinator (future, optional for one unit)
    │
    └── Unit Contract: capabilities, world state, resource reservations,
                       activity execution, health

Unit Coordinator
    ├── Activity DAG and semantic resource scheduling
    ├── One active activity in MVP
    ├── MoveIt arm dispatch
    └── Side lease acquisition for hand actions

MoveIt / move_group
    ├── left_arm planning group
    ├── right_arm planning group
    └── both_arms planning group
          │ FollowJointTrajectory
          ▼
Per-side MIT Controller
    ├── trajectory interpolation
    ├── impedance/gravity command generation
    └── no hardware mode ownership
          │ epoch-tagged MIT command
          ▼
Per-side Hardware Authority
    ├── authoritative side state machine
    ├── SDK serialization
    ├── command validation
    ├── bus recovery / fault lockout / e-stop
    └── hand lease and CAN-push control
          │
          ├── Nero SDK
          └── SideControlState

Per-side Hand Skill Controller
    ├── semantic skill execution
    ├── tactile completion while lease is valid
    └── no background traffic after lease release
          │ lease-tagged hand command
          ▼
Per-side OmniHand Bridge
    ├── lease gate
    ├── vendor mapping
    ├── command verification and bounded retry
    └── adaptive SDK polling only while permitted
```

---

## 6. Closed State Machines

## 6.1 Unit Coordinator State

```text
STARTING
  └──> READY
         ├──> VALIDATING
         │      ├──> READY       validation rejected
         │      └──> EXECUTING   validation passed
         ├──> FAULTED
         └──> ESTOP

EXECUTING
  ├──> CANCELING -> READY
  ├──> READY               successful completion and cleanup
  ├──> FAULTED             child, lease, cleanup, or unit fault
  └──> ESTOP
```

Rules:

- only `READY` accepts an activity goal;
- all other goals are rejected with a structured reason;
- cleanup is part of activity completion;
- an activity cannot succeed if a lease release, arm resume, child cancel, or required final state fails;
- `sync_flag` is strict: if a synchronized execution unit cannot be built, the activity is rejected or aborted;
- every child has acceptance, execution, cancellation, and cleanup deadlines.

## 6.2 Side Hardware Authority State

```text
STARTING
  ├──> DISABLED
  └──> FAULT_LOCKOUT

DISABLED
  └──> ARM_CONTROL

ARM_CONTROL
  ├──> PREPARING_HAND
  ├──> RECOVERING
  ├──> FAULT_LOCKOUT
  ├──> ESTOP
  └──> DISABLED

PREPARING_HAND
  ├──> HAND_CONTROL
  └──> FAULT_LOCKOUT

HAND_CONTROL
  ├──> RESTORING_ARM
  ├──> ESTOP
  └──> FAULT_LOCKOUT

RESTORING_ARM
  ├──> ARM_CONTROL
  └──> FAULT_LOCKOUT

RECOVERING
  └──> FAULT_LOCKOUT

FAULT_LOCKOUT
  ├──> ARM_CONTROL  only via verified rearm transition
  ├──> DISABLED
  └──> ESTOP
```

`HAND_CONTROL` entry conditions:

```text
firmware hold verified
AND non-MIT move mode verified
AND arm command gate active
AND CAN feedback push silence verified
AND side bus quiet criterion verified
AND unique lease created
```

`ARM_CONTROL` re-entry conditions after hand use:

```text
hand command pipeline quiesced
AND pending hand retries cleared
AND hand SDK polling stopped
AND CAN feedback push restored
AND a genuinely new arm feedback frame received
AND driver comm fault absent
AND current arm pose captured as new MIT hold reference
```

Any failed condition transitions to `FAULT_LOCKOUT`, not to a partially resumed state.

## 6.3 MIT Controller State

```text
DISABLED
ARMING
HOLDING
EXECUTING
FREEDRIVE
STANDING_DOWN
FAULTED
```

Rules:

- `EXECUTING`, `HOLDING`, and `FREEDRIVE` require side state `ARM_CONTROL` and a matching control epoch;
- any side transition away from `ARM_CONTROL` aborts the active FJT goal immediately;
- trajectory buffer, start clock, final-hold flag, and stale reference are cleared together;
- on return to `ARM_CONTROL`, the controller waits for fresh feedback and captures a new hold reference;
- the controller never switches firmware mode directly;
- stale feedback produces a bounded dead-man command only while the side authority still permits the same control epoch;
- driver recovery, fault lockout, and e-stop always dominate the MIT state.

## 6.4 OmniHand Skill State

```text
IDLE
EXECUTING
HOLD_LATCHED
FAULTED
```

Rules:

- only one hand action per side is accepted;
- every goal carries a valid hand lease identity;
- `EXECUTING` may publish commands and consume active tactile feedback;
- `HOLD_LATCHED` means the final target was delivered and the vendor controller is expected to maintain it autonomously;
- `HOLD_LATCHED` does not publish recurring commands;
- continuous tactile monitoring is disabled once the lease is released;
- a release or adjustment action acquires a new lease;
- a critical hold-monitoring requirement must explicitly retain the hand resource and block same-side arm motion.

## 6.5 OmniHand Bridge State

The bridge should not invent another complex state machine. It derives permission from `SideControlState` and maintains a small transport state:

```text
BLOCKED
READY_FOR_LEASE
COMMAND_PENDING
COMMAND_VERIFIED
TRANSPORT_FAULT
```

It may call the vendor SDK only when:

- the side state is `HAND_CONTROL`;
- the command lease matches the active lease;
- the lease has not expired;
- the command sequence is newer than the last accepted sequence.

---

## 7. Minimal Lease-Based Handshake

The CAN workaround is temporary, so the handshake should remain compact. It nevertheless needs identity and expiry to prevent stale asynchronous messages from crossing ownership transitions.

## 7.1 Acquire Hand Lease Action

Proposed action: `AcquireHandControl.action`

```text
# Goal
string requester_id
string activity_id
float32 requested_ttl_s

---
# Result
bool success
string lease_id
uint64 control_epoch
builtin_interfaces/Time expires_at
bool hold_verified
bool bus_quiet_verified
string failure_reason

---
# Feedback
uint8 phase
string detail
```

Phases:

```text
GATING_ARM
DAMPING_MIT
COMMANDING_FIRMWARE_HOLD
VERIFYING_HOLD
SILENCING_ARM_PUSH
VERIFYING_BUS_QUIET
LEASE_GRANTED
```

The action is preferable to a `Trigger` service because the transition has meaningful phases, takes bounded time, and must be cancelable and observable.

## 7.2 Release Hand Lease Service

Proposed service: `ReleaseHandControl.srv`

```text
string lease_id
uint64 control_epoch
---
bool success
bool hand_quiesced
bool arm_feedback_restored
bool arm_hold_recaptured
uint64 new_control_epoch
string failure_reason
```

A stale or mismatched lease cannot release a newer owner.

## 7.3 Epoch-tagged commands

Add `control_epoch` to the arm command contract and `lease_id` plus `sequence` to the hand command contract.

Proposed replacement for unrestricted hand `JointState` commands:

```text
# OmniHandCommand.msg
std_msgs/Header header
string lease_id
uint64 control_epoch
uint64 sequence
string[] joint_names
float64[] positions
```

The bridge rejects:

- a missing or expired lease;
- a mismatched epoch;
- duplicate or older sequences;
- unknown, duplicate, non-finite, or out-of-limit joint values.

For MIT commands, either extend `MoveMITMsg` or introduce `ArmMitCommand.msg` with:

```text
uint64 control_epoch
uint64 sequence
int32[] joint_index
float64[] p_des
float64[] v_des
float64[] kp
float64[] kd
float64[] torque
```

The side authority validates state and epoch inside the same serialized hardware operation that sends the command. This closes the race between a callback precondition check and a concurrent handover.

---

## 8. Ownership Matrix

| Layer | Owns | May request | Must not own |
|---|---|---|---|
| Future global coordinator | Unit assignment, global resources, cross-unit task decomposition | Unit activities and reservations | Local CAN or joint control |
| Unit coordinator | Activity DAG, unit activity slot, semantic resource schedule | MoveIt goals, hand leases, child cancel | SDK calls, MIT gains, vendor hand mapping |
| MoveIt | Collision-aware arm planning and trajectory dispatch | FJT execution | Hand leases, activity semantics, recovery |
| MIT controller | Trajectory interpolation, hold, impedance and feedforward setpoints | Epoch-tagged MIT commands | Firmware mode, CAN recovery, re-enable |
| Side hardware authority | Side state, SDK session, CAN mode, arm command gate, lease, e-stop, recovery | None below SDK | Activity planning, semantic skills |
| Hand skill controller | Semantic skill progression and tactile completion | Lease-tagged hand commands | Side ownership, vendor SDK session |
| OmniHand bridge | Vendor conversion, command delivery verification, bounded retry | Vendor SDK calls while leased | Semantic completion policy, resource scheduling |
| Registry resolver | Static configuration resolution and validation | Generated launch/config artifacts | Runtime ownership or fault decisions |

---

## 9. Resource Model

The existing scheduler correctly recognizes physical intersections such as `left_arm`, `left_hand`, and `left_can_bus`. This model should be moved out of hardcoded Python and into the system registry.

Proposed registry resource section:

```yaml
resources:
  unit_activity_slot:
    capacity: 1

  left_arm_motion:
    capacity: 1
  right_arm_motion:
    capacity: 1
  left_hand_motion:
    capacity: 1
  right_hand_motion:
    capacity: 1

  left_can_bus:
    capacity: 1
    policy: step_and_settle
  right_can_bus:
    capacity: 1
    policy: step_and_settle

claims:
  left_arm:
    - left_arm_motion
    - left_can_bus
  right_arm:
    - right_arm_motion
    - right_can_bus
  both_arms:
    - left_arm_motion
    - right_arm_motion
    - left_can_bus
    - right_can_bus
  left_hand:
    - left_hand_motion
    - left_can_bus
  right_hand:
    - right_hand_motion
    - right_can_bus
```

The scheduler consumes resolved claims rather than a module-level `ROBOT_UNITS` constant.

A hand hold after command completion does **not** continue claiming `left_can_bus` or `right_can_bus` if the vendor hardware can maintain the target without communication. It records a semantic state, not a communication resource. If a future skill requires continuous tactile observation or active hand adjustment, its action remains running and retains the bus claim.

---

## 10. Registry, Profiles, and Resolved Manifest

## 10.1 System Registry: physical and geometric truth

The registry is authoritative for:

- unit and device identities;
- namespaces;
- side mapping;
- CAN interfaces;
- canonical joints and prefixes;
- base/tip frames;
- MoveIt group names and joint ordering;
- controller action names;
- hand model and active joints;
- resource definitions and claims;
- capabilities and hardware restrictions.

It must not contain per-run choices such as whether a mock backend is launched.

## 10.2 Execution Profiles: selection only

Execution profiles should select a composition:

```yaml
profiles:
  duo_hand_hardware:
    unit: duo_01
    enabled_devices: [left_arm, right_arm, left_hand, right_hand]
    arm_backend: hardware
    hand_backend: sdk
    launch_moveit: true
    launch_coordinator: true

  duo_arm_hardware:
    unit: duo_01
    enabled_devices: [left_arm, right_arm]
    arm_backend: hardware
    launch_moveit: true

  duo_hand_mock:
    unit: duo_01
    enabled_devices: [left_arm, right_arm, left_hand, right_hand]
    arm_backend: mock
    hand_backend: mock
```

Profiles must not repeat CAN ports, prefixes, frames, controller names, or joint lists.

## 10.3 Resolved immutable manifest

A single resolver loads:

```text
system registry + selected execution profile + explicit launch overrides
```

It validates them and emits a `ResolvedSystemManifest` containing every final runtime value.

The manifest should include:

- `schema_version`;
- `manifest_id`;
- `content_hash`;
- source file paths and hashes;
- enabled devices;
- full node namespaces;
- controller action names;
- joint lists and order;
- CAN ports;
- resource claims;
- generated xacro arguments;
- generated MoveIt controller entries;
- runtime rates and budgets.

All nodes log and publish the same manifest hash. A node receiving a command or state from a mismatched manifest must refuse activation.

## 10.4 Production path resolution

Production launch must choose exactly one configuration origin:

- installed package share; or
- explicit development source path.

It must not combine source execution profiles with installed registries. Silent fallback to built-in CAN interfaces is allowed only in mock/test profiles. Hardware profiles fail at configuration time.

## 10.5 Generated artifacts

Generate, rather than maintain manually:

- MoveIt simple-controller-manager YAML;
- side arm controller action mappings;
- joint-state merger mappings;
- xacro arguments;
- scheduler resource claims;
- controller joint lists;
- launch parameter dictionaries.

Generated files should carry:

```text
DO NOT EDIT
manifest_id
source registry hash
profile name
```

---

## 11. MoveIt Integration Contract

MoveIt remains one instance per Duo unit in the MVP.

Required groups:

- `left_arm`;
- `right_arm`;
- `both_arms`.

Required runtime arm controllers:

- `/left_arm/arm_controller/follow_joint_trajectory`;
- `/right_arm/arm_controller/follow_joint_trajectory`.

A `both_arms` plan is split by MoveIt across these two controllers. There should be no duplicate custom `both_arms` execution path.

The semantic hand-skill route should be the sole production hand execution path. If MoveIt hand controllers are generated for experiments, they must be disabled or non-default in coordinated production profiles. Otherwise, MoveIt and the skill controller become competing hand-command sources.

Strict sync behavior:

- two per-arm actions with the same `sync_flag` must be merged into one `both_arms` MoveIt or ExecuteTrajectory goal;
- if merge or group coverage fails, execution fails closed;
- independent dispatch is not a valid fallback for strict synchronization.

---

## 12. CPU and Runtime-Efficiency Refactor

The optimization goal is not simply “lower every rate.” The goal is to remove duplicated work, align rates with consumers, and make work conditional on ownership.

## 12.1 Establish a baseline first

Record per-process and per-thread measurements for:

1. complete idle stack;
2. two arms holding in firmware mode;
3. one MIT arm active;
4. two MIT arms active;
5. one hand skill window;
6. simultaneous dual-side hand windows if permitted;
7. bus-fault and recovery scenarios.

Recommended evidence:

```text
tegrastats
pidstat -p ALL -t 1
perf stat / perf record where available
ros2 topic hz and bw
SocketCAN frame counts and bus load
loop overrun counters
executor callback durations
```

Report CPU as both:

- percentage of one logical CPU per process; and
- total Jetson CPU utilization.

This avoids the charming ambiguity of “100% CPU,” which can mean one core or the entire machine.

## 12.2 Arm driver: replace the 200 Hz all-purpose publisher loop

The current loop performs readiness checks and repeatedly fetches joint angles, seven motor states, pose, arm status, effector status, and leader feedback. Several getters are called more than once per iteration, and the code comments already document GIL starvation during MIT streaming.

Refactor to one serialized hardware worker per arm:

```text
HardwareWorker
├── command queue, awakened immediately by new command
├── periodic feedback snapshot deadline
├── periodic status deadline
└── recovery transition events
```

The worker uses a condition variable:

```python
condition.wait(timeout=next_deadline - now)
```

It does not spin continuously.

Create one immutable `ArmFeedbackSnapshot` per acquisition cycle containing:

- timestamp;
- joint positions;
- real or estimated velocities;
- efforts;
- arm status;
- mode status;
- feedback freshness;
- communication status.

All publishers and safety checks consume this snapshot. No callback directly calls SDK getters.

Initial proposed rates, subject to measurement:

| Function | Current | Proposed idle | Proposed active |
|---|---:|---:|---:|
| arm joint feedback acquisition | coupled to 200 Hz loop | 25–50 Hz | 50–100 Hz |
| ROS arm joint-state publication | 200 Hz | 25 Hz | 50 Hz, matched to MIT |
| TCP pose publication | 200 Hz path | 10 Hz | 20–25 Hz |
| arm status publication | 200 Hz path | 2–5 Hz or on change | 10 Hz |
| diagnostics/recovery counters | 200 Hz path | 1 Hz | 1 Hz |
| leader feedback publication | every loop | disabled unless leader mode | 50 Hz in leader mode |

The exact active rate must be set by the required MIT control performance. Publishing at four times the MIT consumer rate without a demonstrated benefit should not be the default.

## 12.3 Serialize all SDK operations

MVP implementation:

- all hardware operations enter one queue;
- only the hardware worker calls `agx_arm`;
- emergency-stop requests use a priority queue lane;
- state transitions increment `control_epoch`;
- queued commands with an old epoch are discarded;
- recovery drains normal motion commands before reconnecting.

A single `RLock` is acceptable as a very short interim patch, but the queue is the target because it also provides deterministic ordering and observability.

## 12.4 MIT controller: one trajectory sampler, event-driven action completion

Currently, both the control timer and the blocking action execution loop sample and evaluate trajectory state. The action loop wakes every 20 ms or faster, while the control loop already operates at the actual control cadence.

Refactor:

- the control timer is the sole trajectory sampler;
- it writes one `ExecutionSnapshot` containing desired, actual, errors, elapsed time, and terminal status;
- the action execute callback waits on a condition/event signaled by state change, cancel, feedback publication deadline, or terminal result;
- path and goal tolerance checks occur once per control tick;
- action feedback is emitted from the same snapshot at a decimated rate;
- side-state changes signal immediate abort without waiting for feedback staleness.

The controller should publish MIT commands only when the reference changes or on a documented firmware watchdog/dead-man cadence. If the firmware requires continuous streaming, keep that cadence explicit; otherwise avoid unnecessary repeated identical messages.

## 12.5 Coordinator: event-driven child completion

Replace the 20 Hz activity polling loop with:

- `Future.add_done_callback()` for goal acceptance and result completion;
- an internal event queue;
- scheduler invocation only on activity start, child completion, cancellation, lease transition, or timeout;
- a low-rate watchdog timer only for deadlines;
- no repeated `next_batch()` calls in the same unchanged state;
- no `time.sleep()` inside callback paths.

This removes one long-lived executor thread per active activity and prevents multiple activity loops from racing.

## 12.6 OmniHand bridge: ownership-aware adaptive polling

The bridge should receive `SideControlState` and use the following policy:

### ARM_CONTROL

- reject all hand commands;
- cancel or clear pending command retries;
- perform no normal hand joint, tactile, status, or error-report CAN requests;
- publish cached low-rate status heartbeat only;
- allow diagnostic probing only through an explicit diagnostic hand lease.

### PREPARING_HAND

- remain blocked until lease grant;
- do not preemptively poll.

### HAND_CONTROL

- accept only matching lease-tagged commands;
- perform joint readback at the rate needed for command verification;
- perform tactile readback at the skill's requested bounded rate;
- status/error-report reads run at a low rate or only when joint/tactile calls fail;
- stop retrying immediately after verification or lease expiry.

### RESTORING_ARM

- stop SDK polling;
- clear pending commands;
- acknowledge transport quiescence to the side authority;
- reject late command messages from the old lease.

This policy makes hand CAN traffic proportional to actual hand use, which is both safer and substantially cheaper.

## 12.7 Split bridge timers by data semantics

Do not construct all feedback messages from one 50 Hz timer.

Suggested structure:

- command verification / active joint read timer: enabled only during `HAND_CONTROL`;
- tactile timer: enabled only when requested by the active skill;
- status heartbeat timer: 1–2 Hz from cache;
- retry deadline integrated into the active command state instead of a permanently waking timer;
- publish joint states only on new SDK readback plus a low-rate heartbeat if required by consumers;
- publish tactile only on a new tactile sample;
- publish status on change plus heartbeat.

## 12.8 Remove periodic hand hold commands

Delete the production behavior in `_hold_tick()` that republishes the target continuously.

On grasp confirmation:

1. send the final target;
2. wait until the bridge verifies delivery;
3. latch semantic `HOLD_LATCHED` state;
4. release the hand lease;
5. publish no more hand commands until the next leased skill.

The current bridge already sends position setpoints through a vendor controller and its `stop()` implementation holds the current pose. The MVP must verify experimentally that the hand maintains the grasp without periodic host commands. If it does not, then same-side arm motion and active hand hold are not simultaneously supportable under the current CAN limitation and must remain mutually exclusive resources.

## 12.9 Avoid feedback duplication

The arm driver currently merges cached OmniHand joints into its arm joint-state output. At the same time, the bridge publishes hand joint states and the Duo launch already refers to a joint-state merger.

Target:

- arm driver publishes arm joints only;
- hand bridge publishes hand joints only;
- one unit-level joint-state aggregator produces the combined MoveIt state at a bounded rate;
- static hand values are not recopied at the arm driver's high rate;
- the aggregator validates uniqueness and manifest joint order.

## 12.10 Executor and process policy

For the Python MVP:

- use a single-threaded executor for nodes that do not require concurrent blocking work;
- use explicit callback groups for the unit coordinator and MIT controller;
- set a bounded thread count rather than the executor default;
- never rely on concurrent callbacks for hardware safety;
- isolate each vendor SDK session in its own process.

Do not compose both OmniHand SDK bridges into one Python process while the vendor interface selection depends on the process-global `OMNIHAND_SOCKETCAN_IFACE` environment variable. The explicit backend constructor should eventually own interface selection without global environment mutation.

A C++ rewrite of the high-frequency arm gateway may later provide additional deterministic performance, but it is not required for the first architectural MVP. The Python path should first remove obvious duplicated and unconditional work.

---

## 13. Safety and Validation Improvements

## 13.1 Command validation at the hardware boundary

The arm side authority must reject:

- wrong control epoch;
- duplicate or missing joint indexes;
- empty commands;
- non-finite values;
- out-of-range positions, velocity, gains, and torque;
- incomplete position commands unless explicitly defined as partial;
- commands in any state other than `ARM_CONTROL`;
- stale sequences.

The hand bridge must apply equivalent validation for hand commands.

SDK clamping remains a final hardware protection, not the primary input contract. Invalid values should not be silently transformed into plausible maximum commands.

## 13.2 Fault status contract

`AgxArmStatus.msg` defines an integer `err_status`, but the ROS driver does not assign it from the SDK's structured error object. Replace this ambiguity with either:

- a documented `uint64 error_bits`; or
- explicit structured fields.

The MIT controller should enter `FAULTED` when any of the following is true:

- vendor arm status is not normal;
- any joint communication or limit fault is active;
- side authority is in `RECOVERING`, `FAULT_LOCKOUT`, or `ESTOP`;
- the manifest/control epoch is invalid.

## 13.3 Fault acknowledgement versus verified rearm

Do not use one service to both acknowledge and clear a fault.

Proposed sequence:

```text
acknowledge_fault
verify feedback and vendor status
verify arm is stationary using trustworthy velocity evidence
capture current pose
establish firmware hold
enable MIT controller with a new epoch
transition to ARM_CONTROL
```

A failed condition leaves the side in `FAULT_LOCKOUT`.

## 13.4 One movement ingress in production

The arm driver currently exposes many control topics and services, including general joint state control, `move_j`, `move_js`, pose moves, line/circle moves, and MIT.

Production coordinated profiles should expose only:

- epoch-tagged MIT ingress from the MIT controller;
- emergency stop;
- lifecycle/enable/recovery services;
- leased handover services.

Legacy direct motion topics may remain behind a `development_legacy_commands` profile, disabled by default. This removes accidental bypasses such as `move_home` during a hand window or lockout.

---

## 14. Proposed Code Changes by Component

## 14.1 `agx_arm_ctrl_single_node.py`

- replace Boolean mode collection with `SideControlState` enum plus epoch;
- introduce one hardware worker and priority command queue;
- remove direct SDK calls from ROS callbacks;
- validate all command arrays and finite values;
- make all direct legacy motion APIs development-only;
- replace Trigger hand-window services with acquire/release lease contract;
- require verified bus silence for hand lease success;
- publish authoritative transient-local side state;
- consume hand-bridge quiescence acknowledgement during release;
- fix fault acknowledgement/rearm separation;
- use one feedback snapshot and decimated publishers;
- remove hand joint aggregation from arm joint-state publication;
- fix firmware version parsing using numeric tuples;
- require actual enable readback for success;
- treat forced e-stop recovery independently of the optional normal recovery setting.

## 14.2 Vendor Nero driver

- remove the unconditional `motor_state.msg.velocity = 0.0` override;
- confirm protocol sign and scaling for velocity;
- add a unit/hardware test against position-derived velocity;
- expose thread-safety guarantees or explicitly document non-thread-safe behavior;
- add a public push-control API so repository code no longer mutates `_msg_mode` privately;
- make mode-message updates atomic inside the SDK;
- expose immutable feedback snapshots where possible.

## 14.3 `mit_controller_node.py`

- consume `SideControlState` instead of separate hand-window and inferred fault flags;
- include control epoch and sequence in every command;
- abort FJT immediately when arm authority is lost;
- make the control timer the single trajectory evaluator;
- use event-driven action result waiting;
- perform atomic live parameter validation and reject NaN/Inf;
- remove unreachable log code and redundant state-publication condition;
- publish execution state only on change plus optional low-rate heartbeat;
- treat velocity tolerances as unavailable until velocity feedback is trustworthy.

## 14.4 `coordinator_node.py`

- add one global unit activity lock/state;
- reject concurrent activity goals;
- use event callbacks instead of 20 Hz future polling;
- add child deadlines and cleanup deadlines;
- acquire a side lease before every hand action;
- pass lease identity to the hand skill;
- treat lease release failure as activity failure;
- never discard ownership bookkeeping after a failed release;
- require strict sync merge for `sync_flag` groups;
- convert cancellation into a bounded state transition and verify child termination;
- derive resources from resolved manifest;
- publish one unit state and structured failure reason.

## 14.5 `omnihand_skill_controller_node.py`

- reject concurrent goals;
- require lease metadata and verify expiry;
- publish lease-tagged `OmniHandCommand`, not shared `JointState`;
- remove recurring hold command publication;
- stop continuous tactile monitoring after lease release;
- distinguish `commanded`, `delivery_verified`, and `contact_confirmed` completion;
- do not report successful action completion until the bridge has verified the final command;
- make fallback outcomes explicit: a timed-out grasp held without contact should not be returned as a normal success;
- expose semantic hold state without claiming the CAN resource.

## 14.6 `omnihand_bridge_node.py`

- subscribe to authoritative side state;
- reject commands without matching lease and epoch;
- gate all SDK calls by ownership state;
- split feedback, status, tactile, and retry scheduling;
- poll only during a valid hand lease;
- publish on new data/change plus heartbeat;
- clear pending retries before arm restoration;
- acknowledge transport quiescence;
- avoid SDK read-before-write for full-joint commands;
- use cached positions for partial commands and reject partial commands without a valid cache;
- make production CAN mapping fail hard rather than fall back;
- ensure O12 Pro backend follows the same gating and freshness contract;
- remove process-global interface selection from the long-term backend API.

## 14.7 Registry and profiles

- create one shared resolver package used by control, coordination, description, and MoveIt;
- bump registry schema version;
- add resources, claims, controller action paths, and unit identity;
- reduce profiles to selection only;
- generate MoveIt controller YAML;
- remove duplicate path-resolution implementations;
- validate source/install mode;
- add manifest hash and startup consistency check.

---

## 15. Migration Plan

## Phase 0 — Instrumentation and release blockers

**Purpose:** Make measurements and existing safety claims trustworthy.

Tasks:

1. fix or replace motor velocity feedback;
2. add process/thread CPU and loop-duration instrumentation;
3. log all SDK calls with thread ID in a stress build;
4. record CAN traffic by direction and component;
5. establish baseline scenarios;
6. disable “verified stopped” wording until velocity is fixed;
7. identify the actual generated Duo MoveIt controller configuration.

Exit criteria:

- velocity accuracy is validated against differentiated encoder positions;
- no safety check consumes known synthetic zero velocities;
- CPU/CAN baseline report exists;
- actual launch artifacts and registry hashes are known.

## Phase 1 — Authoritative side state and serialized SDK access

**Purpose:** Close the largest ownership and race gaps.

Tasks:

1. implement `SideControlState` with epoch;
2. introduce hardware worker/command queue;
3. route all SDK calls through the worker;
4. make MIT commands epoch-tagged;
5. make MIT controller abort on side-authority loss;
6. separate fault acknowledgement and verified rearm;
7. disable production legacy motion ingress.

Exit criteria:

- no SDK call occurs outside the hardware worker;
- old-epoch commands are dropped in tests;
- driver, MIT controller, and coordinator agree on every tested transition;
- recovery cannot silently resume an old trajectory.

## Phase 2 — Leased hand control and zero background hand traffic

**Purpose:** Make the CAN workaround deterministic and resource-bounded.

Tasks:

1. implement acquire/release hand lease;
2. require hold and bus-quiet verification;
3. pass lease through coordinator, skill, and bridge;
4. gate bridge SDK access;
5. replace shared hand `JointState` commands;
6. remove periodic skill hold commands;
7. stop all hand polling after release;
8. fail activity on release/resume failure.

Exit criteria:

- zero hand TX frames during same-side arm ownership, excluding explicitly approved diagnostic windows;
- stale lease commands are rejected;
- hand action completion guarantees final command delivery or returns failure;
- arm resume is verified before the next arm action dispatches.

## Phase 3 — Registry and profile consolidation

**Purpose:** Enforce configuration truth and remove legacy leftovers.

Tasks:

1. implement shared registry schema and resolver;
2. reduce execution profiles to composition selection;
3. generate MoveIt controllers and launch parameters;
4. move resource claims out of Python constants;
5. add manifest hash exchange;
6. fail on path/source mismatches;
7. remove or clearly quarantine standalone legacy configuration.

Exit criteria:

- changing a side prefix or CAN port requires one registry edit;
- every runtime node reports the same manifest hash;
- generated MoveIt joint lists exactly match registry order;
- no production node uses a built-in hardware fallback.

## Phase 4 — Event-driven runtime and rate consolidation

**Purpose:** Reduce Jetson CPU load without weakening control behavior.

Tasks:

1. replace coordinator polling with callbacks and event queue;
2. centralize MIT trajectory sampling;
3. decimate arm feedback/status/pose publishers;
4. split bridge timers and activate them by lease;
5. remove duplicate hand-to-arm joint aggregation;
6. publish status/state on change plus heartbeat;
7. bound executor thread counts;
8. remeasure all baseline scenarios.

Exit criteria:

- no unconditional high-rate loop performs non-control work;
- idle hand SDK CAN polling is zero;
- no duplicate trajectory sampling loop exists;
- loop-overrun and callback-duration metrics meet the chosen control budget;
- CPU utilization improves against Phase 0 baseline.

## Phase 5 — Multi-unit-ready contract

**Purpose:** Freeze the local unit boundary before adding more units.

Tasks:

1. namespace every public unit interface under `/unit_<id>`;
2. expose unit capabilities, state, world-state timestamp, and activity action;
3. separate internal side topics from public unit topics;
4. define global resource identifiers and workspace reservations;
5. create a mock global coordinator that delegates to one unit.

Exit criteria:

- the current Duo stack runs unchanged under a unit namespace;
- a higher-level client can treat the whole Duo system as one executor;
- no global client needs access to arm or hand vendor topics.

---

## 16. Verification and Failure-Injection Plan

## 16.1 State-machine conformance

Generate a transition table and test every legal and illegal transition.

Examples:

- `ARM_CONTROL -> HAND_CONTROL` succeeds only after all entry evidence;
- `HAND_CONTROL -> ARM_CONTROL` fails if the bridge still has a pending command;
- `FAULT_LOCKOUT -> ARM_CONTROL` fails without verified rearm;
- direct `DISABLED -> HAND_CONTROL` is rejected;
- stale release lease cannot close a newer lease.

## 16.2 SDK serialization stress test

Run dual-arm MIT streaming while repeatedly requesting:

- hand leases;
- lease releases;
- emergency stops;
- recoveries;
- enable/disable transitions.

Instrument SDK call sequence, thread ID, epoch, and state. Acceptance criterion: all SDK calls for one arm originate from one worker thread and no old-epoch motion is sent after a transition.

## 16.3 Hand traffic test

For each side:

1. acquire hand lease;
2. execute grasp;
3. verify final target;
4. release lease;
5. execute an arm trajectory while recording CAN.

Acceptance criterion:

```text
hand TX frames during ARM_CONTROL = 0
hand SDK read requests during ARM_CONTROL = 0
```

Any exception must be explicitly identified as a leased diagnostic window.

## 16.4 Resume failure test

Force `ReleaseHandControl` to fail at each phase:

- hand pending command not cleared;
- push restore failed;
- no new arm feedback;
- comm fault latched;
- hold-reference recapture failed.

Acceptance criterion: the activity fails, the side remains locked, and no subsequent arm goal is dispatched.

## 16.5 Stale command test

Queue MIT and hand messages immediately before ownership transitions and delay their delivery artificially.

Acceptance criterion: mismatched epoch/lease commands are rejected at the hardware boundary.

## 16.6 Strict synchronization test

Create a synced left/right pair and force merge failure through:

- mixed plan types;
- joint-order mismatch;
- missing both-arms group;
- untaught trajectory.

Acceptance criterion: no independent fallback execution occurs.

## 16.7 Velocity and stop validation

Compare:

- protocol velocity;
- finite-difference position velocity;
- external observation where available.

Test stop verification under actual motion, stale feedback, and dropped stop frames. A software stop may return verified only with fresh, trustworthy velocity evidence.

## 16.8 CPU and scheduling tests

Measure:

- process CPU;
- thread CPU;
- callback execution time;
- timer jitter;
- missed deadlines;
- ROS message rate/bandwidth;
- CAN frame count.

Run before/after comparisons for each migration phase.

Initial proposed gates, to be adjusted after Phase 0 baseline:

- idle stack CPU reduced by at least 30% relative to baseline;
- active dual-arm MIT CPU reduced by at least 20% or demonstrably shifted from Python polling into required control work;
- no single non-control callback regularly exceeds 20% of its period budget;
- arm feedback/control deadline miss rate below 1% during the defined stress scenario;
- no recovery triggered solely by local executor starvation;
- no hand CAN traffic outside a valid lease.

These percentages are engineering gates, not claims about current performance.

## 16.9 Registry drift test

Repository-wide CI should search for duplicated authoritative values:

- canonical joint lists;
- side prefixes;
- CAN interfaces;
- controller names;
- MoveIt group names;
- resource claims.

Allowed duplicates must be generated artifacts or explicit test fixtures carrying a manifest/source marker.

---

## 17. Future Multi-Unit Coordination

The recommended future architecture is hierarchical and centrally assigned at the global level:

```text
Global Coordinator / Global World Model
        │
        ├── Unit A Coordinator + MoveIt A + Jetson A
        ├── Unit B Coordinator + MoveIt B + Jetson B
        └── Unit C Coordinator + MoveIt C + Jetson C
```

### 17.1 Why not one giant MoveIt instance

A single global MoveIt instance would provide one planning scene but would tightly couple:

- network availability;
- robot-model size;
- controller connectivity;
- planning latency;
- failure domains.

It also makes each local unit less autonomous.

### 17.2 Why not fully distributed peer coordination first

Peer-to-peer negotiation requires additional mechanisms:

- leader election;
- conflict resolution;
- distributed leases;
- consistent world-state timestamps;
- partition handling;
- duplicate task prevention;
- recovery after coordinator loss.

This is disproportionate for the current MVP.

### 17.3 Recommended split

Global coordinator owns:

- task decomposition across units;
- global workspace/resource reservations;
- assignment of object/task responsibility;
- cross-unit barriers;
- global world-state authority or fusion policy.

Each unit owns:

- local MoveIt planning;
- local collision model and execution;
- local activity graph;
- local side resources and safety;
- detailed arm/hand control.

For tightly coupled cross-unit motion, add a later explicit cooperative-plan mode. Do not make all normal tasks depend on global joint-level planning.

### 17.4 Namespaces and deployment roles

Prepare now for:

```text
/unit_01/execute_activity
/unit_01/state
/unit_01/capabilities
/unit_01/world_state
/unit_01/left_arm/...
/unit_01/right_arm/...
```

The global coordinator role should initially be a configured deployment role, not dynamic leader election. Unit contracts should nevertheless avoid hardcoding the host so the role can later move or gain redundancy.

---

## 18. Non-Goals for the First MVP

The following are intentionally excluded:

- conversion of MIT control to `ros2_control`;
- full C++ rewrite;
- dynamic multi-Jetson leader election;
- distributed consensus;
- global multi-unit joint-level planning;
- continuous tactile slip monitoring during same-side arm motion;
- arbitrary simultaneous arm and hand traffic on the limited shared CAN bus;
- runtime registry mutation;
- multiple concurrent activities within one unit.

These exclusions keep the first refactor focused on correctness, determinism, and measurable resource savings.

---

## 19. Deliverables

1. Architecture decision record covering ownership and layer boundaries.
2. State-machine specification and transition table.
3. `SideControlState.msg` and lease action/service definitions.
4. Serialized arm hardware worker with epoch validation.
5. MIT controller integration with authoritative side state.
6. Lease-aware OmniHand skill and bridge.
7. Removal of recurring hand hold traffic.
8. Shared registry resolver and versioned schema.
9. Reduced execution profiles and generated MoveIt controller config.
10. Event-driven coordinator with one activity slot and bounded child deadlines.
11. CPU/CAN baseline and post-refactor comparison report.
12. Failure-injection and state-conformance test suite.
13. Multi-unit public unit contract skeleton.

---

## 20. Recommended Implementation Order

The recommended order is intentionally strict:

1. **Fix velocity feedback and stop verification.**
2. **Instrument CPU, SDK calls, and CAN traffic.**
3. **Introduce serialized SDK ownership and side state/epoch.**
4. **Bind MIT execution to the authoritative state.**
5. **Implement hand lease and bridge gating.**
6. **Remove background hand command and polling traffic.**
7. **Make coordinator event-driven and single-activity.**
8. **Consolidate registry/profile resolution and generate MoveIt config.**
9. **Decimate and consolidate feedback publication.**
10. **Freeze the unit-level public contract for future multi-unit coordination.**

This order prevents configuration cleanup or CPU tuning from being built on top of unsafe state assumptions.

---

## 21. Agent Discovery Instructions

### Actual Duo MoveIt configuration

> Locate every launch/config builder that produces `moveit_simple_controller_manager.controller_names`, controller action namespaces, and joint lists for the `duo_hand` profile. Resolve the final runtime configuration and verify that only the two namespaced arm FJT controllers are used for arm execution. Identify whether any hand FJT controller remains active alongside the semantic hand-skill path. Report every use of the standalone unprefixed `moveit_controllers.yaml`.

### Hand traffic after grasp

> Trace `OmniHandSkillController._hold_tick()` through the command topic, bridge callback, retry logic, and O12 Pro backend send function. Measure hand TX and read-request frames after a grasp action has returned and after `resume_arm_control`. Determine whether the vendor controller maintains the final pose without recurring host commands. Acceptance criterion for the proposed MVP: zero recurring hand traffic during arm ownership.

### CPU attribution

> Measure CPU per process and per thread for both arm drivers, both MIT controllers, both hand bridges, both skill controllers, MoveIt, joint-state merger, and coordinator in the defined baseline scenarios. Attribute wakeups to timers, polling loops, SDK getters, message serialization, logging, and action loops. Produce a ranked list of the ten highest CPU consumers with evidence.

### Registry duplication

> Search the complete workspace for CAN port names, arm prefixes, canonical joint lists, controller names, MoveIt groups, frame names, OmniHand joint lists, and resource claims. Classify each occurrence as authoritative source, generated artifact, test fixture, or unauthorized duplicate. Report source/install path-resolution differences.

### O12 Pro backend contract

> Inspect `O12ProSdkBackend` for command send, readback, status, tactile, interface selection, locking, and error behavior. Verify that no internal background thread or periodic SDK poll continues after the bridge stops calling it. Confirm whether full-joint setpoints can be sent without a preceding read request and whether the hand holds the last setpoint autonomously.

---

## 22. Final Recommendation

Proceed with the refactor as one coordinated MVP rather than a collection of further local patches.

The key architectural unit is the **side hardware authority** with a closed state machine, serialized SDK ownership, and an explicit control epoch. The key temporary CAN mechanism is a **verified, expiring hand lease**. The key configuration mechanism is an **immutable resolved manifest** produced from one physical registry and one selection-only execution profile. The key performance mechanism is **ownership-aware, event-driven work**: the hand does nothing to the bus while the arm owns it, status work is decimated, trajectory state is evaluated once, and coordinator progress is triggered by events rather than polling.

This produces a local Duo unit that is deterministic enough to be composed later. A future global coordinator can then coordinate units through stable contracts instead of inheriting the current internal ambiguity at a larger and considerably more expensive scale.

---

## Appendix A — Evidence and Traceability Map

The proposal is grounded in the reviewed implementation snapshot. Key evidence:

| Finding | Implementation evidence |
|---|---|
| Synthetic zero arm velocity | `driver.py:500-540` overwrites `motor_state.msg.velocity` with `0.0` |
| Stop verification consumes that velocity | `agx_arm_ctrl_single_node.py:1738-1770` polls `get_motor_states()` and accepts all velocities below threshold |
| Dedicated high-rate arm publisher thread | `agx_arm_ctrl_single_node.py:81`, `748-817`; default `pub_rate=200` |
| Arm loop performs repeated feedback work | `agx_arm_ctrl_single_node.py:788-810`, `1228-1270` |
| Hand-window topic does not represent the full gate | `agx_arm_ctrl_single_node.py:636-658` publishes `_hand_window_push_silenced` |
| Hand window can report success without verified silence | `agx_arm_ctrl_single_node.py:2248-2269` |
| Coordinator release is best-effort and discards bookkeeping | `coordinator_node.py:296-307` |
| Coordinator accepts every activity goal | `coordinator_node.py:216-226` |
| Coordinator uses periodic polling and sleeps | `coordinator_node.py:522-589` |
| MIT controller has a separate periodic control loop | `mit_controller_node.py:247-249`, `1013+` |
| MIT action execution also polls | `mit_controller_node.py:817-970` |
| Skill controller accepts every hand goal | `omnihand_skill_controller_node.py:268-274` |
| Skill performs blocking rate loops | `omnihand_skill_controller_node.py:342-393`, `411-461` |
| Skill republishes grasp hold indefinitely | `omnihand_skill_controller_node.py:551-585` |
| Bridge accepts hand commands without lease | `omnihand_bridge_node.py:956-968`, `982-1027` |
| Bridge immediately submits/retries each accepted target | `omnihand_bridge_node.py:1029-1122` |
| Bridge feedback and retry timers are independent | `omnihand_bridge_node.py:970-973` |
| Bridge defaults to 50 Hz publication and 20 Hz joint polling | `omnihand_bridge_node.py:796-825` |
| Bridge performs status/tactile caching but still republishes at the main rate | `omnihand_bridge_node.py:1170-1268` |
| Registry already owns side and motion geometry | `duo_motion_registry.yaml:20-80` |
| Execution profiles repeat side runtime mapping | `execution_profiles.yaml:68-124` |
| Standalone MoveIt config is unprefixed and includes legacy gripper controller | `moveit_controllers.yaml:1-27` |
| Scheduler resources are hardcoded in Python | `graph_model.py:27-52` |
| Strict sync currently has an independent-dispatch fallback in coordinator | `coordinator_node.py:420-481` |

Line numbers refer to the reviewed uploaded snapshot and will naturally move during implementation. CI tests and manifest validation should replace line-based traceability once the refactor begins.
