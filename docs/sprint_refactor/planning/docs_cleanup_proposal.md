# Documentation Cleanup Proposal — V02 Refactor

> **Executed 2026-08-16.** Sections 1-10 have been carried out; this proposal is
> now input and history, not authority. The decisions it made live in
> `planning/integration_plan.md` (Phase 2C ownership, Phase 4D target contract),
> `../checklist.md`, `../../checklist.md`, and the `.claude/` + `.github/` agent
> layers. The stale phrases quoted below are quoted deliberately, as the text the
> cleanup removed — they are the only remaining occurrences in the repository.
>
> Section 11 is deliberately **not** executed: those are runtime findings, and
> they are recorded as open work (`../open_questions.md`, the C7 exit item in
> `../checklist.md`) rather than fixed here.

## Purpose

This cleanup pass reconciles the active documentation and agent guidance with the current implementation state of `ROS2_Duo_System_V02_refactor`.

The goal is not to redesign the runtime. It is to remove stale claims, contradictory status descriptions, obsolete sprint guidance, and agent instructions that would cause future work to be based on behavior that no longer exists.

The authoritative order for this pass is:

1. `AGENTS.md` — durable engineering contract
2. `docs/sprint_refactor/planning/integration_plan.md` — canonical refactor plan and constraints C1–C8
3. current implementation in `src/agx_arm_ctrl/` and `src/agx_arm_coordination/`
4. validated evidence in `docs/sprint_refactor/reference/`, `errors_and_fixes.md`, and the sprint checklist
5. global documentation and agent mirrors

A historical statement may remain only when it is explicitly marked as superseded/history and cannot be mistaken for the current contract.

---

## 1. Root README branch and runtime wording

### Files

- `README.md`
- `README_EN.md`

### Current contradiction / ambiguity

Both READMEs present themselves as the entrypoint for the current Duo/Nero workspace but the clone command still uses:

```bash
git clone -b ros2 --recurse-submodules ...
```

while the currently active refactor work is performed on `ROS2_Duo_System_V02_refactor`.

The runtime notes also still emphasize the shared arm-plus-hand CAN path even though the normal V02 topology is now one CAN interface per device and `shared_per_side` is degraded compatibility mode.

### Required cleanup

Do **not** blindly replace `ros2` if that branch is intentionally the future public merge target. Instead make the intent explicit:

- if the README is meant to describe the currently checked-out development branch, use `ROS2_Duo_System_V02_refactor`;
- if `ros2` is intentionally the stable/public branch, make the clone example branch-neutral or explicitly label it as the stable branch example;
- describe `dedicated_per_device` as the normal V02 topology;
- describe shared arm-plus-hand CAN only as degraded/compatibility mode.

### Acceptance check

A reader must not infer that shared-per-side CAN is the normal architecture or accidentally clone a different branch while following branch-specific refactor instructions.

---

## 2. Global sprint status is stale

### File

- `docs/checklist.md`

### Current stale state

The global checklist still states:

- Sprint 6 is `active`;
- current focus is Sprint 6 tactile thresholds, grasp presets, shared-bus timing, and the Hefeweizen demo ladder.

This conflicts with the refactor sprint contract, which explicitly states that the V02 refactor has priority and `docs/sprint6/` adapts afterwards.

### Required cleanup

Rewrite the execution snapshot so that:

- `sprint_refactor` is the active implementation sprint;
- Sprint 6 is paused / awaiting adaptation to the refactored runtime contracts;
- coordinated demo work resumes after the refactor runtime reaches its explicit stabilization gate;
- shared-bus timing is no longer listed as the normal production focus;
- Hefeweizen / `tea_pour_left_v1` is not described as a standing per-phase regression gate until the command contracts are stable and the demo is re-taught.

Do not delete historical Sprint 6 status. Move it into a clearly marked previous-state note if preservation is useful.

---

## 3. Refactor checklist contains internally contradictory completion claims

### File

- `docs/sprint_refactor/checklist.md`

### 3.1 Phase 2B: C7 is not fully closed

The checklist currently marks as complete:

> Derive `handoff_enabled` from the same value; nothing reads it directly.

The implementation now defaults both coordinator `handoff_enabled` and FJT `handshake_enabled` from `handshake_required()`, but they remain separately overridable ROS parameters.

That means the code is improved but the strict C7 target — one topology fact, no independent second switch — is not fully achieved.

#### Required cleanup

Change this item from a fully closed claim to one of:

- `[~] defaults derived; direct override remains and must be removed or validated`, or
- retain `[x]` only for the default derivation subtask and add a separate open item for eliminating contradictory overrides.

The text must distinguish **current implementation** from **C7 exit condition**.

### 3.2 Phase 4 incorrectly says hand FJT is non-default

The Phase 4 checklist still says:

> Keep the MoveIt hand FJT path non-default in coordinated production profiles.

This directly contradicts the canonical plan, which was amended on 2026-08-14:

- hand `FollowJointTrajectory` is the primary trajectory-execution primitive;
- reactive contact-seeking motion is the second legitimate production primitive;
- FJT is neither debug-only nor optional.

#### Required cleanup

Replace the stale checklist item with wording equivalent to:

> Keep hand `FollowJointTrajectory` available as the primary trajectory-execution primitive in production MoveIt profiles, alongside reactive contact-seeking motion as the second production primitive. Both paths must arbitrate through device authority.

Search the entire repository for variants of:

- `FJT non-default`
- `FJT debug-only`
- `MoveIt hand FJT debug`
- `hand trajectory path optional`

and either rewrite them to the current contract or mark them historical/superseded.

### 3.3 Phase 5 still contains work that the canonical plan moved to 2C

The checklist still contains:

> Split OmniHand bridge timers by command verification, tactile, and status semantics.

under Phase 5, while the canonical plan explicitly marks Phase 5C as **moved to 2C**.

The sprint checklist itself also states that the former Phase 5C was folded into 2C on 2026-08-14.

#### Required cleanup

Remove the duplicate active Phase 5 task and replace it with a pointer such as:

> Bridge cadence/transport work moved to Phase 2C on 2026-08-14; Phase 5 retains only executor/process policy and close-out measurement.

Do not leave the same task active in two phases.

### 3.4 `tea_pour_left_v1` is simultaneously deferred and mandatory

Earlier in the checklist the demo regression is explicitly deferred until re-teach, with the L2 harness serving as the standing regression net.

At the bottom, `Every phase` still requires:

> `tea_pour_left_v1` still runs after the phase closes.

These statements cannot both define the current gate.

#### Required cleanup

Replace the standing demo requirement with:

- L1 and L2 are mandatory per-phase software regression gates;
- L3 evidence is required where hardware behavior is claimed;
- `tea_pour_left_v1` becomes a gate again only after the post-refactor command contract is frozen and the demo has been re-taught.

The old demo gate may remain in a historical note explaining why it was suspended.

### 3.5 Phase 4D command identity wording is incomplete

Several places summarize the future hand command stamp as only:

- owner identity;
- control epoch;
- sequence.

The runtime authority model already froze a more precise command stamp:

- `owner_id`;
- `device_epoch`;
- `unit_safety_epoch`;
- `sequence`.

#### Required cleanup

Use those four names consistently everywhere. Avoid the ambiguous term `control epoch` unless it is explicitly defined as a shorthand and never used in an interface definition.

Also make clear that standard ROS `JointState` / `JointTrajectory` messages themselves are **not** being modified. The target is a repo-owned internal command contract that carries standard motion content plus the authority stamp.

---

## 4. The canonical integration plan is stale in its Phase 2C ownership text

### File

- `docs/sprint_refactor/planning/integration_plan.md`

### Current contradiction

The Phase 2C ownership section still says:

> a hand still has no serialized SDK owner

and describes the SDK worker as arm-only.

The sprint checklist records the opposite as implemented and L3-validated on 2026-08-15:

- the hand bridge owns an `SdkWorker`;
- acquisition runs on its own paced thread;
- no SDK call is reachable directly from a ROS callback;
- steady-state SDK work is serialized through one named worker thread with priority lanes.

Because `integration_plan.md` is the canonical plan, leaving the pre-implementation statement unqualified is especially dangerous: agents are explicitly instructed to trust this file over lower-level notes.

### Required cleanup

Rewrite the Phase 2C ownership subsection into current-plan language:

- the design requirement is one serialized SDK owner per hand;
- this requirement landed on 2026-08-15;
- remaining work is the declared stop-latency/bounded-in-flight-call question and any residual transport bounds, not creation of the worker itself.

Keep the old state only as a short superseded rationale if useful.

Also review any `Exit gate` wording that still describes already-landed implementation as future work and mark completed evidence consistently.

---

## 5. OmniHand agent rules describe an implementation that no longer exists

### Files

- `.claude/rules/omnihand-bridge.md`
- `.github/instructions/omnihand-bridge.instruction.md`
- `.github/copilot-instructions.md`

### Current contradiction

These files still state that a hand has no designed serialized SDK owner and reaches the SDK directly from timer/subscription/service callbacks, relying on single-threaded `rclpy.spin` as an accidental serialization property.

That was true before the 2026-08-15 Phase 2C work. It is no longer the implementation.

### Required cleanup

Replace the stale rule with the current invariant:

- every hand bridge owns one `SdkWorker`;
- steady-state SDK calls are serialized through that worker;
- acquisition is paced independently from ROS publication;
- ROS callbacks must not call the vendor SDK directly;
- safety work uses the designated priority lane;
- shutdown must stop the acquisition path and worker cleanly;
- recovery/exception rules, if any, must be stated explicitly rather than inferred from executor behavior.

The `.claude/` and `.github/` mirrors must be changed in the same commit.

### Additional stale pointer

`.claude/rules/omnihand-bridge.md` refers to `phases 2A-2D and 4D`, but the current canonical plan has Phase 2A–2C, not 2D.

Correct the phase range.

---

## 6. `docs-keeper` is itself stale

### Files

- `.claude/agents/docs-keeper.md`
- `.github/agents/docs-keeper.agent.md`

### Current contradictions

The docs-keeper currently says:

- the refactor has **six** binding constraints;
- the normal per-device CAN mapping is described using `can0` / `can1` and `can2` / `can3`.

The current canonical plan defines **C1–C8**, and stable runtime names are:

- arms: `can_nero_left`, `can_nero_right`;
- hands: `hand_left`, `hand_right`.

This is particularly harmful because the docs-keeper is supposed to detect exactly this class of drift.

### Required cleanup

Update both mirrors to:

- explicitly reference constraints C1–C8;
- include C6, C7, and C8 in the audit checklist where relevant;
- use stable runtime interface names, not enumeration-style `can0..can3` names;
- add a specific check for stale claims that a hand lacks an `SdkWorker`;
- add a specific check that FJT is described as a production primitive;
- add a specific check that demo gates marked deferred are not reintroduced in generic `Every phase` sections.

---

## 7. Source-level docstrings still describe retired behavior

These are documentation defects even though they live in source files. Correct them in the cleanup commit without changing runtime behavior.

### 7.1 `omnihand_skill_controller_node.py`

#### Current stale text

The module docstring:

- points to the old Sprint 6 planning surface as its primary contract;
- says that after a confirmed grasp a background timer republishes the grasp target.

The implementation now does the opposite: post-grasp hold monitors contact without recurring command publication, and hand ownership is explicitly coordinated through device authority.

#### Required cleanup

Rewrite the module contract to state:

- the active architecture reference is the refactor contract / current hand skill mapping document;
- the reactive controller claims the hand before commanding;
- a confirmed grasp may retain ownership while holding;
- hold monitoring does not periodically republish the grasp target;
- FJT and reactive control are mutually exclusive production primitives under the same device-authority mechanism.

### 7.2 `device_authority.py` / `UnitSafety`

#### Current stale text

The `UnitSafety` class docstring still says:

> Until a single writer exists in the running system, every driver is still its own writer.

The single `unit_safety_node.py` writer now exists and devices operate as observers.

#### Required cleanup

Rewrite the docstring to describe the current invariant:

- exactly one unit-safety writer process;
- device processes are observers and may request a unit stop but do not mint unit generations;
- local device-stop behavior remains independent so loss of the writer does not prevent a device from stopping itself.

Do **not** use this documentation cleanup to hide the separate implementation question around writer restart / epoch continuity. If that issue remains unresolved, record it as an explicit open runtime issue rather than implying stronger restart guarantees than the code provides.

---

## 8. Architecture wording for the Phase 4D hand command contract

This cleanup should prepare the documentation for the eventual message migration without prematurely claiming that the migration is implemented.

### Required terminology

Use one reusable authority structure in documentation:

```text
owner_id

device_epoch

unit_safety_epoch

sequence
```

Call it consistently, for example, **command authority stamp** or **device command stamp**.

### Recommended target architecture to document

Do not describe the target as "extending `sensor_msgs/JointState`" or "extending `trajectory_msgs/JointTrajectory`". ROS standard messages are external types and should remain untouched.

The intended split should be documented as:

1. `sensor_msgs/JointState` remains a feedback/state message.
2. Standard `FollowJointTrajectory` remains the MoveIt-facing trajectory action.
3. Both production hand primitives ultimately feed one repo-owned, authority-stamped internal hand-command boundary in `agx_arm_msgs`.
4. The bridge admits or rejects that internal command using owner, device epoch, unit-safety epoch, and sequence.
5. The FJT executor binds an accepted standard action goal to its current device claim/epoch and emits stamped internal commands.
6. The reactive skill controller claims the same device and emits the same stamped internal command type.
7. The bridge remains the only hardware transport boundary and does not invent missing authority fields from its own current state.

The exact `.msg` name remains a Phase 4D implementation decision, but documentation should not encourage multiple competing OmniHand-specific command messages.

A reusable nested message is preferred conceptually, e.g.:

```text
# agx_arm_msgs/DeviceCommandStamp.msg
string owner_id
uint64 device_epoch
uint64 unit_safety_epoch
uint64 sequence
```

embedded by the eventual abstract hand command.

Do not encode these fields into `Header.frame_id`, action goal UUIDs, topic names, or ad-hoc JSON metadata.

---

## 9. Repository-wide search pass

After the targeted edits, perform a repository-wide grep for stale phrases and concepts.

At minimum search for:

```text
Sprint 6 is active
shared side bus
shared arm-plus-hand CAN
step-and-settle as normal
handoff_enabled=true
handshake_enabled=true
nothing reads handoff_enabled
FJT non-default
FJT debug
hand has no serialized SDK owner
single-threaded executor serializes the hand SDK
background timer republishes the grasp target
tea_pour_left_v1 still runs after the phase closes
six binding constraints
can0/can1
can2/can3
phases 2A-2D
control epoch
```

Each match must be classified as one of:

- current and correct;
- historical evidence, explicitly marked as such;
- stale and rewritten;
- stale but intentionally retained behind a superseded banner.

Do not mechanically replace historical evidence inside measurement records when the old statement is part of the chronology. The defect is only when historical state can be read as current guidance.

---

## 10. Agent mirror acceptance gate

Before committing the cleanup:

- compare every changed `.claude/rules/*` file with its `.github/instructions/*` mirror;
- compare changed `.claude/agents/*` with `.github/agents/*`;
- verify `CLAUDE.md`, `AGENTS.md`, and `.github/copilot-instructions.md` do not reintroduce the stale statement;
- run the repository's commit-quality skill;
- run at least documentation/link/static checks relevant to the touched files;
- no hardware run is required for a documentation-only change, but all hardware claims retained in the docs must continue to name their existing evidence level/date.

---

## 11. Explicit non-goals of this cleanup

This proposal does **not** itself fix the runtime findings discovered during the review, including:

- unit-safety writer restart / epoch-continuity semantics;
- topology-aware activity validation;
- atomic `sync_flag` scheduling / merge-or-fail behavior;
- remaining direct arm SDK call sites;
- complete worker shutdown handling;
- elimination of the independent `handoff_enabled` / `handshake_enabled` overrides;
- the Phase 4D command-message migration itself.

Documentation must describe these honestly as open implementation work where applicable. Do not "fix" a code defect by documenting the defective behavior as the intended contract.

---

## Completion criterion

The cleanup is complete when a new engineer or agent can read, in order:

1. root README,
2. global checklist,
3. refactor integration plan,
4. refactor checklist,
5. OmniHand rules,
6. docs-keeper rules,

and derive one consistent current model:

- the V02 refactor is the active architectural work;
- each arm and hand normally owns a dedicated CAN interface;
- shared-per-side operation is degraded compatibility mode;
- device authority and unit-safety epochs gate command execution;
- both arms and hands use designed serialized SDK ownership in steady state;
- FJT and reactive contact-seeking control are both production hand primitives and are mutually exclusive through device authority;
- the hand bridge does not yet receive per-command authority identity from standard JointState/JointTrajectory transport, so Phase 4D remains open;
- the eventual hand command boundary carries `owner_id`, `device_epoch`, `unit_safety_epoch`, and `sequence` explicitly;
- Sprint 6 demo work resumes on top of the stabilized refactor contract rather than serving as a standing gate during the migration.
