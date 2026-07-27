# Sprint Refactor - Integration Plan

## Goal

Turn the coordination refactor proposal into a migration sequence that can be
implemented on branch `ROS2_Duo_System_V02` without building later work on top
of unsafe ownership assumptions.

The cross-check in `../reference/proposal_code_crosscheck.md` confirms that the
proposal is still aligned with the current codebase. The plan below therefore
uses the proposal as the architectural target and the current implementation as
the migration baseline.

## Planning rules for this branch

- fix release blockers and ownership bugs before doing cleanup or performance work
- keep package boundaries intact unless a new repo-owned ROS contract requires a
  new interface in `src/agx_arm_msgs`
- prefer compatibility wrappers during migration when they keep the current Duo
  bring-up runnable
- validate the narrowest touched package after each slice
- treat hardware validation as a gated activity, not as something implied by
  editor-only code review

## Cross-checked starting point

- The velocity-based soft e-stop verification path is not trustworthy yet.
- The current arm driver still has multiple SDK callers and mode mutations.
- The hand window contract is stronger than a naive Trigger, but it still does
  not export lease identity, epoch, or transport quiescence.
- The skill controller still republishes hold commands after a grasp succeeds.
- The bridge still polls and retries outside explicit ownership state.
- The coordinator still accepts concurrent activities and advances them through
  a polling loop.
- The registry is already the intended source of truth, but execution profiles
  still carry duplicated per-side runtime mapping.
- The active Duo MoveIt controller surface is already generated in
  `_moveit_config_builder.py`; the legacy standalone unprefixed
  `moveit_controllers.yaml` should be treated as a quarantined legacy artifact.

## Workstream map

| Workstream | Primary packages | Why it belongs early |
| --- | --- | --- |
| Velocity truth and stop semantics | `pyAgxArm`, `agx_arm_ctrl` | Current safety wording is unsound until feedback is honest |
| Side authority and epoch | `agx_arm_msgs`, `agx_arm_ctrl`, `agx_arm_mit_controller` | Every later handover and recovery path depends on one authoritative owner |
| Lease-based hand control | `agx_arm_msgs`, `agx_arm_ctrl`, `agx_arm_coordination` | Prevents background hand traffic from invalidating arm ownership |
| Coordinator exclusivity | `agx_arm_coordination` | Stops concurrent activities from racing shared unit state |
| Registry/profile consolidation | `agx_arm_description`, `agx_arm_ctrl`, `agx_arm_moveit`, `agx_arm_coordination` | Converts existing partial source-of-truth discipline into one resolved manifest |
| Runtime work reduction | `agx_arm_ctrl`, `agx_arm_mit_controller`, `agx_arm_coordination` | Should be measured only after ownership rules are correct |

## Phase 0 - Safety baseline and instrumentation

### 0A. Honest velocity and stop semantics

Targets:

- remove the forced zero velocity path in the Nero driver or add a derived
  velocity path from timestamped joint positions
- change arm-driver stop reporting so `commanded` and `feedback_verified` are
  separate outcomes until velocity is trustworthy end-to-end
- keep the current emergency path fail-closed if velocity evidence is missing

Primary files:

- `pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- relevant driver and arm-control tests

Validation:

- package-scoped tests for the driver and arm control surfaces
- build the touched packages with
  `bash ./scripts/colcon_build_system_python.sh --packages-select ...`
- explicit hardware note if real velocity cannot be validated in the session

### 0B. Baseline instrumentation

Targets:

- add counters or logs for loop duration, callback duration, SDK call origin,
  and CAN traffic direction
- capture the initial hardware baseline for idle, MIT, hand-window, and
  recovery scenarios

Exit gate:

- no stop path still relies on known synthetic zero velocities
- a baseline report exists for later CPU and CAN comparisons

## Phase 1 - Side authority and serialized SDK access

### 1A. Authoritative side state

Targets:

- introduce a per-side state contract with a control epoch
- publish one authoritative state from the arm side authority
- make MIT consumption depend on the authoritative side state instead of a
  derived boolean hand-window topic

### 1B. Serialized hardware worker

Targets:

- move all arm SDK calls for one side onto one worker or queue
- give emergency stop a priority path within that worker
- drop old-epoch queued commands after ownership transitions or recovery

### 1C. Recovery and rearm split

Targets:

- separate fault acknowledgement from verified rearm
- require fresh feedback, comm health, and hold capture before returning to arm
  control

Primary files:

- `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/nero_can_push.py`
- `src/agx_arm_mit_controller/.../mit_controller_node.py`
- new or extended interfaces under `src/agx_arm_msgs`

Exit gate:

- no arm SDK call happens outside the worker on the migrated path
- stale-epoch commands are rejected in tests
- MIT aborts immediately when the side authority leaves arm control

## Phase 2 - Leased hand control

### 2A. Lease contract

Targets:

- add a repo-owned hand-lease action and release service
- add a lease-aware hand command message with sequence and control epoch
- decide whether the current Trigger services remain as temporary wrappers

### 2B. Driver-side lease manager

Targets:

- require verified hold, bus-quiet evidence, and lease identity before the hand
  may use the shared side bus
- require bridge transport quiescence and fresh arm feedback before returning to
  arm control

### 2C. Skill and bridge migration

Targets:

- make the skill controller require a valid lease and stop recurring hold
  publication after grasp completion
- gate bridge command acceptance, polling, and retries on side state, lease, and
  epoch
- reject stale lease commands and old sequences at the bridge boundary

### 2D. Coordinator integration

Targets:

- make the coordinator acquire a lease before every hand action
- propagate lease identity into the hand skill goal
- fail the activity if release or arm resume cannot be verified

Primary files:

- `src/agx_arm_msgs/*`
- `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_skill_controller_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py`
- `src/agx_arm_coordination/agx_arm_coordination/coordinator_node.py`

Exit gate:

- no hand TX or hand read-request traffic occurs during same-side arm ownership,
  except an explicitly leased diagnostic window
- post-grasp hold no longer depends on recurring host-side commands

## Phase 3 - Coordinator exclusivity and event-driven execution

### 3A. One active unit activity

Targets:

- reject new activity goals unless the unit is `READY`
- track one authoritative unit activity state and failure reason

### 3B. Event-driven child management

Targets:

- replace future polling with done-callbacks and an internal event queue
- keep only a low-rate watchdog for deadlines and timeouts
- add bounded child-cancel and cleanup handling

### 3C. Strict synchronization behavior

Targets:

- require `sync_flag` arm pairs to merge into the proper combined execution path
- remove any independent-dispatch fallback for strict synchronization

Primary files:

- `src/agx_arm_coordination/agx_arm_coordination/coordinator_node.py`
- supporting scheduler, performer, and graph-model files under
  `src/agx_arm_coordination/agx_arm_coordination/`

Exit gate:

- concurrent activity goals are rejected
- child completion, cancellation, and cleanup do not rely on a 20 Hz polling
  loop

## Phase 4 - Registry, profiles, and generated runtime artifacts

### 4A. Resolver and manifest

Targets:

- define one resolved manifest that combines the registry, execution profile,
  and explicit launch overrides
- include source hashes and a manifest hash for runtime consistency checks

### 4B. Reduce execution profiles to selection-only composition

Targets:

- stop repeating side namespaces, prefixes, controller names, and CAN ports when
  they already exist in the registry
- build on the current `execution_profiles.py` resolver rather than replacing it

### 4C. Generate runtime artifacts

Targets:

- generate MoveIt simple-controller-manager config from the manifest
- generate resource claims, joint-state merger inputs, and launch parameter
  dictionaries from the same source
- quarantine the legacy standalone `moveit_controllers.yaml` from Duo launch
  paths and document its non-authoritative status

Primary files:

- `src/agx_arm_sim/agx_arm_description/config/duo_motion_registry.yaml`
- `src/agx_arm_ctrl/config/execution_profiles.yaml`
- `src/agx_arm_ctrl/agx_arm_ctrl/execution_profiles.py`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_coordination/.../motion_registry.py`

Exit gate:

- changing a side namespace, prefix, or CAN port requires one authoritative edit
- all migrated runtime nodes can report the same manifest hash

## Phase 5 - Runtime consolidation and measurement close-out

### 5A. Arm feedback and publication budgets

Targets:

- separate acquisition cadence from publication cadence
- remove repeated SDK getter work from unconditional high-rate loops
- keep only rates that are justified by control deadlines or consumers

### 5B. MIT execution-loop consolidation

Targets:

- make one timer the sole trajectory evaluator
- make action completion and feedback emission event-driven off execution
  snapshots

### 5C. Bridge timer split and no background hold traffic

Targets:

- split command verification, tactile, status, and heartbeat work by semantics
- enable active polling only during a valid hand lease
- publish cached or heartbeat status at low rate outside a lease

### 5D. Before/after measurement close-out

Targets:

- re-run the Phase 0 baseline scenarios
- compare CPU, callback duration, timer jitter, and CAN traffic against the
  baseline
- record the remaining hardware-only gaps before broader rollout

Exit gate:

- no unconditional high-rate loop continues doing non-control work in the hot
  path
- post-refactor CPU and CAN measurements are captured and compared against the
  baseline

## Suggested implementation increments

1. `pyAgxArm` plus `agx_arm_ctrl`: velocity truth, stop semantics, narrow tests.
2. `agx_arm_msgs` plus `agx_arm_ctrl`: side state, epoch, worker-owned SDK path.
3. `agx_arm_mit_controller`: authoritative side-state consumption and fast abort.
4. `agx_arm_msgs` plus `agx_arm_ctrl`: lease contract, bridge gate, no recurring
   hold traffic.
5. `agx_arm_coordination`: one activity slot, lease-aware hand actions,
   event-driven child completion.
6. `agx_arm_description`, `agx_arm_ctrl`, `agx_arm_moveit`, and
   `agx_arm_coordination`: resolved manifest and generated runtime artifacts.
7. Runtime rate consolidation and post-refactor measurement report.

## Validation commands to prefer

- `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg_names>`
- `colcon test --packages-select <pkg_names>` from a system-Python ROS shell
- targeted unit tests for the touched slice before broader integration checks

## Promotion path after implementation starts

- promote stable runtime contracts into `docs/assets/`
- promote stable launch and bring-up changes into `docs/control/`
- promote stable package-boundary or generated-artifact policy into
  `docs/project/`
- keep intermediate evidence, unresolved measurements, and branch-local rollout
  notes inside this sprint surface until the V02 migration closes