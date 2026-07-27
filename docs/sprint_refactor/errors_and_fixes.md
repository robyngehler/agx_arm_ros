# Sprint Refactor - Errors & Fixes

This file tracks the defects that the V02 refactor must close. The initial
entries below were cross-checked against the current code on 2026-07-27 in a
read-only editor session; no live hardware validation was performed here.

## 2026-07-27 (soft e-stop verification is not trustworthy yet)

### Synthetic zero motor velocity invalidates stop verification

- Symptom: the Nero driver currently overwrites each reported motor velocity
  with `0.0` before returning motor feedback.
- Current evidence:
  `pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py`
  forces `motor_state.msg.velocity = 0.0`, while
  `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` uses that same
  velocity field in `_arm_velocities_settled()` for soft e-stop verification.
- Impact: a moving arm can be reported as settled, and stop verification cannot
  currently support a safety claim.
- Interim rule: until real or derived velocity is trustworthy, stop results must
  be treated as commanded-only, not feedback-verified.
- Planned fix surface: `pyAgxArm` velocity path, arm-driver stop semantics,
  related driver/controller tests.

## 2026-07-27 (shared-bus hand ownership is only partial)

### Current hand handover does not close background hand traffic

- Symptom: the current hand window verifies arm hold and attempts feedback-push
  silence, but the hand stack can still continue publishing and polling after a
  grasp action reports success.
- Current evidence:
  `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` exposes
  `prepare_hand_window` and `resume_arm_control`, while
  `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_skill_controller_node.py` keeps an
  internal hold timer that republishes the grasp target, and
  `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py` maintains independent
  publication, readback, and retry timers.
- Impact: the coordinator can believe the side bus has returned to the arm while
  the hand stack still emits CAN traffic.
- Interim rule: treat same-side arm motion and ongoing hand hold/polling as
  mutually exclusive until lease-based ownership lands.
- Planned fix surface: lease contract in `agx_arm_msgs`, side authority in
  `agx_arm_ctrl`, bridge/skill gating, coordinator release enforcement.

## 2026-07-27 (SDK state mutation is not serialized)

### The arm SDK still has multiple unsynchronized callers

- Symptom: multiple callbacks, service paths, and helpers can mutate mode and
  control state around the same SDK object.
- Current evidence:
  `src/agx_arm_ctrl/agx_arm_ctrl/nero_can_push.py` mutates the cached mode
  object, and the arm driver still relies on precondition checks before direct
  SDK calls rather than one serialized hardware-owner path.
- Impact: the current code remains vulnerable to time-of-check/time-of-use
  races during recovery, handover, or emergency paths.
- Interim rule: do not widen concurrent command sources before a serialized
  worker or queue exists.
- Planned fix surface: per-side worker/queue, authoritative side state, epoch
  checks at the hardware boundary.

## 2026-07-27 (coordinator and MIT still reason from partial state)

### The current orchestration layer is not yet globally exclusive

- Symptom: the coordinator accepts every activity goal and polls child progress,
  while the MIT controller consumes only a boolean hand-window topic rather than
  the full side-control state.
- Current evidence:
  `src/agx_arm_coordination/agx_arm_coordination/coordinator_node.py` uses a
  20 Hz polling loop with a `ReentrantCallbackGroup`, and the proposal's
  side-state or epoch contract is not present in the current MIT path.
- Impact: unit-level exclusivity, cleanup order, and fault propagation are still
  distributed across nodes with partial state.
- Interim rule: assume nothing about single-activity or fail-closed ownership
  until the authoritative side state and coordinator lock land.
- Planned fix surface: coordinator event queue, unit activity lock,
  side-state/epoch integration in `agx_arm_mit_controller`.

## 2026-07-27 (configuration truth is not fully consolidated yet)

### Registry is authoritative in intent, but profiles still duplicate runtime mapping

- Symptom: `duo_motion_registry.yaml` already holds side prefixes, namespaces,
  and CAN ports, but `execution_profiles.yaml` still repeats per-side runtime
  instance data for Duo bring-up.
- Current evidence:
  `src/agx_arm_sim/agx_arm_description/config/duo_motion_registry.yaml` is the
  documented single source of truth, while
  `src/agx_arm_ctrl/config/execution_profiles.yaml` still lists `namespace`,
  `joint_prefix`, `feedback_joint_prefix`, and `can_port` for each Duo arm
  instance.
- Impact: registry drift is still possible even though some values are already
  resolved from the registry at runtime.
- Interim rule: extend the existing resolver path instead of introducing another
  parallel configuration source.
- Planned fix surface: shared manifest resolver, generated MoveIt/runtime
  artifacts, legacy-config quarantine.