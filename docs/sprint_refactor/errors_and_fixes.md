# Sprint Refactor - Errors & Fixes

This file tracks the defects that the V02 refactor must close. The initial
entries below were cross-checked against the current code on 2026-07-27 in a
read-only editor session; no live hardware validation was performed here. They
were re-verified against the working tree on 2026-08-11 and all still hold,
except where a `Superseded` note says otherwise.

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
- **Superseded 2026-08-11 (hardware topology change).** Each device now has its
  own CAN interface (arms on native `can0`/`can1`, hands on FD-capable USB
  adapters `can2`/`can3`), so background hand traffic no longer competes with
  the arm bus. The *safety* argument for this entry is closed; the *CPU*
  argument is not. Uncoordinated hand polling and recurring hold commands still
  consume Jetson CPU, and CPU starvation — not bus arbitration — is what
  overflowed the CAN RX socket in the sprint6 finding.
- Interim rule (revised): same-side arm and hand motion may now run in parallel;
  hand polling and hold traffic must still be justified by active hand ownership
  because CPU is the remaining shared resource.
- Planned fix surface: per-device bus modelling and hand ownership in
  `agx_arm_ctrl` plus the registry (integration plan Phase 2).

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

## 2026-08-11 (velocity truth, traced in the vendor checkout)

### The zeroed velocity is a vendor workaround on every Nero tier, not one driver

- Traced in the development checkout (`control-layer-pin-2026-07-24`): the CAN
  protocol **does** carry motor speed — `ArmMsgFeedbackHighSpd` bytes 0-1,
  `int16`, unit 0.001 rad/s — and the override was introduced by `cea1cb9`
  (2026-05-26), whose message describes it as a deliberate
  "get_motor_states 电流/速度修正" (current/velocity correction).
- It is present in `nero/default` (velocity zeroed **and** `current *= -1`) and
  in `nero/versions/v111` (velocity only). `nero/versions/v112` does not define
  `get_motor_states` at all — it subclasses the v111 driver, so **it inherits the
  zeroing**. There is no driver tier that reports real velocity.
- Consequence: bumping the firmware tier is not a route to honest velocity. The
  repo derives speed from timestamped joint positions instead (landed), and the
  protocol value can only be compared against it by patching the vendor driver
  in the development checkout and running on hardware — an 0E activity.
- The `current *= -1` sign correction on the default tier is still unaudited; it
  changes the sign of a value nothing in this repo currently consumes, but it
  should be confirmed against a known load before anything starts trusting it.

### The ROS node can never select the v112 driver

- Symptom: `agx_arm_ctrl_single_node._init_agx_arm` maps firmware to driver tier
  with `elif self.is_nero: if current_version >= "1.11": firmeware_version =
  NeroFW.V111`. There is no `NeroFW.V112` branch, so a 1.12 arm runs the v111
  driver.
- The comparison is also lexicographic on strings, which is the
  "firmware version parsing using numeric tuples" defect the proposal listed.
  `"1.9" >= "1.11"` is true, so a 1.9 arm would be given the v111 driver.
- Impact: today this costs only the v112-specific APIs, because all tiers zero
  velocity anyway. It stops being harmless the moment a tier difference matters.
- Planned fix surface: numeric version parsing and the missing tier branch in
  plan 1D, alongside the other driver-boundary defects.

## 2026-08-11 (found while building the L2 harness)

### The left hand's CAN link drops the response half of every exchange

- Symptom: the SDK bridge on the left hand logs `CANFD ID: 0x...... 请求超时`
  for every request, followed by `motor Input size does not match expected motor
  count.` — the vendor SDK reacting to a readback that never arrives.
- Cause: a known bad CAN cable on the left hand, pending replacement. TX leaves
  the host; the RX→TX exchange does not complete.
- How to read it until the cable is fixed: a **rising TX/packet count means the
  device is online**; a **rising drop count means the command was sent, the hand
  tried to answer, and the link ate the response**. Do not read the timeouts as
  a bridge defect or as evidence about the refactor.
- Consequence for Phase 0: the left hand cannot contribute readback-dependent
  numbers to the 0E baseline. Capture the left side's TX and drop counters, but
  take joint-readback, verification-latency, and tactile figures from the right
  side until the cable is replaced.

### The coordinator can survive SIGINT when no activity is running

- Symptom: the L2 harness leaked `/agx_arm_coordinator` into its ROS domain
  after a failed run; the next run's domain guard caught it.
- Current evidence: `coordinator_node.py` installs its own SIGINT handler so
  Ctrl+C can unwind a *running* activity (`request_stop`). With nothing in
  flight there is no loop to unwind, and the process kept spinning.
- Impact: low in production, where the operator sends a second Ctrl+C, but it
  means "Ctrl+C reaches the robot" is not the same statement as "Ctrl+C ends the
  process".
- Interim rule: the L2 harness escalates SIGINT → SIGTERM → SIGKILL rather than
  trusting the first signal.
- Planned fix surface: the stop-ladder migration in plan 3B, which should make
  the idle case exit rather than spin.

### An L2 harness on the default ROS domain reaches live hardware

- Symptom: the first harness run spawned mock hand bridges and a coordinator
  into the ambient domain, where an SDK hand bridge was live on a real CAN
  interface.
- Impact: identically named nodes and overlapping topics in the same graph as
  running hardware. Nothing was damaged in this instance, but the exposure was
  real and the run also hung on discovery against the larger graph.
- Fix (landed): the harness runs on its own `ROS_DOMAIN_ID`, sets
  `ROS_LOCALHOST_ONLY`, and refuses to start when that domain is not empty.
- Rule: no mock-level harness runs on the domain that carries hardware.

## 2026-08-11 (the config layer still assumes one bus per side)

### The hand has no bus of its own in the registry

- Symptom: the OmniHand bridge derives its SocketCAN interface from the *arm's*
  `can_port`, and falls back to `can_nero_right` when the registry cannot be
  read.
- Current evidence:
  `src/agx_arm_sim/agx_arm_description/config/duo_motion_registry.yaml` defines
  `can_port` only under `arm.sides.*`; `omnihand_bridge_node.py:34-42`
  (`resolve_can_interface`) reads that arm value, with
  `_FALLBACK_CAN_INTERFACES` mapping both sides back onto the arm buses.
- Impact: with hands on separate USB-CAN FD adapters, the resolved interface is
  wrong by construction, and the fallback silently points a hand at an arm bus.
- Interim rule: pass `can_interface` explicitly per hand bridge until the
  registry carries a per-device bus entry.
- Planned fix surface: registry schema bump plus fail-closed resolution
  (integration plan 2A).

### Hand backend interface selection is process-global

- Symptom: the SDK backend selects its CAN interface by writing the
  `OMNIHAND_SOCKETCAN_IFACE` environment variable of the whole process.
- Current evidence: `agx_arm_ctrl/omnihand/sdk_o12_pro.py:152` and
  `omnihand_bridge_node.py:517` both assign `os.environ[...]` in the backend
  constructor.
- Impact: two hand backends in one process would overwrite each other's
  interface. Previously masked because both hands shared their side's arm bus;
  with two distinct adapters this is a real correctness hazard.
- Interim rule: never compose two hand backends into one process.
- Planned fix surface: explicit interface argument in the backend constructor,
  no environment mutation (integration plan 2A, 5D).

### CAN bring-up scripts still describe shared side buses

- Symptom: `scripts/activate_native_can.sh` documents and configures
  `can0 -> can_nero_right (right side: right arm + right OmniHand)` and the
  left-side equivalent.
- Impact: the operational documentation contradicts the deployed four-interface
  topology, and there is no stable-naming story for the two USB adapters, which
  can enumerate in either order.
- Planned fix surface: four-interface bring-up with deterministic adapter naming
  (integration plan 0A, 2A).