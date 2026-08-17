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
- Evidence at the time:
  `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` exposed
  `prepare_hand_window` and `resume_arm_control`, while
  `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_skill_controller_node.py` kept an
  internal hold timer that republished the grasp target, and
  `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py` maintained independent
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
- **Recurring hold traffic closed 2026-08-15 (L3).** The skill controller's hold
  no longer republishes the grasp target; it monitors contact only. Measured on
  the right hand on a real grasp: **0 command messages across a 5 s hold**, where
  the old behaviour would have sent 100. `prepare_hand_window` /
  `resume_arm_control` survive only as the `shared_per_side` implementation and
  default off on the declared topology.
- Still open from this entry: the bridge's cadences are decoupled from the arm's
  `pub_rate` but not yet split by semantics (command verification, joint
  readback, tactile, status), and polling does not stop while no hand action is
  active. Both are tracked in the Phase 2C checklist.
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

## 2026-08-11 (first instrumented measurements, L3 comm-only)

Hardware access granted for communication only, no commanded motion. The arm
driver was run with `auto_enable:=false`, so the joints were never energised.

### The 200 Hz publish loop runs at 3.5 Hz when the bus is silent

- Measured: `publish_batch: n=35 mean=0.03ms min=0.02ms max=0.09ms` over a 10 s
  window with `pub_rate=200`, i.e. 35 iterations where 2000 were configured.
- The work inside the batch is not the cost — 0.03 ms mean. The ~285 ms per
  iteration is spent *outside* it, in the readiness and feedback-timeout paths
  the loop takes when no frames arrive.
- Impact on the plan: `pub_rate` is not the loop rate. Any Phase 5 claim about
  decimating this loop has to state which regime it measured, because the
  configured rate, the healthy-bus rate and the silent-bus rate are three
  different numbers. The healthy case was captured the same day and reaches
  198 Hz — see `reference/phase0_baseline.md`.
- Not a fault by itself: degrading under a dead bus is reasonable. It is
  recorded because the refactor's before/after depends on knowing it.

### The faulty left hand costs ~1120 frames/s of pure drain

- Measured on `hand_left` with the SDK bridge running: **1120 RX/s, 0 TX/s**,
  no drops during the window. Cumulative counters show 6.8 M RX against 669 TX,
  215 684 drops, and 19 bus-off/restart cycles.
- Reading: the bridge has backed off and stopped transmitting (its fault
  backoff working as designed), while the hand keeps streaming into the host.
  The host drains 1120 frames per second for no exchange at all.
- Consequence for 0E: this is CPU load with zero information content, and it
  will sit underneath every baseline scenario until the cable is replaced.
  Record it as a constant offset or take the baseline with that bridge stopped;
  do not attribute it to the code under measurement.

### Neither arm bus carries any traffic — check the 40-pin header FIRST

**This has now happened twice, and a kernel update is enough to cause it again.**

> After any kernel update, before diagnosing anything CAN-side:
> `sudo /opt/nvidia/jetson-io/jetson-io.py`

- **2026-08-11:** the Jetson 40-pin header was not configured, so `mttcan`
  presented two interfaces that were UP and completely silent. With the header
  configured both arms push at ~2150 frames/s.
- **2026-08-17:** a kernel update discarded the header configuration. Same
  symptom, and it cost most of a debugging session and produced two wrong
  diagnoses before the header was reconfigured — after which both arms came up
  immediately (left firmware 1.11 / `NeroFW.V111`, right 1.06 /
  `NeroFW.DEFAULT`).

**How to recognise it.** An unconfigured header is indistinguishable from a
powered-off arm by `ip link` alone — both read UP, ERROR-ACTIVE, zero counters.
The discriminator is what happens when something *tries* to send:

```text
RX: packets 0   errors 0  dropped 0
TX: packets 0   errors 0  dropped 0
state ERROR-ACTIVE (berr-counter tx 0 rx 0)   <ONE-SHOT,FD>
```

with the driver reporting sends failing as `ENOBUFS` / "Transmit buffer full".
The pins are not muxed to the controller, so a frame is queued and never
completes; the queue fills and `write()` fails. **The bus error counters stay at
zero and are useless here** — ONE-SHOT aborts an unacknowledged frame instead of
retrying it into error-passive. Read TX *packets*.

A hand bus transmitting normally in the same session does not exonerate the
header: the hands are on USB-CAN FD adapters and do not use the 40-pin header at
all. That contrast narrows the fault to the native interfaces rather than the
host.

- The arms also send nothing until something activates them, so silence has at
  least three causes and `measure_can_baseline.sh` reports SILENT rather than
  guessing.
- The 0 TX from the driver used to be explained by the driver never reaching the
  point of re-asserting the push. That is no longer true: since 2026-08-17
  startup asserts the feedback push on the transport alone, so a driver run now
  distinguishes the cases for you — it reports `Transport session: present;
  feedback-push bootstrap: sent; feedback: none; enable: unverified` when the
  frames cannot leave the host.

### CAN interface names are `hand_left`/`hand_right`

- The hand adapters are named `hand_left` and `hand_right`, not
  `left_hand`/`right_hand` — an earlier revision of the plan had them reversed
  and it has been corrected.
- Worth care in 2A: `left_hand` *is* the scheduler resource name in
  `graph_model.py`, so the two spellings coexist with different meanings and
  must not be derived from one another.

## 2026-08-11 (velocity truth — settled on the wire)

### The Nero firmware does not report joint velocity at all

- Evidence: `evidence/can_nero_{left,right}.pcap`, captured with both MIT
  controllers driving the arms, analysed by `scripts/analyze_can_pcap.py`.
- While the joints were being driven, the reported velocity field in
  `0x251`-`0x257` (bytes 0-1, int16, 0.001 rad/s) stayed at **0**, reaching
  **±1** on three joints. Velocity derived from the position bytes of the *same
  frames* peaked at 542-1403 units/s on the right arm and 3215 on the left.
- The vendor's `velocity = 0.0` override therefore hides nothing: the wire data
  is already zero. Removing it would expose a field that looks plausible and is
  wrong, which is worse than an obvious zero.
- **Deriving velocity from timestamped positions is the only available source**,
  not a stopgap. This closes the question without patching the vendor driver.
- Knock-on: everything that consumed that field was reading zeros, including the
  MIT controller's goal-velocity tolerance checks — those have never constrained
  velocity. Phase 1 must treat them as unavailable until they are fed from the
  derived source.

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
## 2026-08-12 (Phase 1A, found while building the authority and worker)

Both entries are the same shape as the e-stop and the CAN-recovery defects
already recorded in this sprint: an action that could not verify its own effect
reported the effect anyway. Fixed at L1; neither has run on hardware yet.

### An enable the readback contradicted was reported as success

- Symptom: `_enable_arm` checked `get_joint_enable_status(255)` against the
  request, logged a warning when they disagreed, and then returned `True`
  regardless. `self.enable_flag` was only assigned when the readback agreed, so
  a disagreement left it holding its previous value.
- Impact: a failed *disable* was the dangerous direction — `enable_flag` stayed
  `True`, and everything gated on it (`_check_can_control`, the e-stop path, the
  hand window, leader mode) went on believing the arm was commandable. A failed
  *enable* was the quiet direction: the service reported success while control
  stayed blocked.
- Fix: the readback decides both the flag and the return value. The readback is
  served from the last low-speed feedback frame, which can predate the command,
  so it is re-read until it agrees or the enable timeout expires — a lag is not
  a contradiction.
- Fixed in: `agx_arm_ctrl_single_node._enable_arm`, with L1 tests in
  `test/test_arm_enable_and_firmware.py`. The one caller that discarded the
  result, the CAN recovery re-arm, now reports it (see below).

### CAN recovery reported success without saying what it verified

- Symptom: `_recover_bus` called `_enable_arm` and discarded the result, then
  logged `CAN bus recovery succeeded on attempt N` on the strength of feedback
  advancing alone.
- Impact: the 0E fault test produced exactly that line for a bus that had come
  back on its own. Feedback advancing says the link is alive; it says nothing
  about whether the arm is armed.
- Fix: the re-arm outcome is carried to the verdict and named in the log —
  feedback and the enable readback are reported separately, `not requested` is
  distinguished from `NOT confirmed`, and a restored bus with an unconfirmed
  enable logs as an error. The fault lockout after recovery is unchanged.
- Not fixed by this: a reconnect still cannot prove it was the reconnect that
  restored the bus rather than the bus recovering by itself. The log now claims
  only what was checked.

### An arm on firmware 1.12 was driven with the 1.11 protocol

- Symptom: startup mapped the Nero firmware string to a driver tier with
  `if current_version >= "1.11": firmeware_version = NeroFW.V111`. There was no
  `NeroFW.V112` branch, although the pinned SDK ships `NeroDriverV112` and
  documents `V112 >= 1.12`.
- Impact: a 1.12 arm silently ran the 1.11 protocol; both tiers connect, so
  nothing surfaced. The comparison was also done on strings, which only ordered
  correctly because the firmware happens to report a zero-padded minor
  (`1.07`, `1.11`, `1.12`) — an unpadded `1.9` compares as newer than `1.11`.
- Fix: `resolve_nero_firmware()` parses `major.minor` numerically, has a V112
  branch, and returns an explanation that startup logs.
- Open: which tier the two arms actually run on is still unrecorded anywhere in
  this repository. The next hardware session captures it from the startup log.

## 2026-08-12 (Phase 1A, the hardware boundary had no input contract)

### Anything in a MIT message reached the vendor SDK

- Symptom: `_move_mit_callback` checked that the six parallel arrays were the
  same length and that the message was not empty, then forwarded the contents
  joint by joint to `move_mit`.
- Impact: a NaN or infinity, a joint index the arm does not have, the same joint
  commanded twice in one message, or a gain an order of magnitude past the
  protocol range all reached the hardware boundary unexamined. The SDK clamps
  some of these, which is worse than refusing them: a clamped command is a
  *different* command, and the sender never learns it was wrong.
- Fix: `command_validation.validate_mit_command` refuses the message as a whole
  before any frame is sent. Whole, not per joint — the firmware keeps executing
  the last setpoint it received, so admitting six joints and dropping the
  seventh would leave the arm in a pose nobody commanded.
- Deliberately not refused: a position outside the joint's *configured* limit.
  It is warned and still forwarded, because refusing mid-stream would freeze a
  running impedance loop at its last setpoint. Promoted to a rejection once a
  hardware session establishes that the MIT controller never legitimately
  crosses a limit.
- Note on logging: rejections are counted per reason and the log line is
  rate-limited. A malformed stream arrives at the control rate, and on this
  Jetson the logging would itself become the load.

### Readiness was four booleans and none of them were published

- Symptom: whether the arm would accept motion was spread across `enable_flag`,
  `control_ready`, `_fault_lockout` and `_hand_window_active`. The only thing a
  controller could subscribe to was `feedback/hand_window_active`.
- Impact: that boolean says "the shared bus is busy" and nothing about faults,
  emergency stops, or who is commanding, so a controller could not distinguish
  a deliberate quiescence from a dead device — and had no way at all to notice
  that the device it was streaming to had changed hands.
- Fix: the driver now publishes one authoritative `AgxDeviceAuthority` on
  `feedback/authority`, latched, on change. The state is derived from the gates
  above, which is honest at this stage — they are already what the driver acts
  on. The **epochs are not derived**: they come from the authority's own
  transitions, so a command issued before an interruption is rejected after it.
- Open: the MIT controller does not consume it yet, so `hand_window_active`
  remains the live coupling until that lands. The `unit_safety_epoch` is also
  still per process — the coordinator stopping every side covers the current
  e-stop path, but it is not the same guarantee as one synchronised epoch.

## 2026-08-12 (L3, found by running the drivers against both arms)

### The two arms do not run the same firmware

- Finding: the right arm reports **1.06** and is driven by the default Nero
  protocol tier; the left arm reports **1.11** and is driven by `NeroFW.V111`.
  Both drivers connect and both arms work, so nothing has ever surfaced it.
  Startup now logs the tier on every run — it was recorded nowhere before.
- Why it matters beyond bookkeeping: the tiers are not the same protocol. The
  1.11 codec encodes MIT frames differently (12-bit feed-forward torque, no
  CRC), overrides `set_motion_mode`, `move_mit`, `get_flange_pose` and
  `get_motor_states`, and carries its own status enum. Any assumption that both
  arms behave identically at the protocol level is currently unfounded.
- Not changed here: whether the asymmetry is intentional is a hardware
  question, recorded in `open_questions.md`.

### One MIT torque table was applied to both protocol tiers

- Symptom: the new hardware-boundary validation bounded feed-forward torque per
  joint at 24/16/8 N·m, taken from the default tier's `move_mit`.
- Impact: the 1.11 tier bounds **every** joint at 16 N·m. Against the left arm
  the table was wrong in both directions — it would have refused a legitimate
  9-16 N·m command on joints 5-7, freezing a running impedance loop at its last
  setpoint, and it would have admitted 17-24 N·m on joints 1-2 for the SDK to
  raise on. The defect was introduced by the validation change itself and found
  the same day by running against both arms.
- Not currently reachable in this deployment: the MIT controller's configured
  `torque_limit` is 8 N·m on every joint, so live traffic never enters the
  divergent range. The contract was still wrong, and the configs are
  per-deployment.
- Fix: `mit_limits_for_tier()` selects the bounds from the resolved firmware
  tier, the driver stores them at startup and logs them, and an unresolved arm
  falls back to the default tier — which is the driver the SDK builds when no
  tier is given, so the fallback matches what the arm is actually driven with.
- Evidence (L3, 2026-08-12): right arm logs bounds
  `[24, 24, 16, 16, 8, 8, 8]`, left arm logs `[16] * 7`; a 12 N·m command on
  joint 6 is refused by the right arm against `[-8, 8]`, and a 20 N·m command
  on the same joint is refused by the left arm against `[-16, 16]`.

## 2026-08-13 (the correction slice — defects the Phase 1A work itself introduced)

An external review of commits `c234469..14c6eff` found five real defects. Each
was verified against the code before being accepted; two were severe. They are
recorded here because three of them were introduced *by* this sprint's own
changes, which is the interesting part.

### A NaN became the maximum commanded torque

- Symptom: `clamp(value, limit)` in the MIT controller is
  `max(-limit, min(limit, value))`. For NaN, `min(limit, nan)` returns `limit`,
  so `clamp(nan, 8.0) == 8.0` and `clamp(inf, 8.0) == 8.0`.
- Impact: a corrupt gravity solve, a bad trajectory point or a bad live gain
  left the controller as **full commanded torque**, and reached the driver as a
  perfectly plausible number. The hardware-boundary non-finite check added
  earlier in this sprint could not catch it: by then the value was finite.
- Fix: `first_non_finite()` checks every control value before any saturation.
  On a corrupt value the controller holds the measured pose with zero
  feed-forward torque — not silence, because this firmware executes the last
  setpoint it received indefinitely, and not the gravity term, because that is
  the most likely source of the corruption.
- Rule this generalises to: a saturating helper must never be the first thing
  that sees untrusted input. Escalated to `.claude/rules/ros2-development.md`.

### Authority loss stopped the stream but not the goal

- Symptom: the authority callback cleared the trajectory and hold state, but the
  `FollowJointTrajectory` execute loop owns its own buffer and had no authority
  check.
- Impact: the goal stayed active until some unrelated condition timed it out,
  and could still report on a run that had lost permission to command. The claim
  that an epoch change "aborts in-flight work" was therefore only half true.
- Fix: the callback latches a structured reason; the action loop consumes it and
  makes the terminal transition, so the abort happens on the thread that owns
  the goal rather than from an unrelated callback.

### The authority promised what admission would refuse

- Symptom: `accepts_motion` meant "state is READY", while `admit()` also
  requires the commander to own the device.
- Impact: once command stamping goes live, a controller would be told it may
  stream and then have every command refused with `NO_OWNER`.
- Fix: the field is now `motion_ready` — hardware readiness, which is what it
  always was — and `may_command(owner)` answers permission with the same checks
  as admission minus the sequence. A test pins the two against each other.
- Not taken: the review's suggestion to fold ownership into `accepts_motion`.
  Nothing claims ownership yet, so that would have made it permanently false and
  stopped both arms.

### Two processes could mint the same unit-safety generation

- Symptom: every node ran its own `UnitSafety` writer, and `observe()` ignored
  equal epochs.
- Impact: two writers could publish "5, stopped" and "5, rearmed" with no
  ordering between them, and a receiver could not tell a contradiction from a
  duplicate.
- Fix so far: generations carry the writer that minted them, an observer refuses
  to mint at all, and an equal-generation contradiction is counted with the stop
  winning. The single writer itself is still open — a device must be able to
  stop itself without another process being alive, so the target needs the
  device stop to be a device-level fault while only the writer allocates unit
  generations.

### Absence of an authority was read as permission

- Symptom: the controller fell back to its legacy gates whenever no authority
  had ever arrived.
- Impact: fail-open. A namespace typo, a QoS mismatch and an old driver are
  indistinguishable from the controller, and only one of them is a
  configuration anybody chose.
- Fix: requiring the authority is the default; the legacy gates survive only in
  a named development profile. The launch derives `expected_device_id` from the
  same CAN port as the driver, so a controller cannot be gated by the other arm.

### One shared MIT configuration meant two things on two arms

- Symptom: C8 was handled by refusing unencodable commands at the boundary. A
  `torque_limit` above 16 N·m is accepted by the right arm (default tier) and
  refused by the left (1.11 tier).
- Impact: "refuse loudly" is not enough for coordinated execution. A refused MIT
  command leaves the firmware holding its previous setpoint, so a dual-arm
  activity would have one arm moving and one frozen.
- Fix: the driver publishes `AgxDeviceCapability` — the envelope its protocol
  tier can encode — latched, and the controller fits its configured limits to
  its own device before commanding. Reducing a ceiling never grants authority it
  did not have, and per-arm capability is preserved for independent operation.
- Evidence (L3, 2026-08-13): `torque_limit: [20]*7` against the left arm reduces
  every joint to 16; against the right arm it keeps 20 on joints 1-2, reduces
  3-4 to 16 and 5-7 to 8. Same configuration, two correct envelopes, no runtime
  refusals.
- Still open: synchronized `both_arms` execution must be preflighted against
  both devices as a whole, which is coordinator work.

## 2026-08-13 (found on hardware while splitting device and unit safety)

### A verified emergency stop stopped latching the device

- Symptom: after moving unit-generation allocation out of the driver, an
  emergency stop set the authority to FAULTED and the arm was back to
  `state: READY, motion_ready: true` seconds later.
- Cause: `_sync_authority` is a *derived* mapping that runs every publish cycle
  at 200 Hz. It rebuilds the state from the driver's gates, so anything the
  e-stop set directly is overwritten on the next tick. Until this change the
  local unit stop happened to be the thing that held it, because
  `_sync_authority` returns early while the unit is stopped. Removing the local
  stop removed the latch with it, and nothing replaced it: a verified stop left
  the arm accepting motion.
- Fix: an explicit `_estop_latched` flag that the derived mapping honours, set
  by the emergency stop and cleared only by `clear_fault_lockout`. The response
  message says which latch it cleared.
- The general shape, worth remembering: **a derived state mapping erases
  anything set directly.** Whatever must survive it has to be an input to it,
  not an output written behind its back.
- Only hardware surfaced this. The unit tests passed throughout, because none
  of them ran the publish loop after an e-stop — a gap the new regression closes.

## 2026-08-14 (found while following the refinement proposal into the hand bridge)

### The hand publishes at the arm's rate, ten times faster than it has data

- Symptom: each `omnihand_bridge` cost ~160 % of a core while its bus carried
  25 frames/s, and cost the same whether a hand was doing anything or not.
- Cause: `_publish_feedback` rebuilt and published all three feedback messages
  on every timer tick, and the timer ran at `pub_rate` — which every bringup
  filled with the **arm's** publish rate by forwarding `pub_rate` into
  `start_omnihand_bridge.launch.py`. The hand's joints change at
  `joint_read_rate` (20 Hz) and its status and tactile once a second, so nine of
  every ten wakes carried nothing new.
- Measured (`scripts/profile_hand_bridge.py`, mock backend, no CAN): 41.5 % of a
  core at `pub_rate` 200, of which only 4.3 % was the tick body — the rest was
  the executor waking 200 times a second to run it.
- Fix: publication is gated on new data (a joint sample per readback, status on
  change plus a 2 Hz heartbeat, tactile at its read interval); the timer paces
  acquisition at twice the readback interval; `pub_rate` is a ceiling that can
  throttle publication and never drive it; bringups pass `hand_pub_rate` and
  `hand_joint_read_rate`. Result: 7.3 %, and flat across `pub_rate`.
- The general shape: **a rate argument that is forwarded rather than chosen
  belongs to whoever it was chosen for.** 200 Hz is right for an arm whose
  firmware pushes continuously; it was never a statement about the hand.

### Two commanders write the same hand, and neither knows about the other

- Symptom: not yet observed in a run — found by mapping the command surfaces the
  proposal asked about. Recorded before it is hit.
- Evidence: `omnihand_skill_controller_node` commands the hand over the shared
  `control/joint_states` topic, and republishes the grasp target at 20 Hz for as
  long as it holds — deliberately outside the coordinator's resource model, so
  that a hold does not block the side's resources. Meanwhile
  `omnihand_follow_joint_trajectory` commands the same hand over
  `control/omnihand/joint_trajectory`. Both land in `_submit_command`, which is
  latest-wins and keeps exactly one `pending_command`.
- Consequence, and it is a correctness one rather than a cost one: a hold
  republish issued while a trajectory goal is in flight replaces that goal's
  target within ~50 ms, and `_await_delivery` then sees `command_pending` clear
  when the **hold's** target verifies. The action reports the trajectory
  delivered when the hand never went there.
- Also relevant to CPU: every hold republish is a real SDK write plus its
  verification readback, so a held grasp costs 20 writes/s per hand
  indefinitely. This is the "recurring post-grasp hold traffic" in phase 2C.
- Not yet fixed. The hand's `DeviceAuthority` already carries `owner_id` and the
  bridge already serves `claim_device` — but **nothing ever claims a hand**
  (only the MIT controller claims, and only its arm), and the command surfaces
  do not check ownership. Closing this is 2C's single-goal arbitration bullet;
  it changes admission behaviour on the production path, so it is its own slice.

### The hand bridge's cost is a vendor busy-wait, not our polling

- Symptom: after the publication fix above, each `omnihand_bridge` still cost
  ~110 % of a core with its bus carrying 25 frames/s.
- Cause, measured per thread on hardware (L3, 2026-08-14): each bridge process
  carries **exactly one thread more than the same bridge on the mock backend**
  (23 vs 22), and that thread runs at **100.0 % of a core with `wchan=0`** — it
  never sleeps. It appears when the vendor session is opened, lives in compiled
  C++ (`agibot_hand_core`, `libomniHandPro25Can.so`), and is reachable from no
  call site of ours. Our tick thread is 10.9 %; every SDK call we make sums to
  ~50 ms of wall clock per second, about 5 % of a core.
- Why nothing found it before: `MeasuredSdk` instruments the calling thread, and
  a per-process CPU figure attributes the whole sum to the node. Only a
  per-thread census separates a library's internal thread from the code that
  opened it. **A cost that belongs to no call site is invisible to call-site
  profiling, however careful.**
- Consequence for the plan: phase 2C's transport-efficiency remedies — dropping
  the read-before-write, polling only under ownership, bounding round trips per
  setpoint — each divide up the 5 %. They stay worth doing for the CAN bus and
  for latency. They are not the CPU answer, and 2C now says so.
- Not fixed, and not fixable from this side. The SDK exposes report *periods*
  for the device (`set_all_error_report_periods` and siblings) but nothing about
  its own reader thread. The options are a vendor fix, a patch in the tracked
  vendor fork under the pinned-SDK constraint, or accepting one core per hand and
  keeping it off the control cores.
- Mitigating fact: it is a native thread, so it holds no GIL. It burns a core
  without starving the Python nodes — both arms held MIT at 100 Hz throughout the
  measurement, and all four buses ended with zero drops, misses or errors.

  **Fixed 2026-08-14** in the tracked vendor fork (`jetson-orin-socketcan`,
  `f43a0e9`). `CanBusDeviceSocketCan::RecvFrame` called `read()` on a
  **non-blocking** socket in a loop with nothing in between; the comment on that
  line already named the reason — a blocking read would keep the thread from
  being released at shutdown — but the answer to that is `poll()` with a timeout,
  not a spin. The receive thread now sleeps in `poll()` at 0.2 % instead of
  running at 100.0 %, the bridge process falls from 110 % of a core to 13 %, and
  request latency is unchanged (2.23 ms mean against 2.18 ms). Reported upstream:
  `docs/assets/omnihand/omnihand_vendor_socketcan_recv_report.md`.

  Worth keeping as a pattern: **the diagnosis was already written in the code, as
  a comment explaining why the wrong thing was done.** A note saying "we cannot do
  X because Y" is a description of an unsolved problem, not of a settled one, and
  is worth re-reading whenever the cost of the workaround shows up in a
  measurement.

### The poll() fix reintroduced the spin under a socket error

- Symptom: none observed — caught by review while planning fault-injection
  coverage, before it could be hit.
- Cause: the first version of the vendor patch checked only `poll()`'s return
  value. `POLLERR`, `POLLHUP` and `POLLNVAL` are reported in `revents` whether or
  not they were requested, and they do not clear by themselves, so on a
  persistent socket error `poll()` returns immediately every time, the `read()`
  fails, and the loop spins exactly as before — **during a bus fault**, which is
  when the CPU is wanted for recovery rather than least.
- Fix (`c037b38` on `jetson-orin-socketcan`): `POLLNVAL` ends the loop, since the
  descriptor is gone; `POLLERR`/`POLLHUP` without `POLLIN` waits out the poll
  interval before retrying; `POLLIN` wins over a simultaneous error flag, because
  queued frames would otherwise be discarded while a caller waits for them.
- L3 evidence, 25 s link-down on `hand_right` under a running bridge: receive
  thread 0.12 % of a core, in `do_sys_poll` for every sample; process cost fell
  from 14 % to 6 % because requests fail instead of waiting; fault detected,
  backoff entered, recovery automatic, feedback back to 20.1/s. Shutdown still
  completes promptly, which is the property the original blocking-read comment
  was protecting.
- **Stated plainly: this run does not exercise the new branches.** A downed CAN
  link delivers nothing, so `poll()` reaches its timeout and the ordinary path
  handles it. `POLLERR`/`POLLHUP` need the adapter removed and `POLLNVAL` needs
  the descriptor closed underneath the thread. The branches are reasoned, not
  observed, and the docs say so rather than counting the link-down as coverage.
- Recovery latency worth noting separately: the link was down 25 s and recovery
  was declared 50 s after the fault, because the probe cadence escalates 5x
  (2 s to 10 s) after eight failed probes and three clean readbacks are required.
  That escalation exists to stop a hammering hand congesting a *shared* bus; on
  per-device buses the reason is largely gone and only the latency remains.

## 2026-08-14 (found by putting the hand's authority into the production path)

Both defects below were latent for as long as the authority existed. Nothing had
ever claimed a hand, so no claim was ever refused, and the two paths that only
run on a refusal had never executed.

### The hand's claim service collided with the arm's

- Symptom: a hand trajectory refused itself with "arm_right is held by
  'reactive:...'" — naming the *arm* while the test had claimed the *hand*.
- Cause: the bridge served `claim_device` and the arm driver serves
  `claim_device`, both in the same per-side namespace, so both resolve to
  `/<side>_arm/claim_device` and a client silently reaches whichever it
  discovered first. The trajectory took the arm's authority, left the hand
  unclaimed, and then failed delivery having sent nothing.
- Fix: the hand's service is `control/omnihand/claim_device`, next to the rest of
  its hand-specific surface, exported as `HAND_CLAIM_SERVICE` so no caller can
  spell it independently. Four services now, one per device.
- The tell was in the message. A refusal that names the wrong device is worth
  more than a refusal that just says no, which is why the reasons carry the
  device id at all.

### A refused claim killed the bridge

- Symptom: after one successful claim, the claim service stopped answering
  entirely — clients timed out with "did not answer", then "is not available".
- Cause: `rclpy` caches a logger's severity per **call site** and raises if it
  changes. `(self.get_logger().info if accepted else self.get_logger().warn)(msg)`
  is one call site with two severities, so the first refused claim raised out of
  the service callback and the node died. "Not available" was accurate: by then
  there was no bridge.
- Fix: two call sites, plus a regression that refuses a claim and asserts the
  service answers — the path that had never run.
- Worth keeping: **the failure looked like a client problem.** A service that
  stops answering points at the caller, and the investigation went there first.
  A node that dies on an error path is invisible to every test that only takes
  the happy path, and this one had no test at all.

## 2026-08-15 (found by grasping a real object with the reactive primitive)

### Tactile was published once a second, which is exactly when the grasp gives up

- Symptom: every reactive grasp on hardware failed before moving a finger, with
  `tactile stale (1.01s > 1.0s)`. Three consecutive runs, `state=FAILED`,
  `contact_score=0.000`.
- Cause: the bridge read and published tactile at `SDK_TACTILE_READ_INTERVAL_S`
  = 1.0 s, and the skill controller rejects a reading older than
  `tactile_stale_sec` = 1.0 s. The publication interval *was* the staleness
  limit, so a sample was at the limit the moment it arrived.
- Why it was invisible before: until the publication rework the bridge
  republished the cached tactile snapshot on every timer tick, so the message
  arrived with a fresh header stamp while carrying values up to a second old.
  The consumer measured arrival, not content, and read fresh. **The staleness
  check was passing on a lie**; making publication honest is what exposed it.
- Deeper cause: tactile was classified once, as a diagnostic. It is a diagnostic
  to everyone except the reactive primitive, whose whole definition is that it
  ends where the sensor says rather than where the clock does. For that owner it
  is control-critical acquisition.
- Fix: tactile has two cadences and two lanes, chosen by **who holds the hand**.
  With no owner or a trajectory owner it stays a 1 Hz diagnostic on the
  `DIAGNOSTIC` lane. While the reactive primitive holds the device it runs at
  the acquisition rate on the `ACQUISITION` lane. Measured on the right hand:

  | owner | tactile published | bridge CPU |
  | --- | --- | --- |
  | none | 1.0/s | 14.5 % of a core |
  | trajectory | 1.0/s | 14.2 % |
  | **reactive** | **20.0/s** | **26.8 %** |
  | released again | 1.0/s | 14.6 % |

  The cost of contact-seeking is +12 % of a core and it is paid only while
  something is waiting on the sensor.

### The same rounding bug, in the gate I had just written

- Symptom: the reactive cadence was configured at 20 Hz and the bridge's own
  call counter reported roughly half that.
- Cause: a `now - last >= interval` gate inside a loop already paced at that
  same interval. Ordinary jitter makes cycles miss it. This is the third time
  this pattern has cost rate in this repository — it made a 20 Hz joint readback
  measure 15.4 Hz, which is why the acquisition loop was changed from gating to
  pacing in the first place, and I reintroduced it one function away from the
  comment that says so.
- Fix: an interval at or below the loop period means every cycle; a slower
  cadence is compared with half a period of tolerance. `read_tactile` now runs
  at 20/s under a reactive owner, confirmed by the bridge's own per-call counter.

### A pricing script that stopped listening and blamed the bridge

- Symptom: the same reactive cadence measured 6.5/s at the subscriber while the
  bridge's own counter said 20/s.
- Cause: the script sampled process CPU with `time.sleep(window)`. It was not
  spinning during the window, so it was not receiving, and it reported its own
  deafness as the publication rate. Sampling across a window the probe *spends
  spinning* gives 19.9/s.
- Rule this reinforces: a measurement whose method can fail silently is not
  evidence. The bridge-side call counter and the subscriber-side count disagreed,
  and the disagreement was the signal — one number alone would have been believed.
