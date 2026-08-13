# Proposal: Recovery Isolation and Independent Watchdog Boundary

**Status:** Proposed  
**Date:** 2026-08-13  
**Scope:** Duo Nero V02 refactor, arm transport recovery, software control integrity, and future external watchdog integration  
**Primary goal:** Define the boundary between faults that the Jetson/ROS/SDK stack must handle deterministically and faults for which bounded protective action requires an independent watchdog below the software control stack.

---

## 1. Motivation

Recent Phase 1A fault-injection measurements exposed an important limit in the current recovery design.

Healthy SDK calls are normally short enough to fit the software control budget:

| SDK call | Observed behavior |
|---|---:|
| `connect` | ~8.8 ms mean, ~10.1 ms max |
| `enable` | ~0.4 ms mean, ~1.05 ms max |
| ordinary MIT/control calls | low-ms or sub-ms regime in healthy operation |

Under a real transport failure, this assumption no longer holds:

| SDK call | Observed failure behavior |
|---|---:|
| `disconnect` | ~666 ms mean, **1000.2 ms max** |
| `get_firmware` | **175 ms** |
| full recovery sequence | **13.1 s publish-loop gap** across retries |

The important distinction is:

1. a single blocking vendor SDK call cannot be made shorter by splitting the worker queue more finely;
2. the 13.1 s loss of the publish/acquisition loop is still a software architecture problem and must not be accepted as inherent.

The refactor therefore needs an explicit boundary:

- **steady-state command integrity and bounded software reactions remain the responsibility of the Jetson stack;**
- **bounded protective action while the normal transport is blocked or being destructively recovered cannot be guaranteed through the same SDK/CAN control path and must be treated as an independent-watchdog concern.**

---

## 2. Core Architecture Decision

The target architecture separates three layers:

```text
Task / Motion Layer
Coordinator / MoveIt / MIT
            |
            v
Software Control-Integrity Layer
DeviceAuthority
UnitSafety
owner + epochs + sequence admission
serialized SDK ownership
fault lockout / recovery state
            |
            v
Normal arm CAN transport
            |
            v
Independent Watchdog / Protective Layer
not dependent on ROS, Jetson worker,
vendor SDK recovery, or coordinator liveness
```

The external watchdog is **not** a replacement for the software refactor.

The software stack must still:

- detect faults;
- stop normal motion publication;
- invalidate stale command generations;
- isolate recovery from acquisition;
- maintain observable fault state;
- prevent automatic resume;
- perform verified recovery.

The watchdog exists for the remaining case in which the normal host/SDK/transport path cannot provide a bounded protective response.

---

## 3. Revised SDK Ownership Rule

The previous rule:

> all SDK calls for one arm run through one worker

is too strong and is contradicted by the measured recovery behavior.

Replace it with:

> **Exactly one component owns the SDK session at any instant.**

### 3.1 Steady-state ownership

During normal operation:

```text
Device SDK Worker
    owns:
      MIT commands
      feedback reads
      enable/mode operations
      normal control services
```

All steady-state reads and commands remain serialized through the worker.

### 3.2 Recovery ownership

During `RECOVERING`:

```text
Recovery State Machine
    exclusively owns:
      disconnect
      interface reset/restart
      reconnect
      firmware query
      enable verification
      feedback re-establishment
```

The normal worker is quiesced before ownership transfers to recovery.

No normal motion/read task may touch the SDK until recovery returns ownership.

### 3.3 Safety implication

The worker safety lane only prioritizes work that has **not yet started**.

It does not preempt a vendor call already blocking inside the SDK.

Therefore the software must not claim a bounded emergency-stop latency while a blocking teardown/reconnect call owns the transport.

---

## 4. Recovery Must Be Removed from the Publish/Acquisition Loop

The observed 13.1 s publish-loop gap is not an acceptable architectural boundary.

Current coupling:

```text
publish/acquisition loop
        |
detect recovery condition
        |
        v
synchronous recovery
disconnect -> reset -> connect -> verify
        |
13 s loop loss
```

Target:

```text
Acquisition / Health Path
        |
        +--> detect fault
        +--> latch DeviceAuthority fault
        +--> stop normal command admission
        +--> request recovery
        |
        +--> remain alive and publish cached/stale health state

Recovery State Machine
        |
        +--> acquire exclusive SDK ownership
        +--> perform bounded recovery steps
        +--> publish progress / result
        +--> return ownership only after verification
```

The acquisition path must never synchronously wait through multi-attempt recovery.

### Acceptance

A recovery attempt may take seconds, but it must not:

- freeze ROS health/state publication for the whole attempt;
- block unrelated unit-level processing;
- prevent timeout accounting;
- hide how long recovery has been active;
- leave normal command ownership ambiguous.

---

## 5. Recovery Escalation Must Be Fault-Classified

A full SDK `disconnect()` must not be the first response to every transport anomaly.

Introduce an explicit recovery ladder.

### Level 0 — local starvation / stale consumer

Examples:

- executor delay;
- Python/GIL stall;
- feedback socket temporarily not drained while kernel counters advance.

Action:

- do not destructively reconnect;
- latch degraded health;
- recover local processing.

### Level 1 — SDK/parser stale while interface is healthy

Action:

- reset parser/read state if supported;
- re-establish feedback without full transport teardown.

### Level 2 — SocketCAN/controller fault

Examples:

- bus-off;
- interface error state;
- transport-level socket fault.

Action:

- perform the least destructive validated interface restart available;
- avoid full vendor disconnect when possible.

### Level 3 — hard transport/session failure

Action:

- transfer SDK ownership to recovery;
- send the best available local safe command before teardown;
- perform disconnect/reconnect;
- remain in fault lockout until feedback and enable state are verified.

### Level 4 — bounded software reaction cannot be guaranteed

Action:

- external watchdog remains or becomes authoritative;
- software recovery continues only as a restoration procedure;
- normal control cannot resume until the watchdog explicitly releases authority.

---

## 6. External Watchdog Purpose

The watchdog covers failures such as:

- Jetson process stall;
- ROS executor starvation;
- coordinator crash;
- SDK blocking in recovery;
- normal controller heartbeat loss;
- local CAN interface/driver failure where the independent watchdog can still communicate;
- software restart while the robot remains physically powered.

It should detect at minimum:

```text
normal-controller heartbeat freshness
normal motion-command freshness
watchdog/host ownership state
optional bus-health indicators
optional device feedback freshness
```

The watchdog should be intentionally simple.

It must not reproduce the complete MoveIt/MIT stack.

---

## 7. Watchdog Takeover Contract

Proposed watchdog state machine:

```text
ARMED
  |
  | stale heartbeat / explicit stop / critical fault
  v
TAKEOVER_LATCHED
  |
  | protective action asserted
  | normal controller inhibited
  v
SAFE_LATCHED
  |
  | host reconnects and verifies system
  v
RELEASE_PENDING
  |
  | explicit release handshake succeeds
  v
ARMED
```

### 7.1 Trigger

A watchdog trigger may come from:

- missing host heartbeat;
- stale normal-control generation;
- explicit software stop request;
- physical stop input;
- independently detected critical bus/device condition.

### 7.2 Takeover

On takeover the watchdog:

1. latches ownership;
2. applies the simplest available protective primitive;
3. prevents normal controller commands from becoming authoritative;
4. remains latched across Jetson/ROS restart.

### 7.3 Recovery

The Jetson may:

- restart;
- reconnect the SDK;
- read feedback;
- rebuild DeviceAuthority state;
- verify hardware state.

It may **not** resume motion merely because feedback has returned.

### 7.4 Release

Release must be explicit and verified.

Suggested release preconditions:

```text
host heartbeat healthy
AND normal transport healthy
AND fresh device feedback
AND device in known stopped/held state
AND no device fault latch
AND UnitSafety permits rearm
AND operator/supervisor release policy satisfied
```

Only after watchdog release:

```text
device_epoch advances
owner is claimed again
new stamped MIT commands are accepted
```

---

## 8. Integration with DeviceAuthority

The watchdog should become a lower-level prerequisite of motion admission.

Preferred model:

```text
may_accept_motion =
    device state == READY
    AND correct owner
    AND device_epoch matches
    AND unit_safety_epoch matches
    AND sequence is fresh
    AND external_watchdog_released
```

The watchdog state may be represented either as:

- an explicit `EXTERNAL_INHIBIT` DeviceAuthority state; or
- a separate authoritative `external_inhibit` / `watchdog_released` field.

The second option is preferable if the watchdog is physically independent, because it keeps device lifecycle state and external protective authority distinct.

---

## 9. Protective Primitive

The watchdog should use the **simplest reliable protective primitive available**.

Preferred order:

1. dedicated hardware safety/drive-disable input, if supported;
2. dedicated vendor emergency-stop/disable command;
3. minimal firmware-specific safe CAN command;
4. full MIT takeover only as a last resort.

Because the two Nero arms permanently use different firmware/protocol tiers, a watchdog that sends CAN commands must resolve the target device's protocol tier explicitly.

Do not build a second general-purpose arm controller in the watchdog.

---

## 10. Same-Bus Limitation

An external controller on the same CAN wiring improves independence from the Jetson/ROS/SDK stack, but it does not protect against every physical communication failure.

It may still act when:

- Jetson software hangs;
- the Jetson SocketCAN interface fails;
- the normal SDK session blocks;
- the host process restarts.

It cannot guarantee communication through:

- broken CAN wiring;
- common transceiver failure;
- shared physical bus short/open;
- loss of device power;
- faults affecting both normal and watchdog transport.

Therefore distinguish:

### External communication watchdog

Purpose:

> protect against host/software/control-path unavailability.

### Independent protective-stop function

Purpose:

> protect independently of the ordinary communication path.

The latter may require a dedicated hardware safety input, safety relay/controller, or other independent mechanism and is outside the V02 software refactor unless the hardware interface is already available.

No claim that the ROS/watchdog combination is safety-rated should be made without a separate hardware and safety-engineering validation process.

---

## 11. Transmitter Ownership

Normal control and watchdog control must never issue competing actuator commands concurrently.

Required ownership phases:

```text
NORMAL
  Jetson control TX authoritative
  watchdog passive/monitoring

TAKEOVER
  watchdog becomes authoritative
  Jetson command admission revoked

RECOVERY
  Jetson may observe/reconnect
  Jetson normal motion remains inhibited

RELEASED
  watchdog relinquishes protective authority
  device epoch advances
  Jetson explicitly reclaims ownership
```

Where feasible, normal-controller transmission should be physically or logically inhibited while the watchdog is latched.

---

## 12. Software Work Required Before Watchdog Integration

The external watchdog must not become an excuse to leave avoidable software coupling in place.

Before or alongside watchdog prototyping:

### 12.1 Finish steady-state worker routing

Route normal SDK commands and reads through the serialized worker.

### 12.2 Isolate recovery

Move recovery out of the publish/acquisition path and give it explicit exclusive SDK ownership.

### 12.3 Measure queue and recovery latency

Record separately:

- command queue wait;
- SDK execution time;
- recovery-step duration;
- time in `RECOVERING`;
- feedback staleness;
- watchdog takeover latency once hardware exists.

### 12.4 Quarantine legacy ingress

No legacy motion path may bypass DeviceAuthority/admission.

### 12.5 Keep UnitSafety separate

`UnitSafety` remains software-wide command-generation invalidation.

It is not the protective watchdog itself.

A local device fault may request a unit stop, but local protective action must not depend on the UnitSafety writer being alive.

---

## 13. Documentation Corrections

Update the current refactor docs to replace the obsolete invariant:

> no arm SDK call outside the worker

with:

> **exactly one SDK owner at any instant. Steady-state commands and reads use the serialized device worker; destructive recovery exclusively owns the SDK session while the device is in `RECOVERING`.**

Also document explicitly:

- the measured ~1 s `disconnect()` failure latency;
- the 13.1 s recovery-induced publish-loop stall as an open Phase 1B defect;
- worker priority is non-preemptive once an SDK call has started;
- the pre-teardown damped-zero command is mitigation, not a bounded protective guarantee;
- an independent watchdog is the planned boundary for host/SDK unavailability.

---

## 14. Validation Plan

### L1 — logic

Test:

- SDK ownership transfer worker -> recovery -> worker;
- no worker task executes while recovery owns the session;
- stale commands are rejected after recovery epoch change;
- watchdog latch blocks motion admission;
- watchdog release requires explicit transition.

### L2 — mock/integration

Inject:

- blocking `disconnect`;
- delayed reconnect;
- recovery retry;
- lost watchdog heartbeat;
- host restart while watchdog remains latched.

Verify:

- acquisition/health state remains observable;
- active FJT goal aborts;
- normal motion does not resume automatically;
- watchdog ownership survives software restart.

### L3 — hardware

Measure separately:

1. healthy MIT + emergency-stop latency;
2. bus-off/interface-down fault;
3. recovery duration;
4. acquisition/health loop continuity during recovery;
5. external watchdog takeover latency once hardware exists;
6. explicit watchdog release and controller reacquisition.

---

## 15. Implementation Order

### Immediate V02 software work

1. Correct plan/checklist to use **exclusive SDK owner** rather than **all calls on one worker**.
2. Quarantine remaining legacy arm ingress.
3. Complete steady-state worker routing.
4. Re-run CPU, queue-latency, and healthy-path E-stop measurements.
5. Move recovery out of the publish/acquisition loop.
6. Add fault classification and least-destructive recovery escalation.
7. Keep failed recovery latched and observable.
8. Preserve DeviceAuthority/UnitSafety command invalidation.

### Watchdog design work

9. Determine the simplest reliable Nero protective primitive.
10. Determine whether a hardware drive-disable/safety input exists.
11. Select watchdog hardware with independent power/runtime path.
12. Define heartbeat, takeover, latch, and release protocol.
13. Integrate watchdog release into DeviceAuthority admission.
14. Prototype and fault-inject before any protective-safety claim is made.

---

## 16. Acceptance Criteria

The software refactor is complete when:

```text
steady-state SDK access
    -> exactly one serialized owner

recovery
    -> exclusive ownership
    -> does not block acquisition/health publication for its full duration

authority loss
    -> normal MIT publication stops
    -> active FJT terminates
    -> stale command generations cannot resume

hard transport recovery
    -> remains fault-latched until verified
    -> never implies a bounded protective-stop guarantee
```

The watchdog integration is complete when:

```text
host/control heartbeat stale
    -> independent takeover occurs within a measured bound

watchdog latched
    -> normal controller cannot resume motion

software restarts
    -> may observe and recover
    -> cannot reclaim motion authority

explicit verified release
    -> watchdog relinquishes authority
    -> device epoch advances
    -> normal controller reclaims ownership
```

---

## 17. Final Recommendation

Proceed with the current worker refactor, but formalize the newly measured boundary.

Do **not** treat the 1 s blocking `disconnect()` as a software latency problem that queue restructuring can solve. Treat it as evidence that the ordinary SDK/transport path cannot guarantee a bounded protective response during destructive recovery.

At the same time, do **not** accept the observed 13.1 s publish-loop outage. Recovery must be isolated from the acquisition/health path and must have explicit exclusive SDK ownership.

The resulting architecture should therefore be:

```text
local device stop and command invalidation
        -> software responsibility

deterministic steady-state SDK ownership
        -> software responsibility

observable, isolated recovery
        -> software responsibility

bounded protective action while host/SDK/transport is unavailable
        -> independent watchdog responsibility

return to motion after watchdog takeover
        -> explicit watchdog release
           + verified software recovery
           + new device generation
```

This boundary keeps the V02 refactor honest: the software stack remains responsible for everything it can deterministically control, while a future independent watchdog covers the failure regime that the measured vendor SDK and normal CAN recovery path cannot bound.
