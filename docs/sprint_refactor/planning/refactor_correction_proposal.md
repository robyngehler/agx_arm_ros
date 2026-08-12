# Refactor Correction Proposal

**Status:** review correction layer  
**Reviewed branch:** `ROS2_Duo_System_V02_refactor`  
**Reviewed delta:** `c234469..14c6eff`  
**Latest commits reviewed:**  
- `6d110ef` — mixed arm firmware recorded as permanent constraint C8  
- `14c6eff` — MIT controller consumes `AgxDeviceAuthority`

## 1. What the latest commits correctly close

The latest work should be retained:

- The two Nero arms are **permanently different protocol tiers**: right arm firmware 1.06/default tier, left arm firmware 1.11/`NeroFW.V111`. Protocol-derived limits, encodings, status interpretation, and measurements must therefore stay per-device/per-tier.
- The MIT controller now consumes `feedback/authority`, stops publishing when motion authority is lost, and clears stale trajectory/hold state on device- or unit-epoch changes.
- The legacy `feedback/hand_window_active` gate is now explicitly transitional rather than the target authority contract.
- Hardware evidence confirms MIT output falls from ~100 Hz to zero after emergency-stop authority loss and resumes after authority returns.

These changes move Phase 1A in the intended direction. The remaining corrections below should be closed before Phase 1A is considered complete.

---

## 2. P0 — Authority loss must terminate the active FJT goal

### Problem

`_authority_callback()` clears `active_trajectory`, `hold_reference`, and `holding_final_point`, but it does **not** terminate the active `FollowJointTrajectory` goal.

The action execute loop owns its own local trajectory buffer and has no authority-loss check. Therefore an authority transition can stop MIT streaming while the ROS action remains active until another condition eventually aborts or times out.

This is weaker than the documented claim that an epoch/authority change “aborts in-flight work”.

### Correction

Add an explicit authority-abort path shared by the callback and the action loop:

- latch `authority_abort_requested` plus a structured reason;
- clear trajectory and hold state atomically;
- make the action loop immediately call `goal_handle.abort()` when the latch is observed;
- return a deterministic error string identifying authority loss / epoch invalidation;
- clear the latch only when the affected goal has terminated.

Do not call the action goal terminal transition concurrently from an unrelated callback if the action implementation cannot guarantee that safely; signal the execute loop instead.

### Acceptance

- active FJT goal reaches a terminal aborted state within a bounded interval after authority loss;
- no FJT goal can later report success after its device or unit epoch changed;
- L1/L2 test covers `READY -> STOPPED`, device epoch bump while still READY, and unit epoch bump.

---

## 3. P0 — Reject non-finite MIT values before clamping

### Problem

The MIT controller still uses:

```python
def clamp(value, limit):
    return max(-limit, min(limit, value))
```

without a finite-value check. Non-finite values can therefore be transformed before the hardware-boundary validator sees them. Live gain/gravity parameters are also mutated while the parameter batch is still being validated.

The new driver-side non-finite rejection is valuable, but it must remain the **second** protection layer, not the first place malformed controller state is detected.

### Correction

- validate every trajectory/control value with `math.isfinite()` before saturation;
- reject/abort the current control operation on NaN/Inf rather than saturating it;
- validate the complete live-parameter batch into temporary values first;
- only commit parameter changes after the whole batch passes shape, range, and finite checks;
- remove unreachable code after `_on_set_parameters()` returns.

### Acceptance

Tests prove that NaN/Inf in trajectory values, gravity feed-forward, gains, or live scalar parameters never produces a finite maximum command.

---

## 4. P0/P1 — Freeze one complete command-stamp contract before changing `MoveMITMsg`

### Problem

The authority implementation already defines the actual admission identity as:

```text
owner_id
device_epoch
unit_safety_epoch
sequence
```

but the documentation still uses inconsistent forms:

- checklist: “epochs and a sequence”;
- integration plan: both epochs plus sequence;
- `open_questions.md`: `control_epoch` plus sequence;
- hand command plan: `owner_id`, `device_epoch`, `sequence`, currently omitting the unit epoch.

Implementing the ABI change before resolving this creates another migration immediately afterwards.

### Correction

Freeze one common command stamp:

```text
string owner_id
uint64 device_epoch
uint64 unit_safety_epoch
uint64 sequence
```

Use the same semantics for every commandable device. If a device intentionally does not participate in unit safety, document that as an explicit exception instead of silently dropping the field.

Extend `MoveMITMsg` once and migrate producer + consumer + tests in the same change set.

### Acceptance

- `CommandStamp`, ROS messages, docs, MIT producer, driver admission, and hand contract use the same field names and semantics;
- stale device epoch, stale unit epoch, wrong owner, and stale sequence all have tests at the live boundary.

---

## 5. P1 — Make `accepts_motion` consistent with ownership/admission

### Problem

`AuthoritySnapshot.accepts_motion` currently means only:

```text
state == READY
```

while `DeviceAuthority.admit()` additionally requires a current owner and matching owner identity.

The MIT controller now treats `accepts_motion` as its primary gate. Once live command stamping is enabled, this can produce a state where MIT believes it may stream while the hardware boundary rejects every command with `NO_OWNER` or `NOT_OWNER`.

### Correction

Define the contract explicitly.

Preferred interpretation:

```text
accepts_motion =
    state == READY
    AND owner_id is non-empty
    AND unit_stopped == false
```

The MIT controller must additionally verify that `owner_id` matches its configured commander identity.

Claim the MIT controller as owner before exposing commandable READY state in coordinated profiles. Ownership change and epoch change must be one atomic authority transition.

If the desired meaning is only “hardware is ready”, rename the field to `motion_ready` and do not present it as controller permission.

### Acceptance

There is no published authority snapshot for which the MIT controller is told it may command but a correctly stamped command from that controller is rejected solely because no owner exists.

---

## 6. P1 — Make unit safety single-writer

### Problem

`UnitSafety` currently exists independently in multiple processes and each instance can increment its own epoch. `observe()` rejects equal epochs.

Two writers can therefore produce conflicting states with the same epoch number, e.g. one process publishes epoch 5/stopped and another epoch 5/rearmed. Neither can establish a globally ordered truth.

The coordinator calling every arm stop service covers the current e-stop path but is not a unit-wide epoch authority.

### Correction

Introduce exactly one writer for `unit_safety_epoch`.

For the MVP this should be either:

- the Unit Coordinator; or
- a small dedicated Unit Safety Supervisor.

Device nodes may request unit stop/rearm but only observe the authoritative published result.

Also correct the message/docs: the epoch increments on **every unit-safety transition**, including rearm, not only on stop/fault.

### Acceptance

- exactly one process allocates unit-safety generations;
- all four device authorities converge on the same `(epoch, stopped)` state;
- restart/late-join tests cannot create an equal-epoch contradiction.

---

## 7. P1 — Remove fail-open authority bootstrap from coordinated hardware profiles

### Problem

The new MIT migration intentionally uses:

```text
no authority ever received -> legacy gates decide
```

and the first authority message does not invalidate already-running work.

This is useful for staged migration, but it is fail-open if retained in production: a namespace/QoS/topic wiring error becomes indistinguishable from “legacy driver”.

The current arm drivers already publish authority, so coordinated hardware no longer needs this as a permanent compatibility rule.

### Correction

Add an explicit profile-level policy:

```text
require_device_authority: true   # coordinated hardware default
```

When required:

- do not publish MIT commands before the first valid authority snapshot;
- validate `device_id` against the expected arm;
- reject/mask authority from another device;
- treat missing authority at startup as NOT_AUTHORISED after a short startup grace period.

Keep the legacy fallback only in a clearly named development/degraded profile and remove it when the migration closes.

### Acceptance

A wrong authority topic, wrong `device_id`, or missing authority publisher cannot silently enable MIT streaming in coordinated production.

---

## 8. P1 — C8 requires preflight compatibility, not asymmetric runtime refusal

### Problem

C8 correctly records that the two arms have different protocol limits. The current handling still allows a shared MIT configuration above the left arm's supported torque bound: the right arm accepts it while the left driver refuses it.

For a dual-arm activity this can leave one arm executing while the other remains on its previous setpoint. “Refuse loudly” is not sufficient for synchronized/coordinated execution.

### Correction

Resolve effective control limits **per arm/tier before execution**:

- registry/resolved manifest carries the firmware/protocol tier or its resolved capability set;
- MIT controller parameters are generated/validated per arm, not assumed symmetric;
- a `both_arms` execution is preflighted against both devices before either side starts;
- strict synchronized execution fails as a whole if either side cannot encode the requested control envelope.

Do not use the intersection globally unless that is intentionally the desired operating envelope; preserve per-arm capability where independent operation permits it.

### Acceptance

No dual-arm goal can enter execution when one side will deterministically reject its configured MIT command bounds.

---

## 9. P1 — Define the SDK worker's safety-latency guarantee before routing all calls through it

### Problem

The serialized worker removes races, but its safety lane only has priority over **queued** work. It cannot preempt a vendor SDK call already blocking on the worker thread.

The Phase-0 fault evidence already includes multi-second stalls, so moving every SDK operation behind one thread can trade a race for head-of-line blocking on emergency stop.

### Correction

Before full worker wiring:

- measure worst-case latency of every SDK call used in the worker;
- classify calls as bounded / timeout-controlled / potentially blocking;
- define a maximum admissible safety-command queue latency;
- ensure potentially blocking reads/recovery calls have enforceable timeouts or are moved out of the path that can block emergency stop;
- instrument queue wait time separately from SDK execution time.

### Acceptance

An L3 stress test demonstrates the configured emergency-stop command reaches its hardware path within the defined latency bound while normal SDK work, recovery, and MIT streaming are active.

---

## 10. P1/P2 — Define failure semantics for rejected streaming commands

### Problem

Configured joint-limit violations are still forwarded because simply dropping one MIT command can leave the firmware executing its previous setpoint indefinitely.

This reveals a general rule: for this firmware, “reject” is not automatically fail-closed.

### Correction

For safety-critical rejection during an active MIT stream:

1. reject the malformed/out-of-policy payload;
2. invalidate the current control epoch / trajectory ownership;
3. issue the defined damped-stop or verified hold path;
4. abort the active FJT goal;
5. require re-synchronisation before normal streaming resumes.

Only after this behavior exists should configured joint-limit violations be promoted from warning to rejection.

### Acceptance

A deliberately invalid command cannot leave a previous moving MIT setpoint active without triggering the defined stop/hold transition.

---

## 11. P2 — Close documentation drift introduced by the current transition

Apply these corrections together with the next planning/docs commit:

- `target/README.md` still says “MIT consuming the authority” is open although commit `14c6eff` landed and hardware-validated it.
- `target/README.md` and the canonical plan still name branch `ROS2_Duo_System_V02` rather than the active `ROS2_Duo_System_V02_refactor` branch.
- `integration_plan.md` still requires `tea_pour_left_v1` after every phase while the checklist explicitly defers it until re-teach. Until then, the L2 activity harness is the mandatory regression.
- `AgxDeviceAuthority.msg` says `unit_safety_epoch` is bumped only on stop/fault, while `UnitSafety.rearm()` increments it as well.
- `open_questions.md` still calls the arm wire field `control_epoch`; align it with the two-epoch `CommandStamp`.
- historical “starting point” statements in the canonical plan should be marked as historical or rewritten when a landed Phase-1 change makes them false.

---

## 12. Recommended correction order

Do not restart the refactor. Insert one short **Phase-1A contract-closure slice** before worker/runtime admission is completed:

1. finite-value and atomic MIT parameter validation;
2. immediate FJT termination on authority/epoch loss;
3. freeze the complete command stamp;
4. define `accepts_motion` + owner semantics;
5. establish the single unit-safety writer;
6. make authority mandatory in coordinated hardware profiles;
7. add C8 preflight/per-arm configuration validation;
8. define and test SDK-worker safety latency;
9. wire `MoveMITMsg` stamping + live `DeviceAuthority.admit()`;
10. quarantine legacy motion ingress and then close Phase 1A.

After this slice, continue with Phase 1B and the existing Phase 2–6 order.

## 13. Phase-1A close criterion

Phase 1A should be considered complete only when all of the following hold:

```text
Every arm SDK call
    -> one serialized hardware worker

Every normal arm command
    -> expected device
    -> current owner
    -> current device_epoch
    -> current unit_safety_epoch
    -> monotonic sequence
    -> finite and encodable payload

Every authority or epoch loss
    -> MIT publication stops
    -> active FJT goal terminates
    -> stale trajectory/hold state is discarded

Every unit safety transition
    -> one globally ordered generation
    -> observed by every device authority

Every coordinated dual-arm execution
    -> preflighted against both permanent firmware tiers

No production legacy ingress
    -> bypasses the same admission boundary
```

This preserves the current architecture and commit direction while closing the remaining gap between “authority is published and observed” and “authority is the actual enforceable runtime contract”.
