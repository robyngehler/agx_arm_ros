# Unit safety: one writer, and what has to change to get there

status: implemented 2026-08-13, verified on hardware
date: 2026-08-13
closes: correction proposal §6, checklist "unit safety, part 2 of 2"

## Where it actually stands today

Not "partly done" — the cross-process half does not exist at all:

- every node constructs `UnitSafety(device_id)` as a **writer**;
- the arm driver's `emergency_stop` calls `stop()` on its own instance, so its
  own counter advances and its own devices go STOPPED;
- `clear_fault_lockout` calls `rearm()` on the same instance;
- **nothing publishes or subscribes unit safety.** `observe()` is implemented and
  tested, but no topic carries a generation between processes.

So `unit_safety_epoch: 1` on the right arm and `unit_safety_epoch: 1` on the
left arm are unrelated integers that happen to be equal. What actually makes an
emergency stop reach both arms today is the coordinator calling each side's stop
service — a fan-out of independent local stops, not a unit-wide generation.

The 2026-08-12 hardware evidence is correct as recorded; it was taken on one
arm, where a local counter and a unit generation are indistinguishable.

## Why it has to change before the command stamp is enforced

The frozen stamp carries `unit_safety_epoch`. Admission compares it against the
device's own value. While every process keeps a private counter:

- a commander that read one device's unit epoch and stamped a command to
  another device would be comparing unrelated numbers, and admission would
  refuse it for a reason that is not the real one;
- two writers can mint the same generation with opposite meanings. That is
  detectable since 2026-08-12 (`writer_id` plus a contradiction counter) but not
  prevented;
- a unit stop that does not originate from a per-arm service call — a hand
  fault, a supervisor, a physical button wired to some other node — has no path
  to the other devices at all.

## The constraint that shapes the design

**A device must be able to stop itself without another process being alive.**
Safety cannot depend on the coordinator running. This is why "move the writer
into the coordinator" is wrong as stated, and why the fix is a split rather than
a relocation.

## Specification

### 1. Split the two concerns

| concern | who | epoch | needs another process? |
| --- | --- | --- | --- |
| stop *this* device | the device itself, unilaterally | its own `device_epoch` | no |
| which safety era the unit is in | one writer | `unit_safety_epoch` | yes, to advance |

A device stopping itself is a device-level fault on its own epoch. That path
keeps exactly today's latency and independence. The unit generation is a
separate, slower fact used for cross-device command invalidation.

### 2. One writer, in its own node

A dedicated `unit_safety` node — deliberately not inside the coordinator, which
carries MoveIt, the catalogue and the activity DAG, and therefore restarts for
reasons that have nothing to do with safety. The writer must be duller and more
available than the thing it protects.

### 3. Contract

- `AgxUnitSafety.msg` — `header`, `uint64 epoch`, `bool stopped`, `string reason`,
  `string writer_id`. Published latched (transient local, depth 1) on
  `/unit_safety`, on change.
- `/unit_safety/request_stop` — any node may call, with a reason. Returns once
  the generation is allocated.
- `/unit_safety/rearm` — operator surface, refuses while any device still
  reports a latched fault.
- Devices construct `UnitSafety(device_id, writer=False)` and feed the
  subscription into `observe()`. The observer role already refuses to mint.

### 4. Device behaviour on an emergency stop

1. stop the hardware and latch a device fault — immediately, unilaterally,
   unchanged from today;
2. *request* a unit stop from the writer, non-blocking, fire and forget.

With the writer down, step 1 still works and the device is STOPPED; only the
unit generation does not move. That is the correct degradation: safety local,
bookkeeping global.

### 5. Liveness

- **At startup**, no unit-safety state seen within a grace period ⇒ treat the
  unit as stopped. Same reasoning as `require_device_authority`: a missing
  publisher and a wiring error are indistinguishable, and only one of them is a
  configuration anybody chose.
- **Mid-run**, losing the writer degrades what may *begin*, never what is
  already authorised:

  ```text
  writer lost while RUNNING:
      already-authorised execution     may continue
      new ownership era                refused
      rearm                            refused
      new top-level activity           refused
  ```

  The first line is why the writer is not on the critical path: killing it must
  not stop an arm mid-trajectory. The rest is why it still matters — every one
  of them establishes something new about the unit's safety state, and with no
  writer there is nothing that can establish it. Starting a fresh activity is
  the case worth spelling out: it looks harmless because nothing is moving yet,
  and it is exactly when the system would be committing to motion it cannot
  invalidate afterwards.

  Enforced in the coordinator's existing one-activity guard
  (`agx_arm_coordination/unit_activity.py`), which already refuses admission
  with a structured reason — this adds `unit_safety_unknown` beside
  `unit_busy` and `unit_stopping`.

### 6. What this invalidates

`AgxDeviceAuthority.unit_safety_epoch` becomes the observed writer generation
rather than a local counter. The emergency-stop row of
`reference/phase1a_hardware_validation.md` has to be re-run: today the stop
bumps the local unit epoch, afterwards it bumps the device epoch and only
requests the unit generation.

## Sequencing

Lands with the command-stamp slice and before admission is enforced, because the
stamp carries this epoch. Doing it after would mean enforcing a field whose
value is not comparable across devices.

## What this is not

This writer is a **software** safety generation. It orders and invalidates
commands inside our own stack; it is not a protective stop in the functional-
safety sense, and nothing here should be read as one. The independent hardware
emergency stop is recorded as a separate, longer-lived question in
`docs/open_questions.md` — it must not depend on ROS, on this writer, or on any
process in this repository being alive.

## Implemented, and what it looked like on hardware

`agx_arm_ctrl/unit_safety_node.py` is the writer; drivers construct
`UnitSafety(device_id, writer=False)` and adopt generations from `/unit_safety`.

| step | right arm | left arm |
| --- | --- | --- |
| e-stop, **writer not running** | FAULTED, `motion_ready false`, unit epoch 0 | — |
| e-stop, writer running | FAULTED on its own latch | STOPPED, unit epoch 1, `unit stop: arm_right: emergency stop requested` |
| `unit_safety/rearm` | still FAULTED — its own latch | READY, unit epoch 2 |

The first row is the design constraint holding: the device stopped itself with
no writer alive, and the unit generation simply did not advance. The last row is
the split holding from the other side: a unit rearm does not clear a device's
own latch.

One nuance worth stating exactly: the left arm returns to READY rather than
resting in STANDBY, because it never had a device fault — it was stopped only by
the generation, and `_sync_authority` rearms it against live evidence (enable
readback plus advancing feedback), not against an assumption.

## Open decisions

- whether the hand transports request unit stops through the same service or
  keep their own path until the hand contract consolidates (Phase 4D);
- whether `rearm` requires every device to report clear, or only warns — the
  stricter reading is preferred but needs the hand side to report at all.
