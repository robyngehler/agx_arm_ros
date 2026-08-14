# Proposal: Dual Hand Motion Primitives with Exclusive Device Authority

**Status:** Proposed  
**Scope:** OmniHand production command architecture

## Goal

Support both trajectory-based and reactive tactile hand motion without reintroducing the current two-commander race.

The hand shall expose **two legitimate production motion primitives**, but only **one active device owner** at a time.

## 1. Keep two production primitives

### Trajectory execution

`FollowJointTrajectory` remains the primary path for:

- coordinated arm/hand trajectories;
- synchronized motion;
- planned gestures;
- trajectory-based motion primitives and future Physical AI execution.

### Reactive contact-seeking motion

The skill controller remains valid for:

- tactile grasping;
- incremental contact-seeking motion;
- feedback-conditioned stopping;
- grasp-to-hold behavior.

Do not decompose the reactive grasp loop into many small FJT goals.

## 2. Enforce one exclusive hand authority

Both paths must use the same bridge-level authority contract:

```text
owner_id
device_epoch
unit_safety_epoch
sequence
```

Required behavior:

```text
FJT owns hand
    -> reactive commands rejected

reactive grasp owns hand
    -> FJT commands/goals rejected or aborted
```

`claim_device` / release must become active parts of the production path rather than unused infrastructure.

## 3. Make ownership handover an epoch boundary

A transition between command primitives must:

1. stop/quiesce the previous command path;
2. remove its pending commands;
3. change owner;
4. increment `device_epoch`;
5. start a fresh command sequence.

A delayed command from the previous owner must therefore never become executable after the handover.

`unit_safety_epoch` does not change for a normal ownership transfer.

## 4. Keep grasp-to-hold inside the reactive owner

After tactile contact:

```text
reactive grasp
    -> contact detected
    -> stable hold
```

The reactive skill may retain ownership while holding.

Do not force an unnecessary handover to FJT simply to maintain the reached grasp pose.

Transfer to FJT only when a later coordinated trajectory actually requires trajectory ownership.

## 5. Bridge admission

The OmniHand bridge must become the final enforcement boundary:

- verify current owner;
- verify `device_epoch`;
- verify `unit_safety_epoch`;
- reject stale/non-monotonic sequence numbers;
- reject commands from inactive motion primitives.

Topic separation alone is not sufficient protection against concurrent commanders.

## 6. Validation

Add an L3 hand-authority test covering:

```text
reactive grasp
    -> tactile contact
    -> hold
    -> explicit handover
    -> FJT trajectory
```

Verify:

- tactile stopping remains responsive;
- hold begins at the actually reached position;
- no stale FJT or skill command executes after ownership change;
- no command overlap occurs;
- no visible jerk occurs at grasp-to-hold or owner handover;
- authority and epochs remain consistent throughout.

## Documentation wording

Replace:

> FJT is the primary production hand command path.

with:

> **FJT is the primary production trajectory-execution path. Reactive contact-seeking motion is a second legitimate production primitive. Both share one exclusive device-authority contract and may never command a hand concurrently.**

## Exit criterion

This slice is complete when both FJT and reactive grasp remain fully functional on hardware while exclusive ownership at the bridge makes concurrent hand command execution impossible.
