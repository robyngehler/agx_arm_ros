# Global Open Questions

Cross-cutting decisions that remain intentionally open after the documentation cleanup closure.

The repo-routing and documentation-ownership questions are closed. The remaining items here are
runtime or contract questions.

## OmniHand command surface

**Closed 2026-08-17.** `control/omnihand/joint_trajectory` is not compatibility-only *and* subscribed;
it is compatibility-only *and off*. The bridge subscribes it and the shared `control/joint_states`
surface solely under `allow_legacy_hand_command_ingress` (default false, development only), because a
bare command carries no commander, no generations and no sequence, so the bridge would have to invent
the identity it then checks.

The explicit contract the question anticipated exists: `DeviceCommandStamp` carried by
`AuthorizedJointTrajectory` (trajectory execution) and `HandJointTarget` (reactive contact-seeking
motion), with `control/omnihand/claim_device` deciding who may command at all. The external interface
is still the standard `FollowJointTrajectory` action; what changed is what crosses the bridge boundary.

What remains open is narrower and belongs to the status half of C5: `HandStatus`, `GripperStatus` and
`OmniHandStatus` are still three messages where one abstract hand status should fit any hand.

## Independent hardware emergency stop

The V02 refactor gives the unit a software safety generation: one writer
allocates `unit_safety_epoch`, devices observe it, and commands stamped under a
superseded generation are refused
(`docs/sprint_refactor/planning/decision_record.md` §3).

**That is command arbitration, not a protective stop.** It orders and
invalidates commands inside our own stack, and every part of it — ROS, the
writer node, the drivers, the CAN transport, the Jetson — is a thing that can
fail. A stop that depends on the stack being alive is not a stop.

### The window is measured, not hypothetical

A bus fault was provoked on 2026-08-13 (`ip link set can_nero_right down`, 15 s,
then up) with the driver instrumented per call
(`docs/sprint_refactor/reference/sdk_latency_budget.md`):

- `disconnect` blocked for **1.0 s in a single SDK call**;
- the recovery took **three attempts**, and the publish loop did not run for
  **13.1 s** — during which nothing drains the CAN RX socket either;
- `get_firmware` took 175 ms while the bus was sick, against 112 ms healthy.

This is a **software-side limit, accepted rather than engineered away.** No
queue discipline shortens a blocking vendor call, and no amount of restructuring
makes a stack stop an arm through a link it is in the middle of tearing down.

What covers the window today, since 2026-08-17: the driver attempts the firmware
`MOVE-J(current_q)` hold *before* the worker is quiesced and the session is
handed to recovery, asserted until the firmware is positively confirmed out of
MIT, and latches the fault afterwards. That is strictly stronger than the damped
MIT zero it replaced — which had no stiffness and sagged as a terminal state —
but it is still a mitigation, not a guarantee: **it requires trustworthy
feedback, and if there is none no hold is claimed at all.** That case is exactly
the watchdog's regime.

That is precisely the gap the independent watchdog exists to hold. The
requirement is therefore not "a stop that is more reliable in general" but a
specific one: **authority over the devices during a window in which the software
stack is provably unable to command them**, of the order of ten seconds, and
recurring whenever the transport faults.

### The software side deliberately stops short of it (2026-08-20)

The Jetson-side ladder ends at the firmware `MOVE-J(current_q)` hold and never
issues the vendor `electronic_emergency_stop()`. That call is a damped descent —
it releases the stiffness that keeps a raised arm up — so it belongs to the
layer that fires when nothing here is holding the arm anyway, not to a path
whose whole purpose is to hold it. An unverified stop re-asserts the hold; where
no trustworthy pose exists, nothing is commanded and nothing is claimed.

This makes the watchdog's job **larger and more explicit, not smaller**: it now
owns every regime in which the hold cannot be established, and it is the layer
free to command a descent. Detail and the reasoning:
`sprint_refactor/reference/emergency_stop_ladder.md`.

Open: a secondary emergency stop that shares nothing with this software path.
The shape sketched so far is a small PLC or single-board controller that

- carries a physical emergency-stop input directly — one would have to be added,
  because this unit has none today,
- sits on the CAN bus as a parallel watchdog rather than through our drivers,
- and takes authority over the devices when it fires, regardless of what the
  software stack believes.

Nothing about it is decided: not the controller, not how it asserts authority
on a bus our drivers also write to, not how the software stack learns that it
fired, and not which standard is actually being targeted. It is recorded here
because the software work makes it *easier to forget* — the refactor produces a
safety-shaped mechanism that is not one, and the gap should stay visible.

Not scheduled in the V02 refactor; it is hardware work with its own
qualification, and the refactor must not be read as having addressed it.
