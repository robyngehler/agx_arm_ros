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

## Why the right arm's motor-state feedback degrades along the joint chain

**Measured 2026-08-22, unattributed.** The right arm's motor-state frames
(`0x251`–`0x257`) arrive at a rate that falls monotonically with the joint
index; the left arm's do not, at a higher rate, under the same bus load.

| motor state | right, MIT on | right, MIT off | left, MIT on |
| --- | --- | --- | --- |
| `0x251` (J1) | 101.0/s | 105.1/s | 136.2/s |
| `0x256` | 83.3/s | 91.5/s | 136.2/s |
| `0x257` (J7) | 63.4/s | 79.2/s | 136.2/s |

Stopping the MIT stream on the right arm alone (the left kept streaming on its
own bus as a within-run control) shows **both causes are real**: removing
1353 frames/s of our own traffic recovers J7 by 25% and narrows the J7-vs-J1
deficit from 37% to 25%, and the remaining 25% is internal to the arm. Joint
*positions* (`0x2A5`–`0x2A9`) stay uniform on both arms; only the motor-state
group degrades.

Right is FW 1.06 and left is FW 1.11, which fits the per-tier rule, but this
measurement does not separate a firmware difference from a difference in that
individual arm. The bus is not saturated (~51% load) and the interface is clean
(304 drops in 18.9M packets, `missed 0`, no bus errors).

Consequence: velocity and effort in `feedback/joint_states` for the right arm's
distal joints are less than half as fresh as the left arm's. Whether that
reaches the control law is also open — in MIT mode the joint damps locally, so
the Jetson-side velocity may never enter the command. It is recorded because it
is the first measured structural left-right asymmetry in this stack, and it
shows on the arm that has been described as lagging during duo playback.

Open: whether it is firmware tier or this unit, whether it changes during motion
(all numbers above are from a holding arm), and whether it affects control or
only monitoring. Detail and method:
`sprint_refactor/reference/feedback_rate_budget.md`.

## Three numbers for one joint velocity limit

**Opened 2026-08-24.** The manufacturer specifies 180 deg/s on J1-J3 and
225 deg/s on J4-J7 — 3.14 and 3.93 rad/s. Two other declarations disagree:

| source | J1-J3 | status |
| --- | --- | --- |
| manufacturer manual | 3.14 rad/s | taken as ground truth |
| `agx_arm_moveit/config/joint_limits.yaml` | 5.0 rad/s | **above spec**, and it governs every MoveIt-planned anchor move |
| MIT controller `velocity_limit` | 2.0 rad/s | below spec, and it silently clamps |

The planner may therefore plan a motion the controller will not execute: above
2.0 rad/s the controller clamps the commanded velocity, tracking error grows,
and `position_error_limit` drops that joint to a hold mid-motion. Nothing has hit
this yet only because the catalogue runs anchor moves at 0.10-0.15 scaling.

It now matters more, because the time-optimal parameterization saturates
whatever limit it is given. `agx_arm_retiming` carries the manufacturer figures
as `NERO_MAX_VELOCITY`; anything it produces above the controller's clamp is
unexecutable.

**No acceleration is specified at all.** `a_max = 2.5 · v_max` is carried as an
explicitly conservative stand-in, and since acceleration is the binding
constraint in every replay measured, that stand-in — not the hardware — is what
currently sets replay speed.

Open: which number governs, whether the controller's clamp is a deliberate
safety margin or an unexamined default, and whether an acceleration figure can
be obtained or has to be measured.

Related: the manual's **joint range** for J2 is quoted in a convention offset by
90 degrees from what the firmware feedback and the URDF use — shifting it lands
within 0.3 deg of the URDF bound, and every other joint's centre already agrees.
Position ranges therefore cannot be applied to feedback values without that
correction; velocity limits are offset-invariant and are unaffected.
Detail: `sprint_refactor/reference/trajectory_retiming.md`.
