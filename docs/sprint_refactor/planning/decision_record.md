# Sprint Refactor — Decision Record

status: CONSOLIDATED
covers: 2026-07-27 … 2026-08-17
canonical plan: [`integration_plan.md`](integration_plan.md)

This file is the **why** of the V02 refactor. `integration_plan.md` is the
**what and in which order**; the checklist is the **how far**. Where this file
and the plan disagree, the plan wins on scope and sequencing and this file wins
on rationale.

It replaces eleven separate proposal, review and spec documents that each
described one slice of the same architecture from the state of the code on the
day it was written. Keeping them apart produced contradictions — three of them
still asserted a hand had no serialized SDK owner after the worker had landed —
so their decisions are consolidated here and the originals are in git history.

## Where the inputs went

| Original document | Written | Disposition |
| --- | --- | --- |
| `coordination_architecture_refactor_proposal.md` | 2026-07-27 | Architectural input. Absorbed into the plan's phases and the constraints below; its shared-bus half is superseded by C1 |
| `proposal_code_crosscheck.md` | 2026-07-27 | Kept in `reference/` — it is evidence, not a decision |
| `refinement_proposal.md` | 2026-08-14 | §1 and §4 landed; §2, §3, §5 became plan 2C |
| `dual_hand_motion_authority_proposal.md` | 2026-08-14 | Implemented. §5 of this record |
| `recovery_watchdog_boundary_proposal.md` | 2026-08-13 | Software half implemented (§2, §4); watchdog half promoted to `docs/project/control_integrity_architecture.md` |
| `refactor_correction_proposal.md` | 2026-08-13 | All ten items closed. Review log below |
| `unit_safety_writer_spec.md` | 2026-08-13 | Implemented and hardware-verified. §3 of this record |
| `docs_cleanup_proposal.md` | 2026-08-16 | Executed 2026-08-16 |
| `fix_refactor_proposal.md` | 2026-08-17 | All P0/P1 items closed. Review log below |
| `refactor_rc_closeout_proposal.md` | 2026-08-17 | Implemented; became the RC gate in the checklist |
| `refactor_rc_finalization_proposal.md` | 2026-08-17 | Implemented; RC closed 2026-08-17 |
| `firmware_hold_recovery_correction_proposal.md` | 2026-08-17 | Implemented. §4 of this record |

---

# Part I — The eight constraints, and what forced each

The constraints are fixed inputs, not tuning levers. Their normative statements
live in `integration_plan.md`; what follows is why each one exists.

**C1 — one CAN bus per device.** Discovered as hardware fact on 2026-08-11, not
chosen: the arms are on native `mttcan` (`can_nero_left`/`can_nero_right`) and
the hands on two USB-CAN FD adapters (`hand_left`/`hand_right`). This deleted
the premise of the entire original proposal, which was built to arbitrate a
shared side bus. Parallel same-side arm and hand motion became the normal mode
and step-and-settle a selectable degraded topology. The wire-level contention it
removed did not remove the coupling: CPU, the GIL, executor threads and the
kernel socket buffers are still shared, and parallel operation loads them more,
not less.

**C2 — the MIT control rate is a requirement.** 100 Hz is the stability minimum
and 200–250 Hz the target, so no CPU saving may be taken out of the rate. The
lever is per-tick cost. This constraint is what made "lower the rate" an
inadmissible answer every time the CPU numbers were bad.

**C3 — the vendor SDK is pinned; development happens elsewhere.**
`vendor/pyAgxArm` at `control-layer-pin-2026-07-24` is the execution path and
does not move inside a phase. It earned its keep twice: the OmniHand receive-loop
fix and the arm-side observability work both landed in tracked forks and arrived
here as explicit pin bumps rather than as a dirty submodule.

**C4 — the test ladder.** L1 unit → L2 mock → L3 hardware. The platform decides
what is *possible*; the ladder decides what is *required*. L2 is never a
substitute for L3 on timing, CAN, or safety claims, and a phase gate that could
not reach L3 says so. Encoded as the `test-ladder` skill.

**C5 — message policy.** Native ROS interfaces where they already carry the
meaning; repo-owned interfaces only for what ROS lacks; statically defined
fields; hand interfaces abstract enough for any hand. The command half is
settled (§5); the status half is the largest remaining consolidation item.

**C6 — instrumentation form.** In-node log counters plus external tooling. No
new public ROS metrics contract during the measurement phase, and no in-band
publication that loads the node being measured.

**C7 — bus topology is one declared fact.** `bus_topology` in the registry
drives both the scheduler's resource claims and whether the arm/hand handoff
runs at all. This closed a dependency gap rather than a bug:  `handoff_enabled`
and `ROBOT_UNITS` described the same wiring loom, and two independent truths
about one physical fact fail invisibly in both directions — a run with the
handoff off and a still-shared bus token serialises motions that need no
serialising; the reverse drops the handoff on a genuinely shared bus.

**C8 — the two arms speak different protocol tiers, permanently.** Right arm
firmware 1.06 (default tier), left 1.11 (`NeroFW.V111`); the arms were bought as
different versions and cannot be flashed. The tiers are not two revisions of one
protocol — 1.11 encodes MIT frames differently and carries its own status enum.
The rule that follows: **anything derived from the protocol is per tier, not per
robot model, and every measurement names the arm it came from.**

---

# Part II — Decisions by subsystem

Each entry states the decision, what forced it, what it replaced, and what it
left open.

## 1. Device authority and epochs

### Four authorities, one per device — not one per side

**Decision.** `LeftArmAuthority`, `RightArmAuthority`,
`LeftHandTransportAuthority`, `RightHandTransportAuthority`.

**Why.** The original proposal's "side hardware authority" is one grain too
coarse once C1 holds. With one epoch per side, a left arm recovery would
invalidate in-flight left *hand* commands — a grasp aborted by an unrelated arm
fault on a bus the hand does not share. That is exactly the coupling C1 removed
at the wire level, reintroduced in software.

**Naming.** The hand authorities are *transport* authorities deliberately: what
they own is one hand's SDK session and CAN transport, not the semantics of a
grasp, which stays with the skill controller.

### Two epochs, because they answer different questions

**Decision.** `device_epoch` (one device; bumps on that device's ownership
transitions, recovery, re-enable) plus `unit_safety_epoch` (the whole unit;
bumps on anything that invalidates every device at once). A command carries
both and is rejected if either is stale.

**Why.** Keeping the unit-level epoch separate is what keeps the case that
*should* invalidate everything — an emergency stop — genuinely global, without
making every device-local hiccup global too.

### One command stamp for every commandable device, frozen 2026-08-12

**Decision.**

```text
string owner_id           # who is commanding
uint64 device_epoch       # the device generation it was issued under
uint64 unit_safety_epoch  # the unit generation it was issued under
uint64 sequence           # per owner, per epoch, strictly increasing
```

Both epochs, always. A device that deliberately does not participate in unit
safety documents that as a named exception rather than dropping the field —
otherwise the same wire name means two different things on two devices.

**Why frozen before any ABI change.** Four documents were using four different
spellings (`control_epoch`; owner + device epoch + sequence without the unit
epoch; "epochs and a sequence"). Implementing the `MoveMITMsg` extension before
resolving that would have forced a second migration immediately afterwards.

**Superseded spellings:** `control_epoch`, and any three-field form.

### Extend `MoveMITMsg`; do not add `ArmMitCommand`

**Decision.** The stamp went into the existing message. **Why:** C5 creates only
what is missing, and a parallel message would have required migrating the hot
streaming path twice. Adding fields is an ABI change needing a coordinated
workspace rebuild, which is acceptable inside one workspace. *Reopens if* an
out-of-workspace consumer of `MoveMITMsg` appears.

### Readiness is not permission

**Decision.** `motion_ready` says the hardware is ready. `may_command(owner)`
answers whether *this* commander may stream, using the same checks as admission
minus the sequence.

**Why.** The field was called `accepts_motion` and meant "state == READY", while
`admit()` additionally required ownership. Once stamping went live a controller
would have been told it may stream and then had every command refused with
`NO_OWNER`. Reading one as the other is how a controller gets told yes and then
cannot move.

**Not taken:** folding ownership into `accepts_motion`. Nothing claimed
ownership at that point, so it would have been permanently false and stopped
both arms.

### Absence of an authority is a refusal, not permission

**Decision.** The device authority is mandatory; the legacy gates survive only
in a named development profile. The launch derives `expected_device_id` from the
same `can_port` as the driver.

**Why.** The staged-migration fallback ("no authority ever received → legacy
gates decide") is fail-open in production: a namespace typo, a QoS mismatch and
an old driver are indistinguishable from the controller, and only one of them is
a configuration anybody chose.

### Per-arm capability, published — not one shared limit table

**Decision.** Each driver publishes `AgxDeviceCapability` (latched) describing
the envelope its protocol tier can encode; the controller fits its configured
limits to *its own* arm before commanding.

**Why (C8).** "Refuse loudly" is not enough for coordinated execution. A refused
MIT command leaves the firmware on its previous setpoint, so a `torque_limit`
above 16 N·m — accepted by the right arm, refused by the left — would leave a
dual-arm activity with one arm moving and one frozen. Verified on hardware:
`[20]*7` becomes `[16]*7` on the left arm and `[20,20,16,16,8,8,8]` on the right.

**Residue.** A synchronized `both_arms` execution is still not preflighted
against both devices as a whole. That is coordinator work and remains open.

## 2. SDK ownership

### One owner of a device's SDK session at any instant

**Decision, and it is a correction.** The rule is **not** "every SDK call on one
worker". It is: steady-state commands and reads go through the device's
serialized worker; destructive recovery takes the session off the worker and is
the owner while it runs.

**Why the stronger rule was wrong.** A measured vendor `disconnect()` blocks for
~1 s in a single call and a full recovery sequence took 13.1 s across retries.
No queue discipline shortens a blocking vendor call, so routing recovery through
the same worker would put a multi-second block in front of the safety lane. The
invariant that survives is single *ownership*, not single thread.

### Four priority lanes, and the unit of work is bounded work

**Decision.** `SAFETY` > `CONTROL` > `ACQUISITION` > `DIAGNOSTIC`, with
`DIAGNOSTIC` the default on purpose so that work nobody classified cannot
overtake the control stream. One SDK call per task, never a retry loop.

**Why the lane count grew from two to four.** The first routing sent a MIT
setpoint as one task and hardware refused it: 6.4 ms mean and 21.4 ms worst case
of non-preemptible work — more than the whole stop budget — while the
acquisition loop lost half its rate. A setpoint is now a *cycle*: one queue
entry for the epoch check and the supersede, executed one frame at a time with
the safety lane drained between frames.

### The safety lane preempts the queue, not the call in flight

**Decision, stated as a limit rather than engineered away.** The bound on a stop
is the duration of whatever SDK call is already executing.

**Measured.** Arms: an emergency stop reaches the SDK in 0.94 ms worst case
(right) and 0.55 ms (left) against a declared 20 ms budget, with 100 Hz MIT
running. Hand: safety-lane queue wait under 1.9 ms across 150 stops at a
saturated read rate — but the longest single vendor call on the O12 Pro is
36.9 ms (`read_tactile`), above the arms' budget.

**Residue.** The hand has no declared stop budget. Two things have to be decided
rather than assumed: whether a hand stop needs an arm-grade bound at all (it is
a cancel-and-hold, not a unit emergency stop), and whether the 37 ms tactile
read belongs on the same worker as the stop.

### Recovery off the acquisition path

**Decision.** Recovery runs on its own thread and takes the session; the
acquisition and publish loops keep running.

**Why.** Inlined, recovery cost a **13.1 s publish-loop gap**. Re-provoked with
recovery on its own thread there are no overruns at all and 2384 authority
publications across the fault. The recovery still takes 13.1 s — that is the
vendor's `disconnect`, three times — but it no longer costs the loop.

### Worker lifecycle is explicit

**Decision.** Every node owning an `SdkWorker` stops its producers, then the
worker, and joins them in `destroy_node()`. Idempotent.

**Why.** The threads were daemons, so a process exit disposed of them and hid
the omission. Anything that destroys a node without exiting — a test, a repeated
bringup, a composed process — kept a thread holding that device's SDK session,
which is precisely the invariant the worker exists to provide. It surfaced as a
suite where every test passed alone and failed together.

**Residue.** The quarantined legacy motion paths are routed through the worker
when enabled, but they are the one remaining category where a development
profile puts a second writer on the session if misused.

## 3. Unit safety

### One writer, devices as observers

**Decision.** `agx_arm_ctrl/unit_safety_node.py` is the sole allocator of
`unit_safety_epoch`; devices observe `AgxUnitSafety` (latched on `/unit_safety`)
and ask for a generation through `RequestUnitStop` without ever waiting for one.
**A device still stops itself unilaterally on its own epoch.**

**Why.** Every node used to construct its own `UnitSafety` *writer*, and
`observe()` ignored equal epochs — so two processes could publish "5, stopped"
and "5, rearmed" with no ordering between them, and a receiver could not tell a
contradiction from a duplicate.

**Verified on hardware:** an e-stop on the right arm stops the left arm through
the generation, and with the writer not running the right arm still stops
itself.

### Writer restart is ordered per incarnation, not per epoch

**Decision.** Each run of the writer carries an `incarnation` and its start
time. Epochs are compared only within one incarnation; a new incarnation is
adopted outright and **fails closed**, holding the unit stopped until an
explicit rearm; a message from a writer that has since died is dropped rather
than obeyed. Rearm always allocates. A first observation is exempt so a cold
boot still needs no operator.

**Why.** The generation counter lives in memory and starts at zero. After a
restart, every observer that had reached a higher generation silently dropped
what the new instance published until it climbed back past that number — and
during that window the unit could not be told a new safety era had begun. The
transient-local latch and the heartbeat both republish the *restarted* value, so
neither closed the gap.

**Why fail-closed rather than persisted continuity.** A restarted writer
believing nothing is wrong is exactly when observers hold a stop it has no
record of.

### Unit safety is command arbitration, not a protective stop

**Stated so it is not mistaken for one.** It orders and invalidates commands
inside our own stack. Every part of it — ROS, the writer node, the drivers, the
CAN transport, the Jetson — can fail. A stop that depends on the stack being
alive is not a stop. The independent watchdog (§4) is the boundary for that.

## 4. Safety hold, e-stop, recovery, and the watchdog boundary

### The canonical hold is `MOVE-J(current_q)`, asserted until confirmed

**Decision.** One firmware-hold primitive, reused by the hand window, the
emergency stop and pre-recovery teardown:

```text
capture fresh q
-> optional short damped-MIT braking transition
-> repeated MOVE-J(q) on the SAFETY lane
-> confirmation on a known non-MIT move mode
-> settle verification
```

**Why it was a reuse problem, not a missing primitive.** The correct mechanism
already existed in the hand-window code. The e-stop asserted MOVE-J *once*, so a
single dropped mode frame left the firmware in MIT executing a zero-stiffness
command while the software reported a verified stop. Recovery did not establish
the hold at all: it sent a kp=0 damped MIT zero and tore the link down.

**A host-side MIT command is never a hold.** It is a braking transient before
MOVE-J. kp=0 stops a moving arm but has no stiffness, so as a terminal state it
sags — which is what a stopped left arm did on 2026-08-17.

**`disable()` is not the stop primitive.** A disabled Nero has no brakes. The
desired stopped state is: motors enabled, firmware position controller active,
current pose held, motion authority revoked — not a limp arm.

**Confirmation requires a positive reading.** The verifier asked "is this not
MIT?", which an unreadable status read answers with *yes* — so silence confirmed
a hold at exactly the moment a hold most needs checking. Only a known non-MIT
move mode counts.

**Ordering.** The hold is attempted **before** the worker is quiesced and the
session is transferred to recovery, because `_assert_firmware_hold()` uses the
worker and a quiesced worker executes nothing.

**No trustworthy pose, no claimed hold.** A pose synthesised from stale data is
a wrong hold rather than a missing one. The transport recovery continues,
because the transport still needs repair, and the watchdog owns that regime.

### The e-stop advances the worker epoch, not only the authority epoch

**Decision.** `SdkWorker.set_epoch(...)` immediately after
`DeviceAuthority.enter_faulted(...)`.

**Why.** The safety lane overtakes queued work but priority alone does not
*invalidate* it: an old MIT cycle queued before the stop could still execute
after the safety MOVE-J had run.

### Recovery is stop-aware

**Decision.** Auto-enable during recovery is suppressed while an e-stop is
latched or the unit is stopped. Recovery may restore enough transport and
feedback to diagnose the arm; it must not return the hardware to ordinary
enabled operation on its own after a stop.

### The quarantine is about ROS ingress, not the primitive

**Decision, stated because it was nearly got wrong twice.**

```text
PUBLIC  /control/move_j     = unauthenticated ingress, quarantined, off by default
INTERNAL move_j(current_q)  = firmware-hold primitive, required by safety logic
```

Do not remove the internal call while cleaning up legacy interfaces.

### The watchdog boundary

**Decision.** Bounded protective action *while the normal transport is blocked
or being destructively recovered* cannot be guaranteed through the same SDK/CAN
path and is an independent-watchdog concern. The software stack remains
responsible for everything it can deterministically control: fault detection,
stopping publication, invalidating stale generations, isolating recovery,
observable fault state, no automatic resume, verified recovery.

The watchdog design — takeover state machine, TX inhibit, release preconditions,
trigger model, failure coverage — is promoted out of this sprint to
[`docs/project/control_integrity_architecture.md`](../../project/control_integrity_architecture.md),
because it outlives the refactor. Two rules from it are load-bearing here:

- **a returning heartbeat is not a release.** A host reboot, SDK reconnect or
  restored feedback is evidence that recovery may begin, not that motion is safe
  to resume;
- **the Jetson and the watchdog must never both be transmitters.** Takeover
  asserts a hardware TX inhibit before the watchdog sends anything.

## 5. The hand command contract

### Two production motion primitives, one exclusive owner

**Decision.** A hand has exactly two production primitives and they may never
command it at once:

- `<side>_omnihand_controller/follow_joint_trajectory` — the primary
  **trajectory-execution** path;
- reactive **contact-seeking** motion (the skill controller) — a grasp that ends
  where the tactile sensor says rather than where the clock does.

**Why not decompose the reactive loop into FJT goals.** It cannot be expressed
as a time-parameterized trajectory without losing the closed loop that defines
it. That is the one thing it cannot do.

**Why FJT is not a debug surface.** Lower measured latency than a direct
`HandCmd`, it synchronizes with arm trajectories, and it carries the trajectory
semantics later motion primitives need. **Superseded 2026-08-14:** the earlier
"FJT is debug/development-only, keep it non-default in production" reading
contradicted the dual-primitive design and is gone.

### Exclusivity comes from device authority, not topic separation

**Decision.** Both primitives claim `control/omnihand/claim_device` before
commanding and release afterwards; claim and release advance the device epoch;
the bridge is fail-closed, so an unclaimed hand executes nothing. A grasp that
ends holding keeps the claim — the hold *is* the reactive primitive still owning
the hand.

**Why the earlier answer was insufficient.** "Reject a second active goal on one
action server" cannot work when the two primitives are two servers. The
condition the original decision named as its reopening trigger — a second
legitimate commander coexisting with the skill controller — is exactly what
happened.

**Service naming.** The hand's claim service is `control/omnihand/claim_device`,
never plain `claim_device`: the arm driver owns that name in the same namespace,
so both resolved to `/<side>_arm/claim_device` and a client silently reached
whichever it discovered first. An owner declares itself `<primitive>:<node>` —
the primitive half is how the bridge tells a trajectory command from a reactive
one, the node half is how it notices a commander that died holding a claim.

### One reusable authority contract, two motion payloads

**Decision, landed 2026-08-17.**

```text
agx_arm_msgs/DeviceCommandStamp     owner_id, device_epoch, unit_safety_epoch, sequence
agx_arm_msgs/AuthorizedJointTrajectory   stamp + trajectory_msgs/JointTrajectory
agx_arm_msgs/HandJointTarget             stamp + joint_names + positions
```

**Why two payloads rather than one abstract hand command.** *Superseding* the
earlier "one abstract hand command message, not a family" reading: that rule was
aimed at OmniHand-*specific* messages, and two payloads differing by **motion
shape** rather than by **device** satisfy what it was protecting. Forcing them
together would have made the reactive primitive express contact-seeking motion
as a trajectory.

**Why the trajectory is carried whole** even though the current backend executes
only the final position target: the ROS contract preserves information the
backend does not yet use, so a future interpolating or velocity backend can
replace the backend without touching the authority contract.

**Standard ROS messages are not modified.** `sensor_msgs/JointState` stays a
feedback message, standard `FollowJointTrajectory` stays the MoveIt-facing
action. What changed is what crosses the bridge boundary.

### The bridge admits on the stamp the command arrived with

**Decision.** The bridge never substitutes a missing authority field from its
own state.

**Why this was the whole point.** Before the stamp, the bridge built a command's
identity from its own current epoch and its own counter, then checked that
identity against the state it came from — comparing each value with itself. The
stale-epoch and out-of-order checks ran and could refuse nothing.

### No production dual-publish

**Decision.** Production publishers emit only the stamped message. Bare
`JointTrajectory` / `JointState` ingress exists solely under
`allow_legacy_hand_command_ingress` (default false, development only).

**Why.** Publishing both meant two commands for one logical motion, admitted
against one sequence watermark — so the self-stamped legacy copy could starve
the stamped one, and a delayed bare command could be assigned today's authority
although it was produced under yesterday's.

**Residue.** The status half of C5 is open: `HandStatus`, `GripperStatus` and
`OmniHandStatus` are still three messages where one abstract hand status should
fit any hand, carrying joint count, joint naming and tactile layout as **data**
so `o10`, `o12_pro` and the 1-DoF AGX gripper all fit. Also open:
`control/omnihand/stop` is a cancel-and-hold, not a latching device stop, so a
hand can only be latched STOPPED through the unit generation where an arm can
latch itself. Closing that asymmetry belongs with the consolidated contract.

## 6. Hand runtime and transport

### 2C owns how the hand is driven; 4D owns what a caller writes

**Decision, stated once so it stops being renegotiated per slice.** 2C owns
cadence, publication, polling, SDK ownership and serialization, command
admission at the bridge boundary, and arbitration. 4D owns the public hand
contract and the migration off the retired messages. Phase 3 owns coordinator
integration. Phase 5 keeps only the before/after close-out. The former phase 5C
(bridge timer split) was folded into 2C so the bridge is not optimised twice or
not at all.

### The hand's cadence is its own

**Decision.** Bringups pass `hand_pub_rate` and `hand_joint_read_rate`;
publication is driven by new data; `pub_rate` is a ceiling that can throttle and
never drive.

**Why.** Every bringup forwarded the *arm's* 200 Hz publish rate into the
bridge, while the hand's joints change at 20 Hz and its status and tactile once
a second — nine of every ten wakes carried nothing new. 41.5 % → 7.3 % of a
core, and **flat across `pub_rate`**, which is the result worth keeping.

The stack saved four times what the mock measurement predicted, because
publishing ten times more often than the data changed was charged to everyone
reading it too: the trajectory node fell 88.1 % → 10.4 % without a line of it
being touched.

### Tactile has two cadences, chosen by who holds the hand

**Decision.** With no owner or a trajectory owner, tactile is a 1 Hz diagnostic
on the `DIAGNOSTIC` lane. While the reactive primitive holds the device it runs
at the acquisition rate on the `ACQUISITION` lane.

**Why.** Tactile was classified once, as a diagnostic. It *is* a diagnostic to
everyone except the reactive primitive, whose whole definition is that it ends
where the sensor says rather than where the clock does. For that owner it is
control-critical acquisition. The cost of contact-seeking is +12 % of a core and
is paid only while something is waiting on the sensor.

### 91 % of a hand bridge's cost was a vendor busy-wait

**Decision.** Patched in the tracked vendor fork under C3, not worked around.

**Why the plan's premise had to be corrected.** The transport-efficiency items —
dropping the read-before-write, polling only under ownership, bounding round
trips per setpoint — divide up **5 %** of a core, not 100 %. They stay worth
doing for the CAN bus and for latency; they were never the CPU answer.

`CanBusDeviceSocketCan::RecvFrame` called `read()` on a non-blocking socket in a
loop with nothing in between. The comment on that line already named the reason
— a blocking read would keep the thread from being released at shutdown — but
the answer to that is `poll()` with a timeout, not a spin. Both bridges fell
from 222.4 % to 25.3 % of a core; the whole stack from 814.5 % to 399.7 % idle
and 882.9 % to 431.3 % under dual MIT. Latency unchanged.

**Residue in 2C:** the read-before-write round trip in the full-joint command
path, splitting command verification / joint readback / tactile / status into
separate schedules, stopping polling while no hand action is active, bounding
and recording SDK round trips per commanded setpoint, and verifying that the
`shared_per_side` topology still executes an activity.

## 7. Bus topology and parallel operation

### The topology is declared once and everything derives from it

**Decision.** `bus_topology: dedicated_per_device | shared_per_side` in the
registry. `graph_model.robot_units(topology)` returns the scheduler's table;
`handoff_enabled` and `handshake_enabled` default from `handshake_required()`.
An **unknown topology reads as shared** — a value nobody recognised is not a
licence to run a hand beside its arm.

**C7 exit, closed 2026-08-16.** Both parameters remain declared as compatibility
inputs, and a value disagreeing with `bus_topology` **fails startup naming both
sources**. So no entry point — including a bare `ros2 run` — can assert a
topology the registry contradicts. The L2 harness was itself forcing
`handoff_enabled:=true` against a dedicated registry.

**Why the defaults mattered as much as the override.** The hardcoded `True`
meant anything starting those nodes outside the launch files — a test double, a
bare `ros2 run`, a measurement harness — quiesced an arm for a hand that has its
own bus, silently.

### Validation and scheduling read the same resource table

**Decision.** `validate_activity(graph, catalogue, units)` takes the table as a
**required** argument; the catalogue is constructed with the topology's table.

**Why.** Validation used to default to the shared-bus table while the scheduler
used the configured one, so the two disagreed about the same machine: under
`dedicated_per_device` a synchronized `left_arm + left_hand` pair was rejected at
validation while the scheduler would have run it in parallel quite happily.

### The hand's bus is its own registry entry, fail-closed

**Decision.** `omnihand.sides.<side>.can_port`, schema version 3. A hardware
backend refuses to start without its own declared interface rather than opening
the arm's, where no hand ever answers. The interface is an argument to the
backend, held in the environment only for the SDK construction that reads it,
under a process-wide lock.

**Why.** The bridge derived its interface from the *arm's* `can_port` and fell
back to `can_nero_right`. And the process-global `OMNIHAND_SOCKETCAN_IFACE`
worked only because the two bridges run in separate processes; composed into
one, whichever constructed second chose the bus for both.

**Naming, twice-corrected.** The interfaces are `hand_left`/`hand_right`, not
`left_hand`/`right_hand`. `left_hand` *is* the scheduler resource name in
`graph_model.py`, so the two spellings coexist with different meanings and must
never be derived from one another by string surgery.

### `activate_duo_can.sh` is the one supported bring-up

**Decision.** `activate_native_can.sh` and `omnihand_canfd_activate.sh` are
forwarding shims rather than deletions, so a stale runbook still brings a bus
up. Retiring them required porting the TJA1051T/3 TDC offset into the duo script
— the arm buses run CAN FD at 5 Mbit and the transceiver needs it. The script
also sets `txqueuelen` 1000, which roughly halves the per-frame `move_mit` cost.

## 8. Coordinator

### The exclusivity guard came before the parallelism

**Decision.** `READY` accepts one activity; `EXECUTING` rejects every further
goal with a structured reason. The goal callback refuses at the door; the claim
inside execute is authoritative, because two goals can pass the door check at
once on a reentrant callback group.

**Why pulled forward from Phase 3.** Parallel operation multiplies the ways two
activities can interleave, so "only one activity at a time" has to hold *before*
the parallelism exists, not after.

### Sync groups are atomic, and merge is merge-or-fail

**Decision.** A sync group is one candidate during batch construction: all
members ready and all required resources jointly available, or none of the
group. A batch that cannot be merged raises `DispatchError` and aborts the
activity.

**Why.** Greedy per-action admission permitted half a barrier — an independent
action taking `left_arm` first, then one sync member rejected while its partner
was admitted. And an independent-dispatch fallback meant a barrier the plan
declared could silently not happen, which is worse than failing: the two arms
would move unsynchronised and nothing would say so.

### Event-driven completion, with the watchdog as a missed-wakeup guard only

**Decision.** A child goal resolving, a cancel, and a stop all set one event the
activity loop waits on. `watchdog_period_sec` (0.5 s) is only a missed-wakeup
guard, not the progress mechanism.

### Cleanup is part of completion, and authority is released on every path

**Decision.** Cancellation waits, bounded by `cleanup_timeout_sec`, for each
child to confirm it stopped and names the ones that did not. Hand windows still
open are closed in a `finally` alongside the unit-activity release.

**Why.** It used to fire the cancels and clear its bookkeeping in the same
breath, so an activity could report "aborted" while an arm was still executing.
And closing the window is what hands the arm back — a window left open keeps the
arm's MIT gate shut, so the *next* activity would find an arm that silently
refuses to move.

**Residue in Phase 3:** SIGINT with no activity in flight still spins rather
than exiting; the full unit-activity state machine and the Ctrl+C stop-ladder
migration are open.

## 9. Measurement and CPU

### Decompose before optimising — the assumption was wrong three times

**Decision.** No CPU work starts before the decomposition names its target.

**The record that earns the rule.** The sprint assumed the arm driver's
per-joint SDK reads dominated; measured, they were ~0.11 ms of a 1.10 ms batch.
It then assumed the publish batch; measured, the largest single consumer inside
the driver was the rclpy executor thread. It assumed the hand bridge's polling;
measured, 91 % of it was a vendor thread we never call. "Gravity compensation
dominates the MIT tick" is the next such hypothesis and is explicitly labelled
one.

**Consequence for Phase 5B:** the MIT tick is decomposed by segment —
trajectory sampling, feedback snapshot, gravity/RNEA, command construction, ROS
publish, action feedback/tolerance, locking/executor — before anything is cut.

### An attack was dropped on the evidence

The rclpy executor thread inside the arm driver is 23.4 % at rest. Attacking it
would have optimised a fifth of a core inside a system spending eight, would
have invalidated the current tests, and risked multithread problems. Postponed
deliberately, not forgotten.

---

# Part III — Decisions that were reversed

Recorded so they are not resurrected.

| Superseded reading | Replaced by | When |
| --- | --- | --- |
| Shared side bus; step-and-settle as the normal operating model | C1, per-device buses; step-and-settle is a selectable degraded topology | 2026-08-11 |
| Hand lease acquire/release as a bus-arbitration contract; bus-quiet verification; "zero hand TX during arm ownership" | Struck with the old Phase 2. What survives is single-commander arbitration on CPU and correctness grounds | 2026-08-11 |
| "No separate hand ownership contract, no new hand command interface; a topic command cannot carry epochs and a sequence" | `control/omnihand/claim_device` + `DeviceCommandStamp` in the message | 2026-08-17 |
| "One abstract hand command message, not a family" | One reusable authority stamp, two motion payloads | 2026-08-17 |
| "Keep the MoveIt hand FJT path non-default in production profiles" / "FJT is debug-only" | FJT is the primary trajectory-execution primitive; both primitives are production | 2026-08-14 |
| "No arm SDK call outside the worker" | Exactly one SDK owner at any instant; recovery owns the session while `RECOVERING` | 2026-08-13 |
| `accepts_motion` as controller permission | `motion_ready` (hardware) + `may_command(owner)` (permission) | 2026-08-13 |
| `control_epoch` and three-field command stamps | The frozen four-field stamp | 2026-08-12 |
| "A hand still has no serialized SDK owner" | Every hand bridge owns an `SdkWorker` with the same four lanes | 2026-08-15 |
| "Recovery is safe because it sends a damped MIT zero before teardown" | Recovery attempts the firmware MOVE-J hold before destructive ownership transfer; damped MIT is a transient only | 2026-08-17 |
| `tea_pour_left_v1` runs after every phase | L1+L2 are the standing per-phase gates; the demo returns as a gate once it runs again | 2026-08-14 |
| The demo needs re-teaching against the new command contracts before it can run | It ran unchanged on 2026-08-17. The contract changed what crosses the bridge boundary, not what a taught pose means | 2026-08-17 |
| Per-side authority and one epoch per side | Four device authorities, two epoch levels | 2026-08-11 |

---

# Part IV — Deliberately deferred

Not forgotten, and not blockers. Each has a named home.

- **Hand status consolidation** (C5 status half) — Phase 4D.
- **The hand's stop budget** — `open_questions.md`; needs a decision on whether
  a cancel-and-hold needs an arm-grade bound at all.
- **Independent hardware emergency stop** — `docs/open_questions.md` and
  `docs/project/duo_system_architecture_authority_worker_watchdog.md`. Hardware
  work with its own qualification; the refactor must not be read as having
  addressed it.
- **OmniHand advanced control backends** — velocity streaming, position
  streaming, mixed control, joint→actuator differential kinematics. The vendor
  SDK exposes the modes; the units, the Jacobian and the retargeting behaviour
  are not characterised. Recorded in
  [`docs/assets/omnihand/omnihand_pro_analysis.md`](../../assets/omnihand/omnihand_pro_analysis.md);
  belongs to a later control sprint, not to the refactor MVP.
- **The `current *= -1` vendor mutation** on the default tier — unaudited, and
  nothing in this repo consumes the value yet.
- **Promoting the joint-limit check from warned to refused** — needs the defined
  stop/hold transition for a rejection on an active stream first, plus a
  hardware session establishing that the controller never legitimately crosses a
  limit.
- **Synchronized `both_arms` preflight against both firmware tiers** —
  coordinator work.
- **Removal of the `shared_per_side` degraded mode** — reviewed at the Phase 5
  close-out against the measured four-bus evidence; no new work invested
  meanwhile.
- **`rmem_default` / `SO_RCVBUF`** — `activate_duo_can.sh` raises
  `net.core.rmem_max`, but that is only a ceiling an application must request,
  and `python-can` never calls `setsockopt(SO_RCVBUF)`. The intended effect of
  commit `e69daa2` did not take.
- **The RX-drain fault mode that can actually overflow** — the bus stays *up*
  while the reader is stopped or starved. The link-down case is measured and
  cannot overflow: a down link delivers nothing.

---

# Part V — External review log

Four external reviews were folded in. Each is recorded by what it found and what
happened, because the pattern is more useful than the individual findings: **the
reviews were most valuable where they contradicted a claim the sprint had
already marked done.**

### Review 1 — Phase 1A correction (2026-08-13, `c234469..14c6eff`)

Ten items, all closed. Five were real defects and **three were introduced by
this sprint's own changes**:

- a NaN became the maximum commanded torque, because `max(-limit, min(limit, v))`
  maps NaN onto `limit` — and the hardware-boundary non-finite check added
  earlier in the same sprint could not catch it, since by then the value was
  finite;
- authority loss stopped the MIT stream but not the active FJT goal, so the
  claim that an epoch change "aborts in-flight work" was half true;
- `accepts_motion` promised what `admit()` would refuse;
- two processes could mint the same unit-safety generation;
- absence of an authority was read as permission.

### Review 2 — Recovery and watchdog boundary (2026-08-13)

Corrected the SDK ownership invariant from "all calls on one worker" to "exactly
one owner at any instant", on the strength of the measured 1 s `disconnect()`,
and refused to accept the 13.1 s publish-loop outage as inherent. Both landed.
Its watchdog half is now a stable architecture document.

### Review 3 — Fix refactor (2026-08-17)

Eight items; all P0/P1 closed. The two that were not already known: activity
validation and the scheduler reading different resource tables, and sync groups
being admitted greedily rather than atomically. Also produced the
`DeviceCommandStamp` / `AuthorizedJointTrajectory` / `HandJointTarget` shape
that shipped, and the explicit non-goal list that kept the hand backend on the
known-good position path.

### Review 4 — RC closeout and firmware hold (2026-08-17)

Found the arm bootstrap deadlock — a silent arm refused the enable that would
have restored its feedback, because `_check_arm_connected()` used `is_ok()`,
which is feedback health rather than transport presence. Split
`transport_connected` / `feedback_alive` / `enable_verified` / `control_ready` /
`motion_permitted`, split request-enable from verify-enable, and reordered
startup around bootstrap rather than readiness. In the same pass: the firmware
hold reuse above, the stamped-only production path, and the instruction to keep
the quarantined-MOVE-J L3 evidence **separate** from the production MIT/FJT
evidence because they validate different contracts.

### Review 5 — Cleanup and evidence closure (2026-08-18)

Post-RC, and the first review with no defect to report. It asked for the written
state to catch up with a runtime that had moved past it: record the successful
tea-pour run as evidence, remove the last stale pre-refactor statements, make the
operator runbooks four-bus native throughout rather than only in their banners,
and keep the coordinator-crash gap open but scoped away from demo acceptance. Two
contradictions it caught were real: `open_questions.md` still said a topic command
carries no epoch or sequence after the stamp had shipped, and the 2B section still
said parallelism had never been demonstrated while Point-8 further down the same
file recorded it.

**Its account of the hardware session was wrong in two places, and the logs
settled both.** It reported a `Ctrl+C` test that "terminated the activity
correctly"; the coordinator's own log says `no activity running`, 55 minutes after
the last run — so the interrupt proved the idle-exit path, not the stop ladder. It
also reported "no operational anomaly"; the logs carry five hand
delivery-verification give-ups, an acquisition-loop overrun, a one-cycle ownership
race and a teardown publish failure, none of which failed the demo but all of which
are worth having.

The rule that follows, and it is the reason the evidence file exists at all:
**operator recollection and log evidence are two different sources, and a written
record that merges them silently loses the disagreement.** Where they conflicted
here, the logs won, and the parts only the operator can attest to — that the arm
did not visibly sag — are labelled as such.

### What the reviews are worth, stated as a rule

A review that only confirms is cheap. These were valuable because they read the
*code* against the *claim* — the checklist said the hand had a serialized owner
and the canonical plan said it did not; the checklist said `handoff_enabled` was
derived and the parameter was still independently settable. **A completion claim
in one file and the implementation in another drift apart silently.** That is
the reason this record exists as one file.
