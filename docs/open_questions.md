# Global Open Questions

Cross-cutting decisions that remain intentionally open after the documentation cleanup closure.

The repo-routing and documentation-ownership questions are closed. The remaining items here are
runtime or contract questions.

## OmniHand command surface

Should `control/omnihand/joint_trajectory` remain compatibility-only, or should the repo later
promote a more explicit action or controller contract once the stable hardware-backed hand workflow
is fully signed off?
## Independent hardware emergency stop

The V02 refactor gives the unit a software safety generation: one writer
allocates `unit_safety_epoch`, devices observe it, and commands stamped under a
superseded generation are refused
(`docs/sprint_refactor/planning/unit_safety_writer_spec.md`).

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
What covers the window today is the damped zero the driver sends *before* the
teardown and the fault lockout it latches after — a mitigation, not a guarantee.

That is precisely the gap the independent watchdog exists to hold. The
requirement is therefore not "a stop that is more reliable in general" but a
specific one: **authority over the devices during a window in which the software
stack is provably unable to command them**, of the order of ten seconds, and
recurring whenever the transport faults.

Open: a secondary emergency stop that shares nothing with this software path.
The shape sketched so far is a small PLC or single-board controller that

- carries the physical e-stop input directly,
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
