# Global Errors And Fixes

Cross-cutting issues that already produced confusion or wasted debugging time.

## A kernel update discards the Jetson 40-pin header config, killing CAN TX

**Check this before diagnosing any silent arm bus.**

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

The native arm interfaces (`can_nero_left`, `can_nero_right`) ride the 40-pin
header. A kernel update resets its pinmux, after which `mttcan` still presents
both interfaces as UP and ERROR-ACTIVE while nothing can be transmitted.

Signature: `RX=0 TX=0` in `ip -s link show`, sends failing with `ENOBUFS`
("Transmit buffer full"), and `berr-counter tx 0`. The error counters are not a
discriminator — the interfaces run in ONE-SHOT mode, which aborts an
unacknowledged frame rather than retrying it into error-passive. Read TX
*packets*.

The hands are on USB-CAN FD adapters and do not use the header, so a healthy
hand bus in the same session does not rule this out.

**The error counter separates this from an unpowered arm** (2026-08-20). Both
faults show `TX packets 0`, so that number says "nothing got out" and nothing
more. What differs is whether the controller ever drove the line:

| | header not muxed | arm not powered |
| --- | --- | --- |
| state | `ERROR-ACTIVE` | `ERROR-PASSIVE` |
| `berr-counter tx` | `0` | climbs to `128` |
| `error-warn` / `error-pass` | `0` / `0` | `1` / `1` |
| RX packets | 0 | a couple of error frames |
| driver symptom | sends fail `ENOBUFS` | sends leave, nothing ACKs |

With the pins unrouted the frame never reaches transmission, so the counter
cannot move. With the arm off, the controller transmits into a bus where nobody
acknowledges, and the counter walks to error-passive. **There is no mechanical
e-stop on this unit**, so an arm that answers nothing and is wired correctly is
an arm without power — that is the one thing to check.

Happened 2026-08-11 (header never configured) and again 2026-08-17 (kernel
update). The second occurrence cost most of a session and produced two wrong
diagnoses. Detail and the driver-side diagnostic:
`docs/sprint_refactor/errors_and_fixes.md`.

## Native CAN naming versus legacy public naming

Problem: docs and examples mixed native Jetson side-bus names with USB role names.

Current fix:

- `scripts/activate_duo_can.sh` is the one supported bring-up for all four
  interfaces. `activate_native_can.sh` and `omnihand_canfd_activate.sh` are
  forwarding shims, retired 2026-08-17 but kept so a stale runbook still works
- the four stable interface names are `can_nero_right`, `can_nero_left` (arms,
  native `mttcan`) and `hand_right`, `hand_left` (hands, USB-CAN FD adapters)
- old public runtime names such as `can0` and `can_nero` are deprecated, and the
  hands must never be addressed by enumeration order — two identical USB
  adapters swap it
- `hand_left` is an *interface* name; `left_hand` is the *scheduler resource*
  name in `graph_model.py`. The two spellings coexist with different meanings
  and must not be derived from one another

See `control/bringups/launches.md` and `CAN_USER_EN.md`.

## pyAgxArm source drift

Problem: the docs did not clearly separate the pinned repo runtime source from the external
pyAgxArm development checkout.

Current fix:

- `scripts/setup_agx_arm_runtime_env.sh` now installs `vendor/pyAgxArm` first
- it falls back to `../pyAgxArm` only when the vendored checkout is unavailable
- the sibling or external `pyAgxArm` checkout remains the place for upstream pulls, local SDK
	changes, tagging, and preparing the next `vendor/pyAgxArm` pin

See `control/environment.md` and `project/control_layer_and_dependencies.md`.

## Build Python versus runtime Python

Problem: mixing Conda and ROS build shells hides ROS dependencies and creates false failures.

Current fix:

- use `scripts/colcon_build_system_python.sh` for workspace builds
- keep `colcon test` on a system-Python ROS shell
- use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands
- append to `PYTHONPATH`; do not replace it

See `control/environment.md`.

## Shared arm-plus-hand CAN saturation

**Superseded 2026-08-11 by the four-bus topology.** Each device now owns its CAN
interface, so arm and hand traffic no longer compete for one bus and the
hand-command window is a selectable degraded mode rather than the operating
rule. The findings below described the shared-bus baseline and are kept because
`shared_per_side` remains selectable and because one of them was misread for
weeks.

What the shared bus produced: Stall-CAN detection that a CPU stall could trip on
a healthy bus; a recovery path that amplified the failure it was reacting to;
`ONE_SHOT=off` letting hand and arm traffic interleave at the cost of
retransmission buildup on the arm side; and missing-ACK retry spam that could
take the arm path down while the last commanded arm behaviour kept executing.

**The misread.** The RX-socket overflow that motivated step-and-settle was
**host-side socket overflow from CPU starvation, not bus arbitration.** Removing
the shared bus therefore did not remove it — parallel operation makes it *more*
likely, because the remaining shared resources are CPU, the GIL, executor
threads and the kernel socket buffers.

Rules that survive the topology change:

- the bridge's `joint_read_rate` is a real CAN lever; ROS `pub_rate` is not
- the Nero firmware executes the last MIT setpoint it received indefinitely, so
  after any transport event the arm is still doing whatever it was last told
- a rejected command is not automatically fail-closed for the same reason: a
  refusal on an active control stream needs a defined stop or hold with it

Current runtime contract: `project/control_integrity_architecture.md` and
`sprint_refactor/planning/integration_plan.md` (C1, C2).

## Recurring traps that cost more than one session

Escalated from the sprint records because each one recurred, or generalises past
the file it was found in. The detail and the evidence stay in
`sprint_refactor/errors_and_fixes.md` and `sprint6/errors_and_fixes.md`.

### A derived state mapping erases anything set directly

`_sync_authority` rebuilds the arm's authority state from the driver's gates on
every publish cycle. An emergency stop that set FAULTED directly was overwritten
on the next tick, and a verified stop left the arm accepting motion seconds
later. **Whatever must survive a derived mapping has to be an input to it, not
an output written behind its back.**

Only hardware surfaced it: the unit tests passed throughout, because none of
them ran the publish loop after an e-stop.

### An escalation rung that undoes the rung below it is not an escalation

The arm emergency stop established a firmware `MOVE-J(current_q)` hold, verified
it in feedback, and on a failed verification escalated to the vendor
`electronic_emergency_stop()` — which is a **damped descent**, not a hold. The
stronger-sounding command released the stiffness the weaker one had just
established.

Two things made it fire when nothing was wrong with the arm: "not verified"
covered both *measurably moving* and *the measurement produced no evidence*, so a
feedback hiccup inside the 0.5 s verification window was enough; and the
no-trustworthy-pose branch answered the same condition the pre-recovery hold
answers by claiming nothing at all.

Rules: **a safety ladder may only contain commands that are monotonically
stronger in the direction the ladder exists to move.** Where no stronger command
of the right kind exists, re-assert the one you have and say the result is
unverified — the next layer up is a different mechanism (here the external CAN
watchdog), not a different call on the same device. And **"could not measure" is
never evidence to act on**; it is evidence to report.

Found by reading the chain on 2026-08-20; neither call site had a test. Detail:
`sprint_refactor/reference/emergency_stop_ladder.md`.

### A `now - last >= interval` gate inside a loop paced at that interval loses rate

Third occurrence in this repository. It made a 20 Hz joint readback measure
15.4 Hz, which is why the acquisition loop was changed from gating to pacing —
and it was then reintroduced one function away from the comment that says so,
making a 20 Hz tactile cadence measure 10 Hz. Ordinary jitter makes cycles miss
the comparison.

Rule: an interval at or below the loop period means *every* cycle; a slower
cadence is compared with half a period of tolerance. Better still, pace the loop
instead of gating inside it.

### A rate argument that is forwarded rather than chosen belongs to whoever it was chosen for

Every bringup passed the *arm's* 200 Hz `pub_rate` straight into the hand
bridge, whose joints change at 20 Hz and whose status and tactile change once a
second. Nine of every ten executor wakes carried nothing new, and the cost was
charged to every subscriber as well as to the bridge — one consumer fell from
88 % of a core to 10 % without a line of it being touched.

200 Hz was right for an arm whose firmware pushes continuously. It was never a
statement about the hand.

### A cost that belongs to no call site is invisible to call-site profiling

91 % of a hand bridge's CPU was a vendor thread inside a compiled library,
reachable from no call site of ours. Per-call instrumentation attributed none of
it, and per-process CPU attributed all of it to our node. Only a **per-thread
census** separated the two — `ps -L` with `wchan`, where a thread that never
sleeps reads `wchan=0`.

Corollary: three times in this sprint the assumed CPU hot spot was measured
wrong. Decompose before optimising, and say out loud when a target is a
hypothesis.

### A measurement whose method can fail silently is not evidence

A CPU probe slept through its own sampling window and reported its deafness as
the publication rate: 6.5/s measured against 20/s actually published. It was
caught only because the bridge's own counter disagreed. **Two numbers that
disagree are the signal; one number alone gets believed.**

Same class: `ros2 topic hz` block-buffers when redirected, so the last seconds
are lost when the process is killed. Use `scripts/count_topic_messages.py`.

### A node that dies on an error path is invisible to every test that takes the happy path

`rclpy` caches a logger's severity per **call site** and raises if it changes, so
`(log.info if ok else log.warn)(msg)` is one call site with two severities. The
first refused claim raised out of the service callback and killed the bridge.
The symptom pointed at the client — a service that stops answering looks like a
caller problem — and the investigation went there first.

The path had no test at all, which is the general shape: the branches that only
run on failure are the ones that have never run.

## Implicit wrapper defaults

Problem: wrapper examples that omit `execution_profile` fall back to `manual`, which is not the current recommended operational path.

Current fix:

- package README examples now set explicit profiles such as `right_arm`, `right_hand`, or `duo_arm`
- operational launch matrices stay in `control/bringups/launches.md`

See `control/bringups/launches.md`, `src/agx_arm_moveit/README_EN.md`, and `src/agx_arm_mit_controller/README.md`.