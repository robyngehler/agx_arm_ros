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

## `spin_once` in a paced loop captures at a fraction of the loop rate

**Three sessions, 2026-08-22 to 2026-08-24.** `rclpy.spin_once` delivers **one
message from one subscription**. A loop paced at a fixed rate that calls it once
per cycle therefore captures at *loop rate ÷ ready callbacks*. Measured on a node
with four subscriptions at 96 loops/s: exactly 24 messages/s each.

Where it has bitten:

| | symptom |
| --- | --- |
| `count_topic_messages.py` | reported 121 Hz for a topic running at 198 Hz |
| teach recorder | stored 22 Hz of real content from a 100 Hz clock; a single-arm capture changed every 4th–5th row with *no* gap of 1, 2 or 3 anywhere in the file |
| the fix for the second | a fixed 32-call drain cost the loop 16% of its rate |

The rate scales with the clock, which is the tell and also the trap: lowering the
teach sample rate from 200 to 100 halved what was captured, for a reason that had
nothing to do with the arm. (Teach recording has since moved off a paced loop
entirely and has no rate argument.)

Fix: drain the remaining ready callbacks, and **stop as soon as a spin serves
nothing.** Every spin checks the node's whole wait set, so a fixed drain count is
paid even when the queues are empty, and the cost scales with the node — 3.6%
with four subscriptions, 16% with the ten service clients and two action clients
a real node carries. Either an early-exit drain or a spin on its own thread.

Two signatures worth recognising, because they point in opposite directions:

- **different rates per source** — a real per-source difference, look upstream
- **identical rates across sources** — the loop, not the sources

## A repeated sample is not data, and the file cannot tell you which it was

**2026-08-24.** A still arm and a missing update produce the same repeated row.
Once written, no reader can distinguish them — which is why two rounds of
forensics on stored recordings pointed at the wrong cause here.

A finite difference across a repeat alternates between zero and twice the true
value, so this is not cosmetic. The fix is to stop producing them: the recorder
knows whether the arm supplied a new frame, a later reader does not.

**Superseded 2026-08-24, same day:** the first fix removed the repeats *after*
the fact, by de-duplicating before the file was written. That leaves the
survivors on the times they were taken, which turns a uniform grid into a
bimodal 10/20/30 ms one — see the next entry. The recorder is now driven by the
arm's feedback callbacks instead, so a repeat is never stored, and
de-duplication is off by default. `scripts/clean_recording.py` still exists but
is no longer part of the recording path, and running it makes a grid less even.

## Removing rows from a recording is not the same as removing them from its grid

**2026-08-24.** Teach replay in `as_recorded` and `smooth` became too rough to
run, while the TOTG modes stayed smooth. Both timing-preserving modes filtered
and differentiated in **sample index space** — positions made comparable by
index, then divided by unequal intervals. That is a time-domain filter only on a
uniform grid, and de-duplication had just removed the uniformity.

`JointTrajectoryBuffer.sample()` interpolates linearly, so commanded velocity is
piecewise constant with one step per knot: 27-43 rad/s² of commanded
acceleration, ~50 sign changes per second per joint, a ~25 Hz excitation.

Three plausible causes that measured as nothing:

| suspected | measured |
| --- | --- |
| recording rate ≠ control rate | resampling 60-72 Hz → 200 Hz: 27.27 → 27.27, exactly no change |
| filter too narrow | 0.10 s → 0.30 s window: 12.5 → 11.6 |
| a regression from adding TOTG | the pre-TOTG path reconstructed measures 5.1-6.2, and never offered `as_recorded` |

Fix: **resample onto a uniform grid before filtering or differentiating, and
emit on that grid.** `as_recorded` keeps its name and carries a 0.06 s filter
floor — the taught path and pace at the smallest filter that executes.
Measurements and the mode table:
`docs/sprint_refactor/reference/teach_replay_timebase.md`.

The general form: **an operation indexed by sample is only the operation you
meant if the samples are evenly spaced.** Check the grid before trusting a
filter width or a derivative, and resample if you cannot guarantee it.

## A fix in the teach path is not a fix in the activity path

**2026-08-25.** Recorded replay was reworked end to end in the teach manager —
uniform resample, filter floor, velocity feedforward, chord-error waypoints,
`tempo_scale`. The coordinator reaches the same MIT controller through a
different chain (`recorded_to_waypoints` -> catalogue YAML -> `arm_executor` ->
`ExecuteTrajectory`) and had inherited none of it.

The one that matters most: `_build_execute_trajectory_goal` emitted positions and
times only, and the trajectory buffer reads a missing velocity as a **commanded
zero**, so `kd·(0 − q̇)` braked against the motion the position term was asking
for. That is the same defect the teach path was fixed for months earlier —
`|v_des − dp/dt|` measured 0.224 rad/s on the production path against 0.004 on
the teach path.

Catalogue waypoints were also chosen by even sample index, which is selection by
the clock: it spends a scarce budget on dwells rather than on corners. Chord-error
selection costs nothing in storage and measured 1.1-4.2x less chord error at the
same count.

The lesson is about where a contract lives: **two dispatch paths to one
controller will drift unless the thing they share is code, not a convention.**
The teach path and the coordinator both build a `JointTrajectory` for the same
consumer, and only one of them knew what that consumer does with a missing
field. Detail and what is still not shared:
`docs/sprint_refactor/reference/teach_replay_timebase.md`.

## An advancing timestamp is not advancing data

**2026-08-24.** After the two fixes above, a duo replay was still rough on the
right arm while the left was smooth and single-arm replays were fine. The
suspicion was the duo merge's time-axis re-base; it was not — the merged grid
measured exactly uniform at 9.225 ms.

The right arm's recording contained samples where **six of its seven joints
stepped together at 3-7x their own typical sample**, reaching 4.37 rad/s on a
3.93 rad/s joint. The left arm had none anywhere. Six joints accelerating
fivefold for exactly two samples and back is not a motion a hand can make: the
driver's position cache had stalled and then caught up.

The recorder captured on `header.stamp` advancing. That stamp is the receive time
of the **last CAN frame to touch the cache**, and a complete joint update is four
position frames — so the stamp advances while the positions need not. Storing
such a read asserts the arm was at that pose at that instant, which forces the
whole catch-up into one commanded step.

Capture now refuses a read whose positions equal the previous stored one. The
stall becomes a gap and the playback resample interpolates across it, which
reconstructs the underlying constant-velocity motion exactly (0.00 rad/s² of
commanded acceleration against 6.28 when the stall is stored). A genuinely still
arm interpolates flat between two equal poses, so it costs nothing.

**That first fix was half of it.** Refusing an unchanged read catches a stall of
the *whole* vector, and the next two takes still warned at 11.76 rad/s on a
3.93 rad/s joint. The vector is four position frames covering joint pairs, so one
pair can hold while another updates — a read that is genuinely new for the rest of
the arm and cannot be dropped. Each arm's samples now get a per-joint pass that
spreads such a step back over the hold that produced it, capped at 0.1 s so a
real dwell is not ramped.

Gating that pass on the velocity limit was tried and rejected: the observed stall
implied 1.30 rad/s, well under the limit, and still cost 9.86 rad/s² of commanded
acceleration. **Duration separates a stall from a dwell; speed does not.**

Three lessons worth separating:

- **A freshness signal is only as fine-grained as the thing that sets it.** Ask
  what advances the timestamp, not whether it advanced.
- **Fix an artefact at the granularity it occurs at.** The stall is per joint, so
  a per-read remedy could only ever catch the subset where every joint stalled
  at once.
- **A recording cannot be read back for this.** A stall and a still arm are the
  same rows in the file. The recorder is the only place that can report it, so
  each capture logs its stalled-read count, its per-joint spread count, and the
  worst implied joint speed.

## An over-limit sample in a teach recording is not proof of bad data

**2026-08-24.** The warning added above read "the feedback stalled and caught up
rather than the arm moving that fast". That asserts a cause the speed alone
cannot establish. **Freedrive back-drives the arm by hand, and a hand can move a
joint faster than the joint will accept as a setpoint** — 3.93 rad/s is a
commanded-velocity limit, not a mechanical bound on back-driving.

The two shapes separate cleanly, and it is the shape that carries the answer:

| shape | reading |
| --- | --- |
| an isolated step whose neighbours are near zero | the cache stalled and caught up |
| a run of consecutive large steps | the arm really was moved that fast |

Across two duo takes, 11 of 14 over-limit samples were runs — a bell-shaped ramp
over six samples reaching 8.3 rad/s. The recordings were honest; the right arm
was simply taught faster than it can be replayed, which `smooth` faithfully
preserves (velocity utilisation 0.81, max commanded acceleration 72.9 rad/s²
against the left arm's 9.7). `speed_scale` re-times against the limits by
construction and flattens both arms to 1.7.

The lesson is about diagnostics, not about the arm: **a threshold tells you a
sample is unusual, never why.** Say what was measured and what it costs, and
leave the cause to whatever can actually discriminate it.

## A rate configured above the arm's feedback source manufactures duplicates

**Measured 2026-08-22.** Acquisition, publication and teach recording were all
set to 200 Hz. The arms do not supply data that fast: on the wire, complete
joint state updates arrive at **~100/s on the right arm (FW 1.06)** and
**~137/s on the left (FW 1.11)**. A 200 Hz recording from that session carried
33.4% identical consecutive samples.

A frame count is not an update count. One complete state update is **eleven CAN
frames** — four position frames (`0x2A5`, `0x2A6`, `0x2A7`, `0x2A9`, two joints
each) plus seven motor-state frames (`0x251`–`0x257`). So ~2520 frames/s is
~150 updates/s, and the ~2 kHz figure that gets quoted for these joints belongs
to the servo loop *inside* the joint, which MIT closes locally and which never
appears on CAN. Eleven frames at 2 kHz would be 22,000 frames/s, ~2.75x the
whole 1 Mbit/s bus.

There is no knob for it. `0x477` byte 2 toggles only the `0x48X` end-V/acc
report, and `0x151` byte 6 is a boolean CAN-push enable that the driver already
sets to ENABLE.

Why nothing caught it: the driver's acquisition loop ran at ~180 Hz and had no
way to say that the frames it read had not changed, and its stall threshold
(`max(2 / acquisition_rate_hz, 0.2)` = 200 ms) cannot see a 33% shortfall at
all. `0ed2f1e` added the periodic achieved-rate report that makes the loop state
its own cadence.

Measure this below the SDK. `candump` reads the raw socket, so the SDK cannot be
the reason a frame is missing from it — but `candump` does **not** show the TX
loopback, and reading a capture as "no commands on the bus" is a mistake this
investigation made once. Take TX from
`/sys/class/net/<iface>/statistics/tx_packets`.

Full budget, per-joint rates and method: `sprint_refactor/reference/feedback_rate_budget.md`.

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