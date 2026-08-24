---
description: "Use when making ROS2-native decisions in agx_arm_ros such as topics, services, messages, launch surfaces, runtime validation, or value capture."
---

# ROS2 Development

Load this instruction for ROS2-native questions and decisions.

## Core Rules

- keep the public ROS surface agx_arm-centric
- reuse the current owning packages before creating a new ROS2 surface
- when the task is multi-arm or multi-hand bringup, make description and launch surfaces arm-count-aware from the start
- the quarantine on `control/move_j` (and the other bare arm-motion topics) applies to unauthenticated **ROS ingress**, not to the driver's internal `move_j(current_q)` primitive. That internal call is the firmware position hold the emergency stop and pre-recovery hold depend on: a kp=0 damped MIT command has no stiffness and sags as a terminal state. Do not remove it while cleaning up legacy interfaces
- **the arm stop ladder ends at that hold, and no safety path may call the vendor `electronic_emergency_stop()`.** It sends `ArmMsgMotionCtrl(1)` on both Nero tiers and applies damping without stiffness, so a raised arm slowly descends — the state the hold exists to prevent. An unverified stop re-asserts the same hold at the current pose (`ESTOP_HOLD_ATTEMPTS`), then asks for a bus-recovery link reset as *transport repair*; no trustworthy pose means nothing is commanded and nothing is claimed. The external CAN watchdog owns every regime beyond that and is the layer free to command a descent (`docs/sprint_refactor/reference/emergency_stop_ladder.md`). The one surviving call is the Piper-only teach-mode exit, which is a mode transition paired with its own `reset()`
- a safety ladder may only contain commands that are **monotonically stronger in the direction the ladder exists to move**. Where no such command exists, re-assert the one you have and report the result as unverified — the next layer up is a different mechanism, not a different call on the same device. And a check that could not measure has produced no evidence to act on, only a result to report
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; a hand command carries the authority it was issued under — `DeviceCommandStamp` (`owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`) inside `AuthorizedJointTrajectory` for trajectory execution or `HandJointTarget` for reactive motion. Shared `control/joint_states` is legacy and is not subscribed unless `allow_legacy_hand_command_ingress` is set (default false, development only), because a bare command makes the bridge invent the identity it then checks (`docs/sprint_refactor/planning/integration_plan.md`, C5 and 4D). The standard ROS messages are external types and stay untouched
- a payload change is declared by the **action**, never inferred from a hand preset or gesture. `payload_update: attach|detach` in the catalogue drives the MIT controller's `~/payload_attached` (`std_srvs/SetBool`), which swaps between two gravity models preloaded at startup. `pre_grip` and `release` run the same `can_pre_grip` shape and only one of them means the object is gone, so a reusable gesture must not carry the task's physical consequence. The coordinator applies the transition before the node counts as completed and fails the activity if it does not take (`docs/sprint6/reference/payload_gravity_model.md`)
- keep hand-only diagnostics under `feedback/omnihand/*`
- use standard ROS messages first and extend `agx_arm_msgs` only for repo-owned semantics
- do not treat vendor ROS packages or vendor topics as the public repo contract
- keep `colcon build` on system Python; use the repo-owned wrappers for optional Conda runtime and development shells instead of mixing build and runtime interpreters

## Value Capture

- top-level `docs/*.md` for global navigation and repo-wide checklist, fixes, and open questions
- `docs/control/` for how to run the system (environment, bringup launch/arguments, teach loop)
- `docs/project/` for stable structure, architecture, naming, workflow, and ROS2 practice decisions
- `docs/assets/` for stable runtime and bridge contracts
- `docs/assets/` for stable factual inventories and validation state
- `docs/sprintN/` for sprint-level targets, checklist, errors, open questions, and retained evidence
- `docs/project/roadmap_and_phases.md` for long-term roadmap intent and thematic phases
- `docs/checklist.md` for current sprint status and cross-sprint blockers
- use `docs/sprint_refactor/` for the current top-level sprint routing and `docs/sprint_refactor/planning/` plus `docs/sprint_refactor/reference/` for the detailed working record
- keep `.github/` guidance synchronized with the stable docs it mirrors

## Validation

- run diagnostics on touched files
- prefer `bash ./scripts/colcon_build_system_python.sh --packages-select ...` for touched-package builds when the environment supports it
- run `colcon test --packages-select ...` from a system-Python ROS shell when relevant tests exist
- say explicitly when hardware validation could not be run
- when Python environment drift is part of the issue, use `scripts/colcon_build_system_python.sh` for builds and `scripts/run_in_ros_conda.sh` for optional runtime commands
- do not use `ros2 topic hz` to answer "did this node stop publishing?". Its output is block-buffered when redirected, so the last seconds are lost when the process is killed, and a shell marker appended to the same file has no defined position relative to those flushes. It produced two confident-looking zeros during Phase 1A, one of which briefly supported a wrong conclusion. Use `scripts/count_topic_messages.py`, which counts over a fixed window and prints once at the end
- a measurement whose method can fail silently is not evidence. Prefer a tool that reports a number once, over one that streams and may be cut off

## Reaching A Device's SDK

The arm driver serializes every steady-state SDK call for a device onto one worker thread (`agx_arm_ctrl/sdk_worker.py`). The invariant is **one owner of a device's SDK session at any instant**, and it is read off the per-thread call counter in `RuntimeMetrics` — a call that bypasses the worker shows up under a different thread name.

- do not call `self.agx_arm.*` from a subscription callback, a service handler, or a timer. Submit it to the worker and pick the lane
- four lanes, strict priority: `SAFETY` (emergency stop) > `CONTROL` (active control transmits) > `ACQUISITION` (feedback the control loop and watchdog need) > `DIAGNOSTIC` (status and one-off reads, the default). The default is the lowest on purpose, so work nobody classified cannot overtake the control stream
- the unit of work is **bounded** work, not "one SDK call". A fixed batch of cached reads is one task; a retry loop bounded only by a timeout never is, because it converts a 1 ms call into a multi-second block in front of a stop
- a command that is several transmits but one instruction — a MIT setpoint is seven joint frames inside a mode bracket — is a **cycle** (`submit_cycle`): one queue entry for the epoch check and the supersede, executed one step at a time with the safety lane drained between steps. Sent as a single task it measured 21 ms of non-preemptible work, more than the whole stop budget; sent as seven independent submissions, two setpoints interleave and the arm holds half of each
- stamp a submission with the device epoch so a recovery discards what was issued before it, and give a streaming setpoint a `replace_key` so a superseded one is dropped while queued rather than delivered late
- **recovery is the exception.** It quiesces the worker and takes the session, so it calls the SDK directly — at that moment it *is* the owner. Anything routed through the worker during recovery waits for a handover that does not complete until recovery ends
- a timeout on a submitted call means the outcome is **unknown**, not that the call was not sent. Only a drop, a supersede or a rejection establishes non-execution

## A Node Owns The Threads It Starts

`rclpy` joins nothing. `destroy_node()` and `rclpy.shutdown()` both return while a thread the node started is still running, and `daemon=True` does not help — it only stops the *process* from hanging at exit, which is a different moment.

- stop and join the thread in an overridden `destroy_node()`, not only in `main()`. `main` is not the path a test, a composed launch, or a node that fails during construction takes
- the thread holds the node, its logger and its backend, so an orphaned one keeps calling into a destroyed context. It does not fail where the bug is: it fails in whatever runs next. **The signature is a suite where every test passes alone and fails together** — that is how the hand bridge's acquisition thread was found
- name the thread after the device it serves (`hand-acq-right`), because per-thread call attribution is how the one-owner invariant is read, and `Thread-3` proves nothing
- a test that drives such a node by hand calls the loop body (`_acquire_once`), never the loop. The thread belongs to the runtime; the test wants one deterministic cycle

## Rates, Logging, And Derived State

Traps that have each cost a session, and several cost more than one.

- **`spin_once` delivers one message from one subscription.** A loop paced at a fixed rate that calls it once per cycle therefore captures at *loop rate ÷ ready callbacks*, not at the loop rate. Measured on a node with four subscriptions at 96 loops/s: exactly 24 messages/s each. This has cost three sessions in this repository — a measurement script reporting 121 Hz for a 198 Hz topic, a teach recorder storing 22 Hz of real content from a 100 Hz clock, and the fix for the second one costing the loop 16% of its rate. Drain the remaining ready callbacks, and **stop as soon as a spin serves nothing**: every spin checks the node's whole wait set, so a fixed drain count is paid even when the queues are empty, and the cost scales with how many clients and services the node holds (3.6% with four subscriptions, 16% with a realistic wait set). A sampling loop wants a drain with an early exit, or a spin on its own thread
- **a repeated sample is not data, and the stored file cannot tell you which it was.** A still arm and a stalled cache look identical once written. Do not produce them: where a source has its own cadence, take its callbacks instead of sampling it on a clock, and store a sample only when the **payload** changed. Removing repeats *after* the fact is the weaker fix and has its own cost — it leaves the survivors on the times they were taken, which is how a uniform grid became a bimodal 10/20/30 ms one
- **a freshness stamp is only as fine-grained as whatever sets it.** `feedback/joint_states` carries the receive time of the last CAN frame to touch the driver's cache, and a complete joint update is four position frames — so the stamp advances while the positions need not. A recorder keyed on the stamp stored the stalls, and the catch-up arrived as one commanded step: six of seven joints moving 3-7x their typical sample together, at 4.37 rad/s on a 3.93 rad/s joint. Ask what advances a timestamp before trusting it as "new data", and let a stall become a gap that the consumer interpolates across
- **an operation indexed by sample is the operation you meant only if the samples are evenly spaced.** A recording's grid is uneven, so a moving average over N rows is not a filter of fixed width and a difference over row indices is not a derivative. The MIT controller interpolates linearly between trajectory points, so an uneven knot is a step in commanded velocity: 27-43 rad/s² of commanded acceleration, ~50 sign changes/s/joint, against 6 rad/s² for the same recording resampled. Resample onto a uniform grid before filtering or differentiating, and emit on that grid — every replay mode does, `as_recorded` included (`docs/sprint_refactor/reference/teach_replay_timebase.md`)
- **a configured rate above the source rate produces duplicates, not data.** The arms deliver ~100 complete state updates/s (right) and ~137/s (left) on the wire, and the rate is not configurable — acquisition and publication were both set to 200 Hz, and a recording made on the same clock carried 33.4% identical consecutive samples. Check the source before raising anything above it, and remember that a frame count is not an update count: one arm state update is eleven CAN frames, so ~2520 frames/s is ~150 updates/s. Measure below the SDK with `candump` on the raw socket, cross-check `/sys/class/net/<iface>/statistics`, and take TX from `tx_packets` because `candump` does not show the TX loopback (`docs/sprint_refactor/reference/feedback_rate_budget.md`)
- **a loop that reports how fast it ran has not reported how fast its data changed.** The driver's acquisition loop achieved ~180 Hz while reading frames that updated at ~100 Hz, and its stall threshold (`max(2 / rate, 0.2)` = 200 ms) could not see a 33% shortfall at all. A cadence report is evidence about the loop; freshness needs the data's own timestamp
- **do not gate a periodic action on `now - last >= interval` inside a loop already paced at that interval.** Ordinary jitter makes cycles miss the comparison, and the achieved rate lands well below the configured one — it has produced 15.4 Hz from a 20 Hz readback and 10 Hz from a 20 Hz tactile cadence in this repository, the second time one function away from the comment warning about the first. An interval at or below the loop period means *every* cycle; a slower cadence is compared with half a period of tolerance
- **a rate argument that is forwarded rather than chosen belongs to whoever it was chosen for.** The arm's `pub_rate` passed into the hand bridge made it publish ten times faster than its data changed, and every subscriber paid for it too. Publication is driven by new data; a rate parameter is a ceiling that can throttle and never drive
- **`rclpy` caches a logger's severity per call site and raises if it changes.** `(self.get_logger().info if ok else self.get_logger().warn)(msg)` is one call site with two severities: the first time the other branch runs, the exception unwinds out of the callback and can take the node with it. Write two call sites
- **a derived state mapping erases anything set directly.** A state rebuilt from its inputs every publish cycle overwrites whatever a callback assigned behind its back — that is how a latched emergency stop came back as `READY` a second later. Anything that must survive the mapping has to be an *input* to it

## Guarding Untrusted Numbers

- never let a saturating or clamping helper be the first thing that sees an untrusted value. `max(-limit, min(limit, value))` maps NaN and `+inf` onto `limit`, so a corrupt number becomes the *maximum* command and every downstream range check then sees a plausible value. Reject non-finite input first, then saturate
- a rejected command is not automatically fail-closed: the Nero firmware executes the last setpoint it received indefinitely, so dropping a command mid-stream leaves the previous motion running. A refusal on an active control stream has to be accompanied by a defined stop or hold
