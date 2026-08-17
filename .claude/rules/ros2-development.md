---
paths:
  - "src/**"
  - "**/*.launch.py"
  - "**/*.msg"
  - "**/*.srv"
  - "**/*.action"
  - "**/CMakeLists.txt"
  - "**/package.xml"
---

# ROS2 Development

*Use when making ROS2-native decisions in agx_arm_ros such as topics, services, messages, launch
surfaces, runtime validation, or value capture.*

Load this rule for ROS2-native questions and decisions.

## Core Rules

- keep the public ROS surface agx_arm-centric
- reuse the current owning packages before creating a new ROS2 surface
- when the task is multi-arm or multi-hand bringup, make description and launch surfaces arm-count-aware from the start
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; a hand command carries the authority it was issued under — `DeviceCommandStamp` (`owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`) inside `AuthorizedJointTrajectory` for trajectory execution or `HandJointTarget` for reactive motion. Shared `control/joint_states` is legacy and is not subscribed unless `allow_legacy_hand_command_ingress` is set (default false, development only), because a bare command makes the bridge invent the identity it then checks (`docs/sprint_refactor/planning/integration_plan.md`, C5 and 4D). The standard ROS messages are external types and stay untouched
- keep hand-only diagnostics under `feedback/omnihand/*`
- use standard ROS messages first and extend `agx_arm_msgs` only for repo-owned semantics, with statically defined fields
- do not treat vendor ROS packages or vendor topics as the public repo contract
- keep `colcon build` on system Python; use the repo-owned wrappers for optional Conda runtime and development shells instead of mixing build and runtime interpreters

## Value Capture

- top-level `docs/*.md` for global navigation and repo-wide checklist, fixes, and open questions
- `docs/control/` for how to run the system (environment, bringup launch/arguments, teach loop)
- `docs/project/` for stable structure, architecture, naming, workflow, and ROS2 practice decisions
- `docs/assets/` for stable runtime and bridge contracts, factual inventories, and validation state
- `docs/sprintN/` for sprint-level targets, checklist, errors, open questions, and retained evidence
- `docs/project/roadmap_and_phases.md` for long-term roadmap intent and thematic phases
- `docs/checklist.md` for current sprint status and cross-sprint blockers
- keep `.claude/` guidance synchronized with the stable docs it mirrors

## Validation

- run editor diagnostics on touched files
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

## Guarding Untrusted Numbers

- never let a saturating or clamping helper be the first thing that sees an untrusted value. `max(-limit, min(limit, value))` maps NaN and `+inf` onto `limit`, so a corrupt number becomes the *maximum* command and every downstream range check then sees a plausible value. Reject non-finite input first, then saturate
- a rejected command is not automatically fail-closed: the Nero firmware executes the last setpoint it received indefinitely, so dropping a command mid-stream leaves the previous motion running. A refusal on an active control stream has to be accompanied by a defined stop or hold
