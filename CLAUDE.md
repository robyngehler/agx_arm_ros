# agx_arm_ros — Claude Code Project Instructions

@AGENTS.md

<!-- maintainer note: this is the Claude Code mirror of the former .github/copilot-instructions.md.
     Keep it in sync with docs/project/ (human repo structure) and docs/assets/ (component docs). Claude Code reads CLAUDE.md, not AGENTS.md,
     so AGENTS.md is pulled in via the @AGENTS.md import above. -->

This repository keeps `AGENTS.md` as the durable engineering contract and `.claude/` as the Claude Code
adapter layer. Keep the always-on context small and load only the rule, skill, or subagent that matches
the task.

## Operating Model

1. Read `README.md` or `README_EN.md` for repository orientation.
2. Read `docs/README.md` and the matching top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, or `docs/open_questions.md` when the task is cross-cutting or documentation-heavy.
3. `AGENTS.md` is imported above and carries the durable rules.
4. Read the single most relevant file from `.claude/rules/` for the task. Path-scoped rules also
   auto-load when you touch matching files.
5. Consult the matching human doc under `docs/control/`, `docs/project/`, or `docs/assets/` when the
  task changes operational workflow, package boundaries, public runtime contracts, or component
  architecture. Naming and ROS2-practice rules live in `.claude/rules/`.
6. Use a skill from `.claude/skills/` only when the task needs a reusable workflow.

## Source Of Truth Order

- repository overview: `README.md`, `README_EN.md`
- global docs hub and repo-wide status: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`
- durable engineering contract: `AGENTS.md`
- how to run the system (bringup launch/arguments, teach loop): `docs/control/`
- human repository structure and architecture: `docs/project/`
- component, runtime, and OmniHand docs: `docs/assets/`
- Claude Code operating model: this file (`CLAUDE.md`)
- targeted rules: `.claude/rules/`
- reusable workflows: `.claude/skills/`
- specialist personas: `.claude/agents/`

Do not load every rule by default. Match context to the task.

## What Lives Where

- routing and context minimization: `.claude/rules/context-routing.md`
- package boundaries and current roles: `.claude/rules/repository-structure.md`
- naming and package-split policy: `.claude/rules/package-naming.md`
- ROS2-native development and value capture: `.claude/rules/ros2-development.md`
- source versus generated asset rules: `.claude/rules/generated-vs-source-assets.md`
- local workflow and promotion order: `.claude/rules/local-agent-workflow.md`
- OmniHand bridge contract and runtime surface: `.claude/rules/omnihand-bridge.md`
- comment and docstring style in `src/`: `.claude/rules/source-comment-style.md`
- how to run the system (environment + bringup + teach): `docs/control/environment.md`, `docs/control/bringups/launches.md`, `docs/control/bringups/teach_and_run.md`
- global docs hub and repo-wide summaries: `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, `docs/open_questions.md`
- current sprint entrypoint: `docs/sprint_refactor/` (V02 refactor; the canonical
  plan is `docs/sprint_refactor/planning/integration_plan.md`)
- current sprint planning and reference notes: `docs/sprint_refactor/planning/`, `docs/sprint_refactor/reference/`
- `docs/sprint6/` is paused and adapts to the refactor contracts afterwards; its
  step-and-settle notes are superseded by the per-device CAN topology

The files under `.claude/rules/` are the canonical agent-facing rule layer (workflow, naming,
package-split, ROS2 practice). The human docs under `docs/project/` and `docs/assets/` describe repo
structure and component contracts; keep the two consistent.

## Skills

Use these only for task-shaped work:

- `commit-quality` — before every commit, no size threshold
- `test-ladder` — before validating any change, to pick the level and name it
- `omnihand_bridge_work`

## Subagents

Use these when a task benefits from a narrower persona (delegate via the `/agents` UI or by naming them):

- `ros-backend`
- `integration-review`
- `docs-keeper`

## Default Expectations

- keep the public ROS surface agx_arm-centric
- treat `src/duo_body_description` as the Duo body staging package for system assembly while keeping
  `src/agx_arm_sim/agx_arm_description` and `src/agx_arm_moveit` as the long-term canonical surfaces
- keep the OmniHand bridge in `agx_arm_ctrl`
- keep exactly one owner of a device's SDK session at any instant: steady-state
  calls go through that device's serialized worker on a declared priority lane,
  and recovery is the one exception because it takes the session off the worker.
  This holds for arms and hands alike — each hand bridge owns an `SdkWorker` with
  the same four lanes since 2026-08-15, and no ROS callback reaches the vendor SDK
  directly. The safety lane preempts the queue but not the call in flight, and the
  hand has no declared stop budget yet
- keep combined `feedback/joint_states` as the coordinated arm-plus-hand feedback surface; hand
  commands carry `DeviceCommandStamp` (`owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`)
  inside `AuthorizedJointTrajectory` (trajectory execution) or `HandJointTarget` (reactive motion) —
  one authority contract, two motion payloads. Shared `control/joint_states` is legacy and is not
  subscribed unless `allow_legacy_hand_command_ingress` is set (default false, development only)
- the two arms run different, unflashable firmware (right 1.06 default tier, left 1.11 `NeroFW.V111`);
  anything derived from the protocol is per tier, not per robot model, and a measurement names its arm
- the arm's feedback rate is the ceiling for every rate above it and is not
  configurable: ~100 complete state updates/s on the right arm, ~137/s on the
  left (wire, 2026-08-22). A rate set above it manufactures duplicates — a 200 Hz
  recording gave 33.4% identical consecutive samples. One update is eleven CAN
  frames, so a frame count is not an update count, and the ~2 kHz quoted for
  these joints is the joint's own servo loop, which never reaches CAN
  (`docs/sprint_refactor/reference/feedback_rate_budget.md`)
- where a source has its own cadence, take its callbacks rather than sampling it
  on a clock, and store a sample only when the payload changed. Teach recording
  has no rate to configure
- a freshness stamp is only as fine-grained as whatever sets it:
  `feedback/joint_states` carries the receive time of the last CAN frame to touch
  the driver cache, and one joint update is four position frames, so the stamp
  advances while the positions need not. Let a stall become a gap the consumer
  interpolates across, not a sample
- a recorded replay carries velocities on every dispatch path: the MIT
  trajectory buffer reads a missing velocity as a commanded zero and brakes
  against its own position command. Two dispatch paths to one controller drift
  unless what they share is code — the coordinator kept that defect months after
  the teach path was fixed. Catalogue waypoints are chosen by chord error, not
  even sample index (`docs/sprint_refactor/reference/teach_replay_timebase.md`)
- an operation indexed by sample is the operation you meant only if the samples
  are evenly spaced. A recording's grid is uneven, so a moving average over N
  rows is not a fixed-width filter and a row-index difference is not a
  derivative. Resample onto a uniform grid before filtering or differentiating
  and emit on that grid — the MIT controller interpolates linearly, so an uneven
  knot is a step in commanded velocity. Every replay mode does this,
  `as_recorded` included, which therefore means the taught path and pace at the
  smallest filter that executes
  (`docs/sprint_refactor/reference/teach_replay_timebase.md`)
- `rclpy.spin_once` delivers one message from one subscription, so a paced loop
  calling it once per cycle captures at loop rate over ready callbacks. Drain the
  rest and stop as soon as a spin serves nothing — a fixed drain count is paid
  even on empty queues and scales with the node's wait set. Identical rates
  across sources point at the loop; differing rates point upstream
- measure a CAN rate claim below the SDK with `candump` on the raw socket and
  cross-check `/sys/class/net/<iface>/statistics`; `candump` does not show the
  TX loopback, so take TX from `tx_packets`
- each device owns its own CAN bus: arms on `can_nero_left`/`can_nero_right` (native), hands on
  `hand_left`/`hand_right` (USB-CAN FD adapters); same-side arm and hand motion may run in parallel
  and the shared-bus hand window is a selectable degraded mode
- a hand has two production motion primitives — trajectory execution
  (`<side>_omnihand_controller/follow_joint_trajectory`) and reactive
  contact-seeking motion (the skill controller) — and exactly one owner at a
  time. Both claim `control/omnihand/claim_device` before commanding; the bridge
  is fail-closed, so an unclaimed hand executes nothing
- `control/omnihand/stop` is a cancel-and-hold, not a latching device stop
- the arm stop ladder ends at the firmware `MOVE-J(current_q)` hold; no safety
  path calls the vendor `electronic_emergency_stop()`, which is a damped descent
  and would release the stiffness the hold provides. An unverified stop
  re-asserts the hold, then asks for a bus-recovery link reset as transport
  repair; no trustworthy pose means nothing commanded and nothing claimed, and
  the external CAN watchdog owns that regime
  (`docs/sprint_refactor/reference/emergency_stop_ladder.md`)
- use repo-owned `agx_arm_msgs` messages for hand diagnostics and tactile payloads, with statically
  defined fields
- ask before any hardware-touching action; if hardware access is granted, `sudo` is allowed for repo CAN workflows in the intended hardware environment
- use `docs/control/environment.md` and `.claude/rules/ros2-development.md` for environment and ROS2-native decisions
- update stable docs when public contracts change
- keep `.claude/` guidance in sync with the stable docs it mirrors, and with the
  matching `.github/` adapter file — a rule changed in one layer only is a defect
- follow the `commit-quality` skill before every commit, and name the level the
  evidence came from when the change touches CAN, timing, or motion

## Quick Runtime Reference

- build touched packages: `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg_name>`
- run package tests from a system-Python ROS shell: `colcon test --packages-select <pkg_name>`
- Conda runtime command: `bash ./scripts/run_in_ros_conda.sh -- <command>`
- inspect runtime graph: `ros2 node list`, `ros2 topic list`, `ros2 service list`
