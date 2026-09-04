# AGENTS.md

This repository keeps durable, tool-neutral engineering rules here and uses `.github/` as the Copilot-native adapter layer.

## Scope

- Preserve the current stable package boundaries; `src/duo_body_description` is a staging surface for Duo body system assembly while long-term canonical description and planning ownership stays under the existing `agx_arm_*` packages.
- Keep implementation changes small, package-scoped, and validated.
- Keep documentation aligned with any public contract or workflow change.

## Source Of Truth Order

1. `README.md` and `README_EN.md` for repository overview.
2. `docs/README.md` plus top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, and `docs/open_questions.md` for global docs routing and cross-cutting status.
3. This file for durable engineering constraints.
4. `docs/control/` for how to run the system (`environment.md`, `bringups/launches.md`, teach loop).
5. `docs/assets/` for component architecture, validation, and OmniHand/runtime integration docs.
6. `docs/project/` for human-facing repository structure and architecture.
7. `.github/instructions/` and `.claude/rules/` for agent workflow, naming, and ROS2-practice rules (these do not live in `docs/`).
8. `.github/copilot-instructions.md` for the Copilot operating model.
9. `.github/skills/` for reusable workflows.

## Workspace Rules

- Keep `src/agx_arm_sim/agx_arm_description` as the canonical long-term description package and source of shared Nero and OmniHand assets.
- Keep `src/duo_body_description` as the Duo body staging package for the configurable arm-hand system assembly; do not duplicate full Nero or OmniHand asset trees there.
- Keep `src/agx_arm_moveit` as the MoveIt baseline and generalize it in place rather than forking a second MoveIt package for the Duo system.
- Keep runtime arm and hand integration in `src/agx_arm_ctrl`.
- Keep trajectory time parameterization in `src/agx_arm_retiming`. It exists as an `ament_cmake` package solely because MoveIt's time-optimal parameterization is C++ and Humble ships no `moveit_py`; the binding is a pure function over joint vectors and explicit limits, with no node in the graph. A path parameterization computes the timing and therefore discards the taught one, so a replay that must keep its taught dynamics uses the timing-preserving modes instead — `tempo_scale` reconstructs in taught time and scales the clock afterwards, keeping every ratio in the recording (dwells, reversals, duo phase; taught speed-profile correlation 1.000 against TOTG's 0.242) and walking the same path at every tempo, which is what a take taught faster than the arm can command needs. A tempo that would leave the joint speed limits is refused, naming the joint and the largest tempo that would pass (`docs/sprint_refactor/reference/trajectory_retiming.md`).
- Keep production MIT control ownership in `src/agx_arm_mit_controller`.
- Keep MIT demo and workflow apps in `src/agx_arm_mit_demos` instead of the controller runtime package.
- Keep MIT debug bridges, hold checks, and calibration helpers in `src/agx_arm_mit_tools`.
- Keep dual-arm/dual-hand task orchestration (Activity-DAG coordinator, performer routing, catalogue) in `src/agx_arm_coordination`; reuse the existing `both_arms`/per-arm FollowJointTrajectory path for arm execution. **A recorded replay dispatched from here carries velocities like any other**: the MIT trajectory buffer reads a missing velocity as a commanded zero and brakes against its own position command. Catalogue waypoints are selected by chord error, because a sparse budget spent by the clock lands on dwells. This path has the teach loop's retiming modes and its taught density: a recorded action carries an optional `playback` block (default `smooth` at 0.3 s), `PerformActivity.metadata_json` overrides it for one run, and a `recording:` reference keeps full density instead of inlining a decimation no downstream retiming could recover. `playback` is the **single** timing authority on a recorded action — the legacy `velocity_scaling` / `acceleration_scaling` stretch the taught times before the mode sees them, so declaring both is refused rather than multiplied. Every recorded action is planned before the first one moves, and that preplanning is cancellable between actions (`docs/sprint_refactor/reference/teach_replay_timebase.md`).
- Keep the OmniHand bridge in `src/agx_arm_ctrl`; only revisit a package split after a non-mock backend proves a separate boundary is useful.
- Extend `src/agx_arm_msgs` for repo-owned OmniHand messages instead of creating a second message package.
- Treat `vendor/OmniHand-Pro-2025` as upstream input, not as the public ROS contract.
- Keep description and launch surfaces arm-count-aware (single arm, either side, both arms).

## ROS Contract Rules

- Keep the public ROS surface agx_arm-centric.
- Keep combined `feedback/joint_states` as the coordinated arm-plus-end-effector feedback surface. Hand commands carry the authority they were issued under: `DeviceCommandStamp` (`owner_id`, `device_epoch`, `unit_safety_epoch`, `sequence`) travels inside `AuthorizedJointTrajectory` for trajectory execution and `HandJointTarget` for reactive contact-seeking motion. One authority contract, two motion payloads — not one abstract command message, because a time-parameterized trajectory and a next-target-per-cycle loop do not share a shape. Shared `control/joint_states` is legacy and no longer subscribed: the bridge takes it only under `allow_legacy_hand_command_ingress` (default false, development only), because a bare command forces the bridge to invent the identity it then checks. The standard ROS messages are external types and stay untouched.
- Each device owns its own CAN bus: arms on `can_nero_left` / `can_nero_right` (native), hands on `hand_left` / `hand_right` (USB-CAN FD adapters). Same-side arm and hand motion may run in parallel; the shared-bus hand window is a selectable degraded mode, not normal operation.
- Use `feedback/omnihand/*` for hand-only diagnostics, status, and debugging.
- A hand has **two** production motion primitives, and they may never command it at once. `<side>_omnihand_controller/follow_joint_trajectory` is the primary **trajectory-execution** path — coordinated arm/hand motion, planned gestures, later motion primitives. Reactive contact-seeking motion (the skill controller) is the second: it ends where the tactile sensor says rather than where the clock does, so it cannot be expressed as a trajectory goal. Neither is a debug surface. Keep `control/omnihand/joint_trajectory` only as a bridge-specific compatibility surface until a later action or controller contract is finalized.
- Exclusivity is enforced by device authority, not by topic separation. Both primitives claim `control/omnihand/claim_device` before commanding and release afterwards; the bridge is fail-closed, so an unclaimed hand executes nothing. Claim and release advance the device epoch, which is what makes a late command from the previous owner unexecutable.
- `control/omnihand/stop` cancels the pending hand target and holds the current pose. It is not a latching device stop; only the unit generation can latch a hand STOPPED. Closing that asymmetry belongs with the consolidated hand contract.
- The arm `emergency_stop` ladder ends at the firmware `MOVE-J(current_q)` hold, and **no safety path may call the vendor `electronic_emergency_stop()`**. That call applies damping without stiffness on both Nero tiers, so a raised arm slowly descends — precisely the state the hold exists to prevent. An unverified stop re-asserts the same hold at the pose the arm is at now (`ESTOP_HOLD_ATTEMPTS`), then requests a bus-recovery link reset as *transport repair*, not as a further motion escalation. Where no trustworthy pose exists, nothing is commanded and nothing is claimed. The external CAN watchdog owns every regime beyond that and is the layer free to command a descent. **This unit has no mechanical emergency stop** — the arm is either powered or it is not — so the only guaranteed stop is removing arm power, which drops the arm because a de-energized Nero has no brakes. **There is no kp=0 rung at any height**, and shutdown is a rung like any other. A kp=0 MIT command has no stiffness — it ends a moving setpoint and leaves nothing holding the arm up, which is a sag, not a weaker hold. The rungs are: MIT hold at the measured pose -> `MOVE-J(current_q)` (also reachable on its own as `hold_current_pose`, which latches no fault) -> `set_normal_mode`, a mode frame needing neither pose nor feedback -> the external CAN watchdog, which also commands `MOVE-J` at the current pose. The damped-stop helpers were deleted rather than left unused. The one surviving kp=0 command is freedrive, which is kp=0 *with* gravity feedforward and refuses to enter without a gravity model. Detail: `docs/sprint_refactor/reference/emergency_stop_ladder.md`.
- A bus the external CAN watchdog is holding quiet is **not** a fault. The watchdog takes the bus on an emergency stop, commands its own MOVE-J hold and gives the bus back on release; recovery cannot succeed against it, spends its budget, and latches the fault lockout that then refuses every command on the healthy bus that follows. The held-bus signature is: the link is up, RX has been silent longer than the arm's update spacing, and the controller's **transmit error counter** says the host is transmitting into a bus nobody answers. Do not build that last test on `tx_packets` — an unacknowledged ONE-SHOT frame is never counted, so the counter freezes exactly when the hold begins and no command stream can revive it (`docs/sprint_refactor/reference/bus_hold_tx_evidence.md`). Once the hold is entered only RX returning ends it, because the command stream is gated off while it lasts.
- More generally: a safety ladder may only contain commands that are **monotonically stronger in the direction the ladder exists to move**. Where no such command exists, re-assert the one you have and report the result as unverified — the next layer up is a different mechanism, not a different call on the same device. A check that could not measure has produced a result to report, never evidence to act on.
- Keep exactly one owner of a device's SDK session at any instant. Steady-state calls go through that device's serialized worker on a declared priority lane (emergency stop, then active control transmits, then control-critical acquisition, then diagnostics); recovery is the one exception, because it takes the session off the worker and is the owner while it runs. **This holds for arms and hands alike.** Each hand bridge owns an `SdkWorker` with the same four lanes as of 2026-08-15, and its acquisition runs on a separate paced thread, so no ROS callback reaches the vendor SDK directly. The remaining bound is the call already in flight: the safety lane preempts the queue, not the running call, and the hand has no declared stop budget yet.
- Do not extend `HandCmd`, `HandPositionTimeCmd`, or `HandStatus` for OmniHand, and do not add a further OmniHand-only command or status message. The **command** half of that consolidation is settled: `DeviceCommandStamp` plus the two motion payloads above. The **status** half is still open — `HandStatus`, `GripperStatus` and `OmniHandStatus` are to become one abstract hand status that fits any hand, with statically defined fields.

## Documentation And Source Rules

- Treat `.github/instructions/` and `.claude/rules/` as the agent-facing rule layer (workflow, naming, package, ROS2 practice); keep them consistent with the human docs under `docs/control/`, `docs/project/`, and `docs/assets/`.
- Keep `docs/README.md` and the top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, and `docs/open_questions.md` aligned with any cross-cutting documentation restructuring.
- Update `docs/assets/` when an OmniHand or runtime component contract (command, feedback, launch, package) changes.
- Update `docs/project/` when repository structure or architecture changes; update `docs/control/` when environment or launch workflows change; update `.github/instructions/` and `.claude/rules/` when workflow, naming, or package-split rules change.
- Treat `docs/sprintX/` as the first-class sprint entrypoints and keep any surviving historical evidence inside those sprint surfaces.
- Comments and docstrings under `src/` say what the code does and why it exists, short. No history of what the code used to do, no dated incident retellings, no rhetorical framing — that material belongs in `docs/sprintX/errors_and_fixes.md`, `docs/sprintX/reference/`, or the commit message. See `.claude/rules/source-comment-style.md` and `.github/instructions/source-comment-style.instruction.md`.
- Treat `.github/`, `docs/`, `src/`, `scripts/`, `config/`, and `vendor/` as source-managed.
- Do not treat `build/`, `install/`, `log/`, or transient run outputs as canonical source.

## Hardware Access And Platform Rules

- Default to no hardware access until the user explicitly grants it for the current session.
- Before any hardware-touching action, ask whether hardware access is allowed. This includes `sudo` CAN bringup scripts, real arm or OmniHand launches, direct ROS hardware tests, and vendor SDK probes against live devices.
- If hardware access is granted, `sudo` is allowed for repo workflows because the password is intentionally disabled in the intended hardware environment.
- The two arms run different, unflashable firmware: right is 1.06 (default Nero protocol tier), left is 1.11 (`NeroFW.V111`). Mixed protocol tiers are the permanent baseline. Anything derived from the protocol — value ranges, frame encoding, status enums — is per tier, not per robot model, and every measurement names the arm it came from.
- **The arm's feedback rate is the ceiling for every rate above it, and it is not configurable.** Complete joint state updates arrive at ~100/s on the right arm and ~137/s on the left, measured on the wire 2026-08-22. A rate configured above that — acquisition, publication — adds no information and costs file size, dispatch time and CPU; 200 Hz recording produced 33.4% identical consecutive samples. A frame count is not an update count: one update is eleven CAN frames (four position, seven motor state), so ~2520 frames/s is ~150 updates/s. The ~2 kHz quoted for these joints is the servo loop *inside* the joint, which MIT closes locally and which never reaches CAN. Detail: `docs/sprint_refactor/reference/feedback_rate_budget.md`.
- **Where a source has its own cadence, take its callbacks rather than sampling it on a clock, and store a sample only when the payload changed.** Teach recording stores one sample per changed feedback read, so the arm's cadence is the recording's cadence and there is no rate to configure. A clock above the source stores repeats; one below it discards updates; one at the source's rate beats against it and does both.
- **A freshness stamp is only as fine-grained as whatever sets it, and an artefact must be fixed at the granularity it occurs at.** `feedback/joint_states` carries the receive time of the last CAN frame to touch the driver's cache, and a complete joint update is four position frames covering joint pairs, so the stamp advances while the positions need not — and a stall can be **per joint** while the read is genuinely new for the rest of the arm. Teach capture refuses an unchanged read *and* spreads a single joint's catch-up back over its hold, capped at 0.1 s so a real dwell is not ramped. Duration separates a stall from a dwell; speed does not.
- **A threshold tells you a sample is unusual, never why.** A teach recording is back-driven by hand, and a hand can move a joint faster than it can be commanded — 3.93 rad/s is a setpoint limit, not a mechanical bound — so an over-limit sample is not proof of bad data. An isolated step whose neighbours are near zero is a cache catching up; a run of consecutive large steps is a motion really taught that fast. A take taught faster than the arm can reproduce replays cleanly under `speed_scale`, which re-times against the limits by construction.
- **An operation indexed by sample is the operation you meant only if the samples are evenly spaced.** A recording's grid is uneven — the arm jitters, and a dropped frame is a gap — so a moving average over *N rows* is not a filter of fixed width and a difference over row indices is not a derivative. Every replay mode therefore resamples onto a uniform grid before it filters or differentiates, and emits on that grid: the MIT controller interpolates linearly between trajectory points, so an uneven knot is a step in commanded velocity (27-43 rad/s² of commanded acceleration against 6 resampled). **`as_recorded` filters too** — it means the taught path and pace at the smallest filter that executes, not an unprocessed sample dump. Detail: `docs/sprint_refactor/reference/teach_replay_timebase.md`.
- Measure a CAN rate claim **below the SDK**, with `candump` on the raw socket, and cross-check it against `/sys/class/net/<iface>/statistics`. `candump` does not show the TX loopback, so a capture without command frames does not mean nothing was transmitted. **Take TX from `tx_packets` only while frames are being acknowledged.** An unacknowledged ONE-SHOT frame is transmitted but never counted, so the counter freezes exactly when a bus goes quiet under a watchdog hold — measured 2026-09-03, 3589 frames left the host at ~1320/s while `tx_packets` did not move. Take frame direction from a pcap's SLL packet type and transmit-side liveness from the controller's transmit error counter, which is a level and survives the command stream being gated off (`docs/sprint_refactor/reference/bus_hold_tx_evidence.md`).
- Distinguish Jetson or other `aarch64` ROS plus hardware sessions from x86 or editor-only sessions. Do not present x86 checks as a substitute for CAN timing or live-device validation.

## Environment And Build Rules

- Use `docs/control/environment.md` as the operational source of truth for wrappers, overlays, and platform split.
- Use `scripts/colcon_build_system_python.sh` for workspace builds.
- Run `colcon test` from a system-Python ROS shell, not from Conda.
- Use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands.
- Do not mix manual `conda activate` with `source install/setup.bash` in one shell flow.
- Treat `vendor/OmniHand-Pro-2025` as upstream input, not as a default workspace package to build with repo-wide `colcon build`.

## Validation Rules

- Prefer package-scoped `colcon build --packages-select ...` while iterating.
- Prefer the repo wrapper for those builds when the environment supports it.
- Run diagnostics on touched files.
- For launch, message, or bridge changes, include at least one executable package-level validation step.
- If live hardware validation cannot be run, say so explicitly.

## Definition Of Done

- The change respects the current package boundaries, including any temporary staging surface explicitly documented in `docs/project/`.
- The smallest relevant docs were updated.
- The Copilot-native `.github` mirrors remain consistent with the stable docs.
- The narrowest relevant validation was run, or the limitation was called out.

## Commit

- Before every commit, follow `.claude/skills/commit-quality/SKILL.md` (skill name: `commit-quality`). This is not optional and not only for large changes.
- A commit message states the **system-level change, why it was needed, and its consequence** — not a file list or a test inventory.
- Name the level the evidence came from. A change touching CAN, timing, or motion that was not exercised on hardware says so in one clause.
- Do not name co-authorship: no `Co-Authored-By:` trailer, no tool or model attribution, no "generated with" line, in any commit message, amend, or PR body.
- Keep unrelated changes in separate commits.

## Documentation Hygiene

- Use the `docs-keeper` agent at sprint boundaries or after large merges to reconcile documentation with the actual code state.
- When a superseded statement is corrected, rewrite the entry to the current state and mark the old reading as superseded with its date; do not append an "Update:" block that leaves the entry contradicting itself.
- A document that no longer describes current state carries a first-line banner naming what changed and where the current record lives. Operational docs under `docs/control/` are bannered rather than rewritten while the code still implements the old behaviour.
- `.claude/` and `.github/` are parallel adapter layers over the same rules: a rule, skill, or agent changed in one and not the other is a defect.