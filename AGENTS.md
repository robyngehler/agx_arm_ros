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
- Keep production MIT control ownership in `src/agx_arm_mit_controller`.
- Keep MIT demo and workflow apps in `src/agx_arm_mit_demos` instead of the controller runtime package.
- Keep MIT debug bridges, hold checks, and calibration helpers in `src/agx_arm_mit_tools`.
- Keep dual-arm/dual-hand task orchestration (Activity-DAG coordinator, performer routing, catalogue) in `src/agx_arm_coordination`; reuse the existing `both_arms`/per-arm FollowJointTrajectory path for arm execution.
- Keep the OmniHand bridge in `src/agx_arm_ctrl`; only revisit a package split after a non-mock backend proves a separate boundary is useful.
- Extend `src/agx_arm_msgs` for repo-owned OmniHand messages instead of creating a second message package.
- Treat `vendor/OmniHand-Pro-2025` as upstream input, not as the public ROS contract.
- Keep description and launch surfaces arm-count-aware (single arm, either side, both arms).

## ROS Contract Rules

- Keep the public ROS surface agx_arm-centric.
- Keep combined `feedback/joint_states` as the coordinated arm-plus-end-effector feedback surface. Shared `control/joint_states` is the current hand command flow and is legacy: the V02 target is one abstract hand command carrying owner identity, control epoch, and sequence.
- Each device owns its own CAN bus (arms `can0`/`can1` native, hands `can2`/`can3` on USB-CAN FD adapters). Same-side arm and hand motion may run in parallel; the shared-bus hand window is a selectable degraded mode, not normal operation.
- Use `feedback/omnihand/*` for hand-only diagnostics, status, and debugging.
- Keep `control/omnihand/joint_trajectory` only as a bridge-specific compatibility surface until a later action or controller contract is finalized.
- Do not extend `HandCmd`, `HandPositionTimeCmd`, or `HandStatus` for OmniHand, and do not add a further OmniHand-only command or status message. The V02 target consolidates them with `GripperStatus` and `OmniHandStatus` into one abstract hand contract that must fit any hand, with statically defined fields.

## Documentation And Source Rules

- Treat `.github/instructions/` and `.claude/rules/` as the agent-facing rule layer (workflow, naming, package, ROS2 practice); keep them consistent with the human docs under `docs/control/`, `docs/project/`, and `docs/assets/`.
- Keep `docs/README.md` and the top-level `docs/checklist.md`, `docs/errors_and_fixes.md`, and `docs/open_questions.md` aligned with any cross-cutting documentation restructuring.
- Update `docs/assets/` when an OmniHand or runtime component contract (command, feedback, launch, package) changes.
- Update `docs/project/` when repository structure or architecture changes; update `docs/control/` when environment or launch workflows change; update `.github/instructions/` and `.claude/rules/` when workflow, naming, or package-split rules change.
- Treat `docs/sprintX/` as the first-class sprint entrypoints and keep any surviving historical evidence inside those sprint surfaces.
- Treat `.github/`, `docs/`, `src/`, `scripts/`, `config/`, and `vendor/` as source-managed.
- Do not treat `build/`, `install/`, `log/`, or transient run outputs as canonical source.

## Hardware Access And Platform Rules

- Default to no hardware access until the user explicitly grants it for the current session.
- Before any hardware-touching action, ask whether hardware access is allowed. This includes `sudo` CAN bringup scripts, real arm or OmniHand launches, direct ROS hardware tests, and vendor SDK probes against live devices.
- If hardware access is granted, `sudo` is allowed for repo workflows because the password is intentionally disabled in the intended hardware environment.
- The two arms run different, unflashable firmware: right is 1.06 (default Nero protocol tier), left is 1.11 (`NeroFW.V111`). Mixed protocol tiers are the permanent baseline. Anything derived from the protocol — value ranges, frame encoding, status enums — is per tier, not per robot model, and every measurement names the arm it came from.
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