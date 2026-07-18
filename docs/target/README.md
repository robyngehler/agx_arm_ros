# Documentation Restructure Target

status: ACTIVE_CONTROL_PASS
last_updated: 2026-07-18
scope: repo-wide docs and agent cleanup

## Purpose

This document is the global target for the current documentation cleanup pass.

It defines:

- the target documentation structure
- the migration order from the current repo state
- the agent-file rules that must stay short and consistent
- the environment, hardware, and CAN findings that must not get lost during the cleanup

The immediate goal is not to rewrite everything at once. The goal is to turn the first cleanup draft into a verified control document so the next edits can remove duplication instead of moving it around.

## State Before This Pass

The repo already contains useful material, but the information architecture is drifting.

### Observed problems

- The root `README.md` mixed project overview, installation, CAN setup, launch examples, and detailed runtime behavior. It was too large for a global entrypoint.
- `docs/control/bringup.md` and `docs/control/teach_and_run.md` were valuable operational sources, but the root README still duplicated too much of that usage guidance.
- `docs/project/` mixed true project structure with environment workflow details that belonged closer to day-to-day usage.
- `docs/assets/` contains both stable factual inventories and some runtime explanations that overlap with `docs/project/` and `docs/control/`.
- `docs/development/` currently acts as both a cross-sprint workspace and the container for sprint folders. This conflicts with the target model where sprint folders should be first-class docs surfaces under `docs/`.
- The top-level development docs (`docs/development/checklist.md`, `docs/development/errors_and_fixes.md`, `docs/development/open_questions.md`) duplicate the role of the global top-level docs.
- Several package-local READMEs, CAN docs, and historical notes still refer to `control/bringup.md` or `project/python_environment_workflow.md` instead of the new canonical targets.
- The top-level sprint surfaces now exist, but most detailed sprint evidence still remains under `docs/development/sprintX/` and has not yet been physically migrated.
- `.claude/` and `.github/` now route toward the new stable docs, but package-local docs and historical notes still need incremental cleanup to remove older path references.
- Hardware, environment, and validation rules are now anchored more clearly, but some secondary docs still point at compatibility shims.
- The recent CAN findings from the latest test session are not yet promoted into the global docs or agent rules strongly enough, even though they affect how shared arm-plus-hand operation should be described and tested.

### Current canonical strengths to preserve

- `docs/control/bringups/launches.md` is now the canonical command-first operational entrypoint.
- `docs/control/teach_and_run.md` already captures the real teach/replay workflow and should remain usage-focused.
- `docs/control/environment.md` now captures the system-Python versus Conda split clearly.
- `AGENTS.md` already contains the durable engineering contract and should remain the single long-lived agent contract.
- The recent July changes already moved the repo closer to a stable Duo baseline and should be kept, not re-explained from scratch.

## Verified Control-Session Findings

This control pass confirms that the first draft was directionally correct, but several repo entrypoints still need to be realigned before the target structure becomes true in practice.

- `docs/README.md`, `README.md`, and `README_EN.md` now act as routing surfaces instead of mixed operating manuals.
- `docs/control/environment.md`, `docs/control/bringups/launches.md`, `docs/project/architecture.md`, and `docs/project/components/README.md` now exist as canonical stable targets.
- `CLAUDE.md`, `.github/copilot-instructions.md`, and the mirrored rule files now enforce the hardware gate, platform split, and the new docs routing more explicitly.
- The next cleanup wave should target package-local READMEs, CAN-facing docs, and historical notes that still point at compatibility shims or pre-migration paths.

## Target Documentation Structure

The target structure for the repo is:

```text
README.md
docs/
  README.md
  checklist.md
  target/
    README.md
  errors_and_fixes.md
  open_questions.md
  control/
    bringups/
      launches.md
    environment.md
    <component usage docs when needed>
  project/
    architecture.md
    components/
      README.md
      <component docs when they are stable>
  sprintX/
    target/
      README.md
    checklist.md
    errors_and_fixes.md
    open_questions.md
    <optional subfolders for sprint-local evidence>
```

## Target Responsibilities By Surface

### Root README

Keep only:

- what the repo is
- the package and docs map
- the fastest correct path into the operational docs
- a short build/runtime summary

Do not keep full bringup matrices, deep CAN rationale, or long launch argument explanations here.

### docs/README.md

Acts as the human docs hub only.

Keep it short and primarily route to:

- `docs/target/README.md`
- `docs/checklist.md`
- `docs/errors_and_fixes.md`
- `docs/open_questions.md`
- `docs/control/`
- `docs/project/`
- current and historical sprint folders

### docs/checklist.md

Tracks only global goals, integration status, and cleanup progress.

Do not duplicate sprint-local implementation details here.

### docs/target/README.md

Tracks the global repo target, the active documentation migration, and the current sub-phase focus.

This file should become the main place for repo-level goals that are too dynamic for `AGENTS.md` but too stable for one sprint note.

### docs/errors_and_fixes.md

Tracks only cross-cutting, recurring, or repo-level issues.

If a problem is local to one sprint, keep it in that sprint until it clearly affects the stable baseline.

### docs/open_questions.md

Acts as the global exchange surface between human and agents for unresolved design choices.

Questions should be short, actionable, and linked outward instead of explained three times.

### docs/control/

This is the operational source of truth.

Planned split:

- `docs/control/bringups/launches.md`: all stable startup and launch entrypoints
- `docs/control/environment.md`: environment creation, system Python versus Conda, ROS overlay handling, build wrappers, testing wrappers, and platform caveats
- additional component usage docs only when usage needs more than one short section in `launches.md`

`docs/control/teach_and_run.md` remains valid, but it should eventually sit beside the new launch and environment docs as a focused workflow guide.

### docs/project/

This is the stable architecture and component-relationship surface.

Planned split:

- `docs/project/architecture.md`: high-level architecture, package interaction, runtime flow, and Mermaid diagrams
- `docs/project/components/README.md`: index of stable component descriptions
- `docs/project/components/<name>.md`: only for stable components that need more than a short section in `architecture.md`

Content that is mostly "how to use or run it" should move out of `docs/project/` and into `docs/control/`.

### docs/sprintX/

Sprint folders should move out of `docs/development/` and become first-class docs surfaces under `docs/`.

Each sprint folder should contain:

- `target/README.md`
- `checklist.md`
- `errors_and_fixes.md`
- `open_questions.md`
- optional subfolders for evidence, experiments, and component-local notes

Sprint folders should store working evidence and evolving implementation notes. Once a decision becomes stable, it must be promoted into `docs/control/`, `docs/project/`, or the top-level docs.

## Proposed Migration Map From The Current Repo

### Global docs

- keep `docs/README.md`, `docs/checklist.md`, `docs/errors_and_fixes.md`, and `docs/open_questions.md`
- add `docs/target/README.md` as the new global target surface
- remove the need for duplicate development-layer checklist, fix, and open-question files once the sprint migration is complete

### Control docs

- completed: `docs/control/bringup.md` now points at `docs/control/bringups/launches.md`
- keep `docs/control/teach_and_run.md` and later retitle or relocate only if the surrounding structure is already stable
- completed: `docs/project/python_environment_workflow.md` now points at `docs/control/environment.md`
- keep `docs/CAN_USER.md` and `docs/CAN_USER_EN.md` only as lower-level CAN references, not as the main startup path

### Project docs

- slim `docs/project/repository_structure.md` down to package ownership, documentation responsibilities, and stable boundaries
- completed: `docs/project/repo_interaction_diagrams.md` now points at `docs/project/architecture.md`
- completed: a stable component index now exists under `docs/project/components/`

### Sprint docs

- completed: `docs/sprint1/` through `docs/sprint6/` now exist as first-class migration entrypoints
- reduce `docs/development/` to either a temporary redirect surface during migration or remove it once all references are updated

## Documentation Rules For The Cleanup

### Usage-first rule

When choosing where information belongs, prefer the place that helps someone correctly:

- bring the system up
- select the right script or launch file
- run the right test
- understand platform constraints

Deep historical rationale should only stay where it still helps decisions.

### Single source of truth rule

One operational fact should have one canonical home.

Examples:

- launch matrices belong in `docs/control/bringups/launches.md`
- Python environment rules belong in `docs/control/environment.md`
- package boundaries belong in `docs/project/architecture.md` or the remaining slim repository-structure doc
- sprint-local evidence belongs in `docs/sprintX/`

All other places should link, not restate.

### Stable versus working-note rule

- stable instructions and stable architecture go into top-level docs surfaces
- experiments, transient findings, and implementation logs stay in sprint folders until promoted
- package READMEs should describe package-local behavior only, not maintain a second launch matrix

## Agent File Rules

## Durable responsibilities

- `AGENTS.md` remains the durable engineering contract
- `CLAUDE.md` and `.github/copilot-instructions.md` remain thin routing layers that point to the real docs and rules
- `.claude/rules/` and `.github/instructions/` remain concise agent-facing mirrors for workflow, naming, package boundaries, ROS2 rules, and hardware rules

## What agent files must not do

- they must not duplicate large operational procedures from `docs/control/`
- they must not become a second architecture manual next to `docs/project/`
- they must not contain outdated sprint references once the sprint folders move
- they must not keep conflicting wording between `.claude/` and `.github/`

## Required agent rules to add or make explicit

### Hardware access gate

Before any hardware-touching action in a session, agents must explicitly ask whether hardware access is allowed in this session.

Default state: no hardware access until the user grants it for the current session.

Hardware-touching actions include at least:

- `sudo` CAN bringup scripts
- launch commands against real arms or OmniHands
- direct ROS hardware tests
- vendor SDK probes against live devices

If the user grants hardware access for the session:

- agents may run the necessary hardware commands
- `sudo` is allowed for repo workflows because the password is intentionally disabled in this environment
- agents must still state when a command targets live CAN, real arms, or live OmniHands

If hardware access is not granted:

- agents must stay in documentation, offline analysis, code, and non-hardware validation paths
- agents must explicitly say that hardware validation could not be run in the current environment

### Platform split rule

Agent instructions should explicitly separate:

- Jetson or other `aarch64` ROS plus hardware environments
- x86 or non-ROS editor environments without live hardware access

Docs and validation instructions must say which class of environment they apply to.

Session routing rule:

- identify the active environment class near the start of the session
- default this workspace's current x86 editor context to offline analysis unless the user says otherwise
- do not present x86 editor-only checks as substitutes for Jetson hardware validation when the behavior depends on CAN timing, ROS bringup, or real devices

### Environment and build rule

Agent files must consistently point to these repo rules:

- use `scripts/colcon_build_system_python.sh` for workspace builds
- keep `colcon test` on a system-Python ROS shell, not on Conda
- use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands
- do not mix manual `conda activate` with `source install/setup.bash` in one shell flow
- treat `vendor/OmniHand-Pro-2025` as upstream input, not as a normal workspace package to build via the default repo-wide `colcon build`
- mention that the system-Python build wrapper now filters stale local prefix paths and skips the vendor OmniHand SDK by default during repo-wide builds unless the vendor package is selected explicitly

## Captured Current Technical Findings

This section captures details that must survive the documentation cleanup even if they are not yet fully promoted everywhere.

### Findings visible in recent commits

- 2026-07-15: `duo_hand` bringup and execution-profile support landed, along with updated teach and bringup docs.
- 2026-07-15: teach manager gained automatic topic-prefix discovery.
- 2026-07-15: the system-Python build wrapper now filters stale local prefix paths and skips the vendor OmniHand SDK by default during repo-wide builds.
- 2026-07-03: hand-aware gravity handling and OmniHand delivery hardening on the shared CAN path landed.

### Human-provided carry-over findings from the latest CAN test session

These findings were provided directly by the user and should be promoted into the stable docs after the relevant code and behavior are rechecked.

- Stall-CAN detection in `agx_arm_ctrl` is partly faulty because it depends too much on a timer. Under 100 percent CPU load the timer can stall, so the node resets CAN even though CAN still works.
- The current CAN probing and recovery path is incomplete and can be actively harmful for follow-up failure modes.
- `one-shot off` can allow parallel hand and arm traffic, but in the current state it is a dangerous temporary workaround rather than a stable fix.
- Missing ACK can trigger spam and bus overflow, which can also take down the arm side and leave the last command, including torque-producing behavior, active until CAN is brought down and up and the controller is restarted.
- A promising direction is to keep `one-shot on`, freeze or pause active arm control during explicit hand-command windows, and avoid concurrent command pressure on the shared bus when the arm is supposed to hold statically anyway.
- Hand-side error spam when feedback is missing should be reduced so the system uses the shared CAN bandwidth more deliberately.
- A likely operating rule for the stable baseline is: arm active while controlling the arm, hand passive by default; when a hand action is required, arm control pauses into a safe static hold and the hand temporarily gets the bus budget.

These points should also feed the global `docs/errors_and_fixes.md`, sprint-local error tracking, and the future stable CAN/bringup guidance.

## Proposed Migration Phases

### Phase 1

Create the target skeleton and stop new duplication.

- completed in this pass

### Phase 2

Move the operational source of truth.

- largely completed in this pass; remaining work is secondary reference cleanup in older notes

### Phase 3

Move the stable architecture surface.

- largely completed in this pass; remaining work is further trimming older project-era compatibility pages

### Phase 4

Move sprint working notes into the new first-class sprint layout.

- first-class `docs/sprintX/` entrypoints are now in place
- remaining work: move or rewrite detailed sprint evidence out of `docs/development/sprintX/`
- remove duplicated top-level development checklist, fixes, and questions

### Phase 5

Normalize agent guidance.

- largely completed in this pass; remaining work is cleanup of historical references inside older skills or notes only when they still affect active routing

## Next Follow-Up Edits

The next documentation edits should happen in this order:

1. Continue replacing active package-local and top-level doc references that still point at compatibility shims such as `docs/control/bringup.md` or `docs/project/python_environment_workflow.md`.
2. Decide which detailed sprint evidence should be physically moved from `docs/development/sprintX/` into the new `docs/sprintX/` surfaces and which should remain historical.
3. Reduce or retire the duplicate top-level development checklist, fix, and open-question files once the new sprint surfaces carry the needed state.
4. Review older stable asset docs for outdated Sprint 2 phrasing and normalize them only where they still read like active policy rather than history.

## Definition Of Done For This Cleanup

This cleanup pass is done only when:

- the root README is short and routes to the right docs
- operational instructions live under `docs/control/`
- stable architecture and component ownership live under `docs/project/`
- sprint-local evidence lives under `docs/sprintX/`
- global status lives only in the top-level docs files
- `.claude/` and `.github/` no longer repeat stable docs and no longer contradict each other
- hardware access, sudo usage, environment handling, and platform split rules are explicit and consistent

