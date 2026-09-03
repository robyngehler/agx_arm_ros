# Sprint Piper — AGX gripper integration status

Target: bring the AGX parallel gripper from the driver/MoveIt layer up through the
Activity coordinator and the teach manager, without a third execution path and
without weakening the authority model.

Source proposal: `planning/Piper Gripper Pipeline and Teach Manager Integration Proposal.md`.
Naming: the code calls this device `agx_gripper` (AgileX parallel jaw), not "Piper" —
`Piper` is the vendor's arm product and appears nowhere in the ROS surface.

## Done

| Area | State |
| --- | --- |
| Description, gravity, MoveIt end effector (both sides) | landed (`b3baa1e`, `8ad62f4`) |
| Device authority + `FollowJointTrajectory` server | landed (`e2e85b1`) |
| Normalized `closure` [0,1] and its single conversion | landed — `agx_arm_coordination/gripper_closure.py`, endpoints in `duo_motion_registry.yaml` |
| FJT cancellation reaches the device | landed — cancel is polled during the wait and calls `control/gripper/stop` |
| Progress-aware completion | landed — arrived / settled-after-travel / no_progress / faulted / stale / moving |
| `left_gripper` / `right_gripper` as coordinator resources | landed — both topology tables |
| Coordinator gripper execution through FJT | landed — `_dispatch_gripper`, `_GripperChild` |
| Catalogue gripper actions | landed — presets plus any closure in [0,1] |
| Teach manager gripper mode (`e`) | landed — normalized in and out, FJT transport |
| Legacy `control/joint_states` gripper ingress gated | landed — `allow_legacy_gripper_command_ingress`, default false |

## Not done

| Item | Why |
| --- | --- |
| **Hardware validation** | Nothing below has been exercised on the real gripper. See the gate below. |
| Event-based gripper recording (proposal §8.2) | Deferred until manual control is validated on hardware. The recorder trims to the first arm motion (`_trim_pre_motion`), so events would rebase onto that same origin. |
| Gripper events in playback (§9) | Follows the recording format. Uniform time scaling is only correct for `as_recorded`/`smooth`/`tempo_scale`; TOTG retiming needs path-progress binding, not timestamp scaling. |
| Recording → catalogue conversion (§10) | Follows §8.2. |
| Relaxing the arm↔gripper resource conflict | The gripper holds its arm's bus token in **both** topology tables, so it is serialized against its own arm. That is physically true, not merely conservative: it rides the arm's CAN bus and the arm's SDK session by vendor design. Concurrent arm motion + gripper command has never been run. |

## Found in review 2026-09-03, not fixed

Code review against the proposal. None of these is a regression; they are what
the pipeline does not yet cover.

| Item | What happens |
| --- | --- |
| Two concurrent FJT goals on one gripper are neither refused nor serialized | `_goal_callback` always accepts and the callback group is reentrant, so two `_execute_callback` run at once. Both claim under the same `owner_id`, which `DeviceAuthority.claim` grants without bumping the epoch while `_call_claim` resets the sequence — so the second goal's command is refused as a stale sequence and reported as `no_progress`, and whichever goal ends first releases the claim under the other. Three clients share this server. The OmniHand FJT bridge has the same shape. |
| `emergency_stop` does not touch the gripper | It faults the arm's authority and holds the arm; `_gripper_authority` never enters FAULTED and nothing calls the gripper's cancel-and-hold. The jaws keep driving the last commanded width until the unit-safety broadcast arrives, which blocks the next command rather than the running one. |
| The claim/stop/release lifecycle is untested | The ten completion tests drive `_await_outcome` only. Nothing asserts that a cancel calls `control/gripper/stop` before the goal ends, or that the claim is released on the abort path. Testable without hardware. |
| The teach manager cannot cancel a gripper goal | `command_gripper_closure` blocks in `spin_until_future_complete`, so no key is read during travel. Gate item 4 below needs the coordinator or a manual action client; say which. |
| A short contact grasp inside the tolerance band fails | One tolerance carries both halves, so a command whose gap is just over `progress_tolerance_m`, stopped by an object after less than that, reports `no_progress`. At 2 mm tolerance: 3 mm commanded, object at 1.5 mm travel. Belongs to gate item 5. |
| `homing_status` is not a health check | `_health_refusal` reads the fault bits and the enable bit. An unreferenced gripper's width is not a trustworthy measurement to judge progress against. |
| The completion thresholds have no launch argument | The node is launched with `action_name` only. Calibrating `progress_tolerance_m`, `settle_epsilon_m`, `settle_time_s` during gate items 2-3 currently means editing the launch file. |
| Force is a node-level default, not a per-action property | `gripper_default_effort` is applied to every command, so `right_gripper_grasp` squeezes with the same force an open command uses. The proposal allows this (§1 keeps force "a separate optional configuration"), but a catalogue action cannot request one. |
| The gripper is absent from the agent rule layer | `AGENTS.md`, `CLAUDE.md`, `.claude/rules/` and `.github/` do not mention it. The one-owner-per-SDK-session rule and the legacy-ingress rule both name arms and hands only. |
| `docs/sprint_piper/` is not registered globally | Missing from `docs/README.md`, `docs/checklist.md`, and the sprint entrypoint lists in `CLAUDE.md` and `.claude/rules/context-routing.md`. The open questions below are not propagated to `docs/open_questions.md`. |

Pre-existing on this branch, unrelated to the gripper but currently red:
`test_tea_pour_duo_v2.py` fails two cases (dispatch batch index, `tempo_scale`
against an expected `smooth`) — also red at `8ad62f4`, so it came in with the
merged tea-demo actions. `agx_arm_ctrl` has two order-dependent failures when the
whole suite runs in one process (`test_omnihand_bridge_retry`,
`test_omnihand_stop_semantics`), both green in isolation. `agx_arm_coordination`
flake8/pep257 fail on `arm_executor.py`, from `f3ef46d`.

## Hardware validation gate (not started)

In order, on the right gripper:

1. readback only — `feedback/gripper_status` live, closure display sane
2. teach manager `closure=0.0`, then `1.0`, then intermediates
3. repeated open/close
4. cancel during travel — jaws must stop, goal must report CANCELED
5. object between the jaws — must succeed as a contact grasp, residual gap reported
6. **block the jaws so nothing moves — must fail `no_progress`, not succeed**
7. coordinator standalone gripper action
8. activity: arm motion → grip → arm motion → release
9. Ctrl-C during gripper execution
10. authority/epoch invalidation

Item 6 is the one that decides whether the completion rule works. Items 2-3 are
also what calibrates `progress_tolerance_m` (2 mm), `settle_epsilon_m` (0.5 mm)
and `settle_time_s` (0.15 s) — all three are currently chosen, not measured.

## Open questions

- **Is `GripperStatus.force` measured or echoed?** The status frame carries a force
  field the driver documents as "current gripping force". It read 0.5 N with ±0.01 N
  fluctuation against a commanded 0.5 N, which is consistent with either. If it is
  measured, it is a second and better contact signal than width standstill. Decide by
  squeezing an object and watching whether it rises. Until then the completion rule
  does not use it, and only reports it.
- **The ~2 s periodic joint7 correction under MIT hold** (carried over, deferred by
  the user). Ruled out: payload/gravity, mode contention, CAN ID collision, external
  watchdog, `shared_can_recovery`, `duo_soft_estop`, `follow:=true`. Decisive test:
  `candump can_nero_right,160:7FF` during reproduction.
