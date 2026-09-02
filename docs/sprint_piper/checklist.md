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
