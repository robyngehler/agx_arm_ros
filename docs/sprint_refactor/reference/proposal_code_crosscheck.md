# Proposal Code Cross-Check
status: INITIAL_READ_ONLY_AUDIT
date: 2026-07-27
branch: ROS2_Duo_System_V02

This note cross-checks the refactor proposal against the current codebase. The
goal is not to re-audit every module from scratch; it is to confirm whether the
proposal's controlling assumptions still match the implementation that will be
migrated on V02.

Validation boundary for this note:

- read-only workspace inspection only
- no live hardware access
- no runtime CPU or CAN measurements captured in this session

## Confirmed findings

| Proposal area | Status | Current evidence | Planning consequence |
| --- | --- | --- | --- |
| Velocity truth and stop verification | Confirmed | `vendor/pyAgxArm/.../driver.py` still forces `motor_state.msg.velocity = 0.0`; `agx_arm_ctrl_single_node.py` still uses motor velocity in `_arm_velocities_settled()` | Phase 0 must start with honest velocity or derived velocity plus honest stop semantics |
| SDK ownership and TOCTOU risk | Confirmed | mode and CAN-push helpers still mutate shared SDK state outside one serialized hardware-owner path | Side authority plus worker/queue stays the correct first runtime refactor |
| Hand handover contract is only partial | Confirmed | `prepare_hand_window` and `resume_arm_control` exist, but they only expose Trigger success/failure rather than a lease identity plus epoch | Leases should replace or wrap the current handover services before coordinator assumptions get stronger |
| Hand control continues after grasp completion | Confirmed | `omnihand_skill_controller_node.py` still runs `_hold_tick()` and republishes the grasp target while holding internally | Phase 2 must remove recurring post-success hand traffic before same-side arm motion is considered safe |
| Bridge polling is not ownership-aware | Confirmed | `omnihand_bridge_node.py` still carries main publication, joint-read, status/tactile caching, and retry timing independent of lease ownership | Bridge gating and timer split belong in the core migration, not as a later optimization |
| Coordinator is poll-driven and not globally exclusive | Confirmed | `coordinator_node.py` still accepts every goal and uses a 20 Hz polling loop with a `ReentrantCallbackGroup` | Phase 3 needs a unit activity lock and event-driven child completion |
| MIT consumes partial driver state | Confirmed | current hand-window boolean is narrower than the proposal's authoritative side state and epoch contract | MIT should not be left on boolean state once Phase 1 begins |
| Registry/profile duplication remains | Confirmed | `duo_motion_registry.yaml` is the documented source of truth, but `execution_profiles.yaml` still repeats per-side runtime instance details | The integration plan should extend existing resolver code instead of introducing another config layer |

## Nuances and current implementation details

- The current `prepare_hand_window` path already verifies arm settle, control
  mode, and firmware-hold readback before returning success. The remaining gap
  is not a total lack of verification; it is that the exported contract is still
  only a coarse Trigger response plus a boolean hand-window state.
- The active Duo MoveIt path is already generated rather than purely static:
  `src/agx_arm_moveit/launch/_moveit_config_builder.py` builds the
  `moveit_simple_controller_manager.controller_names` map from `arm_instances`.
  For `duo_hand`, it also registers per-side OmniHand FJT controllers when the
  profile carries OmniHands.
- The unprefixed standalone `moveit_controllers.yaml` was found only in the
  legacy standalone package
  `src/agx_arm_sim/Moveit2/nero_gripper_moveit_config/`. No active Duo runtime
  reference to that file was found during this audit.

## Current implementation entry points

- `vendor/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/nero_can_push.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_skill_controller_node.py`
- `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py`
- `src/agx_arm_ctrl/config/execution_profiles.yaml`
- `src/agx_arm_coordination/agx_arm_coordination/coordinator_node.py`
- `src/agx_arm_mit_controller/.../mit_controller_node.py`
- `src/agx_arm_moveit/launch/_moveit_config_builder.py`
- `src/agx_arm_sim/agx_arm_description/config/duo_motion_registry.yaml`

## Use in the migration plan

- Treat the proposal as structurally current, not historical.
- Start with the release blocker and ownership contract rather than with config
  cleanup or performance tuning.
- Reuse the existing registry-resolution and generated MoveIt-controller path;
  harden and complete it instead of inventing a second configuration system.
- Keep live-hardware validation as an explicit phase gate because the remaining
  unanswered questions are timing and bus-behavior questions, not naming
  questions.