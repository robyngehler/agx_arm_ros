# Sprint 2 Target

status: HISTORICAL_ENTRYPOINT
last_updated: 2026-07-18

Sprint 2 was the contract-hardening phase for the current arm-plus-hand baseline.

## Main goal

Establish the first stable repo-owned runtime contract for:

- shared arm control surfaces
- integrated MIT execution
- first launch and runtime visibility
- the first repo-owned OmniHand bridge direction

## What Sprint 2 settled

- the public ROS contract stayed agx_arm-centric
- the OmniHand bridge stayed in `agx_arm_ctrl`
- OmniHand-specific messages stayed in `agx_arm_msgs`
- the shared runtime used `control/joint_states`, combined `feedback/joint_states`, and
	hand-only diagnostics under `feedback/omnihand/*`
- the first real below-ROS OmniHand CAN FD investigation stayed outside the ROS launch path until
	transport behavior was better understood

## Stable outputs promoted elsewhere

- `docs/project/repository_structure.md`
- `docs/project/architecture.md`
- `docs/assets/omnihand/omnihand_ros_integration_options.md`
- `docs/assets/omnihand/omnihand_wrapper_integration_plan.md`

## Historical evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../evidence/mit_runtime_history.md`
- `../evidence/omnihand_canfd_transport_history.md`