# Historical Note: MIT Hold, Replay, And Wakeword Runtime Lineage

Status: historical Sprint 2 summary only.

Do not use this file as a runnable control guide. The canonical operational workflow lives in
`../../control/bringups/launches.md` and `../../control/teach_and_run.md`.

## Why this file still exists

This note keeps the design rationale and the final lessons from the earlier MIT hold, recording,
replay, and wakeword-demo slice without duplicating the current control instructions.

## Historical outcome

Sprint 2 established the controller-side rules that later became the stable teach and replay flow:

1. validate gravity and MIT hold before any trajectory replay
2. keep controller-owned feedforward in the MIT controller instead of replaying recorded torques
3. switch back to Normal Mode before enabling MIT playback
4. load gains and gravity settings from a startup YAML through `params_file`
5. keep the wakeword listener outside `agx_arm_ros` and keep the ROS-side application logic in the
   demo package instead of the controller runtime package

## Stable conclusions that were promoted

- `agx_arm_test_position_hold` became the authoritative static gravity and hold check before replay
- trajectory playback uses recorded positions and velocities, while gravity and feedforward stay in
  the running MIT controller configuration
- the repo's packaged MIT profiles and default parameter discovery became the current startup path
- the teach and wakeword lineage stayed in `agx_arm_mit_demos`, not in `agx_arm_mit_controller`

## Historical components produced by this slice

- `start_nero_mit_controller.launch.py`
- `agx_arm_test_position_hold`
- `agx_arm_record_leader_trajectory`
- `agx_arm_execute_saved_trajectory`
- `agx_arm_wakeword_motion_manager`
- packaged parameter profiles such as `nero_mit_controller_defaults.yaml` and `mit_playback_soft.yaml`

## Remaining historical value

Keep this note only for:

- rationale behind the static-hold-first workflow
- the controller-versus-recording feedforward split
- the wakeword-demo lineage around the MIT demo package

If that rationale is fully captured elsewhere later, this file can be deleted too.