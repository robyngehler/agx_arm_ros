# Historical Proposal: Stage 2 Integrated MIT `FollowJointTrajectory` Controller

Status: historical design note.

Do not use this file as an implementation guide. The integrated MIT action-server path is already
part of the current runtime, and the stable operational workflow now lives in
`../../control/bringups/launches.md` and `../../control/bringups/teach_and_run.md`.

## Original goal

Replace the transitional bridge stack with a single repo-owned ROS-native trajectory execution path:

```text
MoveIt or RViz
  -> FollowJointTrajectory action
  -> MIT controller
  -> control/move_mit
  -> agx_arm_ctrl_single_node
  -> pyAgxArm and CAN
```

## Design conclusions that survived

- the MIT controller should be the official MoveIt trajectory execution endpoint
- `agx_arm_ctrl_single_node` should remain the low-level hardware gateway
- the temporary action-bridge layer was a transitional surface, not the long-term execution contract
- debug-only soft-target paths should stay clearly separate from production MoveIt execution

## What later became stable repo behavior

- the integrated MIT action-server path became part of `src/agx_arm_mit_controller`
- `moveit_mit` became the preferred execution family in the active launch surface
- the debug `~/joint_trajectory` input stayed opt-in rather than a public production control path

## Remaining historical value

Keep this note only for the architectural reasoning behind:

- why the MIT controller, not a temporary bridge, owns the trajectory execution contract
- why the hardware gateway stayed in `agx_arm_ctrl`
- why debug soft-target routing stayed separate from the stable MoveIt execution path