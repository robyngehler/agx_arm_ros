### Observations While Planning Trajectories between Anchor Poses
#### Not Altering the OmniHand Pose
When Planning without modifying the omnihand pose, every plan succeeds but an error is thrown at plan time:
```bash
[move_group-7] [INFO] [1783080778.075841248] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-7] [INFO] [1783080778.076133408] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-7] [INFO] [1783080778.121341632] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-7] [ERROR] [1783080778.121680576] [moveit_robot_state.conversions]: Found empty JointState message
[move_group-7] [ERROR] [1783080778.121803104] [moveit_robot_state.conversions]: Found empty JointState message
[move_group-7] [INFO] [1783080778.122012096] [moveit_collision_detection_fcl.collision_common]: Found a contact between 'right_ring_dip_link' (type 'Robot link') and 'right_middle_pip_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[move_group-7] [INFO] [1783080778.122034048] [moveit_collision_detection_fcl.collision_common]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[move_group-7] [INFO] [1783080778.122051872] [moveit_ros.fix_start_state_collision]: Start state appears to be in collision with respect to group right_arm
[move_group-7] [WARN] [1783080778.157022432] [moveit_ros.fix_start_state_collision]: Unable to find a valid state nearby the start state (using jiggle fraction of 0.050000 and 100 sampling attempts). Passing the original planning request to the planner.
[move_group-7] [ERROR] [1783080778.157162272] [moveit_robot_state.conversions]: Found empty JointState message
[move_group-7] [ERROR] [1783080778.157494368] [moveit_robot_state.conversions]: Found empty JointState message
[move_group-7] [INFO] [1783080778.157927936] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'right_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.

```
with following commands in parallel:
```bash
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Front_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach

Teach manager keys:
  i -> idle / freedrive (MIT zero-force, gravity-compensated)
  r -> record mode      p -> playback mode      t -> transitions mode
  a -> capture current pose as a named anchor (-> arm_config.yaml)
  w -> convert selected recording -> catalogue waypoints
  [ / ] -> select previous / next item (recording or anchor target)
  s -> status   h -> help   q -> quit
Record mode:   n -> record a new trajectory
Playback mode: f -> play selected   c -> cancel active trajectory
Transitions:  f -> plan selected target, press f again -> execute cached plan, c -> clear cached plan
With two arms, record/anchor ask which resource to save (both_arms -> merged 14-dim, or one side -> 7-dim).

[INFO] [1783080615.698522816] [agx_arm_teach_manager]: Planned transition to Front_R on 'right_arm' (44 point(s), planning_time=0.188s). Press 'f' again to execute.
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Front_R, transition_plan=cached, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[ERROR] [1783080630.758308704] [agx_arm_teach_manager]: executing transition to Front_R failed with status=6, MoveIt error_code=-4
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Idle_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_Working_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Post_Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Post_Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Pre_Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[INFO] [1783080647.353834880] [agx_arm_teach_manager]: Planned transition to Pre_Grip_R on 'right_arm' (39 point(s), planning_time=0.176s). Press 'f' again to execute.
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Pre_Grip_R, transition_plan=cached, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[ERROR] [1783080658.079011872] [agx_arm_teach_manager]: executing transition to Pre_Grip_R failed with status=6, MoveIt error_code=-4
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Pre_Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=grasp_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Front_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Idle_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_Working_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Post_Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Post_Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Pre_Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Pre_Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=grasp_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Front_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[INFO] [1783080677.510528320] [agx_arm_teach_manager]: Planned transition to Grip_R on 'right_arm' (21 point(s), planning_time=0.135s). Press 'f' again to execute.
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=cached, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[INFO] [1783080681.224756160] [agx_arm_teach_manager]: Executed transition to Grip_R via MoveIt
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[WARN] [1783080716.123893792] [agx_arm_teach_manager]: unhandled key 'n state=transitions; press 'h' for help
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Front_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[WARN] [1783080718.749972096] [agx_arm_teach_manager]: unhandled key 'A' in state=transitions; press 'h' for help
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Idle_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Init_Working_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Place_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach

```
-> every transition is ok, the trajectory is performed smooth even though errors appear
-> after the first "f" and planing stage the transition is visible in rviz

#### Altering the OmniHand Pose
When altering the pose with
```bash
(base) user@ubuntu:~$ ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --side right --gesture fist_vendor_demo
[INFO] [1783080900.997985376] [omnihand_exerciser]: Exercising model=o12_pro side=right on topic 'control/joint_states' (12 joints)
[INFO] [1783080900.998913216] [omnihand_exerciser]: -> fist_vendor_demo

```
after f.e. `Grip_R`, the following error will appear when triggering another transition to f.e. `Post_Grip_R`:
```bash
[move_group-7] [INFO] [1783080778.157927936] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'right_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-7] [WARN] [1783080778.158410528] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [WARN] [1783080778.158510048] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.158583680] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.158670432] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.158721472] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.158878944] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.158939008] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [ERROR] [1783080778.159076448] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.159188768] [ompl]: ./src/ompl/tools/multiplan/src/ParallelPlan.cpp:138 - ParallelPlan::solve(): Unable to find solution by any of the threads in 0.001042 seconds
[move_group-7] [WARN] [1783080778.159539392] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [WARN] [1783080778.159580960] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.159629184] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.159739040] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.159763168] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [ERROR] [1783080778.160222464] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.160370560] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.160405888] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.160457984] [ompl]: ./src/ompl/tools/multiplan/src/ParallelPlan.cpp:138 - ParallelPlan::solve(): Unable to find solution by any of the threads in 0.001186 seconds
[move_group-7] [WARN] [1783080778.160651296] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [WARN] [1783080778.160702848] [ompl]: ./src/ompl/base/src/Planner.cpp:248 - right_arm/right_arm: Skipping invalid start state (invalid state)
[move_group-7] [ERROR] [1783080778.160730624] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [ERROR] [1783080778.160800896] [ompl]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:219 - right_arm/right_arm: Motion planning start tree could not be initialized!
[move_group-7] [WARN] [1783080778.160851232] [ompl]: ./src/ompl/tools/multiplan/src/ParallelPlan.cpp:138 - ParallelPlan::solve(): Unable to find solution by any of the threads in 0.000337 seconds
[move_group-7] [WARN] [1783080778.167943936] [ompl]: ./src/ompl/base/goals/src/GoalLazySamples.cpp:129 - Goal sampling thread never did any work.
[move_group-7] [INFO] [1783080778.168143296] [moveit.ompl_planning.model_based_planning_context]: Unable to solve the planning problem
[move_group-7] [INFO] [1783080778.168256128] [moveit_move_group_default_capabilities.move_action_capability]: Catastrophic failure
```
with:
```bash
Status: state=transitions, arms=[arm], recordings=1, selected=Wave_R.json, transition=Post_Grip_R, transition_plan=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[ERROR] [1783080724.061818848] [agx_arm_teach_manager]: planning transition to Post_Grip_R failed with MoveIt error_code=99999
[ERROR] [1783080725.340878112] [agx_arm_teach_manager]: planning transition to Post_Grip_R failed with MoveIt error_code=99999
[ERROR] [1783080754.574013152] [agx_arm_teach_manager]: planning transition to Post_Grip_R failed with MoveIt error_code=99999
```
-> no transition is shown in rviz
-> no movement possible

This behavior is reproducable every time the `Omnihand Pose` is altered.

---

## Root-cause analysis & fixes (2026-07-03)

Reproduced offline (no hardware) with pinocchio + FCL against the exact artifacts move_group uses
(right-slice `duo_system` URDF + generated `agx_arm.srdf.xacro`, SRDF-disabled pairs removed,
`fist_vendor_demo` clamped to the o12_pro limits, mimic coupling applied):

- **open hand (zeros): 0 colliding pairs**
- **fist: 5 colliding pairs** — `thumb_dip x middle_pip`, `palm x index_dip/index_tip`,
  `palm x middle_dip/middle_tip` (the live log's `ring_dip x middle_pip` is the same class with the
  real readback pose instead of the clamped command estimate)

**RC1 — `Found empty JointState message` (harmless but noisy, both cases).** The teach manager's
transition mode built the `MotionPlanRequest` without a `start_state`; the default is an *empty,
non-diff* `RobotState`, so move_group logs the conversion error (twice) and then falls back to the
monitored current state — which is why planning still worked. **Fix:** `request.start_state.is_diff =
True` in `plan_selected_transition` (plan from the monitored state, no fallback path, no error spam).

**RC2 — `Skipping invalid start state` / error 99999 after altering the hand pose.** The planning-scene
start state includes the *real* hand joints (merged combined feedback). The SRDF only disabled
*within-finger adjacent* pairs, no cross-finger and no palm-fingertip pairs — but a fist puts the
fingertips on the palm and the thumb on the fingers **by design**. The model then reports the whole
robot start state as in collision, and OMPL refuses to plan the **arm** group (`Motion planning start
tree could not be initialized` -> `Catastrophic failure`, error 99999). In the open-hand case the same
mechanism produced the `fix_start_state_collision` warnings whenever the hand had not fully returned
to open (residual pose from earlier gestures). **Fix:** the SRDF omnihand macro now disables **all
intra-hand collision pairs** (325 generated pairs incl. currently collision-less links; hand-vs-arm
and hand-vs-body pairs stay ACTIVE). Rationale: the hand joints are never planned by the arm groups
(their values come from hardware readback), grasp self-contact is intended, and the vendor meshes +
approximate mimic coupling interpenetrate at legitimate poses anyway — intra-hand FCL checks can only
veto arm planning, never inform it. Verified offline: fist pose -> 0 colliding pairs, 269 (arm/body)
pairs remain active; left and dual-arm SRDF variants still generate.

**RC3 (separate, not yet fixed) — `executing ... failed with status=6, MoveIt error_code=-4`
(CONTROL_FAILED) while the motion visibly completes smoothly.** Most likely the MIT controller's goal
tolerance check: after `duration + goal_time_tolerance_s` (0.5 s) it aborts with
GOAL_TOLERANCE_VIOLATED when the final point is not within `goal_position_tolerance` (0.05 rad
default) — with soft playback gains under gravity load a steady-state error > 0.05 rad on one joint is
enough, and MoveIt maps the controller abort to CONTROL_FAILED even though the arm is essentially at
the target. Diagnosis on next run: check the mit_controller log for GOAL_TOLERANCE_VIOLATED at the
failure timestamp and compare `~/reference_joint_states` vs `feedback/joint_states` at the end of the
motion. The now-correct hand-aware gravity model (see `hand_recordings_2026-07-02.md` P1'') should already
shrink the steady-state error — retest before touching tolerances; if it persists, raise
`goal_position_tolerance` / `goal_time_tolerance_s` for the moveit_mit profile or stiffen the hold
gains.

Touched: `src/agx_arm_mit_demos/agx_arm_mit_demos/teach_manager.py` (is_diff),
`src/agx_arm_moveit/config/agx_arm.srdf.xacro` (intra-hand ACM block). Hardware validation pending:
plan a transition after `fist_vendor_demo` — planning should now succeed without OMPL start-state
warnings.