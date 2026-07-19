#### Log of taught poses / trajectories:
```bash
(base) user@ubuntu:~$ ros2 run agx_arm_mit_demos agx_arm_teach_manager   --arm-config src/agx_arm_coordination/config/arm_config.yaml   --source-joints joint1,joint2,joint3,joint4,joint5,joint6,joint7
[WARN] [1783071598.484279616] [agx_arm_teach_manager]: Waiting for the arm MIT services: set_normal_mode, mit_controller/enable, mit_controller/freedrive, mit_controller/hold_current.
The teach manager does not start the arm — bring it up first, e.g.:
  ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py can_port:=can_nero_right
(no input_joint_prefix for the teach loop; use --source-joints joint1,...,joint7)
[INFO] [1783071599.526437280] [agx_arm_teach_manager]: State -> idle (gravity-compensated freedrive)
Status: state=idle, arms=[arm], recordings=0, selected=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach

Teach manager keys:
  i -> idle / freedrive (MIT zero-force, gravity-compensated)
  r -> record mode      p -> playback mode
  a -> capture current pose as a named anchor (-> arm_config.yaml)
  w -> convert selected recording -> catalogue waypoints
  [ / ] -> select previous / next recording
  s -> status   h -> help   q -> quit
Record mode:   n -> record a new trajectory
Playback mode: f -> play selected   c -> cancel active trajectory
With two arms, record/anchor ask which resource to save (both_arms -> merged 14-dim, or one side -> 7-dim).

Anchor pose name (e.g. Pre_Grip_R): Init_R 
[INFO] [1783071708.884108512] [agx_arm_teach_manager]: inserted new pose 'Init_R': Init_R (nero, 7-dim) = [0.84774, 0.51608, 0.13842, 1.88497, 0.18368, -0.03299, 0.15038]
[INFO] [1783071708.885498080] [agx_arm_teach_manager]: rebuild agx_arm_coordination (or symlink-install) for a launched coordinator
[WARN] [1783071712.379799456] [agx_arm_teach_manager]: unhandled key 'n state=idle; press 'h' for help
[WARN] [1783071742.297280320] [agx_arm_teach_manager]: unhandled key 'A' in state=idle; press 'h' for help
Anchor pose name (e.g. Pre_Grip_R): Pre_Grip_R
[INFO] [1783071749.422733824] [agx_arm_teach_manager]: updated existing pose 'Pre_Grip_R': Pre_Grip_R (nero, 7-dim) = [1.11844, 1.08135, 0.34961, 1.88480, 0.66729, -0.03252, 0.18432]
[INFO] [1783071749.424182144] [agx_arm_teach_manager]: rebuild agx_arm_coordination (or symlink-install) for a launched coordinator
[WARN] [1783071751.591674944] [agx_arm_teach_manager]: unhandled key 'n state=idle; press 'h' for help
[WARN] [1783071765.638387072] [agx_arm_teach_manager]: unhandled key 'A' in state=idle; press 'h' for help
Anchor pose name (e.g. Pre_Grip_R): Grip_R
[INFO] [1783071774.320187616] [agx_arm_teach_manager]: updated existing pose 'Grip_R': Grip_R (nero, 7-dim) = [1.21814, 1.28083, 0.36266, 1.80392, 1.09280, 0.21433, 0.20075]
[INFO] [1783071774.321832672] [agx_arm_teach_manager]: rebuild agx_arm_coordination (or symlink-install) for a launched coordinator
Anchor pose name (e.g. Pre_Grip_R): Post_Grip_R
[INFO] [1783071817.665113184] [agx_arm_teach_manager]: updated existing pose 'Post_Grip_R': Post_Grip_R (nero, 7-dim) = [1.28006, 1.35319, 0.38338, 1.46512, 1.03968, 0.29060, 0.21427]
[INFO] [1783071817.666462592] [agx_arm_teach_manager]: rebuild agx_arm_coordination (or symlink-install) for a launched coordinator
Anchor pose name (e.g. Pre_Grip_R): Place_R
[INFO] [1783071839.859946496] [agx_arm_teach_manager]: inserted new pose 'Place_R': Place_R (nero, 7-dim) = [1.21335, 1.36619, 0.38333, 1.73549, 0.95110, 0.16132, 0.18221]
[INFO] [1783071839.860704256] [agx_arm_teach_manager]: rebuild agx_arm_coordination (or symlink-install) for a launched coordinator

Teach manager keys:
  i -> idle / freedrive (MIT zero-force, gravity-compensated)
  r -> record mode      p -> playback mode
  a -> capture current pose as a named anchor (-> arm_config.yaml)
  w -> convert selected recording -> catalogue waypoints
  [ / ] -> select previous / next recording
  s -> status   h -> help   q -> quit
Record mode:   n -> record a new trajectory
Playback mode: f -> play selected   c -> cancel active trajectory
With two arms, record/anchor ask which resource to save (both_arms -> merged 14-dim, or one side -> 7-dim).

[INFO] [1783071889.411711680] [agx_arm_teach_manager]: State -> record (freedrive; press 'n' to record)
Status: state=record, arms=[arm], recordings=0, selected=<none>, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Trajectory name [teach_20260703_114457]: Wave_R
[INFO] [1783071927.292130432] [agx_arm_teach_manager]: Recording 'Wave_R' as 'nero' — move the arm(s); auto-stops after hold timeout
[INFO] [1783071938.212555392] [agx_arm_teach_manager]: Saved /home/user/agx_arm_trajectories/teach/Wave_R.json (7-dim, resource=nero)
Status: state=record, arms=[arm], recordings=1, selected=Wave_R.json, arm_config=set, library=/home/user/agx_arm_trajectories/teach
Trajectory name [teach_20260703_114612]: Wave_R
[INFO] [1783071977.973047040] [agx_arm_teach_manager]: Recording 'Wave_R' as 'nero' — move the arm(s); auto-stops after hold timeout
[INFO] [1783071987.423284992] [agx_arm_teach_manager]: Saved /home/user/agx_arm_trajectories/teach/Wave_R.json (7-dim, resource=nero)
Status: state=record, arms=[arm], recordings=1, selected=Wave_R.json, arm_config=set, library=/home/user/agx_arm_trajectories/teach
[INFO] [1783071995.219545376] [agx_arm_teach_manager]: State -> playback (MIT on, holding current)
Status: state=playback, arms=[arm], recordings=1, selected=Wave_R.json, arm_config=set, library=/home/user/agx_arm_trajectories/teach

Teach manager keys:
  i -> idle / freedrive (MIT zero-force, gravity-compensated)
  r -> record mode      p -> playback mode
  a -> capture current pose as a named anchor (-> arm_config.yaml)
  w -> convert selected recording -> catalogue waypoints
  [ / ] -> select previous / next recording
  s -> status   h -> help   q -> quit
Record mode:   n -> record a new trajectory
Playback mode: f -> play selected   c -> cancel active trajectory
With two arms, record/anchor ask which resource to save (both_arms -> merged 14-dim, or one side -> 7-dim).

[INFO] [1783072032.461411584] [agx_arm_teach_manager]: Played Wave_R.json on ['arm'] (needs enable_debug_joint_trajectory_topic:=true on the MIT bring-up)
Status: state=playback, arms=[arm], recordings=1, selected=Wave_R.json, arm_config=set, library=/home/user/agx_arm_trajectories/teach
```