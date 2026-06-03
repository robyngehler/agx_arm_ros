# Overview of different Launch Entry Points

## RViz
Body + right arm + hand
```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false \
  use_left_hand:=false \
  use_right_arm:=true \
  use_right_hand:=true \
  gui:=true \
  use_rviz:=true
```

Body + dual arm no hands
```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=true \
  use_left_hand:=false \
  use_right_arm:=true \
  use_right_hand:=false \
  gui:=true \
  use_rviz:=true
```

Explicit mount offsets
```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false \
  use_left_hand:=false \
  use_right_arm:=true \
  use_right_hand:=true \
  right_arm_base_xyz:='0.0 0.0 0.0' \
  right_arm_base_rpy:='0 0 -1.570796' \
  body_mesh_xyz:='0 0 0' \
  body_mesh_rpy:='0 0 0' \
  gui:=true \
  use_rviz:=true
```

## MoveIt standalone
```bash
ros2 launch agx_arm_moveit start_moveit.launch.py \
  arm_type:=nero \
  moveit_profile:=right_arm \
  robot_name:=duo_nero_system \
  custom_model:=$DUO_MODEL \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=false' \
  use_mit_controller:=false \
  follow:=false \
  use_rviz:=true \
  planning_pipelines:=ompl \
  load_simple_obstacles:=false
```

## MIT-RViz-MoveIt 
Debug Path
```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=debug_soft_target \
  can_port:=can_nero \
  arm_type:=nero \
  custom_model:=$DUO_MODEL \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true' \
  input_joint_prefix:=right_arm_ \
  follow:=true \
  tcp_parent_frame:=right_arm_nero_tool0 \
  tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

One-Arm OMPL profile
```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  can_port:=can_nero \
  arm_type:=nero \
  moveit_profile:=right_arm \
  custom_model:=$DUO_MODEL \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=false' \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```

Duo-Arm OMPL profile
```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  arm_type:=nero \
  moveit_profile:=both_arms \
  custom_model:=$DUO_MODEL \
  custom_model_xacro_args:='use_left_arm:=true use_left_hand:=false use_right_arm:=true use_right_hand:=false' \
  arm_instances:='[{name: left_arm, namespace: left_arm, can_port: can_left, joint_prefix: left_arm_, feedback_joint_prefix: left_arm_, launch_driver: true}, {name: right_arm, namespace: right_arm, can_port: can_right, joint_prefix: right_arm_, feedback_joint_prefix: right_arm_, launch_driver: true}]' \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```