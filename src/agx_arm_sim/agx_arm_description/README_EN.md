# agx_arm_description

English overview for the current Nero-focused description package.

This package is the canonical long-term description source for the repo-owned Nero, Revo2, and OmniHand assets. The active package scope in this workspace is:

- arm type: `nero`
- end effectors: `none`, `agx_gripper`, `revo2`, `omnihand`
- visualization entrypoints: `display.launch.py` and `display_control.launch.py`
- custom-model support for staged Duo bringup through `custom_model` and `custom_model_xacro_args`

Current OmniHand support in this package is limited to description, RViz, and simulation-facing integration. The live ROS bridge and hardware bringup stay in `src/agx_arm_ctrl`.

## Main entrypoints

Package-local visualization:

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=nero
```

Control-compatible visualization:

```bash
ros2 launch agx_arm_description display_control.launch.py \
  arm_type:=nero \
  effector_type:=agx_gripper
```

Staged Duo custom model example:

```bash
ros2 launch agx_arm_description display_control.launch.py \
  custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true'
```

## Cross references

- `../../../../docs/control/bringups/launches.md` for current operational bringup
- `../../../../docs/project/repository_structure.md` for package ownership and staging rules
- `./agx_arm_urdf/README_EN.md` for the lower-level description asset tree

For the full Chinese package documentation, see `README.md` in this same directory.