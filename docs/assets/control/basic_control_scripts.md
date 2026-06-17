# Basic Control Scripts

These are the current entry points after the launch-surface simplification.

For day-to-day work, prefer the `agx_arm_ctrl` wrapper launches plus `execution_profile` over manually wiring `custom_model`, prefixes, and Duo xacro arguments each time.

Recommended test order:

1. validate the real OmniHand below ROS on the CAN FD interface,
2. validate the Duo model in RViz,
3. validate single-arm debug or MoveIt with the simplified wrappers,
4. only then widen to the staged right-hand or dual-arm paths.

## Quick conventions

- `execution_profile:=right_arm` resolves the staged Duo right-arm model and `right_arm_` prefixes.
- `execution_profile:=right_hand` resolves the staged Duo right-arm plus right-OmniHand model and bridge defaults.
- `execution_profile:=duo_arm` resolves the staged Duo dual-arm planning model.
- `can_port` selects the arm's CAN interface — the standard is `can_nero_right` (`can0`) and
  `can_nero_left` (`can1`).
- the OmniHand shares its side bus with the arm and is checked below ROS via the vendor SDK and
  `OMNIHAND_SOCKETCAN_IFACE` (same interface, e.g. `can_nero_right`).

## 0. CAN pre-configure (do this first, before any launch)

**Standard:** the Duo arms and OmniHands run on the Jetson **native `mttcan`** CAN FD side buses
(40-pin header, 5 Mbit BRS-capable transceiver). One command brings both sides up:

```bash
sudo bash scripts/activate_native_can.sh          # both sides
# or: sudo bash scripts/activate_native_can.sh right   (right side only)
```

This creates `can_nero_right` (`can0`) and `can_nero_left` (`can1`) in CAN FD mode
(1M/5M, 0.8 sample points, `one-shot on`, `restart-ms 100`). Each side bus carries the arm
(classic frames) and its OmniHand (FD+BRS frames). Verify:

```bash
ip -details link show can_nero_right    # expect: mtu 72, fd, one-shot, bitrate 1000000, dbitrate 5000000
```

Standard and rationale: `../../development/sprint5/planning/can_transport_decision.md` and
`../omnihand/omnihand_canfd_setup.md`.

## Scripts at a glance

| Script | Use it for | Standard? |
|---|---|---|
| `scripts/activate_native_can.sh` | bring up native side buses (arm + hand per side) | **yes — default** |
| `scripts/omnihand_canfd_activate.sh` | a separate **USB** CAN FD adapter for the hand | only if not on the native bus |
| `scripts/prepare_can_interfaces.py` + `config/can_interface_roles.json` | role-based bring-up of **USB** CAN adapters | legacy/USB |
| `scripts/can_activate.sh` | single **USB** classic-CAN adapter, manual | legacy/USB |
| `scripts/find_all_can_port.sh` | list CAN interfaces and their USB bus-info | helper |
| `scripts/colcon_build_system_python.sh` | build ROS with system Python (no conda) | build |

## 1. OmniHand hardware preflight on the real CAN FD path

Use this first for all remaining live-hand tests.

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
python3.10 python/example/demo_get_hardware_info.py
```

If your hand is on a different CAN FD netdevice, replace `can0` with that interface.

This remains the lowest-risk hardware proof point before ROS-side tests.

The repo-owned bridge now also has a first `omnihand_backend_type:=sdk` path with active 10-joint command support: it publishes hand joint states, status, and tactile data while accepting active joint targets on the repo-owned ROS surfaces.

## 2. RViz and model-only validation

Body + right arm + right hand:

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false \
  use_left_hand:=false \
  use_right_arm:=true \
  use_right_hand:=true \
  gui:=true \
  use_rviz:=true
```

Body + both arms, no hands:

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=true \
  use_left_hand:=false \
  use_right_arm:=true \
  use_right_hand:=false \
  gui:=true \
  use_rviz:=true
```

Explicit mount offsets:

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

## 3. Single-arm debug in RViz with MIT soft targets

Current canonical debug path for the mounted right-arm setup:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=debug_soft_target \
  execution_profile:=right_arm \
  can_port:=can_nero_right \
  follow:=true \
  tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

Staged right-arm plus OmniHand debug path:

```bash
PYTHONPATH=$HOME/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg:$PYTHONPATH \
LD_LIBRARY_PATH=$HOME/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=debug_soft_target \
  execution_profile:=right_hand \
  can_port:=can_nero_right \
  follow:=true \
  omnihand_backend_type:=sdk \
  omnihand_device_id:=1 \
  omnihand_canfd_id:=0 \
  tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

Use the `right_hand` profile when you want the current staged Duo arm-plus-hand model and prefixed follow path without manually passing `custom_model` and `input_joint_prefix`.

At the current stage, this bringup should produce `feedback/omnihand/joint_states`, `feedback/omnihand/status`, and `feedback/omnihand/tactile_raw`. In the hand-aware debug path, the RViz soft-target JointState stream now reaches the OmniHand bridge as well, so the staged sliders and the SDK-backed hand use the same 10 active joint names.

## 4. Current one-command MoveIt plus MIT bringup

Right-arm OMPL profile:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=right_arm \
  can_port:=can_nero_right \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```

Staged right-arm plus OmniHand profile:

```bash
PYTHONPATH=$HOME/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg:$PYTHONPATH \
LD_LIBRARY_PATH=$HOME/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=right_hand \
  can_port:=can_nero_right \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl \
  omnihand_backend_type:=sdk \
  omnihand_device_id:=1 \
  omnihand_canfd_id:=0
```

The `sdk` bridge backend now actuates the hand through the vendor `set_all_active_joint_angles(...)` path and reads back the same 10 active joints for RViz and MoveIt. Use `execution_profile:=right_hand` when you want the staged Duo arm-plus-hand model and actual hand control together; `right_arm` remains the arm-only profile.

If you want the direct wrapper instead of the mode multiplexer, the equivalent simplified MoveIt launch is:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_moveit.launch.py \
  execution_profile:=right_arm \
  can_port:=can_nero_right \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```

## 5. Duo-arm planning and staged dual-arm runtime

Simplest Duo planning entry point:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=duo_arm \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```

The `duo_arm` profile is the simplest starting point for both-arms planning. By default it keeps `arm_instances` in planning-only mode with `launch_driver: false`.

For staged live dual-arm runtime, promote the same launch to managed drivers by overriding `arm_instances`:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py \
  mode:=moveit_mit \
  execution_profile:=duo_arm \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl \
  arm_instances:='[{name: left_arm, namespace: left_arm, can_port: can_nero_left, joint_prefix: left_arm_, feedback_joint_prefix: left_arm_, launch_driver: true}, {name: right_arm, namespace: right_arm, can_port: can_nero_right, joint_prefix: right_arm_, feedback_joint_prefix: right_arm_, launch_driver: true}]'
```

## 6. When you need explicit overrides instead of profiles

Use the lower-level wrappers only when the preset profiles are not enough.

Direct RViz debug wrapper with an explicit custom Duo model slice:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
  can_port:=can_nero_right \
  custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=true' \
  input_joint_prefix:=right_arm_ \
  follow:=true \
  control:=true \
  use_mit_controller:=true \
  tcp_offset:='[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

Direct MoveIt wrapper with explicit custom-model control:

```bash
ros2 launch agx_arm_ctrl start_agx_arm_moveit.launch.py \
  can_port:=can_nero_right \
  moveit_profile:=right_arm \
  robot_name:=duo_nero_system \
  custom_model:=/home/user/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro \
  custom_model_xacro_args:='use_left_arm:=false use_left_hand:=false use_right_arm:=true use_right_hand:=false' \
  input_joint_prefix:=right_arm_ \
  follow:=true \
  use_rviz:=true \
  planning_pipelines:=ompl
```