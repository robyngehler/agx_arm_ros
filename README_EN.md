# AgileX Robotic Arm ROS2 Driver

[中文](./README.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## Overview

This workspace is currently focused on the Nero stack and provides the matching ROS2 control, description, MoveIt2, and MIT soft-control surfaces.

| Description | Documentation |
|---|---|
| SDK | [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) |
| CAN module usage | [can_user](./docs/CAN_USER_EN.md) |
| TCP Offset Configuration | [tcp_offset](./docs/assets/tcp_offset/TCP_OFFSET_EN.md) |
| URDF | [agx_arm_description](./src/agx_arm_sim/agx_arm_description/README.md) |
| Moveit| [Moveit](./src/agx_arm_moveit/README_EN.md) |
| Nero MIT soft control | [agx_arm_mit_controller](./src/agx_arm_mit_controller/README.md) |

---

## Quick Start

### 1. Install Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
```

Choose the installation command based on your ROS version:

**Jazzy** installation command:

```bash
pip3 install . --break-system-packages
```

**Humble** installation command:

```bash
pip3 install .
```

### 2. Install ROS2 Driver

1. Create workspace

    ```bash
    mkdir -p ~/agx_arm_ws/src
    cd ~/agx_arm_ws/src
    ```

2. Clone repository

    ```bash
    git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
    ```

    ```bash
    cd agx_arm_ros/
    git submodule update --remote --recursive
    ```

    > The only remaining submodule is `vendor/OmniHand-Pro-2025`; Nero/Revo2 description assets are committed directly in this repository.

### 3. Install Dependencies

Install the system-side dependencies first. The repo script now stays on apt and ROS packages only, so `colcon build` does not drift into whichever Python environment happens to be active.

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts/
chmod +x agx_arm_install_deps.sh
bash ./agx_arm_install_deps.sh
```

If you also want one reproducible Conda environment for runtime and Python-side development, create the repo-owned environment afterwards:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/
bash ./scripts/setup_agx_arm_runtime_env.sh
```

For more detail, see [Python environment workflow](./docs/project/python_environment_workflow.md).

### 4. Build and Source Workspace

Use the repo build wrapper for workspace builds. It strips Conda/Miniforge paths before calling `colcon build` so the ROS overlay is generated with system Python:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/colcon_build_system_python.sh
source install/setup.bash
```

When you want to run ROS commands inside the optional Conda environment, prefer the repo wrapper instead of manually mixing `conda activate` and `source install/setup.bash`:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
bash ./scripts/run_in_ros_conda.sh -- ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=moveit_mit execution_profile:=right_hand
```

If you need a separate TRAC-IK overlay, export `AGX_ARM_TRAC_IK_OVERLAY=/path/to/install/setup.bash` before running the build or runtime wrappers above.

---

## Usage

### Activate CAN Module

CAN module must be activated before use. For details, see: [CAN Configuration Guide](./docs/CAN_USER_EN.md)

The recommended path is the repo-owned role-based preparation script. It reads [config/can_interface_roles.json](./config/can_interface_roles.json) and keeps the SocketCAN naming, bitrate, and CAN FD settings aligned for the Nero arm, effectors, and OmniHand:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
python3 scripts/prepare_can_interfaces.py --list
python3 scripts/prepare_can_interfaces.py --roles nero --dry-run
python3 scripts/prepare_can_interfaces.py --roles nero
```

If you need to pin a specific USB port or Linux interface, add `--nero-can-interface 1-4.2:1.0` or `--nero-can-interface can0`. To prepare both the arm and OmniHand together, use `--roles nero,omnihand`. The native arms and OmniHand use `scripts/activate_native_can.sh` (see docs/control/bringup.md). The legacy USB `can_activate.sh` flow remains in the CAN guide.


### Launch Driver

You can start the driver using a launch file or by running the node directly.

> **Important: Read before launching**
> The parameters in the following launch commands **must** be replaced according to your actual hardware configuration:
> - **`can_port`**: The CAN port connected to the arm, e.g. `can0`; if you follow the recommended `prepare_can_interfaces.py` flow, the default Nero role name is `can_nero`.
> - **`arm_type`**: The arm model. In this workspace the active example is `nero`.
> - **`effector_type`**: The end-effector type, e.g. `none` or `agx_gripper`.
> - **`tcp_offset`**: Tool Center Point (TCP) offset relative to the flange center, e.g. [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
>   - Note: All values of this parameter must be floating-point numbers; for TCP offset configuration examples, see [TCP Offset Guide](./docs/assets/tcp_offset/TCP_OFFSET_EN.md).
>
> For full parameter descriptions, default values and options, see **[Launch Parameters](#launch-parameters)** below.


**Using launch file:**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=nero effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**Running node directly:**

```bash
ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args -p can_port:=can0 -p arm_type:=nero -p effector_type:=none -p tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**Visualization Debug Launch:**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

> **Note:**
> - `start_single_agx_arm_rviz.launch.py` now defaults to `use_mit_controller:=true`, so the RViz control path prefers MIT soft trajectories instead of writing directly to `/control/joint_states`.
> - When `control:=true` and `use_mit_controller:=true`, RViz joint sliders publish to `mit_controller/soft_target_joint_states`, and `agx_arm_mit_joint_state_bridge` turns those targets into short debug `trajectory_msgs/JointTrajectory` segments for `mit_controller`.
> - Use `mit_joint_target_duration_s` to tune the soft segment duration per slider update. Set `use_mit_controller:=false` if you intentionally want the legacy `/control/joint_states` path.
> - `follow:=true` keeps the display synchronized with real feedback. If you only want state-follow visualization, keep `control:=false`.
> - The MIT debug topic is enabled only while `control:=true`, and the slider bridge no longer auto-enables MIT. Enable `mit_controller` explicitly before driving sliders.

**MoveIt One-Click Launch (Arm Control + MoveIt + RViz):**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can_nero \
    arm_type:=nero \
    effector_type:=agx_gripper \
    load_simple_obstacles:=true
```

> This launch file now defaults to `use_mit_controller:=true`: it starts `start_nero_mit_controller.launch.py` and routes MoveIt execution through `arm_controller/follow_joint_trajectory` into the integrated MIT action server on `mit_controller`. `load_simple_obstacles:=true` seeds the planning scene from [simple_obstacles.json](./src/agx_arm_moveit/config/simple_obstacles.json) via `src/agx_arm_moveit/scripts/apply_simple_obstacles.py`; use `simple_obstacles_config:=/abs/path/to/file.json` to replace that baseline. Set `use_mit_controller:=false` only if you explicitly want the legacy direct execution path.

Tune the MIT side with `mit_control_rate_hz` and `mit_params_file` as needed.

**Nero MIT Soft Trajectory Control (application node + arm runtime):**

```bash
ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py \
    can_port:=can_nero \
    arm_type:=nero \
    effector_type:=agx_gripper \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

This launch reuses `agx_arm_ctrl` as the hardware adapter and starts the MIT application node on top. In addition to `can_port`, it now forwards the main `agx_arm_ctrl` runtime parameters directly: `arm_type`, `effector_type`, `omnihand_type`, `launch_omnihand_bridge`, `omnihand_backend_type`, `auto_enable`, `fast_mode`, `speed_percent`, `pub_rate`, `enable_timeout`, `tcp_offset`, `gripper_default_effort`, and `publish_gripper_joint`. It also exposes MIT-specific `control_rate_hz`, `params_file`, `log_level`, and the opt-in `enable_debug_joint_trajectory_topic` guard for RViz soft-target debugging.

The MIT node subscribes to `feedback/joint_states`, serves `arm_controller/follow_joint_trajectory` for MoveIt, publishes `control/move_mit` continuously, and only accepts the debug `~/joint_trajectory` topic when that input is explicitly enabled.

### Launch Parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `can_port` | `can0` | CAN port | - |
| `arm_type` | `nero` | Arm model | `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `namespace` | empty string | Arm instance namespace | Any valid ROS namespace |
| `auto_enable` | `true` | Auto enable on startup | `true`, `false` |
| `fast_mode` | `false` | Enable fast mode (If enabled, `/control/joint_states` will internally switch to the unsmoothed and non-interpolated `move_js` joint control interface to command the robotic arm.) | `true`, `false` |
| `speed_percent` | `100` | Motion speed (%) | `0-100` |
| `pub_rate` | `200` | Status publish rate (Hz) | - |
| `enable_timeout` | `5.0` | Enable timeout (seconds) | - |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | Tool Center Point (TCP) offset relative to the flange center [x, y, z, rx, ry, rz] | - |
| `gripper_default_effort` | `1.0` | The default effort of the gripper (in N) | `>=0.0` |
| `publish_gripper_joint` | `true` | Whether to publish the `gripper` joint (gripper opening width) in `/feedback/joint_states`. Set to `false` when used with MoveIt, as the URDF only defines `gripper_joint1`/`gripper_joint2` | `true`, `false` |
| `log_level` | `info` | Log level | `debug`, `info`, `warn`, `error`, `fatal` |

### URDF Model Visualization

#### Standalone Model View

Load the URDF model in RViz and adjust joints manually using the GUI sliders. No need to start the arm driver node:

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=nero
```

**The following three methods are supported to specify the model:**

1. **Preset model name (via `arm_type`)** (recommended): Use a built-in model name to automatically match the corresponding URDF file

    ```bash
    ros2 launch agx_arm_description display.launch.py arm_type:=nero
    ```

2. **Relative path (via `custom_model`)**: Path relative to the `agx_arm_urdf/` directory, suitable for custom models

    ```bash
    ros2 launch agx_arm_description display_control.launch.py custom_model:=nero/urdf/nero_description.urdf
    ```

3. **Absolute path (via `custom_model`)**: Directly specify the absolute path to a URDF file, suitable for model files at any location

    ```bash
    ros2 launch agx_arm_description display.launch.py custom_model:=/home/user/my_robot/custom_arm.urdf
    ```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_type` | `nero` | Arm model. Preset: `nero` |
| `custom_model` | empty string | Optional custom model path. If relative, it is resolved under `agx_arm_urdf/`; if absolute, it can point to any URDF/xacro file. When set, `arm_type` and `effector_type` are ignored |
| `effector_type` | `none` | End-effector type. Presets: `none`, `agx_gripper`, `revo2` |
| `revo2_type` | `left` | Revo2 dexterous hand type. Presets: `left`, `right` |
| `pub_rate` | `200` | Status publish rate (Hz) |
| `gui` | `true` | Whether to enable the `joint_state_publisher_gui` slider control interface |
| `rvizconfig` | Built-in config | Absolute path to a custom RViz configuration file |
| `follow` | `false` | Whether to follow the real arm state (subscribe to `/feedback/joint_states`, and remap `/joint_states` to `feedback/joint_states` in `robot_state_publisher`) |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP offset [x, y, z, rx, ry, rz] in meters/radians. When non-zero, a `tcp_link` TF frame is published automatically |
| `control` | `true` | Whether to publish control topics via `joint_state_publisher` (or GUI version). When `true`, publishes to `control_topic`; when `false`, only follows/displays without publishing control topics. A common real-arm combination is `follow:=true, control:=false` (follow real arm without sending control from RViz). |
| `control_topic` | `/control/joint_states` | Target topic where `joint_state_publisher_gui` publishes joint slider outputs |

#### Typical Usage Combinations (follow / control)

Below are **common usage scenarios** with recommended `follow` and `control` combinations and example commands:

- **Scenario 1: Pure model debugging (no real arm, view URDF and move via sliders only)**  
  - Real arm required: No  
  - Recommended configuration: `follow:=false, control:=true`  
  - Example:  
    ```bash
    ros2 launch agx_arm_description display_control.launch.py arm_type:=nero follow:=false control:=true
    ```

> Note: If you want to redirect RViz slider joint targets to an impedance controller (e.g. `agx_arm_impedance` joint impedance with `control_type:=joint_impedance`), start `display.launch.py` with `control:=true` and `control_topic:=/impedance/target_joint`.

- **Scenario 2: Real arm + control only, no follow** (use RViz to send control, but RViz does not follow real feedback)  
  - Real arm required: Yes  
  - Recommended configuration: `follow:=false, control:=true`  
  - Example:  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=false control:=true
    ```

- **Scenario 3: Real arm + follow only, no control** (common: monitor state only, avoid RViz interfering with control)  
  - Real arm required: Yes  
  - Recommended configuration: `follow:=true, control:=false`  
  - Example:  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=true control:=false
    ```

- **Scenario 4: Real arm + control + follow** (use RViz to send control and follow real feedback)  
  - Real arm required: Yes  
  - Recommended configuration: `follow:=true, control:=true`
  - Example:  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=nero follow:=true control:=true
    ```

> **Tip:** In general, it is recommended to keep the **control channel unique**, i.e. only one component should be responsible for publishing `/control/*` topics (choose one among the `agx_arm_ctrl` node, MoveIt, or RViz `joint_state_publisher`) to avoid conflicts from multiple control sources.

---

## Control Examples

Open an additional terminal and run the following commands:

```bash
cd ~/agx_arm_ws
source install/setup.bash
cd src/agx_arm_ros
```

### Nero Arm

1. Joint motion

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/nero/test_move_j.yaml)" -1
    ```

2. Point-to-point motion

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_p.yaml)" -1
    ```

3. Linear motion

    ```bash
    ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_l.yaml)" -1
    ```

4. Circular motion (start → middle → end)

    ```bash
    ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
      "$(cat test/nero/test_move_c.yaml)" -1
    ```

### Gripper

1. Gripper control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/gripper/test_gripper_joint_states.yaml)" -1
    ```

### Dexterous Hand

1. Dexterous hand — Position mode (all fingers move to 10)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_position.yaml)" -1
    ```

2. Dexterous hand — Speed mode (all fingers speed 50)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_speed.yaml)" -1
    ```

3. Dexterous hand — Current mode (all fingers current 50)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_current.yaml)" -1
    ```

4. Dexterous hand — Position-time control (all fingers move to 50, time 1 second)

    ```bash
    ros2 topic pub /control/hand_position_time agx_arm_msgs/msg/HandPositionTimeCmd \
      "$(cat test/hand/test_hand_position_time.yaml)" -1
    ```

5. Dexterous hand control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/hand/test_hand_joint_states.yaml)" -1
    ```

### Service Calls

1. Enable arm

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
    ```

2. Disable arm

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
    ```

3. Move to home position

    ```bash
    ros2 service call /move_home std_srvs/srv/Empty
    ```

4. Emergency stop (hold current position)

    ```bash
    ros2 service call /emergency_stop std_srvs/srv/Empty
    ```

### Status Subscription

1. Joint states

    ```bash
    ros2 topic echo /feedback/joint_states
    ```

2. TCP pose

    ```bash
    ros2 topic echo /feedback/tcp_pose
    ```

3. Arm status

    ```bash
    ros2 topic echo /feedback/arm_status
    ```

4. Leader joint angles(For leader arm mode)

    ```bash
    ros2 topic echo /feedback/leader_joint_angles
    ```

5. Gripper status

    ```bash
    ros2 topic echo /feedback/gripper_status
    ```

6. Dexterous hand status 

    ```bash
    ros2 topic echo /feedback/hand_status
    ```

---

## ROS2 Interface

### Feedback Topics

| Topic | Message Type | Description | Condition |
|-------|--------------|-------------|-----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | Joint states | Always available |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP pose | Always available |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | Arm status | Always available |
| `/feedback/leader_joint_angles` | `sensor_msgs/JointState` | Leader joint angles | Leader arm mode |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | Gripper status | AgxGripper configured |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | Dexterous hand status | Revo2 configured |

#### Joint States Details (`/feedback/joint_states`)

This topic contains combined joint states for the arm and end-effector:

**Arm Joints** (`joint1` ~ `joint*`)

| Field | Description |
|-------|-------------|
| `position` | Joint angle (rad) |
| `velocity` | Joint velocity (rad/s) |
| `effort` | Joint torque (Nm) |

**Gripper Joints** (requires `effector_type=agx_gripper`)

By default, three joints are published: `gripper`, `gripper_joint1`, and `gripper_joint2`. If `publish_gripper_joint` is set to `false`, only `gripper_joint1` and `gripper_joint2` are published (the URDF joint names, compatible with MoveIt).

| Joint Name | `position` | `velocity` | `effort` |
|------------|------------|------------|----------|
| `gripper` | Gripper opening width (m), range [0, 0.1] | 0.0 | Force (N) |
| `gripper_joint1` | Single jaw displacement = width × 0.5 (m) | 0.0 | Force (N) |
| `gripper_joint2` | Single jaw displacement = width × -0.5 (m) | 0.0 | Force (N) |

**Dexterous Hand Joints** (requires `effector_type=revo2`)

Left hand joint names:

- `left_thumb_metacarpal_joint`
- `left_thumb_proximal_joint`
- `left_index_proximal_joint`
- `left_middle_proximal_joint`
- `left_ring_proximal_joint`
- `left_pinky_proximal_joint`

Right hand joint names:

- `right_thumb_metacarpal_joint`
- `right_thumb_proximal_joint`
- `right_index_proximal_joint`
- `right_middle_proximal_joint`
- `right_ring_proximal_joint`
- `right_pinky_proximal_joint`

| Field | Description |
|-------|-------------|
| `position` | Finger joint angle (rad) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

#### Arm Status Details (`/feedback/arm_status`)

Message type: `agx_arm_msgs/AgxArmStatus`

**Message Field Description:**

| Field | Type | Description |
|-------|------|-------------|
| `ctrl_mode` | `uint8` | Control mode, see table below |
| `arm_status` | `uint8` | Arm status, see table below |
| `mode_feedback` | `uint8` | Mode feedback, see table below |
| `teach_status` | `uint8` | Teach status, see table below |
| `motion_status` | `uint8` | Motion status: 0=reached target, 1=not reached |
| `trajectory_num` | `uint8` | Current trajectory point number (0~255, feedback in offline trajectory mode) |
| `err_status` | `int64` | Error status code |
| `joint_1_angle_limit` ~ `joint_7_angle_limit` | `bool` | Joint 1~7 angle limit (true=abnormal, false=normal) |
| `communication_status_joint_1` ~ `communication_status_joint_7` | `bool` | Joint 1~7 communication status (true=abnormal, false=normal) |

**Control Mode (`ctrl_mode`):**

| Value | Description |
|-------|-------------|
| 0 | Standby |
| 1 | CAN command control |
| 2 | Teach mode |
| 3 | Ethernet control |
| 4 | WiFi control |
| 5 | Remote control mode |
| 6 | Coordinated teach input |
| 7 | Offline trajectory mode |
| 8 | TCP control |

**Arm Status (`arm_status`):**

| Value | Description |
|-------|-------------|
| 0 | Normal |
| 1 | Emergency stop |
| 2 | No solution |
| 3 | Singularity |
| 4 | Target angle out of range |
| 5 | Joint communication abnormal |
| 6 | Joint brake not released |
| 7 | Collision detected |
| 8 | Teach drag overspeed |
| 9 | Joint status abnormal |
| 10 | Other abnormal |
| 11 | Teaching recording |
| 12 | Teaching executing |
| 13 | Teaching paused |
| 14 | Main controller NTC overtemperature |
| 15 | Release resistor NTC overtemperature |

**Mode Feedback (`mode_feedback`):**

| Value | Description |
|-------|-------------|
| 0 | MOVE P |
| 1 | MOVE J |
| 2 | MOVE L |
| 3 | MOVE C |
| 4 | MOVE MIT |
| 5 | MOVE CPV |

**Teach Status (`teach_status`):**

| Value | Description |
|-------|-------------|
| 0 | Closed |
| 1 | Start teaching record (enter drag teaching) |
| 2 | End teaching record (exit drag teaching) |
| 3 | Execute teaching trajectory |
| 4 | Pause execution |
| 5 | Continue execution |
| 6 | Terminate execution |
| 7 | Move to trajectory start point |

#### Gripper Status Details (`/feedback/gripper_status`)

Message type: `agx_arm_msgs/GripperStatus`

**Message Field Description:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Message header |
| `width` | `float64` | Current gripper opening width (unit: meters) |
| `force` | `float64` | Current gripping force (unit: Newtons) |
| `voltage_too_low` | `bool` | Voltage too low (true=abnormal, false=normal) |
| `motor_overheating` | `bool` | Motor overheating (true=abnormal, false=normal) |
| `driver_overcurrent` | `bool` | Driver overcurrent (true=abnormal, false=normal) |
| `driver_overheating` | `bool` | Driver overheating (true=abnormal, false=normal) |
| `sensor_status` | `bool` | Sensor status (true=abnormal, false=normal) |
| `driver_error_status` | `bool` | Driver error status (true=abnormal, false=normal) |
| `driver_enable_status` | `bool` | Driver enable status (true=enabled, false=disabled) |
| `homing_status` | `bool` | Homing status (true=completed, false=not completed) |

#### Hand Status Details (`/feedback/hand_status`)

Message type: `agx_arm_msgs/HandStatus`

**Message Field Description:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Message header |
| `left_or_right` | `uint8` | Hand type identifier: 1=left hand, 2=right hand |

**Finger Position Fields (range: [0, 100], 0=fully open, 100=fully closed):**

| Field | Type | Description |
|-------|------|-------------|
| `thumb_tip_pos` | `uint8` | Thumb tip position |
| `thumb_base_pos` | `uint8` | Thumb base position |
| `index_finger_pos` | `uint8` | Index finger position |
| `middle_finger_pos` | `uint8` | Middle finger position |
| `ring_finger_pos` | `uint8` | Ring finger position |
| `pinky_finger_pos` | `uint8` | Pinky finger position |

**Finger Motor Status Fields (0=idle, 1=running, 2=stalled/jammed):**

| Field | Type | Description |
|-------|------|-------------|
| `thumb_tip_status` | `uint8` | Thumb tip motor status |
| `thumb_base_status` | `uint8` | Thumb base motor status |
| `index_finger_status` | `uint8` | Index finger motor status |
| `middle_finger_status` | `uint8` | Middle finger motor status |
| `ring_finger_status` | `uint8` | Ring finger motor status |
| `pinky_finger_status` | `uint8` | Pinky finger motor status |

### Control Topics

| Topic                           | Message Type                       | Description           | Condition          |
| ----------------------------- | ---------------------------------- | ------------ | ------------- |
| `/control/joint_states`       | `sensor_msgs/JointState`           | Joint control (with end-effector) | Always available          |
| `/control/move_j`             | `sensor_msgs/JointState`           | Joint control motion       | Always available          |
| `/control/move_p`             | `geometry_msgs/PoseStamped`        | Point-to-point motion        | Always available          |
| `/control/move_l`             | `geometry_msgs/PoseStamped`        | Linear motion         | Always available      |
| `/control/move_c`             | `geometry_msgs/PoseArray`          | Circular motion         | Always available      |
| `/control/move_js`            | `sensor_msgs/JointState`           | MIT mode joint motion   | Always available      |
| `/control/move_mit`           | `agx_arm_msgs/MoveMITMsg`          | MIT torque control     | Always available      |
| `/control/hand`               | `agx_arm_msgs/HandCmd`             | Dexterous hand control        | Revo2 configured      |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | Hand position-time control    | Revo2 configured      |

#### `/control/joint_states` Details

This topic uses the `sensor_msgs/JointState` message type and supports simultaneous control of arm joints and end-effector (gripper/dexterous hand). Only the joints to be controlled need to be sent; joints not included will not be affected.

**Message Field Description:**

| Field | Description |
|-------|-------------|
| `name` | Joint name list |
| `position` | Target position for corresponding joints |
| `velocity` | Not used (can be left empty) |
| `effort` | Used for gripper force control (only effective for `gripper` joint) |

**Gripper control via `/control/joint_states`** (requires `effector_type=agx_gripper`)

Include `gripper` in `name`, set target width via `position`, and set gripping force via `effort`.

| Joint Name | position (width) | effort (force) |
|------------|-----------------|----------------|
| `gripper` | Target width (m), range: [0.0, 0.1] | Target force (N), range: [0.5, 3.0], default: 1.0 |

> **Note:** When `effort` is 0 or not specified, the default force of 1.0N is used.

Example: Control gripper width to 0.05m with force 1.5N
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [gripper], position: [0.05], velocity: [], effort: [1.5]}" -1
```

**Dexterous hand control via `/control/joint_states`** (requires `effector_type=revo2`)

Include dexterous hand joint names in `name`, set the target position via `position` (position mode, unit: rad). Only the joints to be controlled need to be sent; joints not included will maintain their current position.

Example: Control only the left index finger to position 0.5 rad
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [left_index_proximal_joint], position: [0.5], velocity: [], effort: []}" -1
```

| Joint Name | Description | Position Range |
|------------|-------------|---------------|
| `left_thumb_metacarpal_joint` / `right_thumb_metacarpal_joint` | Thumb base | [0, 1.57] |
| `left_thumb_proximal_joint` / `right_thumb_proximal_joint` | Thumb tip | [0, 1.03] |
| `left_index_proximal_joint` / `right_index_proximal_joint` | Index finger | [0, 1.41] |
| `left_middle_proximal_joint` / `right_middle_proximal_joint` | Middle finger | [0, 1.41] |
| `left_ring_proximal_joint` / `right_ring_proximal_joint` | Ring finger | [0, 1.41] |
| `left_pinky_proximal_joint` / `right_pinky_proximal_joint` | Pinky finger | [0, 1.41] |

#### `/control/move_mit` Details

Message type: `agx_arm_msgs/MoveMITMsg`

**Message Field Description:**

| Field | Type | Description |
|-------|------|-------------|
| `joint_index` | `int32[]` | Array of joint indices to control |
| `p_des` | `float64[]` | Desired joint position array (unit: radians) |
| `v_des` | `float64[]` | Desired joint velocity array (unit: radians/second) |
| `kp` | `float64[]` | Position gain array |
| `kd` | `float64[]` | Velocity gain array |
| `torque` | `float64[]` | Desired joint torque array (unit: Newton-meters, N·m) |

> **Note:** All array fields must have the same length as `joint_index`. Supports simultaneous control of multiple joints.

#### `/control/hand` Details

Message type: `agx_arm_msgs/HandCmd`

**Message Field Description:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | `string` | Control mode: `position` (position) / `speed` (speed) / `current` (current) |

**Finger Target Value Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `thumb_tip` | `int8` | Thumb tip target value |
| `thumb_base` | `int8` | Thumb base target value |
| `index_finger` | `int8` | Index finger target value |
| `middle_finger` | `int8` | Middle finger target value |
| `ring_finger` | `int8` | Ring finger target value |
| `pinky_finger` | `int8` | Pinky finger target value |

**Value Ranges for Different Modes:**

| Mode | Value Range | Description |
|------|-------------|-------------|
| `position` | [0, 100] | 0=fully open, 100=fully closed |
| `speed` | [-100, 100] | Negative=open direction, positive=close direction |
| `current` | [-100, 100] | Negative=open direction, positive=close direction |

#### `/control/hand_position_time` Details

Message type: `agx_arm_msgs/HandPositionTimeCmd`

**Message Field Description:**

**Finger Target Position Fields (range: [0, 100], 0=fully open, 100=fully closed):**

| Field | Type | Description |
|-------|------|-------------|
| `thumb_tip_pos` | `int8` | Thumb tip position |
| `thumb_base_pos` | `int8` | Thumb base position |
| `index_finger_pos` | `int8` | Index finger position |
| `middle_finger_pos` | `int8` | Middle finger position |
| `ring_finger_pos` | `int8` | Ring finger position |
| `pinky_finger_pos` | `int8` | Pinky finger position |

**Finger Arrival Time Fields (unit: 10ms, range: [0, 255], e.g.: 200 = 2 seconds):**

| Field | Type | Description |
|-------|------|-------------|
| `thumb_tip_time` | `uint8` | Thumb tip arrival time |
| `thumb_base_time` | `uint8` | Thumb base arrival time |
| `index_finger_time` | `uint8` | Index finger arrival time |
| `middle_finger_time` | `uint8` | Middle finger arrival time |
| `ring_finger_time` | `uint8` | Ring finger arrival time |
| `pinky_finger_time` | `uint8` | Pinky finger arrival time |

### Services

| Service | Type | Description | Condition |
|---------|------|-------------|-----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | Enable/disable arm | Always available |
| `/move_home` | `std_srvs/Empty` | Move to home position | Always available |
| `/emergency_stop` | `std_srvs/Empty` | Emergency stop (hold current position) | Always available |

---

## Parameter Limits

### Gripper

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| width | [0.0, 0.1] m | - | Target opening width |
| force | [0.5, 3.0] N | 1.0 | Gripping force |

> ⚠️ Values out of range will be rejected (not executed), and the node will output a warning log. For example: when sending force=5.0, the command will not be executed, and a warning `force must be in range [0.5, 3.0], current value: 5.0` will be output.

### Dexterous Hand (Revo2)

| Parameter | Range | Description |
|-----------|-------|-------------|
| position | [0, 100] | Finger target position, 0 = fully open, 100 = fully closed |
| speed | [-100, 100] | Finger motion speed |
| current | [-100, 100] | Finger drive current |
| time | [0, 255] | Time to reach target position (unit: 10ms, e.g. 100 = 1 second) |

> ⚠️ Values out of range will be rejected (not executed), and the node will output a warning log. For example: when sending position=120, the command will not be executed, and a warning `position must be in range [0, 100], current value: 120` will be output.

---

## Important Notes

### CAN Communication

- CAN module **must be activated** before use
- Baud rate: **1000000 bps**
- If `SendCanMessage failed` error occurs, check CAN connection

### ⚠️ Safety Warnings

- **Maintain safe distance**: Do not enter the arm's workspace during motion to avoid injury
- **Singularity risk**: Joints may move suddenly and significantly near kinematic singularities
- **MIT mode is dangerous**: High-speed response MIT mode is extremely hazardous, use with caution
