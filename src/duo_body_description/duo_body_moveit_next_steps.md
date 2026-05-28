# Duo Body + Nero Arms MoveIt Integration — Next Steps

## Current body/mount-frame result

The body frame and both arm mount frames have been defined in CAD and transferred into `duo_body.xacro`.

Latest measured transforms from `body_base_link`:

```xml
<!-- left_arm_mount_link -->
<origin xyz="-0.136240 0.105467 0.070150"
        rpy="0 1.570796 -0.837758"/>

<!-- right_arm_mount_link -->
<origin xyz="0.137511 0.104055 0.070000"
        rpy="0 1.570796 0.837758"/>
```

Notes:

- Translation values are converted from millimeters to meters.
- `1.570796 rad = 90 deg`.
- `0.837758 rad = 48 deg`.
- The latest CAD screenshots show `48 deg` between `body_base_link` Y and each mount-link Y axis, so the mount yaw values are currently set to `±48 deg`.
- This assumes the chosen convention:
  - mount `Z` points out of the mounting plate toward the arm,
  - mount `X` points downward,
  - mount `Y` completes a right-handed frame.

URDF/Xacro uses fixed-axis RPY. Practically:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

So the syntax above is valid for URDF/Xacro. The `pitch = +90 deg` case is a gimbal-lock orientation, but it is still valid. The resulting RPY is not unique, so RViz validation is mandatory.

## Files produced here

Use the patched files:

```text
duo_body_patched.xacro
duo_system_patched.urdf.xacro
body_visual.stl
body_collision.stl
```

`duo_system_patched.urdf.xacro` currently still expects a prefix-capable Nero arm macro:

```xml
<xacro:include filename="$(find duo_body_description)/urdf/nero_arm_macro.xacro"/>
<xacro:nero_arm prefix="left_"/>
<xacro:nero_arm prefix="right_"/>
```

That file still needs to be created inside the real repository.

---

# Repository integration plan

## 1. Decide package location

Recommended for now: create a separate package:

```text
src/duo_body_description/
```

This matches the current mesh paths:

```xml
package://duo_body_description/meshes/body_visual.stl
package://duo_body_description/meshes/body_collision.stl
```

Alternative: put everything into the existing `agx_arm_description` package. If doing that, replace all occurrences of:

```text
duo_body_description
```

with:

```text
agx_arm_description
```

Do not mix package names. ROS enjoys punishing that with path errors and emotional damage.

## 2. Create package structure

For the standalone package:

```bash
cd ~/workspace/agx_arm_ros/src
ros2 pkg create duo_body_description --build-type ament_cmake

mkdir -p duo_body_description/urdf
mkdir -p duo_body_description/meshes
mkdir -p duo_body_description/launch
mkdir -p duo_body_description/rviz
```

Copy files:

```bash
cp /path/to/duo_body_patched.xacro \
  ~/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_body.xacro

cp /path/to/duo_system_patched.urdf.xacro \
  ~/workspace/agx_arm_ros/src/duo_body_description/urdf/duo_system.urdf.xacro

cp /path/to/body_visual.stl \
  ~/workspace/agx_arm_ros/src/duo_body_description/meshes/body_visual.stl

cp /path/to/body_collision.stl \
  ~/workspace/agx_arm_ros/src/duo_body_description/meshes/body_collision.stl
```

## 3. Install mesh and URDF folders

Edit `duo_body_description/CMakeLists.txt` and add:

```cmake
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

Then build once:

```bash
cd ~/workspace/agx_arm_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select duo_body_description
source install/setup.bash
```

## 4. Create `nero_arm_macro.xacro`

Copy the existing Nero URDF into the new package:

```bash
cp \
  ~/workspace/agx_arm_ros/src/agx_arm_sim/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf \
  ~/workspace/agx_arm_ros/src/duo_body_description/urdf/nero_arm_macro.xacro
```

Then convert it into a macro:

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="nero_arm" params="prefix">
    ...
  </xacro:macro>
</robot>
```

Required edits:

- Remove the original `world` link.
- Remove the original `world_to_base_link` fixed joint.
- Prefix every Nero link:

```xml
<link name="base_link"/>
```

becomes:

```xml
<link name="${prefix}base_link"/>
```

- Prefix every Nero joint:

```xml
<joint name="joint1" type="revolute">
```

becomes:

```xml
<joint name="${prefix}joint1" type="revolute">
```

- Prefix every joint parent/child link reference:

```xml
<parent link="base_link"/>
<child link="link1"/>
```

becomes:

```xml
<parent link="${prefix}base_link"/>
<child link="${prefix}link1"/>
```

- Prefix `nero_tool0` as well:

```xml
<link name="${prefix}nero_tool0"/>
```

## 5. Keep mount-to-arm-base transform initially zero

In `duo_system.urdf.xacro`, keep:

```xml
<joint name="left_mount_to_left_base" type="fixed">
  <parent link="left_arm_mount_link"/>
  <child link="left_base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<joint name="right_mount_to_right_base" type="fixed">
  <parent link="right_arm_mount_link"/>
  <child link="right_base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

This is correct if each mount frame is intentionally aligned to the Nero `base_link` frame.

Important validation criterion:

```text
base_link + 0.138 m along local Z must land on the real joint1/joint2 center.
```

If not, adjust only these two mount-to-base fixed joints.

## 6. Generate and validate the URDF

```bash
cd ~/workspace/agx_arm_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xacro xacro \
  src/duo_body_description/urdf/duo_system.urdf.xacro \
  > /tmp/duo_system.urdf

check_urdf /tmp/duo_system.urdf
```

If `check_urdf` is missing:

```bash
sudo apt install liburdfdom-tools
```

## 7. RViz validation before MoveIt

Launch `robot_state_publisher` manually:

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(ros2 run xacro xacro src/duo_body_description/urdf/duo_system.urdf.xacro)"
```

Open RViz:

```bash
rviz2
```

Set:

```text
Fixed Frame: body_base_link
```

Add:

```text
RobotModel
TF
```

Check these frames:

```text
body_base_link
left_arm_mount_link
right_arm_mount_link
left_base_link
right_base_link
left_link1
right_link1
left_nero_tool0
right_nero_tool0
```

Expected result:

- Both arms grow out of the mounting plates.
- Both `base_link + Z` directions point toward the physical joint center.
- No arm points into the body.
- Left and right joint axes are mirrored only by placement, not by invalid frame reflection.

## 8. Add temporary joint-center debug spheres

Add this temporarily to `duo_system.urdf.xacro`:

```xml
<link name="left_joint1_debug">
  <visual>
    <geometry>
      <sphere radius="0.015"/>
    </geometry>
  </visual>
</link>

<joint name="left_joint1_debug_joint" type="fixed">
  <parent link="left_base_link"/>
  <child link="left_joint1_debug"/>
  <origin xyz="0 0 0.138" rpy="0 0 0"/>
</joint>

<link name="right_joint1_debug">
  <visual>
    <geometry>
      <sphere radius="0.015"/>
    </geometry>
  </visual>
</link>

<joint name="right_joint1_debug_joint" type="fixed">
  <parent link="right_base_link"/>
  <child link="right_joint1_debug"/>
  <origin xyz="0 0 0.138" rpy="0 0 0"/>
</joint>
```

If the spheres land on the measured real joint center, the body-to-arm mount logic is correct.

Remove these debug links before generating the final MoveIt config.

## 9. Only after this: integrate OmniHand

Do not integrate the hands until the two-arm body model is correct.

Next stage:

```text
left_link7  -> left_omnihand_flange  -> left_omnihand
right_link7 -> right_omnihand_flange -> right_omnihand
```

Use the existing `nero_with_omnihand_flange_description.xacro` as reference for the fixed `link7 -> omnihand_flange` transform, but convert it to prefix-capable form as well.

## 10. MoveIt setup

Only when the full robot description loads correctly in RViz:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Create planning groups:

```text
left_arm
right_arm
both_arms
left_arm_hand
right_arm_hand
```

Then regenerate the self-collision matrix. Keep body collisions active except for obvious fixed adjacent mounting collisions.
