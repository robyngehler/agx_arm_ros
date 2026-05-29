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

## Current in-repo files

The current repository-integrated surfaces are:

```text
duo_body.xacro
duo_system.urdf.xacro
nero_arm_macro.xacro
display_duo_system.launch.py
body_visual.stl
body_collision.stl
```

Current repository status:

```xml
<xacro:include filename="$(find duo_body_description)/urdf/nero_arm_macro.xacro"/>
<xacro:arg name="use_left_arm" default="false"/>
<xacro:arg name="use_right_arm" default="true"/>
<xacro:arg name="use_left_hand" default="false"/>
<xacro:arg name="use_right_hand" default="true"/>
```

`nero_arm_macro.xacro` now exists in the repository and provides:

- prefixable Nero arm links and joints
- a reusable OmniHand flange macro
- a side-selectable OmniHand attachment macro for `left` and `right`

The current assembly uses `left_arm_` and `right_arm_` as arm prefixes so the arm base links do not collide with OmniHand's `left_base_link` and `right_base_link`.

---

# Repository integration plan

## 1. Sprint 3/4 integration target

Reframe the next integration targets around a staged but count-configurable system description:

1. First executable target: `body + right Nero arm + right OmniHand`
2. Second executable target: add the mirrored left arm and left hand without redesigning the top-level Xacro or launch surface
3. Only after both descriptions load cleanly in RViz: generalize the control and MoveIt paths away from single-arm assumptions

For the current repository state, the new default launch target should therefore be the right-side integrated system, while all top-level Xacros and launches remain configurable via `use_left_arm/use_left_hand/use_right_arm/use_right_hand`.

## 2. Decide package location

Current staging location:

```text
src/duo_body_description/
```

This matches the current body mesh paths:

```xml
package://duo_body_description/meshes/body_visual.stl
package://duo_body_description/meshes/body_collision.stl
```

Important repository note: the durable repo rule still wants `agx_arm_description` to remain the long-term single discoverable description package. Treat `duo_body_description` as the current system-assembly staging package unless Sprint 3/4 proves that a permanent separate package boundary is justified.

If the body system is later promoted into the canonical description surface, replace all occurrences of:

```text
duo_body_description
```

with:

```text
agx_arm_description
```

Do not mix package names. ROS enjoys punishing that with path errors and emotional damage.

## 3. Package structure

Current package contents are already in place:

```bash
src/duo_body_description/
  launch/
  meshes/
  rviz/
  urdf/
```

The package now needs to stay focused on system description and visualization staging, not on owning duplicated arm or OmniHand mesh assets.

## 4. Install mesh and URDF folders

`CMakeLists.txt` now installs:

```bash
install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
```

Then build once:

```bash
cd ~/workspace/agx_arm_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agx_arm_description duo_body_description
source install/setup.bash
```

## 5. Use the current mount-to-arm-base correction as the default staging value

In `duo_system.urdf.xacro`, the current staging default is no longer zero. Use:

```xml
<joint name="left_arm_mount_to_base" type="fixed">
  <parent link="left_arm_mount_link"/>
  <child link="left_arm_base_link"/>
  <origin xyz="0.01 0.02 0" rpy="0 0 3.1415926"/>
</joint>

<joint name="right_arm_mount_to_base" type="fixed">
  <parent link="right_arm_mount_link"/>
  <child link="right_arm_base_link"/>
  <origin xyz="0.01 0.02 0" rpy="0 0 3.1415926"/>
</joint>
```

This is the current RViz-backed staging estimate after correcting the body mesh scale and mesh origin. It assumes the local mount-link frame convention is the same on both sides, so the same local translation and `pi` yaw correction can be used as the default on both the left and right arm bases.

Important validation criterion:

```text
right_arm_base_link + 0.138 m along local Z must land on the real joint1/joint2 center.
```

If not, adjust only these two mount-to-base fixed joints.

## 6. Generate and validate the URDF

```bash
cd ~/workspace/agx_arm_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xacro xacro \
  src/duo_body_description/urdf/duo_system.urdf.xacro \
  use_left_arm:=false use_left_hand:=false \
  use_right_arm:=true use_right_hand:=true \
  > /tmp/duo_system.urdf

check_urdf /tmp/duo_system.urdf
```

If `check_urdf` is missing:

```bash
sudo apt install liburdfdom-tools
```

## 7. RViz validation before MoveIt

Fastest current staging launch:

```bash
ros2 launch duo_body_description display_duo_system.launch.py \
  use_left_arm:=false use_left_hand:=false \
  use_right_arm:=true use_right_hand:=true
```

This launch is only for description-level validation. It does not solve the still single-arm assumptions inside the current control or MoveIt runtime.

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
right_arm_base_link
right_arm_link1
right_arm_nero_tool0
right_arm_omnihand_flange
right_base_link
right_palm
```

Expected result:

- The right arm grows out of the right mounting plate.
- `right_arm_base_link + Z` points toward the physical joint center.
- The right OmniHand sits on the flange without a frame-name collision.
- No arm or hand points into the body.

After the right-side chain is correct, repeat the same RViz validation with `use_left_arm:=true use_left_hand:=true` before touching MoveIt.

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
  <parent link="left_arm_base_link"/>
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
  <parent link="right_arm_base_link"/>
  <child link="right_joint1_debug"/>
  <origin xyz="0 0 0.138" rpy="0 0 0"/>
</joint>
```

If the spheres land on the measured real joint center, the body-to-arm mount logic is correct.

Remove these debug links before generating the final MoveIt config.

## 9. After description validation: control and MoveIt generalization

The next blockers are no longer in the body Xacro itself. They are in the single-arm assumptions of:

- launch surfaces in `agx_arm_ctrl`
- the MIT-controller RViz/MoveIt path
- MoveIt config ownership in `agx_arm_moveit`

Treat those as a separate follow-on slice after the right-side description is stable and the left-side mirror has been added.

## 10. MoveIt setup

Only when the full robot description loads correctly in RViz:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Create planning groups:

```text
right_arm
right_arm_hand
left_arm
both_arms
left_arm_hand
right_arm_hand
```

Then regenerate the self-collision matrix. Keep body collisions active except for obvious fixed adjacent mounting collisions.
