# Reproducible TRAC-IK Source Install on ROS 2 Humble / Ubuntu 22.04 / Jetson AGX Orin

Target platform:

- NVIDIA Jetson AGX Orin
- Ubuntu 22.04 Jammy
- Kernel 5.15 tegra
- Architecture: `aarch64` / `arm64`
- ROS 2 Humble
- MoveIt 2 Humble
- TRAC-IK source build, because no suitable `ros-humble-trac-ik` apt package was available

## Summary of what worked

The successful build path was:

1. Build TRAC-IK from source in a separate workspace.
2. Avoid Conda / Miniforge Python during the ROS build.
3. Install required system dependencies with apt and rosdep.
4. Patch the newer TRAC-IK MoveIt plugin headers from `.hpp` to Humble-compatible `.h`.
5. Build with `colcon` while explicitly pointing CMake to `/usr/bin/python3`.

## Workspace-level validation in agx_arm_ros

After sourcing the overlay in the order below, the current `agx_arm_ros` workspace validated the
expected TRAC-IK behavior on 2026-05-21:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/trac_ik_ws/install/setup.bash
source ~/workspace/agx_arm_ros/install/setup.bash
```

Verified outcomes:

- `ros2 pkg prefix trac_ik_kinematics_plugin` resolved to `~/workspace/trac_ik_ws/install/trac_ik_kinematics_plugin`
- `scripts/moveit_profile_smoke_test.sh` reached `You can start planning now!` for `none`, `agx_gripper`, `revo2` left/right, and `omnihand` left/right without TRAC-IK plugin-load errors
- a live `/compute_ik` request on `nero_arm` returned `MoveItErrorCodes.SUCCESS`

Remaining issue:

- the timeout-driven `move_group` shutdown path on this Humble / aarch64 host still ended in a SIGINT teardown crash, which appeared separate from TRAC-IK provisioning

## Historical value

Keep this note only as the reproducible fallback for Humble / Jetson hosts where no suitable TRAC-IK
apt package is available.