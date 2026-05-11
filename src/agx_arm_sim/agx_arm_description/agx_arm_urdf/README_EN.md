# AgileX Local URDF Assets

[中文](./README.md)

This directory is now a fixed asset tree committed directly inside `agx_arm_description`. It no longer depends on a separate `agx_arm_urdf` git submodule.

## Current contents

| Directory | Purpose |
|-----------|---------|
| `nero/` | Nero body, gripper, and Revo2 combination URDF/Xacro and mesh assets |
| `revo2/` | Standalone Revo2 hand URDF and mesh assets used by the Nero combinations |
| `README.md` / `README_EN.md` | Asset-tree documentation |
| `LICENSE` | Upstream license retained in-repo |

## Directory structure

```text
agx_arm_urdf/
├── LICENSE
├── README.md
├── README_EN.md
├── nero/
│   ├── meshes/
│   └── urdf/
└── revo2/
    ├── meshes/
    └── urdf/
```

## Typical files

- `nero/urdf/nero_description.urdf`
- `nero/urdf/nero_with_gripper_description.xacro`
- `nero/urdf/nero_with_left_revo2_description.xacro`
- `nero/urdf/nero_with_right_revo2_description.xacro`
- `revo2/urdf/revo2_left_hand.urdf`
- `revo2/urdf/revo2_right_hand.urdf`

## Usage

Use these assets through the main package entry points:

```bash
ros2 launch agx_arm_description display_control.launch.py arm_type:=nero
```

Or:

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left
```

If you need the assets in another workspace, copy the full `agx_arm_urdf/` directory and install it under a ROS package named `agx_arm_description`.

## License

The upstream [MIT License](./LICENSE) is retained in this directory.
