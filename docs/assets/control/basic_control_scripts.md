# Control Script Reference

This page is a reference map for repo-owned control and bringup scripts.

It does not replace `docs/control/bringups/launches.md` or `docs/control/bringups/teach_and_run.md`.

## Primary operational entrypoints

- `scripts/activate_stack.sh`: the operator entry point — activates the buses, verifies they carry
  clean traffic, and reloads the CAN driver when they do not
- `scripts/activate_duo_can.sh`: the bring-up itself, matched by physical slot; called by the above
- `scripts/isolate_ros_graph.sh --unit top|bottom`: says which unit the machine is and keeps its ROS
  graph on loopback in its own domain; run once per unit
- `scripts/start_demo_stack.py` / `stop_demo_stack.py`: the stack supervisor — brings the launches up
  in order and stays alive owning them
- `scripts/unpack_bottom_unit.py`, `pack_bottom_unit.py`, `unpack_top_unit.py`,
  `pack_top_unit.py`, `wave.py`, `start_tea_demo.py`, `start_block_restack.py`: the activity
  scripts — each runs one activity against a stack that is already up
- `scripts/colcon_build_system_python.sh`: workspace build wrapper
- `scripts/run_in_ros_conda.sh -- <command>`: Conda-backed runtime wrapper
- `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py ...`: canonical combined runtime entrypoint
- `ros2 launch agx_arm_mit_controller start_nero_mit_controller.launch.py ...`: MIT teach and replay baseline

## When to use this page

Use this page only when you need a quick role summary for scripts that are already documented in the
operational control docs.

## Script role summary

| Script or surface | Role |
| --- | --- |
| `scripts/activate_duo_can.sh` | brings up all four Duo buses by physical slot and applies bitrates, one-shot, txqueuelen, rmem and the arm-side TDC offset |
| `scripts/activate_stack.sh` | wraps the above: samples each bus after bring-up (RX advancing on the arms, controller ERROR-ACTIVE, error counters flat) and runs a bounded `rmmod`/`modprobe`/reactivate cycle when a bus does not pass. `--show` and `--verify-only` change nothing; `--recover` forces the reload chain |
| `scripts/isolate_ros_graph.sh` | writes `AGX_UNIT`, `ROS_LOCALHOST_ONLY=1` and the unit's `ROS_DOMAIN_ID` into `~/.bashrc` as one managed block, and stops the `ros2` daemon so it does not keep serving the old domain. `--show` reports, `--revert` removes it |
| `scripts/demo_stack.py` | shared code for both roles: the unit lookup, the stack definitions, the phased readiness waits, the supervisor's state file, and the activity client that runs `run_activity` in the foreground so Ctrl+C reaches its cancel ladder |
| `scripts/start_demo_stack.py` | the stack supervisor: components, then coordination, each waited for; stays alive owning both. Profile from `AGX_UNIT`; `--stack tea\|block` for the two flows that need a different one; `--grippers` for `duo_gripper` on the bottom unit |
| `scripts/stop_demo_stack.py` | signals the supervisor named in the state file and waits for its teardown. Does not search for or kill ROS processes |
| `scripts/unpack_bottom_unit.py` / `pack_bottom_unit.py` | bottom unit between its packing pose and the presentation pose; `--speed fast\|slow` picks the path |
| `scripts/unpack_top_unit.py` / `pack_top_unit.py` | top unit between its packing pose and `Functional_Init_Both_V03` |
| `scripts/wave.py` | top unit: wave with both arms, entering and leaving on `Functional_Init_Both_V03`, so it runs between unpack and pack or on its own |
| `scripts/start_tea_demo.py` | runs `tea_pour_duo_v2` against the `tea` stack |
| `scripts/start_block_restack.py` | runs `block_restack_v1` against the `block` stack — the only flow that needs a parallel gripper on both arms |
| `scripts/prepare_can_interfaces.py` | role-based CAN or CAN FD preparation for USB or fallback adapter setups |
| `scripts/colcon_build_system_python.sh` | keeps builds on system Python and filters stale or conflicting local environment state |
| `scripts/run_in_ros_conda.sh` | runs a ROS command inside the repo-owned Conda runtime after sourcing ROS and local overlays |
| `scripts/setup_agx_arm_runtime_env.sh` | creates or updates the repo-owned Conda runtime environment |

For usage details and stable command lines, go back to `../../control/bringups/launches.md` or
`../../control/environment.md`.