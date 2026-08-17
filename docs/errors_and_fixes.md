# Global Errors And Fixes

Cross-cutting issues that already produced confusion or wasted debugging time.

## A kernel update discards the Jetson 40-pin header config, killing CAN TX

**Check this before diagnosing any silent arm bus.**

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

The native arm interfaces (`can_nero_left`, `can_nero_right`) ride the 40-pin
header. A kernel update resets its pinmux, after which `mttcan` still presents
both interfaces as UP and ERROR-ACTIVE while nothing can be transmitted.

Signature: `RX=0 TX=0` in `ip -s link show`, sends failing with `ENOBUFS`
("Transmit buffer full"), and `berr-counter tx 0`. The error counters are not a
discriminator — the interfaces run in ONE-SHOT mode, which aborts an
unacknowledged frame rather than retrying it into error-passive. Read TX
*packets*.

The hands are on USB-CAN FD adapters and do not use the header, so a healthy
hand bus in the same session does not rule this out.

Happened 2026-08-11 (header never configured) and again 2026-08-17 (kernel
update). The second occurrence cost most of a session and produced two wrong
diagnoses. Detail and the driver-side diagnostic:
`docs/sprint_refactor/errors_and_fixes.md`.

## Native CAN naming versus legacy public naming

Problem: docs and examples mixed native Jetson side-bus names with USB role names.

Current fix:

- native Duo bringup uses `scripts/activate_native_can.sh`
- native side buses are `can_nero_right` and `can_nero_left`
- old public runtime names such as `can0` and `can_nero` are deprecated
- the USB `nero` role now also targets `can_nero_right` by default to stay aligned with the current single-arm baseline

See `control/bringups/launches.md` and `CAN_USER_EN.md`.

## pyAgxArm source drift

Problem: the docs did not clearly separate the pinned repo runtime source from the external
pyAgxArm development checkout.

Current fix:

- `scripts/setup_agx_arm_runtime_env.sh` now installs `vendor/pyAgxArm` first
- it falls back to `../pyAgxArm` only when the vendored checkout is unavailable
- the sibling or external `pyAgxArm` checkout remains the place for upstream pulls, local SDK
	changes, tagging, and preparing the next `vendor/pyAgxArm` pin

See `control/environment.md` and `project/control_layer_and_dependencies.md`.

## Build Python versus runtime Python

Problem: mixing Conda and ROS build shells hides ROS dependencies and creates false failures.

Current fix:

- use `scripts/colcon_build_system_python.sh` for workspace builds
- keep `colcon test` on a system-Python ROS shell
- use `scripts/run_in_ros_conda.sh -- <command>` for Conda-backed runtime commands
- append to `PYTHONPATH`; do not replace it

See `control/environment.md`.

## Shared arm-plus-hand CAN saturation and unsafe recovery

Problem: the current shared-bus runtime still has unsafe failure modes when arm and OmniHand traffic
compete on the same side bus.

Current findings:

- the Stall-CAN detection in `agx_arm_ctrl` depends too much on a timer, so heavy CPU load can
	make the timer stall and trigger a CAN reset even when the bus is still working
- the current probing and recovery path is incomplete and can worsen downstream failure handling
- `ONE_SHOT=off` can let hand and arm traffic progress in parallel, but it is not a stable fix; it
	reintroduces retransmission buildup risk on the arm side
- missing ACK can trigger retry spam and bus overflow, which can also take down the arm path while
	the last commanded arm behavior remains active until CAN is brought down and up and the
	controller is restarted

Current safe guidance:

- the current stable operating rule is: keep `one-shot on`, keep the arm active while the arm is
	being controlled, and switch to explicit hand-command windows only after the arm has settled into
	a safe static hold
- keep `one-shot on` as the default arm-stable baseline
- treat `ONE_SHOT=off` only as a historical or offline transport experiment, not as a recommended runtime mode
- prefer explicit hand-command windows where active arm control is paused or frozen while the arm is
	already in a safe static hold, instead of sustained concurrent arm and hand command pressure
- reduce hand-side error spam and unnecessary shared-bus traffic; the bridge `joint_read_rate` is a
	real CAN lever, while ROS `pub_rate` is not
- after a missing-ACK or overflow event, do not resume motion until the CAN interface has been
	cycled and the affected controller stack has been restarted

Status: stable fix pending, but the current operating policy is settled for safety reasons. See
`control/bringups/teach_and_run.md` for the current shared-bus runtime guidance and
`target/README.md` for the repo-level policy tracking.

## Implicit wrapper defaults

Problem: wrapper examples that omit `execution_profile` fall back to `manual`, which is not the current recommended operational path.

Current fix:

- package README examples now set explicit profiles such as `right_arm`, `right_hand`, or `duo_arm`
- operational launch matrices stay in `control/bringups/launches.md`

See `control/bringups/launches.md`, `src/agx_arm_moveit/README_EN.md`, and `src/agx_arm_mit_controller/README.md`.