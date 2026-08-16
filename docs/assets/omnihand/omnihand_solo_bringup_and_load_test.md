# OmniHand Solo Bring-up, ROS Exerciser, and Load Test

> **Superseded in part — V02 refactor.** Each device now has its own CAN bus
> (arms `can_nero_left`/`can_nero_right` native, hands `hand_left`/`hand_right`
> on USB-CAN FD adapters), so
> same-side arm and hand motion may run in parallel and the shared-bus hand
> window is a selectable degraded mode, not normal operation. This page still
> describes the **current code**, which resolves the hand interface from the arm
> bus; it is rewritten in phase 2A. See
> `docs/sprint_refactor/planning/integration_plan.md` (constraint C1).

Two workflows for working with the OmniHand on its own, below and at the ROS layer:
a **vendor-level communication load test** (for measuring the hand's CAN FD bus
load) and a **ROS solo bring-up + exerciser** (for trying the bridge's
ROS-wrapped functionality, e.g. grasp/skill bring-up).

Prerequisite for both real-hardware paths: the side bus is up with the validated
CAN FD timing and TDCR. See `omnihand_canfd_setup.md`.

## 1. Vendor-level communication load test (below ROS)

The vendor SDK ships only one-shot demos, so there was no way to measure the
hand's steady-state load. `scripts/omnihand/omnihand_load_test.py` holds **one
persistent connection** and drives the same call mix the ROS bridge uses in
normal operation (50 Hz joint readback, 1 Hz status, optional tactile and a
small command sweep), for a fixed duration, printing achieved call rates and
per-call latency.

Run it with a bus capture alongside so the SDK call rate can be compared to the
actual frame rate / utilization on the wire:

```bash
# terminal 1 — capture the side bus
candump -l can_nero_right                       # candump-*.log
# or for Wireshark:
sudo tcpdump -i can_nero_right -w ~/omnihand_load.pcap

# terminal 2 — sustained load (read-only by default)
cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025
PYTHONPATH=$PWD/build/agibot_hand_pkg \
LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
python3.10 ~/workspace/agx_arm_ros/scripts/omnihand/omnihand_load_test.py \
    --hand-type right --duration 10
```

Useful flags:

- `--rate` joint-readback Hz (default 50, matches bridge `pub_rate`)
- `--status-rate` error-report Hz (default 1)
- `--tactile` also read tactile per finger (adds 5 calls/cycle)
- `--with-commands` also exercise the write path with a small clamped sweep —
  **this moves the hand.** It sweeps one finger (index PIP) around the current
  pose; if the firmware's active-joint readback is incomplete (it returns a
  padded 12-value vector that the SDK rejects), it falls back to the **open**
  pose as reference, so the hand first moves to open. It restores open on exit.
- `--duration`, `--report-interval-s`

The `Unexpected actuator vector size: expected 10, got 12` line printed by the
vendor SDK at startup under `--with-commands` is benign: it is the SDK's own
active-joint readback rejecting a padded vector, which the load test detects and
handles by using the open pose as the sweep reference.

This is the measurement to close the **"measure hand load first"** open item in
`docs/sprint5/evidence/can_transport_decision.md` before sharing a
side bus between the arm and its hand. Measure the hand alone first, then the
arm alone (`logs/arm.pcap` is the existing arm baseline), then the sum.

## 2. ROS solo bring-up + exerciser

The bridge has a standalone launch that starts **only** the bridge node:

```bash
# real hand — no PYTHONPATH/LD_LIBRARY_PATH export needed, the bridge
# auto-locates the repo's built vendor SDK:
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
    backend_type:=sdk omnihand_type:=right
# mock (no hardware), for wiring/topic tests:
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py backend_type:=mock
```

`namespace` defaults to `auto`, i.e. the side namespace from
`duo_motion_registry.yaml` (`right` → `right_arm`) — the same namespace the Duo
bringup and `omnihand_exerciser` use, so a standalone bridge is addressable by
the repo's own tooling without extra arguments. Pass `namespace:=''` for the
old root-namespace behaviour.

Do **not** prefix the launch with `PYTHONPATH=...`: the inline form *replaces*
ROS's own `PYTHONPATH` and breaks `ros2` itself (`PackageNotFoundError: ros2cli`).
The bridge finds the vendor SDK on its own (upward search for the repo's
`build/agibot_hand_pkg`; the compiled `.so` uses an `$ORIGIN`
runpath, so no `LD_LIBRARY_PATH` is required). Override with the
`sdk_python_dir:=<path>` launch arg or `AGX_ARM_OMNIHAND_SDK_DIR` env var only if
the SDK lives outside the repo.

### SocketCAN interface selection

The vendor SDK picks its CAN interface **only** from the `OMNIHAND_SOCKETCAN_IFACE`
env var (default `can0`); the numeric `canfd_id` applies to the ZLG USB adapter
path, not native SocketCAN. The bridge resolves the native side-bus name per side
from `agx_arm_ctrl/config/omnihand_can_interfaces.yaml` and exports it before
opening the hand:

| `omnihand_type` | interface (from config) |
|---|---|
| `right` | `can_nero_right` |
| `left`  | `can_nero_left`  |

These are the names `scripts/activate_native_can.sh` creates. Change them in that
config file, or override per launch:

```bash
ros2 launch agx_arm_ctrl start_omnihand_bridge.launch.py \
    backend_type:=sdk omnihand_type:=right can_interface:=can_nero_right
```

The startup log prints the resolved interface and its source, e.g.
`OmniHand SocketCAN interface: can_nero_right (from .../omnihand_can_interfaces.yaml)`.

It exposes the bridge's full ROS surface on its own:

- subscribes `control/joint_states` (shared) and `control/omnihand/joint_trajectory`
- publishes `feedback/omnihand/joint_states`, `feedback/omnihand/status`,
  `feedback/omnihand/tactile_raw`
- service `control/omnihand/stop` — cancels the pending target and holds the current
  pose. It does not latch: the next command re-arms the hand.

To actually *drive* it, use the exerciser (`ros2 run agx_arm_ctrl omnihand_exerciser`).
It sends named active-joint poses over the same path MoveIt uses — the per-side
`FollowJointTrajectory` action (falling back to the bridge's
`control/omnihand/joint_trajectory` topic when the action server is not up) — and
can call stop. Both it and the standalone launch resolve the same Duo side namespace
(`left_arm`/`right_arm`) from the motion registry, so no namespace argument is needed:

```bash
ros2 run agx_arm_ctrl omnihand_exerciser --list
ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --side right --gesture fist
ros2 run agx_arm_ctrl omnihand_exerciser --model o12_pro --side left --gesture zero
# only when the bridge was launched with namespace:='' (or an unnamespaced
# start_single_agx_arm) — the topic fallback also finds that case on its own:
ros2 run agx_arm_ctrl omnihand_exerciser --namespace '' --side right --gesture fist
```

If the `FollowJointTrajectory` action server is not up (standalone bridge without the
trajectory node), the exerciser falls back to the `control/omnihand/joint_trajectory`
topic — on whichever of the namespaced or root topic a bridge is actually **subscribed**
to, and it logs an ERROR when neither is. It used to publish into the registry namespace
regardless, which was a silent no-op against a root-namespace bridge. That fallback path
has no arm↔hand window and no delivery verification.

Watch feedback in another terminal:

```bash
ros2 topic echo /right_arm/feedback/omnihand/joint_states  # standalone + Duo bringup
ros2 topic echo /right_arm/feedback/omnihand/status
ros2 topic echo /feedback/omnihand/joint_states     # only with namespace:=''
```

The poses are the vendor SDK active-joint presets (from the SDK
`demo_set_motion.py`); they are tuned for a specific hand side, but the bridge
clamps every target to the selected side's joint limits, so they are safe on
either side. `open` (all zeros) is side-agnostic and the safe default. In the
default action mode delivery is verified by the bridge's readback-based retry;
the legacy `--topic` mode republishes JointState commands at `--rate` while
holding a pose, so it still doubles as a ROS-side command-traffic generator for
combined load tests.

## Notes

- The compiled SDK lives in the **built** package
  (`build/agibot_hand_pkg`), not the source `python/` tree. Its
  `.so` uses an `$ORIGIN` runpath, so once that directory is on `sys.path` the
  import works — **no `LD_LIBRARY_PATH` is actually required**.
- The **ROS launch** auto-locates the built package; run it with no manual env.
  The launch passes `device_id`/`canfd_id`/`sdk_cfg_path`/`sdk_python_dir`
  through to the bridge.
- The **load-test script** respects an already-set `PYTHONPATH` and self-heals if
  it is unset or points at the source tree (it falls back to the built package).
  The documented `PYTHONPATH=...`/`LD_LIBRARY_PATH=...` prefix on that *script*
  is harmless because it is a dedicated `python3.10` invocation, not `ros2`.
- Never prefix a `ros2` command with an inline `PYTHONPATH=...`: it replaces
  ROS's `PYTHONPATH` instead of appending, the same "PYTHONPATH shadowing"
  failure described in `docs/control/environment.md`.
- The load test is read-only unless `--with-commands` is given, and restores the
  baseline pose on exit.
- The exerciser only sends commands the bridge already accepts; it adds no new
  ROS contract.
