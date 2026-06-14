# Control Chain: Single-Arm vs. Duo Multi-Arm (Method & State Reference)

**Status:** Analysis · **Date:** 2026-06-12 · Companion to `nero_bus_problem_proposal.md`

Purpose: a human-usable map of *which method/topic talks to the `agx_arm_ctrl` driver, how
often, and in which state* — so we can see exactly where the single-arm path and the Duo
multi-arm path differ in interaction frequency, and why ENOBUFS shows up almost only in Duo.

---

## 0. TL;DR

- The two paths are **structurally identical per arm**. Duo = two copies of the single-arm
  stack, each in its own ROS namespace, each on its own CAN channel.
- **CAN TX load per channel is the same** in single and Duo. The only thing that writes CAN
  frames to an arm is the MIT command stream (`move_mit`, ~100 Hz × 7 joints). The
  `agx_arm_ctrl` publish thread at `pub_rate` reads **cached** feedback from the background RX
  thread — it issues **no** CAN requests.
- ENOBUFS is therefore **not** amplified by per-bus traffic or by wrong command routing. It is
  amplified by **shared resources**: one USB host serving two `gs_usb` adapters, and CPU
  contention starving the RX/echo-consume path. Both accelerate the `gs_usb` TX-echo slot leak
  (`nero_bus_problem_proposal.md` §2.1).

---

## 1. Legend

| Symbol | Meaning |
|---|---|
| `→` | ROS topic / action publish direction |
| `⇄ CAN` | actual CAN frame traffic to/from the arm hardware |
| `@N Hz` | steady-state frequency |
| `[cache]` | reads driver state updated by the background RX thread (no CAN request) |

Default rates (from `start_agx_arm_components.launch.py`):
`pub_rate = 200` (agx_arm_ctrl feedback), `mit_control_rate_hz = 100` (MIT command stream).

---

## 2. Who actually writes CAN frames to an arm

Only **command** paths produce CAN TX. Feedback is pulled from cache.

| Caller in `agx_arm_ctrl_single_node` | pyAgxArm call | CAN TX? | Frequency |
|---|---|---|---|
| `_move_mit_callback` | `move_mit()` ×7 joints (+ `set_motion_mode` once/mode) | **yes** | 100 Hz (MIT stream) |
| `_move_j/p/l/c/js`, `_joint_states_callback` | `move_j/move_js/...` | yes | one-shot user/MoveIt cmds |
| `_publish_thread` reads | `get_joint_angles/get_flange_pose/get_motor_states/get_arm_status/get_leader_joint_angles` | **no** `[cache]` | 200 Hz, RX/CPU only |
| `_enable_arm`, services | `enable/disable/reset/...` | yes | rare |

Key point: the 200 Hz publish thread is **RX/CPU work, not CAN TX**. `get_*` return
`self._parser.*` / cached `MessageAbstract` objects filled by the RX read thread
(`driver_context._read_loop`). So raising `pub_rate` costs CPU, not bus bandwidth.

---

## 3. Single-arm chain (e.g. `standalone` profile)

Processes: 1× `move_group` · 1× `mit_controller` · 1× `agx_arm_ctrl_single_node` (one CAN
channel, e.g. `can_nero`).

```
RViz/MoveIt plan
   → move_group: FollowJointTrajectory  (arm_controller/follow_joint_trajectory)
       → mit_controller._execute_follow_joint_trajectory  (samples trajectory)
           → mit_controller._control_loop  @100 Hz
               → publishes control/move_mit (MoveMITMsg, 7 joints)  @100 Hz
                   → agx_arm_ctrl._move_mit_callback
                       ⇄ CAN  move_mit ×7  @100 Hz  → arm
arm feedback frames  ⇄ CAN  → driver RX thread [cache]
   → agx_arm_ctrl._publish_thread @200 Hz  [cache reads, no TX]
       → feedback/joint_states, feedback/arm_status, feedback/tcp_pose, ...
           → mit_controller._feedback_callback (closes the MIT loop)
           → move_group current_state_monitor
```

CAN traffic on the one channel: ~700 cmd frames/s + arm feedback frames. One USB adapter.

---

## 4. Duo multi-arm chain (`duo_arm` profile, `both_arms`)

Processes: 1× `move_group` (group `both_arms`) · **2×** `mit_controller` · **2×**
`agx_arm_ctrl_single_node` · 1× `joint_state_merger` · 1× `duo_soft_estop`.
Per arm a full namespaced stack (`/left_arm/...`, `/right_arm/...`), each on its **own** CAN
channel (`can_nero_left`, `can_nero_right`).

```
RViz/MoveIt plan (both_arms)
   → move_group splits the trajectory across TWO controllers, executed SIMULTANEOUSLY:
   ├─ /left_arm/arm_controller/follow_joint_trajectory
   │     → /left_arm/mit_controller._control_loop @100 Hz
   │         → /left_arm/control/move_mit @100 Hz
   │             → /left_arm agx_arm_ctrl._move_mit_callback
   │                 ⇄ CAN(can_nero_left)  move_mit ×7 @100 Hz
   └─ /right_arm/arm_controller/follow_joint_trajectory
         → /right_arm/mit_controller._control_loop @100 Hz
             → /right_arm/control/move_mit @100 Hz
                 → /right_arm agx_arm_ctrl._move_mit_callback
                     ⇄ CAN(can_nero_right)  move_mit ×7 @100 Hz

feedback (per arm, namespaced, [cache] reads @200 Hz):
   /left_arm/feedback/joint_states  ─┐
   /right_arm/feedback/joint_states ─┤→ joint_state_merger → prefixed /feedback/joint_states
                                      └→ each mit_controller._feedback_callback
                                      └→ move_group current_state_monitor (both arms)
duo_soft_estop  watches both /…/mit_controller (cross-arm safe stop)
```

CAN traffic **per channel**: identical to single-arm (~700 cmd frames/s + feedback). What is
new in Duo lives **above** the per-channel level (see §5).

---

## 5. Where single and Duo differ (the ENOBUFS-relevant part)

| Aspect | Single | Duo | Bus impact |
|---|---|---|---|
| Command routing | 1 namespaced `move_mit` | 2 namespaced `move_mit`, no overlap | none — clean separation |
| CAN TX **per channel** | ~700 frames/s | ~700 frames/s | **identical** |
| CAN channels / USB adapters | 1 | 2 | **shared USB host** |
| `mit_controller` processes (100 Hz + pinocchio gravity/cycle) | 1 | 2 | CPU ↑↑ |
| `agx_arm_ctrl` publish threads @200 Hz [cache] | 1 | 2 | CPU ↑ (no TX) |
| Extra nodes | — | merger + soft-estop + both_arms planning | CPU ↑ |
| TX burst timing | independent | **both arms fire together** (both_arms) | synchronized USB spikes |

### Confirmed USB topology (lsusb -t, 2026-06-12)

```
Bus 01 (480M root hub, tegra-xusb)
  └─ Port 4: Hub (4-port, 480M)            ← shared hub
       ├─ Port 1: gs_usb @12M  → 1-4.1:1.0 → can2   (arm A)
       └─ Port 2: gs_usb @12M  → 1-4.2:1.0 → can3   (arm B)
```

Both arm adapters are two **separate** `gs_usb` netdevs but sit on **one** USB host (Bus 01)
behind **one** hub (1-4), both at **12 Mbit full-speed** → they share that hub's transaction
translator, which serializes their USB transactions. `ip -details link show` confirms
`can2`/`can3` come up with `txqueuelen 10` and `restart-ms 0` (defaults) — exactly what the P3
hardening fixes. The Jetson also exposes **native SoC CAN** (`can0`/`can1`, `c310000/c320000.mttcan`,
not USB) which would avoid the `gs_usb` slot leak and the shared TT entirely if the arms were
wired to on-board transceivers.

### Why Duo provokes ENOBUFS even though per-channel CAN load is unchanged

1. **Shared USB host/hub + `gs_usb` echo-slot leak (primary, confirmed shared path).** Both
   adapters hang off the same host and hub (above). The `gs_usb` "Unexpected unused echo id"
   accounting bug leaks hardware TX slots; it is a race that worsens with concurrent TX across
   two adapters sharing one transaction translator and higher USB IRQ load → slots leak faster
   → permanent ENOBUFS reached sooner. Single-arm leaks slowly enough to rarely surface in one
   session.
2. **CPU contention starves the RX/echo-consume thread.** A TX slot is freed only when the
   driver RX thread consumes the echo/ack frame. Duo roughly doubles CPU load (2× MIT with
   per-cycle pinocchio gravity, 2× 200 Hz publish threads, both_arms planning, merger, estop,
   rviz). If the RX thread is starved on the Jetson, echo frames pile up → slots are not freed
   → ENOBUFS. Single-arm load leaves enough headroom.
3. **Synchronized bursts.** `both_arms` makes both controllers stream in lock-step, so the
   worst-case USB-host and echo-handling pressure on the two adapters coincides.

Conclusion: the amplifier is **shared host + CPU concurrency**, not command misrouting and not
per-channel bandwidth. This refines the proposal: Duo does not add per-bus traffic; it adds
shared-host concurrency that accelerates the existing `gs_usb` slot leak.

---

## 6. Relevant state machines

### `agx_arm_ctrl_single_node` control/recovery gate

```
startup → control_ready=False  (warm-up, all control callbacks return early)
   on first valid feedback in _publish_thread → control_ready=True, _had_control_ready=True
running:
   every control callback gated by _check_can_control() (needs control_ready + enable + ready)
   _move_mit_callback sets motion mode once per transition (_current_motion_mode)
P1 bus-stall watchdog (in _publish_thread, guarded):
   _should_recover_bus() true on ANY of:
     - _tx_stall_detected      (send raised ENOBUFS/ENETDOWN — raise-style comm)
     - has_comm_error()+classify (comm swallowed ENOBUFS — last_error-style comm)
     - not is_ok()  OR  feedback older than feedback_timeout
   → _recover_bus(): control_ready=False → disconnect → [optional ip link down/up]
       → connect → re-enable → reset motion mode/stall flags → wait fresh feedback
       → publish thread flips control_ready=True again
```

### `mit_controller` execution states (per arm)

```
DISABLED → ARMING (gain ramp) → IDLE_HOLD
IDLE_HOLD → EXECUTING_TRAJECTORY (FollowJointTrajectory) → HOLDING_FINAL_POINT → IDLE_HOLD
guards → LEADER_MODE | FAULTED | STALE_FEEDBACK (pause MIT publish)
path safety → PATH_TOLERANCE_VIOLATED aborts the goal (and, under both_arms, PREEMPTS the
              partner arm)
```

---

## 7. Practical checks to confirm the shared-resource hypothesis

- `lsusb -t` — **done (2026-06-12): confirmed both on Bus 01 → hub 1-4 @12M full-speed.**
- During a Duo run: `dmesg -w | grep -i "echo id"` — leak rate vs. single-arm.
- `watch -n0.2 'ip -s -details link show can_nero_left'` and `…_right` — TX errors / state.
- `top`/`htop` during Duo execution — RX-thread / mit_controller CPU saturation.
- Mitigations to weigh: move adapters to separate USB hosts; cap `pub_rate`; offload pinocchio
  gravity; the `gs_usb`/firmware fixes in proposal §P2.
