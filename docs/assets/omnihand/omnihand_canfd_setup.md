# OmniHand CAN FD Setup (current, validated)

The OmniHand Pro runs on the Jetson **native `mttcan`** CAN FD controller (40-pin header) with a
**BRS-capable 5 Mbit transceiver**. This is the current, working setup — control and firmware
read-out are confirmed. The ZLG USB-CANFD adapter is **not** used.

## Validated link configuration

```text
bitrate 1000000      sample-point 0.8
dbitrate 5000000     dsample-point 0.8
fd on
```

For the Duo side bus this is combined with the arm stability flags (`restart-ms 100`,
`one-shot on`). A CAN FD SocketCAN interface transmits **both** classic frames (to the Nero arm)
and FD+BRS frames (to the OmniHand), so one side bus carries the arm and its hand:

| Native iface | Side bus name | Carries |
|---|---|---|
| `can0` | `can_nero_right` | right arm (classic) + right OmniHand (FD/BRS) |
| `can1` | `can_nero_left`  | left arm (classic) + left OmniHand (FD/BRS) |

Bring it up with `scripts/activate_native_can.sh` (CAN FD is the default). Then point the hand SDK
at the same interface, e.g. `OMNIHAND_SOCKETCAN_IFACE=can_nero_right`.

`one-shot on` makes every frame a single attempt (the arm-stability fix). It also applies to hand
frames; if the hand needs retransmission, bring the bus up with `ONE_SHOT=off`.

## Hardware preflight (below ROS)

```bash
cd ~/workspace/agx_arm_ros/vendor/Omnihand-2025-SDK
PYTHONPATH=$PWD/build_phase1_socket/omnihand_2025_pkg \
LD_LIBRARY_PATH=$PWD/build_phase1_socket/omnihand_2025_pkg/omnihand_2025:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
python3.10 python/example/demo_get_hardware_info.py
```

## History / superseded

The Sprint 2 conclusion that *native Jetson mttcan cannot do CAN FD with BRS* was a
**transceiver/wiring limitation**, not a driver limit. With a proper 5 Mbit transceiver, native
FD+BRS works, so the ZLG-adapter path was dropped. The Sprint 2 investigation docs
(`docs/development/sprint2/*omnihand*`, `*canfd*`) are kept as history and point here.

## Open

- Bus-load budget when the arm and hand share one side bus — see
  `docs/development/sprint5/planning/can_transport_decision.md`.
