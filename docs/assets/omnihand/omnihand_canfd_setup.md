# OmniHand CAN FD Setup (current, validated)

The OmniHand Pro runs on the Jetson **native `mttcan`** CAN FD controller (40-pin header) with a
**BRS-capable 5 Mbit transceiver**. This is the current, working setup — control and firmware
read-out are confirmed. The ZLG USB-CANFD adapter is **not** used.

## Hardware

**Transceiver: Adafruit CAN Pal (TJA1051T/3)** — this specific transceiver is confirmed working at
5 Mbit/s CAN FD with BRS. Both the **Nero arm** (classic CAN, 1 Mbit) and the **OmniHand**
(CAN FD, 5 Mbit, BRS) run through this single transceiver on the Jetson 40-pin header.

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

## Mandatory prerequisite: TDC offset (TDCR)

The TJA1051T/3 requires the **Transmitter Delay Compensation offset to be set via sysfs** before
the interface is brought up. Without this, CAN FD frames with BRS will fail at 5 Mbit/s.

Set the TDC offset **while the interface is DOWN**:

```bash
# can0 (right side)
echo "0x800" | sudo tee /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset

# can1 (left side)
echo "0x800" | sudo tee /sys/devices/platform/bus@0/c320000.mttcan/net/can1/tdc_offset
```

Verify:

```bash
cat /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset
cat /sys/devices/platform/bus@0/c320000.mttcan/net/can1/tdc_offset
```

The value `0x800` is validated for the TJA1051T/3 (Adafruit CAN Pal). A different transceiver
may need a different value; override with `TDCR_VALUE=0x...` when calling the activation script.

**`scripts/activate_native_can.sh` handles this automatically** — the sysfs TDCR step is
performed before `ip link set ... up`.

### Dead-end approaches (do not use)

Two alternative TDCR approaches were investigated and confirmed not to work:

- **devmem via register `0xC310048`** — writes appear to succeed but the register state has no
  observable effect on BRS behavior. Confirmed dead end.
- **Custom DTB + `extlinux.conf` boot entry** (patching `prod_c_can_5m` in the device tree,
  adding `JetsonIO-CAN-TDCR-*` labels, switching `DEFAULT`) — also confirmed dead end; the
  boot-time DTB TDCR value does not carry through to the interface behavior in practice.

The sysfs `tdc_offset` write is the **only confirmed working method**.

## Bring up

Use `scripts/activate_native_can.sh` (CAN FD is the default). It sets TDCR, configures
the interface, renames it, and brings it up:

```bash
sudo bash scripts/activate_native_can.sh          # both sides
sudo bash scripts/activate_native_can.sh right    # right side only
```

Equivalent manual sequence (right side):

```bash
sudo ip link set can0 down
echo "0x800" | sudo tee /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    dbitrate 5000000 dsample-point 0.8 fd on restart-ms 100 one-shot on
sudo ip link set can0 name can_nero_right
sudo ip link set can_nero_right up
```

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
**transceiver/wiring limitation**, not a driver limit. With the TJA1051T/3 and the correct sysfs
TDCR setting, native FD+BRS works, so the ZLG-adapter path was dropped. The Sprint 2
investigation docs (`docs/development/sprint2/*omnihand*`, `*canfd*`) are kept as history and
point here.

## Open

- Bus-load budget when the arm and hand share one side bus — see
  `docs/development/sprint5/planning/can_transport_decision.md`. Measure the
  hand's own load first with the vendor-level load test in
  `omnihand_solo_bringup_and_load_test.md`.

## Related

- Solo bring-up, ROS exerciser, and vendor-level load test:
  `omnihand_solo_bringup_and_load_test.md`.
