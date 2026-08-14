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

For an arm bus this is combined with the arm stability flags (`restart-ms 100`, `one-shot on`).

**Current topology — one bus per device** (since 2026-08-13, `bus_topology: dedicated_per_device`
in `duo_motion_registry.yaml`). Bring all four up with `scripts/activate_duo_can.sh`:

| Interface | Parent device | Carries |
|---|---|---|
| `can_nero_right` | native mttcan `c310000.mttcan` | right arm (classic) |
| `can_nero_left`  | native mttcan `c320000.mttcan` | left arm (classic) |
| `hand_right`     | PEAK USB-CAN FD, USB port 1-4.3 | right OmniHand (FD/BRS) |
| `hand_left`      | PEAK USB-CAN FD, USB port 1-4.4 | left OmniHand (FD/BRS) |

The hand adapters are matched by **physical USB slot**, not by kernel `canN` index: the two are
identical hardware, so their indices can swap between boots, and a swap would point one hand's
commands at the other hand.

Superseded reading (before 2026-08-13): a CAN FD interface transmits both classic frames (to the
Nero arm) and FD+BRS frames (to the OmniHand), so one side bus carried the arm and its hand —
`can_nero_right` carried the right arm plus the right OmniHand, `can_nero_left` the left pair. That
remains physically true and is still selectable as the `shared_per_side` degraded mode, but it is no
longer the topology a bring-up produces. It cost the hand its arbitration under arm load; see the
window handshake in `../../control/bringups/teach_and_run.md`.

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

`one-shot on` makes every frame a single attempt and remains the stable shared-bus safety
baseline. It also applies to hand frames. Historical transport experiments used `ONE_SHOT=off` to
explore retransmission behavior, but that mode is not the recommended runtime baseline; use it only
for controlled offline investigation and follow `docs/errors_and_fixes.md` plus
`docs/control/bringups/teach_and_run.md` for the current operating policy.

## Hardware preflight (below ROS)

```bash
cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025
PYTHONPATH=$PWD/build/agibot_hand_pkg \
LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH \
OMNIHAND_SOCKETCAN_IFACE=can_nero_right \
python3.10 python/example/demo_get_hardware_info.py
```

## History / superseded

The Sprint 2 conclusion that *native Jetson mttcan cannot do CAN FD with BRS* was a
**transceiver/wiring limitation**, not a driver limit. With the TJA1051T/3 and the correct sysfs
TDCR setting, native FD+BRS works, so the ZLG-adapter path was dropped. The consolidated Sprint 2
history now lives in `docs/sprint2/evidence/omnihand_canfd_transport_history.md`.

## Open

- Bus-load budget when the arm and hand share one side bus — see
  `docs/sprint5/evidence/can_transport_decision.md`. Measure the
  hand's own load first with the vendor-level load test in
  `omnihand_solo_bringup_and_load_test.md`.

## Related

- Solo bring-up, ROS exerciser, and vendor-level load test:
  `omnihand_solo_bringup_and_load_test.md`.
