# CAN Transport Decision (Duo system)

## Decision

- Run one **native `mttcan` side bus per side** (`can0` → `can_nero_right`, `can1` →
  `can_nero_left`, Jetson 40-pin header), brought up in **CAN FD** mode with `one-shot on` and
  `restart-ms`. A 5 Mbit BRS-capable transceiver is required.
- Put the **arm and its own hand on the same side bus**: an `fd on` SocketCAN interface carries
  both classic (arm) and FD+BRS (hand) frames. See
  `../../../assets/omnihand/omnihand_canfd_setup.md`.
- **Do not** put two arms on one CAN bus.
- **Open:** confirm the bus-load budget holds when arm + hand share one bus (below).

## Why native CAN + one-shot

The USB `gs_usb` adapters wedged with permanent ENOBUFS on the Duo path (see
`../errors_and_fixes.md`). Two failure drivers, both removed by going native:

1. **Shared USB host/hub.** `lsusb -t`: both adapters on Bus 01 behind hub 1-4 at 12 Mbit
   full-speed → one transaction translator serializing both, plus the `gs_usb` echo-slot
   accounting leak under concurrent load. Native `mttcan` is on-SoC: no USB, no TT, no `gs_usb`.
2. **Endless retransmit of unacked frames.** Without `one-shot`, a CAN controller retransmits an
   unacknowledged frame indefinitely, spamming the bus and filling the TX queue → ENOBUFS.
   **`one-shot on`** makes each frame a single attempt: a lost frame is dropped (the next 100 Hz
   MIT frame supersedes it) instead of wedging the bus. `mttcan` advertises `one-shot`; the
   `gs_usb` firmware did not — which is why the same mitigation was impossible on USB.

Native `mttcan` is BRS-capable with a proper 5 Mbit transceiver — the earlier "native cannot do
BRS" Sprint 2 finding was a transceiver limitation, since resolved (see
`../../../assets/omnihand/omnihand_canfd_setup.md`).

### Native bringup (reference)

Use `scripts/activate_native_can.sh` (CAN FD side buses by default), equivalent to:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    dbitrate 5000000 dsample-point 0.8 fd on restart-ms 100 one-shot on
sudo ip link set can0 name can_nero_right
sudo ip link set can_nero_right up
# repeat: can1 -> can_nero_left
```

## Bus budget (measured)

From `logs/arm.pcap` (one arm, single MoveIt execution): ~2830 CAN frames/s over 10.8 s →
**~31–37 % utilization at 1 Mbit** (108–130 bit/frame incl. stuffing/IFS). Matches the observed
~30 %. Implications:

| Layout | Estimated load | Verdict |
|---|---|---|
| 1 arm / bus | ~30–35 % | baseline, healthy margin |
| 2 arms / bus | ~60–74 % + arbitration | **rejected** — no margin, latency/priority contention |
| arm + hand / bus | ~30 % + hand TBD | **to evaluate** — measure hand load first |

The arm command stream is the dominant load (MIT `move_mit`, ~700 frames/s/arm plus feedback).
Before sharing a bus with a hand, measure the hand's command+feedback rate the same way and keep
total utilization with margin (target well under ~70 %) to preserve arbitration latency.
