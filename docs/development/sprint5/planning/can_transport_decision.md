# CAN Transport Decision (Duo system)

## Decision

- Run each **arm on a dedicated native `mttcan` channel** (`can0`/`can1`, Jetson 40-pin header),
  brought up with **`one-shot on`** (and `restart-ms` for bus-off recovery).
- **Do not** put two arms on one CAN bus.
- **Next step:** evaluate **arm + its own hand on one shared bus**, not two arms per bus.

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

### Native bringup (reference)

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 one-shot on
sudo ip link set can0 up
# repeat for can1
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
