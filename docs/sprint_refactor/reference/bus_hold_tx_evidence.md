# Held Bus: Why TX Progress Cannot Be Read From `tx_packets`

*Hardware evidence, left arm (`can_nero_left`, mttcan, ONE-SHOT + FD), 2026-09-03.
Measurement: `docs/sprint_refactor/reference/measurements/estop_bus_hold/20260903_133153/`,
captured with `scripts/measure_estop_bus_hold.py`.*

## The claim

`/sys/class/net/<iface>/statistics/tx_packets` **does not count a frame that is
transmitted but never acknowledged.** On a bus the external watchdog has taken,
that is every frame, so the counter freezes at exactly the moment the hold
begins — the moment a held-bus classifier needs it most.

## The measurement

An emergency stop was triggered mid-trajectory with the arm streaming. The
tcpdump capture is `LINUX_SLL`, so each frame carries its direction (packet type
4 = outgoing):

| second | OUT | IN | phase |
| --- | --- | --- | --- |
| 11 | 1183 | 2274 | healthy |
| 12 | 1283 | 83 | watchdog takes the bus |
| 13 | 1344 | 0 | host still transmitting at full rate |
| 14 | 1066 | 0 | last outgoing frame at +14.77 s |
| 15-21 | 0 | 0 | recovery owns the session |
| 22+ | 0 | ~2150/s | arm back, host never transmits again |

**3589 frames left the host at ~1320/s during the hold while `tx_packets` did
not move.** The transmit error counter went 0 -> 128 (the error-passive ceiling)
within ~100 ms of the peer going away and stayed there, because it decrements
only on a *successful* transmission.

## What follows

- A transmit-side liveness signal must come from the **transmit error counter**
  (netlink, `ip -d link show`), not from `tx_packets`. The counter is a level,
  not an edge, so it stays raised while the command stream is gated off — which
  is what a hold does to it.
- Keeping a command stream running is **not** an operational workaround for a
  classifier built on `tx_packets`. The stream was running in this measurement.
- A pcap settles TX direction; `tx_packets` does not. Take direction from the
  SLL packet type whenever the link may be unacknowledged.

## Consequence in the code

`_bus_hold_defers_recovery()` in `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
entered the hold only if `tx_packets` had advanced since the previous sample, so
the hold could never be entered during a watchdog stop. Recovery ran against a
bus that was never broken and latched the fault lockout that refuses motion
after the release. The entry condition now reads the transmit error counter
(`bus_hold_min_tec`, default 8 — one failed frame), falling back to the packet
edge only where netlink cannot be read.

Replayed against this recording: the old condition never entered the hold; the
new one enters at +11.72 s and releases at +22.09 s, the instant RX returns.

## Still open

After recovery took the session at +14.77 s, **the driver transmitted nothing
for the remaining 25 s**, across several `enable` attempts that each reported
`still pending after 2.0s`. The arm's feedback returned at +22.70 s at full rate
and never reached ROS (`move_group` kept reporting the joint state from the
instant of the stop). Recovery is a one-way door here and the cause is not
established. It is out of the path once the hold is entered, but it remains the
behaviour of every recovery that does run.
