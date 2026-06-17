# Sprint 5 — Open Questions

- **Arm + hand on one bus?** A single arm uses ~30 % of a 1 Mbit bus. Does an OmniHand's command
  + feedback traffic fit within the remaining headroom with safe margin and acceptable latency,
  or does the hand need its own channel? Measure hand load the same way (`tcpdump`/pcap) before
  committing. Two arms on one bus is already rejected (~60–74 % + arbitration contention).
- **Native bringup persistence.** What is the canonical, reproducible way to bring up `can0`/`can1`
  with `one-shot on` + `restart-ms` (systemd-networkd, a boot script, or a repo script)? The
  current USB-oriented `can_*activate.sh` do not cover the native `mttcan` path.
- **Recovery scope.** With native CAN + `one-shot on` removing the ENOBUFS root cause, should the
  node-side bus-recovery watchdog stay as defense-in-depth or be trimmed to avoid masking real faults?
- **Planning robustness.** Is solver-side TracIK `Distance` plus a start-state freshness/unwrap
  guard enough to eliminate the joint1 ~π jumps, or is a broader shared-state adapter still needed?
