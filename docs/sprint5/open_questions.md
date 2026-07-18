# Sprint 5 Open Questions

Historical handoff questions from Sprint 5.

## Native bringup persistence

What should be the canonical reproducible way to bring up `can0` and `can1` with `one-shot on` and
`restart-ms`: a boot-time system service, a repo script, or both?

## Shared bus headroom

Does one arm plus its OmniHand fit within the real latency and bandwidth margin of one side bus
under the stable operating rule, or does the system need a stricter duty-cycle discipline?

## Recovery scope

Once native CAN is considered the stable baseline, should the node-side recovery watchdog remain as
defense-in-depth or be reduced to avoid masking real transport faults?