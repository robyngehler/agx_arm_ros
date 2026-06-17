# Sprint 5 — Stable CAN Transport & Control-Layer Pinning

**Target.** Make the Duo runtime transport-stable and reproducible:

1. Move off the shared USB `gs_usb` adapters onto the Jetson **native CAN FD** side buses
   (`can0` → `can_nero_right`, `can1` → `can_nero_left`, `mttcan`, 40-pin header) with
   **`one-shot on`** — the fix that removed the ENOBUFS stalls. With a 5 Mbit BRS-capable
   transceiver, one side bus carries the arm (classic) **and** its OmniHand (FD/BRS).
2. Pin the control-layer SDK (`pyAgxArm`) so the runtime source can no longer silently drift.
3. Confirm the bus-load budget for arm + hand sharing one side bus (a single arm measures ~30 % at
   1 Mbit). Two arms per bus stays rejected.

This sprint follows the Duo body integration (sprint4). It is transport/runtime stabilization,
not description or planning work.

## Working files

- `checklist.md` — concrete steps and their status.
- `errors_and_fixes.md` — the ENOBUFS / env-drift / config-flip findings and how they were resolved.
- `open_questions.md` — what is still undecided.
- `planning/can_transport_decision.md` — native-CAN decision, `one-shot` rationale, bus budget.

## Durable references (not sprint-local)

- Control chain & shared-bus analysis: `docs/assets/control/single_vs_multi_arm_control_chain.md`
- Control-layer source / submodule: `docs/project/control_layer_and_dependencies.md`

## Control Scripts & Commands:
Set up the jetson native canX ports for downstream usage
```bash
sudo ip link set can0 name can_nero_right
sudo ip link set can1 name can_nero_left
sudo ip link set can_nero_right type can bitrate 1000000 berr-reporting on restart-ms 100 one-shot on
sudo ip link set can_nero_left type can  bitrate 1000000 berr-reporting on restart-ms 100 one-shot on
sudo ip link set can_nero_right up
sudo ip link set can_nero_left up
```
**TODO:** create .sh script like activate_can.sh is used for usb-native usage.
