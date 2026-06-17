# Sprint 5 — Stable CAN Transport & Control-Layer Pinning

**Target.** Make the Duo runtime transport-stable and reproducible:

1. Move the arms off the shared USB `gs_usb` adapters onto the Jetson **native CAN**
   (`can0`/`can1`, `mttcan`, 40-pin header) with **`one-shot on`** — the fix that removed the
   ENOBUFS bus stalls.
2. Pin the control-layer SDK (`pyAgxArm`) so the runtime source can no longer silently drift.
3. Establish the **one-bus-per-arm** baseline and evaluate sharing **arm + its hand** on one bus
   (a single arm measures ~30 % bus load at 1 Mbit, so two arms per bus is rejected).

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
