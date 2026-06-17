# Sprint 5 — Checklist

## Done

- [x] Diagnose the Duo ENOBUFS root cause (shared USB host/hub + `gs_usb` echo-slot leak; per-channel CAN load is unchanged vs single-arm).
- [x] Confirm the shared-USB topology with `lsusb -t` (both adapters on Bus 01 → hub 1-4 @12M FS).
- [x] Measure single-arm bus load from `logs/arm.pcap` (~2830 frames/s ≈ 31–37 % @1 Mbit).
- [x] Migrate the arms to native `mttcan` (`can0`/`can1`, 40-pin header) with `one-shot on` → stable.
- [x] Resolve the control-layer env-drift: editable pyAgxArm in system 3.10.
- [x] Pin pyAgxArm as the `vendor/pyAgxArm` submodule (`control-layer-pin-2026-06-12`).
- [x] Add a node-side bus-recovery watchdog in `agx_arm_ctrl` (defense-in-depth).

## To do

- [ ] Hardware-validate multiple consecutive plan&execute cycles on native CAN (no restart needed).
- [ ] Persist the native bringup (`one-shot on`, `restart-ms`) — script or systemd, not manual `ip link`.
- [ ] After validation: tag pyAgxArm `hw-validated-<date>` and bump the submodule pin.
- [ ] Add `set_start_state_to_current_state()` + joint-state freshness + joint1 ±π unwrap in the planning path.
- [ ] Evaluate **arm + its hand on one bus** against the ~30 % per-arm budget (see planning doc).
- [ ] Decide whether to keep or simplify the node-side recovery now that native CAN is stable.
