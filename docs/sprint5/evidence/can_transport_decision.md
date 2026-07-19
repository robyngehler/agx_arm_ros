# CAN Transport Decision (Duo system)

## Decision

- Run one native `mttcan` side bus per side (`can0` -> `can_nero_right`, `can1` ->
  `can_nero_left`) in CAN FD mode with `one-shot on` and `restart-ms`.
- Put one arm and its own hand on the same side bus.
- Do not put two arms on one CAN bus.
- Keep the arm-plus-hand bus budget as an explicit validation topic rather than assuming it is free.

## Why native CAN plus `one-shot on`

The USB `gs_usb` adapters wedged with permanent ENOBUFS on the Duo path. Two main drivers were
identified:

1. shared USB host or hub pressure and the `gs_usb` echo-slot accounting leak
2. endless retransmit behavior for unacknowledged frames when `one-shot` was not available

Moving to native `mttcan` removed the USB path entirely, and `one-shot on` prevented retransmission
buildup from wedging the bus.

## Native bringup reference

Use `scripts/activate_native_can.sh` as the normal path. The script applies the required sysfs TDCR
step before interface bringup and configures the side buses for the current stable baseline.

## Bus budget summary

- one arm on one bus measured at roughly 31–37% utilization at 1 Mbit
- two arms on one bus were rejected
- one arm plus one hand on one bus remained a deliberate validation topic rather than an assumed
  safe default

## Historical value

Keep this note only for the decision rationale behind the native CAN migration and the rejection of
two arms on one bus. The current operational guidance lives in `../../control/bringups/launches.md`,
`../../control/bringups/teach_and_run.md`, and `../../assets/omnihand/omnihand_canfd_setup.md`.