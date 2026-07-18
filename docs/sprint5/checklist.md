# Sprint 5 Checklist

Historical closure summary for Sprint 5.

## Established in Sprint 5

- [x] Diagnose the Duo ENOBUFS root cause as shared USB host or hub pressure plus `gs_usb`
	echo-slot leakage.
- [x] Confirm the shared-USB topology with `lsusb -t`.
- [x] Measure single-arm bus load and reject the idea of two arms on one bus.
- [x] Migrate the arm baseline to native `mttcan` with `one-shot on`.
- [x] Resolve control-layer environment drift and pin `vendor/pyAgxArm`.
- [x] Keep the node-side bus-recovery watchdog only as defense-in-depth.

## Handed off beyond Sprint 5

- [x] hardware validation of repeated plan-and-execute cycles remained for later sessions
- [x] persistent native bringup policy remained an open operational question
- [x] the exact arm-plus-hand shared-bus budget still needed explicit measurement and later safety guidance