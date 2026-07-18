# Sprint 5 Target

status: HISTORICAL_ENTRYPOINT
last_updated: 2026-07-18

Sprint 5 was the transport-stabilization phase for the Duo runtime.

## Main goal

Make the arm-plus-hand runtime transport-stable and reproducible by:

- moving from shared USB `gs_usb` adapters to native Jetson `mttcan`
- keeping `one-shot on` as the arm-stable baseline
- pinning the control-layer SDK so runtime source drift stops
- understanding whether one arm plus its hand can safely share one side bus

## What Sprint 5 settled

- the real ENOBUFS root cause was the shared USB topology plus `gs_usb` echo-slot behavior
- native `mttcan` plus `one-shot on` became the stable arm-side baseline
- `vendor/pyAgxArm` became the pinned runtime SDK source
- shared-bus arm-plus-hand operation needed explicit operational guidance instead of being treated as
	an automatically safe parallel mode

## Historical evidence kept in this sprint surface

- `../checklist.md`
- `../errors_and_fixes.md`
- `../open_questions.md`
- `../evidence/can_transport_decision.md`