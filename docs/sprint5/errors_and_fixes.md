# Sprint 5 Errors And Fixes

Historical issue summary for the stable CAN transport and control-layer pinning phase.

## ENOBUFS on the Duo arms

Problem:

- the USB `gs_usb` arm path wedged permanently under Duo load after plan-and-execute cycles

Fix:

- move to native `mttcan`
- keep `one-shot on`
- treat node-side recovery as defense-in-depth instead of the primary remedy

## Control-layer source drift

Problem:

- the ROS runtime kept importing a stale non-editable `pyAgxArm` snapshot instead of the intended
	working source

Fix:

- editable-install the system-Python runtime copy
- pin `vendor/pyAgxArm` as the repo-owned runtime source

## Planning robustness follow-up

Problem:

- joint1 around $\pi$ still exposed a planning robustness gap even after solver-side TracIK tuning

Status:

- left open for later planning hardening
- not treated as a transport-layer blocker