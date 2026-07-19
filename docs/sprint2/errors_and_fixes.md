# Sprint 2 Errors And Fixes

Historical issue summary for the Sprint 2 contract-hardening phase.

## Launch and runtime understanding was too diffuse

Problem:

- launch order, ROS graph behavior, file composition, and config dataflow had to be reconstructed
	from several launch files, xacros, and control docs

Fix:

- stable runtime and launch diagrams were promoted into the stable project docs
- the runtime-baseline work was split from the stable docs into historical sprint evidence

## Early real-hand CAN FD bringup was below-ROS by necessity

Problem:

- the real OmniHand path could not be debugged safely from ROS first because Linux transport and
	adapter capability were still unclear

Fix:

- the investigation stayed below ROS and eventually led to the later native `mttcan` baseline
- the surviving historical transport lessons now live in `evidence/omnihand_canfd_transport_history.md`

## MIT workflow knowledge needed a stable home

Problem:

- early MIT hold, replay, and wakeword-demo knowledge existed mostly as working notes and scripts

Fix:

- the stable operational workflow moved into `docs/control/bringups/teach_and_run.md`
- the remaining historical rationale now lives in `evidence/mit_runtime_history.md`