# Sprint 3 Open Questions

Historical handoff questions from Sprint 3.

## `move_group` teardown crash

Is the SIGINT teardown crash reproducible in a smaller Humble/aarch64 MoveIt setup outside this
workspace, or was there a workspace-local lifetime issue layered on top of the upstream
class-loader warning?

## Planning-path evidence threshold

Which additional profile, obstacle, and execution-safety variants would have been sufficient beyond
the near-home OMPL pose-plan smoke test to close the remaining Sprint 3 planning-path gap?