# Development Open Questions

Cross-sprint development questions that are still open after the current cleanup pass.

## Package README scope

How much launch detail should package-local READMEs keep once `docs/control/` is treated as the canonical launch matrix and top-level entrypoint guide?

## Shared-bus operating profile

Once shared arm+hand runtime validation is repeated on current hardware, should the repo standardize a default native CAN profile for that slice, or keep it as an explicit workflow-specific override documented in `docs/control/teach_and_run.md`?