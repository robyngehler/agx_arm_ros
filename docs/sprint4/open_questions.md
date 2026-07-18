# Sprint 4 Open Questions

Historical handoff questions from Sprint 4.

## Body geometry validation

What staged-body clearance evidence and live-scene validation were still required before the
tightened self-collision matrix could be treated as production-safe for coordinated tasks?

## Hand-aware dual-arm semantics

What additional SRDF, controller-ownership, and safety semantics were required before a hand-aware
dual-arm surface could graduate beyond the landed per-arm `left_hand` and `right_hand` profiles?

## Selective-fix versus shared-stop behavior

When coordinated tasks move beyond the current central soft-stop contract, should one-arm faults
still fan out to both arms unconditionally, or should per-arm hold surfaces become the user-facing
selective-fix mechanism?