# Sprint 3 Errors And Fixes

Historical issue summary for the Nero planning and controller hardening phase.

## Naming drift in the active MoveIt surface

Problem:

- the roadmap expected `nero_arm` and `nero_tool0`, but the active surface still exposed `arm` and
	`tcp_link` in ways that would have forced rename churn

Fix:

- remove the `arm` compatibility alias
- move `nero_tool0` into the canonical description package
- keep `tcp_link` as the TCP/planning frame rather than collapsing the semantics

## IK plugin baseline was still undecided

Problem:

- the working MoveIt package still depended on KDL even though the intended baseline needed a more
	explicit IK choice

Fix:

- switch the active baseline to TRAC-IK
- document the Humble/Jetson source-build fallback and overlay order

## `move_group` shutdown crash remained unresolved

Problem:

- timeout-driven shutdown on the Humble/aarch64 host still ended in a `move_group` teardown crash

Status:

- unresolved in Sprint 3
- kept as historical evidence rather than promoted as a stable repo-wide rule