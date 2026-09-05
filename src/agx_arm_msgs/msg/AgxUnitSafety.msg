# The unit-wide safety generation, allocated by exactly one writer (V02, 1A).
#
# Repo-owned, agx_arm-centric. Answers "which safety era is this unit in", so a
# command stamped under a superseded generation can be refused across devices.
#
# It is NOT a protective stop in the functional-safety sense. It is command
# arbitration inside this software stack, and every part of that stack can fail.
# The independent hardware emergency stop is a separate, open question — see
# docs/open_questions.md. Do not read this message as covering it.
#
# Exactly one process may allocate generations. Two allocators produce one
# number carrying two meanings ("5, stopped" and "5, rearmed") with no ordering
# between them, which is why writer_id travels with the epoch: a receiver has to
# be able to tell a contradiction from a duplicate.
#
# Published latched (transient local, depth 1) on change.

std_msgs/Header header

uint64 epoch          # advances on every transition, the stop and the rearm alike
bool stopped
string reason
string writer_id      # the process that allocated this generation

# One *run* of the writer. Epochs are only comparable inside a single
# incarnation, because a restarted writer counts from zero again: without this,
# an observer that had reached generation 5 silently discards everything a
# restarted writer publishes until it climbs back past 5.
#
# writer_started_ns orders the incarnations themselves, so a message from a
# superseded writer cannot be mistaken for news. Adopting a new incarnation
# fails closed — the unit is held stopped until that writer explicitly rearms
# it.
#
# A writer counts as superseded only while the incarnation that replaced it is
# still publishing. Start time orders two live writers; it cannot say that the
# younger one still exists. An observer that keeps following a dead incarnation
# refuses the live writer's rearm, and the unit is then stopped with no running
# process able to clear it.
string writer_incarnation
uint64 writer_started_ns
