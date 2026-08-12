# Coordination-layer event emitted by the coordinator and each executor.
#
# Repo-owned, agx_arm-centric. One event type is shared across the coordinator,
# the performer, the OmniHand skill controller, and the arm executor so a single
# ~/events stream tells the whole story of an activity run. Hand-only tactile
# diagnostics still live on feedback/omnihand/*; this carries coordination
# milestones (started/completed/failed) and grasp-hold warnings (slip).

std_msgs/Header header

string source         # coordinator | omnihand_skill_controller | both_arms_executor | ...
string robot_id       # left_hand | right_hand | both_arms | left_arm | right_arm | "" (coordinator)
string activity_id
string action_id

# info | warning | error | started | completed | failed | rejected | slip_warning |
# contact_lost
#
# 'rejected' is a request the unit refused before it became an activity — a
# second concurrent goal, or one arriving while the coordinator is stopping. It
# is deliberately not 'failed': nothing was executed, so a consumer counting
# failures must not count it.
string event_type
string state          # executor/coordinator state at emission time
float32 contact_score # aggregated tactile contact score when relevant, else 0.0
string message
