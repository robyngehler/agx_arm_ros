# The authority under which one command was issued (V02 refactor, phase 4D).
#
# Repo-owned and device-agnostic: this is the reusable half of every
# authority-carrying command, embedded rather than repeated field by field.
#
# It exists because a bridge cannot otherwise tell a current command from a late
# one. Before this, the hand bridge built the stamp from its OWN current epoch
# and its OWN counter at the moment the command arrived, so the stale-epoch and
# out-of-order checks compared each value against itself and could never refuse
# anything. Identity has to travel WITH the command or it says nothing.
#
# A commander fills this in from the response to control/omnihand/claim_device,
# which returns both generations precisely so the first command need not wait
# for the authority topic to catch up.
#
# These fields are carried explicitly and never encoded into Header.frame_id,
# action goal UUIDs, topic names, or ad-hoc JSON metadata.

# Who is commanding, declared as "<primitive>:<node>". The primitive half is how
# the bridge tells a trajectory command from a reactive one; the node half is how
# it notices a commander that died still holding its claim.
string owner_id

# The device generation this command was issued under. Claim, release, recovery
# and rearm each advance it, so a command from the previous owner is refused
# rather than delivered late.
uint64 device_epoch

# The unit-wide safety generation it was issued under. Separate from
# device_epoch so an arm recovery cannot invalidate a hand grasp on a bus the
# hand does not share.
uint64 unit_safety_epoch

# Monotonic within one owner and one device_epoch, so a reordered command is
# refused instead of overwriting a newer target. Restarts at each new epoch.
uint64 sequence
