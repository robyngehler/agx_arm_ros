# Authoritative state of one commandable device (V02 refactor, phase 1A).
#
# Repo-owned, agx_arm-centric. One device is one arm, or one hand's transport
# session — not a side. Since each device has its own CAN bus, a side is no
# longer a resource: an arm and the hand beside it have separate failure modes
# and separate recovery paths, and a fault on one must not invalidate commands
# to the other.
#
# This replaces `feedback/hand_window_active` as the signal a controller acts
# on. That boolean said only "the shared bus is busy" and nothing about faults,
# emergency stops, or who is commanding; a controller could not tell a
# deliberate quiescence from a dead device.
#
# Published on change and latched, so a late subscriber sees current state
# rather than waiting for the next transition.

std_msgs/Header header

string device_id          # arm_left | arm_right | hand_left | hand_right

uint8 STATE_OFFLINE=0     # no usable session with the device
uint8 STATE_STANDBY=1     # session up, motion not accepted
uint8 STATE_READY=2       # the only state that admits motion
uint8 STATE_RECOVERING=3  # transport recovery running, outcome unknown
uint8 STATE_FAULTED=4     # fault latched; needs acknowledgement, then a rearm
uint8 STATE_STOPPED=5     # halted by unit safety
uint8 state

# Bumped on this device's ownership changes, recovery, and arming transitions.
# A command carrying an older epoch is rejected rather than delivered late.
uint64 device_epoch

# Bumped only by what invalidates every device at once: an emergency stop or a
# unit fault. Separate from device_epoch so an arm recovery cannot abort a hand
# grasp on a bus the hand does not share.
uint64 unit_safety_epoch
bool unit_stopped

# True only in STATE_READY. Carried explicitly so no consumer has to re-derive
# the rule, and so the rule can only change in one place.
bool accepts_motion

string owner_id           # current commander, empty when unclaimed
string reason             # why the device is in this state
