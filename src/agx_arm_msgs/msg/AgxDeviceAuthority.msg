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

# Bumped on every unit-safety transition — the stop and the rearm alike, since
# a device rearmed under the old generation must not be treated as current.
# Separate from device_epoch so an arm recovery cannot abort a hand grasp on a
# bus the hand does not share.
uint64 unit_safety_epoch
bool unit_stopped

# True only in STATE_READY: the hardware is ready to be commanded.
#
# This is NOT permission to command. Admission at the boundary additionally
# requires that the commander is the device's current owner and that both
# epochs and the sequence are current, so a consumer that reads this as "I may
# stream" can be told yes and then have every command refused. Check `owner_id`
# against your own commander identity as well.
bool motion_ready

string owner_id           # current commander, empty when unclaimed
string reason             # why the device is in this state
