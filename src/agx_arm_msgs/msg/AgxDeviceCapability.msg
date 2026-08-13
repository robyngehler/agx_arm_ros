# What one device can actually encode on the wire (V02 refactor, C8).
#
# Repo-owned, agx_arm-centric. Separate from AgxDeviceAuthority on purpose:
# that message is *state* and changes constantly, this is *capability* and is
# fixed for the session. Mixing them would republish an unchanging table at
# every transition and invite consumers to treat a capability as a live signal.
#
# It exists because the two arms of this unit permanently run different firmware
# and therefore different protocol tiers (binding constraint C8, and the arms
# cannot be flashed). A control envelope that one arm accepts may be
# unencodable on the other, and without this a controller only finds out when
# the hardware boundary starts refusing commands — which for an active MIT
# stream means the arm holds its last setpoint while the other side keeps
# moving.
#
# Published latched, once, after the driver resolves the protocol tier.

std_msgs/Header header

string device_id          # arm_left | arm_right | hand_left | hand_right
string protocol_tier      # resolved vendor tier, e.g. "default" | "v111" | "v112"
string firmware_version   # as reported by the device, e.g. "1.06"

uint8 joint_count

# Per-joint feed-forward torque bound in N*m. Per joint because the default
# tier bounds joints 1-2, 3-4 and 5-7 differently while the 1.11 tier bounds
# every joint the same — the concrete asymmetry C8 is about.
float64[] max_torque

# Symmetric bounds shared by every joint of this device.
float64 max_position      # rad, magnitude
float64 max_velocity      # rad/s, magnitude
float64 max_kp
float64 max_kd
