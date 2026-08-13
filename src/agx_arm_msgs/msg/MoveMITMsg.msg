# This message contains the joint position, velocity, and torque control values for a single joint.

int32[] joint_index     # Index of the joint to control
float64[] p_des         # Desired joint position in radians
float64[] v_des         # Desired joint velocity in radians/second
float64[] kp            # Position gain
float64[] kd            # Velocity gain
float64[] torque        # Desired joint torque in N/m

# --- command stamp (V02 refactor, phase 1A) ---
#
# The one identity every commandable device uses — the same four fields in the
# ROS messages, in the driver's admission check, and in the hand contract. A
# command is admitted only if all four are current: the right commander, the
# device generation and the unit generation it was issued under, and a sequence
# that has not already been used in that generation.
#
# Both epochs, always. A device that deliberately does not take part in unit
# safety documents that as a named exception rather than dropping the field,
# because otherwise the same wire name means two different things on two
# devices.

string owner_id           # who is commanding
uint64 device_epoch       # device generation this was issued under
uint64 unit_safety_epoch  # unit generation this was issued under
uint64 sequence           # per owner, per epoch, strictly increasing
