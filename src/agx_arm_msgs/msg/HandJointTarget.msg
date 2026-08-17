# One authority-stamped joint target for a hand (V02 refactor, phase 4D).
#
# The migration path for reactive contact-seeking motion, which ends where the
# tactile sensor says rather than where the clock does. That loop emits a next
# target each cycle and cannot be expressed as a time-parameterized trajectory
# without losing the closed loop, so it gets a target message rather than being
# forced through the trajectory contract.
#
# Deliberately minimal: the same incremental position-target semantics the
# reactive controller already uses, and no velocity field, because nothing in
# the current backend could honour one. Adding fields the hardware path ignores
# would be a contract that lies.
#
# Joint names are carried as data, not implied by position, so this fits o10,
# o12_pro and the 1-DoF gripper alike without fixed-width fields.

agx_arm_msgs/DeviceCommandStamp authority

string[] joint_names
float64[] positions
