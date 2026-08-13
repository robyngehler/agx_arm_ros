# Arm status feedback message
uint8 ctrl_mode             # Control mode
uint8 arm_status            # Arm status
uint8 mode_feedback         # Mode feedback
uint8 teach_status          # Teach mode status
uint8 motion_status         # Motion status: 0=reached, 1=not reached
uint8 trajectory_num        # Current trajectory point number

# --- fault reporting (V02 refactor, phase 1A) ---
#
# Replaces `int64 err_status`, which was declared and never assigned: it
# published 0 for every arm in every state, and a consumer gating on it — the
# MIT controller did — had a fault check that could not fire.
#
# `fault_code` is the vendor's raw 16-bit code. Its bits ARE the per-joint flags
# below, which the driver decodes; the code is kept because a bit nobody has
# decoded yet is still worth reporting.
uint16 fault_code

# True when the arm reports anything wrong. Derived once, here, so no consumer
# re-derives it and the rule can only change in one place.
bool any_fault
bool[] joint_angle_limit    # Joint angle limit flags
bool[] communication_status_joint  # Joint communication status
