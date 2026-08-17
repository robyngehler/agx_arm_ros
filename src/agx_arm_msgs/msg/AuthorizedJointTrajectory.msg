# A standard joint trajectory, plus the authority it was issued under (4D).
#
# The external interface stays standard FollowJointTrajectory: MoveIt and every
# other trajectory client are unchanged. This is the INTERNAL boundary between
# the action server that accepted such a goal and the bridge that drives the
# hardware, and it exists so the authority survives that hop.
#
# trajectory_msgs/JointTrajectory is embedded whole and unmodified. The standard
# ROS messages are external types and are not extended: positions, optional
# velocities and accelerations, and time_from_start all keep their usual
# meaning. Previously only the final target survived this boundary, so the
# trajectory was discarded exactly where the authority was invented.
#
# What the bridge does with the trajectory is a backend question and is
# deliberately not fixed here. The refactor MVP keeps the known-good vendor
# integration and drives the final point through the existing position path; a
# later control sprint may sample it properly without changing this contract.

agx_arm_msgs/DeviceCommandStamp authority
trajectory_msgs/JointTrajectory trajectory
