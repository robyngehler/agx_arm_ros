# Repo-owned OmniHand diagnostic and backend status message

std_msgs/Header header

string hand_side
string backend_name
string control_mode

bool connected
bool initialized
bool is_mock
bool communication_fault

float32[] active_joint_temperatures_c
float32[] active_joint_currents_a

bool[] active_joint_stalled
bool[] active_joint_over_temperature
bool[] active_joint_over_current

# Verified-delivery state of the last hand target. On the shared side CAN bus a
# single send is lossy, so the bridge re-sends until a readback confirms it;
# these fields let a caller (the FollowJointTrajectory bridge) hold its hand
# window open until delivery is actually decided instead of guessing from
# elapsed trajectory time.
bool command_pending
bool command_delivery_failed
uint16 command_attempts

# Age of the last SUCCESSFUL SDK joint readback. feedback/omnihand/joint_states
# is republished from cache at pub_rate even while the backend is faulted, so
# its header stamp says nothing about whether the hand is actually answering.
# Negative means no successful readback has happened yet.
float32 joint_readback_age_s

string status_text