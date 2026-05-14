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

string status_text