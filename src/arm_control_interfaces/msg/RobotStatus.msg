uint8 IDLE = 0
uint8 MOVING = 1
uint8 TEACHING = 2
uint8 STOPPING = 3
uint8 FAULT = 4
uint8 state
float64 speed_ratio
int32 error_code
string error_message
float64[7] joint_angles
float64[6] tcp_pose
float64 finger_width
string tcp_name
bool is_connected
