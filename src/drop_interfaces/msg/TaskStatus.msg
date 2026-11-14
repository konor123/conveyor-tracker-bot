std_msgs/Header header
string cycle_id
string state                      # 상태 머신 상태명
uint8 abort_level                 # 0 = 없음, 1~4 = 중단 등급 L1~L4
uint16 error_code
string command_owner
bool robot_ready
bool protective_stop
bool gripper_holding
float64 time_remaining            # [s]
