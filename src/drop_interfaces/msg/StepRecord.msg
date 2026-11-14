std_msgs/Header header
string cycle_id
uint32 step_seq
string step_name
builtin_interfaces/Time t_start
builtin_interfaces/Time t_end
uint8 result            # 공통 결과 코드(OK=0 … FAULT=5)
string reason_code
uint8 attempt
uint8 abort_level       # 0 = 없음, 1~4 = 중단 등급 L1~L4
string[] metric_keys    # 판정에 쓴 값 (예: "e_xy_m", "v_rel_mps")
float64[] metric_values
