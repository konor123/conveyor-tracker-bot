uint8 OBSERVED=0
uint8 COASTING=1                  # 관측 없이 예측만 하는 중(가림)
uint8 LOST=2
std_msgs/Header header            # stamp = t_state, frame_id = conveyor_frame
uint32 track_id
uint32 epoch                      # 재시작·리셋 시 증가 → 이전 계획 무효
uint8 object_class                # ObjectDetection.CLASS_*
uint8 status
builtin_interfaces/Time last_observed_stamp
uint32 observation_count
geometry_msgs/Point position      # [m]
geometry_msgs/Vector3 velocity    # [m/s]
float64[16] state_cov             # [x, y, vx, vy] 4x4 row-major
builtin_interfaces/Time valid_until
