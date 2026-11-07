# robot_controller 발행(robot/status, 10 Hz, SPEC-010 v0.2). orchestrator 단계 판정과 safety_supervisor L4 판정 입력.
std_msgs/Header header            # stamp = 상태를 읽은 시각, frame_id = robot_base
bool connected                    # 제어기 통신 정상
bool ready                        # 지령 가능(서보 on, 오류·보호정지 없음)
bool protective_stop              # 보호정지·비상정지 활성
geometry_msgs/Pose tcp_pose       # robot_base [m]
bool in_workspace                 # TCP가 투입 작업영역(벨트 위 도달 구간) 안
bool holding                      # 공깃돌을 쥐고 있음(그리퍼 간격·전류 판정)
bool gripper_alive                # 상주 루프 DRL 생존 카운터가 진행 중
float64 gripper_response_s        # 마지막 그리퍼 명령의 응답 시간 [s]
uint16 gripper_error_bits         # RH-P12-RN(A) Hardware Error 비트(0 = 정상)
bool gripper_open                 # 개방 위치(대기 상태)
float64 gripper_width             # 핑거 간격 [m]
float64 gripper_current           # Present Current [mA]
