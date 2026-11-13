# SPEC-500 robot-control

- 버전: v0.11
- 근거: PLAN.md 1.1, 3.3, 4.2~4.4, 5.1, 11.3절
- REQ 영역: ROB, GRP

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.11 | 2026-11-13 | REQ-ROB-017 이동 전 측정이 비유한 값이면 지령 없이 `TCP_READ_FAILED` |
| v0.10 | 2026-11-13 | I0 실제 controller 통합에서 드러난 계약: REQ-ROB-008 SYNCHRONIZE 끝 = 연속 `sync_hold_s + sync_hold_margin_s`(연속 통과 시작 주기에도 feedback), REQ-ROB-010 동기 유지 후퇴는 XY 속도 정착(`keep_sync_settle_s`) 뒤 상승, REQ-ROB-007 PickStone 전류 한도 1~150 mA·PlaceStone RELEASE `sync_held`(개방 전·후 측정 TCP)·RETREAT 안전 높이 상승(측정 z), REQ-ROB-012 `robot/status` 그리퍼 필드 = 마지막 skill 측정값, REQ-ROB-014 정지 유지가 아닌 재활성은 backend 정지를 풀고 Stop은 `stop_history_json`·로그에 남김 |
| v0.9 | 2026-11-13 | REQ-ROB-011 Doosan 드라이버 형식(4.6절 표: 서비스 8종·`speedl_stream`의 이름·형식·필드·단위), virtual 모드 대신 실제 `dsr_msgs2` 형식 stub 시험과 그 한계·OFF-2 확인 항목, REQ-ROB-017 Doosan 단위 client ROS 구현(`dsr_motion_client.DsrMotionClient`, 실기 미검증·노드 미연결) |
| v0.8 | 2026-11-13 | REQ-ROB-016 SafeRetreat place-back(`place_back_xyz_m`, 측정 개방 폭 확인, `stone_placed_back`), REQ-ROB-010 동기 유지 후퇴를 정적 실행기에 주입, REQ-ROB-007 정지 미확인 이동 중단은 `FAULT`, 4.6절 Doosan 실제 backend 준비(REQ-GRP-017 서비스 레지스터 경로, REQ-ROB-017 정적 backend 어댑터, 실기 미검증·노드 미연결), 실수 배열 파라미터 동적 형식 |
| v0.7 | 2026-11-13 | DynamicPlace 순서기·해제 게이트·해제 확정(commit) 경계 transport 계약(4.5절, REQ-ROB-007~010), REQ-ROB-013 UNCERTAIN revision 보류(`uncertain_hold_s`), `backend:=fake` 해제 transport(가정값), speedl 스트림 transport(REQ-ROB-015, 실기 미검증), 가속 단위 변환(REQ-ROB-002) |
| v0.6 | 2026-11-12 | REQ-ROB-013: 계획 수락 기준 3.25 mm와 해제 불확실성 기준 0.5 mm 분리(`release_uncertainty_ok`, 진단용), 그림자 기록 CSV(`sync_metrics`), `config/belt_sync.json` |
| v0.5 | 2026-11-11 | REQ-ROB-014 Stop 정지 유지·작업자 해제(`stop_latch.StopLatch`) |
| v0.4 | 2026-11-11 | REQ-ROB-013 XY 추종 코어(`belt_sync.BeltSynchronizer`): 계획 수락 조건, 감시, 제한·작업영역, 정지 유지와 명시적 재무장, 그림자 미리보기. REQ-GRP-016 지령 결과 레지스터 40과 `RESIDENT_WRITE_FAILED`, 실행 확인 뒤 새 표본으로 판정, 표본 주기 대비 제한시간 검사(응답 5주기·생존 3주기), REQ-GRP-013 생존 감시 기준을 `loop_timeout_s`로 |
| v0.3 | 2026-11-10 | REQ-GRP-014 공통 그리퍼 순서(`GripperSession`), 예약(prewrite) 무효화 규칙, REQ-GRP-015 막힘 파지 판정, REQ-GRP-016 상주 루프 transport·DRL 레지스터 계약, 그리퍼 전류 상한 100 → 150 mA |
| v0.2 | 2026-11-10 | REQ-ROB-007: Stop으로 선점된 goal은 `CANCELED`·`STOPPED`, cancel은 `CANCELED`·`CANCELED` |
| v0.1 | 2026-11-07 | ROB·GRP REQ 정의, SI/Doosan·그리퍼 adapter·정적 skill 계약, 기본 backend disabled |
| v0.0 | 2026-11-07 | 현장 일회성 그리퍼 CLI 요구(REQ-GRP-004·011) |

## 1. 목적·범위

Doosan 스트림·move_line 계약, 그리퍼 상주 루프 경로, 추종 제어, DynamicPlace(순서기·해제 게이트·해제 transport), SafeRetreat place-back, Doosan 실제 backend 준비(서비스 레지스터 경로·정적 backend 어댑터), L0. skill(MoveToPose·PickStone·PlaceStone·DynamicPlace·SafeRetreat·Stop)은 `robot_controller_node`가 노출하고 판단 로직은 ROS와 무관한 순수 Python 모듈(`skill_logic.py`, `dynamic_place.py`, `belt_sync.py`)에 둔다.

## 2. 입력·출력(interface 버전)

`src/drop_interfaces` 스냅샷(SPEC-010)의 `MoveToPose`·`PickStone`·`PlaceStone`·`DynamicPlace`·`SafeRetreat` action, `Stop` service, `RobotStatus`·`Heartbeat` message를 그대로 쓴다. 발행 토픽은 `robot/status`, `system/heartbeat`, service는 `robot/stop`이다. DynamicPlace 입력으로 `drop/intercept_plan`(`InterceptPlan`), `objects/tracked`(`TrackedObjectArray`), `belt/measured_motion`(`BeltMotion`)을 reliable/volatile depth 5로 구독한다(QoS 이벤트 기본 핸들러 없음, REQ-IF-006). 결과 코드는 interface `Result` 값을 사용하며, 성공은 실제 측정값이 기준을 만족할 때만 보고한다.

`SafeRetreat` 결과 `stone_placed_back`은 측정 개방 폭을 확인한 뒤에만 참이다(REQ-ROB-016). 기본 backend는 `disabled`다. 어떤 파라미터도 `/dsr01` 실기 연결을 암묵적으로 만들지 않는다. `backend:=fake`는 시뮬레이션 backend(`SimulatedBackend`)와 DynamicPlace 해제 transport(`SimulatedPlaceTransport`, 같은 TCP·정지 상태)만 쓰며 그 값은 하드웨어 측정으로 표시하지 않는다. `disabled`에서는 해제 transport가 없으므로 DynamicPlace는 `PRECONDITION`(`PLANNER_NOT_READY`)이다. 필수 설정값이 없으면 skill 결과는 `PRECONDITION`이다.

## 3. 파라미터(단위·기본값·근거)

수치 키는 단위 접미사를 붙인다. 기하·그리퍼 폭·전류 임계는 실측 기록이 있을 때만 채우며, 없으면 `-1`(미설정)로 두고 성공 판정 대신 `PRECONDITION`을 낸다. 실수 배열 파라미터(`standby_pose`·`safe_pose`·`place_back_xyz_m`)는 빈 배열이 미설정이며 동적 형식으로 선언해 실수 배열 덮어쓰기를 받는다.

| 키 | 단위 | 의미 |
|---|---|---|
| `backend` | - | `disabled`(기본) 또는 `fake` |
| `pose_tolerance_m` | m | MoveToPose 성공 판정 |
| `approach_height_m` | m | PickStone 접근·상승 높이 |
| `safe_z_m` | m | SafeRetreat 안전 높이 |
| `height_tolerance_m` | m | PlaceStone 높이 오차 허용 |
| `open_width_tolerance_m` | m | 부분 개방 폭 허용오차 |
| `release_rim_clearance_m` | m | 해제 게이트 여유 |
| `grip_min_width_m`·`grip_max_width_m` | m | 파지 폭 판정 범위 |
| `grip_min_current_ma` | mA | 파지 전류 하한(허용 1~150) |
| `move_v_mps`·`move_a_mps2` | m/s·m/s² | move_line·추종 한도 |
| `standby_pose`·`safe_pose` | m·quaternion | 이름 자세 [x, y, z, qx, qy, qz, qw](`robot_base`). 빈 배열 = 미설정 |
| `place_back_xyz_m` | m | SafeRetreat place-back 매트 반납 TCP 위치 [x, y, z](`robot_base`). 빈 배열 = 미설정(좌표는 음수도 유효해 −1 규약을 쓰지 않음). goal `place_back_pose` 위치가 0이 아니면 goal 값. 매트 위 이동 높이는 `safe_z_m`, 개방 폭은 `dynamic.open_width_m`, 허용오차는 `open_width_tolerance_m`(REQ-ROB-016) |
| `fake_tcp_lag_s` | s | `backend:=fake` 해제 transport의 TCP 1차 지연 시정수. 기본 0.029 = OFF-2 참조 부하 speedl 100 Hz 지령 → 위치 지연 29 ms를 시정수로 옮긴 **가정값**(Progress.md 2절) |
| `fake_open_delay_s` | s | `backend:=fake` commit → 개방(공깃돌 낙하) 지연 = 그 transport의 `commit_lead_s`. 기본 0.1, 가정값(개방 지연 미측정, OFF-2) |
| `fake_abort_sync_hold_s` | s | `backend:=fake` `abort(keep_sync=true)` 뒤 계획 XY 속도 유지 시간. 기본 1.0, 가정값 |
| `fake_cmd_hold_s` | s | `backend:=fake` 마지막 지령 유지 시간(speedl `time` 흉내). 기본 0.1, 가정값 |
| `stop_history_json` | - | 진단 전용(판정에 쓰지 않음). 받은 Stop마다 `requester`·`reason_code`·`received_steady_s`의 JSON 배열, 최근 100건(`STOP_HISTORY_MAX`)만 둔다. 작업자가 해제 전에 원인을 본다(REQ-ROB-014) |

DynamicPlace 설정은 `dynamic.<키>`(`dynamic_place.DynamicPlaceCfg`, 아래 표), 추종 코어 설정은 `sync.<키>`(`belt_sync.SyncConfig` = `config/belt_sync.json`)로 준다. `safe_z_m`·`open_width_tolerance_m`은 위 정적 skill 값을 함께 쓴다. 해제 게이트 임계값 5개는 goal 값이 유한한 양수면 goal 값, 아니면 아래 기본값이다.

| 키 | 단위 | 기본값 | 근거 |
|---|---|---|---|
| `e_xy_max_m` | m | 0.00325 | SPEC-100 `sync.e_xy_max_m` |
| `v_rel_max_mps` | m/s | 0.005 | SPEC-100 `sync.v_rel_max_mps` |
| `tau_max_s` | s | 0.3 | SPEC-100 `watchdog.obs_freshness_s`. track·벨트 메시지 나이 한도도 같은 값 |
| `position_std_max_m` | m | 0.0005 | SPEC-100 `sync.release_position_std_max_m` |
| `rim_clearance_m` | m | 0.002 | SPEC-100 `descend.h_clear_m` |
| `approach_above_m` | m | 0.03 | PLAN.md 4.2절 해제점 위 30 mm |
| `approach_tol_m` | m | 0.002 | PLAN.md 4.3절 APPROACH 2 mm |
| `approach_timeout_s` | s | 2.25 | 1.5 × SPEC-400 `t_transit_s` 1.5 s |
| `plan_wait_s` | s | 0.5 | PLAN.md 4.3절 PLAN_INTERCEPT 0.5 s |
| `sync_hold_s` | s | 0.3 | PLAN.md 4.3절, SPEC-100 `sync.hold_s` |
| `sync_hold_margin_s` | s | 0.03 | SYNCHRONIZE를 `sync_hold_s`보다 더 유지하는 시간. orchestrator는 같은 0.3 s를 feedback 수신 시각으로 다시 잰다(SPEC-100 `judge_synchronize`). 수신 지연 차 허용 = feedback 간격 1개 |
| `sync_timeout_s` | s | 2.0 | PLAN.md 4.3절, SPEC-100 `sync.timeout_s` |
| `sync_feedback_period_s` | s | 0.03 | SPEC-010 5절 20 Hz 이상. 노드 주기 지터 한 주기(10 ms)가 더해져도 간격 ≤ 0.05 s |
| `entry_lead_s` | s | 1.3 | MODE_PREDICTED 속도 일치 구간 진입 = `release_time` − 이 값. SPEC-400 `t_sync_s` 0.5 + `t_descend_s` 0.8 |
| `descend_tol_m` | m | 0.0005 | PLAN.md 4.3절 해제 높이 ±0.5 mm |
| `descend_timeout_s` | s | 1.5 | PLAN.md 4.3절 |
| `z_kp_hz` | 1/s | 10.0 | 제안. z 위치 이득 |
| `z_max_speed_mps`·`z_max_accel_mps2` | m/s·m/s² | 0.05·0.3 | 제안(TBD OFF-2), 가속은 `sync.max_accel_mps2`와 같음 |
| `release_wait_max_s` | s | 2.0 | MODE_SYNCHRONIZED 게이트 대기 상한 = `sync_timeout_s` |
| `release_hold_s` | s | 0.3 | 해제 뒤 동기 유지 = 개방 지연 p95 + 여유. TBD(OFF-2), 제안값은 SPEC-400 `t_release_s` |
| `release_timeout_s` | s | 1.0 | 예정 해제 시각부터 개방 확인 제한. TBD(OFF-2, 상주 루프 표본 주기 0.58 s 고려) |
| `commit_late_tol_s` | s | 0.02 | MODE_PREDICTED commit 늦음 허용 2주기 |
| `retreat_speed_mps` | m/s | 0.05 | 제안 |
| `retreat_margin_m` | m | 0.002 | 후퇴 z 목표 = `safe_z_m` + 이 값 |
| `retreat_timeout_s` | s | 3.0 | PLAN.md 4.3절 |
| `sync_speed_tol_mps` | m/s | 0.001 | REQ-ROB-010 ±1 mm/s |
| `keep_sync_ref_max_age_s` | s | 2.0 | 제안. SafeRetreat keep_sync가 쓰는 마지막 계획 속도의 나이 |
| `keep_sync_settle_s` | s | 0.2 | 동기 유지 후퇴에서 XY 속도가 계획 속도 ±`sync_speed_tol_mps`에 들 때까지 z를 올리지 않고 기다리는 최대 시간. 제안(`backend:=fake` 1차 지연 29 ms의 약 7배), TBD(OFF-2) |
| `release_above_rim_m` | m | 0.1656 | SPEC-400 `target.release_above_rim_m`(해제 TCP z − 테두리 z) |
| `fingertip_below_tcp_m` | m | −1(미설정) | 플랜지 → 핑거 끝 높이. 실측 OFF-6 |
| `open_width_m` | m | −1(미설정) | goal `open_width`가 0일 때만 쓴다 |

그리퍼 어댑터 설정에는 pulse 표 경로, protocol(`dynamixel2`·`modbus`), slave ID, 응답 제한시간, 기본 전류 `100 mA`, 최대 전류 `150 mA`(허용 `1~150`, 2026-11-09 실기 파지에 쓴 값)가 있다. 현장 CLI 운용 범위 `0~750 pulse`는 어댑터의 하드웨어 능력 `0~1150 pulse`와 별개이며 반전 규약을 쓰지 않는다.

## 4. 동작(상태·알고리즘)

### 4.1 공통

- REQ-ROB-001: SI 위치·quaternion과 Doosan move_line 위치·ZYZ 오일러(mm·deg)를 양방향 변환한다. 왕복 회전 오차는 `1e-9 rad` 이하이고, β≈0·π 특이 자세는 같은 회전이면 통과한다.
- REQ-ROB-002: 병진 `m/s`는 `mm/s`, 회전 `rad/s`는 `deg/s`, 가속 `m/s²`·`rad/s²`는 `mm/s²`·`deg/s²`(스트림 `acc`)로 변환한다. 변환은 `motion_adapter.py`에서만 한다.
- REQ-ROB-003: 스트림 지령 신선도는 주기 2배까지 유지하고, 초과 시 속도 0을 1회 보낸 뒤 재무장 전까지 지령을 거부한다(L0).
- REQ-ROB-004: 작업영역·속도·비유한 값을 지령 전에 검사하고, 위반이면 송신하지 않으며 L0와 `RANGE`를 낸다.
- REQ-ROB-005: 스트림·move_line·그리퍼 지령 주체는 `robot_controller_node` 하나다. 동시 skill goal은 `BUSY`로 거부한다.
- REQ-ROB-006: 추종 지령은 `v = v_ff + Kp·(p_target − p_tcp)`이며 `|v| ≤ v_max`, `|v − v_prev| ≤ a_max·dt`를 만족한다.

### 4.2 현장 일회성 그리퍼 CLI

`gripper`는 기존 ROS 드라이버 `/dsr01/drl/drl_start`를 통해 플랜지 RS-485에서 일회성 DRL을 실행한다. 상주 루프를 요구하지 않는다. 현장 비상정지 담당자와 간섭 없는 작업 공간을 전제로 한다.

- REQ-GRP-004: 일회성 CLI는 Model Number=35074, Operating Mode=5, Torque Enable=1, Hardware Error=0, 로봇 STANDBY와 DRL 정지 상태를 확인한 뒤 Goal Current, Goal Position 순서로 쓴다. 목표는 반전 없는 하드웨어 값이며 현장 운용 범위는 0~750 pulse, open 기본값 0, close 기본값 750이다. 개폐·위치 명령의 기본 전류는 100 mA, 상한은 150 mA이며 허용 전류는 1~150 mA이다. 범위 밖 입력은 지령 전에 거부하고 EEPROM·펌웨어·통신 설정을 쓰지 않는다. 목표 위치 도달은 Present Position 허용오차 3 pulse로 판정하며 현장 관찰과 별도로 기록한다.
- REQ-GRP-011: Modbus-RTU FC03·FC06·FC16 프레임은 CRC16을 사용하고 응답 길이·ID·기능 코드·주소 또는 바이트 수·CRC를 검사한다. TC-GRP-011-01의 표준 벡터와 일치해야 한다.

### 4.3 그리퍼 adapter

- REQ-GRP-001: pulse↔개구 간격 표는 단조 증가·범위를 검증해 로드하고, 표 밖 요청은 클램프 없이 `OUT_OF_RANGE`로 거부한다.
- REQ-GRP-002: 시작 시 Model Number 35074, Hardware Error 0, 펌웨어 V12 이상, Operating Mode 5, 로봇 STANDBY를 확인한다. 불일치는 `PRECONDITION`이다.
- REQ-GRP-003: 레지스터 주소표는 protocol별로 분리한다. dynamixel2(Torque Enable 512, Hardware Error Status 518, Goal Current 550, Profile Velocity 560, Goal Position 564, Present Current 574, Present Position 580)와 modbus(Torque Enable 256, Hardware Error Status 259, Goal Current 275, Goal Position 282, Present Current 287, Present Position 290)를 섞지 않는다. EEPROM 쓰기는 토크가 꺼진 상태에서만 한다.
- REQ-GRP-005: 부분 개방은 `width_to_pulse(width)`를 Goal Position으로 쓰고 Present Position이 허용오차 안에 들 때 OK다.
- REQ-GRP-006: 파지 판정은 간격·전류 조합으로 하며, 측정값이 없으면 성공으로 보고하지 않는다.
- REQ-GRP-007: 응답 제한시간은 steady 시계로 재고, 초과·무응답은 `FAULT`(`GRIPPER_TIMEOUT`)다.
- REQ-GRP-008: Hardware Error Status 비트를 이름으로 해석해 `FAULT` reason에 포함한다.
- REQ-GRP-009: 개폐 1회마다 `t_cmd`·`t_ack`·`t_motion_start`·`t_done`을 단조 증가로 기록한다.
- REQ-GRP-010: `GripperTransport` 구현은 adapter와 시험 외에서 import하지 않는다.
- REQ-GRP-012: 7비트 분할 레지스터 인코딩은 0~16383 왕복이 일치하고 범위 밖은 예외다. 목표값 쓰기가 모두 끝난 뒤 트리거(seq)를 마지막에 1회 쓴다.
- REQ-GRP-013: 상주 루프 생존 카운터가 `loop_timeout_s`(기본 0.1 s) 넘게 변하지 않으면 `FAULT`(`GRIPPER_LOOP_DEAD`)다. 상태 표본 주기를 알리는 transport에서의 하한은 REQ-GRP-016이 정한다. 로봇 STANDBY가 아니면 `drl_start`를 호출하지 않고 `PRECONDITION`이다.
- REQ-GRP-014: 정적·동적 투입은 같은 그리퍼 순서(`gripper_session.GripperSession`)를 쓴다. 상주 루프 기동과 시작 확인은 세션당 1회이고 시작 전 호출은 `PRECONDITION`(`SESSION_NOT_STARTED`)이다. `ensure_open`은 방금 읽은 폭이 목표 ± 허용오차이면 쓰기·트리거를 보내지 않는다. `arm_open`은 목표값만 쓰고, `fire_open`은 트리거 1회만 보내며 예약이 없으면 `PRECONDITION`(`NOT_ARMED`)이다. 다른 지령이 목표 레지스터를 덮어쓰면 앞선 예약은 무효이고, 무효가 된 예약으로는 트리거하지 않는다. 지령마다 시각과 생략 여부를 기록한다.
- REQ-GRP-015: `grasp_stall_pulse`가 설정되면 닫기는 닫힘 목표 도달 또는 막힘으로 끝난다. 막힘은 상주 루프가 새로 올린 연속 두 표본의 위치 차가 `grasp_stall_pulse` 이하이고, 명령 전 위치에서 `grasp_min_travel_pulse` 넘게 움직였거나 전류가 한도의 `grasp_current_ratio` 이상일 때다. 막힘이면 `holding = true`(`STALLED`), 닫힘 목표까지 갔으면 `holding = false`(`REACHED_END`)다. 생존 카운터가 같은 동안의 읽기는 같은 표본으로 본다.
- REQ-GRP-016: 상주 루프 transport(`resident_transport.ResidentLoopTransport`)는 제어기 출력 레지스터로만 대화한다. PC → DRL은 0~127 값(10 seq, 11·12 목표 위치, 13·14 목표 전류의 7비트 분할, 24 예약, 25 종료), DRL → PC는 0~255 값(30 생존 카운터, 31·32 위치, 33·34 전류, 35 오류 비트, 36 시작 상태, 37 Operating Mode, 38 펌웨어, 39 실행한 seq, 40 그 seq의 결과(0 정상, 242 쓰기 실패, 39보다 먼저 씀))이다. 루프는 전류 쓰기에 실패하면 위치를 쓰지 않는다. transport는 마지막으로 트리거한 지령의 실행 여부와 결과를 알린다. adapter는 루프가 그 지령을 실행했고(39 = 트리거한 seq) 결과가 0이며 그 뒤 생존 카운터가 바뀐 새 표본으로만 도달·막힘을 판정하고, 결과가 실패면 그 지령의 `FAULT`(`RESIDENT_WRITE_FAILED`)다. 앞선 지령의 결과는 다음 지령에 영향을 주지 않으며, 루프를 다시 기동하면 앞 지령은 잊는다. transport는 상태 표본 주기 `sample_period_s`(제안값 0.58 s, 일회성 DRL 왕복 측정값)를 알리고, adapter는 응답 제한시간이 5주기 미만이거나 생존 감시 제한시간이 3주기 미만이면 생성 때 `PRECONDITION`(`TIMEOUT_BELOW_LOOP_PERIOD`)으로 거부한다. 5주기는 전류를 바꾸는 지령의 최악 경우(트리거 대기 1, 전류·위치 쓰기 2, 실행 뒤 표본 1, 막힘 판정 표본 1), 3주기는 쓰기 2회와 다음 읽기 동안 생존 카운터가 멈추는 시간이다. 루프 기동은 시작 상태 1을 본 뒤에만 성공이고 그 밖의 값은 `FAULT`(`RESIDENT_INIT`), 무응답은 `RESIDENT_INIT_TIMEOUT`이다. 루프가 올리지 않는 레지스터 읽기는 값을 지어내지 않고 예외다. 예약 상태의 루프는 Modbus 읽기를 멈추고 seq만 지켜보아 트리거 뒤 그리퍼가 목표를 받기까지 지연이 `spin + 위치 쓰기 1회` 이하다. DRL 본문(`drl/gripper_resident.drl`)의 레지스터 번호는 `ResidentRegs`와 같다.

### 4.4 정적 skill과 추종

- REQ-ROB-007: `MoveToPose`·`PickStone`·`PlaceStone`·`SafeRetreat`·`Stop`은 phase feedback과 결과 코드를 내고, cancel은 0.5 s 안 `CANCELED`(이유 `CANCELED`), 실행 중 `Stop`으로 선점된 goal은 `CANCELED`(이유 `STOPPED`, SPEC-010 3.2절), deadline 초과는 `TIMEOUT`, 실행 중 `Stop`은 goal 선점 후 정지 피드백 확인 뒤에만 `stopped=true`다. MoveToPose는 측정 자세 오차, PickStone은 측정 파지, PlaceStone은 측정 개방 폭으로 성공을 판정한다. PickStone goal `grip_current_limit`은 그리퍼 전류 허용 1~150 mA 안이어야 하고 밖이면 지령 없이 `PRECONDITION`(`CURRENT_OUT_OF_RANGE`)이다. PlaceStone은 `safe_z_m`이 없으면 지령 없이 `PRECONDITION`(`SAFE_Z_UNSET`)이고, RELEASE `sync_held`는 개방 전·후 측정 TCP가 모두 목표에서 `pose_tolerance_m` 안일 때만 1(정지 목표의 동기 유지), RETREAT는 개방 뒤 측정 XY에서 `safe_z_m`까지 수직으로 올리며 `tcp_z_m`은 상승 뒤 측정 z, `outside_workspace`는 측정 z ≥ `safe_z_m`일 때만 1이다. 측정 z가 안전 높이에 못 미치면 `released=true`인 FAILED `RETREAT_FAILED`다. 정적 skill 이동이 cancel·deadline으로 끊기면 `FAILED`가 아니라 `CANCELED`·`TIMEOUT`이다. backend가 이동 중단 뒤 정지를 확인하지 못했다고 알리면(`STOP_NOT_CONFIRMED`) cancel·deadline이어도 `FAULT`(`STOP_NOT_CONFIRMED`)다. `DynamicPlace`도 같은 결과 규칙을 따르며 cancel·deadline·Stop을 요청을 본 주기 안에 끝낸다. 해제 전이면 commit 0회이고, cancel·deadline은 SYNCHRONIZE 이후면 `abort(keep_sync=true)`, APPROACH면 `abort(keep_sync=false)`를 부른다. commit 뒤에 끝난 goal은 `released=true`다.
- REQ-ROB-008: `DynamicPlace`는 해제 transport가 있을 때만 실행하고 없으면 `PRECONDITION`(`PLANNER_NOT_READY`)이다. goal의 `plan_id`·`target_epoch`와 맞는 첫 revision의 track으로 추종 코어를 무장하고(REQ-ROB-013), goal 전에 받은 revision은 수신 나이 ≤ `sync.max_plan_age_s`인 것만 넘긴다. 받은 revision 이하는 무시하고, epoch·식별 변경, 만료, OK·UNCERTAIN이 아닌 상태, 해제 z 비유한은 FAILED `PLAN_INVALID`, UNCERTAIN 보류 초과는 FAILED `UNCERTAIN_HOLD`, `plan_wait_s` 안에 맞는 계획이 없으면 FAILED `NO_PLAN`이다. mode가 PREDICTED·SYNCHRONIZED가 아니거나(`MODE_UNSUPPORTED`), SYNCHRONIZED인데 transport가 `mid_course_update`를 못 하거나(`MODE_NEEDS_MID_COURSE_UPDATE`), 게이트 임계값이 유효하지 않거나, 핑거 끝 오프셋·안전 높이·개방 폭·개방 폭 허용오차가 미설정이거나(`<이름>_UNSET`), 계획 핑거 끝 여유 `release_above_rim_m − fingertip_below_tcp_m`이 `rim_clearance`보다 작으면(`RIM_CLEARANCE_UNREACHABLE`) 지령 없이 `PRECONDITION`이다. phase는 4.5절 순서이고, 각 phase 시작에 `phase_done=false`, 끝에 SPEC-010 2.3.1절 키를 모두 가진 `phase_done=true` feedback을 내며, SYNCHRONIZE 동안 `e_xy`·`v_rel`을 `sync_feedback_period_s`(0.03 s) 간격으로 내고 MODE_SYNCHRONIZED에서는 동기 게이트 연속 통과가 시작한 주기에도 낸다. MODE_SYNCHRONIZED SYNCHRONIZE는 연속 통과가 `sync_hold_s + sync_hold_margin_s` 이어질 때 끝난다: orchestrator가 feedback 수신 시각으로 다시 잰 연속 시간(첫 통과 표본 → 끝 표본)이 수신 지연 차가 `sync_hold_margin_s` 안이면 `sync_hold_s` 이상이다. 하강 중 동기 게이트가 한 번이라도 깨지면(DESCEND `sync_ok = 0`) 해제하지 않고 FAILED `SYNC_LOST`다. OK는 commit 1회, 개방 확인(측정 폭 = 개방 폭 ± 허용오차, 예정 해제 시각 + `release_timeout_s` 안), 해제 뒤 `release_hold_s` 동안 동기 게이트 유지, RETREAT 판정(REQ-ROB-010)을 모두 만족할 때만이다. commit 뒤 실패는 `released=true`인 FAILED(`OPEN_NOT_CONFIRMED`·`OPEN_WIDTH`·`SYNC_LOST`·`SYNC_SPEED`·`RETREAT_TIMEOUT`)다. transport 표본 시각이 주기 시각보다 한 주기 이내로 늦으면 그 표본 시각으로 판정한다(그보다 늦거나 앞서면 REQ-ROB-013 `TCP_STALE`).
- REQ-ROB-009: 해제 게이트(`dynamic_place.release_gate`, PLAN.md 4.4절)는 commit 직전 한 번, transport `commit_lead_s` 앞(해제 순간)으로 등속 외삽한 값으로 판정한다. 항목과 위반 이유는 XY 오차(공깃돌 중심 = TCP + 파지 오프셋, 뚜껑 중심 = 계획 목표 + 파지 오프셋) ≤ `e_xy_max`(`GATE_E_XY`), 상대 속도 ≤ `v_rel_max`(`GATE_V_REL`), track이 OBSERVED이거나 해제 순간 관측 나이 ≤ `tau_max`인 COASTING(`GATE_TRACK_LOST`·`GATE_COASTING`), track·벨트 메시지 나이 ≤ `tau_max`(`GATE_TRACK_STALE`·`GATE_BELT_STALE`), 계획 위치 표준편차 ≤ `position_std_max`(`GATE_POSITION_STD`), 해제 순간 ≤ `valid_until`(`GATE_EXPIRED`), goal·계획·track epoch 일치(`GATE_EPOCH`), 핑거 끝 − 테두리 ≥ `rim_clearance`(`GATE_RIM_CLEARANCE`), 벨트 `steady`(`GATE_BELT_UNSTEADY`), 추종 코어 UNCERTAIN 보류 아님(`GATE_PLAN_UNCERTAIN`), 유한 값(`GATE_NONFINITE`)이고 경계값은 통과다. 위반이면 그리퍼 commit·개방 0회, FAILED `GATE_<항목>`, `released=false`, L3 요청(`abort_request=3`), `abort(keep_sync=true)`다. MODE_SYNCHRONIZED는 DESCEND가 끝난 뒤 매 주기 판정해 열리면 commit하고, 닫힌 동안은 추종을 유지하며 `release_wait_max_s` 또는 `해제 순간 + release_hold_s > valid_until`까지 기다린다. 벨트 `steady=false` 맥동이나 UNCERTAIN 보류가 그 안에 끝나면 해제가 늦어질 뿐 중단하지 않는다. 비유한·epoch·만료·핑거 끝 여유 위반은 기다리지 않는다. MODE_PREDICTED는 `release_time − commit_lead_s` 이후 첫 주기(늦음 ≤ `commit_late_tol_s`)에 한 번만 판정하고 닫혀 있으면 기다리지 않고 실패한다. 결과의 `gate_*`는 commit을 허가하거나 거부한 그 판정 값이다.
- REQ-ROB-010: 동기 유지 후퇴(`dynamic_place.KeepSyncRetreat`)는 XY 지령을 계획 속도 그대로(위치 보정 없음) 두고 z를 `safe_z_m + retreat_margin_m`으로 올린다. 올리기 전에 z를 그대로 둔 채 표본 XY 속도가 계획 속도 ±`sync_speed_tol_mps`(1 mm/s)에 들 때까지 최대 `keep_sync_settle_s`(0.2 s) 기다린다(해제 전 cancel 직후 남은 추종 보정 속도, APPROACH의 `abort(keep_sync=false)` 뒤 정지 상태). 판정은 지령이 아니라 `poll()` 표본의 XY 속도로 하며, 올리기 시작한 뒤 z가 `safe_z_m`에 닿을 때까지 모든 표본이 그 안이어야 OK다. 어긋나도 안전 높이까지는 올리고 FAILED `SYNC_SPEED`다. DynamicPlace RETREAT와 `SafeRetreat(keep_sync=true)`가 같은 판정을 쓴다. `SafeRetreat(keep_sync=true)`는 마지막 DynamicPlace 계획 속도(나이 ≤ `keep_sync_ref_max_age_s`)를 쓰고, 해제 transport가 없거나 `capabilities.keep_sync_retreat`가 거짓이거나 계획 속도가 없으면 움직이지 않고 `PRECONDITION`(`SYNC_NOT_AVAILABLE`)이다. 노드는 이 후퇴(`dynamic_place.run_keep_sync_retreat`)를 정적 실행기 `safe_retreat`에 주입하고, 단일 지령 권한은 정적 실행기가 잡는다. `place_back`이 함께 오면 REQ-ROB-016이다. 해제 전 실패의 `abort(keep_sync=true)` 뒤 transport는 `abort_sync_hold_s` 동안 계획 XY 속도와 현재 z를 유지해, 뒤이은 `SafeRetreat(keep_sync=true)`가 동기 상태에서 시작한다.
- REQ-ROB-016: `SafeRetreat(place_back=true)`는 안전 높이로 후퇴한 뒤 매트에 공깃돌을 내려놓는다. 반납 목표는 goal `place_back_pose`의 위치가 0이 아니면 goal 위치·quaternion, 아니면 `place_back_xyz_m`과 후퇴 뒤 측정 자세다(항등 quaternion을 기본 자세로 쓰지 않는다). 어느 구간도 움직이기 전에 `pose_tolerance_m`·`safe_z_m`·반납 위치·개방 폭(`dynamic.open_width_m`)·`open_width_tolerance_m`을 확인해 없으면 지령 없이 `PRECONDITION`(`POSE_TOLERANCE_UNSET`·`SAFE_Z_UNSET`·`PLACE_BACK_POSE_UNSET`·`OPEN_WIDTH_UNSET`·`OPEN_WIDTH_TOLERANCE_UNSET`)이고, 반납 위치가 비유한이거나 z ≥ `safe_z_m`이거나 goal quaternion이 0·비유한이면 지령 없이 FAILED `RANGE`다. `keep_sync`인데 동기 유지 후퇴를 줄 수 없으면 REQ-ROB-010대로 `PRECONDITION`(`SYNC_NOT_AVAILABLE`)이다. 순서는 RETREAT(`keep_sync`이면 REQ-ROB-010 동기 유지 후퇴, 아니면 수직으로 `safe_z_m`) → PLACE_BACK(`safe_z_m` 높이에서 반납 위치 위로 이동 → 반납 위치로 내려감 → `dynamic.open_width_m`로 개방 → 측정 TCP XY에서 `safe_z_m`으로 올림)이며 옆 이동은 `safe_z_m` 높이에서만 한다. feedback phase는 `RETREAT`·`PLACE_BACK`(각각 시작·끝)이다. RETREAT가 실패하면(`SYNC_SPEED`·`SYNC_NOT_AVAILABLE`·`RETREAT_TIMEOUT` 등) 반납하지 않고 그 결과를 낸다(PLAN.md 5.1절 RETREAT 실패는 L4). 이동은 측정 자세 오차 ≤ `pose_tolerance_m`일 때만 다음 단계로 가고(아니면 FAILED `POSE_ERROR`), 개방은 측정 폭이 개방 폭 ± `open_width_tolerance_m`일 때만 확인이다(아니면 FAILED `OPEN_WIDTH`, 측정 없음은 FAILED). 측정 자세를 읽지 못하면(비유한) 그 자세로 움직이지 않고 FAILED `TCP_POSE_INVALID`이며, 개방 확인 뒤 TCP를 읽지 못하면 반납 위치 XY에서 올린다. `stone_placed_back`은 개방 확인 뒤에만 참이고 그 뒤 상승이 끊겨도 참이다. 각 이동·개방 전의 cancel·deadline은 `CANCELED`·`TIMEOUT`이며 그 뒤 새 지령은 없다. 개방 확인 뒤에는 cancel·deadline을 이유로 상승을 생략하지 않는다(PlaceStone과 같음). 실행 중 `Stop`은 REQ-ROB-007대로 `CANCELED`(`STOPPED`)다.
- REQ-ROB-013: XY 추종 코어(`belt_sync.BeltSynchronizer`)는 ROS·로봇·그리퍼 입출력 없이 SI·`robot_base`로 동작한다. 목표는 `release_xy + v·(t − release_time)`이다(입력 상태 시각이 아니라 해제 시각 기준). `arm(plan_id, track_id, epoch)`으로 지정한 계획만 받고, 다른 식별자가 오면 `PLAN_IDENTITY_CHANGED`로 멈춘다. 받은 revision 이하(버린 UNCERTAIN revision 포함)는 무시하며 수신 감시를 갱신하지 않는다. `status`가 OK·UNCERTAIN이 아님, `frame_id`≠`robot_base`, 비유한 값, `position_std` 초과, 시각 순서(`source_state_stamp ≤ 생성 ≤ 현재`) 위반, 입력 상태 나이 초과, 만료는 `PLAN_INVALID`다. `UNCERTAIN` revision은 위치 표준편차를 뺀 수락 조건을 통과하면 래치하지 않고 버리며, 마지막 OK 계획을 등속으로 계속 따른다(벨트 `steady=false` 맥동 때 속도 공분산이 되돌아가 planner revision이 잠깐 UNCERTAIN이 된다, SPEC-300 4절). 보류는 첫 UNCERTAIN revision의 steady 수신 시각부터 `uncertain_hold_s`(0.6 s = SPEC-300 `recover_s` 0.5 s + 여유)까지이고, 뒤이은·중복 UNCERTAIN은 늘리지 않으며 OK revision 수락만 끝낸다. 보류 중에는 수신·입력 상태 나이 대신 이 상한을 보고 넘으면 `UNCERTAIN_HOLD`로 멈춘다(마지막 OK 계획의 등속 외삽은 최대 `max_plan_age_s + uncertain_hold_s`). OK 계획 전에 온 UNCERTAIN도 같은 상한을 센다. 매 주기 계획 없음은 `WAIT_PLAN`, 비유한 TCP는 `NONFINITE`, ROS 시계 역행은 `CLOCK_BACKWARD`, steady 주기 간격 초과는 `LOOP_GAP`, 만료는 `PLAN_EXPIRED`, 수신(steady)·입력 상태(ROS) 나이 초과는 `PLAN_STALE`, TCP frame·나이 위반은 `TCP_STALE`, 목표·TCP·한 주기 예상 위치가 XY 작업영역 밖이면 `WORKSPACE`다. 지령은 REQ-ROB-006 식과 제한을 따른다. 멈춤(`stop` 포함)은 래치되어 출력은 비활성·속도 0이며, 같은 계획이라도 `arm`으로 다시 무장하기 전까지 계획을 받지 않는다. 비활성 출력의 0은 정지 증거가 아니다. 설정값은 유한한 양수이고 주기 ≤ 간격 한도, 작업영역 최소 < 최대여야 하며 아니면 생성 시 예외다. `belt_sync_preview`는 `/tracking_preview/cmd_vel`·`/tracking_preview/status`만 발행하고 Doosan·그리퍼 지령을 보내지 않는다. 계획 수락의 `position_std` 한도는 `max_position_std_m`(3.25 mm, SPEC-100 `plan.position_std_max_m`·SPEC-400 `uncertainty.position_std_max_m`과 같은 값)이고, 해제 불확실성 한도 `release_position_std_max_m`(0.5 mm, SPEC-100 `sync.release_position_std_max_m`과 같은 값)은 그보다 클 수 없다(생성 시 예외). `release_uncertainty_ok()`는 멈춤·UNCERTAIN 보류가 없고 현재 계획의 `position_std` ≤ 0.5 mm, 입력 상태·수신 나이 ≤ `max_plan_age_s`, 미만료일 때만 참인 진단값이며 해제 허가가 아니다(해제 게이트는 REQ-ROB-009). 설정은 `config/belt_sync.json`(코드 기본값과 같음)을 `belt_sync_preview --config`로 줄 수 있다. `--csv`를 주면 `sync_metrics.SyncCsv`가 주기마다 한 행(시각, 활성 여부·이유, 계획 식별자, TCP·목표·오차·지령, `position_std`, `release_uncertainty_ok`)을 새 파일에 쓰고 기존 파일은 덮어쓰지 않는다. `summarize()`는 비활성 행과 이유별 개수를 포함해 오차 p95·최대, 표본 간격 p99를 낸다. 이 기록은 그림자 관측이며 G5 판정 근거가 아니다.
- REQ-ROB-014: `Stop`은 정지 유지(`stop_latch.StopLatch`)를 건다(SPEC-010 4.5절). 정지 유지 중 `ready=false`, heartbeat `error_code` 101(보호정지면 100), 새 goal은 지령 0회로 `CANCELED`(`STOPPED`)다. 해제는 `stop_release_reason`의 비어 있지 않은 사유로만 하고, 실행 중 goal·보호정지가 있으면 거부한다. lifecycle 비활성·재활성으로 풀리지 않는다. 비활성화는 backend를 멈추고, 정지 유지가 아닐 때의 재활성은 그 정지를 풀어 `ready=true`에서 지령이 실행된다. 받은 Stop은 요청자·사유·수신 steady 시각으로 `stop_history_json` 진단 파라미터(최근 100건, 확인할 수 없는 정지를 감시 노드가 계속 재시도해도 유한)와 로그(`STOP_LATCHED`, 같은 요청자·사유가 이어지면 처음 한 번)에 남긴다(판정에 쓰지 않음).
- REQ-ROB-012: `system/heartbeat`는 10 Hz ±10%로 `seq` 연속 증가, `lifecycle_state`를 싣는다. `robot/status`도 10 Hz로 연결·준비·TCP·파지 상태를 싣는다. 그리퍼 필드는 마지막 skill이 측정한 상태다: 시작은 개방(`holding=false`, `gripper_open=true`, 폭 미상 NaN), PickStone 결과 `holding`이면 측정 폭·전류와 `holding=true`·`gripper_open=false`, PlaceStone·SafeRetreat 반납의 측정 개방 확인 또는 DynamicPlace commit 뒤(`released`) 측정 개방 폭 확인이면 측정 개방 폭과 `holding=false`·`gripper_open=true`. DynamicPlace가 commit했어도 개방을 확인하지 못했으면(`OPEN_NOT_CONFIRMED`·`OPEN_WIDTH`, 측정 폭 없음) 쥔 상태를 그대로 둔다. 쥠 여부나 폭이 바뀌면 skill 결과보다 먼저 한 번 더 발행한다.

### 4.5 DynamicPlace 구조와 해제 transport

`dynamic_place.DynamicPlaceSequencer`가 `sync.period_s`(0.01 s)마다 계획·track·벨트·transport 표본을 받아 phase를 진행하고, 노드는 주기 실행과 feedback·결과 메시지 변환만 한다. 해제 높이 TCP z(`z_rel`)는 계획 `target_tcp_pose.z`(테두리 z + `release_above_rim_m`, SPEC-400 REQ-PLN-004), 테두리 z = `z_rel − release_above_rim_m`, 핑거 끝 z = TCP z − `fingertip_below_tcp_m`이다.

| phase | 기준 | 끝 조건 → 끝 feedback | 실패 |
|---|---|---|---|
| APPROACH | 계획 목표 XY, z = `z_rel + approach_above_m` | 3차원 오차 ≤ `approach_tol_m` → `plan_valid`, `pose_error_m` | `APPROACH_TIMEOUT`(멈춤) |
| SYNCHRONIZE | 같은 기준 | SYNCHRONIZED: `e_xy` ≤ `e_xy_max`·`v_rel` ≤ `v_rel_max`가 연속 `sync_hold_s + sync_hold_margin_s` → `elapsed_s`, `hold_s`, `e_xy_m`, `v_rel_mps` / PREDICTED: `release_time − entry_lead_s` 뒤 `v_rel` ≤ `v_rel_max` → `entry_time_error_s`, `v_rel_mps` | `SYNC_TIMEOUT` |
| DESCEND | z = `z_rel`(프로그램형 transport는 commit 전 접근 높이 유지) | `|z − z_rel|` ≤ `descend_tol_m` → `sync_ok`, `rim_clearance_m`, `height_error_m` | `DESCEND_TIMEOUT`, `SYNC_LOST`, 스트림형 PREDICTED에서 commit 시각까지 못 끝내면 `RELEASE_TIME_MISSED` |
| RELEASE | z = `z_rel`, 해제 게이트 → commit | 개방 확인 뒤 `release_hold_s` → `open_width_m`, `sync_held` | `GATE_<항목>`(해제 전), `OPEN_NOT_CONFIRMED`·`OPEN_WIDTH`·`SYNC_LOST`(해제 뒤, 후퇴는 계속) |
| RETREAT | REQ-ROB-010(XY 속도 정착 뒤 상승) | z ≥ `safe_z_m` → `tcp_z_m`, `outside_workspace`(안전 높이 이상 = 투입 작업영역 밖), `stone_in_gripper`, `sync_speed_error_max_mps` | `SYNC_SPEED`, `RETREAT_TIMEOUT` |

APPROACH·SYNCHRONIZE·DESCEND·해제 전 RELEASE의 XY는 추종 코어(REQ-ROB-013)의 지령이고, 해제 뒤 RELEASE는 고정한 마지막 계획으로 같은 식(REQ-ROB-006)을 쓴다. z는 `z_kp_hz` 비례 지령을 `z_max_speed_mps`·`z_max_accel_mps2`로 제한한다. L3 요청은 결과 FAILED·`released=false`·`abort_request=3`이며, 중단 등급은 orchestrator가 정한다(PLAN.md 5절).

transport 경계는 속도 지령이 아니라 해제 확정(commit)이다(`dynamic_place.PlaceTransport`).

- `capabilities`: `mid_course_update`(commit 뒤에도 `update`를 반영), `commit_lead_s`(commit → 공깃돌이 손가락을 떠나는 시각), `keep_sync_retreat`.
- `update(MotionRef)`: 목표·피드포워드·REQ-ROB-006 지령. `commit_release(ReleasePermit)`: goal당 최대 1회, 인자는 해제 시각·자세·계획 속도·개방 폭·동기 유지 시간·후퇴 z·게이트 값. `poll()`: TCP·속도·시각·그리퍼 폭·`released`·`committed`·`fault`. `abort(keep_sync)`, `stop()`, `is_stopped()`.
- 스트림 + 상주 루프 transport(REQ-ROB-015): `mid_course_update=true`, `commit_lead_s` = 그리퍼 트리거 지연 + 개방 시간, commit = `GripperSession.fire_open`.
- 한 DRL 프로그램 transport(하강·해제·후퇴를 `amovel` + 그리퍼 명령으로 실행, PLAN.md 11.3절): `mid_course_update=false`, `commit_lead_s` = 하강 + 개방 시간, commit = 프로그램 시작(인자 = `ReleasePermit`). commit 전에는 접근 높이에서 추종하고, commit은 DESCEND 중 `release_time − commit_lead_s`에 일어나며 이후 phase는 표본으로 지켜본다. 게이트의 공깃돌 위치는 프로그램 목표 궤적이라 XY 오차를 commit 전에 측정하지 못하고, horizon이 길어 위치 표준편차 0.5 mm를 맞추기 어렵다(추정). 이 transport는 MODE_PREDICTED 수준까지만 지원한다(MODE_SYNCHRONIZED goal은 `PRECONDITION`).
- `backend:=fake`의 `SimulatedPlaceTransport`는 스트림형을 흉내 낸다(TCP 1차 지연 29 ms, 개방 지연 0.1 s, 모두 가정값). 시험에서는 프로그램형(`mid_course_update=false`)도 흉내 낸다.

- REQ-ROB-015: `doosan_stream.DoosanSpeedlTransport`(실기 미검증, 노드 `backend` 선택지에 연결하지 않음)는 `<ns>/speedl_stream`에 `vel`[mm/s×3, deg/s×3]·`acc`[mm/s², deg/s²]·`time` = 2주기를 `motion_adapter` 변환으로 보낸다(`tools/bench/stream_bench.py`와 같은 형식). 지령마다 속도 한도(`RANGE`·`NONFINITE`)와 `FreshnessWatchdog`(2주기, 20.000 ms 유지·20.001 ms L0)를 거치고, 위반·간격 초과·`tick()`이 본 지령 끊김이면 속도 0을 1회 보내고 `rearm()` 전까지 거부하며 `poll().fault`로 알린다. commit은 `GripperSession.fire_open` 1회를 제어 주기 밖(`runner`)에서 실행하고 결과가 오면 `released`이며, 개방 예약(`prepare`)이 없으면 commit 때 쓰고 `late_arm`을 남긴다. `abort(keep_sync=true)`는 `tick()`마다 계획 XY 속도(z 0)를 `abort_sync_hold_s` 동안 보낸 뒤 속도 0을 1회 보낸다. 정지 확인은 TCP 피드백 속도 ≤ `stop_speed_mps`일 때만 참이다. 메시지 형식·발행 함수·TCP 읽기·그리퍼 세션은 주입하고 `dsr_msgs2`를 import하지 않는다.

### 4.6 Doosan 실제 backend 준비(실기 미검증)

REQ-GRP-017·REQ-ROB-017은 코드와 가짜 서비스·가짜 팔 시험까지, REQ-ROB-011은 실제 `dsr_msgs2` 형식 stub 시험까지다. 노드 `backend` 선택지는 `disabled`·`fake` 그대로이며 Doosan backend는 2026-11-14 현장 뒤에 연결한다(`records/reviews/fable-decisions-2026-11-12.md` (d)).

- REQ-GRP-017: `dsr_register_io.DsrRegisterIO`(실기 미검증)는 `resident_transport.RegisterIO`를 Doosan 드라이버 서비스 `<ns>/plc/get_output_register_int`(`address`, `timeout_ms` 1000)·`<ns>/plc/set_output_register_int`(`address`, `value`)·`<ns>/drl/drl_start`(`robot_system` 0, `code`)로 구현한다. 읽은 값은 최상위 바이트만 쓴다(`(value >> 24) & 255`: OFF-2에서 DRL이 쓴 값의 하위 1바이트가 그 자리로 왔다, `static_pick_place.doosan_client.decode_register_byte`와 같은 해석). 쓰기는 0~127 정수만 보내고 그 밖은 송신 없이 예외다. 실패 응답의 쓰기는 False, 읽기는 예외이며, 시간 초과·서비스 없음은 읽기·쓰기 모두 예외다(값을 지어내지 않는다). `drl_start` 전에 `system/get_robot_state` = STANDBY(1)와 `drl/get_drl_state` ∈ {STOP 1, LAST 3}을 확인하고, 아니면 부르지 않고 `ROBOT_NOT_STANDBY`·`DRL_BUSY`다. 서비스 호출(`RosServiceCaller`)은 전역 ROS 인자를 쓰지 않는 전용 helper 노드(`robot_control_dsr_io`)와 그 노드만 도는 executor 스레드에서 하며, 호출하는 노드에서 `spin_until_future_complete`를 부르지 않는다. 제한시간은 steady 시계(`call_timeout_s` 1.0 s, `drl_start_timeout_s` 5.0 s, 제안)로 재고, 넘으면 대기 요청을 지우고 `SERVICE_TIMEOUT`이다. 서비스 형식(`dsr_msgs2.srv`)은 처음 호출할 때 가져오거나 주입한다.
- REQ-ROB-017: `doosan_backend.DoosanBackend`(실기 미검증)는 `skill_logic.StaticBackend`를 Doosan 단위 client 계약(`move_line_async`·`move_stop`·`current_posx`·`robot_state`·`drl_state`)과 `GripperSession`으로 구현하고, SI ↔ Doosan 변환은 `motion_adapter`로 한다(posx [mm, deg ZYZ], `vel` [mm/s, deg/s], `acc` [mm/s², deg/s²]). DRL이 STOP·LAST가 아니거나(`DRL_BUSY`, 그리퍼 상주 루프 실행 중 포함) 로봇이 STANDBY가 아니면(`NOT_STANDBY`) 이동을 보내지 않는다. 이동은 측정 TCP가 목표 위치 `reach_tol_m`·자세 `reach_tol_rad` 안이고 상태가 STANDBY일 때만 완료다. 지령 수락 뒤 `start_grace_s` 안에 MOVING이 보이지 않고 도착하지 않았으면 `MOVE_NOT_STARTED`, 움직인 뒤 목표 밖에서 멈추면 `MOVE_NOT_REACHED`, `거리/속도 + move_timeout_margin_s`를 넘으면 `MOVE_TIMEOUT`이다. cancel·deadline·시간 초과·읽기 실패는 `move_stop`을 보내고 `cancel_stop_timeout_s`(steady) 안에 움직이지 않는 상태(STANDBY 1·SAFE_OFF 3·SAFE_STOP 5)를 확인해야 하며, 확인하지 못하면 `STOP_NOT_CONFIRMED`(REQ-ROB-007 `FAULT`)다. `stop()`은 정지 유지를 걸고(`reset_stop()` 전까지 이동·그리퍼 지령 없음) `move_stop` 뒤 `stop_timeout_s`까지 기다리며, `is_stopped()`는 매번 로봇 상태를 새로 읽는다(읽기 실패는 거짓). 이동 전 측정이 실패하거나 비유한 값이면 지령 없이 실패한다(`TCP_READ_FAILED`, 서비스 오류면 그 이유). TCP 읽기 실패는 NaN이고, 그리퍼 결과는 세션의 측정 폭·전류만 보고하며 그리퍼 예외는 그 이유의 실패다.
  - Doosan 단위 client의 ROS 구현은 `dsr_motion_client.DsrMotionClient`(실기 미검증)다. `ServiceCaller`(노드에서는 `DsrRegisterIO`와 같은 `RosServiceCaller`)로 REQ-ROB-011 표의 `motion/move_line`(`sync_type` 1)·`motion/move_stop`(`stop_mode` 2)·`aux_control/get_current_posx`(`ref` 0, `task_pos_info[0]`의 앞 6개)·`system/get_robot_state`·`drl/get_drl_state`를 부른다. 보내기 전에 `pos`·`vel`·`acc`가 유한한 실수 6·2·2개인지 확인하고 아니면 송신 없이 `ValueError`다(rclpy 기본 설정은 메시지 필드를 검사하지 않는다). `move_line`·`move_stop`의 실패 응답은 False, 읽기의 실패 응답은 `DsrError`(`SERVICE_FAILED`), 비었거나 6개 미만이거나 비유한인 위치 응답은 `DsrError`(`SERVICE_BAD_RESPONSE`), 시간 초과(`call_timeout_s` 1.0 s, 제안)·서비스 없음은 `DsrError`다.

| 키(`DoosanBackendCfg`) | 단위 | 기본값 | 근거 |
|---|---|---|---|
| `v_mps`·`a_mps2` | m/s·m/s² | 0.03·0.05 | `move_v_mps`·`move_a_mps2` 기본값 |
| `ang_v_radps`·`ang_a_radps2` | rad/s·rad/s² | 0.35·0.7 | `static_pick_place.doosan_client.move_line` 기본값(2026-11-09 현장 사용) |
| `reach_tol_m` | m | 0.001 | 같은 함수의 도달 확인 1 mm |
| `reach_tol_rad` | rad | 0.0175 | 제안 1°, TBD(OFF-2) |
| `poll_s` | s | 0.05 | `doosan_client` 감시 주기 |
| `start_grace_s` | s | 1.0 | 제안, TBD(OFF-2) |
| `move_timeout_margin_s` | s | 2.0 | 제안(가감속·서비스 지연), TBD(OFF-2) |
| `cancel_stop_timeout_s` | s | 0.4 | cancel 응답 0.5 s − `poll_s`(생성 때 합 < 0.5 s 검사) |
| `stop_timeout_s` | s | 2.0 | 5절 Stop 확인 대기 |

- REQ-ROB-011: 프로젝트가 Doosan 드라이버(doosan-robot2)에 보내는 요청과 메시지는 아래 표의 이름·형식·필드·배열 길이·단위를 따른다(`<ns>` = `/dsr01`, 이름은 OFF-2 드라이버와 `static_pick_place.doosan_client`·`tools/bench/stream_bench.py`가 쓴 것). 확인은 `mode:=virtual`(지원 시)에서 하고, 지원하지 않으면 실제 `dsr_msgs2` 형식으로 같은 이름의 서비스·토픽을 여는 최소 stub(`src/robot_control/test/fakes/dsr_driver_stub.py`)로 대체한다. virtual 모드는 DRCF 에뮬레이터(docker)가 있어야 하는데 형식 시험을 돌리는 PC(officepc WSL)에는 docker·에뮬레이터가 없어 stub를 쓴다. 시험은 생성 메시지의 필드 검사(`ROS_PYTHON_CHECK_FIELDS`와 같음)를 켜고 `DsrRegisterIO`·`DsrMotionClient`·`DoosanBackend`·`DoosanSpeedlTransport`가 만든 요청을 시험 전용 ROS 도메인에서 그대로 보내며, stub가 아닌 노드가 드라이버 이름(`/dsr01/...` 서비스, `speedl_stream` 구독)을 쓰는 것이 보이면 요청 전에 실패한다. `dsr_msgs2`를 가져올 수 없으면(CI 등) skip하고 사유를 남긴다. stub는 srv 주석의 열거값과 유한값만 검사해 어긋나면 `success=false`로 거부하고, 상태는 고정값(STANDBY 1, DRL LAST 3)이며, `move_line`(base·절대)을 받으면 목표를 곧바로 현재 posx로 둔다. 레지스터 읽기 응답은 OFF-2 관찰 표현(하위 1바이트를 최상위 바이트 자리, 부호 있는 int32)이고 PC가 쓴 값도 같은 표현으로 돌려준다(가정).

| 이름(`<ns>/` 뒤) | 형식(`dsr_msgs2/`) | 요청 필드(프로젝트 값·단위) | 쓰는 응답 필드 | 보내는 코드 |
|---|---|---|---|---|
| `drl/drl_start` | `srv/DrlStart` | `int8 robot_system` 0(real), `string code` | `bool success` | `DsrRegisterIO.drl_start` |
| `drl/get_drl_state` | `srv/GetDrlState` | 없음 | `int8 drl_state`(0 PLAY, 1 STOP, 2 HOLD, 3 LAST), `success` | `DsrRegisterIO`·`DsrMotionClient` |
| `plc/set_output_register_int` | `srv/SetOutputRegisterInt` | `uint16 address`, `int32 value` 0~127 | `success` | `DsrRegisterIO.set` |
| `plc/get_output_register_int` | `srv/GetOutputRegisterInt` | `uint16 address`, `uint32 timeout_ms` 1000 | `int32 value`(`(value >> 24) & 255`, 바이트 128 이상이면 음수), `success` | `DsrRegisterIO.get` |
| `system/get_robot_state` | `srv/GetRobotState` | 없음 | `int8 robot_state`, `success` | `DsrRegisterIO`·`DsrMotionClient` |
| `motion/move_line` | `srv/MoveLine` | `float64[6] pos` [mm×3, deg ZYZ×3], `float64[2] vel` [mm/s, deg/s], `float64[2] acc` [mm/s², deg/s²], `float64 time` 0, `float64 radius` 0, `int8 ref` 0(base), `int8 mode` 0(절대), `int8 blend_type` 0, `int8 sync_type` 1(비동기) | `success` | `DsrMotionClient.move_line_async` |
| `motion/move_stop` | `srv/MoveStop` | `int32 stop_mode` 2(soft stop) | `success` | `DsrMotionClient.move_stop` |
| `aux_control/get_current_posx` | `srv/GetCurrentPosx` | `int8 ref` 0 | `std_msgs/Float64MultiArray[] task_pos_info`(`[0]`의 앞 6개 [mm, deg ZYZ], 7번째 solution space), `success` | `DsrMotionClient.current_posx` |
| `speedl_stream`(토픽, depth 10) | `msg/SpeedlStream` | `float64[6] vel` [mm/s×3, deg/s×3], `float64[2] acc` [mm/s², deg/s²], `float64 time` 2주기 [s] | — | `DoosanSpeedlTransport` |

stub 시험이 확인하지 않는 것(virtual 모드나 OFF-2에서 확인):

- 형식의 출처: 표와 stub는 시험 PC의 `dsr_msgs2`(package 1.1.0)를 따른다. 현장 드라이버(jazzy `dade65d`)의 형식과 같은지는 OFF-2 확인 항목이다.
- 제어기 수락: 실제 드라이버·제어기가 각 요청에 `success=true`를 주는지와 거부 조건(속도·가속 한계, 작업영역, 로봇 모드, 알람, DRL 실행 중).
- 동작과 시간: 이동·정지, robot_state 전이, 서비스 왕복 시간, `move_line`(`sync_type` 1)이 이동 완료 전에 응답하는지, speedl 무지령 정지. stub는 즉시 도착하고 상태가 바뀌지 않는다.
- 레지스터: 제어기가 받는 주소 범위, PC가 쓴 값의 읽기 표현, DRL 쪽 읽기.
- 시리얼·그리퍼: 플랜지 RS-485·Modbus 통신과 상주 루프 DRL 실행.
- QoS: 실제 드라이버 `speedl_stream` 구독의 신뢰성·depth(stub는 depth 10, 기본 신뢰성).

연결 방법(현장 뒤):

1. `backend:=doosan`이면 노드가 `RosServiceCaller` 하나를 만들고, 이를 쓰는 `DsrRegisterIO`와 Doosan 단위 client `DsrMotionClient`(`motion/move_line` `sync_type` 1·`mode` 0·`ref` 0, `motion/move_stop`, `aux_control/get_current_posx` `ref` 0, `system/get_robot_state`, `drl/get_drl_state`)를 둔다.
2. 그리퍼는 `ResidentLoopTransport(DsrRegisterIO)` → `GripperAdapter`(OFF-1 pulse 실측표, 응답 제한시간 ≥ 5 × 표본 주기, 생존 감시 ≥ 3 × 표본 주기, REQ-GRP-016) → `GripperSession`이고, 활성화 때 `start_gripper()`로 상주 루프를 1회 띄운다.
3. `DoosanBackend(client, session, clock, sleep, cancel_requested=노드의 cancel·deadline·Stop 판정)`를 정적 실행기 backend로 쓰고, 작업자 해제 때 `reset_stop()`을 부른다(`backend:=fake`와 같은 경로). DynamicPlace 해제 transport는 OFF-2 결과로 정하며 그 전까지 `PLANNER_NOT_READY`다.
4. 지령 주체 단일(REQ-ROB-005, TC-ROB-005-01)은 helper 노드 `robot_control_dsr_io`를 `robot_controller_node` 프로세스의 일부로 세어 판정한다.

OFF-2에서 확인할 것(결과로 위 기본값·규칙을 고친다):

- 현장 PC에서 `ros2 interface show`로 REQ-ROB-011 표의 srv 8종과 `dsr_msgs2/msg/SpeedlStream` 필드를 표와 비교하고, `ros2 service list -t`·`ros2 topic info -v /dsr01/speedl_stream`으로 이름·형식·구독 QoS를 확인한다(REQ-ROB-011).
- REQ-ROB-011 표의 요청을 프로젝트 client(`DsrRegisterIO`·`DsrMotionClient`·`DoosanSpeedlTransport`)로 보내 각각 `success=true`인지(토픽은 지령대로 움직이는지), `move_line`(`sync_type` 1)이 이동 완료 전에 응답하는지.
- 상주 루프 레지스터 주소(10~14·24·25·30~40)를 `set_output_register_int`·`get_output_register_int`가 받아들이는지.
- 상주 루프 DRL이 PLAY인 동안 `move_line`(`sync_type` 1)이 받아들여지고 움직이는지. 움직이면 `DRL_BUSY` 규칙을 상주 루프가 아닌 DRL로 좁히고, 움직이지 않으면 상주 루프 그리퍼와 이 backend의 이동을 함께 쓰지 않는다.
- `move_line`(`sync_type` 1) 수락 뒤 robot_state STANDBY → MOVING 전이 시간(`start_grace_s` 안인지), 도착 뒤 MOVING → STANDBY, 상태 값(1 STANDBY, 2 MOVING, 3 SAFE_OFF, 5 SAFE_STOP).
- `move_stop`(`stop_mode`, 측정 도구는 2) 뒤 STANDBY까지 시간과 이동 거리가 cancel 0.4 s·Stop 2 s 안인지, 움직이지 않을 때 `move_stop` 응답.
- `get_current_posx`·`get_robot_state` 왕복 시간이 `poll_s`보다 충분히 짧은지(cancel 0.5 s 예산).
- `plc/get_output_register_int`가 PC가 쓴 값(seq 10 등)도 같은 바이트 표현으로 돌려주는지, DRL이 PC가 쓴 값을 `get_output_register_int(addr) & 127`로 읽는지.
- MultiThreadedExecutor 노드의 action 콜백에서 helper 노드를 거친 호출이 끝나는지와 그 지연.

## 5. 시간 제약(주기·제한시간)

- Heartbeat·RobotStatus 10 Hz.
- 스트림 신선도 2×주기.
- 그리퍼 응답 제한시간은 설정값, 판정은 steady 시계.
- cancel 응답 0.5 s, Stop 확인 대기 2 s. Doosan backend의 cancel·deadline 정지 확인 0.4 s(`cancel_stop_timeout_s`), 상태 폴링 0.05 s.
- DynamicPlace 주기 0.01 s(`sync.period_s`), SYNCHRONIZE feedback 0.03 s 간격(≥ 20 Hz), cancel·deadline·Stop은 요청을 본 주기 안에 결과.
- UNCERTAIN 보류 상한 0.6 s(`uncertain_hold_s`), MODE_SYNCHRONIZED 게이트 대기 2.0 s(`release_wait_max_s`).

## 6. 실패 모드와 응답(중단 등급)

- 설정 누락: `PRECONDITION`, 지령 없음.
- 범위·비유한 값: L0, `RANGE`, 송신 없음.
- 신선도 초과: L0, 속도 0 1회 후 래치.
- 그리퍼 모델·펌웨어·모드·오류 비트·시간 초과·루프 정지: `PRECONDITION` 또는 `FAULT`.
- 동시 goal: `BUSY`. lifecycle inactive: `LIFECYCLE_INACTIVE`.
- backend disabled: `BACKEND_DISABLED`, 성공 보고 없음.
- 해제 게이트 위반: FAILED `GATE_<항목>`, 그리퍼 commit·open 0회, L3 요청, 동기 유지한 채 멈춤.
- DynamicPlace 해제 transport 없음(`disabled`): `PLANNER_NOT_READY`. 계획 무효: `PLAN_INVALID`, 계획 없음: `NO_PLAN`, UNCERTAIN 보류 초과: `UNCERTAIN_HOLD`.
- 동기 유지 후퇴 불가: `SYNC_NOT_AVAILABLE`, 속도 이탈: `SYNC_SPEED`(안전 높이까지는 올림).
- SafeRetreat place-back 설정 누락: `PRECONDITION`(`PLACE_BACK_POSE_UNSET` 등), 지령 없음. 후퇴 실패면 반납하지 않음. 개방 폭 불일치: FAILED `OPEN_WIDTH`, `stone_placed_back` 거짓.
- 이동 중단 뒤 정지 미확인: `FAULT`(`STOP_NOT_CONFIRMED`). Doosan 서비스 시간 초과·실패: 값 없이 예외 또는 실패 결과.

## 7. 성공 판정(측정 가능)

- 변환 왕복 회전 오차 ≤ `1e-9 rad`.
- 스트림 유지/거부 경계는 `20.000/20.001 ms`.
- MoveToPose 성공은 측정 `pose_error_m`이 허용오차 이하.
- PickStone 성공은 측정 폭·전류가 파지 기준을 만족하고 `holding`이 참.
- PlaceStone 성공은 측정 개방 폭이 목표 ± 허용오차이고 해제 게이트를 통과.
- Stop 성공은 backend의 실제 정지 피드백이 참이고 실행 goal이 없음.
- SafeRetreat place-back 성공은 측정 이동 오차 ≤ `pose_tolerance_m`, 측정 개방 폭이 목표 ± 허용오차, 안전 높이로 올림 완료. `stone_placed_back`은 개방 확인 뒤에만 참.
- DynamicPlace 성공은 commit 1회, 측정 개방 폭이 목표 ± 허용오차, 해제 뒤 동기 게이트 유지, 후퇴 표본 XY 속도가 계획 속도 ±1 mm/s이고 z ≥ 안전 높이.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- `src/robot_control/test/test_motion_adapter.py`: TC-ROB-001~006.
- `src/robot_control/test/test_gripper_adapter.py`: TC-GRP-001~013. `test_gripper_session.py`: TC-GRP-014. `test_resident_transport.py`: TC-GRP-015·016(가짜 제어기 `test/fakes/resident.py`, 시간은 모델 값. DRL 본문은 `test/fakes/drl_env.py`에서 그대로 실행).
- `src/robot_control/test/test_skill_logic.py`: TC-ROB-005·007, TC-ROB-007-91(전류 한도 1~150 mA)·92(PlaceStone RELEASE·RETREAT 측정 지표), TC-ROB-008-03(transport 없음), TC-ROB-010-02(정적 실행기).
- `src/robot_control/test/test_dynamic_place.py`: TC-ROB-007-03, TC-ROB-008-01~03·91·92(수신 쪽이 다시 재는 동기 유지 시간), TC-ROB-009-01~03·05, TC-ROB-010-01·02·91(상승 전 XY 속도 정착)(`SimulatedPlaceTransport`, FakeClock). `test_release_gate.py`: TC-ROB-009-04.
- `src/robot_control/test/contract/test_robot_controller_contract.py`: TC-ROB-008-04(실제 노드 `backend:=fake`가 `tools/testing/contracts/robot_controller_contract.py` 통과, C).
- `src/robot_control/test/test_doosan_stream.py`: TC-ROB-015-01(가짜 발행·세션만). `test_motion_adapter.py`: TC-ROB-002-02.
- `src/robot_control/test/test_belt_sync.py`: TC-ROB-013-01~05(속도·가속 제한은 TC-ROB-006-01도 함께). `test_sync_revision.py`·`test_sync_config.py`: TC-ROB-013-06. `test_sync_metrics.py`: TC-ROB-013-07. `test_sync_uncertain_hold.py`: TC-ROB-013-08.
- `src/robot_control/test/test_stop_latch.py`: TC-ROB-014-01. `test_robot_controller_stop.py`: TC-ROB-014-02. `test_robot_controller_status.py`: TC-ROB-012-91(그리퍼 상태, C), TC-ROB-014-91(재활성 뒤 지령, C), TC-ROB-014-02(Stop 진단 기록).
- I0(`src/drop_bringup/test/test_i0_real_controller.py`): TC-INT-014-01~018-01. 오케스트레이터 + 실제 노드 `backend:=fake` + 실제 tracker·planner + mock vision으로 DynamicPlace(두 mode)·PlaceStone·SafeRetreat place-back(통과 보내기 소진, 해제 전 동적 구간 실패의 동기 유지 후퇴)을 돈다. 시뮬레이션 backend 값은 하드웨어 측정이 아니다.
- `src/robot_control/test/test_safe_retreat_place_back.py`: TC-ROB-016-01~04. `test_robot_controller_place_back.py`와 `contract/test_robot_controller_contract.py`(place-back 1건): TC-ROB-016-05(C).
- `src/robot_control/test/test_dsr_register_io.py`: TC-GRP-017-01(가짜 서비스 `test/fakes/dsr.py`). `test_dsr_service_caller_ros.py`: TC-GRP-017-02(C, `std_srvs` 서버). `test_doosan_backend.py`: TC-ROB-017-01~03(가짜 팔 `test/fakes/doosan.py`). `test_dsr_motion_client.py`: TC-ROB-017-04(가짜 caller). 모두 실기 미검증 코드의 시험이다.
- `src/robot_control/test/test_dsr_driver_stub_format.py`: TC-ROB-011-01(C, 실제 `dsr_msgs2` 형식 stub `test/fakes/dsr_driver_stub.py`, `dsr_msgs2`가 없으면 skip, 판정은 `dsr_msgs2`가 있는 PC).
- `robot_controller_node` 계약: `tools/testing/contracts/robot_controller_contract.py`가 기대하는 feedback phase·metric, lifecycle, heartbeat, Stop.
- `[ON]` 항목은 실행 전까지 완료로 표시하지 않는다.

## 9. 미결 사항

- 실제 로봇의 DynamicPlace 동작 전부(실기 미검증): speedl 스트림과 상주 루프 동시 동작, TCP 피드백 경로·주기·나이(`max_tcp_age_s`), 그리퍼 트리거 → 개방 지연(`commit_lead_s`·`release_hold_s`·`release_timeout_s`), `move_stop` 정지 확인(OFF-2). `fake_*` 값은 그 측정으로 바꾼다.
- `fingertip_below_tcp_m`(OFF-6), 동적 투입의 `release_above_rim_m`(SPEC-400, OFF-6).
- 한 DRL 프로그램 transport 렌더러와 OFF-2 결과에 따른 transport 선택, 노드 `backend` 연결.
- 실제 드라이버에서 REQ-ROB-011 형식 확인(현장 `dsr_msgs2` 형식 비교, 요청별 수락)과 virtual 모드(DRCF 에뮬레이터) 시험 환경.
- 그리퍼 상주 루프 DRL과 생존 카운터 실기 시험.
- SafeRetreat 동기 유지·place-back 실측: 매트 반납 위치 `place_back_xyz_m`(측정 기록으로 채움), 실제 이동 속도에서 L3 경로 제한시간(PLAN.md 5.2절 15 s) 안에 끝나는지.
- Doosan 실제 backend(4.6절)의 OFF-2 확인 항목과 노드 연결(`DsrMotionClient`·`DsrRegisterIO`).
