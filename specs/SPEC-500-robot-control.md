# SPEC-500 robot-control

- 버전: v0.5
- 근거: PLAN.md 1.1, 3.3, 5.1절
- REQ 영역: ROB, GRP

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.5 | 2026-11-11 | REQ-ROB-014 Stop 정지 유지·작업자 해제(`stop_latch.StopLatch`) |
| v0.4 | 2026-11-11 | REQ-ROB-013 XY 추종 코어(`belt_sync.BeltSynchronizer`): 계획 수락 조건, 감시, 제한·작업영역, 정지 유지와 명시적 재무장, 그림자 미리보기. REQ-GRP-016 지령 결과 레지스터 40과 `RESIDENT_WRITE_FAILED`, 실행 확인 뒤 새 표본으로 판정, 표본 주기 대비 제한시간 검사(응답 5주기·생존 3주기), REQ-GRP-013 생존 감시 기준을 `loop_timeout_s`로 |
| v0.3 | 2026-11-10 | REQ-GRP-014 공통 그리퍼 순서(`GripperSession`), 예약(prewrite) 무효화 규칙, REQ-GRP-015 막힘 파지 판정, REQ-GRP-016 상주 루프 transport·DRL 레지스터 계약, 그리퍼 전류 상한 100 → 150 mA |
| v0.2 | 2026-11-10 | REQ-ROB-007: Stop으로 선점된 goal은 `CANCELED`·`STOPPED`, cancel은 `CANCELED`·`CANCELED` |
| v0.1 | 2026-11-07 | ROB·GRP REQ 정의, SI/Doosan·그리퍼 adapter·정적 skill 계약, 기본 backend disabled |
| v0.0 | 2026-11-07 | 현장 일회성 그리퍼 CLI 요구(REQ-GRP-004·011) |

## 1. 목적·범위

Doosan 스트림·move_line 계약, 그리퍼 상주 루프 경로, 추종 제어, L0. 정적 skill(MoveToPose·PickStone·PlaceStone·SafeRetreat·Stop)은 `robot_controller_node`가 노출하고 판단 로직은 ROS와 무관한 순수 Python 모듈에 둔다.

## 2. 입력·출력(interface 버전)

`src/drop_interfaces` 스냅샷(SPEC-010)의 `MoveToPose`·`PickStone`·`PlaceStone`·`DynamicPlace`·`SafeRetreat` action, `Stop` service, `RobotStatus`·`Heartbeat` message를 그대로 쓴다. 토픽은 `robot/status`, `system/heartbeat`, service는 `robot/stop`이다. 결과 코드는 interface `Result` 값을 사용하며, 성공은 실제 측정값이 기준을 만족할 때만 보고한다.

기본 backend는 `disabled`다. 어떤 파라미터도 `/dsr01` 실기 연결을 암묵적으로 만들지 않는다. `backend:=fake`는 시뮬레이션 backend만 쓰며 그 값은 하드웨어 측정으로 표시하지 않는다. 필수 설정값이 없으면 skill 결과는 `PRECONDITION`이다.

## 3. 파라미터(단위·기본값·근거)

수치 키는 단위 접미사를 붙인다. 기하·그리퍼 폭·전류 임계는 실측 기록이 있을 때만 채우며, 없으면 `-1`(미설정)로 두고 성공 판정 대신 `PRECONDITION`을 낸다.

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

그리퍼 어댑터 설정에는 pulse 표 경로, protocol(`dynamixel2`·`modbus`), slave ID, 응답 제한시간, 기본 전류 `100 mA`, 최대 전류 `150 mA`(허용 `1~150`, 2026-11-09 실기 파지에 쓴 값)가 있다. 현장 CLI 운용 범위 `0~750 pulse`는 어댑터의 하드웨어 능력 `0~1150 pulse`와 별개이며 반전 규약을 쓰지 않는다.

## 4. 동작(상태·알고리즘)

### 4.1 공통

- REQ-ROB-001: SI 위치·quaternion과 Doosan move_line 위치·ZYZ 오일러(mm·deg)를 양방향 변환한다. 왕복 회전 오차는 `1e-9 rad` 이하이고, β≈0·π 특이 자세는 같은 회전이면 통과한다.
- REQ-ROB-002: 병진 `m/s`는 `mm/s`, 회전 `rad/s`는 `deg/s`로 변환한다. 변환은 `motion_adapter.py`에서만 한다.
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

- REQ-ROB-007: `MoveToPose`·`PickStone`·`PlaceStone`·`SafeRetreat`·`Stop`은 phase feedback과 결과 코드를 내고, cancel은 0.5 s 안 `CANCELED`(이유 `CANCELED`), 실행 중 `Stop`으로 선점된 goal은 `CANCELED`(이유 `STOPPED`, SPEC-010 3.2절), deadline 초과는 `TIMEOUT`, 실행 중 `Stop`은 goal 선점 후 정지 피드백 확인 뒤에만 `stopped=true`다. MoveToPose는 측정 자세 오차, PickStone은 측정 파지, PlaceStone은 측정 개방 폭으로 성공을 판정한다.
- REQ-ROB-008: `DynamicPlace`는 plan revision·epoch·만료를 검사하고, 계획기·추종 제어가 준비되지 않은 동안 `PRECONDITION`(`PLANNER_NOT_READY`)으로 닫힌다.
- REQ-ROB-009: 해제 게이트(높이·경사·폭·여유·정지) 중 하나라도 위반하면 그리퍼를 열지 않고 L3를 요청한다.
- REQ-ROB-010: `SafeRetreat(keep_sync=true)`는 z가 안전 높이에 닿을 때까지 XY 속도를 계획 속도 ±1 mm/s로 유지한다. 동기 유지 또는 place-back을 지원하지 않는 경로는 성공으로 보고하지 않는다.
- REQ-ROB-013: XY 추종 코어(`belt_sync.BeltSynchronizer`)는 ROS·로봇·그리퍼 입출력 없이 SI·`robot_base`로 동작한다. 목표는 `release_xy + v·(t − release_time)`이다(입력 상태 시각이 아니라 해제 시각 기준). `arm(plan_id, track_id, epoch)`으로 지정한 계획만 받고, 다른 식별자가 오면 `PLAN_IDENTITY_CHANGED`로 멈춘다. 받은 revision 이하는 무시하며 수신 감시를 갱신하지 않는다. `status`≠OK, `frame_id`≠`robot_base`, 비유한 값, `position_std` 초과, 시각 순서(`source_state_stamp ≤ 생성 ≤ 현재`) 위반, 입력 상태 나이 초과, 만료는 `PLAN_INVALID`다. 매 주기 계획 없음은 `WAIT_PLAN`, 비유한 TCP는 `NONFINITE`, ROS 시계 역행은 `CLOCK_BACKWARD`, steady 주기 간격 초과는 `LOOP_GAP`, 만료는 `PLAN_EXPIRED`, 수신(steady)·입력 상태(ROS) 나이 초과는 `PLAN_STALE`, TCP frame·나이 위반은 `TCP_STALE`, 목표·TCP·한 주기 예상 위치가 XY 작업영역 밖이면 `WORKSPACE`다. 지령은 REQ-ROB-006 식과 제한을 따른다. 멈춤(`stop` 포함)은 래치되어 출력은 비활성·속도 0이며, 같은 계획이라도 `arm`으로 다시 무장하기 전까지 계획을 받지 않는다. 비활성 출력의 0은 정지 증거가 아니다. 설정값은 유한한 양수이고 주기 ≤ 간격 한도, 작업영역 최소 < 최대여야 하며 아니면 생성 시 예외다. `belt_sync_preview`는 `/tracking_preview/cmd_vel`·`/tracking_preview/status`만 발행하고 Doosan·그리퍼 지령을 보내지 않는다.
- REQ-ROB-014: `Stop`은 정지 유지(`stop_latch.StopLatch`)를 건다(SPEC-010 4.5절). 정지 유지 중 `ready=false`, heartbeat `error_code` 101(보호정지면 100), 새 goal은 지령 0회로 `CANCELED`(`STOPPED`)다. 해제는 `stop_release_reason`의 비어 있지 않은 사유로만 하고, 실행 중 goal·보호정지가 있으면 거부한다. lifecycle 비활성·재활성으로 풀리지 않는다.
- REQ-ROB-012: `system/heartbeat`는 10 Hz ±10%로 `seq` 연속 증가, `lifecycle_state`를 싣는다. `robot/status`도 10 Hz로 연결·준비·TCP·파지 상태를 싣는다.

## 5. 시간 제약(주기·제한시간)

- Heartbeat·RobotStatus 10 Hz.
- 스트림 신선도 2×주기.
- 그리퍼 응답 제한시간은 설정값, 판정은 steady 시계.
- cancel 응답 0.5 s, Stop 확인 대기 2 s.

## 6. 실패 모드와 응답(중단 등급)

- 설정 누락: `PRECONDITION`, 지령 없음.
- 범위·비유한 값: L0, `RANGE`, 송신 없음.
- 신선도 초과: L0, 속도 0 1회 후 래치.
- 그리퍼 모델·펌웨어·모드·오류 비트·시간 초과·루프 정지: `PRECONDITION` 또는 `FAULT`.
- 동시 goal: `BUSY`. lifecycle inactive: `LIFECYCLE_INACTIVE`.
- backend disabled: `BACKEND_DISABLED`, 성공 보고 없음.
- 해제 게이트 위반: L3, 그리퍼 open 0회.
- DynamicPlace 미준비: `PLANNER_NOT_READY`.

## 7. 성공 판정(측정 가능)

- 변환 왕복 회전 오차 ≤ `1e-9 rad`.
- 스트림 유지/거부 경계는 `20.000/20.001 ms`.
- MoveToPose 성공은 측정 `pose_error_m`이 허용오차 이하.
- PickStone 성공은 측정 폭·전류가 파지 기준을 만족하고 `holding`이 참.
- PlaceStone 성공은 측정 개방 폭이 목표 ± 허용오차이고 해제 게이트를 통과.
- Stop 성공은 backend의 실제 정지 피드백이 참이고 실행 goal이 없음.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- `src/robot_control/test/test_motion_adapter.py`: TC-ROB-001~006.
- `src/robot_control/test/test_gripper_adapter.py`: TC-GRP-001~013. `test_gripper_session.py`: TC-GRP-014. `test_resident_transport.py`: TC-GRP-015·016(가짜 제어기 `test/fakes/resident.py`, 시간은 모델 값. DRL 본문은 `test/fakes/drl_env.py`에서 그대로 실행).
- `src/robot_control/test/test_skill_logic.py`: TC-ROB-007~010.
- `src/robot_control/test/test_belt_sync.py`: TC-ROB-013(속도·가속 제한은 TC-ROB-006-01도 함께).
- `src/robot_control/test/test_stop_latch.py`: TC-ROB-014-01. `test_robot_controller_stop.py`: TC-ROB-014-02.
- `robot_controller_node` 계약: `tools/testing/contracts/robot_controller_contract.py`가 기대하는 feedback phase·metric, lifecycle, heartbeat, Stop.
- `[ON]` 항목은 실행 전까지 완료로 표시하지 않는다.

## 9. 미결 사항

- DynamicPlace 계획기·추종 제어와 plan revision/epoch 계약.
- 실제 로봇 `/dsr01` move_line·stream 연결과 virtual 모드 지원 확인.
- 그리퍼 상주 루프 DRL과 생존 카운터 실기 시험.
- SafeRetreat의 동기 유지·place-back 구현과 실측 검증.
