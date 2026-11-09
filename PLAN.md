# PLAN — 이동 병뚜껑 공깃돌 투입 개발 계획

- 확정: 2026-11-05 / 진행 계획·게이트: `Progress.md` / 시험 명세: `TEST_SPEC.md`
- 수치는 모두 **제안값(초기 가설)**이다. G0 실측값이 나오면 바꾸고, 바꾼 내용은 `records/param_changes.csv`에 남긴다(10.4절).

## 0. 원칙

### 0.1 개발 원칙

1. **단일 지휘:** 작업 순서와 명령은 오케스트레이터만 정한다. 기능 노드는 맡은 기능만 수행한다(2절).
2. **단계 판정 후 진행:** 모든 단계는 측정 가능한 성공 판정을 통과해야 다음 단계로 넘어간다. 개발 단계도 게이트를 통과해야 넘어간다(4절, `Progress.md`).
3. **모든 동작에 제한시간:** 단계·호출·데이터·사이클·중단 경로마다 제한시간을 두고, 넘으면 정해진 중단 등급으로 처리한다(5, 6절).
4. **최소 대역으로 병렬 통합:** 기능 노드 단독 개발은 테스트 클라이언트와 합성 입력으로 하고, ROS mock 노드는 오케스트레이터 연동·장애 주입·실기 대체에만 쓴다. 실제 노드가 계약 테스트를 통과하면 하나씩 교체한다. 최적화는 기능 통합 뒤에 한다(7, 12절).
5. **spec·시험 명세 우선(TDD):** spec → 시험 명세(`TEST_SPEC.md`) → 실패하는 테스트 → 구현 순서로 진행한다(8절).
6. **정량 기록:** 모든 시도와 단계를 수치로 기록하고, 판단은 기록된 수치로 한다(10절).
7. **온라인 우선:** 실기 장비 없이 할 수 있는 일은 온라인에서 먼저 끝내고, 현장(오프라인) 시간은 실기에서만 할 수 있는 측정·시험에 쓴다(11.4절).

### 0.2 설계 규약

1. **단위·좌표:** 프로젝트 내부는 SI(m, s, rad)와 quaternion만 쓴다. Doosan의 mm/deg·Euler(ZYZ) 변환은 `robot_control/motion_adapter.py` 한 곳에서만 하고 단위 테스트로 고정한다. 그리퍼 pulse ↔ 핑거 간격 변환은 `robot_control/gripper_adapter.py` 한 곳에서만 한다.
2. **시각:** 모든 관측 메시지의 `header.stamp`는 *영상 노출 시각*이다. 수신 시각으로 덮어쓰지 않는다. 상태(`t_state`)·마지막 관측(`t_last_obs`)·계획 생성·해제 예정(`t_release`)·유효기간(`valid_until`)을 별도 필드로 둔다.
3. **TF와 예측 분리:** TF는 고정 좌표 관계와 *현재까지 관측된* 병뚜껑 위치 시각화에만 쓴다. tf2는 미래 시각을 외삽하지 않는다(`lookupTransform`이 extrapolation 오류를 냄). 미래 위치는 `TrackedObject`(위치·속도·공분산)로 planner가 계산한다.
4. **Depth 미사용:** 공깃돌·병뚜껑의 XY는 왜곡 보정 RGB 중심과 *보정된 평면*(매트·벨트 + 대상물별 높이 오프셋)의 광선 교차로 구한다. D455 Depth는 평면 보정 확인과 이상 감시에만 쓴다. 최대 Depth 해상도(1280×720)의 Min-Z가 약 520 mm라는 점도 고려한다.
5. **지령 권한 단일화:** 로봇과 그리퍼에 지령을 보내는 주체는 `robot_controller_node` 하나뿐이다. 오래된 지령(freshness 초과)은 보내지 않고, 감시 타이머가 정지 지령을 낸다.
6. **판단 로직과 ROS 노드 분리:** 검출·필터·예측·시간 예산·단위 변환·상태 전이·단계 판정·중단 판정은 ROS와 무관한 순수 Python 모듈로 만들어 패키지별 `test/`에서 단위 테스트하고, 노드는 그 모듈을 감싸기만 한다.

## 1. 목표와 범위

- 작업: 녹색 컨베이어 벨트에 납작한 병뚜껑을 붙여 두고 벨트를 돌린다. 로봇은 검은 매트 위 공깃돌을 집어 벨트 위 대기 자세에서 기다리고, 영상으로 병뚜껑의 위치·속도를 추정해 이동 중인 병뚜껑 안에 공깃돌을 넣는다.
- 환경: Ubuntu 24.04 Native / ROS 2 Jazzy / Isaac Sim 5.1, Doosan E0509 + ROBOTIS RH-P12-RN(A) 그리퍼, Intel RealSense D455 작업대 고정(eye-to-hand). 컨베이어는 수동 다이얼로 돌리며 엔코더가 없으므로 ROS 제어 패키지·속도 명령 토픽은 없다.
- 단계별 목표(쉬운 것부터): 정지 벨트 투입(G2) → 예측 타이밍 투입 `MODE_PREDICTED`(G4) → 속도 동기 투입 `MODE_SYNCHRONIZED`(G6) → 통합·복구(G7).
- 제외: 컨베이어 속도 제어, 측면 보조 카메라, 공깃돌 자세(yaw) 관측, 병뚜껑에 들어간 공깃돌의 자동 회수(시도 사이에 작업자가 회수).
- 외부 의존성: RealSense ROS 2 Wrapper, doosan-robot2(Jazzy). 설치 커밋 SHA와 제어기·DRFL 버전을 README에 고정 기록한다.

### 1.1 장비와 대상물

| 항목     | 내용                                                                                                                                                                                      | G0에서 확정할 것                                                                                     |
| ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| 로봇     | Doosan E0509                                                                                                                                                                            | 제어기·DRFL 버전, 사용할 스트림 인터페이스(3.3절)                                                               |
| 그리퍼    | ROBOTIS RH-P12-RN(A)(신형 펌웨어). 500 g, 행정 0~~106 mm(하드웨어 Rev0은 0~~109 mm), 최대 닫힘 속도 75 mm/s, 최대 파지력 170 N, 정격 24 V, 위치 0\~750 pulse                                                       | 하드웨어 Rev, 펌웨어 버전, pulse ↔ 핑거 간격 실측표, 핑거 어댑터 치수                                                 |
| 그리퍼 연결 | E0509 플랜지 RS-485 배선 유지. ROS 2 → doosan-robot2 → 제어기 DRL(`flange_serial_*`) → Modbus-RTU → 그리퍼. 공개 참고 구현: E0509 + RH-P12-RN(A)용 `dsr_study`(`dsr_gripper` 패키지, 서비스 `/dsr01/gripper/cmd`) | 그리퍼 Protocol Type이 Modbus-RTU인지, 상주 루프 DRL이 도는 동안 `move_line`·스트림 지령이 받아들여지는지, 명령 지연, 상태 읽기 경로 |
| 카메라    | Intel RealSense D455, 작업대 고정                                                                                                                                                            | 설치 높이·각도                                                                                       |
| 병뚜껑    | 납작한 병뚜껑, 벨트에 접착                                                                                                                                                                         | 안쪽 지름 `D_in`, 테두리 높이 `h_rim`, 바닥 두께, 색, 롤러·벨트 하부 통과 여부, 한 바퀴 주기                                |
| 공깃돌    | 검은 매트 위에 놓음. 팔각기둥, 지름 `d_stone_min` 17 mm(면-면)·`d_stone_max` 18 mm(꼭짓점-꼭짓점)                                                                                                             | 높이, 질량, 색, 놓인 자세, 낙하 시 튕김                                                                      |

그리퍼 운용 규칙:

- **운전 모드:** 전류 기반 위치 제어(Operating Mode 5)를 쓴다. 최대 파지력 170 N은 공깃돌에 과하므로 Goal Current로 파지력을 제한한다. 값은 G0에서 공깃돌이 미끄러지지 않는 최소 전류에 여유를 곱해 정한다.
- **부분 개방:** 해제할 때는 완전히 열지 않고 핑거 간격 `d_stone_max + Δ_open`(초기 4 mm)까지만 연다. 이동 거리가 짧아 해제 시간이 줄고, 핑거가 병뚜껑 테두리에 닿지 않는다.
- **파지·개방 판정:** Present Position을 실측표로 핑거 간격으로 바꾸고, Present Current와 함께 판정한다(4.3절).
- **주요 주소(RH-P12-RN(A) control table, DYNAMIXEL 바이트 주소):** Torque Enable 512, Hardware Error Status 518, Goal Current 550, Profile Velocity 560, Goal Position 564, Present Current 574, Present Position 580. 구형 RH-P12-RN 펌웨어와 주소가 다르므로 펌웨어를 바꾸면 이 표를 다시 확인한다.
- **Modbus 주소:** 플랜지 경로는 Modbus-RTU(FC03 읽기, FC06·FC16 쓰기)를 쓴다. 참고 구현의 쓰기 주소는 바이트 주소의 절반과 일치한다(Torque Enable 256, Goal Current 275, Goal Position 282, Present Current 287, Operating Mode는 워드 5의 상위 바이트). ROBOTIS RH-P12-RN(A) 제어표의 Modbus 주소(40001 = 0)로는 Present Position 290(2워드), Hardware Error Status 259(하위 바이트), Model Number 0, Firmware Version 3(하위 바이트), Protocol Type 6(상위 바이트)이다. 주소는 제어표의 Modbus 열에서 옮겨 `robot_control.yaml`에 두고, 바이트 주소의 절반과 다른 항목(제어표의 Position P·I Gain 행)은 실기 읽기로 확인한 뒤 쓴다. RH-P12-RN(A)의 Modbus-RTU는 펌웨어 V12 이상에서 쓸 수 있다. 주소표는 프로토콜별로 따로 두고 섞지 않는다.
- **공구 설정:** 그리퍼(500 g)와 핑거 어댑터의 무게·무게중심을 Doosan 공구 설정에 등록해 충돌 감지 오작동을 막는다.
- **명령 경로:** 그리퍼 명령은 `gripper_adapter.py`만 보내고, 제어기 호출 방식은 그 안에 감춘다. 다른 모듈은 `open(width)`, `close(current_limit)`, `read_state()`만 쓴다.
- **상주 루프 방식:** bringup 때 로봇이 auto 모드·STANDBY일 때 `<dsr_ns>/dsr_controller2/drl/drl_start`로 상주 루프 DRL을 한 번 띄운다(시리얼을 열어 두고 그리퍼 init 1회). 이후 ROS는 `<dsr_ns>/dsr_controller2/plc/set_output_register_int`로 명령 레지스터에 쓰고, 루프가 20 ms 주기로 읽어 실행한다. 호출마다 DRL을 새로 띄우는 방식은 시리얼 open/close와 정착 대기 때문에 호출당 1 s 이상 걸리므로 해제에 쓰지 않는다.
- **위치 값 규약:** 프로젝트 코드와 pulse 실측표는 그리퍼 하드웨어 Goal Position 값(0~1150 pulse)을 그대로 쓴다. 참고 구현 서비스(`/dsr01/gripper/cmd`)는 요청 값을 0~750으로 제한하고 `750 − 값`으로 반전해 보내므로 하드웨어 751~1150 pulse(좁은 핑거 간격)를 지령할 수 없다. pulse 실측표 측정과 공깃돌 파지는 이 서비스를 거치지 않고 상주 루프의 명령 레지스터에 하드웨어 값을 직접 쓴다(`tools/bench/gripper_pulse_table.py`).
- **해제 트리거:** 참고 구현은 명령 하나에 레지스터 서비스 호출 5회(값 4개 + seq)를 쓴다. 해제 지연을 줄이려 개방 목표값을 RELEASE 전에 미리 써 두고, 해제 순간에는 트리거 레지스터 하나만 쓴다.
- **상태 읽기·생존 확인:** 참고 구현에는 상태 읽기가 없다. 루프가 FC03으로 Present Position·Present Current·Hardware Error Status를 읽어 ROS가 읽을 수 있는 레지스터에 쓰고, 생존 카운터를 매 주기 올리도록 추가한다(정확한 레지스터 방향·서비스는 G0에서 확정).

## 2. 시스템 구조

### 2.1 계층

```
                     ┌──────────────────────────────┐
  사용자/시험 스크립트 ─▶│ orchestrator_node             │  DropTask 액션 서버
                     │  - 상태 머신 / 단계 실행기       │  단계 성공 판정 · 제한시간 · 재시도
                     │  - abort_policy (L1~L3)        │  StepRecord 발행
                     └──────┬───────────────┬────────┘
        skill 호출(action/srv) │               │ 상태 구독(topic)
   ┌──────────────┬─────────┴────┬──────────┴─────┬──────────────────┐
   ▼              ▼              ▼                ▼                  ▼
vision_node   tracker_node   intercept_planner   robot_controller   (calibration 도구:
 DetectObjects ResetTracking   ComputeIntercept    PickStone          실행 중에는 사용 안 함)
 VerifyDrop    SelectTarget                        MoveToPose
                                                   PlaceStone
                                                   DynamicPlace
                                                   SafeRetreat / Stop
   │ detections ─▶ │ tracked ─▶ │ intercept_plan ─▶│ (데이터 스트림: 기능 노드 사이 topic 허용)
                                                   ▼
                                  Doosan 드라이버 · 그리퍼(제어기 상주 DRL 경유)  (L0 freshness 감시)

 safety_supervisor_node : 모든 노드 Heartbeat·로봇 상태 감시 → L4 판정, robot_controller에 Stop 요청
 record_logger_node     : StepRecord·TrialRecord를 records/에 JSONL로 저장
```

핵심 데이터 흐름은 `Vision → Tracking → Planning → Robot Control`이고, 오케스트레이터가 순서·중단·판정을 관리한다.

### 2.2 지휘 규칙

1. **명령 흐름은 오케스트레이터에서만 나간다.** 기능 노드끼리 서로의 service/action을 부르지 않는다. 기능 노드는 작업 순서를 모른다.
2. **데이터 흐름은 topic으로 둔다.** `detections → tracked → intercept_plan` 같은 연속 데이터는 기능 노드 사이에서 직접 주고받는다(명령은 star, 데이터는 pipeline). 동기 추종 중 `robot_controller`는 goal에 지정된 `plan_id`·`epoch`과 일치하는 `intercept_plan`만 사용하고, 이미 받은 것 이하의 revision은 버린다. epoch이 바뀌거나 plan이 만료되면 해당 goal을 `FAILED`(reason `PLAN_INVALID`)로 끝낸다. plan 스트림은 action goal과 따로 갱신되므로 둘의 일치·취소·만료를 함께 시험한다.
3. **로봇 지령 주체는 `robot_controller_node` 하나다.** `safety_supervisor`도 로봇에 직접 지령하지 않고 `robot_controller`의 우선순위 `Stop` 서비스를 부른다. `robot_controller` 자체도 지령이 끊기면 스스로 멈춘다(L0, 다른 노드에 의존하지 않음).
4. **오케스트레이터가 보내는 모든 action goal과 service 요청에는 `deadline`, `cycle_id`, `step_id`가 들어간다.** 서버는 deadline이 지나면 스스로 중단하고 `TIMEOUT`으로 응답한다. 오케스트레이터 쪽 제한시간과 이중으로 걸린다. service는 deadline 안에 끝나는 짧은 조회·계산에만 쓰고, 로봇 동작처럼 오래 걸리거나 취소가 필요한 일은 action으로 만든다(service client의 제한시간은 서버 작업을 취소하지 않는다).
5. **기능 노드는 lifecycle(managed) 노드로 만든다.** 기동 순서와 PRECHECK에서 모든 노드가 `ACTIVE`인지 확인하는 데 쓴다.
6. **필터는 tracker에서 한 번만 적용한다.** planner는 tracker 상태를 받아 예측만 하고 다시 필터링하지 않는다.
7. **기능 노드는 입력 topic만으로 동작한다.** 다른 기능 노드가 떠 있다고 가정하지 않으며, 입력은 rosbag·합성 publisher로 바꿀 수 있어야 한다.

### 2.3 패키지 책임 (10개)

| 패키지 | 책임 | 소유하는 상태/결정 | 노드 | 지휘용 인터페이스 |
|---|---|---|---|---|
| `drop_interfaces` | msg·srv·action 정의 | — | — | 3절 |
| `vision` | 공깃돌·병뚜껑 검출, 보정 평면 교차로 XY 계산, 로봇 가림 마스크, 투입 판정 | 관측(측정값·공분산) | `vision_node` | `DetectObjects`, `VerifyDrop` |
| `tracking` | ID 부여, 칼만 필터, 벨트 속도·변화 검출, 관측 TF | 병뚜껑 상태 `[x,y,vx,vy]`, `OBSERVED/COASTING/LOST` | `tracker_node` | `ResetTracking`, `SelectTarget` |
| `intercept_planner` | 시간 예산, 해제 시각·위치, 도달성 | 미래 예측(유일 소유) | `intercept_planner_node` | `ComputeIntercept` |
| `robot_control` | Doosan·그리퍼 어댑터, 정적 시퀀스, 동기 추종 루프 | 로봇·그리퍼 지령 권한, freshness 감시(L0) | `robot_controller_node` | `PickStone`, `MoveToPose`, `PlaceStone`, `DynamicPlace`, `SafeRetreat`, `Stop` |
| `orchestrator` | 상태 머신, 단계 판정, 중단·제한시간, 안전 감시, 기록 | 작업 단계, 중단 등급, 결과 | `orchestrator_node`, `safety_supervisor_node`, `record_logger_node` | `DropTask` 서버 |
| `calibration` | 내부·외부·평면·TCP 보정, 검증 점 오차 | calibration_id | 보정 도구 | — |
| `drop_mocks` | 시나리오 엔진·장애 주입(7절) | — | `mock_world_node`, `mock_robot_controller_node` | 실제 노드와 같은 인터페이스 |
| `drop_bringup` | launch·공통 설정·통합 테스트 | namespace, `use_sim_time`, `use_mock_<node>` | launch | — |
| `drop_sim` | Isaac Sim 장면·컨베이어·가상 카메라 | sim GT(평가 전용) | 시뮬레이션 스크립트 | — |

### 2.4 디렉터리 구조

```
drop_ws/
├── src/
│   ├── drop_interfaces/                     # ament_cmake + rosidl (rosidl_interface_packages 그룹)
│   │   ├── msg/      ObjectDetection, ObjectDetectionArray, TrackedObject, TrackedObjectArray,
│   │   │             BeltMotion, InterceptPlan, TaskStatus, Heartbeat, StepRecord
│   │   ├── srv/      DetectObjects, VerifyDrop, ComputeIntercept, ResetTracking, SelectTarget, Stop
│   │   ├── action/   DropTask, PickStone, MoveToPose, PlaceStone, DynamicPlace, SafeRetreat
│   │   └── CMakeLists.txt / package.xml
│   │
│   ├── vision/                              # ament_python
│   │   ├── vision/
│   │   │   ├── vision_node.py               # 검출+좌표 변환을 한 노드에서 처리(영상 이중 전송 방지)
│   │   │   ├── detector.py                  # 색 마스크(공깃돌·병뚜껑·벨트 표식), 원·타원 중심, 신뢰도
│   │   │   ├── localizer.py                 # 왜곡 보정, 광선-평면 교차(대상물별 높이), 공분산
│   │   │   ├── robot_mask.py                # 로봇·그리퍼 링크 투영 마스크(가림 영역 제외)
│   │   │   └── verifier.py                  # 투입 판정(SUCCESS/FAILURE/UNKNOWN, 4.5절)
│   │   ├── config/vision.yaml
│   │   └── test/                            # test_detector.py, test_localizer.py, test_verifier.py, contract/
│   │
│   ├── tracking/                            # ament_python
│   │   ├── tracking/
│   │   │   ├── tracker_node.py              # TrackedObject·BeltMotion·TF(cap_<id>) 발행
│   │   │   ├── kalman_cv.py                 # 벨트 좌표계 [x, y, vx, vy] 등속 칼만 필터
│   │   │   ├── association.py               # 게이팅·ID 부여·재획득(통과마다 새 track)
│   │   │   └── speed_change_detector.py     # 다이얼 변경(혁신값 누적) 검출
│   │   ├── config/tracking.yaml
│   │   └── test/                            # 프레임 누락·역순·가림·재획득, contract/
│   │
│   ├── intercept_planner/                   # ament_python
│   │   ├── intercept_planner/
│   │   │   ├── intercept_planner_node.py
│   │   │   ├── time_budget.py               # T_available vs 작업 소요 시간(9절, 유일한 구현)
│   │   │   ├── intercept.py                 # 해제 시각·위치 계산
│   │   │   └── reachability_checker.py      # 작업영역·IK·후퇴 경로 여유
│   │   ├── config/planner.yaml
│   │   └── test/
│   │
│   ├── robot_control/                       # ament_python (지터 미달 시 adapter만 C++ 전환)
│   │   ├── robot_control/
│   │   │   ├── robot_controller_node.py     # skill 액션 서버, 유일한 로봇·그리퍼 지령 주체
│   │   │   ├── motion_adapter.py            # SI/quaternion ↔ mm/deg/ZYZ, freshness 감시(L0)
│   │   │   ├── tracking_controller.py       # v_cmd = v_ff + Kp·(p_target − p_tcp), 제한
│   │   │   ├── gripper_adapter.py           # RH-P12-RN(A) 명령(레지스터 → 상주 DRL → Modbus), pulse ↔ 간격, 전류 제한, 개폐 지연 측정값
│   │   │   ├── pick_sequence.py             # 매트 공깃돌 정적 파지(move_line 기반)
│   │   │   └── dynamic_place.py             # 대기→접근→동기→하강→해제→동기 유지→후퇴
│   │   ├── config/robot_control.yaml        # 그리퍼 주소·pulse 실측표·Goal Current·Δ_open 포함
│   │   └── test/                            # fakes/(transport·gripper)로 단위 변환·stale·cancel, contract/
│   │
│   ├── orchestrator/                        # ament_python
│   │   ├── orchestrator/
│   │   │   ├── orchestrator_node.py         # DropTask 액션 서버, skill 클라이언트, TaskStatus 발행
│   │   │   ├── state_machine.py             # 상태·전이 표(4.2절)를 데이터로 정의
│   │   │   ├── step_judges.py               # 단계별 성공 판정 함수(4.3절)
│   │   │   ├── abort_policy.py              # 중단 등급 판정·승격(5절)
│   │   │   ├── deadlines.py                 # 단계/사이클/동적 구간 deadline(6절), time_budget 사용
│   │   │   ├── safety_supervisor_node.py    # Heartbeat·로봇 상태 감시, L4
│   │   │   └── record_logger_node.py        # StepRecord/TrialRecord → JSONL
│   │   ├── config/orchestrator.yaml         # 제한시간·재시도·판정 임계값(전부 단위 포함)
│   │   └── test/                            # fakes/(skill port·fake clock)로 전이·판정·중단·deadline 단위 테스트
│   │
│   ├── calibration/                         # ament_python
│   │   ├── calibration/
│   │   │   ├── camera_intrinsics_check.py   # CameraInfo·왜곡 계수 확인
│   │   │   ├── plane_calibration.py         # 매트·벨트 평면, 벨트 진행 방향 벡터
│   │   │   ├── eye_to_hand_calibration.py   # 고정 카메라 ↔ robot_base
│   │   │   └── tcp_offset.py                # 파지한 공깃돌 중심 ↔ TCP 오프셋
│   │   ├── config/
│   │   │   ├── camera_extrinsics.yaml       # calibration_id, 날짜, 잔차, 카메라 프로파일
│   │   │   ├── mat_plane.yaml               # 평면 + 공깃돌 높이 오프셋
│   │   │   └── belt_plane.yaml              # 평면 + 진행 방향 + 병뚜껑 테두리 높이
│   │   └── test/
│   │
│   ├── drop_mocks/                          # ament_python
│   │   ├── drop_mocks/
│   │   │   ├── scenario_engine.py           # 단일 세계 모델(시나리오·seed·시계)
│   │   │   ├── mock_world_node.py           # vision·tracking 출력과 서비스, role:=vision|tracking|all
│   │   │   └── mock_robot_controller_node.py # skill 액션·Stop·Heartbeat
│   │   ├── scenarios/*.yaml                 # 7.3절
│   │   └── test/
│   │
│   ├── drop_bringup/                        # ament_cmake (launch/config 설치)
│   │   ├── launch/
│   │   │   ├── vision.launch.py
│   │   │   ├── robot.launch.py              # mode:=real|virtual, model:=e0509
│   │   │   ├── drop.launch.py               # use_mock_<node>:=true|false
│   │   │   ├── mock.launch.py               # 전체 mock + scenario:=<이름>
│   │   │   ├── simulation.launch.py         # use_sim_time:=true 일괄 적용
│   │   │   └── replay.launch.py             # rosbag 재생(로봇 지령 경로 미연결)
│   │   ├── config/system.yaml
│   │   └── test/                            # launch_testing 통합 테스트(7.5절 I0~I2)
│   │
│   └── drop_sim/                            # ament_cmake (자원 설치만)
│       ├── usd/drop_scene.usd               # 대용량 → Git LFS
│       ├── scripts/conveyor_simulation.py   # Isaac Sim python으로 실행(colcon 실행 대상 아님)
│       └── config/simulation.yaml           # physics rate, contact/rest offset, 벨트 표면 속도, 반발계수·마찰
│
├── specs/                                   # 8절
├── records/                                 # 10절
├── tools/
│   ├── eval/                                # record_trial.sh, analyze_tracking.py,
│   │                                        # analyze_drop.py, trial_labeler.py
│   └── spec/trace.py                        # REQ ↔ 테스트 추적 행렬 생성
└── README.md
```

구성 규칙: 테스트는 `colcon test`에 자동 등록되도록 각 패키지 `test/`에 둔다. ament_python 패키지마다 `resource/<패키지명>`, `setup.py`, `setup.cfg`, `package.xml`, entry point, config·launch 설치 규칙을 둔다(트리에서는 생략). 접두 없는 패키지 이름(`vision`, `tracking`, `calibration`, `orchestrator`, `robot_control`)은 underlay(`/opt/ros/jazzy`)·설치 패키지와 겹치지 않는지 M0 첫 빌드에서 `ros2 pkg list`로 확인한다.

### 2.5 TF 트리

```
world
 └── robot_base (base_link, 드라이버/URDF 명칭에 맞춤)
      ├── conveyor_frame        (static: 벨트 장치 고정, x축 = 벨트 진행 방향, z = 벨트 표면 법선)
      ├── mat_frame             (static)
      ├── camera_link → camera_color_optical_frame   (static, eye-to-hand 보정 결과 / 광학 TF는 RealSense 드라이버 소유)
      ├── ... robot links → tcp (robot_state_publisher 소유)
      └── conveyor_frame → cap_<track_id>   (dynamic, tracker 발행, stamp = t_state, 시각화용)
```

- `conveyor_frame`은 벨트 표면을 따라 움직이지 않는다(장치 고정).
- `cap_<id>`의 orientation은 identity(원형 병뚜껑의 yaw는 관측하지 않음)이며 파지 자세와 구분한다.
- 미래 위치는 TF로 발행하지 않고 RViz `Marker`(`drop/predicted_marker`)로만 표시한다.
- 같은 TF를 두 노드가 발행하지 않는다.

## 3. 인터페이스

필드 확정은 M0(`Progress.md` 1절)의 `SPEC-010`에서 한다.

### 3.1 데이터 메시지

```text
# ObjectDetection.msg
uint8 REGION_UNKNOWN=0
uint8 REGION_MAT=1
uint8 REGION_BELT=2
uint8 CLASS_UNKNOWN=0
uint8 CLASS_STONE=1               # 공깃돌
uint8 CLASS_CAP=2                 # 병뚜껑
uint8 CLASS_BELT_MARKER=3         # 벨트 표식(선택 실험)
uint32 detection_id              # 프레임 내 관측 번호(추적 ID 아님)
uint8 region
uint8 object_class
float64 u                         # 왜곡 보정 전 픽셀 중심
float64 v
float64 diameter_px
float32 confidence
float64[4] center_cov_px          # 2x2 row-major
geometry_msgs/Point position      # 보정 평면 교차 결과(평면 높이는 class별), ObjectDetectionArray.header.frame_id 기준 [m]
float64[9] position_cov           # 3x3 [m^2]
bool position_valid
bool occluded_partial             # 로봇 마스크와 겹침
bool stone_inside                 # CLASS_CAP 전용: 뚜껑 원 안에 공깃돌이 보임
bool stone_inside_valid           # CLASS_CAP 전용: false = 가림 등으로 판정 불가(stone_inside 무시)

# ObjectDetectionArray.msg
std_msgs/Header header            # stamp = 영상 노출 시각, frame_id = robot_base
uint64 image_seq
string calibration_id
ObjectDetection[] detections

# TrackedObject.msg
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
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[16] state_cov             # [x, y, vx, vy] 4x4
builtin_interfaces/Time valid_until

# TrackedObjectArray.msg
std_msgs/Header header
TrackedObject[] tracks

# BeltMotion.msg
uint8 SOURCE_CAP=0                # 벨트에 붙인 병뚜껑의 이동에서 추정
uint8 SOURCE_BELT_MARKER=1        # 벨트 표식에서 추정(선택 실험)
std_msgs/Header header            # frame_id = conveyor_frame
geometry_msgs/Vector3 direction   # 단위 벡터
float64 speed                     # [m/s]
float64 speed_std
float64 window_sec
uint8 source
bool steady                       # 속도 변화 미검출
builtin_interfaces/Time last_change_stamp

# InterceptPlan.msg
uint8 OK=0
uint8 TIME_INSUFFICIENT=1
uint8 UNREACHABLE=2
uint8 UNCERTAIN=3
uint8 EXPIRED=4
std_msgs/Header header            # stamp = 계획 생성 시각, frame_id = robot_base
uint32 plan_id
uint32 revision
uint32 target_track_id            # 병뚜껑 track
uint32 target_epoch
builtin_interfaces/Time source_state_stamp
builtin_interfaces/Time release_time
builtin_interfaces/Time valid_until
geometry_msgs/Pose target_tcp_pose        # release_time의 해제 직전 TCP 목표(파지 오프셋 반영: 공깃돌 중심이 뚜껑 중심 위)
geometry_msgs/Vector3 target_velocity     # 동기화할 선속도 [m/s]
float64 position_std              # release_time 예측 표준편차 [m]
float64 time_margin               # 남은 작업 가능 시간 − 필요 시간 [s]
uint8 status
string reason

# TaskStatus.msg  (orchestrator 발행)
std_msgs/Header header
string cycle_id
string state                      # 상태 머신 상태명
uint8 abort_level                 # 0 = 없음, 1~4 = 5절
uint16 error_code
string command_owner
bool robot_ready
bool protective_stop
bool gripper_holding
float64 time_remaining

# DropTask.action  (orchestrator 서버, 외부 진입점)
uint8 MODE_STATIC=0               # 정지 벨트
uint8 MODE_PREDICTED=1            # 예측 해제점 + 짧은 속도 일치 구간
uint8 MODE_SYNCHRONIZED=2         # 지속 갱신 목표 + 속도 동기 추종
uint8 mode
float64 max_belt_speed            # 이 속도를 넘으면 시도하지 않음 [m/s]
float64 timeout                   # 사이클 제한시간 [s]
uint8 max_passes                  # 시간 예산 부족으로 보낼 수 있는 병뚜껑 통과 횟수
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 UNKNOWN=2                   # 판정 불가(성공률 분자에서 제외, 분모에 포함)
uint8 CANCELED=3
uint8 SAFE_ABORT=4
uint8 PROTECTIVE_STOP=5
uint8 NOT_ATTEMPTED=6            # 선행조건·시간 예산 미충족으로 시도하지 않음
uint8 result
string reason_code
float64 release_error             # 해제 순간 공깃돌 중심(TCP+파지 오프셋)과 뚜껑 중심 추정 사이 XY [m]
bool release_error_valid
float64 stone_offset              # 안착 후 뚜껑 중심 대비 공깃돌 중심, 직접 관측 [m]
bool stone_offset_valid
float64 hold_time                 # 뚜껑 안 유지 확인 시간 [s]
float64 cycle_time
uint8 passes_skipped
---
string state
uint32 plan_id
float64 time_remaining
float64 tracking_std
```

### 3.2 지휘용 인터페이스

```text
# 공통 결과 코드 (모든 skill 결과에 포함)
uint8 OK=0
uint8 FAILED=1          # 시도했으나 판정 실패
uint8 TIMEOUT=2         # deadline 경과
uint8 PRECONDITION=3    # 선행조건 불충족, 시도하지 않음
uint8 CANCELED=4
uint8 FAULT=5           # 하드웨어·통신 이상

# Heartbeat.msg  (모든 기능 노드, 10 Hz)
std_msgs/Header header
string node_name
uint8 lifecycle_state
uint32 seq
uint16 error_code
string detail

# StepRecord.msg  (orchestrator, 단계 종료마다)
std_msgs/Header header
string cycle_id
uint32 step_seq
string step_name
builtin_interfaces/Time t_start
builtin_interfaces/Time t_end
uint8 result            # 공통 결과 코드
string reason_code
uint8 attempt
uint8 abort_level       # 0 = 없음, 1~4 = 5절
string[] metric_keys    # 판정에 쓴 값 (예: "e_xy_m", "v_rel_mps")
float64[] metric_values

# 공통 요청 필드 (오케스트레이터가 보내는 모든 srv 요청·action goal 앞부분)
string cycle_id
uint32 step_id
builtin_interfaces/Time deadline   # ROS 시각, 6.2절

# DetectObjects.srv
(공통 요청 필드)
uint8 region            # REGION_MAT / REGION_BELT
uint8 object_class      # CLASS_STONE / CLASS_CAP
uint8 min_count
float64 stable_window   # [s] 이 시간 동안 안정적으로 보인 것만
---
uint8 result
string reason_code
ObjectDetection[] detections
float64[] position_std  # 창 안 위치 표준편차 [m]

# VerifyDrop.srv
(공통 요청 필드)
uint32 cap_track_id
float64 hold_time       # [s]
---
uint8 result
uint8 verdict           # SUCCESS / FAILURE / UNKNOWN
string reason_code
bool stone_inside
float64 stone_offset    # [m]
bool stone_offset_valid

# ComputeIntercept.srv
(공통 요청 필드)
uint32 target_track_id
uint32 target_epoch
uint8 mode
builtin_interfaces/Time earliest_release
---
uint8 result
string reason_code
InterceptPlan plan

# ResetTracking.srv
(공통 요청 필드)
uint8 region            # 초기화할 영역(REGION_UNKNOWN = 전체)
---
uint8 result
string reason_code
uint32 new_epoch

# SelectTarget.srv
(공통 요청 필드)
uint8 object_class      # CLASS_CAP
uint32 min_observation_count
---
uint8 result
string reason_code
uint32 track_id         # 도달 구간에 아직 들어오지 않은 가장 가까운 병뚜껑
uint32 epoch
builtin_interfaces/Time t_enter_window   # 도달 구간 진입 예측 시각

# Stop.srv
(공통 요청 필드)
uint8 requester         # ORCHESTRATOR / SUPERVISOR
string reason_code
---
uint8 result
bool stopped            # 정지 확인 후 응답

# 오케스트레이터가 보내는 action goal은 모두 공통 요청 필드를 포함한다(외부 진입점 DropTask 제외).
# PickStone.action       goal: target position, approach height, grip current limit
#                        result: 공통 코드, grip_width, grip_current, holding / feedback: phase
# MoveToPose.action      goal: target pose 또는 named pose(standby 등)
#                        result: 공통 코드, pose_error
# PlaceStone.action      goal: target pose(정지 목표), release height, open width
# DynamicPlace.action    goal: plan_id, target_epoch, mode, release gate 임계값(e_xy_max, v_rel_max, tau_max,
#                              position_std_max, rim_clearance), open width
#                        feedback: phase, e_xy, v_rel, plan_revision / result: 공통 코드, released, gate 값
# SafeRetreat.action     goal: keep_sync(bool), place_back(bool), place_back_pose
# 필드 정의 원본은 src/drop_interfaces/(SPEC-010 스냅샷 대상)다.
```

### 3.3 토픽·서비스·주기

이름은 상대명이며 launch namespace가 붙는다.

| 이름 | 형식 | 제공/발행 | 주기 | QoS(제안) | 비고 |
|---|---|---|---|---|---|
| `camera/color/image_raw`, `camera_info` | sensor_msgs | RealSense | 30 Hz(1280×800) | sensor data(best effort, depth 1) | 자동 노출·화이트밸런스 고정. FastDDS SHM 세그먼트 확대 프로파일 필수(SPEC-600 4.8절) |
| `objects/detections` | `ObjectDetectionArray` | vision | 영상 주기 | reliable, volatile, depth 5 | 위치·공분산 포함 |
| `objects/tracked` | `TrackedObjectArray` | tracker | 영상 주기 | reliable, volatile, depth 5 | |
| `belt/measured_motion` | `BeltMotion` | tracker | 영상 주기 | reliable, volatile, depth 5 | |
| `drop/intercept_plan` | `InterceptPlan` | planner | 상태 갱신마다 | reliable, volatile, depth 5 | revision 증가 |
| `drop/predicted_marker` | `visualization_msgs/Marker` | planner | 10 Hz | best effort, depth 1 | 시각화 전용 |
| `system/heartbeat` | `Heartbeat` | 모든 기능 노드 | 10 Hz | reliable, volatile, depth 1 | supervisor 감시 |
| `task/status` | `TaskStatus` | orchestrator | 10 Hz + 상태 변경 시 | reliable, transient local, depth 1 | |
| `task/step_record` | `StepRecord` | orchestrator | 단계 종료마다 | reliable, volatile, depth 100 | record_logger 구독 |
| `drop_task` (action) | `DropTask` | orchestrator | — | 기본(service/action) | 외부 진입점 |
| `vision/detect_objects`, `vision/verify_drop` (srv) | `DetectObjects`, `VerifyDrop` | vision | — | 기본(service/action) | orchestrator 전용 |
| `tracking/reset`, `tracking/select_target` (srv) | `ResetTracking`, `SelectTarget` | tracker | — | 기본(service/action) | orchestrator 전용 |
| `planner/compute_intercept` (srv) | `ComputeIntercept` | planner | — | 기본(service/action) | orchestrator 전용 |
| `robot/pick_stone`, `robot/move_to_pose`, `robot/place_stone`, `robot/dynamic_place`, `robot/safe_retreat` (action) | 3.2절 | robot_control | — | 기본(service/action) | orchestrator 전용 |
| `robot/stop` (srv) | `Stop` | robot_control | — | 기본(service/action) | orchestrator·supervisor |
| `<dsr_ns>/servol_stream` 또는 `speedl_stream` (RT 계열 후보: `servol_rt_stream`, `speedl_rt_stream`) | `dsr_msgs2` | robot_control → 드라이버 | G0에서 측정 후 확정(50~100 Hz 후보) | 드라이버 기본값(G0 확인) | 메시지에 Header 없음 → 어댑터가 freshness 검사 |
| `<dsr_ns>/motion/move_line` (srv) | `dsr_msgs2/srv/MoveLine` | robot_control → 드라이버 | — | 기본(service/action) | 정적 pick/place, 대기 자세·접근 이동 |
| `<dsr_ns>/dsr_controller2/drl/drl_start` (srv) | `dsr_msgs2/srv/DrlStart` | robot_control → 드라이버 | bringup 시 1회(STANDBY 필요) | 기본(service/action) | 그리퍼 상주 루프 DRL 기동 |
| `<dsr_ns>/dsr_controller2/plc/set_output_register_int` (srv) | `dsr_msgs2/srv/SetOutputRegisterInt` | robot_control → 제어기 레지스터 → 루프 DRL → 플랜지 RS-485 | 그리퍼 명령 시 | 기본(service/action) | `gripper_adapter.py` 안에서만 호출. 상태 읽기 경로는 G0에서 확정 |

QoS·revision 처리·lifecycle 비활성 시 응답은 `SPEC-010`에서 확정하고 계약 테스트(7.4절)로 검사한다. 발행자와 구독자의 QoS가 맞지 않으면 구독이 오류 없이 실패하므로 계약 테스트에 QoS 일치 확인을 넣는다.

doosan-robot2 인터페이스는 공개 Jazzy 브랜치 커밋 `dade65d` 기준이다. 현장 설치 버전과 E0509 제어기가 실제로 받아들이는지, 그리퍼 호출 경로가 어떤 인터페이스를 쓰는지는 G0에서 확인한다.

## 4. 작업 흐름과 단계별 성공 판정

### 4.1 원칙

- 각 단계는 **선행조건 → 실행 → 성공 판정** 순서로 진행한다. 판정 결과가 `OK`일 때만 다음 단계로 넘어간다. "skill이 오류 없이 끝났다"는 것만으로는 성공이 아니다. 판정은 반드시 **독립 관측**(영상, 그리퍼 핑거 간격·전류, TCP 피드백)으로 한다.
- 판정 함수는 `step_judges.py`에 있는 순수 함수다. 입력은 측정값, 출력은 `(result, reason_code, metrics)`다. 판정에 쓴 값은 그대로 `StepRecord.metric_*`에 기록한다.
- 선행조건이 성립하지 않으면 실행하지 않고 `PRECONDITION`으로 끝낸다. 이 경우 로봇은 움직이지 않는다.
- 재시도할 때는 그 단계의 **선행조건부터 다시** 확인한다.

### 4.2 상태 흐름

```
IDLE
 → PRECHECK            보정 유효, 노드 ACTIVE, 로봇 준비, 그리퍼 응답·비어 있음, 벨트 속도 ≤ max_belt_speed
 → DETECT_STONE        매트 위 공깃돌 검출(매트는 정지)
 → PICK_STONE          병뚜껑이 오기 전에 미리 파지 → 시간 예산에서 파지 시간 제외
 → MOVE_STANDBY        도달 구간 상류 끝 위 안전 높이의 대기 자세로 이동
 → WAIT_CAP            다음 병뚜껑 통과 선택, 추적 안정, 뚜껑 비어 있음, 시간 예산 판정
                       (부족하면 이번 통과를 보내고 다음 통과를 기다림, max_passes까지)
 → PLAN_INTERCEPT
 → APPROACH            예측 해제점 위 안전 높이(예: 테두리 +30 mm)로 이동
 → SYNCHRONIZE         XY를 병뚜껑 속도에 맞춤
 → DESCEND             동기 유지하며 해제 높이(핑거 끝이 테두리 + h_clear)까지 하강
 → RELEASE             해제 게이트(4.4절) 통과 시 Δ_open만 개방, 개방 지연(p95)+여유 동안 동기 유지
 → RETREAT             동기 유지한 채 수직 상승 후 이탈
 → VERIFY              SUCCESS / FAILURE / UNKNOWN
 → DONE                (작업자가 병뚜껑 안 공깃돌을 회수한 뒤 다음 시도)

중단 상태(5절): ABORT_CYCLE(L2), SAFE_ABORT(L3), PROTECTIVE_STOP(L4) → WAIT_OPERATOR
```

자동 재시도는 매트 재검출, 재파지(4.3절 횟수), 병뚜껑 통과 보내기(`max_passes`)만 허용한다. 통과 보내기는 로봇이 대기 자세에 있을 때만 하므로 동적 구간을 되돌리지 않는다.

### 4.3 상태별 판정표 (초기 제안값)

| 단계 | 선행조건 | 성공 판정(모두 만족) | 제한시간 | 실패 시 | 재시도 |
|---|---|---|---|---|---|
| PRECHECK | — | 모든 기능 노드 `ACTIVE`이고 heartbeat가 300 ms 이내 / calibration_id 일치·유효 / `robot_ready`이고 보호정지 아님 / 그리퍼 상주 루프 생존 카운터 증가 중, 응답 ≤ 0.5 s(G0 측정 후 확정), Hardware Error 없음, 개방 상태 / 벨트 `steady`, 속도 ≤ `max_belt_speed` | 5 s | 시도하지 않고 IDLE 복귀(로봇 미동작) | 0 |
| DETECT_STONE | PRECHECK OK | 매트 ROI 공깃돌 ≥ 1, confidence ≥ 0.9, `position_valid`, 0.5 s 창 위치 표준편차 ≤ 0.5 mm, 주변 물체와 핑거 간섭 거리 이상 떨어짐 | 3 s | L1 재검출 | 2 |
| PICK_STONE | 검출 시각이 1 s 이내 | 핑거 간격이 `[d_stone_min − 1, d_stone_max + 1] mm` 안(pulse 실측표 변환) / Present Current ≥ 파지 전류 하한 / 매트 ROI에서 해당 공깃돌 사라짐 / 로봇·그리퍼 오류 없음 | 15 s | 매트에 남아 있으면 L1, 그 외 L2 | 2 |
| MOVE_STANDBY | holding | 대기 자세 오차 ≤ 2 mm / holding 유지(핑거 간격·전류) | 10 s | L3 | 0 |
| WAIT_CAP | 대기 자세, holding | `SelectTarget` 병뚜껑 track `OBSERVED`, `observation_count ≥ 30`, `speed_std ≤ 1 mm/s`, `steady` / 뚜껑 안에 공깃돌 없음(`stone_inside_valid == true`이고 `stone_inside == false`, 판정 불가는 통과 보내기) / `T_available − T_required ≥ margin` | `T_lap × (max_passes + 1)`(`T_lap` = 한 바퀴 주기, G0 측정) | 판정 실패한 통과는 보내고 다음 통과 대기. `max_passes`를 넘으면 L3(결과 `NOT_ATTEMPTED`) | `max_passes` |
| PLAN_INTERCEPT | holding, track `OBSERVED`, epoch 일치 | `status == OK`, `time_margin ≥ 1.0 s`, `position_std ≤ 0.5 mm`, `valid_until > now + T_approach` | 0.5 s | 동적 deadline 안에서 재계획, 넘으면 이번 통과를 보내고 WAIT_CAP(남은 통과가 없으면 L3) | deadline까지 |
| APPROACH | plan 유효 | 해제점 위 30 mm 도달, 오차 ≤ 2 mm / plan 유효 상태 유지 | 1.5 × T_transit(p99) | L3 | 0 |
| SYNCHRONIZE | 접근 완료 | `e_xy ≤ e_xy_max`, `v_rel ≤ v_rel_max`가 연속 0.3 s 유지 | 2 s | L3 | 0 |
| DESCEND | 동기 게이트 유지 | 해제 높이 ±0.5 mm 도달 / 하강 중 동기 게이트 계속 만족 / 핑거 끝이 테두리 + h_clear 위 | 1.5 s | L3(동기 유지하며 상승) | 0 |
| RELEASE | 해제 게이트(4.4절) 모두 만족 | 그리퍼 개방 확인(핑거 간격 ≥ `d_stone_max + Δ_open − 1 mm`) / 개방 지연(p95) + 여유 동안 동기 유지 | 1.5 × T_release(p99) | 해제 후에는 되돌릴 수 없으므로 RETREAT → VERIFY(FAILURE/UNKNOWN) | 0 |
| RETREAT | 해제 완료 또는 L3 | TCP z ≥ 안전 높이 / 작업영역 밖 / 그리퍼에 공깃돌 없음 | 3 s | L4(후퇴 실패는 기계 이상으로 본다) | 0 |
| VERIFY | 후퇴 완료 | `VerifyDrop` 응답(SUCCESS/FAILURE/UNKNOWN), 1 s 유지 확인 | 3 s | UNKNOWN으로 기록 | 0 |

- `MODE_STATIC`은 벨트를 멈추고 병뚜껑을 도달 구간 안에 둔 상태에서 한다. WAIT_CAP은 정지 뚜껑 검출(`DetectObjects`, `CLASS_CAP`)로 바꾸고 SYNCHRONIZE는 건너뛴다. 대신 PLACE 하강 단계에서 정지 목표 기준 오차 ≤ 1 mm를 판정한다. `MODE_PREDICTED`는 SYNCHRONIZE를 "계획된 속도 일치 구간 진입" 판정으로 바꾼다.
- 마지막 VERIFY의 판정은 보조 판정이다. 성공률은 관찰자 판독으로 계산한다(4.5절).

### 4.4 해제 게이트

다음을 모두 만족해야 그리퍼를 연다. 하나라도 어긋나면 해제하지 않고 L3로 간다.

- 공깃돌 중심(TCP + 파지 오프셋)과 뚜껑 중심 예측 사이 XY 오차 ≤ `e_xy_max`. 초기값은 `0.5 × (D_in − d_stone_max) / 2`로 계산하고, G2 낙하 시험에서 측정한 허용 오차로 바꾼다.
- 상대 속도 ≤ `v_rel_max`(초기 5 mm/s, G2 낙하 시험과 G4 결과로 조정).
- 추적 상태 `OBSERVED`, 또는 `COASTING`이면서 `t_now − t_last_obs ≤ τ_max`. 그리퍼가 위에서 뚜껑을 가리므로 COASTING 구간이 길 수 있다. 병뚜껑은 벨트에 붙어 미끄러지지 않으므로 가림 중 오차는 주로 벨트 속도 오차에서 생긴다. τ_max는 G3·G4 측정으로 정한다(초기 0.3 s).
- 위치 예측 표준편차 ≤ 기준, `valid_until` 미경과, `epoch` 일치.
- 핑거 끝 높이 ≥ 테두리 높이 + `h_clear`(초기 2 mm).

### 4.5 투입 판정 (보조 카메라 없음)

측면 카메라가 없으므로 고정 D455 상단 영상과 관찰자 판독을 조합한다. 병뚜껑은 위가 열려 있어 상단 영상으로 안에 든 공깃돌을 직접 볼 수 있다.

1. **기준 정답 = 관찰자 판독:** 실험마다 관찰자가 벨트 옆에서 공깃돌이 뚜껑 안에 들어가 1 s 동안 함께 이동했는지 직접 보고 `tools/eval/trial_labeler.py`로 기록한다(SUCCESS/FAILURE/판독 불가). 성공률은 이 기록으로 계산한다.
2. **자동 판정(`verifier.py`, 보조):** 후퇴 완료 후 아래를 모두 만족하면 SUCCESS 후보, 하나라도 어긋나면 FAILURE, 판단할 수 없으면 UNKNOWN.
   - 선택한 병뚜껑 track의 원 안에 공깃돌이 보이고(`stone_inside`), 1 s 동안 공깃돌 중심이 뚜껑 원 안에 계속 있다.
   - 벨트 ROI에 뚜껑 밖 공깃돌이 없다(있으면 튕겨 나감 → FAILURE).
   - 그리퍼 개방 확인 + 그리퍼에 공깃돌이 붙어 있지 않다(핑거 간격, 그리퍼 주변 ROI에 공깃돌 없음).
   - 매트 위 공깃돌 수가 파지 직후와 같다(이동 중 떨어뜨림 없음).
3. **오차 지표:**
   - 해제 오차(`release_error`): 해제 순간 공깃돌 중심과 뚜껑 중심 추정 사이 XY. 이때 뚜껑은 그리퍼에 가려 있으므로 track 예측 기반 추정값이다. 정적(G2)에서는 정지 뚜껑 측정값 기준으로 계산한다.
   - 안착 오프셋(`stone_offset`): 안착 후 뚜껑 중심 대비 공깃돌 중심, 직접 관측한다. 공깃돌이 뚜껑 안에서 굴러 테두리 쪽에 멈출 수 있으므로 정확도 지표로 쓰지 않고 기록만 한다.
4. 자동 판정과 관찰자 판독의 혼동행렬을 G2부터 기록한다.

## 5. 중단 알고리즘

### 5.1 중단 등급

| 등급 | 이름 | 발생 조건(예) | 동작 | 판단 주체 |
|---|---|---|---|---|
| L0 | LOCAL_STOP | 지령 freshness 초과(2 주기), 지령 범위 초과 | 즉시 속도 0, 이후 지령 거부. supervisor에 `FAULT` 보고 | `robot_controller`(다른 노드와 무관하게 동작) |
| L1 | RETRY | 재시도 가능한 단계 실패, 재시도 횟수와 시간이 남음 | 해당 단계 선행조건부터 재실행 | orchestrator |
| L2 | ABORT_CYCLE | 공깃돌을 들고 있지 않고 동적 구간이 아님 / 시간 예산 부족 | 로봇을 안전 자세로 보냄 → WAIT_OPERATOR | orchestrator |
| L3 | SAFE_ABORT | 공깃돌을 들고 있음 또는 동적 구간(APPROACH~DESCEND)에서 실패 / 통과 보내기 횟수 소진 | 동적 구간이면 동기 유지하며 상승(`SafeRetreat keep_sync`) → 안전 자세 → 공깃돌을 매트에 내려놓음 → WAIT_OPERATOR | orchestrator |
| L4 | PROTECTIVE_STOP | 로봇 보호정지·비상정지 / 로봇 통신 단절 / `robot_controller` heartbeat 300 ms 끊김 / 중단 경로 자체의 제한시간 초과 / RETREAT 실패 | `Stop` 요청, 자동 리셋·원점 복귀 금지 → WAIT_OPERATOR | safety_supervisor(orchestrator도 요청 가능) |

### 5.2 판정 순서

```python
def decide(event, ctx) -> AbortLevel:
    # 1) 하드 이상은 단계와 무관하게 최우선
    if event in HARD_FAULTS:          # 보호정지, E-stop, 로봇 통신 단절, controller heartbeat 손실
        return L4
    # 2) 해제 이후에는 되돌릴 수 없음 → 중단하지 않고 후퇴·판정으로 진행
    if ctx.released:
        return CONTINUE_TO_RETREAT_VERIFY
    # 3) 공깃돌을 들고 있거나 동적 구간이면 제어된 후퇴
    if ctx.holding or ctx.state in DYNAMIC_STATES:
        return L3
    # 4) 재시도 가능 여부: 횟수와 시간 모두 확인
    if ctx.retry_left(ctx.state) > 0 and ctx.now + ctx.est_duration(ctx.state) < ctx.deadline:
        return L1
    return L2
```

- **높은 등급이 낮은 등급을 덮어쓴다.** L3 진행 중에 L4가 발생하면 즉시 L4로 바꾼다. 반대로 낮은 등급이 높은 등급을 덮어쓰지는 않는다.
- **중단 경로에도 제한시간이 있다.** L2는 10 s, L3는 15 s(제안) 안에 끝내야 한다. 넘으면 L4로 올린다. cancel을 요청한 뒤 0.5 s 안에 응답이 없어도 한 단계 올린다.
- **WAIT_OPERATOR는 제한시간 없이 기다린다.** 다만 진입한 시각과 사유를 기록하고, 작업자가 명시적으로 재개(`resume` + 사유 입력)한 다음 PRECHECK부터 다시 시작한다.
- **취소(cancel)는 L0~L4 중 상황에 맞는 등급으로 바꿔 처리한다.** 사용자 cancel이 들어와도 공깃돌을 들고 있으면 L3로 처리한다.
- WAIT_CAP에서 통과 보내기 횟수를 다 써서 L3로 가는 경우는 동적 구간에 들어가지 않았으므로 상승 없이 대기 자세에서 매트로 돌아가 공깃돌을 내려놓고, 결과는 `NOT_ATTEMPTED`로 기록한다.

### 5.3 감시 체계

| 대상 | 방법 | 임계값(제안) | 위반 시 |
|---|---|---|---|
| 기능 노드 생존 | `Heartbeat` 10 Hz | 3회 연속 누락(300 ms) | vision·tracker: 동적 구간이면 L3, 그 외 L2 / robot_controller: L4 |
| 로봇 지령 신선도 | adapter 내부 타이머 | 지령 간격 > 2 × 주기 | L0 |
| 그리퍼 상태 | `gripper_adapter` 상태 읽기(Hardware Error Status, 응답 시간, 상주 루프 생존 카운터) | 오류 비트 발생, 응답 > 0.5 s, 생존 카운터 정지 > 0.1 s(G0 측정 후 확정) | 공깃돌을 들고 있거나 동적 구간이면 L3, 그 외 L2 |
| 관측 신선도 | `t_now − t_last_obs` | 동적 구간 ≤ τ_max(초기 0.3 s) | 해제 전이면 L3 |
| 계획 유효성 | `valid_until`, epoch | 만료 또는 epoch 불일치 | 재계획, 동적 deadline을 넘으면 L3 |
| `/clock` 역행 | 이전 stamp와 비교 | 역행 1회 | track·계획 무효화 → L2/L3 |
| 벨트 속도 급변 | `BeltMotion.steady == false` | — | 해제 전이면 L3 |

중단 판정·deadline 계산은 fake clock 단위 테스트로 검증하고, ROS 수준(I0)에서는 호출 제한시간·cancel 0.5 s·Heartbeat 300 ms·`Stop` 경로만 검증한다(7.3절).

## 6. 제한시간 체계

### 6.1 종류

| 종류 | 적용 대상 | 기본값(제안) | 정하는 방법 |
|---|---|---|---|
| 호출 제한시간 | service 응답, action goal 수락·cancel 응답 | 1 s / 0.5 s | 고정 |
| 단계 제한시간 | 4.3절 각 단계 | 표의 값 | 측정 후 `ceil(1.5 × p99)`, 최소 `p95 + 0.5 s` |
| 동적 deadline | WAIT_CAP ~ RELEASE | `t_exit − T_remaining_required` | 시간 예산 식(9절)으로 매번 계산 |
| 데이터 신선도 | 검출·track·plan | 검출 1 s(정적), τ_max(동적) | G3·G4 측정 |
| 사이클 제한시간 | DropTask 한 번 전체 | `goal.timeout`(기본 90 s, `T_lap × (max_passes + 1)`보다 길게) | G2 cycle time p99 × 1.5 |
| 중단 경로 제한시간 | L2 / L3 | 10 s / 15 s | G0·G2 측정 |

- 단계마다 실제로 적용하는 deadline은 `min(단계 제한시간, 동적 deadline, 사이클 deadline)`이다(`deadlines.py`). skill goal의 `deadline` 필드에 이 값을 넣는다.
- 제한시간 값을 바꿀 때는 근거가 된 측정값(N, p95, p99)과 함께 `records/param_changes.csv`에 기록한다. 측정 없이 늘리지 않는다.

### 6.2 시계 정책

| 시계 | 사용처 | 이유 |
|---|---|---|
| ROS 시각(`use_sim_time` 시 `/clock`) | 메시지 stamp, `valid_until`, `t_last_obs`, 시간 예산, 단계·동적·사이클 deadline | 데이터의 시각 의미를 시뮬레이션·replay 속도에 맞춘다 |
| steady(monotonic) 시계 | L0 지령 freshness, Heartbeat 감시, 그리퍼 응답 감시, 호출 제한시간, cancel 응답 대기, 중단 경로 제한시간 | `/clock`이 멈추거나 역행해도 감시 타이머는 계속 동작해야 한다 |

- 사이클 제한시간에는 steady 시계 상한(`goal.timeout × 2`)을 함께 둔다. `/clock`이 멈춰도 사이클이 끝없이 이어지지 않는다.
- 타이머마다 어느 시계를 쓰는지 `SPEC-110`에 적고, 두 시계 모두 fake clock으로 단위 테스트한다.

## 7. 시험 대역과 병렬 개발

### 7.1 무엇을 대역으로 바꾸는가

명령은 오케스트레이터로 모이지만(star), 데이터는 `detections → tracked → intercept_plan → robot_controller`로 노드 사이를 직접 흐른다(2.2절). 그래서 대역은 시험 대상마다 필요한 만큼만 둔다.

| 시험 대상 | 대역 | 이유 |
|---|---|---|
| 기능 노드 단독 | 없음. 테스트 클라이언트가 해당 노드의 service·action을 직접 호출하고, 입력 topic은 fixture publisher나 rosbag으로 넣는다 | 기능 노드는 다른 노드를 호출하지 않으므로 mock 오케스트레이터나 상대 노드 mock이 필요 없다 |
| 오케스트레이터 판단 로직 | 순수 Python fake(가짜 skill port + fake clock) | 상태 전이·단계 판정·중단 등급·deadline은 ROS 없이 결정적으로 검증한다 |
| 오케스트레이터 ROS 연동 | ROS mock 노드 2종(7.2절) | action 수락·cancel·무응답, 호출 제한시간, lifecycle, Heartbeat는 ROS를 거쳐야 드러난다 |
| 노드 사이 데이터 스트림 | 합성 fixture publisher, rosbag replay, 먼저 완성된 실제 노드 | 실제 planner가 낸 `plan_id`·`revision`·`epoch`을 controller가 따르는지 확인한다 |
| robot_control 하위 계층 | adapter의 fake transport·fake gripper(순수 Python, 레지스터·상주 루프 경유 호출의 지연·무응답·오류 모델 포함), Doosan `mode:=virtual` | 단위 변환·freshness·stale·cancel·그리퍼 지연은 adapter 내부 로직이다. virtual 모드는 G0에서 E0509·사용할 stream 지원을 확인한 뒤 쓴다. 지원하지 않으면 형식 확인용 최소 ROS stub만 둔다. virtual 모드는 실제 그리퍼 통신을 대신하지 않는다 |
| 장애·제한시간 | 순수 Python 이벤트 열(대부분) + ROS mock(일부) | 보호정지·통신 단절·무응답은 실기에서 안전하게 반복할 수 없다 |

Isaac Sim은 대역으로 쓰지 않는다. 무겁고 결정적이지 않아 CI 장애 주입에 맞지 않으며, 별도 통합 단계 S로 둔다.

### 7.2 `drop_mocks`

| 구성 | 동작 | 장애 주입 |
|---|---|---|
| `scenario_engine.py` | 시나리오 YAML 하나로 매트 위 공깃돌·벨트에 붙은 병뚜껑·벨트 속도·한 바퀴 주기·가림 구간·이벤트 시각을 정의하는 단일 세계 모델. seed 고정 | 이벤트 시각표 |
| `mock_world_node` | `scenario_engine`에서 `ObjectDetectionArray`, `TrackedObjectArray`, `BeltMotion`을 일관되게 만들고, `DetectObjects`·`VerifyDrop`·`ResetTracking`·`SelectTarget`에 응답. Heartbeat는 역할별로 발행 | 판정 결과 지정, 상태 전환(COASTING/LOST), epoch 증가, `steady=false`, 뚜껑 안 공깃돌 남음, 역할별 heartbeat 중단 |
| `mock_robot_controller_node` | skill 액션 5종 + `Stop`의 수락·feedback·결과·cancel, Heartbeat. M0에서는 소요시간·결과를 시나리오로 지정한다. TCP 1차 지연 모델은 G5(I3 추종 시험)에서 추가한다 | 소요 시간 배율, 실패, 무응답(hang), 보호정지, 파지 실패, 그리퍼 응답 지연 |

- `mock_world_node`는 `role:=vision|tracking|all` 인자로 역할별 프로세스로 띄울 수 있다. 역할별 프로세스도 같은 시나리오 파일·seed·시계를 쓰므로 detections와 tracked가 같은 세계를 보인다. 역할을 나눠 띄우면 vision만 또는 tracker만 heartbeat를 끊는 시험을 할 수 있다.
- planner가 아직 없을 때만 `mock_world_node`의 임시 옵션(`publish_plan:=true`, 등속 해석해)으로 `InterceptPlan`을 낸다. 실제 planner가 계약 테스트를 통과하면 이 옵션은 쓰지 않는다.
- 실제 노드와 대역이 같은 이름의 서버·발행자를 동시에 띄우지 않도록 bringup의 `use_mock_<node>` 인자로 한쪽만 켠다.
- 대역의 지연·지터 값은 G0 측정값으로 바꾼다. 그 전에는 가정값임을 시나리오 파일에 적어 둔다.
- 순수 Python fake는 각 패키지의 `test/fakes/`에 둔다(`orchestrator`: skill port·fake clock / `robot_control`: fake transport·fake gripper).

### 7.3 시나리오와 실행 위치

| 시나리오 | 종류 | 주 시험 위치 | ROS I0 |
|---|---|---|---|
| `nominal_20mms` | 정상 | I0 | 포함(20/20) |
| `robot_hang` | 장애 | I0: action 무응답 → cancel 0.5 s → 승격 | 포함 |
| `protective_stop` | 장애 | I0: supervisor → `Stop` → WAIT_OPERATOR | 포함 |
| `heartbeat_loss` | 장애 | I0: vision·tracker 누락 시 L2/L3, controller 누락 시 L4 | 포함 |
| `time_insufficient` | 장애 | `time_budget`·`deadlines` 단위 테스트 + I0 2건(첫 통과 부족 → 다음 통과 시도 / 모든 통과 부족 → `NOT_ATTEMPTED`) | 포함 |
| `gripper_miss` | 장애 | `step_judges`·`abort_policy` 단위 테스트(L1 재시도) | — |
| `gripper_cmd_delay` | 장애 | fake gripper 단위 테스트(상주 루프 응답 지연·무응답·생존 카운터 정지 → 해제 전 L3, 그리퍼 감시 L2/L3) | — |
| `occlusion_long` | 장애 | tracker 단위 테스트·rosbag / 오케스트레이터는 `COASTING`+`t_last_obs` 판정 단위 테스트 | — |
| `speed_change` | 장애 | `speed_change_detector` 단위 테스트·rosbag / 오케스트레이터는 `steady=false` → L3 단위 테스트 | — |
| `frame_drop_10pct` | 장애 | tracker 단위 테스트·프레임을 뺀 편집 bag | — |
| `clock_jump` | 장애 | tracker·`deadlines` 단위 테스트(ROS 시각 역행 시 무효화), 감시 타이머 단위 테스트(steady 시계로 계속 동작) | — |

- 장애 10종 모두 `abort_policy`·`deadlines` 단위 테스트에 이벤트 열로 넣고 기대 중단 등급을 확인한다. 경계값은 임계 직전·경계·직후 세 점으로 확인한다.
- 튕겨 나감·뚜껑 안 공깃돌 남음은 장애가 아니라 판정 결과이므로 `verifier.py` 단위 테스트(정지 영상 세트)로 확인한다.
- 가림·누락·속도 변화는 합성보다 실제 녹화 bag(또는 편집 bag)을 우선한다.

### 7.4 계약 테스트

1. 외부 계약 테스트 한 벌(`test/contract/test_<node>_contract.py`, 대상을 인자로 받음)을 대역과 실제 노드에 똑같이 돌린다. 실제 노드가 통과해야 통합 행렬에서 대역을 대체한다.
2. 범위: `robot_controller`는 상세(skill별 수락·feedback·결과 코드·cancel·deadline 자체 중단·`Stop`), vision·tracker·planner는 최소(이름·형식·QoS·deadline 경과 시 `TIMEOUT`·lifecycle 비활성 시 응답). `mock_world_node`는 vision·tracker 계약을 모두 통과해야 한다.
3. 시험 층을 나눈다. 장애 주입 기능은 대역 전용, L0 내부 감시는 실제 controller 전용, 물리 정지·성능·그리퍼 실제 지연은 실기 전용이다. 한 층의 통과를 다른 층의 증거로 쓰지 않는다.
4. 대역 결과로 정확도·성공률을 보고하지 않는다. 기록에는 `env`, interface 버전, 시나리오 이름·seed, 값이 가정인지 실측인지를 남긴다(10.2절).
5. replay는 로봇 지령 경로를 연결하지 않는다.

### 7.5 통합 행렬

| 단계 | vision | tracker | planner | robot | 목적 | 돌리는 시나리오 | 실행 |
|---|---|---|---|---|---|---|---|
| I0 | mock_world | mock_world | (임시 plan 또는 real) | mock | 오케스트레이터 전이·중단·제한시간 | 7.3절 ROS I0 5종 | CI(`launch_testing`), 매 커밋 |
| I1 | replay(real) | real | — | — | 실데이터 검출→추적 | 회귀 bag 세트(정상·가림·누락·속도 변화) | CI |
| I2 | — | fixture 또는 replay→real | real | — | 예측·시간 예산 | 정상, `time_insufficient`, `UNCERTAIN`, 만료·epoch 불일치·clock reset | CI |
| I3 | mock_world | mock_world 또는 fixture | real | real | 실제 로봇·그리퍼 skill, L0/L4, plan 추종 | 정상, deadline 자체 중단, stale 지령(L0), `Stop`, plan 갱신 중단, 추종 중 그리퍼 개방 | 실기, 안전 높이 |
| I4 | real | real | real | mock | 실영상 → 계획까지 인식 경로 | 정상, stale·추적 손실·계획 유효성 | 실기(로봇 지령 없음) |
| I5 | real | real | real | real | G2·G4·G6 실험 | 게이트 실험 + 안전하게 재현 가능한 장애 | 실기 |
| S | sim camera | real | real | sim robot | Isaac Sim 통합 | 정상(기하·TF·전이) | 병행, 게이트 조건 아님 |

행마다 지정한 시나리오만 돌린다. 적용하지 않는 시나리오는 `N/A`와 이유를 기록해 미실시와 구분한다.

## 8. Spec Driven Development

### 8.1 디렉터리

```
specs/
├── README.md                  # 규칙, 템플릿, REQ ID 체계
├── SPEC-000-system.md         # 범위, 용어, 좌표·시각·단위 규약
├── SPEC-010-interfaces.md     # msg/srv/action 필드·단위·의미, 버전
├── SPEC-100-orchestrator.md   # 상태·전이·단계 판정(4절)
├── SPEC-110-abort-timeout.md  # 중단 등급·감시·제한시간(5, 6절)
├── SPEC-200-vision.md
├── SPEC-300-tracking.md
├── SPEC-400-planner.md
├── SPEC-500-robot-control.md  # G0 계약 결과 포함(스트림·그리퍼 상주 루프 경로)
├── SPEC-600-calibration.md
├── SPEC-700-mocks.md          # mock 동작·시나리오·계약 테스트
├── SPEC-800-records.md        # 기록 스키마(10절)
└── TRACE.md                   # tools/spec/trace.py가 생성하는 추적 행렬
```

### 8.2 spec 템플릿

각 spec은 다음 항목을 모두 채운다. 정하지 못한 항목은 `TBD(측정 G0)`처럼 결정 시점을 적는다.

1. 목적·범위 / 2. 입력·출력(interface 버전) / 3. 파라미터(단위·기본값·근거) / 4. 동작(상태·알고리즘) / 5. 시간 제약(주기·제한시간) / 6. 실패 모드와 응답(중단 등급) / 7. 성공 판정(측정 가능) / 8. 검증 방법(`TEST_SPEC.md`의 TC ID, 게이트) / 9. 미결 사항

시험 케이스의 원본은 저장소 루트의 `TEST_SPEC.md` 하나다. SPEC 문서는 TC ID만 참조하고 시험 내용을 옮겨 적지 않는다.

요구사항은 `REQ-<영역>-<번호>`로 쓴다(예: `REQ-ORC-012: L3는 15 s 안에 끝나야 하며, 넘으면 L4로 올린다`). 요구사항마다 정량 기준을 하나 이상 둔다.

### 8.3 작업 순서

```
spec 작성 → 시험 명세(TC) 작성 → 리뷰(멘토 또는 다른 담당자) → interface·mock 반영
 → 테스트 작성(실패 확인) → 구현 → 테스트 통과 → 추적 행렬 갱신 → 게이트 실험 → 결과로 spec 수치 갱신
```

- 시험 케이스 ID는 `TC-<영역>-<REQ 번호>-<순번>`이다(예: `TC-ABT-012-01`은 `REQ-ABT-012`를 검증). 수준(U/C/I/S/B/E)과 온라인·오프라인 구분, 순수 모듈 API는 `TEST_SPEC.md`에 둔다.
- 순수 모듈은 `TEST_SPEC.md`의 API 시그니처로 테스트를 먼저 쓴다. 시그니처를 바꿀 때는 시험 명세부터 고친다.

- 테스트에는 `@pytest.mark.req("REQ-ORC-012")`·`@pytest.mark.tc("TC-ORC-012-01")`처럼 REQ·TC ID를 붙인다. `tools/spec/trace.py`는 모든 REQ가 테스트 하나 이상과 연결되어 있는지 검사하고, 연결되지 않은 REQ 목록을 출력한다.
- 구현하다가 spec과 다르게 만들어야 하면 **spec부터 고친다.** spec에는 버전(`v0.x`)과 변경 이력을 둔다. interface 변경은 `SPEC-010` 버전을 올리고 mock·계약 테스트를 같은 PR에서 고친다.
- 게이트 통과는 그 게이트에 걸린 REQ의 테스트 통과와 실험 기준 충족을 **둘 다** 요구한다.

## 9. 시간 예산

```
T_available = (x_exit − x_cap_now) / v_cap
T_required  = T_transit + T_sync + T_descend + T_release(p95) + T_clear
T_release   = T_cmd(레지스터 트리거 → 루프 DRL → Modbus 송신) + Δ_open / v_open + T_fall
time_margin = T_available − T_required        # 시도 조건: time_margin ≥ margin
```

- `margin`은 `T_required`에 넣지 않고 판정에서 한 번만 쓴다. `InterceptPlan.time_margin`, WAIT_CAP 판정(4.3절), `evaluate_budget()`(`TEST_SPEC.md` 2절)이 모두 이 정의를 따른다. 초기 `margin`은 1.0 s다(PLAN_INTERCEPT의 `time_margin ≥ 1.0 s`와 같은 값).

- `x_exit`: 카메라 관측 구간과 로봇 안전 도달 구간(후퇴 경로 포함)의 교집합 하류 끝.
- `T_transit`: 대기 자세에서 예측 해제점 위까지 이동 시간. 대기 자세를 도달 구간 상류 끝 위에 두어 줄인다.
- 공깃돌은 병뚜껑이 들어오기 전에 집어 두므로 파지 시간은 식에 들어가지 않는다.
- `T_release`: 핑거 이동은 최대 닫힘 속도 75 mm/s 기준 Δ_open 4 mm에서 약 0.05 s, 낙하 `T_fall = √(2h/g)`는 h = 10 mm에서 약 0.045 s다. `T_cmd`(트리거 레지스터 서비스 호출 + 루프 주기 최대 20 ms + Modbus 송신)는 G0에서 측정하며 가장 큰 불확실 성분이다.
- 식은 `intercept_planner/time_budget.py` 한 곳에 구현하고, `deadlines.py`가 이를 사용해 동적 deadline을 계산한다.
- WAIT_CAP(통과마다)과 PLAN_INTERCEPT 두 곳에서 판정한다. 부족하면 그 통과를 보내고 다음 통과를 기다린다(`max_passes`). 판정할 때마다 `T_available`, `T_required`, 각 성분값을 `StepRecord.metric_*`에 남긴다.
- 성분값은 제안값으로 시작하고, `records/`에 쌓인 단계 소요시간 p95로 갱신한다(10.4절 기록).
- 예: 교집합 길이 300 mm, 20 mm/s → 15 s. `T_required`가 4 s이고 `margin`이 1 s이면 `time_margin` 11 s로 시도하며, 같은 조건에서 이론상 60 mm/s(`T_available` 5 s)까지 시도할 수 있다(예시 수치).
- 최종 운전 속도는 벨트가 연속적으로 안정 운전하고, 로봇의 속도·가속·추종 오차, 위 시간 예산, 해제 게이트와 투입 성공 기준을 모두 만족하는 실측 속도 중 가장 빠른 값으로 정한다. 저속에서 발생하는 간헐 정지도 속도 불안정으로 기록하고 해당 조건의 안정 운전 판정에 포함한다.
- 벨트의 아날로그 조절부에는 수치 눈금이 없으므로 속도 조건은 고유 이름으로 구분하고, 실제 속도와 변동은 독립 기준으로 측정한다. 윤활 등 기계 상태가 달라진 자료는 별도 조건으로 구분한다.

## 10. 정량 기록

### 10.1 기록 대상과 위치

| 기록 | 형식·위치 | 생성 | 내용 |
|---|---|---|---|
| 단계 기록 | `records/trials/<date>/<cycle_id>.steps.jsonl` | `record_logger_node` | `StepRecord` 전체 |
| 시도 기록 | `records/trials/<date>/trials.jsonl` | `record_logger_node` + `trial_labeler.py` | 10.2절 스키마 |
| 원시 데이터 | rosbag2(용량 때문에 저장소 밖), 경로·SHA-256만 시도 기록에 | `record_trial.sh` | 영상, TF, 계획, 지령, 피드백, 그리퍼 상태 |
| 성능 측정 | `records/bench/<항목>-<date>.csv` | 측정 스크립트 | 지연·지터·주기 분포(G0 스트림·그리퍼, G3, G5) |
| 파라미터 변경 | `records/param_changes.csv` | 수동 | 10.4절 |
| 개발 지표 | `records/devlog.csv` | 일 1회 스크립트 | 10.3절 |
| 게이트 보고서 | `deliverables/gate-G<n>-report-<YYYY-MM-DD>.md` | 수동 + `tools/eval` | 10.5절 |

### 10.2 시도 기록 스키마 (trials.jsonl 한 줄)

```json
{
  "trial_id": "2026-11-20-G6-0042",
  "env": "real|sim|mock|replay",
  "interface_version": "SPEC-010 0.2",
  "scenario": {"name": "", "seed": 0},
  "assumed_params": [],
  "gate": "G6",
  "git_sha": "...", "spec_versions": {"SPEC-100": "0.3"},
  "calibration_id": "...", "doosan_sha": "...", "controller_ver": "...", "gripper_fw": "...",
  "mode": "MODE_SYNCHRONIZED",
  "belt_speed_ref_mps": 0.020,
  "belt_speed_est_mps": 0.0197,
  "lighting": "lab-A", "operator": "...",
  "result": "SUCCESS|FAILURE|UNKNOWN|CANCELED|SAFE_ABORT|PROTECTIVE_STOP|NOT_ATTEMPTED",
  "abort_level": 0, "reason_code": "",
  "observer_verdict": "SUCCESS", "auto_verdict": "SUCCESS",
  "release_gate": {"e_xy_m": 0.0012, "v_rel_mps": 0.0021, "track_status": "COASTING", "t_since_obs_s": 0.18, "pos_std_m": 0.0004},
  "release_height_m": 0.010,
  "release_error_m": 0.0013, "stone_offset_m": 0.0041,
  "gripper": {"open_cmd_latency_s": 0.12, "grip_current_ma": 150},
  "passes_skipped": 0,
  "time_margin_s": 9.1, "cycle_time_s": 24.3,
  "step_durations_s": {"PICK_STONE": 6.2, "WAIT_CAP": 8.4, "APPROACH": 1.1},
  "rosbag": {"path": "...", "sha256": "..."},
  "notes": ""
}
```

- 실패와 중단 기록을 빼지 않는다. 선행조건·시간 예산 때문에 시도하지 않은 경우도 `result: NOT_ATTEMPTED`와 `reason_code`로 남긴다.
- `belt_speed_ref_mps`는 영상 추정과 독립적인 기준값이다. 벨트 전체 길이를 병뚜껑 한 바퀴 시간(스톱워치 또는 고속 촬영)으로 나눠 구한다.
- `scenario`는 mock·replay일 때 시나리오 이름(또는 bag 이름)과 seed다. `assumed_params`에는 실측이 아닌 가정값으로 돌린 파라미터 이름을 적는다.

### 10.3 개발 지표 (`devlog.csv`, 하루 한 줄)

날짜, git SHA, 단위 테스트 통과/전체, 계약 테스트 통과/전체, REQ 연결률(%), 통합 행렬 통과 단계(I0~I5), ROS I0 시나리오 통과/전체, 단위·replay 시나리오 통과/전체, 게이트 상태, 미결 이슈 수, 실기 사용 시간(h).

### 10.4 파라미터 변경 기록 (`param_changes.csv`)

날짜, 파라미터, 이전 값, 새 값, 단위, 근거 데이터(파일·N·통계값), 변경자, 영향받는 REQ. 제안값을 실측값으로 바꾸는 경우도 모두 여기에 남긴다.

### 10.5 게이트 보고서 필수 항목

- 조건: 날짜, git SHA, calibration_id, 속도·조명·공깃돌·병뚜껑 실측값, 그리퍼 펌웨어, env
- 표본 수 N과 결과 분포(SUCCESS/FAILURE/UNKNOWN/SAFE_ABORT/PROTECTIVE_STOP/NOT_ATTEMPTED)
- 성공률과 Wilson 95% 신뢰구간(`analyze_drop.py`)
- 오차·지연은 p50/p95/p99/최대값 (평균만 보고하지 않음)
- 합격 기준 대비 판정: `통과 / 미달 / 보류(측정 미완)`. 측정하지 않은 항목은 통과로 쓰지 않는다
- 자동 판정과 관찰자 판독의 혼동행렬(G2 이후)
- 최적화 보고서는 같은 조건의 변경 전·후 수치를 나란히 적는다

## 11. 개발 단계와 게이트

### 11.1 순서

```
M0  Spec·계약·골격 (전원)                      → I0 통과
 ├─ 트랙 A  로봇·제어:  G0 → G2(로봇측) → G5        I3
 ├─ 트랙 B  비전·보정:  G1 → G3                     I1, I4
 ├─ 트랙 C  계획·오케스트레이션: 시간 예산·예측(fixture) → 장애 주입 확장 → G4 로직   I2
 └─ 트랙 D  시뮬레이션: 단순 장면 → 접촉 물리        S
M1  G2 정지 벨트 투입 (A+B 합류, I5)
M2  G4 예측 타이밍 투입
M3  G6 동기 투입 (G5 통과 시)
M4  G7 통합·복구·비교 평가
O   최적화 (12절, M2 이후 측정 기반으로)
```

- 트랙은 사람 수에 맞춰 묶는다. 인원이 적으면 A → B → C 순서로 진행하되, M0(오케스트레이터 순수 모듈, mock 2종의 정상 경로, ROS I0 시나리오)은 먼저 만든다. 단위·replay 시나리오와 TCP 지연 모델 같은 대역 기능은 M0 이후 필요한 트랙에서 추가한다.
- 트랙끼리는 interface(3절)·계약 테스트·mock 2종·fixture·rosbag, 그리고 계약 테스트를 통과한 실제 노드로 연결된다. 다른 트랙이 늦어도 통합 시험을 계속 할 수 있다.

### 11.2 통과 조건

마일스톤·게이트별 작업 항목과 합격 기준은 `Progress.md`에 둔다. 게이트 통과에는 다음 세 가지가 모두 필요하다.

1. `Progress.md`의 해당 게이트 합격 기준 충족
2. 해당 게이트에 연결된 REQ 테스트 전부 통과(`tools/spec/trace.py`로 확인)
3. 게이트 보고서(10.5절) 작성

### 11.3 개발 일정 중단(범위 축소) 규칙

프로젝트 기간과 시연 일자를 정하면 게이트마다 timebox를 둔다. timebox를 넘기면 아래처럼 범위를 줄인다. 줄인 범위와 그 근거 수치는 기록한다.

| 상황 | 결정 |
|---|---|
| G0에서 스트림 주기·지연 기준 미달 | `MODE_SYNCHRONIZED` 제외, `MODE_PREDICTED`까지 진행 |
| G0에서 상주 루프가 도는 동안 스트림 지령이 받아들여지지 않거나, 개방 지연·지터가 시간 예산을 넘음 | 하강·해제·후퇴를 한 DRL 프로그램 안에서 비동기 이동(`amovel`) + 그리퍼 명령으로 실행하는 방식을 1회 timebox 안에서 검토(목표는 ROS가 계산해 프로그램 인자로 넘김). 그래도 안 되면 `MODE_SYNCHRONIZED`를 빼고 `MODE_PREDICTED`를 정해진 시각 개방으로 진행. 그리퍼 배선을 PC 쪽으로 옮기는 방안은 사용자 승인 후에만 검토 |
| G5가 timebox 안에 통과하지 못함 | G6 생략, G4 결과로 G7 진행 |
| 핑거 어댑터 검증 실패(공깃돌 파지 불안정, 개방 시 테두리 간섭) | 핑거 패드 형상(V홈·오목) 변경을 1회 timebox 안에서 진행, 그래도 실패하면 공깃돌·병뚜껑 크기 조합을 바꾸고 바꾼 실측값을 기록 |
| G2 정지 벨트에서도 튕겨 나감이 10%를 넘음 | 해제 높이 축소, 더 깊은 병뚜껑으로 교체를 검토하고 대상물 변경을 기록 |
| 병뚜껑이 롤러·벨트 하부를 통과하지 못함 | 단일 통과 운용: `max_passes = 0`, WAIT_CAP 제한시간은 `T_lap` 대신 `wait_target_timeout`(작업자 재부착 후 병뚜껑이 도달 구간에 오는 시간 + 여유)을 쓴다. 재부착은 사이클 밖(WAIT_OPERATOR, 벨트 정지)에서 하고 작업자 준비 확인 뒤 사이클을 시작한다 |
| Isaac Sim 접촉 물리 수렴 실패 | sim은 기하·전이 검증용으로만 쓰고 물리 성공률은 보고하지 않음 |

### 11.4 온라인·오프라인 작업 구분

| 구분 | 표기 | 정의 | 필요한 것 |
|---|---|---|---|
| 온라인 | `[ON]` | 실기 장비 없이 개발 PC·CI에서 끝나는 일 | 개발 PC(Ubuntu 24.04, ROS 2 Jazzy), CI. Isaac Sim은 RTX GPU PC |
| 온라인(현장 자료) | `[ON*]` | 온라인에서 하지만 현장에서 수집한 자료(rosbag·영상·측정값)가 있어야 하는 일 | 해당 자료가 `records/`·bag 저장소에 있음 |
| 오프라인(현장) | `[OFF]` | E0509·RH-P12-RN(A)·D455·컨베이어·실물 대상물이 있는 현장에서만 할 수 있는 일 | 현장 장비, 비상정지 담당자, 관찰자(실험 시) |

- **판정 기준:** 실제 로봇·그리퍼에 지령을 보내거나(I3, I5), 실기 센서의 실시간 입력이 필요하거나(I4), 물리량을 실측하는(B) 일은 `[OFF]`다. 순수 모듈 단위 테스트(U), 계약 테스트(C, 대역 또는 장비 없는 실제 노드), I0~I2, Doosan `mode:=virtual`, Isaac Sim(S), 분석·보고서 작성은 `[ON]`이다. 현장 자료로 회귀·튜닝하는 일은 `[ON*]`이며, 자료가 없으면 그 자료를 수집하는 `[OFF]` 항목을 먼저 잡는다.
- **현장 세션 단위:** 현장 작업은 `Progress.md`의 현장 세션 묶음(OFF-1~OFF-n) 단위로 잡는다. 세션마다 시작 조건(먼저 끝낼 `[ON]` 항목), 할 일, 수집할 자료, 종료 조건을 둔다.
- **현장에서 처음 코드를 짜지 않는다:** 세션 시작 전에 해당 skill·측정 스크립트를 구현하고 U·C·I0 시험을 통과시킨다. 현장에서 고친 코드는 같은 날 커밋하고 `[ON]` 시험을 다시 돌린다.
- **자료는 한 번에 모은다:** 세션마다 rosbag·측정 CSV·사진을 남겨 다음 `[ON*]` 작업에 쓴다. 한 번 방문에 여러 게이트의 자료를 함께 수집한다.
- **원격 접속은 온라인이 아니다:** 현장 PC에 원격 접속하더라도 실제 로봇·그리퍼가 움직이는 일은 `[OFF]`로 보며, 현장에 비상정지 담당자가 있어야 한다.

## 12. 최적화 (기능 통합 이후)

- 시작 조건: M2(G4) 통과. 그 전에는 기능 완성과 측정 기반 마련에 집중하고 성능 최적화는 하지 않는다. 안전·정확도 기준을 맞추는 데 꼭 필요한 경우만 예외로 한다(예: G0 지터 미달 시 adapter C++ 이전).
- 절차: 기준값 측정 → 병목 식별(10.1절 bench) → 한 번에 하나만 변경 → 같은 조건 재측정 → 변경 전·후 보고. 효과가 없으면 되돌린다.
- 후보: adapter·서보 루프 C++ 이전, 컴포넌트 합성(intra-process), QoS·executor 조정, 검출 ROI 축소, 계획 재계산 빈도, 대기 자세·접근 경로 단축, 그리퍼 개방 명령 경로 단축, 판정 임계값 조정(혼동행렬 근거).
- 최적화로 바꾼 파라미터는 모두 10.4절 기록 대상이다. mock 시나리오의 지연 값도 함께 고친다.
