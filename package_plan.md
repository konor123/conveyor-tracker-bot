# ROS 2 패키지 구조 — Button Stacking Project

- 확정: 2026-11-04 / 진행 계획: `progress_plan.md`
- 개발 환경: Ubuntu 24.04 Native / ROS 2 Jazzy / Isaac Sim 5.1
- 로봇: Doosan E0509 + 2지 그리퍼(모델 미기재 → G0에서 확정)
- 비전: Intel RealSense D455 (작업대 고정, eye-to-hand)
- 작업: 검은 매트 위 직경 10 mm 흰 단추 파지 → 녹색 컨베이어 위 배치 → 영상으로 이동 속도 추정 → 두 번째 단추를 이동 중인 첫 번째 단추 위에 적층
- 컨베이어: 수동 다이얼, 엔코더 없음 → ROS 제어 패키지·속도 명령 토픽 없음

## 0. 설계 원칙

1. **단위·좌표 규약:** 프로젝트 내부는 SI(m, s, rad)와 quaternion만 쓴다. Doosan의 mm/deg·Euler(ZYZ) 변환은 `button_robot_control/motion_adapter.py` 한 곳에서만 하고 단위 테스트로 고정한다.
2. **시각 규약:** 모든 관측 메시지의 `header.stamp`는 *영상 노출 시각*이다. 수신 시각으로 덮어쓰지 않는다. 상태(`t_state`)·마지막 관측(`t_last_obs`)·계획 생성·접촉 예정(`t_contact`)·유효기간(`valid_until`)을 별도 필드로 둔다.
3. **TF와 예측의 역할 분담:** TF는 고정 좌표 관계와 *현재까지 관측된* 단추 위치 시각화에만 쓴다. tf2는 미래 시각을 외삽하지 않는다(`lookupTransform`이 extrapolation 오류를 냄). 미래 위치 예측은 `TrackedButton`(위치·속도·공분산)으로 planner가 계산한다.
4. **Depth 미사용 원칙:** 10 mm 단추의 XY는 왜곡 보정 RGB 중심과 *보정된 평면*(매트·벨트·단추 윗면 높이)의 광선 교차로 구한다. D455 Depth는 평면 보정 확인과 이상 감시에만 쓴다. 최대 Depth 해상도(1280×720)의 Min-Z가 약 520 mm라는 점도 고려한다.
5. **제어 권한 단일화:** 로봇에 지령을 보내는 주체는 `robot_controller_node` 하나뿐이다. 오래된 지령(freshness 초과)은 보내지 않고, 감시 타이머가 정지 지령을 낸다.
6. **판단 로직과 ROS 노드 분리:** 검출·필터·예측·시간 예산·단위 변환은 ROS와 무관한 순수 Python 모듈로 만들어 패키지별 `test/`에서 단위 테스트하고, 노드는 그 모듈을 감싸기만 한다.

## 1. 전체 디렉터리 구조

```
button_stacking_ws/
├── src/
│   ├── button_stacking_interfaces/          # ament_cmake + rosidl
│   │   ├── msg/
│   │   │   ├── ButtonDetection.msg
│   │   │   ├── ButtonDetectionArray.msg
│   │   │   ├── TrackedButton.msg
│   │   │   ├── TrackedButtonArray.msg
│   │   │   ├── BeltMotion.msg
│   │   │   ├── InterceptPlan.msg
│   │   │   └── TaskStatus.msg
│   │   ├── action/
│   │   │   └── StackingTask.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml                      # rosidl_interface_packages 그룹
│   │
│   ├── button_vision/                       # ament_python
│   │   ├── button_vision/
│   │   │   ├── __init__.py
│   │   │   ├── vision_node.py               # 검출+좌표 변환을 한 노드에서 처리(영상 이중 전송 방지)
│   │   │   ├── detector.py                  # HSV/명도 마스크, 원·타원 중심, 신뢰도
│   │   │   ├── localizer.py                 # 왜곡 보정, 광선-평면 교차, 공분산
│   │   │   └── robot_mask.py                # 로봇 링크 투영 마스크(가림 영역 제외)
│   │   ├── config/vision.yaml
│   │   ├── resource/button_vision
│   │   ├── test/                            # test_detector.py, test_localizer.py
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── button_tracking/                     # ament_python
│   │   ├── button_tracking/
│   │   │   ├── __init__.py
│   │   │   ├── tracker_node.py              # TrackedButton·BeltMotion·TF(button_<id>) 발행
│   │   │   ├── kalman_cv.py                 # 벨트 좌표계 [x, y, vx, vy] 등속 칼만 필터
│   │   │   ├── association.py               # 게이팅·ID 부여·재획득
│   │   │   └── speed_change_detector.py     # 다이얼 변경(혁신값 누적) 검출
│   │   ├── config/tracking.yaml
│   │   ├── resource/button_tracking
│   │   ├── test/                            # 프레임 누락·역순·가림·재획득 테스트
│   │   ├── setup.py / setup.cfg / package.xml
│   │
│   ├── button_stacking_planner/             # ament_python
│   │   ├── button_stacking_planner/
│   │   │   ├── __init__.py
│   │   │   ├── intercept_planner_node.py
│   │   │   ├── time_budget.py               # T_available vs 작업 소요 시간 판정
│   │   │   ├── intercept.py                 # 접촉 시각·위치 계산(예측의 유일한 소유자)
│   │   │   └── reachability_checker.py      # 작업영역·IK·후퇴 경로 여유
│   │   ├── config/planner.yaml
│   │   ├── resource/button_stacking_planner
│   │   ├── test/
│   │   ├── setup.py / setup.cfg / package.xml
│   │
│   ├── button_robot_control/                # ament_python (지터 미달 시 adapter만 C++ 전환)
│   │   ├── button_robot_control/
│   │   │   ├── __init__.py
│   │   │   ├── robot_controller_node.py     # 유일한 로봇 지령 주체
│   │   │   ├── motion_adapter.py            # SI/quaternion ↔ mm/deg/ZYZ, freshness 감시
│   │   │   ├── tracking_controller.py       # v_cmd = v_ff + Kp·(p_target − p_tcp), 제한
│   │   │   ├── gripper_adapter.py           # 실제 그리퍼 인터페이스 래퍼, 개폐 지연 측정값 보유
│   │   │   ├── pick_sequence.py             # 정적 파지(move_line 기반)
│   │   │   └── dynamic_place.py             # 접근→동기→하강→해제→동기 유지→후퇴
│   │   ├── config/robot_control.yaml
│   │   ├── resource/button_robot_control
│   │   ├── test/                            # fake driver로 단위 변환·stale·cancel 테스트
│   │   ├── setup.py / setup.cfg / package.xml
│   │
│   ├── button_task_manager/                 # ament_python
│   │   ├── button_task_manager/
│   │   │   ├── __init__.py
│   │   │   ├── task_manager_node.py         # StackingTask 액션 서버
│   │   │   ├── state_machine.py
│   │   │   ├── verifier.py                  # 적층 판정(SUCCESS/FAILURE/UNKNOWN)
│   │   │   └── recovery.py
│   │   ├── resource/button_task_manager
│   │   ├── test/
│   │   ├── setup.py / setup.cfg / package.xml
│   │
│   ├── button_calibration/                  # ament_python
│   │   ├── button_calibration/
│   │   │   ├── __init__.py
│   │   │   ├── camera_intrinsics_check.py   # CameraInfo·왜곡 계수 확인
│   │   │   ├── plane_calibration.py         # 매트·벨트 평면, 벨트 진행 방향 벡터
│   │   │   ├── eye_to_hand_calibration.py   # 고정 카메라 ↔ robot_base
│   │   │   └── tcp_offset.py                # 파지한 단추 중심 ↔ TCP 오프셋
│   │   ├── config/
│   │   │   ├── camera_extrinsics.yaml       # calibration_id, 날짜, 잔차, 카메라 프로파일 포함
│   │   │   ├── mat_plane.yaml
│   │   │   └── belt_plane.yaml              # 평면 + 진행 방향 + 단추 두께
│   │   ├── resource/button_calibration
│   │   ├── test/
│   │   ├── setup.py / setup.cfg / package.xml
│   │
│   ├── button_stacking_bringup/             # ament_cmake (launch/config 설치)
│   │   ├── launch/
│   │   │   ├── vision.launch.py
│   │   │   ├── robot.launch.py              # mode:=real|virtual, model:=e0509
│   │   │   ├── stacking.launch.py
│   │   │   ├── simulation.launch.py         # use_sim_time:=true 일괄 적용
│   │   │   └── replay.launch.py             # rosbag 재생(로봇 지령 경로 미연결)
│   │   ├── config/system.yaml
│   │   ├── test/                            # launch_testing + rosbag replay 통합 테스트
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── button_stacking_sim/                 # ament_cmake (자원 설치만)
│       ├── usd/button_stacking_scene.usd    # 대용량 → Git LFS
│       ├── scripts/conveyor_simulation.py   # Isaac Sim python으로 실행(colcon 실행 대상 아님)
│       ├── config/simulation.yaml           # physics rate, contact/rest offset, 벨트 표면 속도
│       ├── CMakeLists.txt
│       └── package.xml
│
├── tools/eval/                              # 성능 평가 스크립트(ROS 패키지 아님)
│   ├── record_trial.sh                      # rosbag2 기록 규칙
│   ├── analyze_tracking.py                  # 독립 GT 대비 위치·속도 오차
│   ├── analyze_stacking.py                  # 성공률 + Wilson 신뢰구간, 중심 오차
│   └── trial_labeler.py                     # 관찰자 판독(적층 성공·실패) 기록
└── README.md
```

구성 규칙: 테스트는 `colcon test`에 자동 등록되도록 각 패키지 `test/`에 둔다. ament_python 패키지마다 `resource/<패키지명>`, `setup.cfg`, entry point, config·launch 설치 규칙을 둔다. 검출과 좌표 변환은 한 노드(`vision_node`)에서 처리하고 모듈만 나눈다. 카메라가 작업대에 고정되어 있으므로 보정은 eye-to-hand 방식이다. 패키지는 9개이며, 인원이 적으면 planner를 task_manager 안의 모듈로 시작해도 된다.

RealSense ROS 2 Wrapper와 doosan-robot2(Jazzy)는 외부 의존성이며 **설치 커밋 SHA·제어기/DRFL 버전을 README에 고정 기록**한다.

## 2. 패키지별 책임

| 패키지 | 책임 | 소유하는 상태/결정 | 주요 노드 |
|---|---|---|---|
| `button_stacking_interfaces` | msg·action 정의 | — | — |
| `button_vision` | 검출, 보정 평면 교차로 XY 계산, 로봇 가림 마스크 | 관측(측정값·공분산) | `vision_node` |
| `button_tracking` | ID 부여, 칼만 필터, 벨트 속도·변화 검출, 관측 TF | 단추 상태 `[x,y,vx,vy]`와 `OBSERVED/COASTING/LOST` | `tracker_node` |
| `button_stacking_planner` | 시간 예산 판정, 접촉 시각·위치, 도달성 | 미래 예측(유일 소유) | `intercept_planner_node` |
| `button_robot_control` | Doosan·그리퍼 어댑터, 정적 시퀀스, 동기 추종 루프 | 지령 권한, freshness 감시 | `robot_controller_node` |
| `button_task_manager` | 상태 머신, 액션 서버, 적층 판정, 복구 | 작업 단계, 결과 | `task_manager_node` |
| `button_calibration` | 내부·외부·평면·TCP 보정, 검증 점 오차 | calibration_id | 보정 도구 |
| `button_stacking_bringup` | launch·공통 설정·통합 테스트 | namespace, `use_sim_time` | launch |
| `button_stacking_sim` | Isaac Sim 장면·컨베이어·가상 카메라 | sim GT(평가 전용) | 시뮬레이션 스크립트 |

필터는 tracker에서 한 번만 적용한다. planner는 tracker 상태를 받아 예측만 하고, 다시 필터링하지 않는다.

## 3. TF 트리

```
world
 └── robot_base (base_link, 드라이버/URDF 명칭에 맞춤)
      ├── conveyor_frame        (static: 벨트 장치 고정, x축 = 벨트 진행 방향, z = 벨트 표면 법선)
      ├── mat_frame             (static)
      ├── camera_link → camera_color_optical_frame   (static, eye-to-hand 보정 결과 / 광학 TF는 RealSense 드라이버 소유)
      ├── ... robot links → tcp (robot_state_publisher 소유)
      └── conveyor_frame → button_<track_id>   (dynamic, tracker 발행, stamp = t_state, 시각화용)
```

- `conveyor_frame`은 벨트 표면을 따라 움직이지 않는다(장치 고정).
- `button_<id>`의 orientation은 identity(원형 단추의 yaw는 관측하지 않음)이며 파지 자세와 구분한다.
- 미래 위치는 TF로 발행하지 않고 RViz `Marker`(`/stacking/predicted_marker`)로만 표시한다.
- 같은 TF를 두 노드가 발행하지 않는다.

## 4. 인터페이스 정의

```text
# ButtonDetection.msg
uint8 REGION_UNKNOWN=0
uint8 REGION_MAT=1
uint8 REGION_BELT=2
uint32 detection_id              # 프레임 내 관측 번호(추적 ID 아님)
uint8 region
float64 u                         # 왜곡 보정 전 픽셀 중심
float64 v
float64 diameter_px
float32 confidence
float64[4] center_cov_px          # 2x2 row-major
geometry_msgs/Point position      # 보정 평면 교차 결과, ButtonDetectionArray.header.frame_id 기준 [m]
float64[9] position_cov           # 3x3 [m^2]
bool position_valid
bool occluded_partial             # 로봇 마스크와 겹침

# ButtonDetectionArray.msg
std_msgs/Header header            # stamp = 영상 노출 시각, frame_id = robot_base
uint64 image_seq
string calibration_id
ButtonDetection[] detections

# TrackedButton.msg
uint8 OBSERVED=0
uint8 COASTING=1                  # 관측 없이 예측만 하는 중(가림)
uint8 LOST=2
std_msgs/Header header            # stamp = t_state, frame_id = conveyor_frame
uint32 track_id
uint32 epoch                      # 재시작·리셋 시 증가 → 이전 계획 무효
uint8 status
builtin_interfaces/Time last_observed_stamp
uint32 observation_count
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[16] state_cov             # [x, y, vx, vy] 4x4
builtin_interfaces/Time valid_until

# TrackedButtonArray.msg
std_msgs/Header header
TrackedButton[] tracks

# BeltMotion.msg
uint8 SOURCE_BUTTON=0             # 단추 이동에서 추정(벨트 직접 측정 아님)
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
uint32 target_track_id
uint32 target_epoch
builtin_interfaces/Time source_state_stamp
builtin_interfaces/Time contact_time
builtin_interfaces/Time valid_until
geometry_msgs/Pose target_tcp_pose        # contact_time의 해제 직전 TCP 목표
geometry_msgs/Vector3 target_velocity     # 동기화할 선속도 [m/s]
float64 position_std              # contact_time 예측 표준편차 [m]
float64 time_margin               # 남은 작업 가능 시간 − 필요 시간 [s]
uint8 status
string reason

# TaskStatus.msg
std_msgs/Header header
string state                      # 상태 머신 상태명
uint16 error_code
string command_owner
bool robot_ready
bool protective_stop
bool gripper_holding
float64 time_remaining

# StackingTask.action
uint8 MODE_STATIC=0               # 정지 벨트
uint8 MODE_PREDICTED=1            # 예측 접촉점 + 짧은 속도 일치 구간
uint8 MODE_SYNCHRONIZED=2         # 지속 갱신 목표 + 속도 동기 추종
uint8 mode
float64 max_belt_speed            # 이 속도를 넘으면 시도하지 않음 [m/s]
float64 timeout                   # [s]
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 UNKNOWN=2                   # 판정 불가(성공률 분자에서 제외, 분모에 포함)
uint8 CANCELED=3
uint8 SAFE_ABORT=4
uint8 result
string reason_code
float64 center_error              # [m] (동적 적층은 track 기반 추정값)
bool center_error_valid
bool center_error_estimated       # 아래 단추 위치를 예측으로 추정했으면 true
float64 hold_time                 # 적층 유지 확인 시간 [s]
float64 cycle_time
---
string state
uint32 plan_id
float64 time_remaining
float64 tracking_std
```

## 5. 토픽·주기

| 토픽(상대명, launch namespace 적용) | 형식 | 발행 | 주기 | 비고 |
|---|---|---|---|---|
| `camera/color/image_raw`, `camera_info` | sensor_msgs | RealSense | 30 Hz(1280×800) | 자동 노출·화이트밸런스 고정 |
| `buttons/detections` | `ButtonDetectionArray` | vision | 영상 주기 | 위치·공분산 포함 |
| `buttons/tracked` | `TrackedButtonArray` | tracker | 영상 주기 | |
| `belt/measured_motion` | `BeltMotion` | tracker | 영상 주기 | |
| `stacking/intercept_plan` | `InterceptPlan` | planner | 상태 갱신마다 | revision 증가 |
| `stacking/predicted_marker` | `visualization_msgs/Marker` | planner | 10 Hz | 시각화 전용 |
| `task/status` | `TaskStatus` | task_manager | 10 Hz + 상태 변경 시 | |
| `stacking_task` (action) | `StackingTask` | task_manager | — | |
| `<dsr_ns>/servol_stream` 또는 `speedl_stream` (RT 계열 후보: `servol_rt_stream`, `speedl_rt_stream`) | `dsr_msgs2` | robot_control | G0에서 측정 후 확정(50~100 Hz 후보) | 메시지에 Header 없음 → 어댑터가 freshness 검사 |
| `<dsr_ns>/motion/move_line` (service) | `dsr_msgs2/srv/MoveLine` | robot_control | — | 정적 pick/place와 접근 이동 |

이 표의 doosan-robot2 인터페이스는 공개 Jazzy 브랜치 커밋 `dade65d`에서 확인한 것이다. **현장 설치 버전과 E0509 제어기가 실제로 받아들이는지는 G0에서 확인한다.**

## 6. 작업 상태 머신

```
IDLE
 → PRECHECK            보정 유효, 로봇 준비, 그리퍼 응답, 벨트 속도 ≤ max_belt_speed, 시간 예산 사전 판정
 → DETECT_BUTTONS      매트 위 두 단추를 처음에 함께 검출(매트는 정지 → 두 번째 탐지 시간 절약)
 → PICK_FIRST
 → PLACE_FIRST_ON_BELT 관측·도달 구간의 상류 끝에 배치
 → WAIT_TRACK_STABLE   관측 수 ≥ N, 속도 공분산 ≤ 기준, 배치 직후 미끄러짐 안정화
 → PICK_SECOND         시작 전 시간 예산 재판정(부족 → ABORT_CYCLE)
 → PLAN_INTERCEPT      InterceptPlan.status == OK
 → APPROACH            예측 접촉점 위 안전 높이(예: +30 mm)로 이동
 → SYNCHRONIZE         XY를 단추 속도에 맞춤, 추종 오차 게이트
 → DESCEND             동기 유지하며 해제 높이까지 하강
 → RELEASE             해제 게이트 통과 시 그리퍼 개방, 개방 지연(p95)+여유 동안 동기 유지
 → RETREAT             동기 유지한 채 수직 상승 후 이탈
 → VERIFY              SUCCESS / FAILURE / UNKNOWN
 → DONE

중단 경로
 해제 전 실패(추적 LOST, 계획 EXPIRED/UNCERTAIN, 시간 부족, 추종 오차 초과, cancel)
   → SAFE_ABORT: 동기 유지하며 상승 → 안전 자세 → 두 번째 단추를 매트에 내려놓음 → WAIT_OPERATOR
 해제 후 이상 → RETREAT 완료 후 VERIFY(FAILURE/UNKNOWN)
 보호 정지·통신 단절·지령 stale → PROTECTIVE_STOP → WAIT_OPERATOR (자동 리셋·무조건 원점 복귀 금지)
 첫 단추가 작업 구간 이탈(ABORT_CYCLE) → WAIT_OPERATOR (작업자가 단추 회수 후 재개)
 자동 재시도 허용: 매트 재검출, 재파지(최대 N회)만
```

해제 게이트(모두 만족해야 개방):
- TCP-목표 XY 오차 ≤ `e_xy_max`(초기 1.0 mm 제안), 상대 속도 ≤ `v_rel_max`(초기 2 mm/s 제안)
- 추적 상태 `OBSERVED`, 또는 `COASTING`이면서 `t_now − t_last_obs ≤ τ_max`(측정으로 결정, 초기 0.2 s 제안)
- 위치 예측 표준편차 ≤ 기준, `valid_until` 미경과, `epoch` 일치

### 6.1 적층 판정 (보조 카메라 없음)

측면 카메라가 없으므로 고정 D455 상단 영상과 관찰자 판독을 조합한다. 위에서 보이는 흰 원 하나만으로는 성공으로 판정하지 않는다.

1. **기준 정답 = 관찰자 판독:** 실험마다 관찰자가 벨트 옆에서 2단 적층·1 s 유지 여부를 직접 보고 `tools/eval/trial_labeler.py`로 기록한다(SUCCESS/FAILURE/판독 불가). 성공률은 이 기록으로 산출한다.
2. **자동 판정(verifier, 보조):** 후퇴 완료 후 아래를 모두 만족하면 SUCCESS 후보, 하나라도 어긋나면 FAILURE, 판단할 수 없으면 UNKNOWN.
   - 벨트 ROI의 흰 단추 수가 1개이고 첫 단추 track과 연속된다(2개면 낙하·이탈 → FAILURE).
   - 그리퍼 개방 확인 + 그리퍼가 단추를 붙여 나오지 않았다(그리퍼 주변 ROI에 흰 물체 없음, 매트 위 단추 수 변화 없음).
   - 단추 외곽 형상이 단일 원에 가깝다(위아래 단추가 크게 어긋나면 외곽이 길어진 형태로 보임).
   - (선택) Depth ROI 평균으로 단추 윗면 높이가 벨트 + 2h 근처인지 확인. 근거리 Depth 정확도가 미확인이므로, 관찰자 판독과 비교해 쓸 만하다고 확인된 뒤에만 판정에 넣는다.
3. **중심 오차 측정:** 아래 단추는 위 단추에 가려 적층 후 직접 볼 수 없다.
   - 정적(G2): 아래 단추 위치 = 배치 후 정지 상태 측정값. 적층 후 위 단추 중심과 비교한다.
   - 동적(G6): 아래 단추 위치 = 가림 직전까지의 track으로 추정한 후퇴 시점 위치. 이 값에는 예측 오차가 섞이므로 "추정 중심 오차"로 표기하고, 정적 측정값과 구분해 보고한다.
4. 자동 판정과 관찰자 판독의 혼동행렬을 G2부터 기록해 verifier 신뢰도를 공개한다.

## 7. 시간 예산

```
T_available = (x_exit − x_now) / v_button
필요 시간   = T_pick2 + T_transit + T_sync + T_descend + T_release(p95) + T_clear + margin
```

- `x_exit`: 카메라 관측 구간과 로봇 안전 도달 구간(후퇴 경로 포함)의 교집합 하류 끝.
- PRECHECK, PICK_SECOND 직전, PLAN_INTERCEPT에서 매번 판정하고 부족하면 시도하지 않는다.
- 예: 교집합 길이 300 mm, 20 mm/s → 15 s. 두 번째 파지·이동에 10 s가 걸리면 5 s가 남는다(예시 수치, G0 측정으로 대체).

## 8. 개발 단계별 패키지 적용 (`progress_plan.md`의 게이트와 공통 ID)

| 게이트 | 개발 목표 | 진행 계획 절 | 우선 구현 패키지 |
|---|---|---|---|
| G0 | 제어 계약·그리퍼·안전·현장 치수 확인 | 1, 4(그리퍼), 11(중단 규칙) | `button_robot_control`(adapter), `button_stacking_interfaces` 일부 |
| G1 | D455 검출·보정 | 3, 4 | `button_vision`, `button_calibration`, `button_stacking_bringup` |
| G2 | 정적 파지·정지 벨트 적층 | 4, 5 | `button_robot_control`, `button_task_manager`(최소), verifier |
| G3 | 추적·속도 추정·관측 TF·지연 측정 | 6, 7, 8(지연) | `button_tracking` |
| G4 | 예측 타이밍 적층(MODE_PREDICTED) | 8, 10A | `button_stacking_planner`, `dynamic_place` |
| G5 | 비접촉 동기 추종 | 9 | `tracking_controller` |
| G6 | 동적 적층(MODE_SYNCHRONIZED) | 10B | 전체 |
| G7 | 통합·복구·비교 평가 | 11, 12 | `button_task_manager`, `tools/eval` |
| 병행 | 가상 환경 | 2 | `button_stacking_sim` |

`button_stacking_interfaces`는 G0에서 `TaskStatus`와 어댑터에 필요한 것부터 만들고, 나머지는 해당 게이트에서 추가한다.

핵심 흐름은 `Vision → Tracking → Planning → Robot Control`이며 Task Manager가 순서·중단·판정을 관리한다.
