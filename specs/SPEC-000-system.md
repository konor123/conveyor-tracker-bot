# SPEC-000 system

- 버전: v0.8
- 근거: PLAN.md 0.2, 1, 2.3~2.5절
- REQ 영역: SYS

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.8 | 2026-11-14 | 종료 신호 처리를 `vision_node`·`static_frames_publisher`·`camera_tf_publisher`로 넓힘(REQ-SYS-007, 사본 7개) |
| v0.7 | 2026-11-13 | 노드 진입점의 종료 신호 처리(4.5절, REQ-SYS-007) |
| v0.6 | 2026-11-12 | TC-SYS-003-01·02를 실제 `vision_node`·`tracker_node` 계약 시험으로 검증(epoch 규모 ns 자리 stamp). tracker는 관측 프레임의 입력 stamp를 그대로 발행한다 |
| v0.5 | 2026-11-11 | TF 사용 범위를 정적 검사로 고정(REQ-SYS-006) |
| v0.4 | 2026-11-09 | 설정 파일 경로 규칙(상대 경로 = 저장소 루트 기준, 4.4절, REQ-SYS-005) |
| v0.3 | 2026-11-09 | 현장 정지 pick-and-place 실행기 `static_pick_place` 추가(11개 패키지), 로봇 지령 주체 예외 범위 |
| v0.2 | 2026-11-09 | M0 검증 범위·게이트 및 독립 계약 기대값 정합성 |
| v0.1 | 2026-11-07 | 패키지 구성·이름 규칙(10개), TF 트리·좌표계, 시각·시계 규약, SI 단위·단위 변환 어댑터·설정 키 단위 접미사(10종) |

## 1. 목적·범위

이동 병뚜껑 공깃돌 투입 시스템(`conveyor-tracker-bot`) 전체에 공통으로 적용되는 시스템 규약을 정의한다.

- 범위: 워크스페이스 패키지 구성과 이름 규칙, 좌표계 및 TF 트리, 관측·추적·계획 타임스탬프와 시계 분리, 프로젝트 내부 SI 단위 규약, 장비 경계 단위 변환 어댑터 분리, 설정·시나리오 YAML 수치 키의 단위 접미사 규칙.
- 범위 밖: 개별 메시지·서비스·액션 필드 정의와 QoS(SPEC-010), 오케스트레이터 상태 머신·단계 판정(SPEC-100), 중단 등급·제한시간·감시 타이머(SPEC-110), 카메라·평면·TCP 보정 절차(SPEC-600).

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4(`src/drop_interfaces/`) 기준.

- 입력 설정 파일: 각 패키지 `src/*/config/*.yaml`, 대역 시나리오 `src/drop_mocks/scenarios/*.yaml`.
- 주요 좌표계(`frame_id`):
  - `ObjectDetectionArray.header.frame_id = "robot_base"`
  - `TrackedObject.header.frame_id = "conveyor_frame"`, `TrackedObjectArray.header.frame_id = "conveyor_frame"`
  - `BeltMotion.header.frame_id = "conveyor_frame"`
  - `InterceptPlan.header.frame_id = "robot_base"`
- TF 방송(`/tf_static`, `/tf`):
  - 고정 좌표계: `world → robot_base → {conveyor_frame, mat_frame, camera_link → camera_color_optical_frame}`
  - 로봇 링크: `robot_base → ... → tcp` (`robot_state_publisher`)
  - 관측 동적 TF: `conveyor_frame → cap_<track_id>` (`tracker_node`, 시각화 전용)

## 3. 파라미터(단위·기본값·근거)

### 3.1 프로젝트 내부 물리 단위 규약

프로젝트 내부 코드, 순수 Python 판단·계산 모듈, ROS 메시지(`drop_interfaces`)는 모두 **SI 단위와 quaternion**만 사용한다.

| 물리량 | 내부 단위 | 비고 |
|---|---|---|
| 길이·위치·오차 | `m` | 픽셀 좌표(`u`, `v`, `diameter_px`)는 영상 평면 `[px]` |
| 시간·주기·지연 | `s` | 주파수는 `Hz` |
| 선속도 | `m/s` | |
| 선가속도 | `m/s^2` | |
| 각도 | `rad` | |
| 3차원 자세(회전) | `quaternion (x, y, z, w)` | `geometry_msgs/Pose`, `Transform` |
| 그리퍼 제어기 레지스터 | `pulse` (0~1150), `mA` | 하드웨어·레지스터 경계 값만 예외 |

### 3.2 설정·시나리오 YAML 수치 키 단위 접미사

`src/*/config/*.yaml` 및 `src/*/scenarios/*.yaml` 안의 모든 수치 키(정수·부동소수점 스칼라 및 수치 리스트, `bool` 제외)는 아래 10개 단위 접미사 중 하나로 끝나야 한다.

| 접미사 | 물리량 | 단위 | 예시 키 |
|---|---|---|---|
| `_m` | 길이·거리·위치·높이·표준편차 | m | `d_stone_min_m`, `e_xy_max_m`, `safe_z_m` |
| `_s` | 시간·제한시간·창 길이·유효기간 | s | `timeout_s`, `hold_s`, `heartbeat_timeout_s` |
| `_mps` | 선속도 | m/s | `speed_mps`, `v_rel_max_mps` |
| `_mps2` | 선가속도 | m/s² | `a_max_mps2` |
| `_rad` | 각도·시야각·요(yaw) | rad | `conveyor_yaw_rad`, `hfov_rad` |
| `_hz` | 주파수·발행 주기 | Hz | `frame_rate_hz` |
| `_ma` | 전류 | mA | `grip_current_min_ma` |
| `_pulse` | 그리퍼 위치 펄스 | pulse | `open_pulse`, `close_pulse` |
| `_count` | 개수·횟수·해상도(픽셀 수) | 무차원 정수 | `observation_count_min_count`, `width_count` |
| `_ratio` | 비율·신뢰도·배율 | 무차원 | `confidence_min_ratio`, `drop_ratio` |

- 접미사가 없는 무차원 수치 키(예: `seed`)는 해당 YAML 파일 최상단 `unitless: [<키 또는 점 경로>, ...]` 목록에 명시한다.
- 시나리오 파일의 `measured:` 블록(실측 출처 문자열 매핑)은 수치 키 검사 대상에서 제외한다.

### 3.3 단위 변환 어댑터 제한

외부 장비 고유 단위와 내부 SI 단위 사이의 변환은 다음 두 어댑터 모듈에서만 수행한다(PLAN.md 0.2-1절).

| 변환 대상 | 외부 표현 | 내부 표현 | 유일한 허용 모듈 |
|---|---|---|---|
| Doosan 로봇 위치·자세 | `mm`, `deg`, Euler(ZYZ) | `m`, `rad`, `quaternion (x, y, z, w)` | `src/robot_control/robot_control/motion_adapter.py` |
| RH-P12-RN(A) 그리퍼 간격 | `pulse` (0~1150) | 핑거 간격 `m` | `src/robot_control/robot_control/gripper_adapter.py` |

## 4. 동작(상태·알고리즘)

### 4.1 패키지 구성과 이름·구조 규칙 (PLAN.md 2.1~2.4절)

1. 워크스페이스는 다음 11개 패키지로 구성한다:
   `drop_interfaces`, `vision`, `tracking`, `intercept_planner`, `robot_control`, `orchestrator`, `calibration`, `drop_mocks`, `drop_bringup`, `drop_sim`, `static_pick_place`.
2. 모든 패키지 이름은 ROS 2 Jazzy underlay(`/opt/ros/jazzy`)의 기본 패키지와 충돌하지 않아야 한다.
3. 검출, 칼만 필터, 미래 위치 예측, 시간 예산, 단위 변환, 상태 전이, 단계 성공 판정, 중단 등급 판정은 ROS와 무관한 순수 Python 모듈로 작성하여 단위 시험(U)으로 검증하고, ROS 노드는 이를 감싸는 얇은 어댑터로만 동작한다(PLAN.md 0.2-6절).
4. 지휘 규칙(PLAN.md 2.2절):
   - 작업 순서와 서비스/액션 호출은 `orchestrator_node`에서만 나가며, 기능 노드끼리 서로의 서비스/액션을 호출하지 않는다.
   - 연속 데이터(`objects/detections → objects/tracked → drop/intercept_plan`)는 토픽 파이프라인으로 전달한다.
   - 로봇과 그리퍼에 지령을 내리는 주체는 `robot_controller_node` 하나뿐이다. 예외: `static_pick_place`의 `pick_place` 실행기는 `robot_controller_node`에 실기 backend가 생기기 전까지 쓰는 `[OFF]` 현장 도구다. 사람 입회에서만 `--execute`로 실행하고, `robot_controller_node`·오케스트레이터와 동시에 띄우지 않는다.

### 4.2 좌표계와 TF 트리 (PLAN.md 0.2-3, 2.5절)

```
world
 └── robot_base (base_link, 드라이버/URDF 명칭에 맞춤)
      ├── conveyor_frame        (static: 벨트 장치 고정, x축 = 벨트 진행 방향, z축 = 벨트 표면 법선)
      ├── mat_frame             (static: 매트 평면 기준)
      ├── camera_link → camera_color_optical_frame   (static: eye-to-hand 보정 결과 / 광학 TF는 RealSense 드라이버 소유)
      ├── ... robot links → tcp (dynamic: robot_state_publisher 소유)
      └── conveyor_frame → cap_<track_id>   (dynamic: tracker_node 발행, stamp = t_state, 시각화 전용)
```

- `conveyor_frame`은 벨트 표면을 따라 움직이지 않고 컨베이어 장치에 고정된다(`x`축: 벨트 진행 방향, `z`축: 벨트 표면 위쪽 법선, 오른손 좌표계).
- `cap_<track_id>`의 회전(`orientation`)은 항상 단위 사원수(`x=0, y=0, z=0, w=1`)로 발행하며, 원형 병뚜껑의 yaw는 관측하지 않고 파지 자세와 구분한다.
- TF와 예측의 분리: TF(`tf2`)는 고정 좌표 변환과 **현재까지 관측된** 병뚜껑 위치 시각화에 한정한다. `tf2`로 미래 시각을 외삽하지 않으며, 미래 위치 예측은 `intercept_planner`가 `TrackedObject` 상태·속도·공분산으로 계산해 `InterceptPlan`과 RViz `Marker`(`drop/predicted_marker`)로만 내보낸다.
- 단일 발행 원칙: 동일한 parent→child TF 프레임 쌍을 두 개 이상의 노드가 중복 발행하지 않는다.

### 4.3 시각과 시계 규약 (PLAN.md 0.2-2절, SPEC-110 5절)

- **관측 시각 보존:** `ObjectDetectionArray.header.stamp`는 카메라 **영상 노출 시각**(`sensor_msgs/Image.header.stamp`)을 그대로 복사한다. 노드가 영상을 수신하거나 처리를 끝낸 시각으로 덮어쓰지 않는다.
- **추적·계획 시각 분리:**
  - `TrackedObject.header.stamp` 및 `TrackedObjectArray.header.stamp`는 칼만 필터 상태의 기준 시각 `t_state`다. 검출을 받은 프레임에서는 입력 `ObjectDetectionArray.header.stamp`를 그대로 쓴다(float 초에서 되살리면 epoch 규모에서 ns 자리가 바뀐다).
  - 마지막으로 실제 검출이 연관된 시각 `last_observed_stamp`(`t_last_obs`)와 추정 유효 만료 시각 `valid_until`을 별도 필드로 둔다.
  - `InterceptPlan`은 계획 생성 시각(`header.stamp`), 입력 상태 시각(`source_state_stamp`), 목표 해제 시각(`release_time`), 계획 만료 시각(`valid_until`)을 별도 필드로 구분한다.
- **시계 이원화:** 메시지 타임스탬프, `valid_until`, 관측 신선도(`t_now - t_last_obs`), 시간 예산, 단계·동적·사이클 `deadline`은 ROS 시각(`use_sim_time` 적용 대상)을 쓰고, heartbeat 감시·그리퍼 응답 시간·호출/취소/중단 경로 제한시간·사이클 steady 상한은 단조 증가(steady) 시계를 쓴다.

### 4.4 설정 파일 경로 규칙

노드 파라미터·launch 인자·실행기 설정에 들어가는 파일·폴더 경로(보정 파일, 평면 파일, 검출 설정, 기록 폴더, 시나리오 파일 등)가 상대 경로면 **저장소 루트 기준**이다. 실행 위치나 설정 YAML의 위치에 따라 달라지지 않는다. 저장소에 넣는 현장 설정은 절대 경로를 쓰지 않는다.

- 저장소 루트: 환경 변수 `DROP_REPO_ROOT` → 모듈 위치에서 위로 올라가며 `src`와 `records` 폴더를 함께 가진 첫 폴더(소스 실행·colcon install 모두 워크스페이스 = 저장소 루트 아래) → 현재 작업 폴더 순으로 정한다.
- `~`는 홈 폴더로, 절대 경로는 그대로, 빈 값은 "지정 안 함"으로 둔다.
- 원본 구현은 `calibration.repo_paths`다. calibration에 의존하지 않는 `orchestrator`·`drop_mocks`는 같은 규칙의 `repo_path`를 두고 같은 시험으로 확인한다.
- 명령줄 인자로 주는 설정 파일 자체의 경로(`--config`, `--params-file`)는 셸 관례대로 실행 위치 기준이다. launch 인자 `vision_params_file`은 이 규칙을 따른다.

### 4.5 노드 진입점의 종료 처리

launch는 끝날 때 노드 프로세스에 SIGINT를 보내고(셸·시험이 함께 보내면 두 번 이상 받는다), 제한시간 뒤 SIGTERM을 보낸다. 다음 진입점은 이 신호로 생긴 예외를 traceback 없이 종료 코드 0으로 끝낸다: `drop_mocks.ros_common.spin_mock`, `orchestrator_node`, `safety_supervisor_node`, `record_logger_node`, `tracker_node`, `intercept_planner_node`, `robot_controller_node`, `vision_node`, `calibration.static_frames`의 `main`과 `calibration.ros_nodes.camera_tf_publisher`.

- `rclpy.init()` 전에 `ShutdownGuard`를 만든다. 패키지 사이 의존을 늘리지 않도록 같은 내용의 `node_shutdown.py`를 `drop_mocks`·`orchestrator`·`tracking`·`intercept_planner`·`robot_control`·`vision`·`calibration`에 둔다. rclpy는 init 때 기존 SIGINT 처리기를 저장해 자기 처리기(context 종료) 안에서 이어 부르므로, init 뒤에 처리기를 바꾸면 rclpy의 context 종료가 빠져 대기 중인 executor가 깨지 않는다.
- 첫 SIGINT만 `KeyboardInterrupt`로 올리고, 그 뒤의 SIGINT는 세기만 한다. 정리(`destroy_node`) 도중의 SIGINT가 traceback을 남기지 않는다.
- rclpy가 받은 메시지를 Python 객체로 바꾸는 중에 `KeyboardInterrupt`가 나면 pybind11이 이를 `RuntimeError: Unable to convert call argument '0' to Python object`로 바꾸고 원래 예외를 남기지 않는다(rclpy 7.1 `Subscription::take_message`). SIGINT를 받은 뒤의 `RuntimeError`는 종료로 본다. SIGINT 전의 `RuntimeError`, `SystemExit`, 그 밖의 예외는 그대로 올린다.
- 정리(`destroy_node`·`try_shutdown`)를 마치면 `guard.close()`로 SIGINT를 무시한다. Python은 종료 단계(`Py_FinalizeEx` → `_PySignal_Fini`)에서 Python 처리기가 걸린 신호를 기본 처리로 되돌리므로, 그 뒤 도착한 SIGINT(launch가 늦게 보낸 것)에 traceback 없이 종료 코드 −2로 끝난다. 무시(SIG_IGN)는 되돌리지 않는다.
- 노드 생성도 같은 처리 안에서 한다. 대역의 동시 기동 거부(`REAL_NODE_PRESENT`, SPEC-700 4.7절)는 종료 신호가 없을 때의 `RuntimeError`라 종료 코드 1을 유지한다.
- 대상이 아닌 진입점: `static_pick_place`(단계마다 Enter 확인을 받는 정적 투입 CLI, 실기 검증 경로), `calibration.ros_nodes.check_camera_info`(일회성 CLI), `robot_control.belt_sync_preview`. `camera_tf_publisher`는 노드가 낸 `SystemExit` 종료 코드를 그대로 돌려준다.

## 5. 시간 제약(주기·제한시간)

| 항목 | 주기 / 제한시간 | 시계 | 근거 |
|---|---|---|---|
| 카메라·검출·추적 데이터 토픽 | 30 Hz (프레임 주기 `1 / frame_rate_hz`) | ROS | PLAN.md 3.3절 |
| `system/heartbeat`, `task/status` | 10 Hz | steady / ROS | PLAN.md 3.3절, SPEC-010 |
| 단위 시험(U) 1건 실행 시간 | ≤ 1.0 s (`FakeClock` 사용, `time.sleep` 금지) | — | TEST_SPEC.md 0.4절 |

## 6. 실패 모드와 응답(중단 등급)

- 패키지 이름 충돌(`REQ-SYS-001` 위반), YAML 단위 접미사 누락(`REQ-SYS-002` 위반), 어댑터 외 단위 변환 코드(`REQ-SYS-004` 위반)는 정적·단위 시험(U)에서 즉시 실패로 검출하여 게이트 통과를 차단한다.
- 런타임 중 관측·추적 타임스탬프가 역행(`CLOCK_JUMP`)하거나 동적 구간에서 관측 신선도(`t_now - t_last_obs > obs_freshness_s`)가 초과되면 오케스트레이터가 SPEC-110 4.3절에 따라 트랙·계획을 무효화하고 L2 또는 L3 중단으로 전이한다.

M0는 감시 판정 함수와 이벤트 입력의 U 검증 범위다. ROS 관측 신선도(G3/G4)·시각 역행 무효화(G3)·그리퍼(G0)·벨트 BELT_UNSTEADY 이벤트(G3)의 노드 연결은 해당 게이트 필수 조건이며 실제 경로 사용 전에 C/I 시험을 통과한다. 연결 소유자는 orchestrator_node._watch와 tracking/reset·planner 무효화 경로이며 supervisor의 독립 L4 역할은 유지한다.

## 7. 성공 판정(측정 가능)

- REQ-SYS-001: 워크스페이스를 source하지 않은 깨끗한 underlay(`/opt/ros/jazzy`) 환경에서 `src/*/package.xml`의 11개 패키지 각각에 대해 `ros2 pkg prefix <name>`을 실행하면 11개 모두 `"not found"`(종료 코드 ≠ 0, 충돌 0건)를 반환한다.
- REQ-SYS-002: `src/*/config/*.yaml`과 `src/*/scenarios/*.yaml`의 모든 수치(스칼라·수치 리스트, `bool` 제외) 키는 파일 내 `unitless:` 목록(또는 `measured:` 출처 블록)에 명시된 경우를 제외하면 `_m`, `_s`, `_mps`, `_mps2`, `_rad`, `_hz`, `_ma`, `_pulse`, `_count`, `_ratio` 10종 중 하나로 끝나며 위반 건수는 0건이다.
- REQ-SYS-003: 처리 지연(50~80 ms)이 주입되어도 `ObjectDetectionArray.header.stamp`는 입력 영상 노출 시각 `T`와 정확히 일치(오차 0 ns)하고, `TrackedObject.header.stamp`는 상태 시각 `t_state == T`와 정확히 일치(오차 0 ns)한다.
- REQ-SYS-004: `src/**/*.py`(`build`·`install`·`test` 제외)에서 단위 변환 토큰 패턴(`* 1000`, `/ 1000`, `*= 1000`, `/= 1000`, `1000 *`, `math.degrees`, `math.radians`)은 `motion_adapter.py`와 `gripper_adapter.py` 두 파일에만 존재하며 그 외 파일에서의 검출 건수는 0건이다.
- REQ-SYS-005: 설정 파일 경로의 상대 경로는 4.4절 규칙으로 저장소 루트 기준 절대 경로가 되며, 실행 위치를 바꿔도 같은 파일을 가리킨다. `DROP_REPO_ROOT`가 있으면 그 폴더를 루트로 쓴다.
- REQ-SYS-006: `src/**/*.py`(`build`·`install`·`test` 제외)에서 TF 조회(`lookup_transform`·`can_transform`·`TransformListener`)는 `calibration/ros_nodes.py`(고정 카메라 TF 합성, 시각 `Time()` = 최신 고정값) 한 곳에만 있고, `TimePointZero`는 0건이다. TF 발행은 `/tf`가 `tracking/tracker_node.py`, `/tf_static`이 `calibration`(`ros_nodes.py`, `static_frames.py`)뿐이다. 예측·계획·지령(`tracking`의 판단 모듈, `intercept_planner`, `robot_control`, `orchestrator`)은 TF가 아니라 메시지 시각으로 계산한다(4.2절).
- REQ-SYS-007: 4.5절 진입점은 (a) spin 중 받은 메시지를 바꾸는 도중의 SIGINT, (b) 정리 도중의 두 번째 SIGINT, (c) 깨어날 일이 없는 대기 중의 SIGINT·SIGTERM, (d) 정리를 마친 뒤 프로세스가 끝날 때까지 이어지는 SIGINT에서 stderr에 traceback 없이 종료 코드 0으로 끝나고, (c)는 신호 뒤 3 s 안에 끝난다. 진입점은 `rclpy.init()` 전에 `ShutdownGuard`를 만들고 `rclpy.try_shutdown()` 뒤마다 `guard.close()`를 부르며 `KeyboardInterrupt`·`ExternalShutdownException`을 이름으로 잡지 않고, `node_shutdown.py` 사본 7개는 바이트 단위로 같다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- `TC-SYS-001-01`, `TC-SYS-002-01`, `TC-SYS-004-01`: 수준 U, 게이트 M0 (`tools/spec/test_sys_conventions.py`).
- `TC-SYS-003-01`: 수준 C, 게이트 G1 (`vision_node` 계약 시험, `src/vision/test/contract/test_vision_node_contract.py`).
- `TC-SYS-003-02`: 수준 C, 게이트 G3 (`tracker_node` 계약 시험, `src/tracking/test/contract/test_tracker_node_contract.py`).
- `TC-SYS-006-01`: 수준 U, 게이트 G3 (`tools/spec/test_tf_usage.py`).
- `TC-SYS-007-01`: 수준 C, 게이트 G7 (`src/drop_mocks/test/test_node_shutdown.py`, 진입점과 같은 모양의 탐침 프로세스).
- `TC-SYS-007-02`: 수준 U, 게이트 G7 (`tools/spec/test_node_shutdown.py`, 진입점·사본 정적 검사).

## 9. 미결 사항

- 고정 TF 프레임(`conveyor_frame`, `mat_frame`, `camera_link`, `tcp`)의 실기 보정 수치: TBD(현장 보정 OFF-3 / G1, SPEC-600).
