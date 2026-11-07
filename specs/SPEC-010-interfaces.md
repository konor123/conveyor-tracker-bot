# SPEC-010 interfaces

- 버전: v0.2
- 근거: PLAN.md 2.2, 3, 7.4절, `src/drop_interfaces/`
- REQ 영역: IF

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-11-07 | msg 9종·srv 6종·action 6종 규약, 공통 요청 필드·결과 코드·상수 표, QoS 표, revision·epoch 규칙, lifecycle 비활성 응답, Heartbeat 오류 코드, 스냅샷 버전 규칙 |
| v0.2 | 2026-11-07 | `RobotStatus` msg 신설(`robot/status`, 10 Hz), `PlaceStone`·`DynamicPlace` feedback에 `phase_done`·`metric_keys`·`metric_values` 추가 및 phase별 측정값 키 규약(REQ-IF-005) |

## 1. 목적·범위

`drop_interfaces` 패키지에 정의된 메시지(10종), 서비스(6종), 액션(6종)의 의미·단위·상수 규약과 노드 간 토픽/서비스/액션 이름, QoS 프로파일, `epoch`·`revision` 처리 규칙, Managed Lifecycle 비활성 상태에서의 응답 계약, 인터페이스 스냅샷 버전 관리 규칙을 정한다.

필드 선언의 정확한 원문은 `src/drop_interfaces/{msg,srv,action}/` 및 `specs/snapshots/*.txt`가 원본이며, 이 문서는 필드 전체를 중복 나열하지 않고 단위·의미·동작 계약 중심으로 기술한다.

## 2. 입력·출력(interface 버전)

### 2.1 데이터 및 상태 메시지 (`src/drop_interfaces/msg/`, 10종)

| 메시지 | 토픽(상대명) | `header.frame_id` / `stamp` 규약 | 주요 필드 단위·의미 요약 |
|---|---|---|---|
| `ObjectDetection` | (`ObjectDetectionArray` 원소) | 배열 `header` 기준 | 프레임 내 `detection_id`(추적 ID 아님), `region`, `object_class`, 왜곡 보정 전 중심 `u, v, diameter_px` [px] 및 `center_cov_px` [px², 2×2 row-major], `confidence` [0~1], 보정 평면 교차 `position` [m] 및 `position_cov` [m², 3×3 row-major], `position_valid`, 로봇 가림 `occluded_partial`, 병뚜껑 내부 공깃돌 유무 `stone_inside` 및 판정 가능 여부 `stone_inside_valid`(`CLASS_CAP` 전용, `false`면 `stone_inside` 무시) |
| `ObjectDetectionArray` | `objects/detections` | `robot_base` / **영상 노출 시각** | 프레임 일련번호 `image_seq`, 적용된 `calibration_id`, `detections[]` |
| `TrackedObject` | (`TrackedObjectArray` 원소) | `conveyor_frame` / 상태 시각 `t_state` | `track_id`, 리셋 세대 `epoch`, `object_class`, `status`(`OBSERVED`/`COASTING`/`LOST`), 마지막 관측 시각 `last_observed_stamp`, 누적 `observation_count`, `position` [m], `velocity` [m/s], `state_cov` (`[x,y,vx,vy]` 4×4 row-major), 예측 만료 시각 `valid_until` |
| `TrackedObjectArray` | `objects/tracked` | `conveyor_frame` / `t_state` | `tracks[]` (배열 내 모든 트랙의 `header.stamp`는 배열 `header.stamp`와 일치, `valid_until >= header.stamp`) |
| `BeltMotion` | `belt/measured_motion` | `conveyor_frame` / 추정 시각 | 진행 단위 벡터 `direction`, 벨트 속력 `speed` [m/s], 표준편차 `speed_std` [m/s], 추정 창 `window_sec` [s], 추정원 `source`, 정상 상태 여부 `steady`(속도 변화 미검출 시 `true`), `last_change_stamp` |
| `InterceptPlan` | `drop/intercept_plan` | `robot_base` / 계획 생성 시각 | `plan_id`, 동일 계획 내 단조 증가 `revision`, 대상 `target_track_id`·`target_epoch`, 입력 상태 시각 `source_state_stamp`, 목표 해제 시각 `release_time`, 계획 유효 만료 `valid_until`, 파지 오프셋이 반영된 해제 직전 목표 `target_tcp_pose` [m, quaternion], 동기화 선속도 `target_velocity` [m/s], 예측 표준편차 `position_std` [m], 여유 시간 `time_margin` (`T_available - T_required` [s]), `status`, `reason` |
| `RobotStatus` | `robot/status` | `robot_base` / 상태를 읽은 시각 | 제어기 통신 `connected`, 지령 가능 `ready`(서보 on, 오류·보호정지 없음), `protective_stop`, `tcp_pose` (`robot_base` [m, quaternion]), 투입 작업영역 진입 `in_workspace`, 파지 여부 `holding`, 상주 루프 DRL 생존 `gripper_alive`, 마지막 그리퍼 명령 응답 시간 `gripper_response_s` [s], RH-P12-RN(A) Hardware Error `gripper_error_bits`(0=정상), 개방 위치 `gripper_open`, 핑거 간격 `gripper_width` [m], Present Current `gripper_current` [mA] |
| `TaskStatus` | `task/status` | — / 발행 시각 | `cycle_id`, 상태 머신 상태명 `state`, `abort_level`(0=없음, 1~4), `error_code`, 지령 소유자 `command_owner`, `robot_ready`, `protective_stop`, `gripper_holding`, 남은 시간 `time_remaining` [s] |
| `Heartbeat` | `system/heartbeat` | — / 발행 시각 | `node_name`, `lifecycle_state`(`lifecycle_msgs/State` ID), 단조 증가 일련번호 `seq`, `error_code`(3.4절), `detail` |
| `StepRecord` | `task/step_record` | — / 기록 시각 | `cycle_id`, `step_seq`, `step_name`, `t_start`, `t_end`, 공통 결과 코드 `result`, `reason_code`, 시도 번호 `attempt`, `abort_level`, 판정 지표 `metric_keys[]`·`metric_values[]` |

### 2.2 지휘용 서비스 (`src/drop_interfaces/srv/`, 6종)

모든 서비스 요청은 3.1절 공통 요청 필드(`cycle_id`, `step_id`, `deadline`)로 시작하고, 모든 서비스 응답은 3.2절 공통 결과 코드 상수(`OK=0` ~ `FAULT=5`)와 `result`, `reason_code`를 포함한다.

| 서비스 | 이름(상대명) | 서버 노드 | 요청 주요 필드(단위) | 응답 주요 필드(단위) |
|---|---|---|---|---|
| `DetectObjects` | `vision/detect_objects` | `vision_node` | `region`, `object_class`, `min_count`, 안정 관측 창 `stable_window` [s] | `detections[]`, 창 내 위치 표준편차 `position_std[]` [m] |
| `VerifyDrop` | `vision/verify_drop` | `vision_node` | `cap_track_id`, 유지 확인 시간 `hold_time` [s] | `verdict`(`SUCCESS`/`FAILURE`/`UNKNOWN`), `stone_inside`, 안착 오프셋 `stone_offset` [m], `stone_offset_valid` |
| `ComputeIntercept` | `planner/compute_intercept` | `intercept_planner_node` | `target_track_id`, `target_epoch`, `mode`, `earliest_release` | `plan`(`InterceptPlan`) |
| `ResetTracking` | `tracking/reset` | `tracker_node` | 초기화 영역 `region`(`REGION_UNKNOWN=0`이면 전체) | 증가된 `new_epoch` |
| `SelectTarget` | `tracking/select_target` | `tracker_node` | `object_class`(`CLASS_CAP`), `min_observation_count` | 도달 구간 진입 전 가장 먼저 들어올 `track_id`, `epoch`, 진입 예측 시각 `t_enter_window` |
| `Stop` | `robot/stop` | `robot_controller_node` | 호출 주체 `requester`(`ORCHESTRATOR`/`SUPERVISOR`), `reason_code` | 정지 확인 완료 여부 `stopped` |

### 2.3 액션 (`src/drop_interfaces/action/`, 6종)

- **외부 진입점 `DropTask` (`drop_task`, 서버: `orchestrator_node`):**
  - 외부 클라이언트(사용자·시험 스크립트)가 호출하는 최상위 액션이므로 공통 요청 필드(`cycle_id`, `step_id`, `deadline`)를 두지 않는다.
  - Goal: `mode`(`MODE_STATIC`/`MODE_PREDICTED`/`MODE_SYNCHRONIZED`), 허용 최대 벨트 속도 `max_belt_speed` [m/s], 사이클 제한시간 `timeout` [s], 통과 허용 횟수 `max_passes`.
  - Result: 최종 판정 `result`(`SUCCESS=0` ~ `NOT_ATTEMPTED=6`), `reason_code`, 해제 순간 XY 오차 `release_error` [m] 및 `release_error_valid`, 안착 후 중심 오프셋 `stone_offset` [m] 및 `stone_offset_valid`, 유지 시간 `hold_time` [s], 총 소요 `cycle_time` [s], `passes_skipped`.
  - Feedback: 현재 상태명 `state`, `plan_id`, `time_remaining` [s], `tracking_std` [m].
- **로봇 스킬 액션 5종 (서버: `robot_controller_node`):**
  - 모두 3.1절 공통 요청 필드(`cycle_id`, `step_id`, `deadline`)로 시작하며, Result는 3.2절 공통 결과 코드(`result`, `reason_code`)를 포함하고, Feedback은 현재 세부 실행 단계명 `phase`(비어 있지 않은 문자열)를 포함한다.

| 액션 | 이름(상대명) | Goal 주요 필드(단위) | Result 추가 필드(단위) | Feedback 필드(단위) |
|---|---|---|---|---|
| `PickStone` | `robot/pick_stone` | `target_position` (`robot_base` [m]), `approach_height` [m], `grip_current_limit` [mA] | `grip_width` [m], `grip_current` [mA], `holding` | `phase` (`APPROACH`·`DESCEND`·`GRIP`·`LIFT`) |
| `MoveToPose` | `robot/move_to_pose` | `named_pose`(비어 있지 않으면 우선) 또는 `target_pose` (`robot_base` [m, quat]) | `pose_error` [m] | `phase` (`MOVE`) |
| `PlaceStone` | `robot/place_stone` | 정지 목표 `target_pose` [m, quat], `release_height` [m], `open_width` [m] | `released`, `release_error` [m] | `phase` (`APPROACH`·`DESCEND`·`RELEASE`·`RETREAT`), `phase_done`, `metric_keys[]`, `metric_values[]` |
| `DynamicPlace` | `robot/dynamic_place` | `plan_id`, `target_epoch`, `mode`, 해제 게이트 임계값(`e_xy_max` [m], `v_rel_max` [m/s], `tau_max` [s], `position_std_max` [m], `rim_clearance` [m]), `open_width` [m] | `released`, 해제 순간 게이트 측정값(`gate_e_xy` [m], `gate_v_rel` [m/s], `gate_t_since_obs` [s], `gate_position_std` [m]) | `phase` (`APPROACH`·`SYNCHRONIZE`·`DESCEND`·`RELEASE`·`RETREAT`), `e_xy` [m], `v_rel` [m/s], `plan_revision`, `phase_done`, `metric_keys[]`, `metric_values[]` |
| `SafeRetreat` | `robot/safe_retreat` | 동기 상승 여부 `keep_sync`, 매트 반납 여부 `place_back`, `place_back_pose` [m, quat] | `stone_placed_back` | `phase` (`RETREAT`, `place_back`이면 `PLACE_BACK`) |

#### 2.3.1 `PlaceStone`·`DynamicPlace` phase feedback 규약

- 각 phase가 시작될 때 `phase_done = false`인 feedback을 1회 이상 발행한다.
- 각 phase가 끝날 때 `phase_done = true`이고 아래 표의 키를 모두 채운(`metric_keys`와 같은 길이·순서의 `metric_values`, 불리언은 `1.0`/`0.0`) feedback을 발행한다.
- `DynamicPlace`의 `SYNCHRONIZE` 진행 중에는 20 Hz 이상으로 `e_xy`·`v_rel`을 채운(`phase_done = false`) feedback을 발행하며, `orchestrator_node`는 이를 `SyncSample` 열로 수집해 동기 판정에 사용한다.

| 액션 | `phase` | `phase_done = true`일 때 필수 `metric_keys` (`step_judges.py` 측정값 필드) |
|---|---|---|
| `DynamicPlace` | `APPROACH` | `plan_valid`, `pose_error_m` |
| `DynamicPlace` | `SYNCHRONIZE` (`MODE_SYNCHRONIZED`) | `e_xy_m`, `v_rel_mps` |
| `DynamicPlace` | `SYNCHRONIZE` (`MODE_PREDICTED`) | `entry_time_error_s`, `v_rel_mps` |
| `DynamicPlace` | `DESCEND` | `sync_ok`, `rim_clearance_m`, `height_error_m` |
| `DynamicPlace` | `RELEASE` | `open_width_m`, `sync_held` |
| `DynamicPlace` | `RETREAT` | `tcp_z_m`, `outside_workspace`, `stone_in_gripper` |
| `PlaceStone` | `APPROACH` | `plan_valid`, `pose_error_m` |
| `PlaceStone` | `DESCEND` | `target_error_m`, `rim_clearance_m`, `height_error_m` |
| `PlaceStone` | `RELEASE` | `open_width_m`, `sync_held` |
| `PlaceStone` | `RETREAT` | `tcp_z_m`, `outside_workspace`, `stone_in_gripper` |

## 3. 파라미터(단위·기본값·근거)

### 3.1 공통 요청 필드 규약

오케스트레이터가 기능 노드에 보내는 모든 서비스 Request(6종)와 로봇 스킬 액션 Goal(5종, 외부 진입점 `DropTask` 제외)은 반드시 다음 세 필드를 **처음 세 필드(순서 고정)**로 가진다.

| 순서 | 필드명 | 타입 | 의미 및 규약 |
|---|---|---|---|
| 1 | `cycle_id` | `string` | 현재 사이클 식별자 |
| 2 | `step_id` | `uint32` | 사이클 내 단계 일련번호 |
| 3 | `deadline` | `builtin_interfaces/Time` | ROS 시각 기준 제한시각. `sec == 0 && nanosec == 0`(0.0 s)이면 제한시간 없음 |

### 3.2 공통 결과 코드

모든 서비스 Response(6종), 로봇 스킬 액션 Result(5종), `StepRecord.result`는 동일한 결과 코드 체계를 따른다.

| 상수 | 값 | 의미 | 대표 `reason_code` 예시 |
|---|---|---|---|
| `OK` | 0 | 정상 완료 | `""` |
| `FAILED` | 1 | 시도했으나 판정·실행 조건 미달 | `NOT_ENOUGH_OBJECTS`, `NO_TARGET`, `PLAN_INVALID` |
| `TIMEOUT` | 2 | 요청 `deadline` 경과로 서버가 자체 중단 | `DEADLINE` |
| `PRECONDITION` | 3 | 선행조건 불충족으로 시도하지 않음 | `NODE_INACTIVE` |
| `CANCELED` | 4 | 클라이언트 cancel 또는 `Stop` 서비스로 중단 | `STOPPED`, `CANCELED` |
| `FAULT` | 5 | 하드웨어·통신 이상 또는 보호정지 | `PROTECTIVE_STOP` |

### 3.3 인터페이스 상수 정의표

| 인터페이스 | 상수 정의 |
|---|---|
| `ObjectDetection` | `REGION_UNKNOWN=0`, `REGION_MAT=1`, `REGION_BELT=2`, `CLASS_UNKNOWN=0`, `CLASS_STONE=1`, `CLASS_CAP=2`, `CLASS_BELT_MARKER=3` |
| `TrackedObject` | `OBSERVED=0`, `COASTING=1`, `LOST=2` |
| `BeltMotion` | `SOURCE_CAP=0`, `SOURCE_BELT_MARKER=1` |
| `InterceptPlan` | `OK=0`, `TIME_INSUFFICIENT=1`, `UNREACHABLE=2`, `UNCERTAIN=3`, `EXPIRED=4` |
| `VerifyDrop.Response` | `SUCCESS=0`, `FAILURE=1`, `UNKNOWN=2` |
| `Stop.Request` | `ORCHESTRATOR=0`, `SUPERVISOR=1` |
| `DropTask.Goal` | `MODE_STATIC=0`, `MODE_PREDICTED=1`, `MODE_SYNCHRONIZED=2` |
| `DropTask.Result` | `SUCCESS=0`, `FAILURE=1`, `UNKNOWN=2`, `CANCELED=3`, `SAFE_ABORT=4`, `PROTECTIVE_STOP=5`, `NOT_ATTEMPTED=6` |

### 3.4 `Heartbeat.error_code` 코드 표

| `error_code` | `detail` | 발행 노드 | 의미 (SPEC-700 4.5절 호환) |
|---|---|---|---|
| `0` | `""` | 모든 기능 노드 | 정상 |
| `100` | `"PROTECTIVE_STOP"` | `robot_controller_node` | 로봇 보호정지·비상정지 활성(L4 전이 조건) |

### 3.5 토픽·서비스·액션 QoS 표 (PLAN.md 3.3절)

| 이름(상대명) | 종류 / 인터페이스 | 발행·제공 주체 | 주기 | Reliability | Durability | History Depth |
|---|---|---|---|---|---|---|
| `camera/color/image_raw`, `camera_info` | topic (`sensor_msgs`) | RealSense | 30 Hz | Best Effort | Volatile | 1 |
| `objects/detections` | topic (`ObjectDetectionArray`) | `vision_node` | 영상 주기(30 Hz) | Reliable | Volatile | 5 |
| `objects/tracked` | topic (`TrackedObjectArray`) | `tracker_node` | 영상 주기(30 Hz) | Reliable | Volatile | 5 |
| `belt/measured_motion` | topic (`BeltMotion`) | `tracker_node` | 영상 주기(30 Hz) | Reliable | Volatile | 5 |
| `drop/intercept_plan` | topic (`InterceptPlan`) | `intercept_planner_node` | 상태 갱신마다 | Reliable | Volatile | 5 |
| `drop/predicted_marker` | topic (`visualization_msgs/Marker`) | `intercept_planner_node` | 10 Hz | Best Effort | Volatile | 1 |
| `robot/status` | topic (`RobotStatus`) | `robot_controller_node` | 10 Hz | Reliable | Volatile | 1 |
| `system/heartbeat` | topic (`Heartbeat`) | 모든 기능 노드 | 10 Hz | Reliable | Volatile | 1 |
| `task/status` | topic (`TaskStatus`) | `orchestrator_node` | 10 Hz + 상태 변경 시 | Reliable | Transient Local | 1 |
| `task/step_record` | topic (`StepRecord`) | `orchestrator_node` | 단계 종료마다 | Reliable | Volatile | 100 |
| 모든 srv·action (`vision/*`, `tracking/*`, `planner/*`, `robot/*`, `drop_task`) | srv / action | 각 서버 노드 | 요청 시 | 기본(Services/Actions) | Volatile | 기본 |

## 4. 동작(상태·알고리즘)

### 4.1 `deadline` 자체 중단 규칙 (PLAN.md 2.2절 4항, SPEC-700 5절)

- 요청의 `deadline`이 `0`(`sec == 0 && nanosec == 0`)이면 서버 측 제한시간을 두지 않는다.
- `deadline > 0`일 때:
  - 서비스 서버는 요청 처리 시작 시점(또는 완료 전)에 현재 ROS 시각 `t_ros >= deadline`이면 즉시 작업을 중단하고 `result = TIMEOUT(2)`, `reason_code = "DEADLINE"`으로 응답한다.
  - 액션 서버는 실행 루프 중 `t_ros >= deadline`에 도달하면 스스로 동작을 멈추고 Goal을 `result = TIMEOUT(2)`, `reason_code = "DEADLINE"`으로 종료한다(오케스트레이터 측 제한시간과 이중 안전장치).

### 4.2 `revision`·`epoch` 일관성 규칙 (PLAN.md 2.2절 2항)

- **트랙 `epoch`:** `tracker_node`는 기동 시 `epoch = 0`에서 시작하며, `ResetTracking` 서비스 호출 또는 트래커 리셋 시마다 `epoch`를 1씩 증가시켜 이전 세대 트랙과 계획을 무효화한다.
- **계획 `revision`:** `intercept_planner_node`는 활성 `plan_id`에 대해 새로운 관측으로 계획을 갱신할 때마다 `revision`을 단조 증가시켜 `drop/intercept_plan`으로 발행한다.
- **추종 제어기 검증:** 동기 추종(`DynamicPlace`) 중 `robot_controller_node`는 Goal에 지정된 `plan_id` 및 `target_epoch`와 일치하는 `InterceptPlan`만 적용하며, 이미 반영한 `revision` 이하(`plan.revision <= last_revision`)의 메시지는 버린다.
- **계획 무효화:** 트랙/계획의 `epoch`가 `target_epoch`와 달라지거나, `valid_until`이 만료(`t_ros > valid_until`)되거나, `InterceptPlan.status != OK`이면 `robot_controller_node`는 해당 Goal을 즉시 `result = FAILED(1)`, `reason_code = "PLAN_INVALID"`로 끝낸다.

### 4.3 Lifecycle 비활성 시 응답 계약 (PLAN.md 2.2절 5항, SPEC-700 4.6절)

모든 기능 노드(`vision_node`, `tracker_node`, `intercept_planner_node`, `robot_controller_node` 및 대응 대역 노드)는 Managed Lifecycle 노드로 구현하며, 현재 상태가 `ACTIVE`(`lifecycle_msgs/State.PRIMARY_STATE_ACTIVE == 3`)가 아닐 때 다음 규칙을 지킨다.

1. **데이터 토픽 미발행:** `objects/detections`, `objects/tracked`, `belt/measured_motion`, `drop/intercept_plan`, `robot/status` 등 기능 데이터·상태 토픽을 발행하지 않는다.
2. **서비스 호출 거부 응답:** 모든 서비스 요청에 대해 내부 동작을 수행하지 않고 `result = PRECONDITION(3)`, `reason_code = "NODE_INACTIVE"`로 즉시 응답한다.
3. **액션 Goal 거부:** 모든 액션 Goal 요청을 수락 콜백에서 거부(`accepted == False`)한다. 동시에 실행 중인 액션 Goal은 최대 1개로 제한하며(단일 지령 권한), 실행 중 새 Goal이 들어오면 거부한다.
4. **Heartbeat 유지:** 비활성(`UNCONFIGURED=1`, `INACTIVE=2` 등) 상태에서도 `system/heartbeat`는 10 Hz로 계속 발행하되, `lifecycle_state` 필드에 현재 상태 ID를 정확히 담아 내보낸다.

### 4.4 인터페이스 버전과 스냅샷 관리 (`specs/snapshots/`)

- `src/drop_interfaces/` 아래의 `.msg`, `.srv`, `.action` 정의를 수정할 때는 반드시 `SPEC-010-interfaces.md`의 버전을 올린다.
- 빌드 후 `tools/spec/snapshot_interfaces.sh`를 실행해 `specs/snapshots/*.txt` 스냅샷 파일(22개)을 갱신하고, 영향을 받는 대역(`drop_mocks`) 및 계약 시험(`tools/testing/contracts/`)을 같은 변경에서 함께 수정한다.
- `tools/spec/snapshot_interfaces.sh --check`는 현재 빌드된 `drop_interfaces`의 `ros2 interface show` 출력과 `specs/snapshots/*.txt`를 비교하여 차이가 있으면 종료 코드 1을 반환한다.

## 5. 시간 제약(주기·제한시간)

- `system/heartbeat` 및 `robot/status`: 10 Hz 발행(계약 시험 기준 `10 Hz ± 20%`, 즉 `8.0 ~ 12.0 Hz`, 인접 메시지 간격 `< 0.3 s`, heartbeat `seq` 엄격 단조 증가).
- `DynamicPlace` `SYNCHRONIZE` 중 feedback: 20 Hz 이상 발행.
- `robot_controller_node` 액션 취소(`cancel_goal_async`) 응답 시간: 요청 후 `0.5 s` 이내 `CANCELED` 종료.
- `robot_controller_node` `deadline` 자체 중단 지연: `deadline` 시각 경과 후 `0.15 s` 이내 `TIMEOUT` 종료.

## 6. 실패 모드와 응답(중단 등급)

- **QoS 불일치(`QOS_MISMATCH`):** Best Effort 발행자와 Reliable 구독자가 연결되면 ROS 2 / DDS는 예외를 던지지 않고 메시지를 조용히 폐기한다. 계약 시험 도구(`common.check_data_qos`, `common.check_qos_compatible`)는 토픽 그래프의 발행자 QoS를 조회해 `QOS_MISMATCH`로 즉시 실패시킨다.
- **스냅샷 불일치:** 승인되지 않은 인터페이스 변경은 `TC-IF-001-01`(`snapshot_interfaces.sh --check`)에서 실패한다.
- **런타임 오류 전파:** `NODE_INACTIVE`(`PRECONDITION`), `DEADLINE`(`TIMEOUT`), `PLAN_INVALID`(`FAILED`), `PROTECTIVE_STOP`(`FAULT` 및 `Heartbeat.error_code = 100`, `RobotStatus.protective_stop = true`)은 오케스트레이터와 안전 감시 노드로 전달되어 SPEC-110 중단 등급(L1~L4)을 구동한다.

## 7. 성공 판정(측정 가능)

- REQ-IF-001: `drop_interfaces`의 모든 msg(10종)·srv(6종)·action(6종)에 대한 `ros2 interface show` 출력이 `specs/snapshots/*.txt`(22개 파일)와 완전히 일치한다(`tools/spec/snapshot_interfaces.sh --check` 종료 코드 0, 차이 0건).
- REQ-IF-002: 외부 진입점 `DropTask`를 제외한 `drop_interfaces`의 모든 srv Request(6종)와 action Goal(5종) 생성 Python 클래스에서 `get_fields_and_field_types()`의 처음 세 필드 키는 순서대로 `('cycle_id', 'step_id', 'deadline')`이다(불일치 0건).
- REQ-IF-003: `ObjectDetection`, `TrackedObject`, `BeltMotion`, `InterceptPlan`, `VerifyDrop`, `Stop`, `DropTask` 및 공통 결과 코드(`OK=0` ~ `FAULT=5`)의 모든 상수 값이 3.2~3.3절(`TEST_SPEC.md` 3.2절) 명세와 100% 일치한다.
- REQ-IF-004: 데이터 토픽 발행자의 QoS는 3.5절 표(`Reliability=RELIABLE`, `Durability=VOLATILE`, `History depth=5` 또는 `1`)와 일치하며, 호환되지 않는 QoS 조합(Best Effort 발행자 + Reliable 구독자)은 계약 시험 검사(`check_qos_compatible`)에서 `QOS_MISMATCH` 오류로 100% 검출된다.
- REQ-IF-005: `PlaceStone`·`DynamicPlace`의 각 phase 끝 feedback은 `phase_done=true`이고 2.3.1절 표의 키를 모두 가진다(누락 0).

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- `TC-IF-001-01`, `TC-IF-002-01`, `TC-IF-003-01`: 수준 U, 게이트 M0 (`tools/spec/test_interfaces.py`).
- `TC-IF-004-01`, `TC-IF-004-02`: 수준 C, 게이트 M0 (`tools/spec/test_qos_contract.py` 및 `src/*/test/contract/`).
- `TC-IF-005-01`: 수준 C, 게이트 M0 (`tools/testing/contracts/robot_controller_contract.py` 및 `src/drop_mocks/test/contract/test_mock_robot_controller_contract.py`).

## 9. 미결 사항

- Doosan 드라이버 실시간 스트림 토픽(`servol_stream` / `speedl_stream` / RT 계열) 및 그리퍼 상태 읽기 레지스터 경로의 확정 인터페이스·QoS: TBD(측정 G0, SPEC-500).
- `Heartbeat.error_code`의 로봇·그리퍼 세부 오류 코드 확장(100 외 추가 코드): SPEC-500 작성 시 확정.
