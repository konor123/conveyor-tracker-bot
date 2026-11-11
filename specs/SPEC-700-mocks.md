# SPEC-700 mocks

- 버전: v0.8
- 근거: PLAN.md 7절
- REQ 영역: MCK, INT

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.8 | 2026-11-11 | REQ-MCK-008 대역의 Stop 정지 유지·작업자 해제 |
| v0.7 | 2026-11-10 | 실시간 실행 판정은 PC(LEGION 또는 officepc), CI는 실행·기록만 |
| v0.6 | 2026-11-10 | 실시간 데이터 수신 지연을 미들웨어 수신 시각으로 측정 |
| v0.5 | 2026-11-09 | 이벤트 프레임 위상과 실시간 최신 프레임 수신의 검증 기준 분리 |
| v0.4 | 2026-11-09 | M0 검증 범위·게이트 및 독립 계약 기대값 정합성 |
| v0.1 | 2026-11-07 | 시나리오 YAML 형식, `ScenarioEngine` 세계 모델, 이벤트 10종, `mock_world_node`·`mock_robot_controller_node` 동작, 가정값 표기 |
| v0.2 | 2026-11-07 | SPEC-010 v0.2 반영: `mock_robot_controller_node`의 `robot/status`(`RobotStatus`, 10 Hz) 발행, `PlaceStone`·`DynamicPlace` phase별 feedback `phase_done`·`metric_keys`·`metric_values` 및 `phase_metrics` 대본 덮어쓰기 |
| v0.3 | 2026-11-09 | 동시 기동 금지의 기동 뒤 감시(4.7절), heartbeat·`robot/status` 타이머의 steady 시계(5절) |

## 1. 목적·범위

`drop_mocks` 패키지의 시나리오 엔진(`scenario_engine.py`)과 ROS 대역 2종(`mock_world_node`, `mock_robot_controller_node`)을 정한다. 대역은 I0·C 시험에서 vision·tracker·robot_controller를 대신한다. 대역 결과로 정확도·성공률을 보고하지 않는다(PLAN.md 7.4절).

M0 planner 대역은 등속 해석해·`publish_plan`·`ComputeIntercept`를 제공하며 실제 planner가 계약 시험을 통과하면 사용하지 않는다. `time_insufficient` 시나리오, `test/contract/`의 역할별 계약 시험과 trial 가정값 복사는 M0 범위다. TCP 1차 지연 모델은 G5 범위다.

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4(`src/drop_interfaces/`) 기준. 이름은 PLAN.md 3.3절의 상대명이다.

| 노드 | role | 발행 | 서비스·액션 서버 | Heartbeat `node_name` |
|---|---|---|---|---|
| `mock_world_node` | `vision` | `objects/detections` | `vision/detect_objects`, `vision/verify_drop` | `vision_node` |
| `mock_world_node` | `tracking` | `objects/tracked`, `belt/measured_motion` | `tracking/reset`, `tracking/select_target` | `tracker_node` |
| `mock_world_node` | `all` | 위 둘 합집합 | 위 둘 합집합 | 둘 다 |
| `mock_robot_controller_node` | — | `robot/status` | `robot/pick_stone`, `robot/move_to_pose`, `robot/place_stone`, `robot/dynamic_place`, `robot/safe_retreat`(action), `robot/stop`(srv) | `robot_controller_node` |

모든 노드는 `system/heartbeat`를 10 Hz로 발행하고, `mock_robot_controller_node`는 활성 상태에서 `robot/status`(`RobotStatus`)를 10 Hz로 발행한다. QoS는 PLAN.md 3.3절·SPEC-010 3.5절 제안값을 쓴다.

노드 파라미터:

| 이름 | 기본값 | 의미 |
|---|---|---|
| `scenario` | `nominal_20mms` | 시나리오 이름(`share/drop_mocks/scenarios/<이름>.yaml`) 또는 YAML 경로 |
| `seed` | -1 | -1이면 시나리오 파일의 `seed` |
| `t0_s` | 0.0 | 시나리오 시각 0에 해당하는 ROS 시각 [s]. 0이면 노드 기동 시각. 역할별 프로세스는 같은 값을 받아야 한다 |
| `role` | `all` | `mock_world_node` 전용 |
| `autostart` | true | 기동 시 configure·activate까지 진행 |
| `assumed_params` | (읽기 전용) | 엔진이 낸 가정값 목록(REQ-MCK-006). 기록 노드가 파라미터로 읽는다 |

## 3. 파라미터(단위·기본값·근거)

### 3.1 시나리오 YAML

수치 키는 단위 접미사를 붙인다(TC-SYS-002-01). 접미사가 없는 수치 키는 파일 안 `unitless:` 목록에 둔다.

```yaml
name: nominal_20mms
seed: 0
unitless: [seed]
frame_rate_hz: 30.0
world:
  belt:    {speed_mps, lap_length_m, unsteady_hold_s, speed_std_mps, window_s}
  zones:   {obs_start_m, obs_end_m, reach_start_m, reach_end_m}     # conveyor_frame x
  frames:  {conveyor_xyz_m, conveyor_yaw_rad, mat_xyz_m, mat_yaw_rad}  # robot_base 기준
  camera:  {xyz_m, hfov_rad, width_count, height_count, pixel_std_ratio}
  objects: {stone_diameter_m, stone_height_m, cap_outer_diameter_m, cap_rim_height_m}
  noise:   {position_std_m}
  tracking: {valid_for_s}
  caps:    [{x0_m, y_m}, ...]        # 시각 0의 벨트 좌표(conveyor_frame), 한 바퀴마다 반복
  stones:  [{x_m, y_m}, ...]         # mat_frame
  verify:  [{verdict, stone_inside, stone_offset_m}, ...]   # 호출 순서대로, 다 쓰면 마지막 반복
robot:
  duration_scale_ratio: 1.0
  status:  {tcp_xyz_m, orientation_xyzw_ratio}              # 생략 시 (0.40, 0.0, 0.20), identity
  skills:                             # skill별 호출 순서대로, 다 쓰면 마지막 반복
    PickStone: [{duration_s, result, reason_code, hang, cancel_ack, protective_stop_at_ratio, phase_metrics, ...결과 값}]
events: [{t_s, type, ...}]            # 4.3절
assumed:  [<키 경로 또는 하위 트리>]
measured: {<키 경로>: <출처>}
```

- 키 경로는 점으로 잇고 목록 첨자는 뺀다(`world.caps.x0_m`).
- `world`·`robot` 아래 수치 값은 모두 `assumed`(가정) 또는 `measured`(실측, 출처 기록) 중 정확히 하나로 분류한다. `assumed`에는 하위 트리 접두(`robot.skills`)를 쓸 수 있다. 최상위 값(`seed`, `frame_rate_hz`)과 `events`는 시나리오 설계값이라 분류하지 않는다.
- 실측값(v0.1): `world.objects.stone_diameter_m` 0.0175(OFF-1 S01·S02, 17~18 mm 중간값), `world.objects.cap_outer_diameter_m` 0.032(OFF-1 C02), `world.camera.xyz_m`의 높이 0.35(OFF-1 L01, XY는 가정이므로 키 전체를 `assumed`로 둔다).

### 3.2 skill 대본 항목

| 키 | 기본값 | 의미 |
|---|---|---|
| `duration_s` | 1.0 | 소요시간(× `duration_scale_ratio`) |
| `result` | `OK` | 공통 결과 코드 이름 |
| `reason_code` | `""` | |
| `hang` | false | goal을 수락하고 끝내지 않는다. deadline 자체 중단도 하지 않는다 |
| `cancel_ack` | true | false면 cancel 요청을 무시한다 |
| `protective_stop_at_ratio` | 없음 | 소요시간의 이 비율에서 보호정지 시작(4.5절) |
| `phase_metrics` | `{}` | `PlaceStone`·`DynamicPlace` phase 끝 feedback 측정값 덮어쓰기(`{DESCEND: {rim_clearance_m: 0.001}, ...}`) |
| 결과 값 | 아래 | `grip_width_m` 0.0175, `grip_current_ma` 150, `holding`(OK면 true), `pose_error_m` 0.0005, `released`(OK면 true), `release_error_m` 0.0005, `gate_e_xy_m` 0.001, `gate_v_rel_mps` 0.002, `gate_t_since_obs_s` 0.05, `gate_position_std_m` 0.0003 |

## 4. 동작(상태·알고리즘)

### 4.1 시각과 프레임

- 시나리오 시각 `t = t_ros − t0_s`. 프레임 `k`의 시각은 `k / frame_rate_hz`, 발행 stamp는 `t0_s + k / frame_rate_hz + offset(t)`(offset은 `clock_jump`).
- 프레임 출력은 `(seed, k, 대상)`에서 만든 난수로만 결정한다. 같은 시나리오·seed·`t0_s`면 프로세스가 달라도 같은 프레임 `k`에 같은 값이 나온다.
- 노드는 ROS 시계 타이머(`use_sim_time` 따름)로 최신 프레임만 발행하고, 놓친 프레임은 다시 발행하지 않는다.

### 4.2 세계 모델

- 벨트 이동량 `s(t)`는 `speed_change` 이벤트로 나뉜 구간별 등속의 적분이다. 병뚜껑 `i`의 벨트 좌표 `u = x0_i + s(t)`, 바퀴 번호 `lap = floor(u / lap_length_m)`, `x = u − lap·lap_length_m`.
- 병뚜껑은 `obs_start_m ≤ x ≤ obs_end_m`일 때 보인다. track ID는 통과마다 새로 붙인다: `track_id = 100·(lap + 2) + i + 1`.
- 공깃돌은 매트에 고정이다(파지해도 사라지지 않는다. 시도 사이 회수는 작업자 몫).
- 위치: 병뚜껑은 conveyor_frame `(x, y, cap_rim_height_m)`, 공깃돌은 mat_frame `(x, y, stone_height_m)`. robot_base로는 `frames`의 평행이동·yaw로 바꾼다. 잡음 `position_std_m`은 검출과 track에 같은 값을 더한다.
- 픽셀: 광축이 −z인 핀홀, `f = (width/2) / tan(hfov/2)`, `u = cx + f·(X − Xc)/(Zc − Z)`, `v = cy − f·(Y − Yc)/(Zc − Z)`, `diameter_px = f·D/(Zc − Z)`.
- track: 속도 = 벨트 속도 × conveyor_frame x축. 상태는 `track_lost` 중 LOST, `occlusion` 중 COASTING(위치는 참값 + 잡음, `last_observed_stamp`는 가림 직전 프레임), 그 외 OBSERVED. `observation_count`는 시야 진입 후 프레임 수, `valid_until = stamp + valid_for_s`.
- `BeltMotion`: 방향 (1, 0, 0), 속도 = 현재 벨트 속도, `steady`는 마지막 `speed_change` 뒤 `unsteady_hold_s` 동안 false.
- epoch: `epoch_increment` 이벤트 수 + `ResetTracking` 호출 수.

### 4.3 이벤트 10종

| `type` | 키 | 적용 대상 | 효과 |
|---|---|---|---|
| `occlusion` | `duration_s` | world | 병뚜껑 검출 없음, track COASTING |
| `track_lost` | `duration_s` | world | track LOST |
| `epoch_increment` | — | world | epoch + 1 |
| `speed_change` | `speed_mps` | world | 벨트 속도 변경, `steady = false` |
| `frame_drop` | `duration_s`, `drop_ratio` | world | 해당 프레임은 detections·tracked·belt 모두 발행하지 않음 |
| `heartbeat_stop` | `role`(`vision`/`tracking`/`robot_controller`), `duration_s` | 해당 노드 | heartbeat 발행 중단 |
| `clock_jump` | `offset_s` | world | 이후 발행 stamp에 offset(ROS 시각 역행 모사) |
| `stone_in_cap` | `duration_s` | world | 병뚜껑 검출 `stone_inside = true` |
| `protective_stop` | `duration_s` | robot | 4.5절 보호정지 |
| `gripper_delay` | `delay_s`, `duration_s` | robot | 그리퍼를 쓰는 skill(PickStone, PlaceStone, DynamicPlace, `place_back`인 SafeRetreat) 소요시간 + `delay_s` |

`duration_s`를 생략하면 시나리오 끝까지 유지한다. 구간은 `[t_s, t_s + duration_s)`이다.

### 4.4 mock_world 서비스

| 서비스 | 응답 |
|---|---|
| `DetectObjects` | 요청 시각의 프레임에서 `region`·`object_class`가 맞는 검출. `min_count` 미만이면 FAILED `NOT_ENOUGH_OBJECTS`. `position_std` = `position_std_m`. `stable_window`는 v0.1에서 쓰지 않는다 |
| `VerifyDrop` | `world.verify` 대본(기본 SUCCESS, `stone_offset_m` 0.002) |
| `ResetTracking` | epoch + 1, `new_epoch` 반환 |
| `SelectTarget` | 도달 구간 시작(`reach_start_m`)에 아직 들어오지 않았고 보이며 `observation_count ≥ min_observation_count`인 병뚜껑 중 가장 먼저 들어올 것. `t_enter_window`는 현재 속도 유지 가정. 없으면 FAILED `NO_TARGET` |

### 4.5 mock_robot_controller

- `robot/status`(`RobotStatus`)를 활성 상태에서 10 Hz(Reliable, Volatile, depth 1, `header.frame_id = "robot_base"`)로 발행한다. 비활성(lifecycle)이면 발행하지 않으며, `heartbeat_stop`(`role: robot_controller`) 구간에서도 `robot/status`는 계속 발행한다.
  - 필드 값: `connected = true`, `ready = active and not protective_stop`, `protective_stop = protective_stop()`, `tcp_pose` = 시나리오 `robot.status` 절이 있으면 그 값, 없으면 `(x=0.40, y=0.0, z=0.20)` m·orientation identity `(0, 0, 0, 1)`, `in_workspace` = `DynamicPlace`·`PlaceStone` 진행 중 `APPROACH`~`RELEASE` phase이면 `true`(그 외 `false`), `holding` = 마지막 `PickStone` 결과 `holding`이 `true`였고 그 뒤 `PlaceStone`·`DynamicPlace`의 `RELEASE` phase 또는 `SafeRetreat(place_back=true)` 완료가 아직 일어나지 않았으면 `true`, `gripper_alive = true`, `gripper_response_s = 0.1 + gripper_delay(t)`, `gripper_error_bits = 0`, `gripper_open = not holding`, `gripper_width = 0.0175 if holding else 0.022`, `gripper_current = 150.0 if holding else 0.0`.
- skill goal마다 해당 skill 대본의 다음 항목을 쓴다. feedback `phase`는 소요시간을 같은 간격으로 나눠 차례로 보낸다(PickStone: APPROACH·DESCEND·GRIP·LIFT / MoveToPose: MOVE / PlaceStone: APPROACH·DESCEND·RELEASE·RETREAT / DynamicPlace: APPROACH·SYNCHRONIZE·DESCEND·RELEASE·RETREAT / SafeRetreat: RETREAT, `place_back`이면 PLACE_BACK 추가).
- `PlaceStone`·`DynamicPlace` feedback(SPEC-010 2.3.1절):
  - 각 phase 시작 시 `phase_done = false`인 feedback을 1회 이상 보내고, phase 끝에 `phase_done = true`와 해당 phase의 `metric_keys`·`metric_values`를 채운 feedback을 보낸다(`DynamicPlace`의 `SYNCHRONIZE` 중에는 20 Hz 이상으로 `e_xy`·`v_rel`을 채운 `phase_done = false` feedback을 보낸다).
  - `DynamicPlace` goal의 `mode`가 `MODE_PREDICTED(1)`이면 `SYNCHRONIZE` 끝 키는 `entry_time_error_s`, `v_rel_mps`이고, 그 외(`MODE_SYNCHRONIZED`)에는 `e_xy_m`, `v_rel_mps`이다.
  - 측정값 기본값: `plan_valid=1`, `pose_error_m=0.0005`, `e_xy_m=0.001`, `v_rel_mps=0.002`, `entry_time_error_s=0.01`, `sync_ok=1`, `rim_clearance_m=0.0025`, `height_error_m=0.0002`, `target_error_m=0.0005`, `open_width_m=0.022`, `sync_held=1`, `tcp_z_m=0.12`, `outside_workspace=1`, `stone_in_gripper=0`. 대본 항목의 `phase_metrics: {DESCEND: {rim_clearance_m: 0.001}, ...}`로 덮어쓸 수 있다.
- 보호정지는 `protective_stop` 이벤트 구간, 또는 대본의 `protective_stop_at_ratio`에 이른 뒤 노드가 다시 뜰 때까지다(작업자 처리 모사, `Stop`으로 풀리지 않는다).
- 보호정지 중: 진행 중 goal은 FAULT `PROTECTIVE_STOP`으로 끝내고, 새 goal은 수락 후 바로 같은 결과로 끝낸다. Heartbeat `error_code = 100`, `detail = "PROTECTIVE_STOP"`.
- `Stop`: 진행 중 goal(hang 포함)을 CANCELED `STOPPED`로 끝내고, 모두 끝난 뒤 `stopped = true`로 응답한다.
- 동시에 진행하는 goal은 하나다. 진행 중에 새 goal이 오면 거부한다(단일 지령 권한).

### 4.6 lifecycle

- 노드는 lifecycle 노드다. `autostart`면 configure·activate까지 진행한다.
- ACTIVE가 아닐 때: 데이터·상태 topic(`objects/*`, `belt/*`, `robot/status`)을 발행하지 않고, 서비스는 PRECONDITION `NODE_INACTIVE`로 응답하며, action goal은 거부한다. Heartbeat는 계속 발행하고 `lifecycle_state`에 현재 상태 ID(`lifecycle_msgs/State`)를 넣는다(SPEC-010 4.3절).

### 4.7 동시 기동 금지(REQ-MCK-007)

- 기동할 때 `exclusive_check_s`(기본 1.0 s) 동안 같은 역할의 서버(vision: `vision/detect_objects`·`vision/verify_drop`, tracking: `tracking/reset`·`tracking/select_target`, robot: `robot/stop`)가 그래프에 있는지 본다. 있으면 `REAL_NODE_PRESENT`를 기록하고 종료 코드 1로 끝난다.
- 서버 이름은 namespace·remap을 푼 이름으로 센다. 기동 뒤에도 0.5 s(steady)마다 같은 서버의 제공자 수를 본다. 자기 외의 제공자가 생기면 `REAL_NODE_PRESENT`를 기록하고 종료 코드 1로 끝난다. DDS discovery는 수 s 걸릴 수 있으므로(실측 최대 약 2.6 s) 기동 확인 창에서 놓친 서버도 이 감시로 잡는다.
- bringup은 대역이 0이 아닌 코드로 끝나면 전체를 내린다.

## 5. 시간 제약(주기·제한시간)

- 데이터 topic: `frame_rate_hz`(기본 30 Hz). Heartbeat 및 `robot/status`: 10 Hz, steady 시계 타이머(ROS 시각이 멈추거나 역행해도 발행, SPEC-110 REQ-ABT-009).
- 요청의 `deadline`(0이면 없음)이 지났으면 서비스는 TIMEOUT `DEADLINE`으로 응답하고, action은 deadline에 이르면 스스로 TIMEOUT `DEADLINE`으로 끝낸다(`hang` 제외).
- 지연·지터·소요시간은 G0 측정 전까지 가정값이다(`assumed`).

## 6. 실패 모드와 응답(중단 등급)

대역은 중단 등급을 정하지 않는다. 등급은 오케스트레이터가 SPEC-110으로 정한다. 대역은 4.3절 이벤트와 3.2절 대본으로 장애를 만든다.

## 7. 성공 판정(측정 가능)

- REQ-MCK-001: 같은 시나리오·seed로 엔진을 두 번 만들면 프레임 0~599의 출력 열 SHA-256이 같다.
- REQ-MCK-002: `role:=vision`과 `role:=tracking`을 별도 프로세스로 띄우면 같은 stamp의 병뚜껑 위치(검출을 conveyor_frame으로 바꾼 값과 track 값) 차가 1e-9 m 이하이다.
- REQ-MCK-003: `mock_world_node`는 vision·tracker 계약 시험을 통과한다.
- REQ-MCK-004: `mock_robot_controller_node`는 robot_controller 계약 시험을 통과한다.
- REQ-MCK-005: 이벤트 10종은 엔진에서 지정 시각 이후 첫 프레임(`k = ceil(t_event × frame_rate_hz)`, 지연 ≤ 1/`frame_rate_hz` s)에 효과가 나타나고 직전 프레임에는 나타나지 않는다. 30 Hz 데이터의 위상은 시험용 ROS 시각을 한 프레임씩 진행하고 처리·수신을 확인하여 검증한다. 실시간 실행에서는 4.1절의 최신 프레임 발행 정책을 적용한다. 수신한 모든 프레임은 시나리오 stamp와 이벤트 상태가 일치해야 한다(이벤트 시작 전 효과 없음, 시작 후 1프레임부터 종료 전 효과 있음, 종료 후 1프레임부터 효과 없음). 각 데이터 이벤트의 적용 상태를 1회 이상 수신해야 하며, 누락 프레임의 수·sequence는 진단으로 기록한다. 10 Hz heartbeat는 주입 이후 첫 발행 주기(≤ 0.1 s), skill은 주입 이후 첫 검사 주기에 효과가 나타난다. 각 측정은 주입 위상과 수신 시각을 기록한다. 데이터 프레임의 수신 시각은 미들웨어 수신 시각(rclpy `MessageInfo.received_timestamp`)이며, 시스템 시계 보정의 영향을 받지 않도록 콜백 steady 시각에서 콜백 때 시스템 시각과의 차이만큼 거슬러 올라가 단조 시각으로 옮긴다. 프레임 시각 `k / frame_rate_hz` 대비 허용 상한은 0.2 s다. 발행 시각(`source_timestamp`)과 시험 프로세스의 콜백 시각은 진단으로 기록한다. heartbeat·skill 측정은 수신 steady 시각을 쓴다. 실시간 실행 판정은 PC(LEGION, 꺼져 있으면 officepc WSL)에서 하며, 공유 CI 러너에서는 실행·기록만 하고 실패해도 CI 통과 조건에 넣지 않는다(시각 조작 실행과 엔진 시험은 CI에서 판정한다).
- REQ-MCK-006: 시나리오의 `world`·`robot` 수치 값은 100% `assumed` 또는 `measured`로 분류되고(중복 0), `assumed`로 분류된 키 목록이 trial 기록 `assumed_params`로 복사된다.
- REQ-MCK-007: 같은 역할의 실제 노드와 대역을 동시에 지정하면 bringup이 기동을 거부한다.
- REQ-MCK-008: `mock_robot_controller_node`는 SPEC-010 4.5절의 정지 유지·해제를 실제 노드와 같게 따른다(`ready=false`, heartbeat 101, 유지 중 goal은 지령 없이 `CANCELED`(`STOPPED`), `stop_release_reason` 해제 조건). 대본의 보호정지(100)와는 별개다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-MCK-001-01, 002-01, 003-01, 004-01, 005-01, 005-02, 006-01, 006-02, 007-01, 007-02, 008-01(M0).

## 9. 미결 사항

- 대역 지연·소요시간: TBD(측정 G0).

### 관측용 진단 파라미터

`mock_robot_controller_node`의 `cancel_count`·`cancel_received_steady_s`는 cancel 수신 횟수·최근 수신 단조 시각이며 `stop_history_json`은 requester·reason_code·수신 steady 시각을 요청마다 보존한다. `skill_run_count`는 정지 유지 검사를 지나 실제로 실행을 시작한 skill goal 수다. 이 값은 시험에서 읽는 관측 정보이며 지령 판정에는 쓰지 않는다.
