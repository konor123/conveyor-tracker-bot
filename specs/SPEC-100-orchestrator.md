# SPEC-100 orchestrator

- 버전: v0.2
- 근거: PLAN.md 4절
- REQ 영역: ORC

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-11-07 | 상태 열(모드 3종), 단계별 skill·선행조건·판정·기록 키, 재시도·통과 보내기, WAIT_OPERATOR 재개 |
| v0.2 | 2026-11-07 | 비동기 장애(`SkillPort.pending_fault`), 진행 알림 콜백(`on_state`·`on_record`), `orchestrator_node`의 ROS 대응(4.8절) |

## 1. 목적·범위

`orchestrator` 패키지의 상태 머신(`state_machine.py`), 단계 판정(`step_judges.py`), 사이클 실행기(`cycle_runner.py`)를 정한다. 중단 등급·제한시간·감시는 SPEC-110, 메시지 정의는 SPEC-010을 따른다. 판단 로직은 ROS와 무관한 순수 Python이고 `orchestrator_node`는 skill 호출을 `SkillPort`로 감싸 넘긴다.

## 2. 입력·출력(interface 버전)

SPEC-010 v0.1 기준.

- 입력: `DropTask` goal(mode, max_belt_speed, timeout, max_passes), skill 결과·feedback, `TrackedObjectArray`, `BeltMotion`, `InterceptPlan`, `Heartbeat`, 로봇·그리퍼 상태.
- 출력: `DropTask` 결과·feedback, `TaskStatus`(10 Hz), `StepRecord`(단계 종료마다).
- 판정 결과: `Judgement(result, reason_code, metrics, detail)`. `result`는 공통 결과 코드(OK=0 … FAULT=5), `metrics`는 판정에 쓴 값 전부이며 `StepRecord.metric_keys`·`metric_values`로 그대로 복사한다.

## 3. 파라미터(단위·기본값·근거)

`src/orchestrator/config/orchestrator.yaml`의 `judge:`·`step_timeout:`·`retry:` 절이 원본이다. 공깃돌·병뚜껑 치수는 OFF-1 실측 전 제안값이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `precheck.heartbeat_age_max_s` | 0.3 | 10 Hz 3회 |
| `precheck.gripper_response_max_s` | 0.5 | TBD(측정 G0) |
| `detect_stone.confidence_min_ratio` | 0.9 | PLAN.md 4.3절 |
| `detect_stone.window_std_max_m` | 0.0005 | PLAN.md 4.3절 |
| `detect_stone.clearance_min_m` | 0.02 | TBD(핑거 어댑터 치수, OFF-1) |
| `pick_stone.d_stone_min_m` | 0.017 | 실측 OFF-1(2026-11-07), 팔각기둥 면-면 |
| `pick_stone.d_stone_max_m` | 0.018 | 실측 OFF-1(2026-11-07), 팔각기둥 꼭짓점-꼭짓점 |
| `release.d_stone_max_m` | 0.018 | 같음 |
| `pick_stone.width_tol_m` | 0.001 | PLAN.md 4.3절 |
| `pick_stone.grip_current_min_ma` | 100 | TBD(측정 G0, TC-BEN-008-01) |
| `pick_stone.detection_age_max_s` | 1.0 | PLAN.md 4.3절 선행조건 |
| `standby.pose_error_max_m` | 0.002 | PLAN.md 4.3절 |
| `wait_cap.observation_count_min_count` | 30 | PLAN.md 4.3절 |
| `wait_cap.speed_std_max_mps` | 0.001 | PLAN.md 4.3절 |
| `wait_cap.margin_s` | 1.0 | PLAN.md 9절 |
| `plan.time_margin_min_s` | 1.0 | PLAN.md 4.3절 |
| `plan.position_std_max_m` | 0.0005 | PLAN.md 4.3절 |
| `plan.t_approach_s` | 1.0 | TBD(측정 G0) |
| `sync.e_xy_max_m` | 0.00275 | `0.5 × (D_in − d_stone_max) / 2`, TBD(`D_in` 실측 OFF-1, G2 낙하 시험) |
| `sync.v_rel_max_mps` | 0.005 | PLAN.md 4.4절 |
| `sync.hold_s` | 0.3 | PLAN.md 4.3절 |
| `sync.timeout_s` | 2.0 | PLAN.md 4.3절 |
| `predicted.entry_time_tol_s` | 0.05 | TBD(측정 G4) |
| `descend.height_tol_m` | 0.0005 | PLAN.md 4.3절 |
| `descend.h_clear_m` | 0.002 | PLAN.md 4.4절 |
| `static_place.target_error_max_m` | 0.001 | PLAN.md 4.3절(MODE_STATIC) |
| `release.delta_open_m` | 0.004 | PLAN.md 1.1절 |
| `release.open_tol_m` | 0.001 | PLAN.md 4.3절 |
| `retreat.safe_z_m` | 0.10 | TBD(현장 배치 OFF-1) |
| `verify.timeout_s` | 3.0 | PLAN.md 4.3절 |
| `retry.detect_stone_count` | 2 | PLAN.md 4.3절 |
| `retry.pick_stone_count` | 2 | PLAN.md 4.3절 |

단계 제한시간(`step_timeout:`)은 PLAN.md 4.3절 값을 쓰고 측정 후 SPEC-110 5절 식으로 바꾼다.

## 4. 동작(상태·알고리즘)

### 4.1 상태 열

| 모드 | 상태 열 |
|---|---|
| MODE_SYNCHRONIZED | IDLE → PRECHECK → DETECT_STONE → PICK_STONE → MOVE_STANDBY → WAIT_CAP → PLAN_INTERCEPT → APPROACH → SYNCHRONIZE → DESCEND → RELEASE → RETREAT → VERIFY → DONE |
| MODE_PREDICTED | MODE_SYNCHRONIZED와 같다. SYNCHRONIZE 판정을 "계획 속도 일치 구간 진입" 판정(`judge_predicted_entry`)으로 바꾼다 |
| MODE_STATIC | PLAN_INTERCEPT·SYNCHRONIZE가 없다(목표 = 정지 뚜껑 검출 위치). WAIT_CAP은 정지 뚜껑 검출(`DetectObjects(REGION_BELT, CLASS_CAP)`), APPROACH~RETREAT는 `PlaceStone`, DESCEND는 정지 목표 기준 오차 판정(`judge_static_place`) |

중단 상태: ABORT_CYCLE(L2), SAFE_ABORT(L3), PROTECTIVE_STOP(L4) → WAIT_OPERATOR. `next_state()`는 판정·중단 등급·문맥만으로 다음 상태를 정하는 순수 함수다.

### 4.2 단계별 skill·선행조건·판정

| 상태 | skill | 선행조건 | 판정 함수 | 실패 시 이벤트 |
|---|---|---|---|---|
| PRECHECK | 없음(상태 조회) | — | `judge_precheck` | 실패 시 시도하지 않고 IDLE(로봇 미동작) |
| DETECT_STONE | `DetectObjects(REGION_MAT, CLASS_STONE)` | PRECHECK OK | `judge_detect_stone` | 단계 실패(재시도 가능) |
| PICK_STONE | `PickStone` | 검출 시각이 `detection_age_max_s` 이내 | `judge_pick_stone` | `STONE_ON_MAT`만 재시도 가능 |
| MOVE_STANDBY | `MoveToPose(standby)` | holding | `judge_move_standby` | 단계 실패 |
| WAIT_CAP | `SelectTarget`(STATIC: `DetectObjects(REGION_BELT, CLASS_CAP)`) | 대기 자세, holding | `judge_wait_cap` | `PASS_SKIP`이면 다음 통과, 소진 시 `PASSES_EXHAUSTED` |
| PLAN_INTERCEPT | `ComputeIntercept`(STATIC: 상태 없음) | holding, track `OBSERVED`, epoch 일치 | `judge_plan_intercept` | 동적 deadline까지 재계획, 넘으면 다음 통과 |
| APPROACH | `DynamicPlace`(STATIC: `PlaceStone`) | plan 유효 | `judge_approach` | 단계 실패 |
| SYNCHRONIZE | `DynamicPlace` feedback | 접근 완료 | `judge_synchronize` / `judge_predicted_entry` | 단계 실패 |
| DESCEND | `DynamicPlace` feedback(STATIC: `PlaceStone` feedback) | 동기 게이트 유지 | `judge_descend` / `judge_static_place` | 단계 실패 |
| RELEASE | `DynamicPlace` feedback | 해제 게이트 | `judge_release` | 해제 후 실패는 RETREAT → VERIFY |
| RETREAT | `DynamicPlace` 결과(STATIC: `PlaceStone` 결과) | 해제 완료 또는 L3 | `judge_retreat` | `RETREAT_FAILED`(L4) |
| VERIFY | `VerifyDrop` | 후퇴 완료 | `judge_verify` | UNKNOWN 기록 |

### 4.3 판정 이유 코드(`reason_code`)

| 상태 | 이유 코드(검사 순서) |
|---|---|
| PRECHECK | `NODE_INACTIVE`, `HEARTBEAT_STALE`, `CALIBRATION_INVALID`, `PROTECTIVE_STOP`, `ROBOT_NOT_READY`, `GRIPPER_DEAD`, `GRIPPER_ERROR`, `GRIPPER_SLOW`, `GRIPPER_NOT_OPEN`, `BELT_UNSTEADY`, `BELT_TOO_FAST` |
| DETECT_STONE | `NO_STONE`, `POSITION_INVALID`, `LOW_CONFIDENCE`, `POSITION_UNSTABLE`, `CLEARANCE` |
| PICK_STONE | `ROBOT_ERROR`, `GRIPPER_ERROR`, `STONE_ON_MAT`, `GRIP_WIDTH`, `GRIP_CURRENT_LOW` |
| MOVE_STANDBY | `DROPPED`, `POSE_ERROR` |
| WAIT_CAP | `CAP_NOT_EMPTY`(FAILED), `PASS_SKIP`(detail `CAP_STATE_UNKNOWN` / `TIME_MARGIN`), `WAIT`(detail `TRACK_NOT_OBSERVED` / `OBSERVATION_COUNT` / `SPEED_STD` / `BELT_UNSTEADY`) |
| PLAN_INTERCEPT | `PLAN_STATUS`, `EPOCH_MISMATCH`, `TIME_MARGIN`, `POSITION_STD`, `PLAN_EXPIRES` |
| APPROACH | `PLAN_INVALID`, `POSE_ERROR` |
| SYNCHRONIZE | `WAIT`(진행 중), `TIMEOUT`(result TIMEOUT) |
| DESCEND | `SYNC_LOST`, `RIM_CLEARANCE`, `HEIGHT_ERROR` |
| RELEASE | `OPEN_WIDTH`, `SYNC_LOST` |
| RETREAT | `Z_LOW`, `IN_WORKSPACE`, `STONE_IN_GRIPPER` |
| VERIFY | `VERIFY_TIMEOUT`(UNKNOWN), 그 외는 `VerifyDrop` 결과 |

`WAIT`는 판정이 아직 나지 않았음을 뜻하며 실행기는 같은 단계에서 관측을 계속한다. `StepRecord`에는 최종 판정만 남긴다.

### 4.4 기록 키(`Judgement.metrics`)

| 상태 | 키 |
|---|---|
| PRECHECK | `nodes_active`, `heartbeat_age_max_s`, `calibration_valid`, `robot_ready`, `protective_stop`, `gripper_alive`, `gripper_response_s`, `gripper_error_bits`, `gripper_open`, `belt_steady`, `belt_speed_mps` |
| DETECT_STONE | `stone_count`, `confidence`, `position_valid`, `window_std_m`, `clearance_m` |
| PICK_STONE | `grip_width_m`, `grip_current_ma`, `stone_on_mat`, `robot_error`, `gripper_error` |
| MOVE_STANDBY | `pose_error_m`, `holding` |
| WAIT_CAP | `track_observed`, `observation_count`, `speed_std_mps`, `belt_steady`, `stone_inside`, `stone_inside_valid`, `t_available_s`, `t_required_s`, `time_margin_s`, 시간 예산 성분 `budget_<성분>_s` |
| PLAN_INTERCEPT | `plan_ok`, `epoch_match`, `time_margin_s`, `position_std_m`, `valid_for_s` |
| APPROACH | `plan_valid`, `pose_error_m` |
| SYNCHRONIZE | `elapsed_s`, `hold_s`, `e_xy_m`, `v_rel_mps` |
| SYNCHRONIZE(PREDICTED) | `entry_time_error_s`, `v_rel_mps` |
| DESCEND | `sync_ok`, `rim_clearance_m`, `height_error_m` |
| DESCEND(STATIC) | `target_error_m`, `rim_clearance_m`, `height_error_m` |
| RELEASE | `open_width_m`, `sync_held` |
| RETREAT | `tcp_z_m`, `outside_workspace`, `stone_in_gripper` |
| VERIFY | `responded`, `verdict` |

불리언 값은 1.0/0.0으로 기록한다.

### 4.5 재시도·통과 보내기

- 재시도(L1)는 DETECT_STONE(2회), PICK_STONE(`STONE_ON_MAT`일 때 2회)만 허용한다. 재시도는 그 단계의 선행조건부터 다시 본다. PICK_STONE 재시도 시 검출 시각이 `detection_age_max_s`를 넘었으면 DETECT_STONE부터 다시 실행한다.
- WAIT_CAP에서 `PASS_SKIP`이면 그 통과를 보내고 `passes_skipped`를 1 올린다. PLAN_INTERCEPT가 동적 deadline까지 OK를 내지 못해도 같다. `passes_skipped`가 `max_passes`를 넘으면 L3(`SafeRetreat(keep_sync=false, place_back=true, place_back_pose=매트)`), 결과 `NOT_ATTEMPTED`.
- 통과 보내기는 로봇이 대기 자세에 있을 때만 한다. APPROACH 이후에는 통과 보내기를 하지 않는다.

### 4.6 WAIT_OPERATOR

- 진입 시각·사유·중단 등급을 기록한다. 제한시간 없이 기다린다.
- `resume`은 비어 있지 않은 사유 문자열이 있을 때만 받아들이고 PRECHECK부터 다시 시작한다. 자동 리셋·원점 복귀는 하지 않는다.

### 4.7 실행기와 결과

- `CycleRunner.run(goal)`은 한 사이클을 동기식으로 실행한다. skill은 `SkillPort`로 부르고, 포트는 deadline 안에 응답이 없으면 `None`을 돌려준다. 응답이 없으면 판정은 `TIMEOUT`(metrics 비어 있음)이다.
- action(`PickStone`, `MoveToPose`, `DynamicPlace`, `PlaceStone`)이 응답하지 않으면 cancel을 보내고 SPEC-110 4.2절대로 승격한다. `DynamicPlace`·`PlaceStone` goal은 APPROACH부터 RETREAT 결과까지 이어진다.
- WAIT_CAP은 통과마다 `SelectTarget`(STATIC: `DetectObjects(REGION_BELT, CLASS_CAP)`)으로 시작하고, `WAIT`이면 `observe_cap`으로 관측을 계속한다. 통과 시간이 `lap_s`를 넘으면 `PASS_SKIP`(detail `PASS_TIMEOUT`). 기록은 통과마다 최종 판정 1건이다.
- PLAN_INTERCEPT는 실제 deadline(`min(now + 0.5 s, 동적 deadline, 사이클 deadline)`)까지 재계산하고, 통과마다 최종 판정 1건을 기록한다.
- PICK_STONE 선행조건(검출 나이)이 깨지면 `PRECONDITION`을 기록하고 DETECT_STONE으로 돌아간다. 이 재검출은 `retry.pick_stone_count`번까지이고 넘으면 L2다.
- holding은 PICK_STONE OK부터 RELEASE 응답까지 참으로 둔다(MOVE_STANDBY의 `DROPPED`도 L3로 처리해 매트 쪽으로 정리한다).
- 비동기 장애: 포트의 `pending_fault()`가 SPEC-110 이벤트(보호정지, controller heartbeat 손실, vision·tracker heartbeat 손실, 그리퍼 이상 등)를 돌려주면 실행기는 단계 실행 전과 직후에 그 이벤트로 중단 등급을 정한다(판정 `FAULT`, 이유 = 이벤트 이름). 포트는 장애가 생기면 진행 중인 대기를 바로 끝내고 `None`을 돌려준다. 장애는 사이클이 끝날 때까지 유지된다(포트가 latch).
- 진행 알림: `CycleRunner(..., on_state=f, on_record=g)`이면 상태가 바뀔 때 `f(state)`, 단계 기록이 생길 때 `g(StepRec)`를 부른다(TaskStatus·StepRecord 발행용). 결과 열은 콜백 유무와 같다.

| 끝난 방식 | `DropTask.result` |
|---|---|
| PRECHECK 실패(IDLE 복귀) | `NOT_ATTEMPTED`(reason = PRECHECK 이유 코드) |
| DONE | `VerifyDrop` 판정(SUCCESS / FAILURE / UNKNOWN). RELEASE 판정이 실패했으면 SUCCESS 대신 FAILURE |
| L2 | `FAILURE` |
| L3 | `SAFE_ABORT`(통과 보내기 소진이면 `NOT_ATTEMPTED`) |
| L4(중단 경로 중 승격 포함) | `PROTECTIVE_STOP` |
| 사용자 cancel | `CANCELED` |

### 4.8 `orchestrator_node`(ROS 대응)

| SkillPort | ROS | 측정값 출처 |
|---|---|---|
| `status` | 구독 캐시 | `system/heartbeat`(노드 ACTIVE·나이), `robot/status`, `belt/measured_motion`, `objects/detections.calibration_id` |
| `detect_stones` | `vision/detect_objects`(MAT, STONE) | 응답 검출. `clearance_m` = 가장 가까운 다른 검출까지 거리(없으면 1 m). 검출 시각 = 응답 수신 시각 |
| `pick_stone` | `robot/pick_stone` | 결과 값. `stone_on_mat` = 결과 뒤 `DetectObjects(MAT)`에 목표 10 mm 안 공깃돌이 남음. 이유 코드 `GRIPPER*` = 그리퍼 오류, 그 밖 FAULT = 로봇 오류 |
| `move_to_pose` | `robot/move_to_pose`(named_pose) | `pose_error`, holding = `robot/status.holding` |
| `select_target`·`observe_cap` | `tracking/select_target`, `objects/tracked` | track 상태·관측 수, `BeltMotion`, 병뚜껑 검출 `stone_inside`, 시간 예산(4.9절) |
| `detect_cap`(STATIC) | `vision/detect_objects`(BELT, CAP) | 정지 뚜껑 검출 |
| `compute_intercept` | `planner/compute_intercept` | `InterceptPlan` |
| `place_start`·`place_feedback` | `robot/place_stone`·`robot/dynamic_place` | phase 끝 feedback의 `metric_keys/values`(SPEC-010 v0.2). SYNCHRONIZE는 진행 중 feedback의 `e_xy`·`v_rel` 열 |
| `verify` | `vision/verify_drop` | `verdict` |
| `safe_retreat`·`stop`·`cancel_active` | `robot/safe_retreat`, `robot/stop`, goal cancel | — |
| `pending_fault` | 구독 캐시 | `robot/status.protective_stop` → PROTECTIVE_STOP, `connected = false` → ROBOT_COMM_LOST, heartbeat 손실(SPEC-110) |

### 4.9 시간 예산(M0 임시)

planner의 `time_budget`이 생기기 전까지 WAIT_CAP 예산은 `orchestrator_node`가 conveyor_frame에서 계산한다: `t_available = (x_exit − x_cap) / v`, `t_required = Σ budget 성분`(설정 `budget:` 절). 실제 planner가 계약 시험을 통과하면 planner 결과로 바꾼다.

## 5. 시간 제약(주기·제한시간)

| 상태 | 제한시간 [s] |
|---|---|
| PRECHECK | 5 |
| DETECT_STONE | 3 |
| PICK_STONE | 15 |
| MOVE_STANDBY | 10 |
| WAIT_CAP | `T_lap × (max_passes + 1)` |
| PLAN_INTERCEPT | 0.5 |
| APPROACH | `1.5 × T_transit(p99)`, 측정 전 3.0 |
| SYNCHRONIZE | 2 |
| DESCEND | 1.5 |
| RELEASE | `1.5 × T_release(p99)`, 측정 전 1.0 |
| RETREAT | 3 |
| VERIFY | 3 |

`TaskStatus`는 10 Hz로 발행한다. 실제 deadline 계산은 SPEC-110 5절을 따른다.

## 6. 실패 모드와 응답(중단 등급)

- REQ-ORC-001: 모든 판정이 OK면 모드별 상태 열이 4.1절과 정확히 같다(MODE_STATIC은 PLAN_INTERCEPT·SYNCHRONIZE 없이 WAIT_CAP에서 `DetectObjects(REGION_BELT, CLASS_CAP)` 1회 이상 호출, MODE_PREDICTED는 SYNCHRONIZE 판정 입력이 진입 시각 오차·속도 오차).
- REQ-ORC-002: 어느 단계든 판정이 OK가 아니면 그 다음 단계의 skill을 0회 호출한다.
- REQ-ORC-003: 단계 판정은 3절 임계값으로 4.3절 이유 코드를 낸다. 경계값은 `TEST_SPEC.md` TC-ORC-003-01~14의 세 점(직전·경계·직후)에서 기대값과 일치한다.
- REQ-ORC-004: `Judgement.metrics`의 키 집합은 4.4절 표와 같고 `StepRecord.metric_keys`·`metric_values`에 순서대로 그대로 복사된다(누락 0건).
- REQ-ORC-005: 선행조건이 성립하지 않으면 결과 `PRECONDITION`, 해당 skill 호출 0회다(예: PICK_STONE에서 검출 시각이 1.01 s 전).
- REQ-ORC-006: DETECT_STONE은 최대 2회 재시도한다(3번째 시도 OK면 진행, 3회 모두 실패면 L2). 재시도는 선행조건부터 다시 본다.
- REQ-ORC-007: 통과 보내기는 `max_passes`회까지 한다. 소진하면 L3, `SafeRetreat(keep_sync=false, place_back=true)`, 결과 `NOT_ATTEMPTED`, `passes_skipped`를 기록한다.
- REQ-ORC-008: PLAN_INTERCEPT가 동적 deadline까지 OK가 아니면 APPROACH를 0회 호출하고, 남은 통과가 있으면 WAIT_CAP으로 돌아간다.
- REQ-ORC-009: WAIT_OPERATOR에서 사유 없는 `resume`은 거부하고, 사유 있는 `resume`은 PRECHECK부터 다시 시작하며 진입 시각·사유를 기록한다.

## 7. 성공 판정(측정 가능)

- U 시험(TC-ORC-*) 전부 통과, `trace.py --area ORC` 누락 0.
- M0: I0 `nominal_20mms` 20/20, 오케스트레이터 REQ 연결률 100%.
- G2: 실패한 단계 뒤에 다음 단계가 실행된 사례 0회, 단계 판정과 관찰자 기록 불일치 ≤ 5%.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-ORC-001-01 ~ TC-ORC-009-01(U, M0). ROS 수준은 TC-INT-001-01, TC-INT-005-01, TC-INT-006-01.

## 9. 미결 사항

- 병뚜껑 안쪽 지름 `D_in`과 이에 따른 `e_xy_max_m`, `clearance_min_m`, `safe_z_m`: TBD(실측 OFF-1).
- `grip_current_min_ma`, `gripper_response_max_s`, `t_approach_s`: TBD(측정 G0).
- `entry_time_tol_s`: TBD(측정 G4).
