# SPEC-100 orchestrator

- 버전: v0.18
- 근거: PLAN.md 4절
- REQ 영역: ORC

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.18 | 2026-11-14 | RELEASE 단계 제한 측정 전 1.0 → 2.0 s(REQ-ABT-007 식, 실제 controller I0 fake transport 0.91 s 기준 잠정). 여러 단계 goal deadline 동적 11.5 s·STATIC 9.5 s(REQ-ORC-015) |
| v0.17 | 2026-11-13 | phase 끝 feedback을 못 받은 경로를 경고로 남기고, 실패 단계 경고에 단계 길이를 붙인다(REQ-ORC-020) |
| v0.16 | 2026-11-13 | Stop 해제 직후 goal: 수락 뒤 controller heartbeat 2개·`robot/status`를 새로 받은 다음 장애 latch 무장(REQ-ORC-012). WAIT_CAP 시간 예산 = `time_budget.evaluate_budget`, 동적 deadline = `deadlines.dynamic_deadline`(4.9절, REQ-ORC-019). `budget.x_reach_start_m`·`x_exit_m` ROS 파라미터. PRECHECK `BELT_UNSTEADY`는 지속 규칙(SPEC-110 REQ-ABT-013). 감시 latch 구간(4.8절) |
| v0.15 | 2026-11-12 | PRECHECK 이유 코드 `CALIBRATION_MISMATCH`: vision 검출과 orchestrator `calibration_id`가 둘 다 있고 다르면(SPEC-600 REQ-CAL-005). vision 쪽이 비었으면 `CALIBRATION_INVALID`, orchestrator 쪽이 비었으면 비교하지 않는다 |
| v0.14 | 2026-11-12 | WAIT_CAP `t_to_window_s`를 매 관측의 현재 track 위치·속도로 다시 계산(REQ-ORC-018). 선택 순간 한 번 받은 진입 예상 시각은 갓 생긴 track의 속도 추정을 그대로 담아 실영상에서 수~수십 s 늦었다 |
| v0.13 | 2026-11-12 | 여러 단계에 걸친 goal의 서버 deadline = goal 전체 상한(REQ-ORC-015), WAIT_CAP 대상 track 소실 시 같은 통과 안 재선택(REQ-ORC-016), 대기 중 끝난 WAIT_CAP의 기록·실패 단계 로그(REQ-ORC-017) |
| v0.12 | 2026-11-11 | 계획 단계 위치 표준편차 기준(3.25 mm)과 해제 게이트 기준(0.5 mm) 분리, WAIT_CAP 계획 시점(도달 구간 진입 3.0 s 전부터, REQ-ORC-013·014) |
| v0.11 | 2026-11-11 | controller heartbeat `error_code` 100·101을 사이클 전 구간 감시에 적용(REQ-ORC-012) |
| v0.10 | 2026-11-11 | 병뚜껑 안쪽 지름 `D_in` 31 mm로 `sync.e_xy_max_m` 0.00325 |
| v0.9 | 2026-11-10 | skill 결과 `STOPPED`·`PROTECTIVE_STOP`을 하드 이상으로 latch(REQ-ORC-011) |
| v0.8 | 2026-11-10 | 재검출·재파지(L1) 제거: DETECT_STONE·PICK_STONE 실패와 검출 나이 초과는 L2 |
| v0.7 | 2026-11-09 | 해제 후 RETREAT·VERIFY 독립 단계 예산과 준비된 응답 우선 |
| v0.6 | 2026-11-09 | ROS 역행을 고려한 포트 steady 예산 배율 명시 |
| v0.5 | 2026-11-09 | 미확정 action 수락 보존·취소 및 포트 대기 중단 계약 |
| v0.4 | 2026-11-09 | M0 검증 범위·게이트 및 독립 계약 기대값 정합성 |
| v0.1 | 2026-11-07 | 상태 열(모드 3종), 단계별 skill·선행조건·판정·기록 키, 재시도·통과 보내기, WAIT_OPERATOR 재개 |
| v0.2 | 2026-11-07 | 비동기 장애(`SkillPort.pending_fault`), 진행 알림 콜백(`on_state`·`on_record`), `orchestrator_node`의 ROS 대응(4.8절) |
| v0.3 | 2026-11-09 | 해제 뒤 비하드 장애의 포트 대기 처리와 latch 교체 규칙(4.7절, REQ-ORC-010) |

## 1. 목적·범위

`orchestrator` 패키지의 상태 머신(`state_machine.py`), 단계 판정(`step_judges.py`), 사이클 실행기(`cycle_runner.py`)를 정한다. 중단 등급·제한시간·감시는 SPEC-110, 메시지 정의는 SPEC-010을 따른다. 판단 로직은 ROS와 무관한 순수 Python이고 `orchestrator_node`는 skill 호출을 `SkillPort`로 감싸 넘긴다.

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4 기준.

- 입력: `DropTask` goal(mode, max_belt_speed, timeout, max_passes), skill 결과·feedback, `TrackedObjectArray`, `BeltMotion`, `InterceptPlan`, `Heartbeat`, 로봇·그리퍼 상태.
- 출력: `DropTask` 결과·feedback, `TaskStatus`(10 Hz), `StepRecord`(단계 종료마다).
- 판정 결과: `Judgement(result, reason_code, metrics, detail)`. `result`는 공통 결과 코드(OK=0 … FAULT=5), `metrics`는 판정에 쓴 값 전부이며 `StepRecord.metric_keys`·`metric_values`로 그대로 복사한다.

## 3. 파라미터(단위·기본값·근거)

`src/orchestrator/config/orchestrator.yaml`의 `judge:`·`step_timeout:` 절이 원본이다. 공깃돌·병뚜껑 치수는 OFF-1 실측 전 제안값이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `precheck.heartbeat_age_max_s` | 0.3 | 10 Hz 3회 |
| `precheck.gripper_response_max_s` | 0.5 | TBD(측정 G0) |
| `detect_stone.confidence_min_ratio` | 0.9 | PLAN.md 4.3절 |
| `detect_stone.window_std_max_m` | 0.0005 | PLAN.md 4.3절 |
| `detect_stone.clearance_min_m` | 0.02 | TBD(기본 핑거 치수, OFF-1) |
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
| `wait_cap.plan_lead_s` | 3.0 | 도달 구간 진입 이 시간 전부터 계획한다. planner `release_lead` 2.8 s를 덮는 값. 2026-11-11 로컬 실험(worklog) |
| `plan.time_margin_min_s` | 1.0 | PLAN.md 4.3절 |
| `plan.position_std_max_m` | 0.00325 | 계획 단계 기준 = `sync.e_xy_max_m`. 해제 게이트와 분리(PLAN.md 4.3·4.4절) |
| `plan.t_approach_s` | 1.0 | TBD(측정 G0) |
| `sync.e_xy_max_m` | 0.00325 | `0.5 × (D_in − d_stone_max) / 2`, `D_in` 31 mm·`d_stone_max` 18 mm |
| `sync.v_rel_max_mps` | 0.005 | PLAN.md 4.4절 |
| `sync.hold_s` | 0.3 | PLAN.md 4.3절 |
| `sync.timeout_s` | 2.0 | PLAN.md 4.3절 |
| `sync.release_position_std_max_m` | 0.0005 | 해제 게이트 위치 표준편차 기준. `DynamicPlace.Goal.position_std_max`로 보낸다(PLAN.md 4.4절) |
| `predicted.entry_time_tol_s` | 0.05 | TBD(측정 G4) |
| `descend.height_tol_m` | 0.0005 | PLAN.md 4.3절 |
| `descend.h_clear_m` | 0.002 | PLAN.md 4.4절 |
| `static_place.target_error_max_m` | 0.001 | PLAN.md 4.3절(MODE_STATIC) |
| `release.delta_open_m` | 0.004 | PLAN.md 1.1절 |
| `release.open_tol_m` | 0.001 | PLAN.md 4.3절 |
| `retreat.safe_z_m` | 0.10 | TBD(현장 배치 OFF-1) |
| `verify.timeout_s` | 3.0 | PLAN.md 4.3절 |

`wait_cap.time_margin_s = t_available_s − t_required_s`이며 통과 조건은 `time_margin_s >= margin_s`다.

`wait_cap.t_to_window_s = max(0, (x_reach_start − x_cap) / v)`(매 관측의 대상 track 위치·속도, `budget.x_reach_start_m`; 이미 지났거나 STATIC이면 0, 속도 ≤ 0(`BELT_STOPPED`·`BELT_REVERSED`)이면 ∞)이며 `t_to_window_s > plan_lead_s`면 `WAIT`(`TOO_EARLY`)다. 뚜껑을 처음 볼 때 계획하면 해제까지 예측 구간이 길어(nominal에서 약 10 s) 위치 표준편차가 커진다.

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
| DETECT_STONE | `DetectObjects(REGION_MAT, CLASS_STONE)` | PRECHECK OK | `judge_detect_stone` | 단계 실패(L2) |
| PICK_STONE | `PickStone` | 검출 시각이 `detection_age_max_s` 이내 | `judge_pick_stone` | 단계 실패(L2) |
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
| PRECHECK | `NODE_INACTIVE`, `HEARTBEAT_STALE`, `CALIBRATION_MISMATCH`, `CALIBRATION_INVALID`, `PROTECTIVE_STOP`, `ROBOT_NOT_READY`, `GRIPPER_DEAD`, `GRIPPER_ERROR`, `GRIPPER_SLOW`, `GRIPPER_NOT_OPEN`, `BELT_UNSTEADY`, `BELT_TOO_FAST` |
| DETECT_STONE | `NO_STONE`, `POSITION_INVALID`, `LOW_CONFIDENCE`, `POSITION_UNSTABLE`, `CLEARANCE` |
| PICK_STONE | `ROBOT_ERROR`, `GRIPPER_ERROR`, `STONE_ON_MAT`, `GRIP_WIDTH`, `GRIP_CURRENT_LOW` |
| MOVE_STANDBY | `DROPPED`, `POSE_ERROR` |
| WAIT_CAP | `CAP_NOT_EMPTY`(FAILED), `PASS_SKIP`(detail `CAP_STATE_UNKNOWN` / `TIME_MARGIN`), `WAIT`(detail `TRACK_NOT_OBSERVED` / `OBSERVATION_COUNT` / `SPEED_STD` / `BELT_UNSTEADY` / `TOO_EARLY`) |
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
| WAIT_CAP | `track_observed`, `observation_count`, `speed_std_mps`, `belt_steady`, `stone_inside`, `stone_inside_valid`, `t_available_s`, `t_required_s`, `time_margin_s`, `t_to_window_s`, 시간 예산 성분 `budget_<성분>_s` |
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

### 4.5 재시도 없음·통과 보내기

- 단계 실패는 재시도하지 않는다. DETECT_STONE·PICK_STONE 실패(`STONE_ON_MAT` 포함)는 L2이며 작업자가 확인한 뒤 `resume`으로 PRECHECK부터 다시 시작한다.
- WAIT_CAP에서 `PASS_SKIP`이면 그 통과를 보내고 `passes_skipped`를 1 올린다. PLAN_INTERCEPT가 동적 deadline까지 OK를 내지 못해도 같다. `passes_skipped`가 `max_passes`를 넘으면 L3(`SafeRetreat(keep_sync=false, place_back=true, place_back_pose=매트)`), 결과 `NOT_ATTEMPTED`.
- 통과 보내기는 로봇이 대기 자세에 있을 때만 한다. APPROACH 이후에는 통과 보내기를 하지 않는다.
- WAIT_CAP에서 고른 track이 사라지면(대상을 고른 뒤 받은 `objects/tracked`에 그 track이 없거나 `LOST`이거나 epoch가 다름: 가림 초과, `ResetTracking`, 시각 역행) 통과를 보내지 않고 같은 통과 안에서 `SelectTarget`을 다시 호출한다(`CapObservation.target_lost`). `COASTING`은 소실이 아니며 기다린다. 다음 통과에서 제외하는 track은 통과가 바뀔 때의 대상뿐이고, 같은 통과의 재선택은 같은 track을 다시 고를 수 있다.

### 4.6 WAIT_OPERATOR

- 진입 시각·사유·중단 등급을 기록한다. 제한시간 없이 기다린다.
- `resume`은 비어 있지 않은 사유 문자열이 있을 때만 받아들이고 PRECHECK부터 다시 시작한다. 자동 리셋·원점 복귀는 하지 않는다.

### 4.7 실행기와 결과

- `CycleRunner.run(goal)`은 한 사이클을 동기식으로 실행한다. skill은 `SkillPort`로 부르고, 포트는 deadline 안에 응답이 없으면 `None`을 돌려준다. 응답이 없으면 판정은 `TIMEOUT`(metrics 비어 있음)이다.
- action(`PickStone`, `MoveToPose`, `DynamicPlace`, `PlaceStone`)이 응답하지 않으면 cancel을 보내고 SPEC-110 4.2절대로 승격한다. `DynamicPlace`·`PlaceStone` goal은 APPROACH부터 RETREAT 결과까지 이어진다.
- WAIT_CAP은 통과마다 `SelectTarget`(STATIC: `DetectObjects(REGION_BELT, CLASS_CAP)`)으로 시작하고, `WAIT`이면 `observe_cap`으로 관측을 계속한다. 통과 시간이 `lap_s`를 넘으면 `PASS_SKIP`(detail `PASS_TIMEOUT`). 기록은 통과마다 최종 판정 1건이다.
- PLAN_INTERCEPT는 실제 deadline(`min(now + 0.5 s, 동적 deadline, 사이클 deadline)`)까지 재계산하고, 통과마다 최종 판정 1건을 기록한다.
- PICK_STONE 선행조건(검출 나이)이 깨지면 `PRECONDITION`을 기록하고 재검출 없이 L2로 끝낸다.
- holding은 PICK_STONE OK부터 RELEASE 응답까지 참으로 둔다(MOVE_STANDBY의 `DROPPED`도 L3로 처리해 매트 쪽으로 정리한다).
- 비동기 장애: 포트의 `pending_fault()`가 SPEC-110 이벤트(보호정지, controller heartbeat 손실, vision·tracker heartbeat 손실, 그리퍼 이상 등)를 돌려주면 실행기는 단계 실행 전과 직후에 그 이벤트로 중단 등급을 정한다(판정 `FAULT`, 이유 = 이벤트 이름). 포트는 장애가 생기면 진행 중인 대기를 바로 끝내고 `None`을 돌려준다. 단, RELEASE 응답을 받은 뒤에는 하드 이상(SPEC-110 4.1절)만 대기를 끝내고, 비하드 장애가 latch된 동안에도 RETREAT·VERIFY 응답을 기다린다(해제 뒤 비하드 장애는 `CONTINUE_TO_RETREAT_VERIFY`). 장애는 사이클이 끝날 때까지 유지된다(포트가 latch). latch된 비하드 장애는 하드 이상이 오면 그것으로 바뀌고, 하드 이상은 다른 장애로 바뀌지 않는다.
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
| `status` | 구독 캐시 | `system/heartbeat`(노드 ACTIVE·나이), `robot/status`, `belt/measured_motion`(`belt_steady` = `steady == false`가 `belt_unsteady_hold_s` 넘게 이어지지 않음, SPEC-110 REQ-ABT-013), `objects/detections.calibration_id` |
| `detect_stones` | `vision/detect_objects`(MAT, STONE) | 응답 검출. `clearance_m` = 가장 가까운 다른 검출까지 거리(없으면 1 m). M0 임시 포트의 검출 시각 = 응답 수신 시각(영상 freshness 증명 아님) |
| `pick_stone` | `robot/pick_stone` | 결과 값. `stone_on_mat` = 결과 뒤 `DetectObjects(MAT)`에 목표 10 mm 안 공깃돌이 남음. 이유 코드 `GRIPPER*` = 그리퍼 오류, 그 밖 FAULT = 로봇 오류 |
| `move_to_pose` | `robot/move_to_pose`(named_pose) | `pose_error`, holding = `robot/status.holding` |
| `select_target`·`observe_cap` | `tracking/select_target`, `objects/tracked` | track 상태·관측 수, `BeltMotion`, 병뚜껑 검출 `stone_inside`, 시간 예산(4.9절) |
| `detect_cap`(STATIC) | `vision/detect_objects`(BELT, CAP) | 정지 뚜껑 검출 |
| `compute_intercept` | `planner/compute_intercept` | `InterceptPlan` |
| `place_start`·`place_feedback` | `robot/place_stone`·`robot/dynamic_place` | phase 끝 feedback의 `metric_keys/values`(SPEC-010 wire schema v0.2 / runtime contract v0.4). SYNCHRONIZE는 진행 중 feedback의 `e_xy`·`v_rel` 열 |
| `verify` | `vision/verify_drop` | `verdict` |
| `safe_retreat`·`stop`·`cancel_active` | `robot/safe_retreat`, `robot/stop`, goal cancel | — |
| `pending_fault` | 구독 캐시 | `robot/status.protective_stop` → PROTECTIVE_STOP, `connected = false` → ROBOT_COMM_LOST, heartbeat 손실, controller `error_code` 100·101(REQ-ORC-012). SPEC-110 4.4절 구간에서 `robot/status` 그리퍼 → GRIPPER_FAULT, `BeltMotion` 지속 불안정 → BELT_UNSTEADY, 대상 track `last_observed_stamp` → OBS_STALE, ROS 시각 역행 → CLOCK_JUMP |
| skill 결과(`_result`·phase 대기) | action 결과 `reason_code` | `PROTECTIVE_STOP` → PROTECTIVE_STOP. `STOPPED`(실행 중 `Stop` 선점, SPEC-010 3.2절) → 자체 감시를 먼저 돌려 구체 원인을 latch하고, 없으면 `ROBOT_STOPPED`. latch된 `ROBOT_STOPPED`는 뒤이어 확인된 다른 하드 이상으로 바뀐다 |

검출의 실제 freshness 원점은 PLAN.md 4.3절·SPEC-000의 영상 노출 시각이다. 검출 안정 창은 `stable_window=0.5 s`로 요청한다. M0 서비스 응답에는 관측 stamp·image_seq가 없어 수신 시각을 임시 대용하며 실제 vision 경로에는 사용할 수 없다. G1 필수 C/I 시험은 응답과 영상 노출 stamp의 연결 계약, 지연 응답·오래된 cache의 PICK_STONE 거부, 0.5 s 안정 창 충족을 확인한다.

### 4.9 시간 예산

WAIT_CAP 예산은 `orchestrator_node`가 conveyor_frame의 대상 track 위치·속도로 `intercept_planner.time_budget.evaluate_budget(x_cap, v, x_exit, 성분, margin)`을 불러 정한다(식은 SPEC-400 4.2절 한 곳). `t_available`·`t_required`·성분은 그 결과이고, 성분은 설정 `budget:` 절의 `*_s`(`retreat_margin_s` 제외), `x_exit = budget.x_exit_m`, `margin = wait_cap.margin_s`, `v_min = 0`이다. 벨트가 멈췄거나 거꾸로 가면(`BELT_STOPPED`·`BELT_REVERSED`) `t_available` 0, `t_to_window` ∞라 그 통과는 `TOO_EARLY`로 기다린다. 동적 deadline은 `deadlines.dynamic_deadline(now, t_available, budget.retreat_margin_s)`다(SPEC-110 5절). `t_to_window = (x_reach_start − x_cap) / v`.

`budget.x_reach_start_m`·`budget.x_exit_m`은 같은 이름의 ROS 파라미터로 덮을 수 있고, `drop.launch.py`는 tracker·planner와 같은 도달 구간을 넣는다(SPEC-400 REQ-PLN-012).

### 4.10 포트 대기와 미확정 action

수락 future가 미완료인 action은 취소 확인으로 간주하지 않는다. 늦게 수락한 goal에도 cancel을 보내고 terminal result(status 4·5·6)를 확인한다. 이전 모션 종료가 미확정이면 대체 모션을 보내지 않는다. feedback은 goal별 token으로 격리한다. 포트 대기의 steady 예산은 진입 시 남은 ROS deadline 시간 × `watchdog.cycle_steady_factor_ratio`(기본 2.0)다(deadline 없는 호출은 1 s × 배율). ROS deadline 자체는 계속 검사한다. 포트 대기는 이 steady 예산, 사이클 steady 상한, 사용자 cancel, context 종료를 검사한다. 해제 뒤 비하드 장애의 후퇴·검증 규칙(REQ-ORC-010)은 유지한다. phase 끝 대기에서 action result가 정상(`OK`)으로 먼저 처리되고 그 phase의 `phase_done` feedback이 아직 없으면 0.2 s(ROS deadline 안)까지 더 기다린다. 정상이 아닌 result(`STOPPED` 등)는 기다리지 않는다.

phase 끝 대기가 feedback 없이 끝나면(포트가 None → `STEP_TIMEOUT`) 판정은 그대로 두고 경고 한 줄을 남긴다: phase, 경로(대기 끝 = ROS deadline·steady 상한·장애 / phase 끝 feedback 전에 결과, 이때 result 코드와 `reason_code`), 대기 시작 뒤 ROS·steady 경과, 마지막으로 받은 feedback의 phase·끝 여부·나이, latch된 장애. 실패한 단계 기록 경고(REQ-ORC-017)에는 단계 길이(`t_end − t_start`, ROS)를 붙인다.

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
| RELEASE | `1.5 × T_release(p99)`(REQ-ABT-007 식), 측정 전 2.0(실제 controller I0 fake transport 기준) |
| RETREAT | 3 |
| VERIFY | 3 |

해제 뒤 RETREAT·VERIFY에는 단계 진입 ROS 시각 + 각 단계 제한시간(각 3 s 제안값)을 주며 원 사이클·동적 deadline으로 자르지 않는다. 하드 이상·실제 후퇴 실패의 L4 우선순위는 유지한다. 포트는 context 종료·cancel·하드 이상·guard 확인 뒤 이미 준비된 응답을 steady 상한보다 먼저 인정한다. 응답이 없으면 남은 ROS 시간 × 기존 배율의 유한 steady 상한과 ROS deadline을 검사한다.

`TaskStatus`는 10 Hz로 발행한다. 실제 deadline 계산은 SPEC-110 5절을 따른다.

## 6. 실패 모드와 응답(중단 등급)

- REQ-ORC-001: 모든 판정이 OK면 모드별 상태 열이 4.1절과 정확히 같다(MODE_STATIC은 PLAN_INTERCEPT·SYNCHRONIZE 없이 WAIT_CAP에서 `DetectObjects(REGION_BELT, CLASS_CAP)` 1회 이상 호출, MODE_PREDICTED는 SYNCHRONIZE 판정 입력이 진입 시각 오차·속도 오차).
- REQ-ORC-002: 최종 실패 판정 뒤에는 정상 작업 경로의 다음 skill을 호출하지 않는다. 단 WAIT 재관측, 대기 자세의 통과 보내기 및 해제 후 RETREAT→VERIFY 정리 경로는 각 규정에 따른다. 하드 이상과 RETREAT 실패는 L4가 우선한다.
- REQ-ORC-003: 단계 판정은 3절 임계값으로 4.3절 이유 코드를 낸다. 경계값은 `TEST_SPEC.md` TC-ORC-003-01~14의 세 점(직전·경계·직후)에서 기대값과 일치한다.
- REQ-ORC-004: 관측값을 받은 judge 최종 판정의 `Judgement.metrics` 키 집합은 4.4절의 해당 모드 표와 같고 `StepRecord.metric_keys`·`metric_values`에 순서대로 그대로 복사된다(누락 0건). 관측 응답이 없는 TIMEOUT/PRECONDITION은 `metrics={}`를 허용하고 `reason_code`로 구별한다.
- REQ-ORC-005: 선행조건이 성립하지 않으면 결과 `PRECONDITION`, 해당 skill 호출 0회다(예: PICK_STONE에서 검출 시각이 1.01 s 전).
- REQ-ORC-006: DETECT_STONE·PICK_STONE은 재시도하지 않는다. 첫 실패(PICK_STONE `STONE_ON_MAT`·선행조건 불충족 포함)에서 같은 skill과 DETECT_STONE을 다시 부르지 않고 L2다.
- REQ-ORC-007: 통과 보내기는 `max_passes`회까지 한다. 소진하면 L3, `SafeRetreat(keep_sync=false, place_back=true)`, 결과 `NOT_ATTEMPTED`, `passes_skipped`를 기록한다.
- REQ-ORC-008: PLAN_INTERCEPT가 동적 deadline까지 OK가 아니면 APPROACH를 0회 호출하고, 남은 통과가 있으면 WAIT_CAP으로 돌아간다.
- REQ-ORC-009: WAIT_OPERATOR에서 사유 없는 `resume`은 거부하고, 사유 있는 `resume`은 PRECHECK부터 다시 시작하며 진입 시각·사유를 기록한다.
- REQ-ORC-011: 실행 중 skill이 `STOPPED`로 끝나면 단계 판정 대신 하드 이상(L4)으로 처리한다. 같은 시점에 자체 감시가 보호정지·통신 단절·controller heartbeat 손실을 보고 있으면 그 사유를 남기고, 원인을 모르면 `ROBOT_STOPPED`를 남긴다. 클라이언트 cancel 결과(`CANCELED`)는 하드 이상이 아니다.
- REQ-ORC-012: `robot_controller_node`의 `Heartbeat.error_code`가 100이면 `PROTECTIVE_STOP`, 101(정지 유지, SPEC-010 4.5절)이면 `ROBOT_STOPPED`를 감시 주기마다 latch한다. 실행 중 skill이 없을 때(PRECHECK, DETECT_STONE 서비스 대기, WAIT_CAP 선택·관측 대기, 단계 사이)도 L4로 끝나고 그 뒤 Stop 외 로봇 skill 호출은 0회다. 정지 유지가 풀리지 않은 채 시작한 사이클은 PRECHECK에서 L4다. `DropTask` goal을 수락한 뒤 controller heartbeat 2개와 `robot/status` 1개를 새로 받은 다음(최대 `node.startup_wait_s`) 장애 latch를 무장하므로, 해제 직후 goal은 수락 전 캐시의 101로 끝나지 않고 새 heartbeat가 101이면 PRECHECK에서 L4다.
- REQ-ORC-013: WAIT_CAP은 다른 조건이 모두 통과여도 `t_to_window_s > plan_lead_s`(3.0 s)면 `WAIT`(`TOO_EARLY`)이고 PLAN_INTERCEPT로 가지 않는다. 경계 3.001 / 3.000 / 2.999 s에서 WAIT / OK / OK다.
- REQ-ORC-018: WAIT_CAP의 `t_to_window_s`는 고른 순간 값이 아니라 매 관측의 대상 track으로 다시 계산한다. 대상의 `velocity.x`가 고른 뒤 바뀌면 `t_to_window_s`도 `(x_reach_start − x) / v`로 따라 바뀌고, `x ≥ x_reach_start`면 0, `v ≤ 0`이거나 대상이 목록에 없으면 ∞다.
- REQ-ORC-019: WAIT_CAP 관측의 `t_available_s`·`t_required_s`·`budget_<성분>_s`는 `intercept_planner.time_budget.evaluate_budget`(4.9절 입력)의 값과 같고, 동적 deadline은 `now + t_available − budget.retreat_margin_s`다. 벨트가 멈췄거나 거꾸로 가면 `t_available_s` 0, `t_to_window_s` ∞이고 판정은 `WAIT`(`TOO_EARLY`)다.
- REQ-ORC-014: `DynamicPlace.Goal.position_std_max`는 `sync.release_position_std_max_m`(0.5 mm)이고 계획 단계 기준 `plan.position_std_max_m`(3.25 mm)와 독립이다.
- REQ-ORC-015: `PlaceStone`·`DynamicPlace`처럼 여러 단계(APPROACH~RETREAT)를 한 goal이 수행할 때 `goal.deadline`은 전송 시각 + 그 goal이 거치는 단계 제한시간의 합(동적: `approach_s + synchronize_s + descend_s + release_s + retreat_s` = 11.5 s, STATIC: SYNCHRONIZE 제외 9.5 s)이다. APPROACH 단계 deadline을 넣지 않는다. 단계·동적·사이클 deadline은 실행기가 포트 대기와 cancel로 집행한다. 대역 소요시간 축척 1.0(`DynamicPlace` 3.0 s)에서 한 사이클이 SUCCESS다.
- REQ-ORC-016: WAIT_CAP 대기 중 관측이 `target_lost`이면 `passes_skipped`를 올리지 않고 같은 `pass_index`로 `select_target`을 다시 호출한다. 실제 tracker가 WAIT_CAP 도중 초기화(epoch +1)되어도 그 통과에서 사이클이 SUCCESS로 끝난다.
- REQ-ORC-017: WAIT_CAP이 대기(`WAIT`) 중에 통과 제한·사이클 제한·장애로 끝나면 그 기록에 마지막 대기 판정의 지표와 사유(`detail`, 예 `OBSERVATION_COUNT`·`TOO_EARLY`·`TRACK_NOT_OBSERVED`)를 남긴다. 대상을 한 번도 얻지 못했으면 `detail`은 `NO_TARGET`이고 지표는 없다. `orchestrator_node`는 실패한 단계 기록마다 단계·사유·`detail`·지표를 WARN 로그로 남긴다.
- REQ-ORC-020: phase 끝 대기가 feedback 없이 끝나면 `orchestrator_node`는 4.10절 경고를 정확히 한 줄 남긴다. controller 결과가 먼저 끝난 경우 그 result 코드와 `reason_code`(예 `GATE_TRACK_STALE`)가, 대기가 끝난 경우 ROS·steady 경과가 들어가며, 단계 판정·중단 등급은 바뀌지 않는다.
- REQ-ORC-010: 포트 대기는 해제 전에는 latch된 모든 장애로, RELEASE 응답 뒤에는 하드 이상으로만 끝난다. latch된 비하드 장애는 하드 이상으로 바뀌고, 하드 이상은 비하드 장애로 바뀌지 않는다.

## 7. 성공 판정(측정 가능)

- U 시험(TC-ORC-*) 전부 통과, `trace.py --area ORC` 누락 0.
- M0: I0 `nominal_20mms` 20/20, 오케스트레이터 REQ 연결률 100%.
- G2: 실패한 단계 뒤에 다음 단계가 실행된 사례 0회, 단계 판정과 관찰자 기록 불일치 ≤ 5%.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-ORC-001-01 ~ TC-ORC-012-02(U, M0), TC-ORC-013-01~TC-ORC-019-01(U, G4), TC-ORC-020-01(U, G4, `test_phase_miss_log.py`). ROS 수준은 TC-INT-001-01, TC-INT-005-01, TC-INT-006-01, TC-INT-008-01, TC-INT-009-01(실제 tracker·planner, REQ-ORC-013), TC-INT-010-01(실시간 축척, REQ-ORC-015), TC-INT-011-01(대상 재선택, REQ-ORC-016), TC-INT-013-01(MODE_STATIC 전체 상태 열, REQ-ORC-001).

## 9. 미결 사항

- `clearance_min_m`, `safe_z_m`: TBD(현장 배치 OFF-1).
- `grip_current_min_ma`, `gripper_response_max_s`, `t_approach_s`: TBD(측정 G0).
- `entry_time_tol_s`: TBD(측정 G4).
