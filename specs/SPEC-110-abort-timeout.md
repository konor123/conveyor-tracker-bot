# SPEC-110 abort-timeout

- 버전: v0.9
- 근거: PLAN.md 5, 6절
- REQ 영역: ABT

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.9 | 2026-11-11 | `ROBOT_STOPPED`에 controller 정지 유지(heartbeat 101) 포함, 재개 순서(로봇 해제 → `resume`) |
| v0.8 | 2026-11-10 | 하드 이상에 `ROBOT_STOPPED`(실행 중 skill이 Stop으로 끝남) 추가 |
| v0.7 | 2026-11-10 | G7 판정에서 실기 고장 주입 제외 |
| v0.6 | 2026-11-10 | L1(재시도) 제거, `AbortCtx`를 상태·holding·released로 축소 |
| v0.5 | 2026-11-09 | 해제 후 cleanup deadline 예외 |
| v0.4 | 2026-11-09 | 포트 대기 steady 배율과 독립 중단 상한 정합화 |
| v0.3 | 2026-11-09 | 포트 대기 steady 상한·Stop 전달 확인과 재시도 계약 |
| v0.2 | 2026-11-09 | M0 검증 범위·게이트 및 독립 계약 기대값 정합성 |
| v0.1 | 2026-11-07 | 중단 등급 판정 순서, 승격, 중단 경로·cancel 제한시간, deadline, 시계 정책, 감시 임계값 |

## 1. 목적·범위

오케스트레이터와 `safety_supervisor_node`가 쓰는 중단 등급 판정(`abort_policy.py`), 제한시간 계산(`deadlines.py`), 감시 타이머(`watchdogs.py`)를 정한다. L0(지령 freshness)는 `robot_control`이 소유하며 SPEC-500에서 정한다.

M0는 감시 판정 함수와 이벤트 입력의 U 검증 범위다. ROS 관측 신선도(G3/G4)·시각 역행 무효화(G3)·그리퍼(G0)·벨트 BELT_UNSTEADY 이벤트(G3)의 노드 연결은 해당 게이트 필수 조건이며 실제 경로 사용 전에 C/I 시험을 통과한다. 연결 소유자는 orchestrator_node._watch와 tracking/reset·planner 무효화 경로이며 supervisor의 독립 L4 역할은 유지한다.

## 2. 입력·출력(interface 버전)

- 입력: 이벤트(`Event`)와 중단 문맥(`AbortCtx`: 상태, holding, released), steady 시각, ROS 시각, `Heartbeat`(SPEC-010 wire schema v0.2 / runtime contract v0.4), 그리퍼 상태.
- 출력: `AbortLevel`(`NONE=0`, `L2~L4`, `CONTINUE_TO_RETREAT_VERIFY=10`. 값 1은 쓰지 않는다), `TaskStatus.abort_level`, `StepRecord.abort_level`(SPEC-010 wire schema v0.2 / runtime contract v0.4). `CONTINUE_TO_RETREAT_VERIFY`는 내부 값이며 메시지에는 0으로 쓴다.

## 3. 파라미터(단위·기본값·근거)

`src/orchestrator/config/orchestrator.yaml`의 `abort:`·`timeouts:`·`watchdog:` 절이 원본이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `abort_path_l2_s` | 10.0 | PLAN.md 5.2절 제안값, G0·G2 측정으로 확정 |
| `abort_path_l3_s` | 15.0 | 같음 |
| `cancel_response_s` | 0.5 | PLAN.md 6.1절 고정값 |
| `call_timeout_s` | 1.0 | PLAN.md 6.1절 고정값 |
| `heartbeat_timeout_s` | 0.3 | 10 Hz 3회 누락 |
| `obs_freshness_s`(τ_max) | 0.3 | PLAN.md 4.4절 초기값, G3·G4 측정으로 확정 |
| `gripper_response_max_s` | 0.5 | PLAN.md 4.3절, G0 측정으로 확정 |
| `gripper_alive_stall_s` | 0.1 | PLAN.md 5.3절, G0 측정으로 확정 |
| `cycle_timeout_s` | 90.0 | `DropTask.timeout` 기본값 |
| `cycle_steady_factor_ratio` | 2.0 | PLAN.md 6.2절 |

경계 비교는 부동소수점 허용오차 1e-9 s(또는 m)로 한다. "이내·이하" 판정은 `x ≤ 임계 + 1e-9`, "이상·초과 검출"은 각 REQ에 적힌 부등호를 같은 허용오차로 적용한다.

## 4. 동작(상태·알고리즘)

### 4.1 판정 순서

`decide(event, ctx)`는 다음 순서로 첫 번째로 맞는 등급을 낸다.

1. `HARD_FAULTS`(보호정지, 비상정지, 로봇 통신 단절, `robot_controller` heartbeat 손실, 중단 경로 제한시간 초과, RETREAT 실패, 실행 중 skill이 `Stop`으로 끝남 또는 controller가 `Stop` 정지 유지 중(heartbeat `error_code` 101, SPEC-010 4.5절) `ROBOT_STOPPED`) → L4
2. `ctx.released` → `CONTINUE_TO_RETREAT_VERIFY`
3. `ctx.holding` 또는 `ctx.state ∈ DYNAMIC_STATES`(APPROACH, SYNCHRONIZE, DESCEND, RELEASE) 또는 이벤트가 통과 보내기 소진 → L3
4. 그 외 → L2(단계 실패는 재시도하지 않는다)

### 4.2 승격

- 심각도 순서: `NONE < L2 < L3 < CONTINUE_TO_RETREAT_VERIFY < L4`. `escalate(current, new)`는 둘 중 심각한 쪽을 낸다.
- 중단 경로가 시작되면(L2·L3) steady 시계로 경과를 잰다. 제한시간을 넘으면 L4.
- cancel을 보낸 뒤 `cancel_response_s` 안에 응답이 없으면 한 단계 올린다(L2→L3, L3→L4).
- WAIT_OPERATOR에는 제한시간이 없다. `Stop`을 받은 controller는 작업자가 정지 유지를 해제(`stop_release_reason`)할 때까지 지령을 실행하지 않으므로, 재개는 로봇 해제 뒤 `resume` 순서다(SPEC-010 4.5절, REQ-ORC-012).

### 4.3 감시

| 감시 | 시계 | 검출 조건 | 이벤트 |
|---|---|---|---|
| 기능 노드 heartbeat | steady | 마지막 수신 후 `≥ heartbeat_timeout_s` | `HEARTBEAT_LOST`(vision·tracker), `CONTROLLER_HEARTBEAT_LOST`(robot_controller) |
| 관측 신선도(동적 구간) | ROS | `t_now − t_last_obs > obs_freshness_s` | `OBS_STALE` |
| ROS 시각 역행 | ROS | 직전 값보다 작음 | `CLOCK_JUMP`(track·plan 무효화) |
| 벨트 속도 급변 | — | `BeltMotion.steady == false` | `BELT_UNSTEADY` |
| 그리퍼 | steady | 오류 비트 ≠ 0, 응답 `> gripper_response_max_s`, 생존 카운터 정지 `> gripper_alive_stall_s` | `GRIPPER_FAULT` |
| 사이클 | ROS + steady | ROS 시각 `≥` 사이클 deadline 또는 steady 경과 `≥ timeout × cycle_steady_factor_ratio` | `CYCLE_TIMEOUT` |

## 5. 시간 제약(주기·제한시간)

- 해제 후 RETREAT·VERIFY는 원 사이클·동적 deadline과 독립적으로 각 단계 진입 ROS 시각 + 기존 단계 제한시간(각 3 s 제안값)을 쓴다. 각 포트 대기는 남은 ROS 시간 × cycle_steady_factor_ratio의 유한 steady 상한을 유지한다. 하드 이상·실제 RETREAT 실패는 L4다.
- 그 외 실제 deadline = `min(now + 단계 제한시간, 동적 deadline, 사이클 deadline)`. 동적 deadline이 없으면 빼고 계산한다.
- 위 deadline은 실행기의 단계 대기에 쓴다. 여러 단계를 수행하는 goal에 싣는 서버 `deadline`은 goal 전체 상한이다(SPEC-100 REQ-ORC-015).
- 측정 기반 단계 제한시간 = `max(ceil(1.5 × p99), p95 + 0.5)` [s].
- 시계 정책: ROS 시각은 메시지 stamp, `valid_until`, `t_last_obs`, 시간 예산, 단계·동적·사이클 deadline에 쓴다. steady 시계는 heartbeat, 그리퍼 응답, 호출 제한시간, cancel 응답, 중단 경로 제한시간, 사이클 steady 상한에 쓴다.

## 6. 실패 모드와 응답(중단 등급)

- REQ-ABT-001: `decide()`는 4.1절 순서를 따른다. 기대표 `src/orchestrator/test/data/decide_table.csv`의 모든 행과 일치해야 한다(불일치 0건).
- REQ-ABT-002: 높은 등급은 낮은 등급을 덮어쓰고 낮은 등급은 높은 등급을 덮어쓰지 못한다(L3 진행 중 L4 → L4, L4 진행 중 L2 → L4).
- REQ-ABT-003: L2 경로는 steady 10.000 s, L3 경로는 15.000 s까지 유지하고 그보다 1 ms 뒤에는 L4로 승격한다.
- REQ-ABT-004: cancel 후 0.5 s 안에 응답이 없으면 한 단계 승격한다.
- REQ-ABT-005: 공깃돌을 들고 있을 때 사용자 cancel은 L3로 처리한다.
- REQ-ABT-006: 해제 후 RETREAT·VERIFY는 위 독립 단계 예산을 쓰며, 그 외 실제 deadline은 단계·동적·사이클 deadline의 최솟값이고, 동적 deadline이 없으면 나머지 둘의 최솟값이다.
- REQ-ABT-007: 측정 기반 단계 제한시간은 `max(ceil(1.5 × p99), p95 + 0.5)` s다((1.0, 1.2) → 2.0 s, (2.0, 2.1) → 4.0 s).
- REQ-ABT-008: `/clock`이 멈춰도 steady 경과가 `goal.timeout × 2`에 이르면 사이클을 중단한다(들고 있지 않으면 L2, 들고 있으면 L3).
- REQ-ABT-009: 감시 타이머는 steady 시계를 쓴다. ROS 시각이 5 s 역행해도 heartbeat 오검출 0건, ROS 시각이 멈춰도 0.3 s 끊김을 검출한다.
- REQ-ABT-010: heartbeat는 마지막 수신 후 0.300 s에 손실로 검출한다(0.299 s는 유지). vision·tracker 손실은 동적 구간이면 L3, 그 외 L2, robot_controller 손실은 L4다.
- REQ-ABT-011: 동적 구간에서 `t_now − t_last_obs`가 0.300 s 이하면 유지하고 0.301 s면 해제 전 L3다.
- REQ-ABT-012: ROS 시각이 1회라도 역행하면 epoch를 증가시키고 기존 track을 제거하며 기존 plan 재사용을 거부하고, 들고 있으면 L3, 아니면 L2다.
- REQ-ABT-013: `BeltMotion.steady == false`이면 해제 전에는 L3, 해제 후에는 `CONTINUE_TO_RETREAT_VERIFY`다.
- REQ-ABT-014: 그리퍼 오류 비트 ≠ 0 또는 응답 0.501 s 이상이면 들고 있을 때 L3, 아닐 때 L2다(응답 0.500 s는 정상).
- REQ-ABT-015: `PLAN.md` 7.3절 장애 10종의 이벤트 열마다 최종 등급이 `src/orchestrator/test/data/fault_expectations.yaml`과 같다(10/10).
- REQ-ABT-016: WAIT_OPERATOR는 steady 1 h가 지나도 상태를 유지하고 승격하지 않는다.

## 7. 성공 판정(측정 가능)

- U 시험(TC-ABT-*) 전부 통과, `trace.py --area ABT` 누락 0.
- M0: 장애 10종 이벤트 열마다 기대 등급(10/10).
- G7: 100 사이클에서 중단 경로 제한시간 초과 0회.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-ABT-001-01 ~ TC-ABT-016-01(U, M0). ROS 수준은 TC-ABT-009-02(heartbeat·감시 타이머 시계), TC-INT-002-01(cancel 0.5 s), TC-INT-003-01(`Stop`), TC-INT-004-01(heartbeat 300 ms).

REQ-ABT-011~014의 U 판정·이벤트 입력은 M0다. TC-ABT-011-02는 G3/G4 관측 ROS 연결, TC-ABT-012-02는 G3 역행 무효화, TC-ABT-013-02는 G3 벨트 ROS 이벤트, TC-ABT-014-02는 G0 그리퍼 ROS 연결의 C/I 필수 시험이다.

### 8.1 노드 대기·Stop 전달 계약

포트 대기의 자체 steady 상한은 진입 시 남은 ROS 시간 × `cycle_steady_factor_ratio`다(SPEC-100 4.10절). 대기 중에도 REQ-ABT-003·008의 독립 steady 예산을 검사한다. cancel 응답·Stop 호출·L2/L3 중단 경로 제한시간에는 이 배율을 적용하지 않는다. 중단 경로는 사용자 cancel을 재적용하지 않으며 독립 steady 상한을 따른다. context 종료는 하드 중단으로 처리한다. 미확정 action 수락·terminal result는 cancel 확인이 아니며 대체 모션을 금지한다.

Supervisor는 장애 latch와 Stop 전달 확인을 분리한다. `stopped=true` 전에는 0.1 s 이상 간격으로 재시도하며 서비스 부재·호출 예외·`stopped=false`를 경고한다. 진행 중 Stop RPC는 최대 1개다. 응답이 없는 RPC가 남으면 중복 요청을 보내지 않고 1 s 간격으로 미확인 상태를 경고한다. Stop만 호출하는 단일 지령 권한은 유지한다.

## 9. 미결 사항

- `abort_path_l2_s`·`abort_path_l3_s`: TBD(측정 G0·G2).
- `gripper_response_max_s`·`gripper_alive_stall_s`: TBD(측정 G0).
- `obs_freshness_s`(τ_max): TBD(측정 G3·G4).
