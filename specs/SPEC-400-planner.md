# SPEC-400 planner

- 버전: v0.1
- 근거: PLAN.md 3.1, 4.3, 4.4, 9절
- REQ 영역: PLN

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-11-09 | 예측·공분산 전파, 시간 예산, 해제 시각 선택, 도달성, 상태 우선순위, plan_id·revision·만료, τ_max |

## 1. 목적·범위

`intercept_planner` 패키지가 병뚜껑 track에서 해제 시각·해제 직전 TCP 목표·동기 속도를 계산하고, 시간 예산과 도달성을 판정하는 규칙을 정한다. 판단 로직은 순수 Python 모듈(`time_budget.py`, `prediction.py`, `intercept.py`, `plan_session.py`, `tau.py`)이고 `intercept_planner_node`는 메시지 변환·서비스·발행·Heartbeat만 맡는다.

- 대상 모드: `MODE_PREDICTED`, `MODE_SYNCHRONIZED`. `MODE_STATIC`은 planner를 쓰지 않는다(SPEC-100).
- 시간 예산 식은 `time_budget.py` 한 곳에 둔다. 오케스트레이터 `deadlines.py`와 WAIT_CAP 판정은 이 모듈을 쓴다(PLAN.md 9절).

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4 기준.

- 입력: `objects/tracked`(`TrackedObjectArray`, `conveyor_frame`), `belt/measured_motion`(`BeltMotion`), static TF `robot_base → conveyor_frame`(파라미터 `T_base_conveyor`로 받는다), 파지 오프셋(SPEC-600 `tcp_offset`), `planner/compute_intercept` 요청.
- 출력: `ComputeIntercept` 응답의 `InterceptPlan`, `drop/intercept_plan`(같은 계획의 갱신, revision 증가), `drop/predicted_marker`(시각화), `system/heartbeat`.
- 순수 API는 `TEST_SPEC.md` 2절 intercept_planner 절을 따른다. `evaluate_budget`은 키워드 인자 `v_min`(기본 0.0)을 받고 `Budget`에 `reason: str`(통과 시 빈 문자열)을 둔다.

## 3. 파라미터(단위·기본값·근거)

`src/intercept_planner/config/planner.yaml`이 원본이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `budget.x_exit_m` | 0.30 | 관측 구간 ∩ 로봇 안전 도달 구간의 하류 끝(`conveyor_frame`). TBD(배치도 OFF-1) |
| `budget.margin_s` | 1.0 | PLAN.md 9절 |
| `budget.v_min_mps` | 0.002 | SPEC-300 `belt.v_min_mps`와 같은 값 |
| `budget.t_transit_s` | 1.5 | TBD(측정 G0, skill 소요시간 p95) |
| `budget.t_sync_s` | 0.5 | `sync.hold_s` 0.3 s + 여유. TBD(측정 G5) |
| `budget.t_descend_s` | 0.8 | TBD(측정 G0) |
| `budget.t_release_s` | 0.3 | `T_cmd + Δ_open / v_open + T_fall`. TBD(측정 G0) |
| `budget.t_clear_s` | 0.9 | 해제 후 후퇴 시작까지 동기 유지. TBD(측정 G0) |
| `reach.x_reach_start_m` | −0.05 | SPEC-300 `select.x_reach_start_m`과 같은 값. TBD(배치도 OFF-1) |
| `reach.workspace_min_m` | [0.20, −0.40, 0.00] | `robot_base` 기준 해제 TCP 허용 상자 하한. TBD(배치도 OFF-1) |
| `reach.workspace_max_m` | [0.70, 0.40, 0.40] | 상한. 같음 |
| `target.release_tcp_z_m` | 0.10 | 해제 직전 TCP 높이(`robot_base`). 핑거 끝 = 테두리 + `h_clear`가 되도록 TBD(보정 OFF-3, 핑거 어댑터 OFF-4) |
| `target.tool_quat_xyzw` | [1, 0, 0, 0] | 수직 하향 고정 자세. TBD(G0) |
| `target.tcp_offset_m` | [0.0, 0.0] | 공깃돌 중심 − TCP(XY, `robot_base`). TBD(측정 OFF-4, `tcp_offset.py`) |
| `uncertainty.position_std_max_m` | 0.0005 | PLAN.md 4.3절 |
| `T_base_conveyor` | 단위 행렬 | `robot_base ← conveyor_frame` 4×4. TBD(보정 OFF-3) |

`T_required = t_transit + t_sync + t_descend + t_release + t_clear`(제안값 합 4.0 s).

## 4. 동작(상태·알고리즘)

### 4.1 예측

- `predict(track, t)`: `dt = t − t_state`, `p(t) = p + v·dt`. 위치 공분산은 `Σ = P_pp + dt·(P_pv + P_vp) + dt²·P_vv`(축별로 `σ² = P_xx + 2·dt·P_xv + dt²·P_vv`).
- 지연 보상은 `t − t_state`에 모두 들어 있다. 노드 처리 지연·통신 지연을 따로 더하지 않는다.
- `position_std = √λ_max(Σ)`(`release_time`에서의 값).

### 4.2 시간 예산

```
T_available = (x_exit − x_cap) / v_x            # v_x: 진행 방향 속도 성분
T_required  = Σ components
time_margin = T_available − T_required
ok          = time_margin ≥ margin
```

- `v_x < 0`이고 `v_x ≤ −v_min`이면 `ok = false`, `reason = BELT_REVERSED`. 그 밖에 `v_x ≤ v_min`이면(0 포함) `ok = false`, `reason = BELT_STOPPED`. 예산 미달은 `reason = TIME_MARGIN`. `T_available`은 정지·역방향일 때 0으로 둔다.
- `margin`은 `T_required`에 넣지 않고 판정에서 한 번만 쓴다.

### 4.3 해제 시각·목표

1. `t_lead = t_transit + t_sync + t_descend`.
2. 도달 구간 진입 시각 `t_reach = t_state + (x_reach_start − x) / v_x`(이미 구간 안이면 `t_state`).
3. `release_time = max(now + t_lead, earliest_release, t_reach)`.
4. `x_cap`은 `now` 시점 예측 위치다. earliest_release로 해제가 늦춰지면 실효 필요 시간은 `max(T_required, (release_time − now) + t_release + t_clear)`이고 `time_margin`은 이 값으로 계산한다.
5. 해제 위치 `p_rel = predict(track, release_time)`를 `T_base_conveyor`로 `robot_base`에 옮긴다. `target_tcp_pose.position.xy = p_rel.xy − tcp_offset`, `z = release_tcp_z_m`, orientation = `tool_quat_xyzw`.
6. `target_velocity = R_base_conveyor · [vx, vy, 0]`.
7. `valid_until = release_time + t_release + t_clear`(해제 창 끝). 이후 갱신이 없거나 track이 끝나면 4.5절 규칙으로 무효화한다.

### 4.4 판정 순서

요청 검증(`ComputeIntercept.result`)과 계획 상태(`InterceptPlan.status`)를 구분한다.

| 순서 | 조건 | result / reason_code | plan.status / reason |
|---|---|---|---|
| 1 | `deadline` 경과 | `TIMEOUT` | — |
| 2 | `mode == MODE_STATIC` 또는 알 수 없는 mode | `PRECONDITION` / `MODE_UNSUPPORTED` | — |
| 3 | `target_epoch ≠` 현재 epoch | `FAILED` / `EPOCH_MISMATCH` | — |
| 4 | track 없음 | `FAILED` / `TRACK_NOT_FOUND` | — |
| 5 | track `LOST` 또는 `now > track.valid_until` | `FAILED` / `TRACK_LOST` | `EXPIRED` |
| 6 | 예산 불통과 | `FAILED` / reason | `TIME_INSUFFICIENT` / `BELT_STOPPED`·`BELT_REVERSED`·`TIME_MARGIN` |
| 7 | 해제 TCP가 workspace 상자 밖 또는 `x_rel > x_exit` | `FAILED` / `UNREACHABLE` | `UNREACHABLE` |
| 8 | `position_std > position_std_max` | `FAILED` / `UNCERTAIN` | `UNCERTAIN` |
| 9 | 그 외 | `OK` / `""` | `OK` |

경계는 `≥`·`≤` 쪽을 통과로 본다(예산 `time_margin = margin`은 통과, `position_std = position_std_max`는 통과).

### 4.5 계획 세션(`plan_id`·`revision`)

- `ComputeIntercept` 호출마다 새 `plan_id`(노드 기동부터 단조 증가)를 만들고 `revision = 1`로 응답·발행한다. 이전 활성 계획은 대체된다.
- 활성 계획의 대상 track이 새 상태로 발행되면 같은 `release_time`으로 다시 계산해 `revision += 1`로 발행한다. `plan_id`, `target_track_id`, `target_epoch`, `release_time`은 바꾸지 않는다. 상태가 OK가 아니면 그 상태로 발행한다.
- `now > valid_until`, epoch 변경, track `LOST`가 되면 `status = EXPIRED`로 한 번 발행하고 계획을 닫는다. 닫힌 계획은 다시 갱신하지 않는다.

### 4.6 τ_max

- `e_pred(τ) = e_pos + |δv|·τ + ½·a·τ² + |v|·δt`. `tau_max`는 `e_pred(τ) ≤ e_allow`를 만족하는 최대 τ다. `a > 0`이면 양의 근, `a = 0`이고 `δv ≠ 0`이면 선형 해, 둘 다 0이면 `inf`, `e_pos + |v|·δt > e_allow`이면 0.
- 결과는 해제 게이트 `tau_max`와 SPEC-110 `obs_freshness_s` 결정 근거로 쓴다(PLAN.md 4.4절).

### 4.7 노드

- `intercept_planner_node`는 Managed Lifecycle 노드이며 SPEC-010 4.3절 비활성 응답을 따른다. 시각은 ROS 시각(`use_sim_time` 지원)이며 `/clock` 역행 시 활성 계획을 `EXPIRED`로 닫는다.

## 5. 시간 제약(주기·제한시간)

- `ComputeIntercept` 응답 p99 ≤ 50 ms. PLAN_INTERCEPT 단계 제한시간 0.5 s(SPEC-100) 안에 끝나야 한다.
- 계획 갱신 발행은 track 갱신마다(30 Hz), 처리 p99 ≤ 5 ms. 마커 10 Hz.

## 6. 실패 모드와 응답(중단 등급)

| 실패 | planner 응답 | 오케스트레이터 쪽 처리 |
|---|---|---|
| 시간 부족·벨트 정지·역방향 | `TIME_INSUFFICIENT` | 이번 통과 보내기, `max_passes` 소진 시 L3(NOT_ATTEMPTED) |
| 도달 불가 | `UNREACHABLE` | 통과 보내기 |
| 불확실 | `UNCERTAIN` | deadline 안에서 재계획, 넘으면 통과 보내기 |
| epoch 불일치·track 소실 | `FAILED` | 통과 보내기, 동적 단계면 L3 |
| 만료 | `EXPIRED` 발행 | controller `PLAN_INVALID`, L3 |

## 7. 성공 판정(측정 가능)

- REQ-PLN-001: `evaluate_budget`은 PLAN.md 9절 식으로 `t_available`·`t_required`(margin 미포함)를 1e-9 이내로 계산하고, `t_available − t_required ≥ margin`일 때만 통과하며, 동적 모드에서 `v ≤ v_min`이면 `BELT_STOPPED`, 역방향이면 `BELT_REVERSED`로 불통과한다.
- REQ-PLN-002: 예측은 `t_state`를 기준으로 하며 지연을 중복 보상하지 않는다(위치 오차 1e-9 이내).
- REQ-PLN-003: 예측 위치 분산은 축마다 `P_xx + 2·dt·P_xv + dt²·P_vv`와 1e-12 이내로 일치한다.
- REQ-PLN-004: `target_tcp_pose`의 XY는 예측 뚜껑 중심 − `tcp_offset`과 1e-9 이내로 일치한다.
- REQ-PLN-005: 해제 TCP가 workspace 상자 밖이면 status `UNREACHABLE`이다.
- REQ-PLN-006: 같은 계획을 갱신하면 `plan_id`는 그대로이고 revision이 1씩 단조 증가하며, `valid_until`이 지나면 `EXPIRED`를 발행한다.
- REQ-PLN-007: 요청 epoch가 현재 epoch와 다르면 result `FAILED`, reason `EPOCH_MISMATCH`다.
- REQ-PLN-008: 해제 시각 예측 표준편차가 `position_std_max`(0.5 mm)를 넘으면 status `UNCERTAIN`이다.
- REQ-PLN-009: `release_time ≥ earliest_release`이다.
- REQ-PLN-010: `tau_max`는 4.6절 식을 만족하는 최대 τ를 해석해와 1e-6 s 이내로 돌려준다.
- REQ-PLN-011: fixture tracked 입력 → 실제 planner(I2)에서 정상·`time_insufficient`·`UNCERTAIN`·만료·epoch 불일치·clock reset 시나리오의 status·reason이 기대표와 모두 일치한다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- U: TC-PLN-001~005·008~010(`intercept_planner/test/`). C: TC-PLN-006·007(`intercept_planner/test/contract/`). I2: TC-PLN-011.
- 게이트: G4(PLN 전부), G6(PLN 전부). 예측 오차 실측은 TC-BEN-011·TC-EXP-G4.

## 9. 미결 사항

- 시간 예산 성분값: TBD(측정 G0 skill 소요시간, G4).
- `x_exit`·`x_reach_start`·workspace 상자·`release_tcp_z_m`: TBD(배치도 OFF-1, 보정 OFF-3, 핑거 어댑터 OFF-4).
- `MODE_PREDICTED`에서 해제 구간 속도 일치 시작 시각(`release_time − t_sync`)과 controller 계약: TBD(SPEC-500 `DynamicPlace`, G4).
