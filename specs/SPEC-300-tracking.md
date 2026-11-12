# SPEC-300 tracking

- 버전: v0.5
- 근거: PLAN.md 2.5, 3.1, 4.3, 5.3, 7절
- REQ 영역: TRK

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.5 | 2026-11-12 | 관측 구간 밖 검출은 새 track을 만들지 않음(REQ-TRK-014), 시각 역행 초기화 WARN 로그(REQ-TRK-008) |
| v0.4 | 2026-11-10 | `coast.max_s` 근거를 가림 시간 분포에서 가림 없는 배치의 검출 누락 구간으로 |
| v0.3 | 2026-11-09 | `sigma_accel_mps2` 0.01(OFF-1 벨트 영상), REQ-TRK-001 정확도 기준, 병뚜껑 2개·가림 없는 배치 |
| v0.2 | 2026-11-09 | `tracker_node`: `T_base_conveyor` 파라미터, 입력 중단 시 빈 프레임 처리(`input_timeout_s`), 프레임 공백 뒤 coast 한도 초과 track의 연관 금지 |
| v0.1 | 2026-11-09 | 등속 칼만 필터, 연관·재획득, 상태 전이, 통과별 track, 속도 변화 검출, 시각 역행, `BeltMotion`·`SelectTarget`·`ResetTracking`·관측 TF |

## 1. 목적·범위

`tracking` 패키지가 벨트에 붙은 병뚜껑을 `conveyor_frame`에서 추적하고, 벨트 속도를 추정하며, 오케스트레이터에 다음 통과 대상을 알려 주는 규칙을 정한다. 판단 로직은 ROS와 무관한 순수 Python 모듈(`kalman.py`, `association.py`, `speed_change.py`, `track_manager.py`, `belt_motion.py`, `select_target.py`)이고 `tracker_node`는 메시지 변환·TF·서비스·Heartbeat만 맡는다(PLAN.md 0.2절).

- 대상 class: `CLASS_CAP`. 벨트에는 병뚜껑 2개가 붙어 있고 관측 구간에는 한 번에 하나씩 들어온다(연관은 여러 track을 지원한다). 컨베이어는 해제 자세의 그리퍼가 카메라 시야에서 뚜껑을 가리지 않는 위치에 둔다(PLAN.md 1.1절). 공깃돌·벨트 표식 추적은 범위 밖이다(벨트 표식은 Progress.md 7절 선택 실험에서 정한다).
- 미래 시각 위치 예측은 하지 않는다. tracker는 상태 시각 `t_state`의 상태와 공분산만 내고, 해제 시각 예측은 SPEC-400 planner가 한다.

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4 기준.

- 입력: `objects/detections`(`ObjectDetectionArray`, `robot_base`, stamp = 노출 시각), `robot_base ← conveyor_frame` 변환(파라미터 `T_base_conveyor`, 4×4 row-major 16개. 같은 값을 static TF로 발행하는 쪽은 bringup이며 SPEC-400 planner와 같은 값을 쓴다), `/clock`(sim·replay) 또는 시스템 ROS 시각.
- 출력: `objects/tracked`(`TrackedObjectArray`, `conveyor_frame`, stamp = `t_state`), `belt/measured_motion`(`BeltMotion`), 동적 TF `conveyor_frame → cap_<track_id>`, `system/heartbeat`.
- 서비스: `tracking/reset`(`ResetTracking`), `tracking/select_target`(`SelectTarget`).
- 순수 모듈 API는 `TEST_SPEC.md` 2절 tracking 절을 따른다. 위치·속도는 `conveyor_frame`의 XY 성분만 다룬다(z는 벨트 평면 + 테두리 높이로 고정, 출력 `position.z`는 관측 평균).

## 3. 파라미터(단위·기본값·근거)

`src/tracking/config/tracking.yaml`이 원본이다. 배치 관련 값은 OFF-1 배치도·OFF-3 보정 전 제안값이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `kf.sigma_accel_mps2` | 0.01 | 이산 백색 가속도 모델 표준편차. OFF-1 벨트 영상 5조건 재생(`records/bench/belt-video-2026-11-07/`)에서 0.1 s 앞 예측 XY p95 2.6~3.4 mm(0.001은 3.1~3.8 mm). 합성 등속 시험에서 위치 p95 최대 0.334 mm, 속도 p95 최대 1.16 mm/s. `records/param_changes.csv` |
| `kf.meas_std_default_m` | 0.0003 | 검출 `position_cov`가 비었거나 양의 정부호가 아닐 때 쓰는 측정 표준편차(TEST_SPEC 3.6절 합성 조건) |
| `kf.init_vel_std_mps` | 0.05 | 새 track 초기 속도 표준편차. 최고 벨트 속도 후보의 1배 수준 |
| `assoc.gate_chi2` | 100.0 | 정상 상태 혁신 표준편차 약 0.31 mm에서 gate 반경 약 3.1 mm(병뚜껑 간격보다 충분히 작다). χ²(2) 99%(9.21, 반경 약 0.95 mm)는 합성 시험에서 측정 이상치와 속도 계단 때 새 track을 만들어 TC-TRK-006-01 불통과. 여러 뚜껑이 가까운 배치는 TBD(OFF-5) |
| `coast.max_s` | 1.0 | 관측 없음이 이 시간을 넘으면 `LOST`(TEST_SPEC TC-TRK-004-01) |
| `region.x_obs_min_m` | −0.35 | 관측 구간 상류 끝. 이 구간 밖 검출은 새 track을 만들지 않는다. TBD(배치도 OFF-1, 보정 OFF-3) |
| `region.x_obs_max_m` | 0.35 | 관측 구간 하류 끝. 같음 |
| `region.exit_margin_m` | 0.01 | 예측 위치가 `x_obs_max + exit_margin`을 넘으면 통과 종료(`LOST`) |
| `speed_change.cusum_k` | 1.0 | 정규화 혁신값 CUSUM 허용 편차. 0.5는 등속 오검출. 1.0·`h` 8에서 계단 검출 지연 최대 0.167 s, 60 s 등속 오검출 0(seed 20개) |
| `speed_change.cusum_h` | 8.0 | CUSUM 경보 임계. TC-TRK-006-01·02로 정한 값, TBD(OFF-5 실측 bag) |
| `speed_change.recover_s` | 2.0 | 마지막 검출 뒤 이 시간 동안 재검출이 없으면 `steady = true` |
| `belt.window_max_s` | 2.0 | `BeltMotion.window_sec` 상한 |
| `belt.v_min_mps` | 0.002 | 이 속력 이하이면 벨트 정지로 본다(SPEC-400과 같은 값) |
| `select.x_reach_start_m` | −0.05 | 로봇 도달 구간 상류 끝. TBD(배치도 OFF-1) |
| `T_base_conveyor` | 단위 행렬 | `robot_base ← conveyor_frame` 4×4. TBD(보정 OFF-3, 벨트 진행 방향 측정) |
| `input_timeout_s` | 0.2 | 마지막 검출 수신 뒤 이 시간(ROS 시각)이 지나면 노드가 30 Hz steady 타이머로 현재 ROS 시각의 빈 프레임을 넣어 `COASTING → LOST`를 진행시킨다 |
| `clock.backstep_tol_s` | 0.0 | ROS 시각이 이 값보다 크게 역행하면 초기화(SPEC-110 `ClockMonitor`와 같은 규칙) |

## 4. 동작(상태·알고리즘)

### 4.1 필터

- 상태 `s = [x, y, vx, vy]`, 공분산 `P`(4×4). 축마다 등속 모델 `F(dt) = [[1, dt], [0, 1]]`, 이산 백색 가속도 잡음 `Q(dt) = σ_a² · [[dt⁴/4, dt³/2], [dt³/2, dt²]]`.
- 측정 `z = [x, y]`, `H = [I₂ 0]`. `R`은 검출 `position_cov`의 XY 2×2 블록을 `conveyor_frame`으로 회전한 값이고, 사용할 수 없으면 `meas_std_default_m²·I₂`.
- `KalmanCV.update`는 Joseph 형식으로 공분산을 갱신하고 NIS `νᵀS⁻¹ν`를 돌려준다. `predict(t)`는 `t < t_state`이면 상태를 바꾸지 않는다.
- 새 track: `x0 = [z, 0, 0]`(벨트 추정이 있으면 속도 초기값은 `BeltMotion` 속도), `P0 = diag(R, init_vel_std², init_vel_std²)`.

### 4.2 연관·track 수명

- 프레임마다 `CLASS_CAP`이고 `position_valid`인 검출만 쓴다.
- 처리 순서: (1) 시각 검사 → (2) 모든 track을 검출 stamp로 예측 → (3) 마할라노비스 거리 `d² ≤ gate_chi2`인 쌍을 거리 오름차순으로 탐욕 배정 → (4) 배정된 track 갱신(`OBSERVED`, `observation_count += 1`, `last_observed_stamp` 갱신) → (5) 배정되지 않은 검출 중 관측 구간 안(`x_obs_min ≤ x ≤ x_obs_max`)의 것으로 새 track 생성 → (6) 배정되지 않은 track 상태 전이.
- 예측 단계에서 `t − t_last_obs > coast.max_s`인 track은 연관 전에 `LOST`가 된다(프레임 공백 뒤 첫 검출이 만료된 track에 붙지 않는다).
- 관측 구간 밖의 배정되지 않은 검출은 버린다. 구간 끝을 지나 `LOST`가 된 뒤에도 카메라에 계속 보이는 뚜껑이 프레임마다 새 track을 만들었다가 바로 `LOST`가 되는 것을 막는다. 이미 있는 track의 갱신은 구간과 무관하게 `x_obs_max + exit_margin`까지 이어진다.
- 상태 전이: 관측이 있으면 `OBSERVED`. 관측이 없고 `t − t_last_obs ≤ coast.max_s`이면 `COASTING`(예측만, 공분산 증가). 초과하면 `LOST`. 예측 `x > x_obs_max + exit_margin`이면 즉시 `LOST`.
- `LOST` track은 한 번 발행한 뒤 제거한다. 한번 `LOST`가 된 track_id는 다시 쓰지 않는다. 같은 병뚜껑이 `T_lap` 뒤 다시 관측 구간에 들어오면 새 track_id를 받는다(epoch는 바뀌지 않는다).
- `track_id`는 노드 기동부터 단조 증가하며 epoch가 바뀌어도 되돌리지 않는다.
- `valid_until = last_observed_stamp + coast.max_s`.

### 4.3 시각 규칙

- 검출 stamp가 현재 `t_state`보다 이르면(역순 도착) 그 프레임을 버리고 폐기 카운트를 1 올린다. 상태는 바꾸지 않는다.
- ROS 시각이 `backstep_tol_s`보다 크게 역행하면(`/clock` 역행·재시작) 모든 track을 지우고 `epoch += 1`, 속도 변화 검출기와 벨트 추정을 초기화한다(PLAN.md 5.3절).
- `TrackedObjectArray.header.stamp`와 각 track의 `header.stamp`는 그 프레임의 `t_state`(처리한 검출 stamp)다. 수신 시각이나 현재 시각을 쓰지 않는다.

### 4.4 속도 변화 검출

- 입력 `innovation_v`: 진행 방향(`x`) 위치 혁신값을 그 성분의 혁신 표준편차 `√S_xx`로 나눈 무차원 값. 벨트 추정에 쓰는 track(4.5절)에서만 넣는다.
- 양쪽 CUSUM: `g⁺ = max(0, g⁺ + ν − k)`, `g⁻ = max(0, g⁻ − ν − k)`. 어느 쪽이 `h`를 넘으면 검출(`True`)하고 두 누적값을 0으로 되돌린다.
- 검출하면 `steady = false`, `last_change_stamp = t`, 해당 track의 속도 공분산을 `init_vel_std²·I₂`로 되돌리고 위치-속도 교차 공분산을 0으로 두어(양의 준정부호 유지) 빠르게 다시 수렴시킨다. `recover_s` 동안 재검출이 없으면 `steady = true`.

### 4.5 벨트 속도(`BeltMotion`)

- 추정원 track: `OBSERVED` 또는 `COASTING`인 cap track 중 `observation_count`가 가장 큰 것. 없으면 마지막 추정을 그대로 두고 `header.stamp`를 바꾸지 않는다(소비자는 stamp 나이로 신선도를 판단한다).
- `direction = v / |v|`(`|v| ≤ v_min`이면 `+x`), `speed = |v|`, `speed_std = √(dᵀ P_vv d)`, `source = SOURCE_CAP`, `window_sec = min(t_state − 추정원 track 생성 시각, window_max_s)`.

### 4.6 `SelectTarget`

- 후보: `object_class` 일치, `status == OBSERVED`, `observation_count ≥ min_observation_count`, `vx > v_min`, `x < x_reach_start`.
- 선택: `x_reach_start − x`가 가장 작은 후보. `t_enter_window = t_state + (x_reach_start − x) / vx`.
- 결과: 후보가 있으면 `OK`와 `track_id`·현재 `epoch`·`t_enter_window`. 벨트가 정지(`|v| ≤ v_min`)이면 `PRECONDITION`·`BELT_STOPPED`. 후보가 없으면(대상 없음, 모두 이미 도달 구간 안) `PRECONDITION`·`NO_TARGET`.

### 4.7 `ResetTracking`

- `region == REGION_UNKNOWN`이면 모든 track을 지우고 `epoch += 1`, 결과 `OK`, `new_epoch`. 다른 region 값은 같은 동작이다(tracker는 벨트 영역만 추적한다).

### 4.8 노드

- `tracker_node`는 Managed Lifecycle 노드이며 SPEC-010 4.3절의 비활성 응답 규칙을 따른다.
- 관측 TF `conveyor_frame → cap_<track_id>`는 `OBSERVED`·`COASTING` track마다 프레임 처리 직후 발행한다. stamp = `t_state`, orientation identity. 미래 stamp는 발행하지 않는다. TF는 시각화 전용이며 planner·controller는 TF가 아니라 `objects/tracked`를 쓴다.
- 공통 요청 필드의 `deadline`이 이미 지났으면 처리하지 않고 `TIMEOUT`으로 응답한다.
- 검출 입력이 `input_timeout_s` 넘게 끊기면 빈 프레임을 넣어 상태 전이를 진행한다. 정상 지연 범위의 입력 사이에는 빈 프레임을 넣지 않으며, ROS 시각이 멈추면 상태도 늙지 않는다.

## 5. 시간 제약(주기·제한시간)

- 처리 주기: 검출 프레임마다(30 Hz). 한 프레임 처리 시간 p99 ≤ 5 ms(track 10개 이하, PC 기준, TBD(측정 G3)).
- 서비스 응답: `SelectTarget`·`ResetTracking` p99 ≤ 50 ms.
- Heartbeat 10 Hz(SPEC-010).

## 6. 실패 모드와 응답(중단 등급)

| 실패 | tracker 응답 | 오케스트레이터 쪽 처리(SPEC-110) |
|---|---|---|
| 가림(관측 없음 ≤ `coast.max_s`) | `COASTING`, 공분산 증가 | 해제 게이트 τ_max 판정 |
| 가림 초과 | `LOST` | 해제 전이면 L3 |
| 속도 변화 | `steady = false` | WAIT_CAP 보류, 동적 단계면 L3 |
| 역순 stamp | 프레임 폐기, 카운트 | 없음 |
| ROS 시각 역행 | 전체 초기화, `epoch += 1` | 이전 epoch 계획 무효(L3) |
| 입력 끊김 | track이 `COASTING` → `LOST` | 관측 신선도 감시 |

## 7. 성공 판정(측정 가능)

- REQ-TRK-001: 등속 합성 궤적(30 Hz, σ = 0.3 mm, 20 mm/s, 60 s, seed 20개)에서 관측 30회 이후 위치 오차 p95 ≤ 0.4 mm, 속도 오차 p95 ≤ 1.5 mm/s이고, 모델과 같은 잡음으로 만든 궤적에서 평균 NEES가 χ²(4) 95% 구간 안이다.
- REQ-TRK-002: `t_state`보다 이른 stamp의 검출은 상태를 바꾸지 않고 폐기 카운트를 1 올린다.
- REQ-TRK-003: 관측이 끊기면 `coast.max_s`까지 `COASTING`이며 공분산 trace가 단조 증가하고, 재관측이 게이트(`gate_chi2`) 안이면 같은 track_id로 `OBSERVED`, 밖이면 새 track을 만든다.
- REQ-TRK-004: 관측 없음이 `coast.max_s`(1.0 s)를 넘으면 `LOST`가 된다.
- REQ-TRK-005: 관측 구간을 나갔다가 다시 들어온 병뚜껑은 새 track_id를 받고 epoch는 바뀌지 않는다.
- REQ-TRK-006: 벨트 속도가 20 → 30 mm/s로 바뀌면 1.0 s 안에 `steady = false`가 되고, 60 s 등속(seed 20개)에서는 오검출이 0회다.
- REQ-TRK-007: 프레임 10%를 무작위로 빼도 유효 추적(`OBSERVED`·`COASTING`) 비율 ≥ 99%, 위치·속도 오차가 REQ-TRK-001 기준의 1.1배 이내다.
- REQ-TRK-008: ROS 시각이 역행하면 모든 track을 지우고 epoch를 1 올린다. `tracker_node`는 그때 역행 크기와 새 epoch를 WARN 로그 1건으로 남긴다(`ResetTracking`은 INFO).
- REQ-TRK-009: `ResetTracking`은 `new_epoch = 이전 epoch + 1`을 돌려주고 track을 비운다.
- REQ-TRK-010: `SelectTarget`은 도달 구간 상류의 가장 가까운 병뚜껑을 고르고 진입 시각 오차 ≤ 50 ms이며, 대상이 없거나 모두 도달 구간 안이면 `PRECONDITION`이다.
- REQ-TRK-011: 등속 입력에서 `BeltMotion.source = SOURCE_CAP`, `|direction| = 1 ± 1e-9`, speed 오차 ≤ 0.5 mm/s다.
- REQ-TRK-012: 관측 TF `cap_<id>`의 stamp는 `t_state`와 같고, 미래 stamp는 0건이며, orientation은 identity다.
- REQ-TRK-014: 배정되지 않은 검출은 `x_obs_min ≤ x ≤ x_obs_max`일 때만 새 track을 만든다(경계 포함). 구간을 지나가는 뚜껑 하나는 track 하나를 만들고 `LOST`는 한 번만 나온다.
- REQ-TRK-013: OFF-1 실제 bag replay에서 유효 추적 ≥ 99%(가림 구간은 별도 집계), 속도 변화 검출 지연 ≤ 1.0 s다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- U: TC-TRK-001~008·014(`tracking/test/`, `TrajectoryFixture`). C: TC-TRK-008-02·009~012(`tracking/test/contract/`). I1: TC-TRK-013(`needs_data("off1_bags")`).
- 게이트: G3(TEST_SPEC 4절 게이트 표). G3 실측(독립 GT 대비 오차)은 TC-BEN·TC-EXP-G3.

## 9. 미결 사항

- 관측 구간·도달 구간 경계값: TBD(배치도 OFF-1, 보정 OFF-3).
- 속도 변화 검출: `sigma_accel_mps2` 0.01에서 TC-TRK-006(10 mm/s 계단 1 s 안 검출)을 지키는 CUSUM은 `k` 1·`h` 8뿐이며, OFF-1 벨트 영상에서 약 60 s당 22~38회 경보가 난다(실제 맥동 범위가 시험 계단보다 크다). 다이얼 조작과 맥동을 구분하는 방법과 WAIT_CAP의 `steady` 조건은 TBD(OFF-5 다이얼 조작 bag).
- `coast.max_s`: 그리퍼가 뚜껑을 가리지 않는 배치(TC-BEN-010-01)에서 검출 누락 구간 분포로 정한다. TBD(측정 G3, OFF-5).
