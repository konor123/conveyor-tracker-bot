# 움직이는 병뚜껑 추적과 해제 계획 (2026-11-09)

움직이는 벨트 위 병뚜껑에 공깃돌을 넣으려면 두 질문에 답해야 한다. "지금 뚜껑은 어디에 있고 얼마나 빨리 가는가?"(추적, SPEC-300)와 "언제 어디서 그리퍼를 열어야 하고, 그럴 시간이 있는가?"(계획, SPEC-400)다. 이 문서는 두 패키지의 순수 Python 모듈을 무엇을 기준으로 만들었고 어떻게 확인했는지 설명한다. ROS 노드·계약 시험·실장비 측정은 아직이다.

## 1. 큰 그림

```
검출(30 Hz, robot_base) → [tracker] conveyor_frame에서 [x, y, vx, vy] 추정
                              ├─ objects/tracked (t_state의 상태와 공분산)
                              ├─ belt/measured_motion (벨트 속도, steady)
                              └─ tracking/select_target (다음 통과 대상)
                         → [planner] release_time 예측, 시간 예산, 도달성
                              └─ InterceptPlan (robot_base의 TCP 목표, plan_id·revision)
```

- tracker는 **미래를 예측하지 않는다.** 마지막 관측 시각(`t_state`)의 상태만 낸다. 미래 위치는 planner가 `t_state`부터 계산한다. 이렇게 하면 지연을 한 곳에서만 다룬다.
- 둘 다 ROS를 모르는 순수 모듈이다(`PLAN.md` 0.2절). 노드는 메시지 변환과 서비스만 맡는다. 그래서 가짜 시계와 합성 궤적만으로 1초 안에 시험할 수 있다.

## 2. 칼만 필터에서 배운 것 (`tracking/kalman.py`)

- **모델:** 등속 모델. 상태 `[x, y, vx, vy]`. 예측할 때 위치에 `v·dt`를 더하고, "가속도가 조금 있을 수 있다"는 불확실성 `Q`를 공분산에 더한다. `Q`의 크기는 `sigma_accel_mps2` 하나로 정한다.
- **σ_a를 0.02에서 0.001로 줄인 이유:** 0.02이면 필터가 측정 잡음(0.3 mm)을 "속도가 변한 것"으로 오해해 속도 오차 p95가 2.24 mm/s로 나왔다(기준 0.5 mm/s). 0.001로 줄이니 0.214 mm/s가 되었다. 대가는 필터가 "굼떠진다"는 점이다. 벨트 속도를 실제로 바꾸면 따라가는 데 오래 걸린다. 그래서 3절의 속도 변화 검출기가 짝으로 필요하다.
- **NEES로 필터의 정직함 검사(TC-TRK-001-02):** 오차가 작은지만 보면 안 된다. 필터가 말하는 공분산(자기 확신)이 실제 오차와 맞는지도 본다. `NEES = eᵀP⁻¹e`의 평균이 상태 차원(4) 근처면 정직하다. 이 검사는 **필터와 같은 잡음 모델로 만든 궤적**에서만 의미가 있다. 완전 등속 궤적에 넣으면 필터가 늘 "자신 없는 척"을 해서 NEES가 낮게 나온다.
- **함정, 여러 seed를 하나씩 95% 판정하기:** seed 20개를 각각 95% 구간으로 판정하면 하나쯤은 우연히 떨어진다. 독립 실행 N개의 평균을 χ²(4N)/N 구간으로 **한 번** 판정한다.

## 3. 게이트와 속도 변화 검출

- **연관 gate:** 새 검출이 기존 track의 것인지 판정할 때 마할라노비스 거리 `d²`(오차를 불확실성으로 나눈 값)를 쓴다. 교과서 값 χ²(2) 99% = 9.21은 정상 상태에서 반경 약 0.95 mm다. 측정 이상치 하나나 벨트 속도 계단만으로 track이 끊기고 새 ID가 생겼다. 100으로 넓히면 반경 약 3.1 mm다. 뚜껑 간격(수십 mm)보다 충분히 작아 다른 뚜껑과 헷갈리지 않는다. "통계 기준"보다 "물리적 반경"으로 설명하면 판단하기 쉽다.
- **CUSUM 속도 변화 검출(`speed_change.py`):** 진행 방향 혁신값(예측과 측정의 차이를 표준편차로 나눈 값)을 누적한다. 등속이면 혁신값은 0 주변에서 오르내려 누적되지 않는다(허용 편차 `k`를 빼므로). 속도가 바뀌면 혁신값이 한쪽으로 쏠려 누적이 `h`를 넘는다. 20 → 30 mm/s에서 최대 0.167 s 만에 검출했고, 60 s 등속 20회에서 오검출은 0이었다. 검출하면 속도 공분산을 크게 되돌려 필터가 다시 빨리 배우게 하고 `BeltMotion.steady = false`로 오케스트레이터에 알린다.
- **현장 주의:** 실제 벨트는 맥동(주기적 속도 흔들림)이 있다. 합성 시험에서 정한 σ_a·gate·CUSUM 값이 실데이터에서 오검출을 만들 수 있다. OFF-5 bag으로 다시 정한다(SPEC-300 9절).

## 4. 시간 규칙

- 검출 stamp가 현재 상태보다 이르면(늦게 도착한 옛 프레임) 버리고 센다. 상태를 되돌리지 않는다.
- ROS 시각 자체가 뒤로 가면(`/clock` 재시작, bag 반복 재생) 모든 track을 지우고 `epoch`를 올린다. planner와 controller는 epoch가 다른 계획을 무효로 본다. "시각이 꼬이면 이어 붙이지 말고 세대를 바꾼다"가 원칙이다.
- 같은 뚜껑이라도 한 바퀴 돌아오면 새 track_id다. 통과마다 성공·실패를 따로 세기 위해서다.

## 5. 해제 계획 (`intercept_planner`)

- **예측:** `p(t) = p + v·(t − t_state)`, 위치 분산 `P_xx + 2·dt·P_xv + dt²·P_vv`. 처리·통신 지연은 `t − t_state`에 이미 들어 있으니 따로 더하지 않는다(TC-PLN-002-01). 지연을 두 번 보상하면 뚜껑보다 앞에 떨어뜨린다.
- **시간 예산:** `T_available = (x_exit − x_cap) / v`, `T_required = 이동 + 동기 + 하강 + 해제 + 이탈`, `time_margin = T_available − T_required ≥ margin`. `margin`은 한 번만 쓴다(`T_required`에 넣지 않는다). 벨트가 `v_min` 이하이면 BELT_STOPPED, 거꾸로 가면 BELT_REVERSED.
- **경계값 정리:** 처음 SPEC 본문은 `|v| < v_min`을 정지로, REQ와 TC는 `v ≤ v_min`을 정지로 적어 정확히 `v = v_min`에서 어긋났다. worker가 이를 보고했고 REQ 쪽(`≤`)으로 통일했다. 경계 시험(직전·경계·직후)을 써야 이런 불일치가 드러난다.
- **판정 순서:** 요청 자체가 잘못되면(`EPOCH_MISMATCH`, `TRACK_NOT_FOUND`, `MODE_UNSUPPORTED`) 계획을 만들지 않는다. 계획은 만들었지만 쓸 수 없으면 status로 구분한다(`TIME_INSUFFICIENT` → `UNREACHABLE` → `UNCERTAIN`). 순서표는 SPEC-400 4.4절에 있다.
- **plan_id와 revision:** 서비스 호출마다 새 `plan_id`. 같은 계획을 새 관측으로 다시 계산하면 `revision`만 올리고 **해제 시각은 고정**한다. 로봇은 정해진 시각에 갈 곳의 좌표만 계속 고쳐 받는다. 만료·epoch 변경·track 소실이면 `EXPIRED`를 한 번 내고 닫는다. 검수 중 "track의 epoch가 계획과 다를 때 갱신하면 예외가 난다"는 결함을 찾아 회귀 시험을 넣고 고쳤다.
- **τ_max(`tau.py`):** 그리퍼가 뚜껑을 가려 관측이 없을 때 얼마나 오래 예측만 믿어도 되는지 계산한다. `e_pos + |δv|·τ + ½·a·τ² + |v|·δt ≤ e_allow`의 최대 τ다. 입력 값(δv, a, δt)은 G3·G4 측정으로 정한다.

## 6. 확인 방법

```bash
PYTHONPATH=src/tracking python3 -m pytest -q src/tracking/test                  # 109 passed, 약 28 s
PYTHONPATH=src/intercept_planner python3 -m pytest -q src/intercept_planner/test  # 97 passed
python3 tools/spec/trace.py --area TRK,PLN --check   # 남은 TC 8개는 C·I1·I2(노드 필요)
```

## 7. 남은 일

`tracker_node`·`intercept_planner_node`와 계약 시험(TC-TRK-009~012, TC-PLN-006·007), I2(TC-PLN-011), 오케스트레이터 `deadlines.py`·WAIT_CAP 연동, `mock_world_node`의 임시 계획을 실제 planner로 바꾸기, OFF-5 실데이터로 파라미터 다시 정하기.

## 자료

[SPEC-300](../../specs/SPEC-300-tracking.md), [SPEC-400](../../specs/SPEC-400-planner.md), [TEST_SPEC](../../TEST_SPEC.md) 3.6·3.7절, [Progress](../../Progress.md) 7·9절, [11월 9일 worklog](../../records/worklog/2026-11-09.md)의 "G3·G4 준비" 절.
