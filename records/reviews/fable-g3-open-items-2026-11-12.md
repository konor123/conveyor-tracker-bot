# G3·G4 남은 결정 사항 분석과 제안 (2026-11-12)

- 작성: coordinator(Claude Fable 5.2). 분석은 GCE, 실험은 officepc WSL(임시 폴더, 시험 뒤 삭제).
- 배경: `fable-g3-review-2026-11-12.md`의 미결 항목(C-4·C-6·C-7·C-8, A-1~A-3)과 WSL 검증 중 새로 찾은 것.
- 이 문서는 분석과 제안이다. 채택 여부는 팀장이 정한다.

## 1. 실험으로 확인한 것

### 1.1 `DynamicPlace` goal의 deadline이 APPROACH 단계 제한시간이다 (C-7의 RETREAT `STEP_TIMEOUT` 원인)

- 코드: `cycle_runner._do_approach` → `place_start(mode, d)`의 `d`는 APPROACH 단계 deadline(`min(now + approach_s 3.0, 동적, 사이클)`)이고, `orchestrator_node._send`가 이것을 `goal.deadline`에 넣는다. 그런데 `DynamicPlace`·`PlaceStone` goal 하나가 APPROACH~RETREAT 5단계를 모두 수행한다. 서버는 SPEC-010 4.1절대로 `t_ros ≥ deadline`이면 스스로 `TIMEOUT(DEADLINE)`으로 끝난다.
- 실험(officepc WSL, 전체 대역 + 오케스트레이터, `nominal_20mms`, `MODE_SYNCHRONIZED`, 한 사이클):

| 대역 소요시간 축척 | `step_timeout.approach_s` | `DynamicPlace` 길이 | 결과 |
|---|---|---|---|
| 1.0 | 3.0 | 3.0 s | FAULT, RETREAT `STEP_TIMEOUT`(APPROACH~RELEASE 각 0.6 s 통과) |
| 0.8 | 3.0 | 2.4 s | SUCCESS |
| 1.0 | 6.0(임시) | 3.0 s | SUCCESS |

- 뜻: goal 전체가 `approach_s`(3 s)보다 길면 반드시 실패한다. 자동 시험은 축척 0.12(1.8 s)라 드러나지 않았다. 실제 로봇의 접근 + 동기 + 하강 + 해제 + 후퇴는 3 s를 넘으므로(시간 예산 성분 합만 4.0 s) 실제 `DynamicPlace`를 붙이면 매 사이클 해제 뒤 후퇴 중에 끊긴다.
- 제안: 여러 단계에 걸친 goal의 `goal.deadline`은 goal 전체 상한(`now + approach_s + synchronize_s + descend_s + release_s + retreat_s` = 10.5 s)으로 준다. 단계별·동적·사이클 제한은 지금처럼 오케스트레이터가 cancel로 집행한다. SPEC-100·SPEC-110에 한 줄, 단위 시험 1건(goal.deadline 값), I0 실시간 축척 1사이클을 회귀 시험으로 추가(약 20 s).

### 1.2 대상 track이 사라지면 WAIT_CAP이 통과 끝까지 기다린다 (C-7의 "WAIT_CAP 대상 없음"과 같은 증상)

- 실험(officepc WSL, TC-INT-009-01): 파일 전체 실행 2회, 해당 시험만 실행 2회 모두 seed 2(동기)·seed 3(예측)에서 `SAFE_ABORT CYCLE_TIMEOUT`, `passes_skipped` 1. 단독 실행 1회는 통과.
- 실패 사이클의 `objects/tracked`: goal 뒤 4.3~5.3 s에 track ID가 바뀌고(4 → 5, `observation_count` 134 → 6) epoch가 1 올랐다. 위치는 이어진다(같은 뚜껑). 같은 시각에 시스템 시계가 약 1.0 s 뒤로 갔다(시계 감시: 31.5 s마다 −1.0 s, WSL 커널 로그 `Time jumped backwards`). tracker는 ROS 시각 역행에 전체 초기화한다(REQ-TRK-008, 설계대로).
- 문제는 그 다음이다. 오케스트레이터는 WAIT_CAP에서 처음 고른 `(track_id, epoch)`만 본다. 그 track이 없어지면 `TRACK_NOT_OBSERVED`로 통과 끝(`lap_s` 30 s 또는 사이클 제한)까지 기다리고, 같은 뚜껑의 새 track이 있어도 다시 고르지 않는다. 통과 하나 또는 사이클 전체를 잃는다.
- 현장에서 track이 바뀌는 경우: 그리퍼 가림이 `coast.max_s`(1 s)를 넘을 때, 속도 변화·검출 튐으로 게이트를 벗어날 때, 시계 보정. 2026-11-12 팀원 노트북의 "WAIT_CAP 30 s 뒤 PASS_SKIP, 지표 없음"도 같은 증상이다(원인이 같은지는 미확인).
- 제안:
  1. WAIT_CAP 대기 중 대상 track이 목록에 없거나 epoch가 달라지면 같은 통과 안에서 `SelectTarget`을 다시 호출한다(`COASTING`은 지금처럼 기다림). 단위 시험 1건 + 가짜 시계 역행을 넣은 I0 1건.
  2. tracker가 초기화할 때 사유와 역행 크기를 WARN 로그로 남긴다(지금은 로그가 없어 원인을 찾는 데 시계 감시가 필요했다).
  3. `clock.backstep_tol_s` 0.0은 유지한다. 1 s 역행이면 검출 stamp도 같이 뒤로 가므로 초기화가 맞는 대응이다.
  4. 현장 장비(LEGION)에서 시연 전에 시계가 계단 보정되지 않는지 확인한다(`timedatectl`, `chronyc tracking` 또는 `timedatectl timesync-status`).

### 1.3 officepc WSL은 실제 tracker를 쓰는 실시간 시험에 맞지 않는다

- 이 WSL의 시스템 시계는 약 31.5 s마다 1.0 s 뒤로 간다(3회 연속 관측). 시뮬레이션 시각을 쓰는 계약 시험과 전체 대역 I0는 영향이 없어 통과한다. 실제 tracker가 벽시계로 도는 TC-INT-009-01은 이 장비에서 실패한다(CI는 통과).
- 제안: (a) 실제 노드 I0의 판정 장소를 CI와 LEGION으로 한다. (b) WSL 시계를 고치려면 PC 설정 변경(WSL 재시작 또는 시간 동기 설정)이 필요하므로 승인 뒤 따로 한다.

## 2. 코드 분석으로 확인한 것

### 2.1 관측 구간 밖 검출이 새 track을 만든다 (C-6)

- `track_manager.process_frame`: 예측 위치가 `x_obs_max + exit_margin`을 넘은 track은 `LOST`로 지운다. 뚜껑이 카메라에 계속 보이면 다음 프레임의 검출은 짝이 없어 새 track이 되고, 그 track도 구간 밖이라 바로 `LOST`가 된다. 프레임마다 반복된다(mock에서 한 통과에 135개).
- `x_obs_min_m`은 저장만 하고 쓰지 않는다. 상류 끝 밖의 검출도 track을 만든다.
- 영향: 관측 수 30 미만이라 대상 선택에는 안 걸린다. 다만 track ID와 `cap_<id>` TF 프레임이 통과마다 수십~백여 개 늘고, 발행 메시지에 `LOST` track이 매 프레임 실린다. 실영상 최고속 조건의 시야 출구 짧은 track 4개도 `x_obs_max`를 시야 안쪽에 두면 같은 규칙으로 없어진다.
- 제안: 짝 없는 검출은 `x_obs_min ≤ x ≤ x_obs_max`일 때만 새 track을 만든다(SPEC-300 4.2절 처리 순서 (5), REQ·TC 1건, 단위 시험). 기존 track의 갱신·종료 규칙은 그대로다.

### 2.2 `release_tcp_z_m` 0.10 (C-8)

- 값의 출처: 2026-11-09 planner 첫 커밋(`d50f088`)의 기본값이고 SPEC-400 표에 TBD(보정 OFF-3, 기본 핑거 OFF-4)로 적혀 있다. 실측값이 아니다.
- 계산: 잠정 벨트 평면 z 0.0697 m + 테두리 0.022 m = 테두리 윗면 0.092 m. 정지 투입 성공 높이는 플랜지 − 테두리 = 0.1606 m(설정 0.1656 m)이므로 해제 플랜지 z ≈ 0.257 m다. 0.10이면 플랜지가 테두리 위 8 mm이고, 길이 약 160 mm인 그리퍼가 벨트에 부딪친다.
- 지금은 실제 `DynamicPlace`가 없어 쓰이지 않는다. workspace 검사(z 0~0.40 m)는 두 값 모두 통과시켜 막아 주지 않는다.
- 제안: 절대 높이 대신 상대 높이로 바꾼다. planner가 `z = (뚜껑 테두리 z, robot_base) + target.release_above_rim_m`으로 계산하고 `release_above_rim_m`은 정지 투입에서 성공한 0.1656 m를 쓴다. 테두리 z는 track의 z(검출이 벨트 + `h_rim` 높이로 준다)를 `T_base_conveyor`로 옮겨 얻는다. 컨베이어를 다시 놓거나 벨트 평면을 다시 재도 값이 따라간다. 당장 바꾸지 않더라도 기본값 0.10은 충돌 값이므로 0.257로 고쳐 둔다.

### 2.3 `frames.launch.py`와 mock 세계의 `conveyor_frame` (C-4)

- `frames.launch.py`는 tracking.yaml(항등)을, mock 세계의 tracker·planner는 시나리오 변환 (0.30, −0.25, 0)을 쓴다. RViz에서 `cap_<id>`와 예측 Marker가 약 0.39 m 어긋난다.
- 제안: `frames.launch.py`에 `scenario` 인자를 추가한다. 주면 `drop.launch.py`와 같은 함수로 시나리오 변환을 쓰고, 없으면 tracking.yaml을 쓴다. 그 뒤 mock 사이클에서 `cap_<id>`와 Marker가 겹치는지 육안으로 확인하면 Progress 8절 RViz 항목을 닫을 수 있다(사람이 화면을 봐야 한다).

## 3. 기준값(A-1~A-3)

실영상 재생(`records/bench/belt-video-2026-11-07/replay-sigma-0.01.json`, 통과 전체 집계, 압축 영상·잠정 보정)의 실제 예측 오차 XY p95:

| 조건(벨트 속도) | 0.05 s | 0.1 s | 0.2 s | 0.5 s | 1 s |
|---|---|---|---|---|---|
| stable(31 mm/s) | 1.5 mm | 2.7 | 5.0 | 13.8 | 27.6 |
| fast-1(54 mm/s) | 1.5 | 2.6 | 4.3 | 6.5 | 10.5 |
| fast-1-long(53 mm/s) | 1.5 | 2.6 | 4.2 | 8.4 | 16.1 |
| fast-2(82 mm/s) | 2.3 | 2.7 | 3.3 | 4.5 | 6.0 |
| maximum(127 mm/s) | 2.7 | 3.4 | 3.8 | 4.4 | 7.1 |

- 투입 허용 오차 `e_xy_max`는 3.25 mm다. 실제 오차는 0.1 s 앞에서 이미 2.6~3.4 mm다. 해제 순간의 정확도는 "마지막 관측에서 해제까지 얼마나 짧게 예측하느냐"가 정하고, 계획 단계(3 s 앞)의 표준편차 기준은 정확도에 거의 기여하지 않는다.
- 필터가 보고하는 표준편차(mock에서 3 s 앞 2.7 mm)는 실제 오차(1 s 앞 6~28 mm)보다 작다. 보고 표준편차 0.5 mm 해제 게이트는 실제 오차 0.5 mm를 뜻하지 않는다.
- 제안:
  1. 계획 단계 기준(3.25 mm, `plan_lead_s` 3.0 s)은 "계획이 터무니없지 않은지" 보는 용도로 두고 잠정값으로 유지한다. 실영상에서 보고 표준편차가 이 값을 넘는지는 재생 도구에 보고 표준편차 출력을 추가해 기존 OFF-1 자료로 먼저 확인한다(현장 없이 가능).
  2. 정확도 판정은 해제 게이트에 둔다. 게이트 항목은 보고 표준편차가 아니라 마지막 관측 나이(`tau_max`)와 추종 오차(`e_xy`, `v_rel`)를 주로 본다(PLAN.md 4.4절, 9절 `tau.py`). `tau_max`는 위 표 기준으로 0.1 s 안쪽이어야 한다.
  3. 시연은 `MODE_SYNCHRONIZED`와 낮은 속도(≤ 50 mm/s)를 기본으로 한다. 동기 추종은 계획을 계속 고치므로 3 s 앞 예측 오차의 영향이 가장 작다.
  4. `plan_lead_s`는 G6 속도 조건을 정할 때 `plan_lead_s + 도달 구간 길이 / v ≥ t_required + margin`을 함께 확인한다(구간 0.20 m면 약 100 mm/s까지).
  5. 해제 게이트(0.5 mm 포함)를 실제로 판정하는 것은 실제 `DynamicPlace` 구현의 완료 조건이다(Progress 10절).

## 4. 권장 순서

1. 1.1 goal deadline(실제 `DynamicPlace` 전에 반드시), 1.2 대상 재선택 + tracker 초기화 로그.
2. 2.1 새 track 규칙, 2.2 해제 높이(최소한 기본값 교정).
3. 2.3 `frames.launch.py` 인자와 RViz 육안 확인.
4. 3절 재생 도구 보강 → OFF-5에서 기준 확정.
