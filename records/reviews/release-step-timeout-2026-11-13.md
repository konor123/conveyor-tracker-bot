# 실제 controller I0의 RELEASE `STEP_TIMEOUT` 분석 (2026-11-13)

- 수행: coordinator(Claude Opus 5.5). 실행 장소: GCE(소스 읽기·순수 Python 시험), 사무실 PC WSL(ROS 시험).
- 대상: `records/reviews/heartbeat-stall-evidence-2026-11-13.md` 4절의 한 launch(`test_i0_real_controller_dynamic_cycle[synchronized]`, 실제 controller launch 66개 중 1개). 로그에 남은 것은 `RELEASE STEP_TIMEOUT  []`와 SAFE_ABORT 한 줄뿐이다.
- 표기: **[확인]** = 실행·시험으로 확인, **[코드]** = 소스로 확인, **[추론]** = 근거 있는 추정.

## 1. 로그 한 줄이 가리는 두 경로 [코드]

기준 코드는 `2c4580d`다. orchestrator의 RELEASE 판정(`cycle_runner.py:427` `_do_release`)은 포트가 None을 내면 `STEP_TIMEOUT`이다. 포트(`orchestrator_node.py:506` `RosSkillPort._phase_done`)가 None을 내는 경로는 둘이다.

| 경로 | 조건 | 그때까지 남는 것 |
|---|---|---|
| (a) 대기 끝 | RELEASE 끝 feedback 없이 ROS deadline(진입 + 1.0 s), steady 상한(남은 ROS 시간 × 2), 또는 latch된 장애 | 없음 |
| (b) 결과 먼저 | controller의 DynamicPlace 결과가 RELEASE 끝 feedback보다 먼저 끝남(예: 해제 게이트 대기 창에서 실패) | controller 사유는 `_terminal_fault`(PROTECTIVE_STOP·STOPPED만 처리)에서 버려짐 |

controller(`robot_controller_node.py`)는 skill 결과를 로그에 남기지 않았다. 그래서 그 한 건이 (a)인지 (b)인지, (a)라면 몇 s 걸렸는지 알 수 없다.

## 2. 설계 간극: 게이트 대기 2.0 s와 RELEASE 제한 1.0 s [코드·확인]

- MODE_SYNCHRONIZED에서 controller는 해제 게이트가 닫혀 있으면 실패하지 않고 기다린다. 끝내는 이유(`TERMINAL_GATE_REASONS`)는 `GATE_NONFINITE`·`GATE_EPOCH`·`GATE_EXPIRED`·`GATE_RIM_CLEARANCE` 네 가지뿐이다. `GATE_TRACK_STALE`·`GATE_BELT_STALE`·`GATE_BELT_UNSTEADY`·`GATE_PLAN_UNCERTAIN`·`GATE_TRACK_LOST`·`GATE_COASTING`·`GATE_POSITION_STD`·`GATE_E_XY`·`GATE_V_REL`이면 RELEASE 진입부터 `release_wait_max_s`(2.0 s, `dynamic_place.py:244`)까지 기다린다(`dynamic_place.py:646`). 짧은 맥동을 기다리는 것은 의도된 동작이다(TC-ROB-009-02, 0.5 s 맥동).
- orchestrator RELEASE 단계 제한은 1.0 s(`src/orchestrator/config/orchestrator.yaml` `step_timeout.release_s`, 측정 전 제안값, SPEC-100 5절)다.
- `backend:=fake` transport에서 RELEASE 길이 = 게이트 대기 + commit→개방 0.1 s(`fake_open_delay_s`) + 개방 확인 유지 0.3 s(`release_hold_s`). 게이트가 0.7 s 닫혀 있으면 RELEASE가 1.1 s 걸리고 controller 결과는 OK다 **[확인, GCE TC-ROB-018-01]**. 게이트가 약 0.6 s(feedback 전달 시간만큼 더 짧게) 넘게 닫히면 controller가 정상으로 끝나도 orchestrator RELEASE가 먼저 끝난다.
- 실제 controller I0의 정상 사이클은 이미 1.0 s에 가깝다 **[확인, 4절]**: RELEASE 0.88~0.91 s. SYNCHRONIZED는 계획 위치 표준편차가 해제 기준(0.5 mm, `sync.release_position_std_max_m`) 아래로 내려오기를 약 0.49 s 기다리고(`GATE_POSITION_STD`), PREDICTED는 예정 해제 시각까지 0.47 s 기다린 뒤 commit한다. 여유는 약 0.1 s이므로 게이트 대기가 0.1 s만 늘어도 orchestrator RELEASE가 먼저 끝난다.
- 게이트를 더 닫을 수 있는 I0 원인 하나 **[추론]**: track 신선도 한도 τ는 0.3 s(`watchdog.obs_freshness_s` → goal `tau_max`)다. 실제 tracker나 대역 세계가 발행에서 0.3 s 이상 멈추면(같은 날 대역 세계에서 0.31~0.38 s 정지 3건, 실제 controller에서 기동 직후 공백 1건) `GATE_TRACK_STALE`로 게이트가 닫힌다. 그 한 건의 대역 세계에는 공백 경고가 없었고 실제 tracker는 진단 대상이 아니어서 확인할 수 없다.
- 이 간극은 앞선 대역 controller I0의 `STEP_TIMEOUT`(seed 7·18, `records/reviews/fable-m0-risk-2026-11-11.md` 1절)을 설명하지 못한다. 대역 controller에는 해제 게이트가 없다.

## 3. 바꾼 것(판정·제한값은 그대로)

- orchestrator(REQ-ORC-020, SPEC-100 v0.17): (a)·(b)를 가리는 경고 한 줄. 경로, 대기 시작 뒤 ROS·steady 경과, 마지막 feedback의 phase·끝 여부·나이, latch된 장애, (b)면 controller의 result 코드와 `reason_code`. 실패한 단계 경고에 단계 길이(ROS)를 붙였다.
- controller(REQ-ROB-018, SPEC-500 v0.13): DynamicPlace가 끝날 때 한 줄. 결과·사유, goal 시작 기준 phase 진입 시각과 commit 시각, RELEASE 진입→commit, commit 전 게이트가 닫혀 기다린 주기 수(이유별).
- 바꾸지 않은 것: `step_timeout.release_s` 1.0 s, `release_wait_max_s` 2.0 s, 게이트 판정, 결과 분류(`STEP_TIMEOUT`). 두 값의 관계는 SPEC-500 9절 미결 사항으로 두고 G0 측정 뒤 함께 정한다.

## 4. 사무실 PC WSL 실행

최종 코드(작업 사본, 사무실 PC WSL)에서 실제 controller I0 8건 묶음을 3번 돌렸다: 7 passed·1 failed, 8 passed, 8 passed. 실패 1건은 RELEASE와 무관한 `test_i0_real_controller_dynamic_failure_keep_sync_place_back`의 동기 유지 상승 속도 검사였다(`robot/status` 발행 stamp로 계산한 첫 상승 쌍 0.0233 m/s, 허용 0.020 ± 0.003). 가짜 transport가 DynamicPlace 끝 → SafeRetreat 시작 사이에 진행되지 않아 그때 발행된 위치가 오래된 탓이었고, 시험의 표본 선택을 고쳤다(`records/worklog/2026-11-13.md` 16:00 절). `STEP_TIMEOUT`은 0건이었다.

뒤 두 묶음은 launch 로그를 모두 남겨(`pytest -s`) 새 controller 한 줄을 모았다. 정상으로 끝난 DynamicPlace 4건:

| mode | 단계(goal 시작 기준, s) | RELEASE 진입→commit | 게이트 대기 | RELEASE 길이 |
|---|---|---|---|---|
| SYNCHRONIZED | APPROACH 0.000, SYNCHRONIZE 0.840, DESCEND 1.300, RELEASE 2.030, commit 2.530, RETREAT 2.940 | 0.500 s | `GATE_POSITION_STD`×49 | 0.910 s |
| SYNCHRONIZED | 0.000, 0.840, 1.310, 2.040, commit 2.530, 2.950 | 0.490 s | `GATE_POSITION_STD`×48 | 0.910 s |
| PREDICTED | 0.000, 0.840, 1.640, 2.370, commit 2.840, 3.260 | 0.470 s | - | 0.890 s |
| PREDICTED | 0.000, 0.850, 1.630, 2.360, commit 2.830, 3.240 | 0.470 s | - | 0.880 s |

해제 전 실패 시험(vision heartbeat 끊음)의 새 orchestrator 경고 3건은 모두 경로 "대기 끝", 장애 `HEARTBEAT_LOST`, 대기 0.24~0.31 s였고 controller 한 줄은 `CANCELED`였다(판정은 전과 같음). 세 묶음의 로그에 `Traceback`은 0건이었다.

## 5. 남은 것

- Progress 1절 I0 `STEP_TIMEOUT` 항목은 닫지 않는다. 그 한 건은 여유 약 0.1 s에서 생긴 것으로 보이지만(4절) 확인하지 못했다. 다음 발생 때 3절 경고로 (a)·(b)와 게이트 대기 이유를 판정한다.
- RELEASE 제한과 게이트 대기 상한의 관계(G0 측정 뒤).
