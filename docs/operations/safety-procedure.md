# 현장 안전 절차 (E0509 이동 병뚜껑 투입)

- 작성: OpenCode(oamp/auto) 초안, coordinator(Claude Opus 5.5) 검수(2026-11-12). 표의 설정값은 저장소 설정 파일과 대조했다. "현장에서 기입" 칸은 현장에서 채운다.

- 적용 대상: Doosan E0509 + ROBOTIS RH-P12-RN(A) 그리퍼 + 컨베이어 작업대에서 하는 정지·이동 병뚜껑 공깃돌 투입 실험(`PLAN.md` 1절)
- 근거: `PLAN.md` 1.1, 5, 6, 11.4절 · `AGENTS.md` 장비·안전 · `specs/SPEC-010-interfaces.md` 4.5절 · `specs/SPEC-110-abort-timeout.md` · `specs/SPEC-500-robot-control.md` · `src/robot_control/config/*` · `src/static_pick_place/config/static_pick_place.yaml` · `records/sessions/*.md` · `docs/learn/2026-11-10-fast-static-cycle.md` · `docs/learn/2026-11-11-stop-latch.md` · `Progress.md` 2절
- 수치는 저장소에 기록된 값(실측 또는 제안값)만 쓴다. 이 문서에서 새 값을 정하지 않는다. 값을 바꾸면 `records/param_changes.csv`에 근거를 남긴다(`AGENTS.md` 문서 규칙).

## 1. 적용 범위와 역할

이 절차는 실제 로봇·그리퍼에 지령을 보내는 `[OFF]` 현장 작업에만 적용한다. 순수 모듈 단위 시험·계약 시험·`mode:=virtual`·Isaac Sim은 대상이 아니다(`PLAN.md` 11.4절). 원격 접속은 온라인 작업이 아니다. 로봇·그리퍼가 움직이는 일은 원격 접속 중이라도 `[OFF]`로 보며 현장에 비상정지 담당자가 있어야 한다(`PLAN.md` 11.4절).

| 역할 | 담당 | 할 일 | 근거 |
|---|---|---|---|
| 운전자(지령·기록) | 세션 실행자(사람 또는 에이전트) | 세션 시작 전 점검(2절) 수행, 지령 실행, 로그·기록 저장, 이상 발생 시 정지 방법(4절) 적용, 기록 의무(7절) 수행 | `records/sessions/OFF-4-2026-11-09.md`(참석자 항목), `records/sessions/TEMPLATE.md` |
| 비상정지 담당자 | 사용자(현장) | 시작 전 입회 확인, 로봇이 예상과 다르게 움직이면 펜던트·비상정지로 즉시 정지, 재개 승인(5절) | `records/sessions/OFF-4-2026-11-09.md`(“비상정지 담당: 사용자(시작 전 ‘입회 중’ 확인…)”), `records/sessions/OFF-2-2026-11-07.md`, `PLAN.md` 11.4절 |
| 관찰자 | 실험 시 현장 인원 | 벨트 옆에서 투입 결과(SUCCESS/FAILURE/판독 불가)를 직접 보고 `tools/eval/trial_labeler.py`로 기록. 성공률은 이 판독 기록으로 계산 | `PLAN.md` 4.5절, `PLAN.md` 11.4절(“관찰자(실험 시)”) |

## 2. 시작 전 점검(체크리스트)

세션 시작 전에 아래를 순서대로 확인한다. 하나라도 만족하지 않으면 시작하지 않는다.

- [ ] 비상정지 담당자 입회 확인을 시작 전에 명시적으로 남긴다(예: “입회 중” 확인). 근거: `records/sessions/OFF-4-2026-11-09.md`
- [ ] 세션에서 사용할 코드·측정 스크립트는 현장에 오기 전에 구현하고 시험을 통과시킨다. 현장에서 처음 코드를 짜지 않는다. 현장에서 고친 코드는 같은 날 커밋하고 `[ON]` 시험을 다시 돌린다. 근거: `PLAN.md` 11.4절, `AGENTS.md` 장비·안전
- [ ] 로봇 제어기가 연결되어 GRANT·STANDBY 상태인지 확인한다. 근거: `records/sessions/OFF-4-2026-11-09.md`
- [ ] 펜던트에서 로봇 모드가 자동·대기인지 확인한다. 제어기 재부팅 뒤 MANUAL 모드이면 외부 제어 권한이 거부된다. 근거: `records/sessions/OFF-4-2026-11-09.md`, `docs/learn/2026-11-10-fast-static-cycle.md`(`NOT_AUTONOMOUS`)
- [ ] 실행 중인 다른 DRL 프로그램이 없는지 확인한다(`DRL_BUSY`·`NOT_STANDBY`는 시작 조건 불충족). 근거: `docs/learn/2026-11-10-fast-static-cycle.md`
- [ ] 그리퍼 통신 설정(Protocol Type, ID, 통신 속도)은 조회만 하고 설정 파일·펌웨어는 건드리지 않는다(6절). 근거: `AGENTS.md` 장비·안전, `Progress.md` 2절(OFF-1 “설정 변경 없이 조회만”)
- [ ] `robot_controller_node` backend를 확인한다. 기본은 실기 연결을 만들지 않는 `disabled`이며, 시험은 `backend:=fake`로 명시한다. 어떤 파라미터도 `/dsr01` 실기 연결을 암묵적으로 만들지 않는다. 근거: `src/robot_control/config/robot_control.yaml`, `specs/SPEC-500-robot-control.md` 2절
- [ ] 시스템 기동 후 **2 s 대기**한 뒤 controller heartbeat `error_code`를 확인한다. `101`이고 Stop 사유가 `CONTROLLER_HEARTBEAT_LOST`이며 로봇이 정지·대기 자세이면 `stop_release_reason`으로 사유를 남기고 해제한다(5절). 근거: `Progress.md` 2절
- [ ] 보정·평면 파일이 지금 배치와 맞는지 확인한다(컨베이어·매트를 옮긴 뒤에는 평면을 다시 잰다). 근거: `docs/learn/2026-11-10-fast-static-cycle.md`(배치 확인), `records/sessions/OFF-4-2026-11-09.md`(재배치 후 재측정 미완)
- [ ] 비상정지 위치·펜던트 위치: 현장에서 기입
- [ ] 작업 구간과 비상정지 담당자·관찰자 위치: 현장에서 기입

## 3. 속도·가속·작업영역 제한

지령 전에 작업영역·속도·비유한 값을 검사하고, 위반이면 송신하지 않고 L0와 `RANGE`를 낸다(`specs/SPEC-500-robot-control.md` REQ-ROB-004, `PLAN.md` 5.1절 L0 “지령 범위 초과”). 추종 지령은 `v = v_ff + Kp·(p_target − p_tcp)`이며 `|v| ≤ v_max`, `|v − v_prev| ≤ a_max·dt`로 제한한다(`specs/SPEC-500-robot-control.md` REQ-ROB-006, `src/robot_control/robot_control/motion_adapter.py` `tracking_command`).

| 항목 | 값 | 단위 | 설정 위치 | 근거 |
|---|---|---|---|---|
| 추종 최대 속도 | 0.1 | m/s | `src/robot_control/config/belt_sync.json`(`max_speed_mps`), 코드 기본값 `src/robot_control/robot_control/belt_sync.py` `SyncConfig` | `specs/SPEC-500-robot-control.md` REQ-ROB-013 |
| 추종 최대 가속 | 0.3 | m/s² | `src/robot_control/config/belt_sync.json`(`max_accel_mps2`) | 같음 |
| 추종 비례게인 | 4.0 | - | `src/robot_control/config/belt_sync.json`(`kp`) | 같음 |
| 추종 주기 | 0.01 | s | `src/robot_control/config/belt_sync.json`(`period_s`) | 같음 |
| 추종 루프 간격 한도 | 0.03 | s | `src/robot_control/config/belt_sync.json`(`max_loop_gap_s`) | 같음 |
| 추종 XY 작업영역(x) | 0.20 ~ 0.75 | m | `src/robot_control/config/belt_sync.json`(`min_xy_m`/`max_xy_m`) | 같음. 벗어나면 `WORKSPACE`로 정지(`src/robot_control/robot_control/belt_sync.py`) |
| 추종 XY 작업영역(y) | −0.40 ~ 0.40 | m | 같음 | 같음 |
| 계획 수락 위치 표준편차 한도 | 0.00325 | m | `src/robot_control/config/belt_sync.json`(`max_position_std_m`) | `specs/SPEC-500-robot-control.md` REQ-ROB-013 |
| 해제 불확실성 한도(진단) | 0.0005 | m | `src/robot_control/config/belt_sync.json`(`release_position_std_max_m`) | 같음. `release_uncertainty_ok()`는 해제 허가가 아니다 |
| 정적 투입 수평(경유) 속도 | 0.100 | m/s | `src/static_pick_place/config/static_pick_place.yaml`(`travel_v_mps`) | `records/sessions/OFF-4-2026-11-09.md`(2026-11-09 현장 실측 기반 기본값) |
| 정적 투입 수직 속도 | 0.050 | m/s | `src/static_pick_place/config/static_pick_place.yaml`(`vertical_v_mps`) | 같음 |
| 정적 투입 수평 가속 | 0.300 | m/s² | `src/static_pick_place/config/static_pick_place.yaml`(`travel_a_mps2`) | 같음 |
| 정적 투입 수직 가속 | 0.150 | m/s² | `src/static_pick_place/config/static_pick_place.yaml`(`vertical_a_mps2`) | 같음 |
| 정적 실행기 작업영역(x) | 0.30 ~ 0.75 | m | `src/static_pick_place/config/static_pick_place.yaml`(`x_min_m`/`x_max_m`) | 같음 |
| 정적 실행기 작업영역(y) | −0.25 ~ 0.25 | m | 같음(`y_min_m`/`y_max_m`) | 같음 |
| 정적 실행기 목표 z 하한 | 0.10 | m | 같음(`z_min_m`) | 같음 |
| 정적 실행기 목표 z 상한 | 0.2844 미만 | m | `src/static_pick_place/config/static_pick_place.yaml`(`travel_z_m`, 이동 높이) | `src/static_pick_place/static_pick_place/plan.py`(파지·해제 목표는 `z_min_m ≤ z < travel_z_m`, 아니면 `BAD_HEIGHT`) |
| 관절 J3 하한 | 0.1745 | rad | `src/static_pick_place/config/static_pick_place.yaml`(`j3_min_rad`) | 같음(도달 경계 확인: `records/sessions/OFF-4-2026-11-09.md`, 원래 벨트 위치는 J3 = 0° 도달 경계) |
| 대기 자세(park) | (0.373, 0.0, 0.405) | m | `src/static_pick_place/config/static_pick_place.yaml`(`park_x_m`/`park_y_m`/`park_z_m`) | `records/sessions/OFF-4-2026-11-09.md`(플랜지 (373.0, 0.0, 405.0) mm) |
| `move_v_mps`·`move_a_mps2`(move_line·추종 한도 키) | 미설정 | m/s·m/s² | `src/robot_control/config/robot_control.yaml`에는 키가 없다 | `specs/SPEC-500-robot-control.md` 3절(키 정의). 필수 설정값이 없으면 skill 결과는 `PRECONDITION`(같은 문서 2절) |
| 로봇 보호정지 판정 | 제어기 판정 사용 | - | — | `specs/SPEC-010-interfaces.md` `RobotStatus.protective_stop`, `PLAN.md` 5.1절 L4 |

감시·제한시간 참고: 중단 경로 제한시간은 L2 10 s, L3 15 s(제안)이고 넘으면 L4로 승격한다(`PLAN.md` 5.2, 6.1절, `specs/SPEC-110-abort-timeout.md` REQ-ABT-003). robot_controller heartbeat 300 ms 끊김은 L4다(`PLAN.md` 5.1절, `specs/SPEC-110-abort-timeout.md` REQ-ABT-010).

## 4. 비상정지와 정지 방법

### 4.1 중단 등급 개요

`PLAN.md` 5.1절의 중단 등급 중 현장에서 직접 관련되는 것:

| 등급 | 의미 | 현장에서의 동작 |
|---|---|---|
| L0 LOCAL_STOP | 지령 freshness 초과(주기 2배), 지령 범위 초과 | 즉시 속도 0, 이후 지령 거부. 재무장은 작업자 확인 뒤에만 한다 |
| L2 ABORT_CYCLE | 단계 실패(공깃돌을 안 들었고 동적 구간 아님) | 안전 자세 → 대기 |
| L3 SAFE_ABORT | 공깃돌을 들고 있거나 동적 구간 실패 | 동기 유지하며 상승 → 안전 자세 → 공깃돌을 매트에 내려놓음 → 대기 |
| L4 PROTECTIVE_STOP | 로봇 보호정지·비상정지, 통신 단절, heartbeat 끊김, 중단 경로 제한시간 초과 등 | `Stop` 요청, **자동 리셋·원점 복귀 금지** → 작업자 확인 후 재개(5절) |

### 4.2 펜던트·비상정지

- 프로그램(DRL) 실행 중에는 PC에서 단계 사이에 끼어들 수 없다. 멈추는 방법은 비상정지나 펜던트다. 근거: `docs/learn/2026-11-10-fast-static-cycle.md`(“멈추는 법”)
- 비상정지 담당자는 로봇이 예상과 다르게 움직이면 지체 없이 펜던트·비상정지로 멈춘다. 실행 로그가 `CYCLE_DRL_TIMEOUT`·`DRL_NOT_STOPPED`를 내면 제어기 프로그램은 **아직 움직일 수 있으므로** 뒤따르는 `operator_action` 로그대로 펜던트·비상정지로 멈춘다. `drl_stop`은 자동으로 부르지 않는다. 근거: `docs/learn/2026-11-10-fast-static-cycle.md`(`operator_action`)
- 펜던트·비상정지 위치: 현장에서 기입
- 비상정지를 사용한 경우 5절의 재개 절차를 따르고 7절의 기록을 남긴다.

### 4.3 Stop 서비스(정지 유지)

- 감시 노드(safety_supervisor)는 보호정지·통신 단절·controller heartbeat 손실을 보면 `robot/stop`을 부른다. 근거: `docs/learn/2026-11-11-stop-latch.md`
- `robot/stop`을 받으면 요청자(`ORCHESTRATOR`·`SUPERVISOR`)와 무관하게 controller가 **정지 유지(latch)** 상태가 된다. 실행 중 goal은 `CANCELED`(`STOPPED`)로 끝나고, 정지 유지 중 받은 goal은 지령 0회로 `CANCELED`(`STOPPED`)로 끝난다. 근거: `specs/SPEC-010-interfaces.md` 4.5절
- 정지 유지 중에는 `RobotStatus.ready = false`, heartbeat `error_code = 101`(보호정지가 함께 있으면 `100`)이다. lifecycle 비활성·재활성으로 풀리지 않는다. 근거: `specs/SPEC-010-interfaces.md` 4.5절, `specs/SPEC-500-robot-control.md` REQ-ROB-014
- 지령 신선도(L0): 지령 간격이 주기 2배를 넘으면 속도 0을 1회 보낸 뒤 재무장 전까지 지령을 거부한다. 근거: `specs/SPEC-500-robot-control.md` REQ-ROB-003, `src/robot_control/robot_control/motion_adapter.py` `FreshnessWatchdog`

### 4.4 DRL이 PLAY로 남은 경우

2026-11-09 실기에서 확인된 사례와 절차(`records/sessions/OFF-4-2026-11-09.md`, `docs/learn/2026-11-10-fast-static-cycle.md`):

1. 실패한 그리퍼 DRL이 `exit()` 뒤 PLAY로 남고 `drl_stop`(값 0·1·2)이 모두 거부된다.
2. **드라이버를 다시 띄우지 말고** 상태(로그, 알람)를 기록한 뒤 11-09 절차(제어기 재부팅)를 따른다. 이때 팔·그리퍼가 움직일 수 있으면 먼저 펜던트·비상정지로 멈춘다(4.2절).
3. 재부팅 뒤 로봇 모드가 MANUAL이 되고 펜던트가 자동·대기가 아니면 외부 제어 권한이 거부된다(ACCESS_CONTROL_DENY). 펜던트에서 자동·대기로 맞춘다.
4. 재부팅 뒤 MANUAL 모드에서는 `drl_start`가 거부되므로 로봇 모드 설정을 먼저 한다(OFF-4에서는 `set_robot_mode 1`).
5. 남은 알람이 이후 작업의 안전 확인을 막을 수 있다(예: 알람 1206). 지정 번호만 넘기는 절차(`--ack-alarm`)를 썼다.
6. 원인을 알기 전에 프로그램을 다시 실행하지 않는다. 근거: `docs/learn/2026-11-10-fast-static-cycle.md`(`DRL_STOPPED_NO_RESULT` 행)

## 5. 보호 정지·Stop 후 재개 절차

**자동 리셋·원점 복귀는 금지다.** L4(PROTECTIVE_STOP)는 `Stop` 요청 후 작업자 확인까지 대기하며, 재개는 로봇 정지 유지 해제(`stop_release_reason`) 뒤 `resume` 순서로만 한다(`PLAN.md` 5.1절 L4, `PLAN.md` 12절 “자동 리셋·무조건 원점 복귀 금지”, `specs/SPEC-110-abort-timeout.md` 4.2절). WAIT_OPERATOR에는 제한시간이 없고 진입 시각·사유를 기록한다(`PLAN.md` 5.2절, `specs/SPEC-110-abort-timeout.md` REQ-ABT-016).

작업자(비상정지 담당자·운전자) 재개 절차:

1. **원인 확인**: 보호정지·비상정지·알람·heartbeat 사유를 확인하고 로그로 남긴다. 원인을 알기 전에는 프로그램을 다시 실행하지 않는다(`docs/learn/2026-11-10-fast-static-cycle.md`).
2. **로봇 해제**: `robot_controller_node` 파라미터 `stop_release_reason`에 **비어 있지 않은 사유**를 넣는다. 빈 사유는 `REASON_REQUIRED`로 거부되고, 정지 유지 중 실행 중 goal이 있거나 보호정지가 활성이면(`GOAL_ACTIVE`·`PROTECTIVE_STOP`) 설정이 실패한다. 자동 해제는 없다. 근거: `specs/SPEC-010-interfaces.md` 4.5절, `docs/learn/2026-11-11-stop-latch.md`
3. **해제 확인**: 해제 요청과 새 `Stop`이 수 ms 안에 겹치면 정지 유지가 풀릴 수 있는 한계가 있으므로, 해제 뒤 heartbeat `error_code`가 0인지 확인한다. 근거: `specs/SPEC-010-interfaces.md` 4.5절
4. **orchestrator 재개**: `resume_reason`(사유 입력)을 넣는다. 순서가 바뀌어 로봇을 해제하지 않고 재개하면 다음 사이클 시작(PRECHECK)에서 다시 L4가 된다. 근거: `specs/SPEC-010-interfaces.md` 4.5절(운용 순서: 원인 확인 → 로봇 `stop_release_reason` → orchestrator `resume_reason` → 새 `DropTask`), `docs/learn/2026-11-11-stop-latch.md`
5. **재시작**: 새 `DropTask`로 PRECHECK부터 다시 시작한다(WAIT_OPERATOR에서 이어가지 않는다). 근거: `PLAN.md` 5.2절
6. **기록**: 해제 시각·사유·판단을 7절 형식으로 남긴다.

기동 직후 감시 노드가 마지막 controller stamp를 늦게 보던 가짜 `CONTROLLER_HEARTBEAT_LOST` Stop 사례는 2절 체크리스트의 기동 → 2 s 대기 → `error_code` 확인 절차로 다룬다. 근거: `Progress.md` 2절

## 6. 그리퍼 안전

그리퍼는 ROBOTIS RH-P12-RN(A)이며 E0509 플랜지 RS-485 + 제어기 DRL(Modbus-RTU) 배선을 유지한다. PC 쪽으로 옮기는 방안은 사용자 승인 후에만 검토한다. 펌웨어·통신 설정(Protocol Type, ID, 통신 속도)은 바꾸지 않는다. (`AGENTS.md` 장비·안전)

| 항목 | 값 | 근거 |
|---|---|---|
| 기본 전류 | 100 mA | `specs/SPEC-500-robot-control.md` 3절·REQ-GRP-004 |
| 전류 상한 | 150 mA(허용 1~150 mA; 2026-11-09 실기 파지에 쓴 값) | 같음. 150 mA로 파지 4회, 들어 올림·이동 중 낙하 0회(`Progress.md` 2절) |
| 범위 밖 입력 | 지령 전에 거부(전송 없음) | `specs/SPEC-500-robot-control.md` REQ-GRP-004 |
| 현장 운용 범위 | 0~750 pulse(하드웨어 능력 0~1150 pulse와 별개, 참고 구현의 `750 − 값` 반전 규약은 프로젝트 코드에 쓰지 않음) | `specs/SPEC-500-robot-control.md` 3절, `AGENTS.md` 장비·안전 |
| 정적 실행기 기본값 | open 600 pulse / close 750 pulse / 150 mA | `src/static_pick_place/config/static_pick_place.yaml`(`open_pulse`·`close_pulse`·`grip_current_ma`, 2026-11-09 현장 실측 기반) |
| EEPROM·펌웨어·통신 설정 쓰기 | 금지 | `specs/SPEC-500-robot-control.md` REQ-GRP-004, `AGENTS.md` 장비·안전 |
| Modbus-RTU | 펌웨어 V12 이상에서만 사용 | `AGENTS.md` 장비·안전, `PLAN.md` 1.1절 |
| 응답 대기 | 0.5 s(왕복 약 0.58 s 고정; 0.25 s는 가끔, 0.15 s 이하는 항상 놓침) / 전체 제한시간 15 s | `src/static_pick_place/config/static_pick_place.yaml`(`gripper_read_timeout_s`·`gripper_timeout_s`), `Progress.md` 2절 |

그리퍼 이상 시: Hardware Error 비트·응답 시간·상주 루프 생존 카운터를 감시한다(`PLAN.md` 5.3절, `specs/SPEC-110-abort-timeout.md` 4.3절). 통신 설정은 조회만 하고 바꾸지 않는다(`docs/learn/2026-11-10-fast-static-cycle.md` `GRIPPER_*` 행: “통신 설정은 바꾸지 않는다”).

## 7. 기록 의무

현장 세션마다 `records/sessions/OFF-<n>-<date>.md`를 남긴다. 필수 항목은 `records/sessions/TEMPLATE.md`과 `Progress.md` 현장 세션 묶음 기준:

- 참석자(사람·에이전트)와 **비상정지 담당자**(시작 전 입회 확인을 포함)
- 수행 항목(시각별)과 결과, 수집 자료 경로(저장소 밖 원본은 위치·SHA-256 기록, rosbag은 `records/bags.csv`만 남김)
- 미완 항목과 사유
- **장비 이상·보호정지·비상정지 사용 여부와 경위**: 사용하지 않았으면 “비상정지 사용 없음”으로 명시한다(`records/sessions/OFF-4-2026-11-09.md` “장비 이상·보호정지” 절 참고). 4.4절(PLAY 잔류·재부팅)과 같은 대응 경위도 이 절에 남긴다.

보호 정지 해제·재개(5절)도 같은 세션 기록에 해제 사유(`stop_release_reason`에 넣은 값)·시각·판단을 남긴다. 세션이 끝나면 `AGENTS.md` 작업 단계 기록·보고 규칙에 따라 worklog·Progress.md 갱신·보고를 진행한다.
