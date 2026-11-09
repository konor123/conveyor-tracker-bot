# Progress — Moving Frame 기반 이동 병뚜껑 공깃돌 투입 진행 계획

- 확정: 2026-11-05 / 구조·인터페이스·판정·중단 규칙: `PLAN.md` / 시험 케이스: `TEST_SPEC.md`
- M0을 먼저 통과한 뒤, 각 절 끝의 **게이트(G0~G7)**를 통과해야 다음 게이트로 넘어간다. 게이트 통과 조건은 `PLAN.md` 11.2절(합격 기준 + REQ 테스트 + 게이트 보고서)을 따른다.
- 트랙 A(로봇·제어), B(비전·보정), C(계획·오케스트레이션), D(시뮬레이션)는 interface·계약 테스트·mock 2종·fixture·rosbag으로 연결해 병렬로 진행한다(`PLAN.md` 7, 11절). Isaac Sim(부록 A)은 실기 게이트를 막지 않는다.
- 아래 합격 기준 수치는 모두 **제안값(초기 가설)**이다. 0절의 실측값이 나오면 조정하고 `records/param_changes.csv`에 남긴다.
- 상태 표기: `[x]` 완료(재현 기록 보관), `[ ]` 완료 조건 미충족(부분 수행 포함).
- 작업 장소 표기(`PLAN.md` 11.4절): `[ON]` 온라인(장비 없이 개발 PC·CI), `[ON*]` 온라인이지만 현장 자료 필요, `[OFF]` 현장 장비 필요. `[OFF]` 항목은 아래 현장 세션 묶음으로 모아 진행한다.
- 구현 항목은 `TEST_SPEC.md`의 해당 시험 케이스(`TC-<영역>-*`)를 먼저 테스트 코드로 쓰고 실패를 확인한 뒤 구현한다.

## 게이트 개요

| 게이트 | 목표 | 절 | 트랙 | 우선 구현 패키지 | 통합 단계(`PLAN.md` 7.5절) | 현장 세션 | 보고서 |
|---|---|---|---|---|---|---|---|
| M0 | spec·시험 명세·계약·오케스트레이터 골격·최소 대역 | 1 | 전원 | `drop_interfaces`, `orchestrator`, `drop_mocks`(mock 2종), `drop_bringup` | I0 | 없음(전부 `[ON]`) | `deliverables/gate-M0-report-<date>.md` |
| G0 | 제어 계약·그리퍼(상주 DRL 경유)·안전·현장 치수 | 0, 2 | A | `robot_control`(adapter, skill 서버) | I3 | OFF-1, OFF-2 | `gate-G0-report` |
| G1 | D455 검출·보정 | 4, 5 | B | `vision`, `calibration` | I1 | OFF-1(자료), OFF-3 | `gate-G1-report` |
| G2 | 정적 파지·정지 벨트 투입 | 5, 6 | A+B | `robot_control`, `orchestrator`(MODE_STATIC), `verifier.py` | I5 | OFF-4 | `gate-G2-report` |
| G3 | 추적·속도 추정·관측 TF·지연 | 7, 8 | B | `tracking` | I1, I4 | OFF-1(자료), OFF-5 | `gate-G3-report` |
| G4 | 예측 타이밍 투입(MODE_PREDICTED) | 9, 11A | C+A | `intercept_planner`, `dynamic_place` | I2 → I5 | OFF-6 | `gate-G4-report` |
| G5 | 비접촉 동기 추종 | 10 | A | `tracking_controller` | I3 → I5 | OFF-5 | `gate-G5-report` |
| G6 | 동적 투입(MODE_SYNCHRONIZED) | 11B | 전원 | 전체 | I5 | OFF-6 | `gate-G6-report` |
| G7 | 통합·복구·비교 평가 | 12, 13 | 전원 | `orchestrator`, `tools/eval` | I5, S | OFF-7 | `gate-G7-report` |
| 병행 | 가상 환경 | 부록 A | D | `drop_sim` | S | 없음(`[ON]`, 실측값은 OFF-1 자료) | — |

## 현장 세션 묶음

각 세션은 시작 조건의 `[ON]` 항목이 모두 끝난 뒤에 잡는다. 세션 기록은 `records/sessions/OFF-<n>-<date>.md`에 남긴다(참석자, 비상정지 담당자, 수행 항목, 수집 자료 경로, 미완 항목).

| 세션 | 시작 조건(먼저 끝낼 `[ON]`) | 할 일 | 수집할 자료 | 종료 조건 |
|---|---|---|---|---|
| OFF-1 현장 실측·자료 수집 | 측정 양식(`records/templates/*.csv`), `tools/eval/record_trial.sh`, D455 확인 스크립트(`tools/bench/check_d455.sh`), 버전 수집 스크립트(`tools/bench/collect_versions.sh`), pulse 실측 스크립트(`tools/bench/gripper_pulse_table.py`, 단위 테스트 통과) | 0절 실측 전부, 벨트 테이프 표식으로 다이얼 설정별 한 바퀴 시간, 병뚜껑 부착·롤러 통과, 수동 낙하 예비 시험, 카메라 최종 설치·프로파일 확인, 그리퍼 pulse ↔ 간격 실측표, 그리퍼 Protocol Type·현재 제어 방식 확인(설정 변경 없이 조회만) | 정지·이동 병뚜껑과 매트 위 공깃돌 rosbag(속도 3종 × 조명 2종, 손으로 가린 구간 포함), 한 bag 안에서 다이얼을 바꾼 속도 변화 bag(조작 시각 기록), 손으로 공깃돌을 넣어 1 s 이상 유지·1 s 안에 꺼냄·가림 후 재출현을 담은 연속 bag, 각 조명에서 벨트면·매트면 여러 위치에 놓은 평면 기준판(크기를 아는 체커보드) bag, 정지 영상 세트(뚜껑 빈 것·든 것·밖으로 나간 것), 작업 구간 기준선 두 개를 지나는 벨트 표식의 휴대폰 고속 영상(국소 속도·맥동 기준), RealSense 파라미터·펌웨어·버전 기록, 카메라 거치 위치 사진·표시 | `records/bench/site-survey-<date>.csv`와 bag 세트 저장, 첫 10 s 시험 bag을 카메라 없이 재생해 영상·CameraInfo·stamp 복원 확인, 퇴장 전 bag별 메시지 수·기간·SHA-256 확인과 별도 매체 복사, 카메라 거치 위치 표시 |
| OFF-2 G0 제어 계약 | `motion_adapter`·`gripper_adapter`·skill 서버 U·C 통과, I0 통과, 스트림·그리퍼 측정 스크립트 | 스트림 주기·지연 측정, 그리퍼 명령 지연, 스트림 동작 중 그리퍼 개방, stale·cancel·통신 단절 정지, I3, skill 소요시간, 공구 무게 등록, Goal Current 결정 | `records/bench/stream-*.csv`, `gripper-*.csv`, I3 로그 | G0 합격 기준 측정 완료 |
| OFF-3 G1 보정·검출 | 보정 도구 U, OFF-1 bag으로 `vision` 검출 `[ON*]` 통과 | eye-to-hand 대응점, 평면·진행 방향, 검증 점 20개 이상, 실시간 검출 확인 | 보정 결과, 검증 점 CSV, 추가 bag | G1 합격 기준 측정 완료 |
| OFF-4 G2 정지 투입 | G0·G1 통과, MODE_STATIC I0 통과, `verifier` U(OFF-1 정지 영상) 통과 | 핑거 어댑터 검증, 파지 50회, 낙하 튕김 시험(해제 높이 × 의도 오차), 정적 투입 50회, 관찰자 판독 | 시도 기록, 관찰자 판독, bag | G2 합격 기준 측정 완료 |
| OFF-5 G3 추적 + G5 추종 | `tracking` U·I1(OFF-1 bag) 통과, `tracking_controller` U, I2 통과 | 속도별 60 s 관측(한 바퀴 시간 GT), 지연 구성 측정, 그리퍼 가림 시간 측정, I4, 비접촉 추종 60 s × 조건, 추종 중 그리퍼 개방 | `tracking-*.csv`, 지연 CSV, bag | G3·G5 합격 기준 측정 완료 |
| OFF-6 G4·G6 동적 투입 | G3 통과(G6은 G5 통과), `DynamicPlace` C, I2 통과 | 저속 30회(G4), 3속도 각 50회(G6) | 시도 기록, 관찰자 판독, bag | G4·G6 합격 기준 측정 완료 |
| OFF-7 G7 통합 | G4 이상 통과, 고장 주입 절차서 | 100 사이클, 안전하게 재현 가능한 고장 주입, 시연 녹화 | 시도 기록, 시연 영상 | G7 합격 기준 측정 완료 |

## 0. 사전 확정 사항 (G0 전에 측정·기록)

| 항목 | 장소 | 측정 방법 | 영향 |
|---|---|---|---|
| 공깃돌 실측: 지름(최대·최소), 높이, 질량, 형상, 색 | `[OFF]` OFF-1 | 캘리퍼, 저울 | 파지 폭 판정, 핑거 패드, 해제 높이, 검출 색 |
| 병뚜껑 실측: 안쪽 지름 `D_in`, 테두리 높이 `h_rim`, 바닥 두께, 색, 접착 방법 | `[OFF]` OFF-1 | 캘리퍼 | 해제 게이트 `e_xy_max`, 해제 높이, 검출 |
| 병뚜껑 벨트 통과: 롤러·하부·가드 간섭, 한 바퀴 주기 `T_lap`, 벨트 전체 길이 | `[OFF]` OFF-1 | 최저 속도에서 한 바퀴 관찰, 줄자. 한 바퀴 시간은 벨트에 붙인 테이프 표식으로 재서 병뚜껑 통과 여부와 무관하게 기준 속도를 얻는다 | `max_passes`, 기준 속도, WAIT_CAP 제한시간 |
| 낙하 예비 시험: 정지 뚜껑에 손으로 해제 높이 5/10/20 mm 각 10회 | `[OFF]` OFF-1 | 수동 | 해제 높이 초기값, 대상물 조합 재검토 여부 |
| 카메라 거치 고정: OFF-1에서 정한 거치 위치·각도를 OFF-3 보정까지 유지 | `[OFF]` OFF-1 | 거치대 테이프 표시, 사진, 평면 기준판 bag | OFF-1 bag을 mm 단위 검출·추적 회귀에 쓸 수 있는지. 거치가 바뀌면 OFF-1 bag은 픽셀 단위 시험에만 쓴다 |
| 그리퍼 RH-P12-RN(A): 하드웨어 Rev, 펌웨어 버전, pulse ↔ 핑거 간격 실측표, 핑거 어댑터 두께·길이 | `[OFF]` OFF-1 | 사양서 + 캘리퍼·게이지 블록 | 파지·개방 판정, 핑거 간섭, `Δ_open` |
| 그리퍼 통신 경로: 그리퍼 Protocol Type(Modbus-RTU 여부), 통신 속도·ID, 현재 쓰는 DRL 코드 | `[OFF]` OFF-1 조회, OFF-2 측정 | 제어기 설정·DRL 코드 확인 | `gripper_adapter` 구현, 해제 지연 |
| 벨트 속도 범위(다이얼 최소·최대·안정 최저 속도), 맥동 | `[OFF]` OFF-1 | 벨트 길이 ÷ 병뚜껑 한 바퀴 시간(스톱워치/고속 촬영) | 운전 속도 범위, 시간 예산 |
| 관측 구간 ∩ 로봇 안전 도달 구간 길이, 관측 구간 시작 ~ 도달 구간 시작 거리 | `[OFF]` OFF-1 | 배치도 작성, 카메라 FOV 투영 | `T_available`, `x_exit`, 추적 안정화 거리(WAIT_CAP) |
| 카메라 설치 높이·각도 | `[OFF]` OFF-1 | 실측 | 픽셀 분해능(0.35 m 수직 시 약 0.55 mm/px, 시야 약 0.70 × 0.45 m), 공깃돌·병뚜껑의 픽셀 크기, 로봇·그리퍼와 카메라 간섭 |
| doosan-robot2 설치 커밋 SHA, 제어기·DRFL 버전 | `[OFF]` OFF-1 | `git rev-parse`, 티치펜던트 | 사용할 수 있는 스트림 인터페이스 |
| 프로젝트 기간·시연 일자, 팀 인원 | `[ON]` | 사용자 확인 | 게이트별 timebox(`PLAN.md` 11.3절), 트랙 배정, 현장 세션 일정 |
| 비상정지 담당자·관찰자 | `[ON]` | 사용자 확인 | 실기 안전 절차, 관찰자 판독 |

측정 결과는 `records/bench/site-survey-<date>.csv`에 남긴다.

## 1. Spec·시험 명세·계약·골격 → M0 (전부 `[ON]`)

- [x] `[ON]` `specs/` 구성, spec 템플릿·REQ ID 체계(`PLAN.md` 8절)
- [ ] `[ON]` SPEC-000(system), 010(interfaces), 100(orchestrator), 110(abort-timeout), 700(mocks), 800(records) v0.1 작성·리뷰
  - SPEC 6종의 M0 범위·해제 후 정리·감시 게이트·기록 계약과 REQ·TC 연결. 독립 리뷰 지적 사항에 대한 재확인이 확정 조건이다.
- [ ] `[ON]` `TEST_SPEC.md`의 M0 대상 시험 케이스(SYS·IF·ORC·ABT·MCK·REC·INT-I0) 리뷰 후 확정
  - M0 게이트 REQ 49·TC 93, 연결 누락 0. 독립 리뷰 지적 사항에 대한 재확인이 확정 조건이다.
- [ ] `[ON]` `drop_interfaces` 1차: 데이터 msg, `TaskStatus`, `Heartbeat`, `StepRecord`, skill action/srv, `DropTask` (TC-IF-*)
  - officepc WSL의 10개 패키지 빌드와 계약 시험 통과. SPEC-010·M0 TC 재확인이 완료 조건이다.
- [x] `[ON]` 공통 요청 필드(`cycle_id`·`step_id`·`deadline`), QoS, revision·epoch 처리, lifecycle 비활성 시 응답을 SPEC-010에 고정
- [x] `[ON]` 타이머별 시계 정책(ROS 시각 / steady)을 SPEC-110에 고정(`PLAN.md` 6.2절)
- [x] `[ON]` 접두 없는 패키지 이름이 underlay와 겹치지 않는지 첫 빌드에서 확인(TC-SYS-*)
- [x] `[ON]` `orchestrator` 골격: `state_machine.py`, `step_judges.py`, `abort_policy.py`, `deadlines.py` 순수 모듈 + `test/fakes/`(skill port·fake clock) 단위 테스트(TC-ORC-*, TC-ABT-*)
- [x] `[ON]` 장애 10종을 `abort_policy`·`deadlines` 단위 테스트의 이벤트 열로 작성(`PLAN.md` 7.3절)
- [ ] `[ON]` `orchestrator_node`, `safety_supervisor_node`, `record_logger_node`
  - F1~F8의 결함 처리 및 F4 감시 연결의 G0/G3/G4 게이트 조건, 해제 후 N1 정리 경로 시험. N1 수정 후 노드 묶음 독립 재검수가 완료 조건이다.
- [x] `[ON]` `drop_mocks`: `scenario_engine`, `mock_world_node`(`role:=vision|tracking|all`), `mock_robot_controller_node`(소요시간·결과 지정형). 정상 경로 우선, 지연·지터 값은 가정값으로 표기(TC-MCK-*)
- [x] `[ON]` 계약 테스트 틀(`test/contract/`, 대상을 인자로 받음), `mock_world_node`의 vision·tracker 계약 통과
- [x] `[ON]` `mock.launch.py`, `launch_testing` 기반 I0 통합 테스트(ROS I0 시나리오 5종)를 CI에 등록(TC-INT-*)
  - `drop.launch.py`·`mock.launch.py`·동시 기동 거부와 ROS I0 정상·장애 시험(pytest에서 launch를 띄우는 방식), GitHub Actions `CI (M0 Offline & I0 Mock Suite)`. TC-INT-001~006 시험 연결 100%, 커밋 `199e363`에서 CI run 42563966553 2회 연속 통과(`records/worklog/2026-11-09.md`).
- [ ] `[ON]` `records/` 디렉터리, trial·step JSONL 기록, `devlog.csv`, `param_changes.csv`(TC-REC-*)
  - JSON Schema·StepRecord/trial JSONL 및 2026-11-09 devlog: unit 793/793·contract 72/72·I0 8/8(officepc WSL). 통합 커밋 `74013cf`, CI 환경 수정 `6bf0307`의 run 42584786687 attempt 1·2는 TC-MCK-005-01 일정 경계 실패로 미통과. TC-MCK-005-01은 최신 프레임만 발행하는 정책(SPEC-700 4.1)과 "예정 시각 직후 첫 수신 프레임" 판정의 불일치로 2코어 CI에서 간헐 실패했다(3.10·3.20·6.27 s). 위상은 제어 시계 시험, 실시간은 stamp·상태 일치 시험으로 분리했다(SPEC-700 v0.5). 커밋 `763a792`의 CI run 42609669267은 attempt 1 통과, attempt 2 실패(실시간 시험의 수신 관측 지연 0.279 s > 상한 0.2 s, 2코어 CI에서 프로세스 정지 0.27~0.5 s로 낡은 ROS 시각의 프레임 발행). CI 연속 통과가 완료 조건이다.
- [x] `[ON]` `tools/spec/trace.py` 추적 행렬 생성(REQ·TC 연결 검사)

**M0 합격 기준(제안):** SPEC 6종 v0.1과 M0 대상 시험 케이스 리뷰 완료 / interface 빌드 / 단위 테스트에서 장애 10종 이벤트 열마다 기대 중단 등급 / I0 정상 시나리오 20/20, ROS I0 장애 시나리오 4종(`robot_hang`, `protective_stop`, `heartbeat_loss`, `time_insufficient`)마다 정의된 중단 등급 동작 / 오케스트레이터 REQ 연결률 100% / `records/`에 StepRecord·trial 기록 생성 확인.

단위·replay 시나리오의 실제 데이터 버전, TCP 지연 모델 등 나머지 대역 기능은 M0 합격 조건이 아니며 해당 게이트에서 추가한다.

## 2. E0509 컨트롤러와 ROS 2 연결 → G0 (트랙 A)

- [x] `[OFF]` Ubuntu PC와 E0509 제어기 TCP 통신
- [x] `[OFF]` doosan-robot2 Jazzy 빌드, SHA `dade65d`·DRCF `GF02100300`·DRFL `GL013304` 기록(`records/bench/versions-2026-11-07-170841.txt`)
- [x] `[OFF]` `model:=e0509 mode:=real` 연결, Joint State 수신, RViz2 동기화
- [x] `[OFF]` 저속 단일 관절 이동
- [x] `[OFF]` RH-P12-RN(A) 개폐(플랜지 RS-485 배선, 제어기 DRL이 통신 담당)
- [ ] **연속 제어 인터페이스 계약 확정** (SPEC-500)
  - [ ] `[ON]` 후보 조사·문서화: `servol_stream` / `speedl_stream`(일반), `servol_rt_stream` / `speedl_rt_stream`(RT, `realtime/connect_rt_control` 등 초기화 필요). 단위(mm, deg)·자세 규약·`time` 인수 의미·timeout 동작
  - [ ] `[ON]` 측정 스크립트(사인 궤적 지령, 주기·지연·지터·추종 오차 기록)를 `mode:=virtual`에서 동작 확인
    - `tools/bench/stream_bench.py`·`stop_bench.py`의 단위 시험·dry-run·실기 기록. virtual 확인과 구분한다.
  - [ ] `[OFF]` OFF-2: 안전 높이에서 작은 진폭 사인 궤적으로 지령 주기·지연 p50/p95/p99·지터·추종 오차 측정(비전·rosbag 부하 포함) → `records/bench/stream-<date>.csv`
    - 로봇 단독 4조건과 D455·rosbag·ArUco 참조 부하 4조건의 원시 기록. 참조 부하 speedl 100 Hz: 간격 p99 10.146 ms, 지연 29 ms, 지연 보정 후 추종 p95 0.062 mm. servol 50 Hz는 실제 이동 부재로 추종 불합격. 생산 vision 부하의 성능 판정과 구분한다(`records/bench/loaded-stream-comparison-2026-11-07.csv`).
  - [ ] `[OFF]` OFF-2: 통신 단절·stale 지령·cancel 시 실제 정지 동작 확인
    - 단독 stale: speedl 20/20 정지, p95 149 ms, 이동 ≤ 2.34 mm(`records/bench/stop-2026-11-07-172032.*`). 참조 부하 stale은 실제 이동 부재로 무효. 통신 단절·cancel·프로세스 종료 시험과 스트림 무응답 원인 검증은 별도 항목.
  - [ ] `[OFF]` OFF-2: `robot_controller_node` 프로세스 종료·executor 정지 시 드라이버·제어기 쪽에서 스트림 동작이 멈추는지, 정지까지 시간·이동 거리 확인. 멈추지 않으면 L0가 controller 밖에서도 성립하도록 정지 경로를 SPEC-500에 정한 뒤 G0를 판정한다
  - [ ] 목표 주기를 못 맞추면 MODE_SYNCHRONIZED를 범위에서 빼고 MODE_PREDICTED까지로 제한(`PLAN.md` 11.3절)
- [ ] **그리퍼 제어 계약 확정** (SPEC-500)
  - [x] `[ON]` 일회성 CLI `gripper open|close|pos`: 운용 범위 0~750 pulse, 기본 전류 100 mA·상한 150 mA(2026-11-09 사용자 지시), 범위 밖 지령 전송 전 거부, `--enable-torque`(Operating Mode 5 확인 뒤 RAM 256만), DRL 상태 STOP·LAST에서만 실행. 상주 adapter 계약과 구분한다.
  - [x] `[OFF]` 일회성 0→500→0 pulse 실제 개폐 관찰 및 `gripper open` 동작 기록(`records/sessions/gripper-takeover-2026-11-07.md`).
  - [ ] `[OFF]` OFF-1: 그리퍼 Protocol Type(Modbus-RTU = 10, RH-P12-RN(A)는 펌웨어 V12 이상 필요)·통신 속도·ID와 현재 쓰는 DRL 코드 확인
    - 플랜지 RS-485 57600 bps 8N1, Modbus ID 1, Model Number 35074 응답. 일회성 DRL 경로이며 펌웨어·Protocol Type 값 조회 및 상주 루프 계약은 별도 검증 대상(`records/sessions/gripper-takeover-2026-11-07.md`).
  - [ ] `[ON]` 상주 루프 DRL 작성(`PLAN.md` 1.1절): 참고 구현 `dsr_study`의 `gripper_service`(상주 루프) 구조에 트리거 레지스터 1개, FC03 상태 읽기, 생존 카운터 추가. Modbus 프레임·레지스터 인코딩은 단위 테스트(TC-GRP-011~013)
  - [ ] `[ON]` `gripper_adapter` transport 구현(`drl_start`, `set_output_register_int`), fake gripper로 지연·무응답·오류·생존 카운터 정지 단위 테스트(TC-GRP-*)
  - [ ] `[ON]` `mode:=virtual`에서 `drl_start`·레지스터 서비스 형식 확인(시리얼·그리퍼 동작은 확인하지 않음)
    - 실기 일회성 경로: `/dsr01/drl/drl_start`, 레지스터 `/dsr01/plc/*`. 하위 1바이트 반환 관측은 `records/sessions/OFF-2-2026-11-07.md`를 기준으로 하며 virtual 시험을 대체하지 않는다.
  - [ ] `[OFF]` OFF-2: 상주 루프 기동(STANDBY), 상태 읽기 레지스터 방향·서비스 확정, 상주 루프가 도는 동안 `move_line`이 받아들여지는지 확인
  - [ ] `[OFF]` OFF-2: 명령 지연(개방 명령 → 개방 시작·완료 p50/p95/p99), 상태(Present Position·Current) 읽기 주기 → `records/bench/gripper-<date>.csv`
  - [ ] `[OFF]` OFF-2: **상주 루프 + 스트림 동시 동작**. 상주 루프가 도는 상태에서 선정한 stream 지령이 받아들여지는지, 안전 높이 사인 추종 중 개방 트리거를 보내 추종이 끊기지 않는지, 개방 지연이 정지 상태와 같은지 확인. 가장 먼저 확인할 위험 항목이다. 안 되면 `PLAN.md` 11.3절 대안(한 DRL 프로그램 안에서 `amovel` + 그리퍼 명령) 적용
  - [ ] `[OFF]` OFF-2: 파지 전류(Goal Current) 결정(공깃돌이 미끄러지지 않는 최소값 × 여유), `Δ_open` 개방 시간 측정
    - 150 mA(사용자 지정)로 파지 4회, 들어 올림·이동 중 낙하 0회. 최소값 탐색과 `Δ_open` 측정은 별도. 그리퍼 DRL의 Modbus 왕복은 약 0.58 s 고정(`records/worklog/2026-11-09.md`).
  - [ ] `[OFF]` OFF-2: Doosan 공구 무게·무게중심 등록(그리퍼 500 g + 핑거 어댑터)
  - [ ] `[ON*]` OFF-1 실측표로 pulse ↔ 간격 변환 테이블 작성·단위 테스트
- [ ] `[ON]` `motion_adapter` 구현: SI/quaternion ↔ mm/deg/ZYZ 변환 + 단위 테스트, 단일 지령 권한, freshness 감시(L0)(TC-ROB-*)
- [ ] `[ON]` `robot_controller_node` skill 서버: `PickStone`, `MoveToPose`, `PlaceStone`(move_line 기반), `SafeRetreat`, `Stop`, Heartbeat 발행, goal deadline 자체 중단
- [ ] `[ON]` `test/fakes/`의 fake transport·fake gripper로 단위 변환·stale·cancel 단위 테스트
- [ ] `[ON]` Doosan `mode:=virtual`에서 E0509 모델·선정한 stream·`move_line` 형식이 동작하는지 확인. 지원하지 않으면 형식 확인용 최소 ROS stub을 두고 선택 근거를 기록(그리퍼 통신은 virtual에서 확인하지 않음)
- [ ] `[ON]` `robot_controller` 계약 테스트를 `mock_robot_controller_node`와 실제 노드(virtual 또는 stub)에 모두 통과
- [ ] `[OFF]` OFF-2: 실제 노드 계약 테스트를 실기에 대해 재실행(안전 높이)
- [ ] `[OFF]` OFF-2: I3. `mock_world_node` + 실제 로봇·그리퍼로 오케스트레이터 연동(안전 높이). deadline 자체 중단, stale 지령(L0), `Stop` 확인
- [ ] `[OFF]` OFF-2: skill별 소요시간 분포 측정
- [ ] `[ON*]` 측정값으로 `PLAN.md` 6절 제한시간 1차 확정, `mock_robot_controller_node`·fake gripper의 소요시간·지연값 갱신
- [ ] `[ON]` 안전 절차 문서: 속도·가속 제한, 작업영역 제한, 비상정지 위치, 관찰자 역할, 보호 정지 후 재개 절차
- [ ] `[OFF]` OFF-2: 안전 절차 현장 확인(비상정지 동작, 작업영역 제한 설정)

**G0 합격 기준(제안):** 0절 항목 기록 완료 / 단위 변환·pulse 변환 테스트 통과 / 통신·추적 손실·cancel 모사 20회에서 정의된 정지·대기 전환 20/20, 예기치 않은 재개 0회 / I3에서 L0·L4 모사 20/20 / 스트림 주기·지연 분포 보고서 / 그리퍼 개방 지연 분포와 스트림 중 개방 결과 보고서.

## 3. Isaac Sim 시뮬레이션 환경 구축 → 부록 A

트랙 D(병행, 전부 `[ON]`) 항목은 부록 A에 둔다. 실기 게이트를 막지 않는다.

## 4. RealSense D455 연결 및 캘리브레이션 → G1 (트랙 B)

- [x] `[OFF]` 현재 조명에서 벨트 안정·빠른1·빠른2·사용자 설정 최고속도 4조건의 RGBD 원시 자료, 경로·SHA-256 및 품질 기록(`records/bench/moving-conditions-2026-11-07.csv`, `records/bags.csv`). 아날로그 수치 눈금·실측 mm/s 없음. 조명 변경 불가로 단일 조명 자료이며 3속도×2조명 전체 요건과 구분한다.
- [x] `[ON]` rosbag 8개의 RGB 8개·깊이 미리보기 6개 MP4 압축·전체 디코딩 검증, 원래 프레임 시각·SHA-256·원본 대응표 보존(`records/media/2026-11-07/README.md`).
- [x] `[OFF]` 카메라 없는 10 s 시험 bag 재생: RGB/CameraInfo 286쌍 stamp 일치·역행 없음(`records/bench/camera-replay-2026-11-07.json`).
- [x] `[OFF]` 카메라 높이 사용자 보고 600 mm의 최종 배치에서 hand-eye용 50자세·250장 수집 및 해시 보존.
- [x] `[OFF]` OFF-1: D455 ROS 2 연결, RGB 1280×800/30 fps, CameraInfo 확인. realsense-ros 4.58.4, FastDDS SHM 확대 프로파일, 고정 K(`records/bench/d455-check-2026-11-07-161256.txt`, SPEC-600 4.8절)
- [ ] `[OFF]` OFF-1: 자동 노출·화이트밸런스 고정, 노출 시간 ≤ 10 ms 목표(50 mm/s에서 흐림 0.5 mm)
- [ ] `[OFF]` OFF-1: RealSense 헤더의 global_time·호스트 offset·장시간 drift 검증. 초기 수신−stamp p50 약22 ms. 속도 자료의 retained 선두 프레임·전송 누락·최대 지연은 bag별 integrity JSON으로 구분한다.
- [x] `[ON]` 왜곡 보정·광선-평면 교차 모듈(`localizer.py`) 합성 카메라로 단위 테스트(TC-VIS-*)
- [x] `[ON]` 보정 도구(`plane_calibration`, `eye_to_hand_calibration`, `tcp_offset`, `charuco`, 보정 파일, `calibrate_camera`·`charuco_view`) 합성 대응점·합성 영상으로 단위 테스트(TC-CAL-001~007, 2026-11-07, SPEC-600 v0.1). 내부 파라미터는 D455 공장값(`records/bench/d455-intrinsics-2026-11-07.txt`), 현장 촬영은 `tools/bench/capture_d455.py`
- [ ] `[OFF]` OFF-3: Calibration Target(ChArUco 5x7) 배치, 카메라-로봇 대응점 취득(로봇 TCP 포인터로 보드 코너 접촉, 면마다 4장면 + verify 5장면, 기록 양식 `records/templates/calib-session.yaml`, SPEC-600 4.4절). 2026-11-07 보드 검출 예비 확인 완료(로봇 접촉 전)
- [x] `[ON*]` eye-to-hand 후보 행렬·프로파일·잔차·원시 자료 보존: ArUco ID25, 40 mm, 30 FIT + 20 holdout, 250장. holdout RMS 1.1473 px. 독립 TCP 접촉 정확도와 운용 보정 채택은 별도 판정(`records/calib/2026-11-07-final-layout/`).
- [ ] `[OFF]` OFF-3: 매트·벨트 평면 높이 및 **벨트 진행 방향 벡터** 측정(벨트에 붙인 병뚜껑을 이동시키며 추적)
    - 잠정: 매트 평면 = ChArUco 1위치 PnP × 후보 외부 변환(재투영 0.13 px, offset 표준편차 0.57 mm), 벨트 평면 = 매트면 + 93 mm(사용자 실측, 평행 가정). 파일 `records/calib/2026-11-09-planes/`. 2026-11-09 컨베이어·매트 재배치 뒤 재측정 대상.
- [ ] `[OFF]` OFF-3: 보정에 쓰지 않은 20개 이상 위치에서 좌표 변환 오차 평가
- [ ] `[OFF]` OFF-3: Depth는 평면 확인·이상 감시에만 사용(설치 높이에서 평면 ROI의 유효 픽셀 비율·편향·표준편차 기록)

## 5. 공깃돌·병뚜껑 인식 및 파지 → G1(검출), G2(파지)

- [x] `[ON]` 검출기 구현: 매트(검정) ROI 공깃돌, 벨트(녹색) ROI 병뚜껑·뚜껑 안 공깃돌·뚜껑 밖 공깃돌(색 마스크 + 원 적합). 합성 영상으로 단위 테스트(TC-VIS-*)
- [ ] `[ON*]` OFF-1 영상으로 색 임계값·원 적합 조정, 하이라이트·그림자·부분 가림 사례 테스트 영상 세트 구성
    - 정지 영상 6장 매트 공깃돌 29/29, 마커·벨트 위 돌 오검출 0. 로봇 투입 뒤 장면(하이라이트로 끊긴 고리, 벨트와 색이 가까운 뚜껑 안 노란 돌)으로 원 적합·뚜껑 안 검출 조정. 라벨 세트(`off1_static_images`)와 회귀 시험은 미구성.
- [ ] `[ON]` 중심 좌표 + 공분산 계산
- [x] `[ON]` 광선-평면 교차: 공깃돌 = 매트 + 높이 오프셋, 병뚜껑 = 벨트 + 테두리 높이, 뚜껑 안 공깃돌 = 벨트 + 바닥 두께 + 높이 오프셋
- [x] `[ON]` `vision_node`의 `DetectObjects` 서비스(안정 창·최소 개수·deadline), Heartbeat, 계약 테스트(TC-VIS-008-01·02 포함 8개, LEGION 2026-11-09)
- [ ] `[ON*]` I1: OFF-1 rosbag replay로 실데이터 검출 회귀 시험
- [ ] `[OFF]` OFF-3: 실시간 검출 recall·오선택률 확인
- [x] `[ON]` Grasp Pose 계산(위에서 수직 접근, yaw는 그리퍼 접근 방향으로 고정): `static_pick_place/plan.py`, 플랜지 = 공깃돌 윗면 + 162.0 mm(현장 실측), 단위 시험
- [ ] `[ON]` 핑거 패드 설계: 공깃돌을 가운데로 모으는 V홈·오목 형상, 공깃돌 아래쪽이 핑거 끝보다 나오도록 파지 높이 설정(낙하 높이 축소)
- [ ] `[OFF]` OFF-4: **핑거 어댑터 초기 검증(필수 선행)**. 실제 공깃돌로 파지 반복성, 매트 여유, `Δ_open` 개방 시 핑거가 병뚜껑 테두리에 닿지 않는지 확인
- [ ] `[OFF]` OFF-4: 파지한 공깃돌 중심 ↔ TCP 오프셋 측정(`tcp_offset.py`)
- [ ] `[ON*]` PICK 단계 판정값 확정: 핑거 간격 범위, 파지 전류 하한, 매트 ROI 소멸 확인(`PLAN.md` 4.3절)
- [ ] `[OFF]` OFF-4: 실제 E0509 파지, 파지 성공률·위치 오차 측정
    - 검출 좌표 1점 육안 확인(수평 2 mm 이하), 파지 4회 성공(659~661 pulse 막힘, 149 mA). 성공률 표본·위치 오차 측정은 별도.

**G1 합격 기준(제안):** 검증 점 XY p95 ≤ 1 mm, 최대 ≤ 2 mm / 공깃돌·병뚜껑 각각 검출 recall ≥ 99%, 오선택 ≤ 1% / 정지 영상 세트에서 `stone_inside` 판독 정확도 ≥ 99% / `DetectObjects` 계약 테스트 통과.

## 6. 정지 벨트 투입 → G2 (트랙 A+B 합류)

- [ ] `[ON]` 오케스트레이터 MODE_STATIC 전체 시퀀스를 I0(mock)에서 통과
- [ ] `[ON*]` `verifier.py`(`VerifyDrop`)를 OFF-1 정지 영상 세트로 단위 테스트(빈 뚜껑·든 뚜껑·밖으로 나감·가림)
- [x] `[OFF]` OFF-4: 벨트를 멈추고 병뚜껑을 도달 구간 안에 둠, 공깃돌 파지 → 뚜껑 위 해제 높이까지 하강 → `Δ_open` 개방 → 수직 후퇴(2026-11-09, 4회 모두 카메라로 뚜껑 안 확인, `records/trials/2026-11-09-static-pick-place/`)
    - 원래 벨트 위치(robot_base x ≈ 0.81 m)는 수직 그리퍼로 도달 경계(J3 = 0°)라 컨베이어를 로봇 쪽으로 옮김. 카메라·로봇 기준 보정은 후보값 그대로 사용.
- [ ] `[OFF]` OFF-4: 해제 높이 = 핑거 끝이 테두리 + `h_clear`(초기 2 mm). 낙하 높이(공깃돌 아래면 − 뚜껑 바닥)를 시도마다 기록
- [ ] `[OFF]` OFF-4: **낙하 튕김 시험**. 해제 높이 2~3단계 × 의도한 해제 오차 0/1/2/3 mm × 각 10회 이상 → 허용 해제 오차를 실측해 `e_xy_max` 확정
- [ ] `[OFF]` OFF-4: 오케스트레이터 MODE_STATIC 전체 시퀀스(I5), 단계마다 성공 판정·제한시간 적용
- [ ] `[OFF]` OFF-4: **투입 판정**. 관찰자 판독을 기준 정답으로 기록하고, `verifier.py` 자동 판정을 보조로 쓴다(`PLAN.md` 4.5절). 결과는 SUCCESS/FAILURE/UNKNOWN
- [ ] `[ON*]` 자동 판정 vs 관찰자 판독 혼동행렬, 단계 판정 결과와 관찰자 기록 대조
- [ ] `[ON*]` 단계 소요시간 p95 → 시간 예산 성분값·단계 제한시간 갱신
- [x] `[OFF]` 현장 정지 pick-and-place 실행기(`static_pick_place`, `[OFF]` 사람 입회): 카메라 1프레임 검출 → 매트 공깃돌 파지 → 빈 뚜껑 투입 → 대기 자세 → `stone_inside` 확인. 처음부터 끝까지 끊김 없는 실행 1회(36.7 s), 단위 시험 19개
- [ ] `[OFF]` OFF-4: 실제 E0509 정적 투입 반복 성공률 측정

**G2 합격 기준(제안):** 50회에서 단일 파지 ≥ 95%, 정적 투입 ≥ 95%(관찰자 판독 기준), 해제 오차(정지 뚜껑 측정값 기준) p95 ≤ 1 mm, UNKNOWN 건수 별도 공개 / 허용 해제 오차 실측 보고 / 단계 판정과 관찰자 기록 불일치 ≤ 5% / 실패한 단계 뒤에 다음 단계가 실행된 사례 0회.

## 7. 컨베이어 위 이동 병뚜껑 추적 → G3 (트랙 B)

- [ ] `[ON]` 벨트 좌표계 등속 칼만 필터 `[x, y, vx, vy]`(진행 방향 성분이 주, 횡방향은 작음) 합성 궤적으로 단위 테스트(TC-TRK-*)
- [ ] `[ON]` 상태 `OBSERVED / COASTING / LOST`, 가림 중 공분산 증가, 재획득 시 게이팅, 통과마다 새 track
- [ ] `[ON]` 다이얼 조작에 따른 속도 변화 검출(혁신값 누적) → `BeltMotion.steady = false`
- [ ] `[ON]` `ResetTracking`, `SelectTarget`(도달 구간에 들어오기 전 가장 가까운 병뚜껑 선택, 진입 시각 예측) 서비스, Heartbeat, 계약 테스트
- [ ] `[ON*]` OFF-1 bag으로 실데이터 추적 확인, 병뚜껑 검출 시각은 노출 시각 기준
- [ ] `[ON*]` rosbag 회귀 자료(정상·가림·프레임 누락·속도 변화) 정리 → I1 CI 자료. 프레임을 뺀 편집 bag으로 `frame_drop_10pct` 시험
- [ ] `[ON]` `occlusion_long`·`speed_change`·`frame_drop_10pct`·`clock_jump` 단위 테스트(`PLAN.md` 7.3절)
- [ ] `[OFF]` OFF-5: 독립 GT로 오차 측정. 실기는 벨트 길이 ÷ 한 바퀴 시간, 또는 별도 영상(같은 카메라 결과끼리 비교하지 않음). sim GT는 `[ON]`
- [ ] `[OFF]` OFF-5: 그리퍼·공깃돌이 병뚜껑을 가리는 시간 측정(대기·접근·하강 중) → τ_max 근거
- [ ] `[OFF]` OFF-5: 영상 처리 지연, 프레임 누락, 추적 손실 조건 확인
- [ ] `[ON*]` (선택) 벨트 표식: 병뚜껑 앞뒤 일정 간격에 표식을 붙여, 뚜껑이 가려져도 벨트 위상을 관측(뚜껑-표식 거리 고정). OFF-5에서 자료를 모으고 온라인에서 효과 평가

## 8. Moving Reference Frame 구현 → G3

- [ ] `[ON]` `world → robot_base → {conveyor_frame, mat_frame, camera_link}` static TF, 로봇 링크는 robot_state_publisher
- [ ] `[ON]` tracker가 `conveyor_frame → cap_<track_id>` 동적 TF 발행(stamp = 상태 시각, orientation identity)
- [ ] `[ON]` **미래 시각 TF 조회 금지**: tf2는 외삽하지 않으므로 예측은 9절 planner가 `TrackedObject`로 계산
- [ ] `[ON]` TF Timestamp 동기화: 관측 시각 기준 변환, `TimePointZero`(최신값) 남용 금지, `/clock` 역행·재시작 시 track·계획 무효화(`PLAN.md` 5.3절)
- [ ] `[ON]` RViz2: 관측 TF + 예측 Marker 별도 표시
- [ ] `[OFF]` OFF-5: 실제 병뚜껑 위치와 TF 위치 비교, I4(실영상 → 계획까지 전체 인식 경로, 로봇 지령 없음)

**G3 합격 기준(제안):** 속도 조건별 60 s 관측에서 독립 GT 대비 XY p95 ≤ 1 mm, 속도 오차 p95 ≤ 2 mm/s / timestamp 역행 0회 / 관측 가능 구간 유효 추적 ≥ 99%(가림 구간 별도 집계) / 가림 시간 분포 보고.

## 9. 이동 목표 위치 예측 → G4 (트랙 C)

- [ ] `[ON]` 예측 `p(t_release) = p(t_state) + v·(t_release − t_state)`와 공분산 전파 단위 테스트(TC-PLN-*)
- [ ] `[ON]` 이미 `t_now − t_obs`에 포함된 지연은 중복 보상하지 않음(단위 테스트로 고정)
- [ ] `[ON]` 시간 예산 판정(`PLAN.md` 9절) `time_budget.py` 구현, `deadlines.py` 연동, 통과 보내기(`max_passes`) 판정
- [ ] `[ON]` 벨트 속도 변화 시 예측 갱신(revision 증가), 계획 만료 처리
- [ ] `[ON]` `ComputeIntercept` 서비스, 계약 테스트, I2(합성 tracked fixture 또는 replay → 실제 tracker 입력으로 예측·시간 예산 CI: 정상, `time_insufficient`, `UNCERTAIN`, 만료·epoch 불일치·clock reset)
- [ ] `[ON]` 실제 planner가 계약 테스트를 통과하면 `mock_world_node`의 임시 plan 발행(`publish_plan`)을 쓰지 않음
- [ ] `[OFF]` OFF-5: 지연 구성 측정: 노출→수신→검출 완료→상태→계획→지령 송신→TCP 응답→그리퍼 개방 명령→개방 시작→공깃돌 이탈
- [ ] `[ON*]` 가림 허용 시간 τ_max 산출: `e_pred ≤ e_pos + |δv|·τ + ½·a·τ² + |v|·δt` → 해제 게이트 값(`PLAN.md` 4.4절). 측정한 가림 시간보다 짧으면 벨트 표식 관측을 도입
- [ ] `[ON*]` OFF-5 bag으로 50/100/200 ms horizon별 예측 오차를 이후 실제 관측과 대조

## 10. Moving Frame 기반 로봇 추종 제어 → G5 (트랙 A)

- [ ] `[ON]` 2절에서 확정한 스트림으로 추종 루프 구현(비전 30 Hz와 분리된 고정 주기)
- [ ] `[ON]` `v_cmd = v_ff(병뚜껑 속도) + Kp·(p_target(t) − p_tcp)`, 속도·가속 제한, freshness 초과 시 정지(L0) 단위 테스트(TC-ROB-*)
- [ ] `[ON]` 초기 실험 조건 정의: 고정 자세, XY 동기 + Z 독립
- [ ] `[ON]` `DynamicPlace` 액션의 접근·동기 구간 먼저 구현, `intercept_plan` revision·epoch 확인
- [ ] `[ON*]` `mock_robot_controller_node`에 TCP 1차 지연 모델 추가(G0 측정값 사용)
- [ ] `[ON]` plan 갱신 중단·epoch 변경·만료 시 `DynamicPlace`가 `PLAN_INVALID`로 끝나고, 이미 받은 것 이하의 revision은 무시하는지 확인(fake transport, TC-ROB-008-01)
- [ ] `[OFF]` OFF-5: **비접촉 동기 추종**. 안전 높이(병뚜껑 테두리 위 30 mm)에서 추종만 시험(I3: 합성 tracked → 실제 planner → 실제 controller, 실제 planner가 낸 `plan_id`·`revision`·`epoch`을 controller가 따르는지 확인 → I5: 실제 병뚜껑)
- [ ] `[OFF]` OFF-5: 추종 중 그리퍼 개방 명령(공깃돌 없이) 시 추종 오차 변화 측정
- [ ] `[OFF]` OFF-5: 벨트 속도 변화에 대한 추종 성능, TCP 위치·속도 오차 측정 → `records/bench/tracking-<date>.csv`
- [ ] `[ON*]` Python executor 지터가 기준을 못 맞추면 adapter·서보 루프만 C++로 이전(이전 후 OFF 재측정)

**G5 합격 기준(제안):** 조건별 60 s, TCP-목표 상대 XY p95 ≤ 1 mm, 상대 속도 p95 ≤ 2 mm/s / 지령 간격 p99 ≤ 1.5 × 주기 / 추종 중 그리퍼 개방 시 상대 XY p95 증가 ≤ 0.5 mm. timebox 안에 통과하지 못하면 G6을 생략한다(`PLAN.md` 11.3절).

## 11. 이동 중 투입 → G4(11A), G6(11B)

공통 시퀀스(`PLAN.md` 4.2절): 공깃돌 파지 → 대기 자세 → 병뚜껑 통과 선택·시간 예산 확인(부족하면 다음 통과) → 접근 → (동기) → 하강 → 해제 게이트 → 개방 지연 동안 동기 유지 → 수직 후퇴 → 판정 → 작업자가 공깃돌 회수. 단계마다 `PLAN.md` 4.3절 판정을 통과해야 다음 단계로 넘어가고, 실패하면 5절 중단 등급으로 처리한다.

### 11A. 예측 타이밍 투입 (MODE_PREDICTED, 중간 단계)

- [ ] `[ON]` 예측 해제점·시각으로 접근하고, 마지막 하강·해제 구간만 미리 계획한 속도로 벨트와 일치시킴(지속 피드백 없음). I0·S에서 시퀀스 확인
- [ ] `[OFF]` OFF-6: 저속(초기 10~30 mm/s 중 실제 안정 범위)에서 시험

**G4 합격 기준(제안):** 해제 horizon 예측 오차 p95 ≤ 1.5 mm / 저속 30회 성공 ≥ 80% / 통과 보내기 횟수와 NOT_ATTEMPTED 횟수 공개.

### 11B. 속도 동기 추종 투입 (MODE_SYNCHRONIZED)

- [ ] `[ON]` 지속 갱신 목표 + 10절 추종 루프로 하강·해제, 해제 게이트 미통과 시 SAFE_ABORT(동기 유지하며 상승). I0·S에서 시퀀스 확인
- [ ] `[OFF]` OFF-6: 실제 E0509 동적 투입, 투입 후 공깃돌이 뚜껑과 함께 1 s 이동하는지 확인
- [ ] `[ON*]` 해제 오차·성공률 분석

**G6 합격 기준(제안):** 저·중·고 3속도 각 50회, 초기 ≥ 80%(개선 목표 ≥ 90%), 해제 오차(추정) p95 ≤ `e_xy_max`, 1 s 이상 유지(관찰자 판독 기준). 게이트 미통과로 시도하지 않은 횟수와 통과 보내기 횟수도 공개.

## 12. 전체 시스템 통합 → G7

- [ ] `[ON]` ROS 2 노드·Launch 통합(real / sim / replay / mock 분리, replay는 로봇 지령 경로 미연결)
- [ ] `[ON]` 상태 머신·단계 판정·중단 등급(`PLAN.md` 4, 5절): PRECHECK, WAIT_CAP, SAFE_ABORT, PROTECTIVE_STOP, WAIT_OPERATOR 포함. I0 전 시나리오 통과
- [ ] `[ON]` 공깃돌 파지부터 투입까지 자동화(시도 사이 공깃돌 회수는 작업자)
- [ ] `[ON]` 검출·파지 실패: 재검출·재파지 최대 N회(L1)
- [ ] `[ON]` 추적 손실·속도 급변: 해제 전이면 SAFE_ABORT(L3)
- [ ] `[ON]` 작업 가능 시간 부족: 다음 통과 대기, 통과 보내기 소진 시 공깃돌을 매트에 내려놓고 NOT_ATTEMPTED
- [ ] `[ON]` 보호 정지·통신 끊김: 자동 리셋·무조건 원점 복귀 금지, 작업자 확인 후 재개(L4)
- [ ] `[ON]` 중단 경로 제한시간(L2 10 s, L3 15 s) 초과 시 승격 확인
- [ ] `[ON]` 고장 주입 절차서(실기에서 안전하게 재현 가능한 시나리오만): 영상 누락, 가림 연장, 다이얼 변경, 지령 지연, 노드 heartbeat 중단, 그리퍼 응답 지연
- [ ] `[OFF]` OFF-7: 고장 주입 시험, 실제 장비 반복 작업(I5)

**G7 합격 기준(제안):** 100 사이클에서 성공·실패·안전 중단·UNKNOWN·NOT_ATTEMPTED·통과 보내기·cycle time 분포 공개 / 주입한 고장마다 정의된 중단 등급 동작 / 중단 경로 제한시간 초과 0회.

## 13. 성능 평가 및 시연

- [ ] `[ON*]` 정지 벨트 투입 성공률(G2)
- [ ] `[ON*]` 벨트 속도별 투입 성공률(G6), 성공률에 표본 수와 Wilson 95% 신뢰구간 표기
- [ ] `[ON*]` Vision 위치·속도 추정 오차(독립 GT 기준)
- [ ] `[ON*]` Moving Frame TF 정확도
- [ ] `[ON*]` TCP 추종 위치·속도 오차
- [ ] `[OFF]` OFF-6·OFF-7: **제어 방식 비교(같은 속도·조명·그리퍼·실패 정의)**
  - Static Target: 마지막 관측 위치를 목표로 해제(지연·이동 보상 없음, 기준선). 지연 동안 이동한 거리와 허용 해제 오차를 함께 제시
  - Predicted Target: 예측 해제점 + 계획된 속도 일치(11A)
  - Moving Frame: 지속 갱신 목표 + 속도 동기 추종(11B)
- [ ] `[ON*]` end-to-end 해제 오차 예산(보정·예측·추종·그리퍼 지연·낙하 성분) 분석
- [ ] `[ON*]` 단계별 소요시간·실패 사유(reason_code) 분포, 중단 등급별 발생 빈도
- [ ] `[ON*]` Cycle Time 및 안정성
- [ ] `[ON*]` 최적화 전·후 비교(`PLAN.md` 12절)
- [ ] `[OFF]` OFF-7: 시연 영상(D455 상단 녹화 + 관찰자 판독 기록 동기화)

## 부록 A. Isaac Sim 시뮬레이션 (병행, 트랙 D, 전부 `[ON]`)

통합 단계 S(`PLAN.md` 7.5절). sim GT는 평가에만 쓰고 제어 입력으로 쓰지 않는다. 실기 게이트(G0~G7)의 통과 조건이 아니다.

### A.1 환경 구축

- [ ] `[ON]` E0509 Import, Articulation Root·Joint Control, RH-P12-RN 모델(ROBOTIS 제공 description을 쓸 수 있는지 확인) 개폐
- [ ] `[ON*]` 공깃돌 Rigid Body: OFF-1 실측 형상·질량 반영, convex collider, contact/rest offset을 크기에 맞춤, 필요 시 CCD. 반발계수·마찰은 OFF-1 낙하 예비 시험과 G2 낙하 시험 결과에 맞춤
- [ ] `[ON]` 매트·미니 컨베이어: 벨트는 **표면 속도(마찰 운반)** 방식. 병뚜껑은 실제로 벨트에 붙어 있으므로 벨트 표면과 함께 움직이는 kinematic 물체로 둔다. 공깃돌은 반드시 동역학 물체로 두고 위치를 덮어쓰지 않는다
- [ ] `[ON]` 고정 RGB-D 가상 카메라(실기와 같은 설치 자세·해상도), ROS 2 Bridge, `use_sim_time` 일괄 적용
- [ ] `[ON]` ROS 2에서 가상 로봇 관절·그리퍼 제어
- [ ] `[ON]` 접촉 안정성 수렴 시험: physics 120 / 240 Hz 비교, step 절반에서 정지 뚜껑 낙하 시 안착·튕김 비율과 최대 관통(공깃돌 높이의 10% 미만 목표)이 수렴하는지 확인
- [ ] `[ON]` sim GT는 평가에만 사용하고 제어 입력으로 쓰지 않음(TC-SIM-*)
- 우선순위: 1차는 기하·TF·시각·추적·상태 전이 검증용 단순 장면(통합 단계 S), 2차에 접촉 물리
- 수렴하지 않으면 sim은 기하·전이 검증용으로만 쓰고 물리 성공률은 보고하지 않음(`PLAN.md` 11.3절)

### A.2 게이트별 시뮬레이션 검증

- [ ] `[ON]` Isaac Sim 파지 검증 (본문 5절 파지 → G2)
- [ ] `[ON]` Isaac Sim 정지 뚜껑 투입 (본문 6절 정지 벨트 투입 → G2)
- [ ] `[ON]` Isaac Sim GT와 TF 위치 비교 (본문 8절 Moving Reference Frame → G3)
- [ ] `[ON]` Isaac Sim 예측 검증 (본문 9절 이동 목표 위치 예측 → G4)
- [ ] `[ON]` Isaac Sim 추종 검증 (본문 10절 로봇 추종 제어 → G5)
- [ ] `[ON]` Isaac Sim 동적 투입 (본문 11B절 속도 동기 추종 투입 → G6)
- [ ] `[ON]` Isaac Sim 전체 자동화 검증(S) (본문 12절 전체 시스템 통합 → G7)
