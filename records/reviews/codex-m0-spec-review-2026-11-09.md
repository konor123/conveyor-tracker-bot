# M0 명세·시험 독립 리뷰

- 시각: 2026-11-09 14:10 KST 이후(GCE, UTC 05시대).
- 수행자: dispatched Codex worker W1, task_bdfb0579b62e / ctx_e72f0478e68c. 모델 세부 식별자·계정 잔여량·초기화 시각은 조회하지 못했으므로 사용량 미확인. 추가 worker 배정 없음. Claude Code coordinator와 분리된 검토이며, 기존 일부 Codex 작성 코드에 대한 조직 수준 완전 독립성까지 의미하지 않는다.
- 대상: SPEC-000/010/100/110/700/800, TEST_SPEC의 SYS·IF·ORC·ABT·MCK·REC·INT-I0, PLAN·Progress·오늘 worklog, 대응 src·tools 구현과 시험.
- 저장소 HEAD 확인: 47308d8200fc65f7475042ed011fb1aba5639cd7. 공유 작업 트리이므로 문서 판독 도중 다른 worker의 REC-007/TC-REC-007-01~05 초안이 나타났다. 아래 REC 판정의 본 검토 범위는 처음 배정된 REC-001~006이며 REC-007 신규 묶음은 독립 확정 대상으로 재검토해야 한다.
- 변경: 이 보고서만 작성. 저장소 파일 편집·commit·push·PC 접속 없음.

## 결론

6종 SPEC 모두 수정 후 확정이 적절하다. M0 TC 전체 묶음도 그대로 확정할 수 없다. 가장 큰 문제는 요구사항·게이트 표 사이의 상충과 실제 확인 범위보다 강한 검증 주장이다. 순수 판정 로직·인터페이스 기본 정의가 전반적으로 갖추어졌다는 점과 ROS 감시 경로가 모든 명세를 구현했다는 주장은 구분해야 한다.

## 검증 근거와 한계

1. GCE `python3 tools/spec/trace.py --area SYS,IF,ORC,ABT,MCK,REC,INT --check`: 초기 검토 시 REQ 49 / TC 85, TC 없는 REQ 0, 테스트 없는 TC 3, 미정의 ID 0, 종료 코드 1. 미연결은 TC-SYS-003-01·02, TC-INT-007-01이다. 영역 단위 trace에는 게이트 필터가 없으므로 이 결과를 곧바로 M0 구현 누락 3건이라고 해석하면 안 된다.
2. GCE `PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider tools/spec tools/eval src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py`: **634 passed, 8 skipped, 3.09 s**. ROS·생성 interface 의존 skip은 통과로 세지 않는다.
3. 대역 엔진은 안내된 PYTHONPATH 그대로 실행하면 `ModuleNotFoundError: drop_mocks`로 수집 실패했다. 패키지 경로를 보충한 `PYTHONPATH=src/drop_mocks:src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider src/drop_mocks/test/test_scenario_engine.py`: **22 passed, 0.68 s**.
4. ROS 빌드·C·I0는 여기서 재실행하지 않았다. `records/worklog/2026-11-09.md` 13:25~14:10 기록의 officepc WSL build 10/10, spec 15, eval 35, orchestrator 617, drop_mocks 58, robot_control 43, contract 25, bringup 8 passed 및 CI 42563966553 attempt 1·2 success를 기록상 근거로만 사용했다. 수정 뒤 nominal 13회 중 1회 실패의 원인은 기록상 미확정이다.
5. 명세의 고정 TF·SI·ROS/steady 분리, interface 공통 필드·상수·snapshot, 모드별 상태 열·deadline 식, 대역의 역할별 좌표 일관성과 가정값 분류, 기록 통계식은 해당 PLAN 절과 대체로 일치한다. 아래는 확정을 막거나 범위를 명시해야 하는 구체 차이다.

## 발견 사항 — 심각도 순

### P1-1. 감시 요구는 존재하지만 ROS 입력에서 이벤트·무효화로 이어지는 경로가 없다

- 위치: `specs/SPEC-000-system.md:130`, `specs/SPEC-110-abort-timeout.md:89`(REQ-ABT-011~014), `src/orchestrator/orchestrator/orchestrator_node.py:514`, `src/orchestrator/test/test_watchdogs.py:113`.
- 문제: `_watch()`는 보호정지·로봇 연결·heartbeat만 본다. FreshnessMonitor/ClockMonitor/GripperMonitor는 watchdogs.py와 U 시험에만 있고 ROS 노드에서 사용하지 않는다. 실행 중 BeltMotion.steady=false도 BELT_UNSTEADY 이벤트로 올리지 않는다. tracking/reset 클라이언트가 없어 역행 시 track·plan 무효화도 없다. `test_clock_regression`은 역행 bool과 decide 등급만 확인하며 TC-ABT-012의 무효화를 전혀 검증하지 않는다.
- 영향: heartbeat가 살아 있는 동안 관측만 오래되거나, ROS 시각이 역행하거나, 파지 뒤 그리퍼 상태가 나빠져도 문서가 약속한 중단이 발생한다는 근거가 없다. M0 순수 판정의 통과를 실제 감시 완성으로 해석할 수 없다.
- 권고 문구: SPEC-110에 “M0는 감시 판정 함수와 이벤트 입력에 대한 U 검증 범위다. ROS 관측 신선도·시각 역행·그리퍼·벨트 이벤트 연결은 각각 G3/G4, G3, G0, G3의 필수 조건이며 해당 실제 경로 사용 전에 C/I 시험을 통과한다.”를 명시하되, M0에서 완성된 감시로 선언하려면 지금 연결·시험한다. 연결 소유자는 orchestrator_node._watch(상태/관측/벨트)와 tracker reset·planner invalidation 경로이고 supervisor의 독립 L4 역할은 유지한다. TC-ABT-012에는 “epoch 증가·기존 track 제거·기존 plan 재사용 거부”를 독립 상태 저장소 fixture로 직접 확인하는 조건을 넣는다.

### P1-2. M0 필수 목록이 SPEC 게이트와 충돌해 미구현 G1/G3/I1을 M0로 끌어온다

- 위치: `TEST_SPEC.md:517`, `specs/SPEC-000-system.md:142`, `TEST_SPEC.md:464`, `Progress.md:82`.
- 문제: M0 표는 SYS 전부와 INT-001~007을 요구하지만 SPEC-000은 SYS-003-01을 G1, SYS-003-02를 G3로 지정하며 INT-007은 I1 replay다. 실제 노드/replay 시험이 없고 trace 종료 코드 1은 재현된다. PLAN 통합 행렬도 I0와 I1을 구분한다.
- 권고 문구: M0 표를 “SYS-001·002·004, IF 전부, ORC 전부, ABT 전부, MCK 전부, REC 전부, INT-001~006”으로 하고 SYS-003-01은 G1, SYS-003-02는 G3, INT-007은 I1 gate로 명시한다. 또는 세 시험을 실제로 구현·실행한 뒤 SYS 전부라는 범위를 유지한다. 게이트별 제외는 검증 결과를 숨기는 전역 trace 무시가 아니라 게이트별 ID 목록으로 남긴다.

### P1-3. REQ-ORC-002와 이를 달고 있는 시험이 정반대 행동을 요구한다

- 위치: `specs/SPEC-100-orchestrator.md:210`, `TEST_SPEC.md:246`, `src/orchestrator/test/test_cycle_runner.py:117`; 비교 `PLAN.md:545` 부근 상태 판정표 RELEASE 행, SPEC-100:93·162.
- 문제: “어느 단계든 OK가 아니면 다음 단계 skill 0회”와 RELEASE 실패 후 RETREAT→VERIFY 지속은 양립하지 않는다. 현재 같은 TC-ORC-002-01 아래 실패 뒤 VerifyDrop 호출을 의도적으로 요구하는 시험이 있어 TC 연결률 100%가 논리 일치를 뜻하지 않는다. WAIT 재관측·L1 재시도·PLAN 통과 보내기도 포괄문에는 예외다.
- 권고 문구: “최종 실패 판정 뒤에는 정상 작업 경로의 다음 skill을 호출하지 않는다. 단 WAIT 재관측, 허용된 L1 재시도, 대기 자세의 통과 보내기 및 해제 후 RETREAT→VERIFY 정리 경로는 각 규정에 따른다. 하드 이상과 RETREAT 실패는 L4가 우선한다.” RELEASE 예외는 별도 TC로 분리하고 나머지 실패 차단 시험을 유지한다.

### P2-1. 0.5초 cancel 승격과 supervisor Stop의 인과 관계를 I0 시험이 입증하지 않는다

- 위치: `src/drop_bringup/test/test_i0_integration.py:248`(robot_hang), `:310`(protective_stop), `TEST_SPEC.md:459`·460.
- 문제: robot_hang은 timeout=1.2와 최종 PROTECTIVE_STOP/Stop 존재만 확인한다. cancel 송신 시각·무응답 0.5 s·승격 전후 등급은 관찰하지 않아 즉시 L4로 잘못 가도 통과할 수 있다. 보호정지 시험은 requester 변수를 읽지만 SUPERVISOR인지 assert하지 않아 orchestrator만 Stop을 보내도 통과한다.
- 권고 문구/시험: TC-INT-002의 “steady cancel_sent→승격 간격은 0.5 s 이상이며 관측 허용 지연 상한을 별도 수치로 정한다; 이전/이후 등급 및 cancel 호출 횟수를 기록한다.” TC-INT-003은 “SUPERVISOR requester의 Stop 요청 ≥1건”으로 한다. 마지막 요청자만 저장하면 coordinator/감시기 경쟁으로 유실될 수 있으므로 mock의 requester별 호출 이력을 관찰한다. 자동 재개 거부·WAIT_OPERATOR 검증 자체는 현재 시험에 있다.

### P2-2. 기록 키 검증이 구현 상수를 정답으로 쓰고 I0는 일부 키만 검사한다

- 위치: `specs/SPEC-100-orchestrator.md:212`, `specs/SPEC-800-records.md:55`, `src/orchestrator/test/test_step_judges.py:368`, `src/drop_bringup/test/test_i0_integration.py:230`.
- 문제: U는 j.metrics와 같은 모듈의 sj.METRIC_KEYS를 비교한다. 두 곳에서 같은 키를 누락하면 통과한다. I0는 12줄·키/값 길이와 6개 단계의 대표 키만 검사해 SPEC-100 4.4절 “키 집합 동일/누락 0”을 확인하지 않는다. 12줄이 중복 step을 포함해도 완전한 단계 집합을 비교하지 않는다. TIMEOUT의 빈 metrics(SPEC-100:150)는 REQ-ORC-004의 무조건 키 집합 규정과도 충돌한다.
- 권고 문구: “관측값을 받은 judge 최종 판정은 4.4절의 해당 모드 키 집합을 기록한다. 관측 응답이 없는 TIMEOUT/PRECONDITION은 metrics={}를 허용하고 reason_code로 구별한다.” 시험에는 SPEC 표에서 고정한 독립 expected key fixture를 두고 모든 step 이름·순서·attempt·키/값 전체를 JSONL과 비교한다. WAIT_CAP budget 동적 키는 원본 budget 성분 집합과 비교한다.

### P2-3. robot/status 계약의 주기 측정 시계가 SPEC-010 v0.3에 어긋난다

- 위치: `specs/SPEC-010-interfaces.md:182`, `tools/testing/contracts/robot_controller_contract.py:96`.
- 문제: 문서는 heartbeat와 robot/status 주기·간격을 수신 steady 시각으로 재도록 정했지만 robot/status 시험은 h.now() 시간창과 메시지 ROS header.stamp 간격을 쓴다. heartbeat 공통 시험만 steady로 바뀌었다. /clock 정지·시계 역행 시 정상 steady 발행을 실패로 보거나 분모가 0이 될 수 있다.
- 권고 수정: robot/status를 `h.record(..., steady=True)`로 수집하고 time.monotonic() 시간창과 수신 시각 차로 8~12 Hz·gap<0.3 s를 검사한다. header.stamp가 ROS 시각임은 별도 검증한다. TC-ABT-009-02의 단순 발행 개수 확인과 이 주기 계약은 별개다.

### P2-4. WAIT_CAP margin의 의미가 TC 입력과 코드에서 1초 다르다

- 위치: `TEST_SPEC.md:254`, `specs/SPEC-100-orchestrator.md:47`, `src/orchestrator/test/test_step_judges.py:175` 부근 test_wait_cap_empty_and_budget.
- 문제: TC는 “margin -0.01 / 0 → PASS_SKIP / OK”인데 구현·시험은 t_available-t_required=0.99 / 1.00, 설정 margin_s=1.0을 쓴다. margin이 남은 시간 차인지 설정 여유를 뺀 잔여인지 정의되지 않았다. PLAN 4.3·9절은 차이≥margin이므로 현재 시험은 PLAN과 맞고 TC 문구가 모호하다.
- 권고 문구: “t_available−t_required=0.99 / 1.00 / 1.01 s, margin_s=1.00 s → PASS_SKIP / OK / OK”로 고정한다. 잔여 slack을 사용하려면 `slack=t_available−t_required−margin_s`라는 명시식을 둔다.

### P2-5. VERIFY 3초 제한 TC가 시간과 최종 결과를 검증하지 않는다

- 위치: `TEST_SPEC.md:260`, `src/orchestrator/test/test_step_judges.py:338`, `src/orchestrator/orchestrator/cycle_runner.py:423`.
- 문제: 현 시험은 responded=False를 직접 주어 Judgement.TIMEOUT/detail UNKNOWN만 확인한다. VerifyDrop 3 s 무응답 대본, deadline, 최종 CycleOutcome UNKNOWN은 확인하지 않는다. judge와 실행기 중 하나가 시간을 잘못 적용하거나 UNKNOWN을 FAILURE로 바꾸어도 이 TC는 통과한다.
- 권고 수정: fake port의 VerifyDrop hang 대본과 fake clock으로 제한시간 직전·경계·직후를 확인하고, 최종 결과 UNKNOWN·reason VERIFY_TIMEOUT·추가 로봇 지령 0회를 검사한다. U의 판정 함수 시험은 보조 시험으로 유지한다.

### P2-6. 관측 freshness 선행조건의 시각 원점이 SPEC-000/PLAN과 다르다

- 위치: `specs/SPEC-100-orchestrator.md:173`, `src/orchestrator/orchestrator/orchestrator_node.py:207`, `specs/SPEC-000-system.md:112`, `PLAN.md:535` 부근 PICK_STONE 선행조건.
- 문제: DetectResult의 검출 시각을 self.clock.ros_now()(서비스 응답 수신 시각)로 정해 1초 freshness를 검사한다. 오래된 영상/오래 걸린 검출도 받은 직후 신선한 것으로 취급된다. stable_window=0.0 요청도 PLAN의 0.5 s 안정 창을 요청하지 않는다. 대역은 stable_window를 의도적으로 무시하므로 현재 I0 정상 통과로 실제 검출 freshness·안정성을 입증할 수 없다.
- 권고 문구: “검출 시각은 DetectObjects 응답에 포함된 영상 노출 시각이다. 검출 안정 창은 0.5 s로 요청한다.” 현재 서비스 응답에는 stamp가 없으므로 응답 관측 stamp 필드를 interface 버전·snapshot과 함께 정의하거나, 대응 image_seq/관측 cache의 시각을 명확히 연결한다. 실제 vision 경로 사용 전 G1에서 지연·오래된 cache 시험이 필수다. M0 임시 포트라면 수신시각 대체의 범위와 한계를 SPEC에 분명히 제한한다.

### P3-1. 문서 버전 참조·범위·미결 목록이 현재 코드와 맞지 않는다

- 위치: `specs/SPEC-000-system.md:21`, `specs/SPEC-100-orchestrator.md:21`, `specs/SPEC-110-abort-timeout.md:21`, `specs/SPEC-700-mocks.md:19`·190, `specs/SPEC-800-records.md:15`·45·70, `src/orchestrator/orchestrator/record_logger_node.py:78`.
- 문제: SPEC-010 현재 v0.3인데 여러 문서가 v0.1/v0.2를 현재 계약처럼 참조한다. SPEC-700은 임시 plan·time_insufficient·계약 틀을 범위 밖/향후 결정으로 적지만 실제 엔진·ROS 노드·시험에 있다. SPEC-800은 기록 노드 저장 동작을 향후로 남겼으나 파일명·invalid.jsonl 보관·쓰기 경로가 구현돼 있다. trial interface_version은 0.2로 고정된다. v0.3이 wire format 변경은 아니므로 그 사실을 구분하면 된다.
- 권고 문구: “wire schema: SPEC-010 v0.2, runtime contract: SPEC-010 v0.3”을 일관되게 쓰거나 실제 적용 버전을 기록한다. SPEC-700에 “M0 planner 대역은 등속 해석해·publish_plan·ComputeIntercept를 제공하며 실제 planner 계약 통과 시 사용하지 않는다”를 최종 계약으로 적는다. SPEC-800에는 실제 경로·날짜 기준 시간대·쓰기/오류 보관 동작을 정의한다. 과거 미결 구현 작업 문구는 삭제한다.

### P3-2. QoS TC의 일부는 실제 노드 대신 시험 자체가 만든 발행자를 검사한다

- 위치: `tools/spec/test_qos_contract.py:61`, `:82`, `TEST_SPEC.md:232` 부근 TC-IF-004-01.
- 문제: publisher_qos_matches_spec_table은 시험이 직접 정답 QoS로 만든 발행자들을 조회한다. mock의 vision/tracker/robot 데이터는 별도 계약 시험이 보완하지만 실제 orchestrator의 task/status·task/step_record QoS는 이 시험으로 확인되지 않는다. TaskStatus는 depth=1 assert도 없다. IF-006은 event handler 검사라 이 공백을 대신하지 않는다.
- 권고 수정: 실제 orchestrator 노드를 띄운 C 시험에서 status의 Reliable/Transient Local/depth1과 step_record Reliable/Volatile/depth100을 조회한다. 현 fixture 시험은 검사 함수 자체의 U/C 시험으로 이름과 목적을 한정한다.

## drop_interfaces 1차 미완 판단

`Progress.md:66`을 “인터페이스 파일이 아직 없다/빌드되지 않았다”는 뜻으로 해석할 근거는 없다. msg 10·srv 6·action 6, snapshot 22개, 공통 요청 순서·상수 시험이 존재하고 PC build·CI 통과가 기록돼 있다. TC-IF-001~003의 구조와 기대값은 현재 범위에서 확정 가능하다. 그러나 SPEC-010 v0.3 계약 전체를 TC-IF-*로 확정하려면 P2-3의 status 측정 시계와 P3-2의 실제 orchestrator QoS 검증을 보완하고 버전 의미를 맞춰야 한다. TC-IF-005는 phase 순서·phase_done·필수 키·키/값 길이를 실제 계약에서 확인하며 기본 요건에 부합한다. IF-006도 해당 노드 생성 시험에 연결되어 있다. 따라서 미완 유지의 타당한 근거는 **독립 계약 리뷰 후 확정 조건 미충족**이며 interface 생성 자체의 미완으로 보고해서는 안 된다.

## 문서별 판정표

| 문서 | 판정 | 근거/확정 조건 |
|---|---|---|
| SPEC-000 | 수정 후 확정 | PLAN의 TF·단위·시계 규칙과 대체로 일치. SYS-003 게이트 충돌과 런타임 무효화 범위 명시(P1-1·2), interface 참조 정리 |
| SPEC-010 | 수정 후 확정 | 기본 msg/srv/action·snapshot·상수 계약 양호. status steady 측정·실제 QoS 검증·버전 의미 정리(P2-3, P3-1·2) |
| SPEC-100 | 수정 후 확정 | 상태 열·모드·통과 보내기·재시도와 순수 구현 일치. ORC-002 예외, timeout metrics, margin·freshness 보완(P1-3, P2-2·4·6) |
| SPEC-110 | 수정 후 확정 | deadline 식·등급 우선순위·steady watchdog U는 측정 가능. 무효화까지 확인하는 TC 및 ROS 연결 게이트 명시(P1-1, P2-1) |
| SPEC-700 | 수정 후 확정 | 엔진 결정성·역할 분리·가정값·장애 주입과 코드 대체로 일치. 이미 존재하는 M0 planner/계약/시나리오 범위 정리(P3-1). ±1 frame은 heartbeat 10Hz와 데이터30Hz의 관측 방식/주입 위상을 구분할 것 |
| SPEC-800 | 수정 후 확정 | 13필수 키·통계·trace와 코드 일치. 기록 노드 최종 계약·전체 metric 검증 정리(P2-2, P3-1). 동시 작성 REC-007은 별도 검토 필요 |

## TC 영역별 판정표

| 영역 | 판정 | 세부 근거 |
|---|---|---|
| SYS | 수정 후 확정 | 001/002/004는 확정 가능. 003-01·02는 G1/G3 실행 보류로 명시해야 M0 범위 확정 가능(P1-2) |
| IF | 수정 후 확정 | 001~003·005·006은 기본 기대값 측정 가능. 004의 실제 대상·depth 및 status 시계 보완(P2-3, P3-2) |
| ORC | 수정 후 확정 | 모드·경계·retry·pass·latch 시험은 구체적. 002 예외와 003-08 margin, 003-14 시간·최종 결과, 004 독립 키 oracle 필요 |
| ABT | 수정 후 확정 | 등급 전수·steady·deadline 경계는 구체적. 012의 무효화 기대가 시험에 없음. ROS 연결 범위/게이트 구분 필요 |
| MCK | 수정 후 확정 | 001/002/003/004/006/007 기본 조건은 구체적이고 구현/시험 연결. 005의 1frame 기대는 데이터/heartbeat/skill별 관측 시계·sampling phase 정의 필요; 기존 PC 간헐 실패는 별도 원인 미확정 |
| REC | 수정 후 확정 | 001/002/004/005/006은 확정 가능. 003은 전체 단계/키 집합 검증 필요. 신규 007은 동시 작업이라 본 보고서로 확정하지 않음 |
| INT-I0 (001~006) | 수정 후 확정 | nominal20/20·시간부족·비활성 기대는 대체로 구체적. 002 cancel0.5s와 003 supervisor 인과 검증 보완. INT-007(I1)은 이 묶음 밖이며 실행 보류 |

전체 M0 확정의 다음 조치: 우선 게이트 목록·ORC 실패 예외·측정 시계·TC 기대와 실제 assertion의 차이를 일치시킨 뒤 coordinator가 ROS C/I0를 PC에서 재확인한다. 실제 장비·PC 접속이나 코드 수정은 이 리뷰 작업에 포함하지 않았다.
