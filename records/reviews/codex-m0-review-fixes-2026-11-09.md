# M0 독립 리뷰 발견 사항 처리 (W4)

- 시각: 2026-11-09T14:18:22+09:00; 실행 장소 GCE.
- 수행자: dispatched Codex worker task_16d38ff0b49e / ctx_036364d6f050. 계정 사용량·잔여 한도·초기화 시각은 조회 수단이 없어 사용량 미확인이다. 추가 worker 배정은 하지 않았으며 작업 배정은 Claude Code coordinator가 수행했다.
- 이 보고서는 W4 처리 범위다. 공유 작업 트리의 기존 AGENTS.md, devlog/REC-007, W5 노드·시험 변경을 보존했다. commit/push, PC·장비 접속, Progress.md·records/ 수정은 하지 않았다.
- 노드 파일 3개 수정 금지 후속 지시를 준수했다. cycle_runner.py의 W4 수정은 `reason = reason or j.reason_code` 한 줄이며 set_wait_guard·abort_path 변경은 W5 소유다. SPEC-110/800·TEST_SPEC에는 coordinator/W5가 동시에 신규 TC/계약을 넣었으므로 해당 부분을 W4 구현으로 주장하지 않는다.

## 발견별 처리

### P1-1

`specs/SPEC-110-abort-timeout.md:19`, `specs/SPEC-000-system.md:133`, `TEST_SPEC.md:298`

M0 U 판정·이벤트 범위와 ROS 연결 G0/G3/G4 필수 조건을 분리했다. 역행 U는 검출·등급, G3 C는 epoch 증가·기존 track 제거·기존 plan 재사용 거부를 독립 저장소 fixture로 확인하도록 정했다. 실제 ROS 연결은 coordinator 결정대로 구현하지 않았다.

### P1-2

`TEST_SPEC.md:526`, `TEST_SPEC.md:536`

M0는 SYS-001·002·004, IF/ORC/ABT/MCK/REC, INT-001~006으로 한정했다. SYS-003-01=G1, SYS-003-02=G3, INT-007=I1이며 신규 ABT-011~014 -02는 명시된 후속 게이트다.

### P1-3

`specs/SPEC-100-orchestrator.md:220`, `TEST_SPEC.md:247`, `src/orchestrator/test/test_cycle_runner.py:118`

최종 실패의 정상 경로 차단과 WAIT/L1/통과 보내기/해제 후 정리 예외를 정합화했다. RELEASE 실패 정리 시험 마커를 TC-ORC-002-02로 분리했다.

### P2-1

`src/drop_bringup/test/test_i0_integration.py:149`, `src/orchestrator/test/test_cycle_runner.py:471`, `src/drop_mocks/drop_mocks/mock_robot_controller_node.py:86`

coordinator의 범위 확장 승인(msg_f02658bf8811 응답)에 따라 mock에 cancel 횟수·수신 steady 시각·requester별 Stop 수신 이력만 추가했다. 기존 ACCEPT/실행/타이머/QoS는 유지한다. U는 송신→Stop 0.5 s·cancel 1회·L2→L3→L4를 확인하고, I0는 cancel 수신→orchestrator Stop 수신 0.48~0.70 s와 승격 전 L4 부재·최종 L4를 확인한다. 0.02 s는 송신/수신 원점 오차 허용, 0.20 s는 관측 허용 지연이며 PC 검증 전이다. 보호정지는 마지막 requester 대신 전체 이력에서 SUPERVISOR ≥1을 검사한다.

### P2-2

`tools/testing/spec_metrics.py:3`, `src/orchestrator/test/test_step_judges.py:370`, `src/drop_bringup/test/test_i0_integration.py:239`, `specs/SPEC-100-orchestrator.md:222`

SPEC-100 표를 독립 literal fixture로 작성했고 구현 METRIC_KEYS 참조를 제거했다. WAIT budget은 입력 budget 성분 집합과 비교한다. I0는 12단계 이름·순서·attempt·전체 키와 모든 키/값을 수신 StepRecord 대 JSONL로 비교한다. 무관측 TIMEOUT/PRECONDITION은 빈 metrics를 허용한다. VERIFY는 무응답 상태를 관측한 responded=false/verdict 기록을 유지한다.

### P2-3

`tools/testing/contracts/robot_controller_contract.py:61`, `tools/testing/contracts/robot_controller_contract.py:96`, `tools/testing/contracts/robot_controller_contract.py:113`

robot/status 수집을 steady=True, 창·간격·8~12 Hz 계산을 time.monotonic/수신 시각으로 통일했다. header.stamp는 별도 ROS 현재 시각 비교 시험으로 분리했다. ROS 실행은 PC 확인이 필요하다.

### P2-4

`TEST_SPEC.md:255`, `src/orchestrator/test/test_step_judges.py:180`

PLAN.md 4.3·9절의 차이≥설정 margin을 기준으로 판단했다. 코드는 맞으므로 변경하지 않고 TC를 0.99/1.00/1.01 s, margin_s=1.00 s → PASS_SKIP/OK/OK로 명시하고 직후 시험을 보강했다.

### P2-5

`src/orchestrator/test/test_cycle_runner.py:450`, `src/orchestrator/orchestrator/cycle_runner.py:268`

fake VerifyDrop 2.999/3.000/3.001 s와 hang으로 deadline·최종 결과·이유·추가 지령 0회를 검사했다. 신규 시험이 최종 UNKNOWN reason_code 누락을 발견하여 DONE 진입 때 최종 판정 이유를 보존했다.

### P2-6

`specs/SPEC-100-orchestrator.md:188`

PLAN.md 0.2·4.3절에 따라 영상 노출 시각이 원점이다. M0 서비스에는 관측 stamp/image_seq가 없어 수신 시각을 대용하는 임시 포트의 한계를 명시하고, 실제 사용 전 G1 C/I에서 영상 연결·지연 응답·오래된 cache·0.5 s 안정 창을 필수화했다. node 파일 소유권 변경 지시에 따라 stable_window=0.5 요청 코드 수정은 W5/coordinator 후속이며 현재 0.0이다.

### P3-1

`specs/SPEC-010-interfaces.md:24`, `specs/SPEC-700-mocks.md:20`, `specs/SPEC-800-records.md:43`, `src/drop_mocks/test/test_mock_nodes.py:53`

6개 SPEC 버전/이력을 갱신하고 current 계약은 wire schema v0.2/runtime contract v0.4로 분리했다(과거 changelog는 보존). trial interface_version 0.2는 wire 버전이므로 logger 상수를 바꾸지 않는다. planner 등속 대역·publish_plan·ComputeIntercept·time_insufficient·계약 시험을 M0 범위로 정하고 구현 미결 문구를 제거했다. logger의 실제 경로·호스트 로컬 날짜·기동 날짜 고정·append/flush·invalid 보관을 문서화했고 W5 변경의 storage_error/직렬화 계약과 정합화했다. 주입 시각은 30 Hz 데이터/10 Hz heartbeat/skill 검사 주기 및 전송 지연을 구별하고 해당 ROS 시험을 steady 수신 기준으로 보강했다.

### P3-2

`src/orchestrator/test/test_watch_timers_ros.py:100`, `tools/spec/test_qos_contract.py:38`

실제 OrchestratorNode를 생성해 status Reliable/Transient Local/depth1, step_record Reliable/Volatile/depth100을 조회하는 C 시험을 추가했다. DDS discovery가 depth=0을 반환할 수 있어 owning node의 실제 publisher QoS depth도 확인한다. 기존 직접 생성 publisher 시험은 검사 함수 fixture 시험으로 목적과 이름을 제한했다.

## Red → green 근거

1. P2-5: 강화 `test_verify_deadline_and_final_outcome` 수정 전 **2 failed / 2 passed**. 3.001 s와 hang에서 최종 reason_code가 빈 문자열로 실패했다. 순수 실행기 한 줄 수정 후 **4 passed**. 로그 `/tmp/w4-tests.log`는 중간 전체 결과이며 최종 전체 결과는 `/tmp/w4-tests-final.log`, 최초 red는 이 세션의 실행 출력에 남아 있다.
2. P2-2: 프로세스 내 변이로 `_metrics`와 `METRIC_KEYS['PICK_STONE']` 양쪽에서 `grip_current_ma`를 동시에 제거했다. 기존 공유 oracle은 같은 누락을 정답으로 인정하지만 독립 SPEC oracle은 **1 failed**로 잡았다(`/tmp/w4-oracle-red.log`). 변이는 메모리 안에서만 적용하여 저장소 원본은 바꾸지 않았다.
3. P2-1: fake port에서 정확히 0.5 s 대기·cancel 1회·중간 L2→L3 및 최종 L4를 계측한다. W5가 공유 abort_path를 수정하는 중 일시적으로 예상 전이 assertion이 실패해 최종 계약의 `_set` 전이 이력으로 검사했다. 최종 필수 묶음에 포함되어 통과했다.
4. 나머지 TC 보강은 현재 명세/올바른 순수 구현을 더 엄격히 확인한다. ROS 전용 시험은 GCE에 rclpy가 없어 red/green 실행을 하지 못했다. 이들을 실행 통과로 표시하지 않는다.

## GCE 최종 검증

- `PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider tools src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py`: **711 passed, 8 skipped, 5.31 s**. 공유 W5 신규 시험도 이 시점에 포함되었다. ROS/생성 interface skip은 통과가 아니다.
- `PYTHONPATH=src/drop_mocks:src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider src/drop_mocks/test/test_scenario_engine.py`: **22 passed, 0.97 s**.
- 변경 ROS 시험·mock 노드의 `python3 -m compileall -q ...`: 통과(문법 검사만).
- `python3 tools/spec/trace.py`: TRACE.md 생성, 전역 후속 단계의 미구현 TC로 exit 1.
- `python3 tools/spec/trace.py --area SYS,IF,ORC,ABT,MCK,REC,INT --check`: **REQ 50 / TC 99 / TC 없는 REQ 0 / 테스트 없는 TC 7 / 미정의 ID 0**, exit 1. 7개는 ABT-011-02(G3/G4), ABT-012-02(G3), ABT-013-02(G3), ABT-014-02(G0), SYS-003-01(G1), SYS-003-02(G3), INT-007-01(I1)로 전부 M0 밖이다. 전역 trace를 무시하거나 시험 누락을 삭제하지 않았다.
- `git diff --check`: 통과.

## PC 확인 필요

변경 ROS 시험 파일은 다음과 같다. PC에서 격리된 임시 저장소·시험 ROS_DOMAIN_ID로 실행하며 실제 로봇 지령 경로와 현장 저장소를 사용하지 않는다. GCE에서는 PC 접속을 하지 않았다.

- `src/drop_bringup/test/test_i0_integration.py`: 20회 JSONL 전체 키·값, cancel 시간 인과 관계, SUPERVISOR Stop 이력.
- `src/drop_mocks/test/test_mock_nodes.py`: 이벤트 30 Hz 데이터·10 Hz heartbeat·skill별 관측 시계·sampling phase.
- `src/orchestrator/test/test_watch_timers_ros.py`: 실제 orchestrator QoS와 depth.
- `tools/spec/test_qos_contract.py`: QoS 검사 함수 fixture 목적·status depth.
- `tools/testing/contracts/robot_controller_contract.py`: mock 및 실제 controller 계약 시험으로 수신 steady rate/gap과 ROS stamp.
- `src/drop_mocks/drop_mocks/mock_robot_controller_node.py`: 진단 파라미터의 ROS 타입·cancel/Stop 경합 및 기존 동작 유지.

PC 명령(워크스페이스 루트, 별도 임시 mock 환경):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=183
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
colcon test --packages-select orchestrator drop_mocks drop_bringup
colcon test-result --verbose
PYTHONPATH=src/orchestrator:src/drop_mocks:tools/testing:tools/testing/contracts:tools/spec python3 -m pytest -q -p no:cacheprovider tools/spec/test_qos_contract.py src/orchestrator/test/test_watch_timers_ros.py src/drop_mocks/test/test_mock_nodes.py src/drop_mocks/test/contract src/drop_bringup/test/test_i0_integration.py
```

## 남은 보류와 이유

- P2-6 `detect_stones(stable_window=0.5)` 실제 노드 요청 수정은 W5/coordinator 후속이다. 금지된 노드 파일은 W4가 편집하지 않았다. 실제 영상 노출 시각 연결은 G1 gate 필수로 남는다.
- ROS C/I0 검증과 cancel 수신 원점 오차 20 ms·관측 지연 200 ms 상한의 현실성은 PC 실행 전 미확인이다. 실제 cancel 송신 시각은 U에서 정확히 계측하고 I0에서는 수신 시각을 별도로 기록하므로 두 측정의 원점을 혼동하지 않는다.
- G0/G1/G3/G4/I1 미실행 TC는 해당 게이트에서 구현·실행해야 한다. 이 보고서는 실제 감시 ROS 연결·장비 안전 경로 완성을 선언하지 않는다.
- CI/commit SHA는 생성하지 않았다. Progress.md·기록·commit/push·PC 확인은 coordinator 소유다.
