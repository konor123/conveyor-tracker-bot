# RETREAT 비동기 장애 중단 처리 수정

- 확인 시각: 2026-11-09 02:59 UTC / 11:59 KST
- 수행자: Codex dispatched worker, 실행 장소: GCE
- Task: task_b79fa2eb3d23 / Dispatch: ctx_5cb93cff9f13
- 범위: 상태 머신·순수 Python 실행기·단위시험·TEST_SPEC. 커밋·푸시·records 수정 없음.
- 사용량: 본 worker의 수치·잔여 한도·초기화 시각은 조회하지 못하여 사용량 미확인. Coordinator 및 다른 에이전트 사용량은 coordinator가 관리하며 이 worker는 작업을 재분배하거나 별도 worker를 실행하지 않았다.

## 원인

1. `abort_policy.decide()`는 해제 후 모든 비하드 이벤트를 CONTINUE_TO_RETREAT_VERIFY로 올바르게 판정한다. 그러나 기존 `next_state()`는 RETREAT와 VERIFY를 CONTINUE 분기에서 제외했다. VERIFY는 별도 DONE 분기가 있어 예외가 없지만 RETREAT의 FAULT 판정은 마지막 ValueError로 떨어졌다. 해당 조합은 HEARTBEAT_LOST를 포함한 비하드 이벤트 11종 전부다.
2. 실행기는 단계 시작 전 latch된 장애가 있으면 skill을 실행하지 않았다. next_state만 고치면 RETREAT/VERIFY 상태 이름만 지나가고 실제 후퇴·검증은 생략된다. VERIFY 시작 전 장애도 예외 없이 UNKNOWN으로 끝나지만 검증 호출을 생략하는 결함이 있었다.
3. 단계 직후 비동기 장애가 단계 이벤트를 무조건 덮어서 실제 RETREAT_FAILED(L4)가 heartbeat 등 비하드 이벤트(CONTINUE)로 낮아질 수 있었다.
4. 단계 직후 USER_CANCEL 이벤트는 CANCELED 결과가 보존되지 않았다. pending 이벤트와 단계 직후 이벤트 모두 같은 결과 처리 지점을 사용한다.

## 명세 근거

- `specs/SPEC-110-abort-timeout.md:43`~51: 첫 매칭 우선순위는 하드 이상 L4 → released CONTINUE → holding/동적 구간/통과 소진 L3 → 재시도 가능한 단계 실패 L1 → 나머지 L2.
- `specs/SPEC-110-abort-timeout.md:55`: CONTINUE보다 높은 등급은 L4이며 높은 등급을 낮출 수 없다.
- `PLAN.md:607`~624: 해제 뒤에는 중단하지 않고 후퇴·판정으로 진행하며 하드 이상이 최우선이다.
- `specs/SPEC-100-orchestrator.md:92`~94: RELEASE 뒤 실패는 RETREAT → VERIFY, RETREAT 실패는 L4, VERIFY 실패는 UNKNOWN 기록.
- `specs/SPEC-100-orchestrator.md:149`~155: 포트 무응답은 TIMEOUT, holding/released 경계 및 비동기 장애 latch 계약.
- `specs/SPEC-100-orchestrator.md:160`~165: DONE 결과는 VerifyDrop 판정, L4는 PROTECTIVE_STOP, 사용자 cancel은 CANCELED.

해제 전 HEARTBEAT_LOST와 기타 비하드 비재시도 이벤트는 공깃돌을 들지 않고 비동적 단계이면 L2 → ABORT_CYCLE, holding 또는 APPROACH/SYNCHRONIZE/DESCEND/RELEASE이면 L3 → SAFE_ABORT다. RETREAT/VERIFY의 released 문맥에서는 비하드 이벤트가 CONTINUE이며 RETREAT → VERIFY → DONE이다. 모든 단계에서 HARD_FAULTS는 L4 → PROTECTIVE_STOP → WAIT_OPERATOR다. STEP_FAILED/STEP_TIMEOUT만 재시도 조건에 따라 L1일 수 있다.

## 바꾼 파일

- `src/orchestrator/orchestrator/state_machine.py:36`: CONTINUE의 RETREAT → VERIFY 및 VERIFY → DONE 전이를 명시한다.
- `src/orchestrator/orchestrator/cycle_runner.py:238`: 해제 후 비하드 pending 이벤트에도 단계 skill을 실행하며, 단계 직후 hard 이벤트는 latch된 soft 이벤트로 덮지 않는다.
- `src/orchestrator/test/test_fault_transitions.py`: 모든 State 18종 × Event 17종 = 306 pytest 항목. 각 항목에서 holding/released/retry 8조합 및 Mode 3종을 순회하여 7,344개 전이와 정책 기대값을 검사한다. 하드 이상 목록·기대 우선순위는 구현에서 복사 호출하지 않고 명세대로 독립 작성했다. terminal 및 비물리적 문맥 조합도 함수 입력 전체 계약을 확인하기 위해 포함한다.
- `src/orchestrator/test/test_cycle_runner.py:377`: 17 Event × 해제 응답 직후/RETREAT 전·직후/VERIFY 전·직후 5지점 = 85항목, latch된 soft fault 중 후퇴 무응답·판정 실패 2항목, 이후 controller heartbeat L4 승격 1항목.
- `TEST_SPEC.md:274`: TC-ABT-001-02(등급·전이 전수), TC-ABT-001-03(후퇴·검증 실행·hard 우선순위). 승격은 기존 TC-ABT-002-01을 사용한다. 모든 새 시험은 req/tc/level U 마커를 갖는다.

SPEC 정책은 그대로이므로 버전은 바꾸지 않았다. abort_policy.py는 수정이 필요하지 않았다.

## TDD 및 검증

1. 구현 수정 전에 새 시험과 TC를 먼저 작성했다. 실행: `PYTHONPATH=src/orchestrator:tools/spec /tmp/ctb-venv/bin/python -m pytest -q -p no:cacheprovider src/orchestrator/test/test_fault_transitions.py src/orchestrator/test/test_cycle_runner.py --tb=no`. 결과 **59 failed, 367 passed**, 실행 로그 `/tmp/ctb-abort-red.txt`. 실패 원인은 RETREAT ValueError, VERIFY 호출 생략, 후퇴 실패 우선순위 및 cancel 결과였다.
2. 구현 수정 뒤 지정된 전체 시험 실행: `PYTHONPATH=src/orchestrator:tools/spec /tmp/ctb-venv/bin/python -m pytest -q -p no:cacheprovider src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py`. 최종 결과 **591 passed in 1.90s**, 실패·경고 0. 기존 197개 및 새 394개를 포함한다.
3. `python3 tools/spec/trace.py --area ORC,ABT`: **REQ 25 / TC 45 / TC 없는 REQ 0 / 테스트 없는 TC 0 / 명세에 없는 ID 0**. 이 명령이 생성하는 축약 specs/TRACE.md는 작업 범위 밖이므로 원래 HEAD 내용으로 되돌렸다.
4. `git diff --check`: 통과. ROS·실제 장비·I0 mock bringup 시험은 이 worker에서 실행하지 않았다.

## 남은 모호점·통합 확인

- SPEC-110 REQ-ABT-010의 heartbeat 설명은 “동적 구간이면 L3, 그 외 L2”로 요약되어 해제 후 CONTINUE 예외를 반복하지 않는다. 구현과 시험은 같은 명세 4.1의 명시적 ordered policy 및 PLAN 5.2를 따른다. 요약 REQ를 독립적인 예외 규칙으로 읽는 선택지는 ordered policy와 충돌한다. 이 작업에서는 정책을 임의로 바꾸지 않았다.
- SPEC-100 4.7은 latch된 장애가 포트 대기를 None으로 끝낸다고 규정한다. 실제 ROS 포트가 해제 후 RETREAT 결과 대기도 즉시 None으로 끝내면 이 수정은 RETREAT_FAILED로 **L4** 종료한다. 비하드 장애가 존재해도 실제 후퇴 결과를 계속 받을 수 있는지는 coordinator의 ROS 포트/I0 통합 확인 대상이다. 후퇴 무응답을 CONTINUE로 낮추는 선택지는 명시적 RETREAT_FAILED L4 규칙에 어긋나므로 채택하지 않았다.
- records/worklog·Progress·커밋·CI 기록 및 heartbeat 지연 원인 수정은 coordinator 소유이며 이 dispatch에서 수행하지 않았다.
