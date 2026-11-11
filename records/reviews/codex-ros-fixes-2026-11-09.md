# W7 ROS 통합 실패 수정 — 2026-11-09

- 완료 시각: 2026-11-09T06:09:47.169674+00:00 (UTC).
- 수행: Codex dispatched worker / coordinator Claude Code. task `task_fe6680a4532e`, dispatch `ctx_b347ab183963`.
- 실행: GCE 편집·순수 Python 시험, **officepc WSL Ubuntu-24.04 / ROS 2 Jazzy / rclpy 7.1.12** 빌드·ROS 시험. PC 소스는 `~/ctb-codex`, 전송·로그는 `C:\tmp\codex-w7`만 사용했다. coordinator의 `~/ctb-work/colcon-venv/bin/colcon`은 읽기 전용 실행 경로로 사용했다.
- 사용량: 이 worker의 사용량·잔여 한도·초기화 시각은 미확인이다. 추가 worker 배정 없이 coordinator가 지정한 W7만 수행했다. fleet의 사용량 확인·배정은 coordinator 소관이다.
- 기존 미커밋 W3~W6 통합 변경을 보존했다. Progress.md·records/·AGENTS.md·.github/ 수정, commit/push, 장비 접속, 설정·서비스·인증 변경은 수행하지 않았다.

## 실패별 원인과 판정

### 1. RosSkillPort 6건: 시험 stub 결함

PC 원본 실행 `orch.log`: 6 failed / 4 passed. 모두 `cancel_requested()`의 `goal_handle.is_cancel_requested`에서 `AttributeError`였다. 실제 rclpy goal handle에는 해당 속성이 있으므로 SPEC-100 4.10절의 cancel 검사 구현이 맞다.

- `src/orchestrator/test/test_ros_skill_port.py:26`: `object()` 대신 `SimpleNamespace(is_cancel_requested=False)` goal 사용. port의 설정 입력도 실제 node와 맞췄다.
- PC 수정 후: 10 passed (`fixed-orch.log`). 기존 latch·release·hard fault 경합 assertion을 유지했다.

### 2. TC-MCK-005-01: 서로 다른 시계의 잘못된 대응

시나리오 진행은 `MockLifecycleNode.scenario_t() = ROS now − t0`인데 W4 시험은 최초 ROS↔steady 차이를 고정하여 이후 모든 image_seq·heartbeat·skill 시각에 적용했다. 호스트 ROS 시각이 역행하면 시나리오 진행은 늦어지지만 수신 steady는 늦어지지 않으므로 정상 데이터를 전송 지연 초과로 오판한다. SPEC-700의 시나리오 주입 시각·전송 관측을 검증하려면 동일 시나리오 시계에서 측정해야 한다.

- 원본 단발은 1 passed였으나, PC에서 world·robot·Harness의 ROS now에 동일한 −0.8 s correction을 넣으면 원본의 `t − steady_t0 − image_seq*FRAME ≤ 0.2` assertion이 실패했다 (`controlled-old-mock.log`). 실제 OS 시계·서비스는 변경하지 않았다.
- `tools/testing/drop_ros.py:72`: `SteadyRosClock`가 시험용 `/clock`을 단조 시계에서 발행하며 `Harness(use_sim_time=True)`를 지원한다. 종료 시 publisher thread join·node destroy를 수행한다.
- `src/drop_mocks/test/test_mock_nodes.py:51`: 해당 이벤트 시험만 위 clock을 사용한다. 시나리오 자체 `clock_jump` 이벤트(메시지 stamp −5 s)는 그대로 검증하며 이벤트·전송 허용값을 완화하지 않았다.
- `src/drop_mocks/package.xml:20`: 테스트 의존성 `rosgraph_msgs`. helper는 clock 생성 시에만 해당 메시지를 import한다.
- PC 수정 후 이벤트 시험 4/4 passed. TEST_SPEC TC-MCK-005-01 입력 시계를 명시했다.

### 3. TC-INT-004-01: 장애가 PRECHECK에 먼저 발생하는 시험 경합

기존 시험은 시나리오 t=0.2 s에 vision heartbeat를 끊고, 파라미터 설정·clock reset·goal discovery 뒤 실행되는 PRECHECK보다 반드시 늦다고 가정했다. PRECHECK 전에 끊기면 SPEC-100의 초기 검사 실패 규칙상 `NOT_ATTEMPTED(6) / HEARTBEAT_LOST`가 맞다. L2 FAILURE를 기대하는 시험 조건이 충족되지 않았다.

- PC에서 원본 첫 goal 전송 전에 0.6 s 지연을 넣어 **동일한 result=6 / HEARTBEAT_LOST**를 재현했다 (`controlled-old-hb.log`). 원본 개별 무지연 실행은 1 passed였다.
- `src/drop_bringup/test/test_i0_integration.py:382`: 각 trial 전 모든 heartbeat의 정상 수신을 확인하고, TaskStatus에서 PICK_STONE(vision·controller, 아직 holding=false) 또는 MOVE_STANDBY(tracking, holding=true)를 관찰한 뒤 장애를 주입한다. L2/L3/L4·reason·Stop assertion은 유지했다.
- TEST_SPEC TC-INT-004-01에 주입 상태를 명시했다. PC 수정 후 4/4 passed.

### 4. nominal STEP_TIMEOUT: 신규 steady 상한이 정상 RELEASE를 조기 종료

PC 수정 전 nominal 반복: #1 passed(61.44 s), #2 passed(61.77 s), #3 seed=9 RELEASE STEP_TIMEOUT(32.78 s), #4 passed, #5 seed=9 RELEASE STEP_TIMEOUT. 원인 확보 후 #6 실행은 SIGINT로 중단했으며 성공/실패 통계에서 제외했다.

#3 trial `20261109-143647-0010`의 RELEASE 기록: `t_start_s=1794202609.0248973`, `t_end_s=1794202609.2196949`, ROS 경과 **0.1947976 s**, result TIMEOUT, metrics 없음. W5 `_wait`는 진입 시 남은 ROS 1 s를 곧바로 steady 1 s 상한으로 적용했다. 정상 mock RELEASE는 ROS 약 0.36 s지만 WSL의 약 0.8 s 역행을 지나면 steady 약 1.16 s가 필요하여 ROS deadline 전에 잘렸다. SYNCHRONIZE는 같은 cycle에서 hold=0.335851 s로 이미 통과했으므로 동기 hold 부족 문제가 아니다.

- `src/orchestrator/test/test_node_failure_regressions.py:347`: production RosSkillPort를 추출한 fake-clock 회귀. ROS −0.8 s, 정상 phase 0.36 s, deadline 1 s를 설정하면 기존 구현 **1 failed**. 수정 후 정상 완료(steady 약 1.16 s), 완전 ROS 정지는 steady 2 s에서 종료를 확인한다.
- `src/orchestrator/orchestrator/orchestrator_node.py:153`: 포트 자체 steady 예산을 `남은 ROS 시간 × watchdog.cycle_steady_factor_ratio`(기존 설정 2.0)로 계산한다. ROS deadline 검사와 cycle/cleanup guard는 계속 적용한다. cancel 응답·Stop 호출·L2/L3 독립 steady 제한시간은 바꾸지 않았다.
- SPEC-100 v0.6 4.10절, SPEC-110 v0.4 노드 대기 계약, TEST_SPEC TC-ABT-003-03에 이 명시적 예산을 기록했다. 설정 수치·현장 측정값은 바꾸지 않았다.
- nominal assertion에 실패 cycle의 단계·reason·metrics를 추가해 후속 실패를 구별할 수 있게 했다.

### 5. 반복 중 기록 시각 assertion: 원본 ROS stamp에 단조성을 요구한 시험 결함

첫 수정 후 반복 #1/#2/#4는 통과했고 #3은 20/20 cycle SUCCESS 뒤 기록 assertion에서 실패했다(`t_end_s=1794203602.732513 < t_start_s=1794203603.2291`). #5는 원인 확보 후 중단했다. STEP_TIMEOUT의 재발이 아니며, JSONL은 실제 메시지의 역행한 ROS stamp를 그대로 저장하고 있었다.

- `src/drop_bringup/test/test_i0_integration.py`: 원본 StepRecord의 두 timestamp와 JSONL의 정확한 일치로 검증한다. 단조 순서를 강제하지 않는다.
- `test_node_failure_regressions.py::test_step_record_preserves_backward_ros_stamps`: production serializer에 ROS 8.0→7.5 s 입력을 넣어 양 끝 원본 보존을 확인한다(U).
- SPEC-800 v0.5 및 TEST_SPEC TC-REC-003-01: 단계 timestamp는 원본 ROS 값이며 역행할 수 있음을 명시한다. logger 구현·interface는 바꾸지 않았다.
- 위 수정 포함 최종 PC 전체 및 nominal 8회를 다시 실행해 모두 통과했다. 이전 반복의 실패 로그는 `before-stamp-fix/`에 보존한다.

## PC 전체·반복 검증

최종 PC 전체는 `full.sh`의 순서로 실행하여 모두 exit 0이었다. 소스는 위 최종 archive이며 Git identity 복원 뒤 동일 소스로 전체 순서를 다시 실행했다.

| 순서 | 실행 | 결과 |
|---|---|---|
| 1 | colcon build --symlink-install | 10 packages, 6.12 s |
| 2 | pytest tools/spec | 34 passed, 13.41 s |
| 3 | pytest tools/eval | 47 passed, 1.88 s |
| 4 | pytest src/orchestrator/test | 644 passed, 3.79 s |
| 5 | pytest src/drop_mocks/test --ignore=src/drop_mocks/test/contract | 59 passed, 57.33 s |
| 6 | pytest src/robot_control/test | 43 passed, 1.69 s |
| 7 | pytest src/drop_mocks/test/contract | 26 passed, 18.36 s |
| 8 | pytest src/drop_bringup/test | 8 passed, 113.85 s |
| 9 | trace.py --gate M0 --check | REQ 49 / TC 91 / 누락 0 |

총 pytest **861 passed**, skip·failure 없음. PC 실행 로그는 증거 디렉터리 `final-{build,spec,eval,orch,mocks,robot,contract,bringup,trace}.log`다.

반복 runner 첫 시도는 `--basetemp` 상위 `nominal-results` 폴더 누락으로 8개 호출 모두 fixture setup의 FileNotFoundError에서 종료했다. 제품 코드를 실행한 실패로 세지 않았으며 runner에 mkdir을 추가했다(증거 `runner-setup-errors/`). 기록 시각 시험 수정까지 포함한 최종 nominal 독립 **8/8 passed**, 각 seed 0~19 **20/20 SUCCESS**, 합계 **160/160 SUCCESS**다. 이 반복은 위 전체 묶음의 정상 20사이클 통과와 별개다.

| 반복 | pytest 결과 | 실행 시간 |
|---|---|---|
| 1 | 1 passed / 20 SUCCESS | 62.54 s |
| 2 | 1 passed / 20 SUCCESS | 61.59 s |
| 3 | 1 passed / 20 SUCCESS | 62.48 s |
| 4 | 1 passed / 20 SUCCESS | 61.58 s |
| 5 | 1 passed / 20 SUCCESS | 61.70 s |
| 6 | 1 passed / 20 SUCCESS | 61.78 s |
| 7 | 1 passed / 20 SUCCESS | 61.64 s |
| 8 | 1 passed / 20 SUCCESS | 61.89 s |

PC `summarize.sh`가 각 trial JSONL의 20개 seed·SUCCESS·abort_level=0, trial당 12개 단계 기록을 검사했다. GCE에서도 전달받은 실제 archive를 다시 읽어 trial 160개 SUCCESS 및 단계 기록 **1,920줄**을 교차 확인했다. `nominal-summary.json`에 trial 파일별 SHA-256을 보존했다. 최종 8회에서 raw timestamp 양 끝이 역전된 단계는 0개이며, 실제 역전 사례는 앞선 실패 로그와 추가 U 회귀에 남아 있다.

- 결과 archive: `nominal-results.tar.gz`, SHA-256 `d5a42a80d7dae480aa8500210f669dea79c8425aaca6d2bfea9696627d3e8fe2`.
- STEP_TIMEOUT·HEARTBEAT_LOST 등 실패 결과 0개. 실제 장비 성공률을 의미하지 않는 I0 mock 판정 결과다.

## GCE 검증

```bash
PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q tools src/orchestrator/test \
  --ignore=src/orchestrator/test/test_safety_supervisor.py \
  --ignore=src/orchestrator/test/test_record_logger.py
PYTHONPATH=src/drop_mocks python3 -m pytest -q src/drop_mocks/test/test_scenario_engine.py
PYTHONPATH=src/robot_control python3 -m pytest -q src/robot_control/test
python3 tools/spec/trace.py --gate M0 --check
```

- core **732 passed, 8 skipped**; scenario engine **22 passed**; robot **41 passed, 1 skipped**.
- M0 trace: REQ 49 / TC 91 / TC 없는 REQ 0 / 테스트 없는 TC 0 / 명세 밖 ID 0.
- 수정 Python pyflakes, `git diff --check -- src tools/testing specs TEST_SPEC.md`: exit 0.
- GCE에 ROS가 없어 ROS 파일을 통과로 세지 않았다. 처음 모든 package를 단일 pytest에 합친 실행은 conftest/fakes 모듈 충돌과 ROS import 부재로 collection error 6건이었다. 이후 기존 프로젝트 방식대로 package별 실행·ROS 두 파일 제외로 검증했다. PC에서는 해당 ROS 파일을 포함한 각 package 전체를 실행한다.

## 증거와 재현 환경

- `w7-only.patch`·`w7-files.txt`는 작업 시작 소스 archive와 최종 트리의 차이를 보존한다. 총 11개 파일이며 W3~W6 전체 diff와 구별된다.
- 증거 디렉터리: `records/reviews/evidence/codex-ros-evidence-2026-11-09/`.
- 최종 PC 소스 archive SHA-256: `1645bc880d2eb5ff820e74b8c2a2a1bd79a8430db2c83c1f352dbcd5c4a2506e`.
- 소스 tar만 전달한 첫 전체 실행의 tools/eval 5건은 Git 이력이 없어 실패했다. product/test 변경 대신 원본 저장소의 `git bundle create --filter=blob:none ... HEAD`로 commit·tree 이력만 전달하고 PC에서 `git init`·`git bundle unbundle`·기존 HEAD ref 연결로 동일 이력을 복원했다. Git은 filtered bundle의 clone을 지원하지 않아 unbundle을 사용했다. 새 commit은 만들지 않았으며 records/media blob을 전송하지 않았다. Git SHA는 기준 commit 식별자이며 미커밋 통합 소스의 동일성은 위 archive hash로 식별한다.
- bundle SHA-256: `2a3e450d5ca0dfe990fe6be75c1f59955ddb4b6ac697bc7b40e851bdc7ded38f`.
- 제약: 실제 로봇·센서·LEGION·CI는 실행하지 않았다. 이번 시험은 mock과 PC WSL ROS 환경 검증이다. 임의 크기의 ROS 역행에도 정상 완료를 보장하는 변경이 아니라 ROS deadline과 유한 steady cap을 함께 적용하는 명시적 정책이다.


## 정리·인계

- officepc WSL에서 task workspace 경로·cwd를 사용하는 잔여 프로세스가 없는 것을 확인: `NO_CTB_CODEX_PROCESSES`.
- `~/ctb-codex` 삭제 및 부재 확인: `WSL_CTB_CODEX_REMOVED`.
- `C:\tmp\codex-w7` 삭제 및 Test-Path 부재 확인: `WINDOWS_CODEX_W7_REMOVED`.
- 위 결과는 실제 officepc에서 실행했고, 중앙 증거·보고서는 보존했다. coordinator의 `~/ctb-work` 및 venv를 변경·삭제하지 않았다.
- 요청한 수정·PC/GCE 검증·8회 반복·정리 완료. 후속은 coordinator의 통합 diff 검토와 저장소 단계 기록·커밋·푸시·CI 확인이다. 이 worker는 해당 금지 범위를 실행하지 않았다.
