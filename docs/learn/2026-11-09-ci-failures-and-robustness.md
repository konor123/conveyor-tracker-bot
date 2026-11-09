# CI 실패에서 배운 시계·동시성·검증

11월 9일의 출발점은 CI run 42371150606에서 실패한 두 시험이었다. 하나는 실제 provider가 실행 중인데도 mock이 기동을 거부하지 않은 경우, 다른 하나는 정상 I0 사이클이 HEARTBEAT_LOST로 끝난 경우다. 아래 수치·커밋·run ID의 근거는 [당일 worklog](../../records/worklog/2026-11-09.md)다. 명령은 저장소 루트 기준이며 ROS 명령은 장비와 분리된 PC 임시 워크스페이스에서 실행한다.

## 1. DDS discovery: 프로세스 시작과 통신 준비는 다르다

먼저 officepc WSL의 ROS 2 Jazzy 환경에 소스를 옮겨 실패를 재현했다. 새 프로세스가 기존 서비스를 발견하기까지 8회 측정에서 0.08~2.6초가 걸렸다. 그런데 mock의 시작 시 확인 창은 1초였다. 창이 닫힌 다음 실제 provider가 발견되면 기동 시 검사만으로는 중복 실행을 막지 못한다.

해결은 실행 중에도 감시하고, remap을 적용한 서비스 이름을 `resolve_service_name()`으로 확인하는 것이었다. 시험도 고정 시간 sleep 뒤 한 번 발행하는 대신 구독 매칭과 첫 수신을 확인한 뒤 측정을 시작하도록 바꿨다. 수정 뒤 해당 거부 시험은 CI run 42559480292에서 통과했다([worklog 11:36·12:00·12:46·13:10 절](../../records/worklog/2026-11-09.md), [bringup 시험](../../src/drop_bringup/test/test_mock_bringup.py)).

배운 점: “실행됨”과 “DDS graph에서 발견됨”은 서로 다른 준비 단계다.

흔한 함정: sleep을 늘려 한 번 통과시키고 준비 조건을 검증했다고 생각하는 것이다. 서비스 remap을 빼먹으면 같은 기능을 다른 이름으로 실행했을 때 감시가 빠진다.

직접 해 보기(ROS PC, mock 전용 환경 설정 후):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=183
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest -q src/drop_bringup/test/test_mock_bringup.py
```

## 2. WSL 시계 역행: watchdog도 감시 대상이 된다

I0에서는 heartbeat 콜백과 20 Hz 감시 타이머가 함께 0.4~0.75초 멈췄다. 스레드 덤프는 메인 스레드가 wait_set.wait에서 대기하고 GIL은 비어 있음을 보였다. 이후 time.time이 약 0.7초 역행한 로그를 찾았고, WSL 단독 측정에서도 약 30초마다 시스템 시계가 0.5~0.8초 뒤로 뛰는 현상을 확인했다. UDP 전용으로도 실패가 남아 SHM만의 문제로 볼 수 없었다([worklog 11:36 절](../../records/worklog/2026-11-09.md)).

ROS 시각을 따르는 타이머는 시계가 뒤로 가면 다음 만료까지 더 기다릴 수 있다. heartbeat를 보내는 타이머와 이를 감시하는 타이머가 모두 같은 영향을 받으면, 통신 상대가 살아 있어도 감시 공백이 생긴다. [SPEC-110 REQ-ABT-009](../../specs/SPEC-110-abort-timeout.md)는 감시 타이머에 steady 시계를 요구한다. 기존 fake-clock 시험은 HeartbeatMonitor 판정 함수만 확인했으므로 실제 ROS 타이머의 clock 종류를 놓쳤다.

heartbeat·감시를 steady 타이머로 전환하고 ROS 시각 정지·역행 조건에서 실제 노드의 발행·감시를 확인하는 시험을 추가했다. 수신 간격도 monotonic으로 재고 메시지 header.stamp는 ROS 시각으로 별도 검사한다. 이 분리는 “메시지가 언제 생성됐는가”와 “실제로 얼마 동안 안 왔는가”를 각각 확인한다([타이머 시험](../../src/orchestrator/test/test_watch_timers_ros.py), [controller 계약](../../tools/testing/contracts/robot_controller_contract.py)).

배운 점: 안전 상한을 재는 시계는 작업 데이터의 시계와 독립적이어야 한다.

흔한 함정: WSL에서 찾은 원인을 GitHub 러너에서도 관측했다고 쓰는 것이다. CI 러너의 시계 점프는 직접 확인하지 않았다. steady 타이머로 바꿔도 메시지 stamp까지 단조 증가한다고 가정하면 시험이 다시 틀릴 수 있다.

직접 해 보기:

```bash
# ROS 없는 GCE에서도 가능한 판단 로직 시험
PYTHONPATH=src/orchestrator python3 -m pytest -q src/orchestrator/test/test_watchdogs.py
# ROS PC에서 실제 타이머 확인
python3 -m pytest -q src/orchestrator/test/test_watch_timers_ros.py
```

## 3. py-spy: 전체 CPU 여유가 있어도 콜백은 늦어진다

steady 전환 뒤에도 I0 간헐 실패가 남았다. 전체 CPU는 여유가 있었지만 orchestrator 한 프로세스의 부하와 기상 지연이 컸다. py-spy로 I0 nominal 한 번의 6046 샘플을 수집하니 메인 스레드 비중 74% 중 실제 wait_set.wait 대기는 21%였고, 나머지 대부분은 MultiThreadedExecutor의 대기 집합 재구성이었다. QoS EventHandler waitable 순회·ExitStack·callback group 검사에 비용이 들었다. 의심했던 5 ms `_wait` 폴링은 4.7%였다([worklog 13:25 절](../../records/worklog/2026-11-09.md)).

| 비교 조건, 각 3회 | orchestrator CPU | 감시 타이머 지연 >100 ms | I0 통과 |
|---|---:|---:|---:|
| 기준선 | 41~45% | 2~16회, 최대 0.233 s | 1/3 |
| 기본 QoS 이벤트 핸들러 제거 | 12.7~13.0% | 0회 | 3/3 |
| QoS 불일치 핸들러만 유지 | 34~44% | 2~10회 | 3/3 |

직접 만든 발행·구독에 `use_default_callbacks=False`를 적용하고 QoS 불일치는 계약 시험으로 검사하도록 했다. 이후 5회 측정에서는 CPU 14.3~15.1%, 지연 >100 ms 0회, 5/5 통과였다. 커밋 199e363의 CI run 42563966553은 두 attempt 모두 통과했다. 다만 당시 수정 뒤 누적 13회 중 원인 미기록 실패 1회가 있어 간헐 실패가 완전히 사라졌다고 단정하지 않았다([worklog](../../records/worklog/2026-11-09.md), [SPEC-010 REQ-IF-006](../../specs/SPEC-010-interfaces.md)).

배운 점: 가장 의심스러운 루프보다 프로파일의 실제 비용 분포를 먼저 본다. 전체 CPU 유휴율은 특정 executor의 지연 상한을 보장하지 않는다.

흔한 함정: 시험을 통과시키려고 제품 heartbeat 0.3초 제한부터 느슨하게 만드는 것이다. 제품 기준 변경에는 현장 지연 분포 근거가 필요하다. 기본 이벤트 핸들러를 줄이면 그 역할을 계약 검증으로 보완해야 한다.

직접 해 보기(코드·계약 확인, GCE에서 ROS 시험 통과로 해석하지 않는다):

```bash
rg -n 'use_default_callbacks|EventHandler' src tools/spec/test_qos_contract.py
# ROS PC
python3 -m pytest -q tools/spec/test_qos_contract.py
```

## 4. raise_fault: 검사와 저장 사이의 경쟁

독립 검수는 `RosSkillPort.raise_fault`의 “현재 장애 확인→새 장애 저장” 사이에 다른 스레드가 들어올 수 있음을 찾았다. 감시 타이머와 실행 스레드가 동시에 호출하면 하드 장애를 비하드 장애가 덮어쓰거나 소비 순서 때문에 유실할 수 있었다. Python 코드라는 사실이나 개별 대입의 특성만으로 복합 연산의 원자성을 가정할 수 없다.

`_fault_lock`으로 raise_fault·pending_fault·reset·released·interrupting 관련 상태를 보호하고, 계측 lock으로 원하는 교차 실행 순서를 강제하는 회귀 시험을 추가했다. 무작위 반복에서 우연히 경쟁이 발생하기를 기다리는 대신 검사의 틈을 재현한 것이다([worklog 12:46 절](../../records/worklog/2026-11-09.md), [경쟁 시험](../../src/orchestrator/test/test_ros_skill_port.py)).

배운 점: 경쟁 조건의 시험에는 입력값뿐 아니라 실행 순서도 입력으로 필요하다.

흔한 함정: lock을 추가했다는 코드 모양만 확인하거나, 하드 장애가 낮은 등급에 덮이지 않는다는 최종 불변식을 검사하지 않는 것이다.

직접 해 보기(ROS PC):

```bash
python3 -m pytest -q src/orchestrator/test/test_ros_skill_port.py -k concurrent
```

## 5. W1·W2·W8 독립 리뷰: green이어도 계약은 빠질 수 있다

W1은 M0 명세와 시험 기대값을 검토했다. M0 목록에 후속 게이트의 SYS-003·INT-007이 섞였고, 감시 판정 함수 U 통과와 실제 ROS 입력 연결이 구분되지 않았다. metric 기대값을 구현에서 가져오는 시험, timeout 사건 이름만 주입하고 실제 시계를 넘기지 않는 시험도 보완 대상이었다. 그래서 게이트 범위를 명시하고 독립 metric oracle·실제 시계 경계 시험을 강화했다([worklog W1·W4 절](../../records/worklog/2026-11-09.md), [TEST_SPEC](../../TEST_SPEC.md), [독립 oracle](../../tools/testing/spec_metrics.py)).

W2는 노드 구현을, W8은 수정 후 상호작용을 다시 검토했다. 아래 상태는 W9·W10 반영 기록까지 포함한 11월 9일 시점이다. 수정 구현·PC 시험과 최종 독립 승인 완료를 혼동하지 않는다([worklog W2·W5·W8·W9·W10 절](../../records/worklog/2026-11-09.md), [장애 회귀 시험](../../src/orchestrator/test/test_node_failure_regressions.py)).

| 발견 | 왜 위험한가 | 수정 방향·남은 조건 |
|---|---|---|
| F1 늦은 action 수락 | timeout 뒤 수락된 goal을 잃고 cancel 성공으로 오판 | pending future 보존·늦은 수락 cancel·미확정 goal 동안 새 모션 차단. W8 원 지적 해결 판정 |
| F2 Stop 실패 latch | 최초 전송 실패를 처리 완료로 잠가 복구 뒤 재시도 없음 | 장애 상태와 전달 상태 분리·진행 RPC 하나 유지·실패 재시도. W8 해결 |
| F3 포트 대기 | ROS 시각 동결 중 steady 상한·cancel·shutdown이 작동하지 않음 | 유한 steady cap·guard 추가. W8에서 N1 때문에 부분 해결 판정 |
| F4 필수 감시 입력 미연결 | heartbeat는 정상이어도 그리퍼·벨트·관측 freshness 이상을 못 봄 | U만 M0, 실제 ROS 연결은 G0/G3/G4 필수 C/I 조건. 기능 완성으로 표시하지 않음 |
| F5 logger I/O 오류 | callback 예외 전파·저장 실패 trial의 재전달도 유실 | 예외 격리·정상 append/flush 후 dedup 확정. 자동 재전달·전원 장애 내구성 보장 없음 |
| F6 logger 동시성 | 쓰기 순서와 metadata snapshot이 어긋남 | 공통 RLock으로 직렬화. DDS 수신 순서 재정렬 queue는 아님 |
| F7 유효 JSON의 잘못된 내용 | 배열·잘못된 숫자도 파싱은 성공하지만 후속 변환 실패 | object·변환 검증 및 invalid 격리. W8 해결 |
| F8 설치 schema 부재 | 일반 copy install에서 schema를 못 찾아 검증 우회 | 먼저 fail-closed, W9 share 설치·ament 탐색, W10 저장소 밖 CWD에서 존재 확인 |
| N1 해제 뒤 deadline | 원 cycle deadline이 지나 준비된 RETREAT도 거부·VERIFY 없이 L4 | 해제 후 RETREAT·VERIFY에 독립 유한 예산. 실제 시계 초과 회귀·PC 검증, 최종 독립 재검수 남음 |

N1은 특히 중요한 경계 사례다. 해제 뒤에는 비하드 이상에도 후퇴·결과 확인을 해야 하는데, 기존 사이클 deadline을 그대로 적용하면 그 정리 경로를 차단한다. W9는 기존 단계 제한시간을 독립 적용하면서 하드 이상·실제 후퇴 실패의 L4 우선순위와 동결 시계의 steady 상한을 유지했다([SPEC-100](../../specs/SPEC-100-orchestrator.md), [SPEC-110](../../specs/SPEC-110-abort-timeout.md)).

배운 점: 검수는 함수 하나의 정상 출력뿐 아니라 timeout·수락·cancel·정리 경로의 조합을 살펴야 한다.

흔한 함정: CYCLE_TIMEOUT 이벤트만 넣은 시험을 실제 deadline 초과 시험으로 부르는 것이다. F4를 게이트로 이관한 문구 역시 기능 구현을 대신하지 않는다.

직접 해 보기(GCE, ROS 없이 production 메서드를 추출해 검증하는 회귀 포함):

```bash
PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q src/orchestrator/test/test_node_failure_regressions.py src/orchestrator/test/test_cycle_runner.py
```

이 명령의 성공도 실제 ROS transport에서 늦은 수락·영구 무응답·SIGINT를 모두 주입한 증거는 아니다.

## 6. trace·devlog·CI 결과를 읽는 방법

`trace.py --gate M0 --check`는 M0에 속한 TC만 엄격 검사하도록 추가됐다. 후속 게이트 TC를 삭제하거나 전역 누락을 무시하는 방식이 아니다. W10의 결과는 REQ 49·TC 93·누락 0이었다. devlog는 JUnit XML의 level을 읽어 하루 한 줄을 갱신한다. 최종 PC XML은 873 passed였고 지표는 unit 793/793·contract 72/72·I0 8/8이었다([worklog 17:14 절](../../records/worklog/2026-11-09.md), [devlog](../../records/devlog.csv), [도구](../../tools/eval/devlog.py)).

통합 커밋 74013cf 뒤 CI는 Git dubious ownership 때문에 eval 단계에서 실패했다. 정확한 checkout 경로를 일회성 컨테이너의 safe.directory에 등록한 6bf0307 이후 그 문제는 해소됐지만 run 42584786687 attempt 1·2는 TC-MCK-005-01에서 실패했다. occlusion 예정 3.00초에 대한 첫 COASTING stamp가 각각 3.10·3.20초로 1/30초 허용 오차를 넘었다. 프레임 누락·일정 지연·관측 경로 중 원인은 미확정이며 기준을 완화하지 않고 남겼다. 이 run의 후속 계약/I0·trace 단계는 미실행이다([worklog 17:17·17:24 절](../../records/worklog/2026-11-09.md)).

배운 점: PC 전체 통과, REQ 연결률 100%, CI 성공은 별도 결과다. 최종 devlog의 SHA 47308d8은 시험 소스의 커밋 전 기준점이며 당시 미커밋 통합 변경이 포함됐다는 설명까지 함께 읽어야 한다.

흔한 함정: skip·미실행을 성공에 더하거나, 과거 두 번의 green CI를 최신 커밋의 green으로 옮겨 쓰는 것이다. 최신 완료 조건은 [Progress](../../Progress.md)를 확인한다.

직접 해 보기:

```bash
python3 tools/spec/trace.py --gate M0 --check
cat records/devlog.csv
# 실제 시험 XML을 얻은 뒤 임시 CSV로 집계 연습
python3 tools/eval/devlog.py --junit /tmp/drop-tests.xml --out /tmp/devlog-practice.csv --gate M0:진행 --integration-stage I0
```

마지막 명령의 XML은 직접 실행한 결과로 준비해야 한다. 없으면 명령 성공을 기대하거나 지표를 만들어 채우지 않는다.

## 7. coordinator·worker·실행 장소를 나누기

coordinator는 범위·소유 파일·완료 조건을 정하고 결과를 통합 검증했다. worker는 지정된 코드·시험·리뷰를 수행하고 보고서를 남겼다. W1·W2·W8처럼 독립 리뷰를 둬 작성자의 정상 경로 가정을 다시 확인했다. 같은 작업 트리를 쓰는 경우 파일 소유권을 정하고 다른 worker의 변경을 보존해야 한다([worklog 병렬 작업 절](../../records/worklog/2026-11-09.md)).

GCE는 편집·순수 Python·조정, officepc WSL은 ROS 빌드·통합·반복 측정, GitHub Actions는 CI를 맡았다. 실제 구동 장비 LEGION은 해당 시점 꺼져 있어 그 장비의 I0 반복 측정은 남았다. 계정 사용량과 가용성을 확인해 배정하되 미확인은 수치로 추정하지 않는다. 이 문서 작성도 coordinator가 배정한 범위의 worker 작업이다.

배운 점: “누가 보고했는가”와 “어디에서 어떤 소스로 실행했는가”를 함께 남겨야 증거를 합칠 수 있다.

흔한 함정: worker의 통과 보고를 coordinator가 재확인한 결과처럼 쓰거나, ROS 없는 GCE의 skip을 PC 시험으로 대체해 표현하는 것이다.

직접 해 보기:

```bash
rg -n '수행:|실행 장소:|남은 일:|커밋' records/worklog/2026-11-09.md
```
