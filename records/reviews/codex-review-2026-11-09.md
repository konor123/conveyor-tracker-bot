# 미커밋 변경 독립 검수 — 2026-11-09

- 검수자: Codex, Orca dispatch `ctx_c4f077549bf3`, task `task_1c6c6e553969`.
- 장소/시각: GCE, 2026-11-09 03:50 UTC 전후(12:50 KST). 기준 HEAD `cbdc1b9`, main 미커밋 작업 트리.
- 범위: 전체 코드·명세 diff, 새 시험 4개, 관련 호출 경로·타이머·I0 시험. 검수 도중 coordinator의 worklog 수정도 관측했다. 저장소·specs·records는 수정하지 않았고 이 보고서만 작성했다. commit/push, PC 접속, 장비 지령 없음.
- 사용량/배정: dispatched 검수 worker이며 재위임하지 않았다. coordinator가 제공한 배정에 따라 독립 검수만 수행. 이 세션에서 Codex 및 다른 에이전트의 계정 잔여량·초기화 시각은 조회하지 않아 사용량 미확인이다.
- `ocr delegate preview --repo .` 및 `ocr delegate rule` 적용: 정확성·경쟁 상태 중심으로 검토. OCR이 제외한 시험·Markdown도 직접 검토했다. `.codegraph/` 없음.

## 발견 사항 (심각도 순)

**최종 상태:** P1은 아래 재현 후 coordinator가 작업 트리에서 lock으로 수정했다. 후속 독립 확인에서 lock 적용과 200회 동시 soft/hard 검증을 확인했다. P2 두 건은 이 보고서 작성 기준 미해결이다. 아래 P1 행 번호·실패 순서는 수정 전 snapshot에 해당한다.

### P1 — 장애 latch의 검사·저장이 경쟁하여 하드 장애가 영구 누락될 수 있다

위치: `src/orchestrator/orchestrator/orchestrator_node.py:119` (119–122행), 관련 `:129` (pending_fault), `:478` (감시 타이머).

실행기는 Reentrant action callback에서 `pending_fault() → node._watch()`를 호출하며, 다른 executor 스레드는 별도 hb_group의 steady 타이머로 `_watch()`를 호출한다. hb_group의 mutual exclusion은 실행기에서 직접 호출하는 `_watch()`까지 보호하지 않는다. `raise_fault()`의 검사와 `_triggered_faults.add`/`_fault` 저장에는 lock이 없다.

실패 순서:

1. 실행기 스레드에서 비하드 HEARTBEAT_LOST가 119행의 `_fault is None` 검사를 통과하고 121행 직전에 멈춘다.
2. 감시 타이머 스레드가 CONTROLLER_HEARTBEAT_LOST를 set에 추가하고 `_fault`에 저장한다.
3. 실행기 스레드가 재개하여 `_fault`를 HEARTBEAT_LOST로 덮는다.
4. 실행기는 비하드 이벤트만 소비한다. CONTROLLER_HEARTBEAT_LOST는 `_triggered_faults`에 이미 있으므로 지속되는 장애를 다시 올려도 무시된다. RELEASE 뒤에는 비하드 장애로 RETREAT·VERIFY가 계속 진행될 수 있다.

GCE 재현: ROS import 없이 AST로 현재 파일의 `raise_fault`, `pending_fault`, `_interrupting` 세 메서드를 그대로 추출했다. `sys.settrace`와 `threading.Event`로 위 121행 직전 교차 실행을 강제했다. 결과:

```text
race latch: Event.HEARTBEAT_LOST
triggered: {Event.CONTROLLER_HEARTBEAT_LOST, Event.HEARTBEAT_LOST}
runner consumes: Event.HEARTBEAT_LOST
persistent controller loss re-raise: None interrupting: False
```

이는 DDS/WSL 타이밍을 추측한 결과가 아니라 실제 메서드의 강제 교차 실행 결과다. 독립 supervisor가 별도로 Stop을 호출할 수 있어도 orchestrator의 L4 전이·결과·REQ-ORC-010은 보장되지 않는다.

권고: 전용 lock으로 검사·우선순위 판정·set 추가·저장과 pending 소비를 같은 임계구역으로 보호한다. `pending_fault`의 `_watch()`는 lock 밖에서 실행하거나 재진입 설계를 명시한다. `_interrupting()`은 보호된 일관된 snapshot을 읽고 reset도 같은 동기화 원칙을 적용한다. 순차 시험만 있는 `test_ros_skill_port.py`에 soft/hard 동시 raise와 소비/raise 교차 실행 회귀시험을 추가한다. `pending_fault`의 읽기·clear와 concurrent raise 사이 유실 가능성도 함께 방지해야 한다.

### P2 — 새 동시 제공자 감시가 서비스 remapping을 반영하지 않는다

위치: `src/drop_mocks/drop_mocks/ros_common.py:145`, `:152`.

`_exclusive_services`에는 생성 전 상대 이름을 그대로 저장한다. rclpy Jazzy `Node.count_services()`는 namespace 확장은 하지만 remapping은 하지 않는다. 예를 들어 `-r vision/detect_objects:=camera/detect_objects`로 기동하면 자기 서버와 실제 서버 모두 `/camera/detect_objects`에 있어도 감시는 `/vision/detect_objects`만 조회해 0을 보고 종료하지 않는다. remap되지 않은 각 감시 서비스에는 자기 서버 1개가 존재하므로, 일반 경로의 `> 1` 자체는 타당하다. 다만 그것을 모든 ROS 기동 인자에 일반화할 수 없다.

기동 전 조회에도 같은 기존 한계가 있다. 새 기동 후 감시가 그 한계를 보완하지 못하는 사례이며, 현재 기본 launch 경로에서 재현된 장애라는 뜻은 아니다.

권고: `resolve_service_name()`으로 remapping까지 반영한 이름을 기동 전 검사와 기동 후 감시에 일관되게 사용한다(또는 실제 Service.service_name을 저장). remap된 서비스에서 실제 제공자를 뒤늦게 생성하는 계약 시험을 추가한다. GCE에 ROS가 없어 remap subprocess 시험은 실행하지 않았으며, API 동작과 코드 경로에 근거한 정적 발견이다.

근거: [ROS 2 Jazzy rclpy Node 구현](https://raw.githubusercontent.com/ros2/rclpy/jazzy/rclpy/rclpy/node.py), `count_services`, `_count_clients_or_servers`.

### P2 — 새 supervisor 시험의 단발 heartbeat가 discovery 지연 시 유실되어 시험이 불안정하다

위치: `src/orchestrator/test/test_watch_timers_ros.py:66` (66–72행).

volatile heartbeat publisher를 만든 뒤 고정 0.5초만 기다리고 단 한 번 발행한다. subscription과의 DDS 매칭이 그보다 늦으면 메시지는 보존되지 않는다. SupervisorMonitor는 controller heartbeat를 한 번이라도 봐야 손실을 검출하므로, 올바른 steady 타이머 구현에서도 Stop이 영원히 발생하지 않고 2초 timeout으로 실패한다. 이번 수정 배경 자체가 DDS discovery에 최대 약 2.6초가 걸린 관측이다.

권고: publisher의 subscription matching 및 supervisor의 실제 수신을 확인한 후 heartbeat를 끊는다. 검출 지연의 기준도 발행 요청 직전 t0가 아니라 supervisor의 마지막 수신 시각에 둔다. discovery 준비에 넉넉한 상한을 두되, 검출시간 assertion은 별도로 유지한다. 이 결함은 정적 확인이며 GCE에서는 ROS 시험을 실행하지 않았다.

## 요청한 네 가지 집중 확인

### (a) spin_once 루프

`ros_common.py:211`의 `spin_once(timeout_sec=0.1)`는 고정 100ms sleep이 아니다. 준비된 callback이 있으면 즉시 ThreadPoolExecutor에 submit하며, rclpy `spin()`도 내부에서 같은 실행 루틴을 반복한다. 따라서 모든 callback에 100ms 지연을 더한다는 결함은 찾지 못했다. 차이는 작업이 없을 때 유한 시간으로 돌아와 conflict를 확인한다는 점이다. conflict timer 실행 후 대략 최대 추가 0.1초의 idle 대기 가능성은 있으며 OS scheduler 지연은 별도다.

`executor.shutdown()` 없이 node를 먼저 파괴하는 cleanup은 기존 main에도 있던 패턴이다. 새 conflict 종료 경로는 동작 중 action/service와 겹칠 수 있으므로 후속 ROS 검증에서 active action 중 conflict도 시험할 가치가 있다. 이번 검수에서 종료 불능을 재현하지 않아 확정 결함으로 분류하지 않았다.

근거: [ROS 2 Jazzy rclpy Executor 구현](https://raw.githubusercontent.com/ros2/rclpy/jazzy/rclpy/rclpy/executors.py), `Executor.spin`과 `MultiThreadedExecutor._spin_once_impl`.

### (b) count_services 자기 서버

정상 생성 완료 후 executor를 돌리므로 감시 대상 vision 2개, tracking 2개, robot/stop에는 각각 자기 Service가 생성된다. 일반 namespace 확장에서는 자기 서버를 세는 graph count에 다른 서버가 더해지면 >1인 판정은 맞다. discovery가 비동기라 즉시 정확한 값이라는 보장은 하지 않지만 지속 제공자는 후속 주기에 검출한다. remapping 예외는 위 P2에 적었다.

### (c) RELEASE 시점 및 latch 소비

`place_feedback('RELEASE')`의 `_phase_done`은 `phase == RELEASE && phase_done` feedback을 기다린 뒤 metrics를 돌려준다. `_released=True`는 그 응답을 실제 소비한 시점이며 CycleRunner `_do_release`도 같은 반환값을 받자마자 released=True로 바꾼다. 명세에서 RELEASE 응답을 phase_done feedback으로 해석하는 현재 action 계약과 일치한다. `judge_release`가 실패해도 응답 수신 자체로 released가 되는 것도 기존 CycleRunner와 같다. 최종 PlaceStone/ DynamicPlace action result까지 기다리는 의미로 해석하면 안 된다.

순차 실행에서는 pending_fault가 `_fault`를 비우고 set으로 재발행을 막아도, CycleRunner의 AbortTracker가 처리된 등급을 유지하므로 비하드 cleanup 진행과 hard 승격이 가능하다. 따라서 소비 방식만으로 별도 순차 결함을 주장하지 않는다. 다만 실제 latch는 포트 내 영구 Event가 아니라 소비형이며 명세의 '유지'는 실행기의 등급 유지와 합쳐 이해해야 한다. 동시 접근에서는 P1의 유실이 실제로 가능하다.

### (d) use_sim_time / Isaac Sim / replay

발행 타이머와 감시만 steady로 바꾸고 stamp·관측·action 진행·deadline은 ROS 시각으로 남겨두는 방향은 PLAN 7절 및 SPEC-110 시계 정책에 맞다. `/clock`이 멈춰도 process liveness heartbeat가 유지되는 것이 의도이며, simulation pause 중 정상 동작하는 process를 죽었다고 간주하지 않는다. 반면 replay에서 bag의 과거 heartbeat만 재생하거나 heartbeat publisher까지 pause하면 steady 감시는 0.3초 후 손실을 감지한다. replay는 로봇 지령 경로를 연결하지 않는 기존 원칙을 지켜야 한다. replay 속도에 맞추려고 안전 감시 시계를 다시 ROS 시각으로 돌리는 것은 권고하지 않는다.

새 freeze 시험들은 원래 ROS 타이머였다면 heartbeat/status 7회 조건 또는 ClockType assertion에서 실패하므로 수정 전에도 그대로 통과하는 시험은 아니다. 다만 발행량 시험(`test_mock_nodes.py:305`, `test_robot_controller_heartbeat.py:34`)도 0.3초 discovery warmup + 정확히 1초 수집에 의존해 느린 호스트에서 실패할 수 있다. 매칭/첫 수신을 확인한 후 별도 측정 구간을 여는 편이 좋다. 주파수 검증과 discovery 준비를 분리하되, 무조건 긴 대기로 주기 위반을 감추지 않는다. RosSkillPort 시험은 실제 ROS executor·스레드 교차를 포함하지 않아 P1을 잡지 못한다.

## 남은 I0 간헐 실패에 대한 권고 (결정 아님)

1. **우선은 I0 실행환경과 시험 여유를 분리해서 조정한다.** nominal은 `test_i0_integration.py:171`에서 mock skill만 0.12배로 압축하고 heartbeat 10Hz·감시 0.3초는 압축하지 않는다. 짧은 phase, 제한시간 15초, 스레드 기상 지연이 결합하면 nominal의 목적 외에 호스트 scheduler stress까지 섞인다. scale을 덜 압축하거나 1.0으로 돌리는 별도 반복 비교, 사이클 상한의 일관된 조정, discovery readiness 확인을 우선 권고한다. scale 변경 시 phase별 synchronization hold 측정 및 TIMEOUT 발생 단계도 확인해야 한다. 20 seeds와 성공 조건을 줄이거나 실패 시 무조건 재시도하는 방식은 권고하지 않는다.
2. **단, nominal의 timeout만 늘려도 HEARTBEAT_LOST는 해결되지 않는다.** 프로세스가 실제로 0.3초 실행되지 못한다면 현 제품 감시기가 중단하는 것은 명세상 정답이다. heartbeat 송신 간격·수신 간격·executor callback 지연·timeout 단계·ROS clock jump를 같은 steady 축으로 기록하고, native Linux/CI에서 반복 비교하여 host 영향과 제품 callback starvation을 구분해야 한다. 압축을 완화해도 부하가 줄어든다는 보장은 없다. 시험 전용 heartbeat 여유를 늘리는 선택은 별도 '여유 있는 nominal 흐름 시험'으로 명시하고, 제품 0.3초 설정의 손실 검출 시험을 따로 유지할 때만 정당화할 수 있다. 조용히 임계값을 늘려 M0 전체 통과로 간주하면 안 된다.
3. **제품 파라미터 변경은 지금 근거 부족이다.** WSL의 0.3초 scheduling stall은 현장 heartbeat 요구의 완화 근거가 아니다. 현장/목표 환경의 latency 분포(p95/p99/max), 누락·오탐률, 안전 반응시간 요구를 보고 결정해야 한다. 변경하면 records/param_changes.csv와 TEST_SPEC 기대값·명세를 함께 갱신해야 한다. 여기서는 실제 수정하거나 값을 결정하지 않았다.
4. **그대로 두고 현장만 기다리는 것도 권고하지 않는다.** P1 수정과 시험 discovery 안정화, 장비 없는 ROS nominal 반복·지연 측정은 지금 할 수 있다. 남은 간헐 실패는 미해결로 보고하고 안정 통과로 표시하지 않아야 한다. 현장 측정은 제품 숫자 확정용이며 WSL/CI 실패 원인 조사를 대체하지 않는다.

## 수행 검증과 한계

GCE 실행:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py
```

결과: **591 passed, 2 skipped in 1.79s**. 첫 호출 출력은 tool yield로 일부만 회수되어 최종 집계를 얻기 위해 한 번 재실행했으며 위 수치는 두 번째 완결 실행이다. ROS 관련 2개 파일은 importorskip으로 skip되므로 steady 타이머·ROS 포트 시험을 GCE에서 통과했다고 주장하지 않는다. 별도 AST/스레드 재현은 P1 시나리오를 확인했다.

Coordinator가 제공한 PC WSL 빌드·ROS 시험 결과와 지연 관측은 배경 자료로만 사용했고 독립 재실행하지 않았다. 이 검수는 변경 승인·M0 완료 판정이 아니며 위 발견 사항의 수정과 ROS 회귀 검증은 coordinator에 남아 있다.


## Coordinator 후속 수정 재검수

완료 직전 inbox에서 coordinator의 `_fault_lock` 적용 통지를 받아 현재 파일을 다시 읽었다. `raise_fault`의 검사·set·저장, `pending_fault`의 읽기·clear, reset, `_released` 설정, `_interrupting` 읽기가 동일 lock으로 보호되며 `_watch()`는 소비 lock 밖에서 호출되어 직접적인 재진입 교착을 피한다. 현재 변경은 보고한 P1 교차 실행을 막는 적절한 수정이다.

GCE에서 현재 메서드 3개를 AST 추출하여 `threading.Barrier(3)`로 soft/hard 두 raise 스레드를 동시에 출발시키는 시험을 200회 실행했다. 전부 `pending_fault() is CONTROLLER_HEARTBEAT_LOST`를 만족했다. 이는 ROS 없이 수행한 보조 검증이며 ROS executor 계약 시험을 대체하지 않는다.

권고하는 영구 동시성 회귀시험 설계:

- 테스트용 lock wrapper는 내부에 진짜 threading.Lock을 둔다. soft thread가 lock을 획득하면 Event를 알리고 임계구역 직전에서 대기시킨다. hard thread의 lock 획득 시도 Event를 확인한 뒤 soft를 풀고 두 thread를 join한다. 최종 pending은 반드시 hard다. soft가 hard 저장을 덮는 옛 구현은 계측 lock을 사용하지 않으므로 별도 join 상한 또는 동기화 미참여 assertion으로 실패하도록 설계한다.
- 반대 순서(hard가 먼저 lock 획득), 기존 soft latch에서 hard 승격, pending 소비와 hard raise가 교차하는 순서를 별도 실행한다. 소비/raise 시험에서는 소비 결과와 남은 pending 중 정확히 한 곳에서 hard가 관측되는 것을 확인한다. 단순 `_fault is None` assertion은 올바른 소비도 유실로 잘못 분류하므로 피한다.
- 양쪽 thread의 예외를 main thread로 전달하고 모든 Event wait/join에 짧은 상한을 둔다. 고정 sleep으로 race를 우연히 유도하지 않는다. reset은 사이클 경계라는 전제를 지키고, reset 경합 시험이 필요하면 이전 사이클/새 사이클 이벤트 귀속 정책부터 명시한다.
- 실제 ROS 회귀에는 RELEASE 뒤 RETREAT 응답을 지연시킨 중간에 비하드 후 하드 장애를 주입하여, 대기 종료와 Stop·WAIT_OPERATOR·PROTECTIVE_STOP 결과를 함께 확인한다.
