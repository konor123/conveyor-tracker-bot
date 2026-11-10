# CI에서만 실패하는 시간 시험: 시험 시계가 executor를 굶긴 사례 (2026-11-10)

## 무엇이 실패했나

TC-MCK-005-01(`test_injected_events_appear_at_scheduled_time`)은 mock 세계 노드가 시나리오 일정대로 이벤트를 내는지 실시간으로 확인한다. 11월 9일부터 GitHub Actions에서 첫 시도가 자주 실패했고, 11월 10일에는 6번 시도가 모두 실패했다. 실패 메시지는 `(3.87, 4.35, [3.87, 8.87])`처럼 "프레임 시각 3.87 s인데 4.35 s에 받았다"는 뜻이었다. 수신 지연 상한 0.2 s(SPEC-700 REQ-MCK-005)를 넘은 것이다. officepc WSL에서는 한 번도 실패하지 않았다.

## 어떻게 찾았나

1. **시험이 남긴 추적을 읽는다.** 이 시험은 실패할 때 프레임 콜백이 깨어난 시각, 그때의 ROS 시각, 발행 시각, 수신 시각을 모두 출력한다. CI 로그에서 그 줄을 꺼내 Python으로 분석했다(`gh run view <id> --log | grep frame_wakes_ros_k_previous_matches_steady`).
2. **구간을 나눈다.** 발행→수신은 약 1 ms로 정상이었다. 문제는 그 앞이었다. 프레임 콜백이 10 s 동안 64번만 깨어났고(정상은 약 266번), 최대 2.3 s 동안 한 번도 깨어나지 않았다. 깨어난 순간의 ROS 시각은 실제 시각보다 최대 0.4 s 뒤처져 있었다. 그래서 노드는 오래된 시각의 프레임을 "최신"으로 알고 발행했다.
3. **부하원을 센다.** 시험은 노드 3개(world·robot·시험 client)를 한 Python 프로세스의 `MultiThreadedExecutor`에서 돌린다. `use_sim_time`을 켠 노드는 `/clock` 메시지마다 Python 콜백을 실행한다. 시험 시계(`SteadyRosClock`)는 `/clock`을 200 Hz로 냈으므로 초당 약 600번의 콜백이 생겼다. rclpy executor는 콜백마다 대기 집합을 다시 만드는 비용이 있어서 느린 CI CPU에서는 이 양만으로 밀린다. action 실행이 없는 1~5 s 구간에도 공백이 있었으므로 action 폴링만이 원인은 아니다.
4. **재현을 시도하고 한계를 기록한다.** officepc WSL에서 스레드 수를 2로 고정하고(`MultiThreadedExecutor` 기본 스레드 수는 `os.cpu_count()`, WSL은 12) `taskset -c 0,1`로 코어를 묶고, 같은 코어에 CPU 부하 프로세스까지 붙였다. 그래도 재현되지 않았다(최대 공백 0.09 s). 그래서 CI 흔적에 근거한 수정을 넣고 CI에서 판정했다.

## 무엇을 고쳤나

- 시험 시계 `/clock`을 200 Hz → 50 Hz로 낮췄다(`tools/testing/drop_ros.py`). 시각이 20 ms 단위로 나뉘지만 30 Hz 한 프레임(33 ms)보다 작아 프레임을 건너뛰지 않고, 0.2 s 상한과도 거리가 멀다. 이 시계를 쓰는 시험은 이것 하나뿐이다.
- 0.2 s 상한은 바꾸지 않았다. 상한을 넓히면 실패는 사라지지만, 시험이 확인하려던 "mock이 제때 프레임을 낸다"는 성질을 더 이상 보지 않게 된다.
- 다음에 또 실패하면 원인을 바로 볼 수 있게, 실패 출력에 CPU 수·executor 스레드 수·시계 주기를 추가했다.
- 결과: CI run 42768394758에서 TC-MCK-005-01이 3회 모두 통과했고, attempt 2·3은 전 단계 통과했다.

## 배운 점과 함정

- **"시험용 장치도 부하다."** 시각을 통제하려고 넣은 시계가 오히려 시험 대상의 시각을 늦췄다. 시험 기반 코드의 주기·스레드 수도 제품 코드처럼 근거를 두고 정한다.
- **빠른 개발 PC에서 통과는 증거가 약하다.** CPU 수에 따라 executor 스레드 수가 달라지고, 코어 속도도 다르다. CI 실패는 CI 로그의 원시 시각으로 분석한다.
- **재시도가 결함을 가린다.** 같은 CI 실행의 첫 시도에서 다른 I0 시험이 실패했다. 감시 노드가 heartbeat 끊김으로 Stop을 보냈는데, 오케스트레이터는 중단된 파지를 일반 파지 실패로 판정했다. 예전에는 재파지(L1)가 이 실패를 덮었다. L1을 지우자 드러났다([성공 경로 정리](2026-11-10-scope-trim-success-path.md)). 이 문제는 [Progress](../../Progress.md) 1절 노드 항목에 남아 있다.

관련 기록: [11월 10일 worklog](../../records/worklog/2026-11-10.md), [11월 9일 QoS·위상 시험](2026-11-09-camera-qos-and-event-phase-test.md).

## 이어서: 감시 노드의 Stop을 오케스트레이터가 알아차리게 하기

위 CI 실패에서 드러난 문제를 같은 날 고쳤다.

- **무엇이 빠져 있었나:** 안전 감시 노드(`safety_supervisor_node`)와 오케스트레이터는 heartbeat를 각자 감시한다. 감시 노드만 끊김을 보고 `Stop`을 보내면, 오케스트레이터에게 남는 단서는 "진행 중이던 skill이 끝났다"는 결과뿐이다. 그 결과를 일반 파지 실패로 읽어 L2로 끝냈다.
- **신호를 구분하기:** SPEC-010은 원래 `CANCELED`의 이유 코드를 `STOPPED`(Stop 선점)와 `CANCELED`(클라이언트 cancel)로 나눠 두었다. mock 로봇은 이를 지켰지만 실제 로봇 노드는 둘 다 `CANCELED`로 냈다. 계약 시험이 이유 코드를 확인하지 않아서 몰랐다. 이번에 계약에 이유 코드 확인을 넣었다(`tools/testing/contracts/robot_controller_contract.py`). "명세에 적혀 있다"와 "시험이 확인한다"는 다르다.
- **원인 순서:** 결과에서 `STOPPED`를 보면 바로 일반 이상(`ROBOT_STOPPED`)을 올리지 않고, 자체 감시(`_watch`)를 먼저 한 번 돌린다. 감시가 보호정지·heartbeat 손실을 이미 보고 있으면 그 구체적 원인이 기록에 남는다. 일반 이상이 먼저 latch됐더라도 뒤이어 구체 원인이 오면 바꾼다. 작업자가 기록을 보고 원인을 바로 알 수 있게 하려는 것이다.
- **rclpy 함정:** action 서버에서 `goal_handle.canceled()`는 클라이언트가 cancel을 요청한 goal에만 쓸 수 있다. Stop처럼 서버 쪽 이유로 끝내는 goal은 `abort()`로 끝내고, 결과 코드(`CANCELED`·`STOPPED`)로 이유를 알린다.
- **아직 남은 빈틈:** 실행 중인 skill이 없을 때(검출 서비스 호출 중, 단계 사이, WAIT_CAP 대기) 감시 노드가 보낸 Stop은 오케스트레이터에 전달되지 않는다. 로봇 쪽에서 "작업자가 풀 때까지 정지 중"을 알리는 신호가 필요하며 인터페이스 결정이 필요하다.

확인: `python3 -m pytest src/orchestrator/test/test_node_failure_regressions.py -k stopped`(GCE, ROS 없이), `python3 -m pytest src/robot_control/test/test_robot_controller_stop.py`(ROS 필요, PC).
