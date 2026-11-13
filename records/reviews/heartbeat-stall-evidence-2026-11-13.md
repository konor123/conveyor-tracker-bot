# heartbeat 경로 정지 진단 증거 (2026-11-13)

대역 프로세스의 heartbeat 경로 정지 진단(SPEC-700 4.8절, REQ-MCK-012)을 확인하고, 그 진단으로 잡은 자연 발생 정지 1건과 같은 날 관찰한 관련 실패를 남긴다. 로그는 원본에서 잘라냈고 경로 앞부분은 `<ws>`(colcon 작업 공간), `<ros>`(ROS Jazzy Python 패키지), `<py>`(Python 표준 라이브러리), `<check>`(저장소 밖 확인용 노드), `<ci-ws>`(CI 작업 공간)로 바꿨다. 실행 장소는 표마다 적었다.

## 1. CI 실패의 모양(`eb5c8ec`, run 43369190540, GitHub Actions 공유 러너)

사이클 전 기동 안정 대기 중 감시 노드가 대역 controller heartbeat 손실로 Stop을 냈다. 재수신 로그의 "발행 뒤 수신까지 0.001 s"는 막 발행된 heartbeat를 받았다는 뜻이다. 즉 전달이 아니라 발행이 늦었다.

```text
2026-11-13T05:15:32.1454626Z I0 기동 과도 감시 노드 Stop 1회 → 해제
2026-11-13T05:15:32.1456997Z [INFO] [mock_robot_controller_node-2]: process started with pid [8479]
2026-11-13T05:15:32.1463469Z [mock_robot_controller_node-2] [INFO] [1794546748.012187547] [mock_robot_controller_node]: scenario=nominal_20mms seed=0 t0_s=1794546747.418 file=<ci-ws>/install/drop_mocks/share/drop_mocks/scenarios/nominal_20mms.yaml
2026-11-13T05:15:32.1465610Z [mock_robot_controller_node-2] [INFO] [1794546748.013366470] [mock_robot_controller_node]: STOP_RELEASED reason=
2026-11-13T05:15:32.1467577Z [safety_supervisor_node-6] [WARN] [1794546751.192442876] [safety_supervisor_node]: CONTROLLER_HEARTBEAT_LOST: 마지막 수신 0.337 s 전, 받은 개수 18, 마지막 seq 18, 직전 감시 주기 간격 0.050 s, 기동 뒤 3.150 s
2026-11-13T05:15:32.1469378Z [mock_robot_controller_node-2] [INFO] [1794546751.270653304] [mock_robot_controller_node]: STOP_CALLED requester=1 reason=CONTROLLER_HEARTBEAT_LOST
2026-11-13T05:15:32.1471612Z [safety_supervisor_node-6] [WARN] [1794546751.271170818] [safety_supervisor_node]: controller heartbeat 재수신: 손실 판정 0.091 s 뒤, seq 18→19 (빠진 seq 0), 발행 뒤 수신까지 0.001 s
2026-11-13T05:15:32.1473459Z [mock_robot_controller_node-2] [INFO] [1794546752.899527227] [mock_robot_controller_node]: STOP_RELEASED reason=i0: bring-up supervisor heartbeat transient
2026-11-13T05:15:32.1480514Z [INFO] [mock_robot_controller_node-2]: sending signal 'SIGINT' to process[mock_robot_controller_node-2]
```

## 2. 진단 동작 확인(사무실 PC WSL, 일부러 멈춤, 기준 0.25 s, `ros2 launch`로 띄운 확인용 노드)

확인용 노드는 `MockRobotControllerNode`를 상속해 기동 2 s 뒤 한 번 멈춘다. launch 출력에 faulthandler 스택과 공백 경고가 나오는지 본다.

```text
# mode group (기본 group 콜백 안 0.6 s sleep)
[mock_robot_controller_node-1] [INFO] [1794548220.881105965] [mock_robot_controller_node]: DROP_MOCK_STALL_DUMP_S=0.250: heartbeat 경로 정지 진단 켬
[mock_robot_controller_node-1] [WARN] [1794548222.882598968] [mock_robot_controller_node]: INJECT_STALL mode=group
[mock_robot_controller_node-1] Timeout (0:00:00.250000)!
[mock_robot_controller_node-1] Thread 0x000076129a7ba6c0 (most recent call first):
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 29 in injected_sleep_in_default_group
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 24 in _inject
[mock_robot_controller_node-1] Thread 0x000076129afbb6c0 (most recent call first):
[mock_robot_controller_node-1] Thread 0x000076129bfbd6c0 (most recent call first):
[mock_robot_controller_node-1] Thread 0x00007612bc0d1740 (most recent call first):
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 37 in <module>
[mock_robot_controller_node-1] [WARN] [1794548223.484963062] [mock_robot_controller_node]: STALL heartbeat 경로 공백 0.602 s(기준 0.250 s): 그동안 프로세스 CPU 0.006 s, 자발 문맥 전환 32, 비자발 0. 스택은 바로 앞 faulthandler "Timeout" 출력

# mode gil (별도 group, GIL을 놓지 않는 C 호출 한 번)
[mock_robot_controller_node-1] [INFO] [1794548338.937707221] [mock_robot_controller_node]: DROP_MOCK_STALL_DUMP_S=0.250: heartbeat 경로 정지 진단 켬
[mock_robot_controller_node-1] [WARN] [1794548340.939147800] [mock_robot_controller_node]: INJECT_STALL mode=gil
[mock_robot_controller_node-1] Timeout (0:00:00.250000)!
[mock_robot_controller_node-1] Thread 0x000076937a7fc6c0 (most recent call first):
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 35 in injected_gil_hold
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 26 in _inject
[mock_robot_controller_node-1] Thread 0x000076937affd6c0 (most recent call first):
[mock_robot_controller_node-1] Thread 0x000076937bfff6c0 (most recent call first):
[mock_robot_controller_node-1] Thread 0x000076939c7ce740 (most recent call first):
[mock_robot_controller_node-1]   File "<check>/stall_node.py", line 39 in <module>
[mock_robot_controller_node-1] [WARN] [1794548341.658851880] [mock_robot_controller_node]: STALL heartbeat 경로 공백 0.719 s(기준 0.250 s): 그동안 프로세스 CPU 0.722 s, 자발 문맥 전환 143, 비자발 4. 스택은 바로 앞 faulthandler "Timeout" 출력
[mock_robot_controller_node-1] GIL_HOLD 0.708 s
```

판독: 같은 group 콜백의 sleep은 CPU 증가가 작고(0.006 s) 스택에 그 함수가 보인다. GIL을 놓지 않는 C 호출은 CPU 증가가 공백과 같다(0.722 s / 0.719 s). 짧은 `sum()`을 반복한 첫 확인은 호출 사이에 GIL이 풀려 정지가 생기지 않았다.

## 3. 자연 발생 정지 3건(사무실 PC WSL, 진단 켠 실제 controller I0, 세 시험 모두 통과)

| 묶음(5절) | launch(시험) | 대역 프로세스 기동 뒤 | 공백 | 그동안 CPU | 멈춘 스레드 |
|---|---|---|---|---|---|
| 3 | `test_i0_real_controller_static_cycle` | 약 2.7 s | 0.330 s | 0.009 s | heartbeat 타이머 → `Publisher.publish` |
| 10 | `test_i0_real_controller_dynamic_cycle` | 약 2.5 s | 0.313 s | 0.008 s | heartbeat 타이머 → `Publisher.publish` |
| 10 | `test_i0_real_controller_passes_exhausted_place_back` | 약 3.2 s | 0.379 s | 0.011 s | 프레임 타이머 → `Publisher.publish`(데이터 발행) |

셋 다 `mock_world_node`에서 기동 2.5~3.2 s 뒤, 다른 노드들이 올라오는 구간이다. 아래는 첫 건의 전체 출력이다.

```text
[INFO] [launch]: All log files can be found below <ros-log>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mock_world_node-1]: process started with pid [31021]
[INFO] [tracker_node-2]: process started with pid [31022]
[INFO] [intercept_planner_node-3]: process started with pid [31023]
[INFO] [robot_controller_node-4]: process started with pid [31024]
[INFO] [orchestrator_node-5]: process started with pid [31025]
[INFO] [safety_supervisor_node-6]: process started with pid [31026]
[INFO] [record_logger_node-7]: process started with pid [31027]
[mock_world_node-1] [INFO] [1794550139.970473107] [mock_world_node]: scenario=nominal_20mms seed=0 t0_s=1794550139.228 file=<ws>/install/drop_mocks/share/drop_mocks/scenarios/nominal_20mms.yaml
[mock_world_node-1] [INFO] [1794550141.013454856] [mock_world_node]: DROP_MOCK_STALL_DUMP_S=0.250: heartbeat 경로 정지 진단 켬
[mock_world_node-1] Timeout (0:00:00.250000)!
[mock_world_node-1] Thread 0x000078539cdfe6c0 (most recent call first):
[mock_world_node-1]   File "<ros>/rclpy/publisher.py", line 70 in publish
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/mock_world_node.py", line 126 in _on_heartbeat
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 115 in await_or_execute
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 454 in _execute
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 592 in handler
[mock_world_node-1]   File "<ros>/rclpy/task.py", line 286 in _execute_coroutine_step
[mock_world_node-1]   File "<ros>/rclpy/task.py", line 270 in __call__
[mock_world_node-1]   File "<py>/concurrent/futures/thread.py", line 58 in run
[mock_world_node-1]   File "<py>/concurrent/futures/thread.py", line 92 in _worker
[mock_world_node-1]   File "<py>/threading.py", line 1010 in run
[mock_world_node-1]   File "<py>/threading.py", line 1073 in _bootstrap_inner
[mock_world_node-1]   File "<py>/threading.py", line 1030 in _bootstrap
[mock_world_node-1] 
[mock_world_node-1] Thread 0x000078539d5ff6c0 (most recent call first):
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/scenario_engine.py", line 293 in _rng
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/scenario_engine.py", line 397 in frame
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/mock_world_node.py", line 83 in _on_frame
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 115 in await_or_execute
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 454 in _execute
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 592 in handler
[mock_world_node-1]   File "<ros>/rclpy/task.py", line 286 in _execute_coroutine_step
[mock_world_node-1]   File "<ros>/rclpy/task.py", line 270 in __call__
[mock_world_node-1]   File "<py>/concurrent/futures/thread.py", line 58 in run
[mock_world_node-1]   File "<py>/concurrent/futures/thread.py", line 92 in _worker
[mock_world_node-1]   File "<py>/threading.py", line 1010 in run
[mock_world_node-1]   File "<py>/threading.py", line 1073 in _bootstrap_inner
[mock_world_node-1]   File "<py>/threading.py", line 1030 in _bootstrap
[mock_world_node-1] 
[mock_world_node-1] Thread 0x00007853b181f740 (most recent call first):
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 781 in _wait_for_ready_callbacks
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 877 in wait_for_ready_callbacks
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 991 in _spin_once_impl
[mock_world_node-1]   File "<ros>/rclpy/executors.py", line 1015 in spin_once
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/ros_common.py", line 260 in spin_mock
[mock_world_node-1]   File "<ws>/build/drop_mocks/drop_mocks/mock_world_node.py", line 208 in main
[mock_world_node-1]   File "<ws>/install/drop_mocks/lib/drop_mocks/mock_world_node", line 33 in <module>
[mock_world_node-1] [WARN] [1794550142.948643812] [mock_world_node]: STALL heartbeat 경로 공백 0.330 s(기준 0.250 s): 그동안 프로세스 CPU 0.009 s, 자발 문맥 전환 205, 비자발 6. 스택은 바로 앞 faulthandler "Timeout" 출력
[orchestrator_node-5] [INFO] [1794550146.701329983] [orchestrator_node]: 20261113-150905-0001: UNKNOWN  (IDLE → PRECHECK → DETECT_STONE → PICK_STONE → MOVE_STANDBY → WAIT_CAP → APPROACH → DESCEND → RELEASE → RETREAT → VERIFY → DONE)
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[INFO] [record_logger_node-7]: sending signal 'SIGINT' to process[record_logger_node-7]
[INFO] [safety_supervisor_node-6]: process has finished cleanly [pid 31026]
[INFO] [orchestrator_node-5]: sending signal 'SIGINT' to process[orchestrator_node-5]
[INFO] [robot_controller_node-4]: sending signal 'SIGINT' to process[robot_controller_node-4]
[INFO] [intercept_planner_node-3]: sending signal 'SIGINT' to process[intercept_planner_node-3]
[INFO] [tracker_node-2]: sending signal 'SIGINT' to process[tracker_node-2]
[INFO] [mock_world_node-1]: process has finished cleanly [pid 31021]
child process pid 31022 exit status already read:  will report returncode 255
```

- 첫 건의 0.25 s 시점: heartbeat 스레드는 `_on_heartbeat` → rclpy `Publisher.publish`(C 호출) 안, 프레임 스레드는 엔진 Python 코드 중간, 주 스레드는 대기.
- `mock_world_node`의 heartbeat 타이머는 자기 callback group에 있다. callback group 경합이 아니라 발행 호출이 멈췄다. 셋째 건은 heartbeat가 아니라 프레임 데이터 발행에서 멈췄고(아래), 그동안 heartbeat 타이머도 돌지 못했다. 막힌 것은 특정 토픽이 아니라 발행 호출이다.

```text
[mock_world_node-1] [INFO] [1794551197.129178830] [mock_world_node]: DROP_MOCK_STALL_DUMP_S=0.250: heartbeat 경로 정지 진단 켬
[mock_world_node-1] Timeout (0:00:00.250000)!
[mock_world_node-1] Thread 0x00007dcc7bfff6c0 (most recent call first):
[mock_world_node-1]   File "<py>/threading.py", line 1073 in _bootstrap_inner
[mock_world_node-1] [WARN] [1794551199.712693444] [mock_world_node]: STALL heartbeat 경로 공백 0.379 s(기준 0.250 s): 그동안 프로세스 CPU 0.011 s, 자발 문맥 전환 156, 비자발 2. 스택은 바로 앞 faulthandl
```

- rclpy jazzy(`ros2/rclpy` 브랜치 `jazzy`, 커밋 `521012e60b`)의 `rclpy/src/rclpy/publisher.cpp`: `Publisher::publish`는 메시지를 변환한 뒤 `rcl_publish`를 부르고(130행), 바인딩 `.def("publish", &Publisher::publish, ...)`(182행)에는 GIL 해제(`py::gil_scoped_release`·`call_guard`)가 없다. 발행이 막히면 프로세스의 모든 Python 스레드가 멈춘다.
- 추정(미확인): 멈춘 곳이 rmw·DDS 안이라는 것(Python 스택만 있음), 기동 구간의 discovery(새 참여자 매칭)와 관련 있다는 것, CI의 대역 controller 공백(기동 3.0~3.25 s 뒤 0.33~0.43 s)이 같은 원인이라는 것.

## 4. 같은 날의 관련 관찰(사무실 PC WSL)

실제 controller I0 한 launch(`test_i0_real_controller_dynamic_cycle[synchronized]`, 진단 켬)에서 실제 `robot_controller_node`의 기동 직후 heartbeat 공백과 그 뒤 RELEASE `STEP_TIMEOUT`. 대역 세계 프로세스에는 공백 경고가 없었다.

```text
[INFO] [launch]: All log files can be found below <ros-log>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mock_world_node-1]: process started with pid [30385]
[INFO] [tracker_node-2]: process started with pid [30386]
[INFO] [intercept_planner_node-3]: process started with pid [30387]
[INFO] [robot_controller_node-4]: process started with pid [30388]
[INFO] [orchestrator_node-5]: process started with pid [30389]
[INFO] [safety_supervisor_node-6]: process started with pid [30390]
[INFO] [record_logger_node-7]: process started with pid [30391]
[mock_world_node-1] [INFO] [1794550102.591959091] [mock_world_node]: scenario=nominal_20mms seed=0 t0_s=1794550101.915 file=<ws>/install/drop_mocks/share/drop_mocks/scenarios/nominal_20mms.yaml
[safety_supervisor_node-6] [WARN] [1794550103.475245962] [safety_supervisor_node]: CONTROLLER_HEARTBEAT_LOST: 마지막 수신 0.336 s 전, 받은 개수 2, 마지막 seq 2, 직전 감시 주기 간격 0.050 s, 기동 뒤 0.800 s
[safety_supervisor_node-6] [WARN] [1794550103.539470425] [safety_supervisor_node]: controller heartbeat 재수신: 손실 판정 0.080 s 뒤, seq 2→3 (빠진 seq 0), 발행 뒤 수신까지 0.000 s
[robot_controller_node-4] [INFO] [1794550103.557492103] [robot_controller_node]: STOP_LATCHED requester=1 reason=CONTROLLER_HEARTBEAT_LOST
[mock_world_node-1] [INFO] [1794550103.632535486] [mock_world_node]: DROP_MOCK_STALL_DUMP_S=0.250: heartbeat 경로 정지 진단 켬
[robot_controller_node-4] [INFO] [1794550107.487450551] [robot_controller_node]: 정지 유지 해제: i0: bring-up supervisor heartbeat transient
[orchestrator_node-5] [WARN] [1794550117.637243807] [orchestrator_node]: RELEASE STEP_TIMEOUT  []
[orchestrator_node-5] [INFO] [1794550119.092239256] [orchestrator_node]: 20261113-150827-0001: SAFE_ABORT STEP_TIMEOUT (IDLE → PRECHECK → DETECT_STONE → PICK_STONE → MOVE_STANDBY → WAIT_CAP → PLAN_INTERCEPT → APPROACH → SYNCHRONIZE → DESCEND → RELEASE → SAFE_ABORT → WAIT_OPERATOR)
```

실제 controller I0 66개 launch(진단 켬·끔 합계)에서 기동 직후 감시 노드 Stop은 3개, `STEP_TIMEOUT`은 위 1개이며 둘이 겹친 것은 이 1개다. 기동 Stop·해제가 `STEP_TIMEOUT`을 부른다고 볼 근거는 없다.

## 5. 진단을 켠 실행과 끈 실행(사무실 PC WSL, `test_i0_real_controller.py` 8건 묶음)

| 묶음 | 진단 | 결과 | 실패 내용 |
|---|---|---|---|
| 1 | 켬 | 3 failed, 5 passed | 셋 다 launch 종료 때 실제 노드(`safety_supervisor_node` 2, `intercept_planner_node` 1)의 `RuntimeError: Unable to convert call argument '0' to Python object`(`take_message`) traceback → `'Traceback' not in log` 실패 |
| 2 | 끔 | 8 passed | |
| 3 | 켬 | 1 failed, 7 passed | 4절 RELEASE `STEP_TIMEOUT`. 같은 묶음의 다른 launch(정적 사이클)에 3절 첫 정지 |
| 4 | 끔 | 8 passed | |
| 5 | 켬 | 8 passed | |
| 6 | 켬 | 1 failed, 7 passed | 종료 때 같은 `take_message` traceback |
| 7 | 끔 | 8 passed | |
| 8 | 켬 | 8 passed | |
| 9 | 끔 | 8 passed | |
| 10 | 켬 | 8 passed | 3절 둘째·셋째 정지(두 launch) |
| 11 | 끔 | 8 passed | |

진단은 대역 프로세스에만 작용하고 traceback을 낸 실제 노드는 이 값을 읽지 않는다. 연결 방식을 찾지 못했지만 켠 묶음 6개 중 3개, 끈 묶음 5개 중 0개라 CI 워크플로 전체에 켜지 않고, 대역만 띄우는 ROS I0 시험(`test_i0_integration.py`)의 launch에서만 켠다. 그 시험은 진단을 켜고 18 passed ×2(12코어, 2코어 제한)였고 공백 경고는 0건이었다. `beat()` 1회 비용은 약 100 µs(GCE, 10 Hz면 CPU 약 0.1%)다.

## 6. 바꾸지 않은 것

감시 노드의 손실 판정 0.3 s, heartbeat 타이머의 callback group, DDS 발행 설정(동기·비동기), 실제 노드. 다음 확인 후보: native 스택(멈춘 곳이 rmw·DDS 안인지), 비동기 발행 모드와의 비교, 실제 노드 종료 때 `take_message` traceback의 원인.
