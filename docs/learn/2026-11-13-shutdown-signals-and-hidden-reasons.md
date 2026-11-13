# 종료 신호의 traceback, 판정이 가린 실패 사유, 늦게 오는 노드 이름, 0.33초 멈추는 발행

2026-11-13 오후에 원격으로 처리한 네 가지 문제를 "무엇을 했고, 왜 그랬는지, 어떻게 찾아냈는지" 중심으로 정리했다. 네 문제 모두 **로그에 보이는 것이 원인이 아니었다.** 사실 기록은 [11월 13일 worklog](../../records/worklog/2026-11-13.md)에 있다.

---

## 1) 노드 종료 때의 traceback

### 증상

실제 컨트롤러 I0 런치가, 노드 로직의 실패가 아니라 **launch가 끝나는 순간의 traceback 때문에** 실패하는 경우가 있었다. launch가 종료될 때 `safety_supervisor_node` 또는 `intercept_planner_node`가 이런 traceback을 출력했다:

```
RuntimeError: Unable to convert call argument '0' to Python object
```

(발생 위치는 rclpy의 `take_message` 내부.) I0 시험은 로그에 `Traceback`이라는 문자열이 없다는 것을 검증하기 때문에, 이 traceback이 보이면 시험이 실패했다.

### 왜 이런 일이 생기는가 — 배경 개념

**Python의 시그널과 KeyboardInterrupt.** 운영체제는 프로세스에 "종료해라" 같은 신호(signal)를 보낼 수 있고, `SIGINT`는 보통 사용자가 Ctrl+C로 중단시킬 때 오는 신호다. Python은 SIGINT를 받으면 기본 동작으로 메인 스레드의 "다음 바이트코드"에서 `KeyboardInterrupt` 예외를 발생시킨다. 중요한 점은 **언제 오는지 예측할 수 없다**는 것이다. 프로그램이 그 순간 무슨 일을 하고 있었는지에 따라 예외가 터지는 자리가 달라진다.

**rclpy의 SIGINT 핸들러 체인.** rclpy는 `rclpy.init()` 시점에 기존의 SIGINT 핸들러를 저장해 두고, 자기 핸들러를 설치한다. rclpy 핸들러는 (1) 이전 핸들러(Python의 것)를 호출하고, (2) 별도 스레드에 알려 ROS 컨텍스트를 shutdown하고 대기 중인 executor를 깨운다(rclpy jazzy `signal_handler.cpp`). 즉 SIGINT가 오면 컨텍스트가 shutdown되고, 메인 스레드에서 `KeyboardInterrupt`가 뜬다. 이후 노드 코드는 보통 `rclpy.spin()`이 예외로 빠져나오고 cleanup을 실행하는 흐름이다.

**pybind11의 예외 변환.** ROS 2의 rclpy 아래에는 C/C++로 된 라이브러리가 있고, 이 경계에서는 C 함수가 Python 예외를 만나면 `NULL`을 반환해야 한다. 문제는 **C 함수가 Python 객체를 다루는 도중(예: 받은 메시지를 Python 객체로 변환하는 도중)에 `KeyboardInterrupt`가 끼어드는 경우**다. 이때 pybind11의 `make_tuple`은 "이 인자를 Python 객체로 변환할 수 없다"는 의미의 `RuntimeError`를 만든다. 원래의 `KeyboardInterrupt`는 사라진다 — 확인해 보니 `__cause__`와 `__context__`가 모두 `None`이었다. 그래서 로그에는 "메시지 변환 실패"라는 엉뚱한 에러만 남는다.

**단일 스레드 vs 멀티 스레드 executor.** `KeyboardInterrupt`는 메인 스레드에서만 난다. 단일 스레드 spin(`rclpy.spin`)은 메시지 변환(`take_message`)도 메인 스레드에서 한다. traceback을 낸 두 노드(supervisor, planner)는 모두 `rclpy.spin`을 쓴다. 멀티 스레드 executor는 변환을 작업 스레드에서 하는 것으로 보여 이 경쟁에 덜 걸릴 것으로 추정하지만, 확인하지는 않았다.

### 어떻게 확인했는가

- 소스 코드 분석 + 재현으로 원인을 확정했다. 재현 트릭은 이렇다: 메시지 클래스의 `__init__`이, **호출한 프레임이 executor의 `_take_subscription`일 때만** `signal.raise_signal(SIGINT)`를 호출하도록 패치한다. 이렇게 하면 원래 아주 드문 race를 결정적으로 재현할 수 있다.
- 사무실 PC WSL(rclpy 7.1.12)에서 이 재현을 3번 돌려 3번 모두 `RuntimeError: Unable to convert call argument '0' to Python object`가 났고, 예외의 `__cause__`·`__context__`는 둘 다 `None`이었다. 원래의 `KeyboardInterrupt`를 예외에서 되찾을 방법이 없다는 뜻이다. 그래서 "신호를 받았는가"를 예외가 아니라 신호 처리기에 기록해야 한다.

### 두 번째 문제: cleanup 중에 오는 SIGINT

SIGINT가 두 번 오면(시험 하네스와 launch가 각각 보내는 경우), 두 번째 `KeyboardInterrupt`가 cleanup 중의 `destroy_node()` 안쪽에 떨어질 수 있다. 그러면 또 traceback이 찍히고, mock 프로세스는 종료 코드 -2로 끝났다.

### 해결: ShutdownGuard

`node_shutdown.py`에 작은 클래스 `ShutdownGuard`를 만들었다. 새 패키지 의존성을 만들지 않으려고 **같은 사본 5개**를 패키지들에 두었다(바이트 단위로 같아야 한다 — 시험이 확인한다). 설정 경로 규칙 `repo_path`도 같은 방식이다([SPEC-000](../../specs/SPEC-000-system.md) 4.4절).

핵심 설계와 이유:

- **`rclpy.init()`보다 먼저 생성한다.** rclpy는 init 시점에 이전 SIGINT 핸들러를 저장하므로, init 전에 설치된 우리 핸들러는 rclpy 핸들러 안에서 체인처럼 호출된다. 반대로 init **후에** Python 핸들러를 설치하면 rclpy 핸들러를 덮어쓰고, idle 상태의 executor는 wakeup되지 않아 종료 신호를 못 받는다.
- 첫 번째 SIGINT는 `KeyboardInterrupt`를 발생시키고, 이후 SIGINT는 **카운트만** 한다(그때그때 예외를 띄우지 않음).
- `is_shutdown(exc)`은 `KeyboardInterrupt`, `ExternalShutdownException`, 그리고 **SIGINT를 받은 뒤에 온 `RuntimeError`**에 대해 `True`를 반환한다. 그 외(예: `SystemExit`)는 다시 raise한다.

적용 노드: `spin_mock`(drop_mocks), `orchestrator_node`, `safety_supervisor_node`, `record_logger_node`, `tracker_node`, `intercept_planner_node`, `robot_controller_node`. **적용하지 않은** 노드: `static_pick_place`, `vision`, `calibration` — 다음 날 현장 코드는 그대로 두기로 했다.

시험: TC-SYS-007-01(진입점과 같은 모양의 탐침 프로세스 — 메시지 변환 중 신호, 정리 중 두 번째 SIGINT, 깨어날 일이 없는 대기 중 SIGINT·SIGTERM은 3초 안에 종료), TC-SYS-007-02(정적 AST 검사: guard가 `rclpy.init`보다 앞에 있는지, `KeyboardInterrupt`·`ExternalShutdownException`을 이름으로 잡는 except가 없는지, 사본 5개가 같은지). 근거: SPEC-000 v0.7의 REQ-SYS-007. 사무실 PC WSL에서 TC-SYS-007-01 5건이 통과했고, 정적 검사는 고치기 전 코드에서 실패·고친 뒤 통과했다(GCE).

---

### 더 남아 있던 것: traceback 없는 종료 코드 −2

고친 뒤 launch 로그를 모두 남겨(`pytest -s`) 다시 보니 traceback은 사라졌지만 노드 20개가 `process has died ... exit code -2`로 끝났다. −2는 "SIGINT 신호로 죽었다"는 뜻이다. 원인은 Python 자체에 있었다. CPython 3.12는 종료 단계(`Py_FinalizeEx` → `_PySignal_Fini`)에서 Python 처리기가 걸린 신호를 기본 처리(SIG_DFL)로 되돌린다. 노드가 정리를 마치고 끝나는 중에 launch가 늦게 보낸 SIGINT가 도착하면, 그때는 기본 처리라 프로세스가 그냥 죽는다. 그래서 정리를 마친 뒤 `guard.close()`로 SIGINT를 무시(SIG_IGN)하게 했다. Python은 무시는 되돌리지 않는다.

시험은 "정리를 마쳤다(`CLEAN`)는 줄을 본 뒤 프로세스가 끝날 때까지 0.5 ms마다 SIGINT를 보낸다"로 만들었다. `close()`를 빈 함수로 바꾸면 이 시험이 3번 모두 −2로 실패하고, 넣으면 통과한다. 실제 controller I0 두 묶음에서 −2는 0건이 됐다.

## 2) RELEASE STEP_TIMEOUT이 가린 것

### 증상

실제 컨트롤러를 쓴 66번의 launch 중 1번에서 로그에 이것만 남았다:

```
RELEASE STEP_TIMEOUT  []
```

걸린 시간도 없고, 컨트롤러가 실패한 이유도 없었다.

### 원인 — 같은 `None`, 두 가지 길

`orchestrator_node`의 `RosSkillPort._phase_done`은 포트가 `None`을 반환하면 STEP_TIMEOUT으로 판정한다. 그런데 `None`이 나오는 경로가 **두 가지** 있다:

- (a) 대기가 끝난 경우 — RELEASE의 ROS 데드라인 1.0초, steady 캡, 또는 fault.
- (b) **컨트롤러의 DynamicPlace 결과가 RELEASE의 phase_done 피드백보다 먼저 끝난 경우** — 예를 들어 컨트롤러가 gate 사유로 실패한 경우.

(b) 경로에서는 **컨트롤러의 사유가 버려졌다.** 로그가 `[]`뿐이었던 이유다. 즉 STEP_TIMEOUT이라는 "판정"이 진짜 실패 사유를 가리고 있었다.

### 설계상 겹치는 타이밍 — 왜 (b)가 나올 수 있는가

MODE_SYNCHRONIZED에서는 컨트롤러가 release gate를 닫고 대기한다(비단말 사유: `GATE_TRACK_STALE`, `GATE_BELT_STALE`, `GATE_POSITION_STD`, `GATE_E_XY`, `GATE_V_REL`, `GATE_PLAN_UNCERTAIN`). 이 대기는 설계상 최대 `release_wait_max_s` = 2.0초까지 허용된다(TC-ROB-009-02가 0.5초 pulse를 기다려 통과하는 것이 그 근거).

여기에 두 값이 겹친다:

- fake transport 기준 RELEASE 길이 = gate 대기 + 0.1초(commit → open) + 0.3초(open 확인 유지).
- orchestrator의 RELEASE 스텝 한도는 1.0초(제안값이며 G0에서 측정 예정).

따라서 gate 대기가 길면 controller는 OK로 끝날 상황인데 orchestrator가 먼저 타임아웃한다. 새 로그로 재 보니 실제 controller I0의 **정상 사이클이 이미 0.88~0.91초**였다. SYNCHRONIZED는 계획 위치 표준편차가 0.5 mm 아래로 내려오기를 약 0.49초 기다리고(`GATE_POSITION_STD`), PREDICTED는 예정 해제 시각까지 0.47초 기다린다. 여유가 약 0.1초뿐이라, 무엇이든 0.1초만 늦어져도 `STEP_TIMEOUT`이 된다. track 신선도 한도 tau가 0.3초라서, 같은 날 대역 세계와 실제 controller에서 본 0.3초 넘는 발행 정지가 tracker에도 생기면 gate가 그만큼 닫힐 수 있다. 이것은 그 1건의 **가능한 원인이지 확정된 원인은 아니다**. 참고로 이전에 본 mock-controller 타임아웃(seed 7, 18)에는 gate가 없으므로 이 설명으로는 커버되지 않는다.

### 무엇을 바꿨는가 (로그만, 판정 로직은 그대로)

- orchestrator: 경로, ROS/steady 경과 시간, 마지막으로 받은 피드백과 그 나이, latch된 fault를 한 번 warn한다(REQ-ORC-020, SPEC-100 v0.17). 실패 스텝 경고에는 스텝 길이도 포함.
- controller: goal 종료 시 DynamicPlace 요약 한 줄을 로그로 남긴다 — 결과, phase 진입 시각과 goal 시작 대비 commit 시각, RELEASE 진입→commit, 사유별 gate 닫힌 사이클 수(REQ-ROB-018, SPEC-500 v0.13).

한도는 바꾸지 않았고, 두 값의 관계는 SPEC-500 섹션 9에 open item으로 남겼다.

### 확인 시험

TC-ROB-018-01: RELEASE 진입 후 track을 0.7초간 stale로 유지한다 → 결과 OK, RELEASE 진입→commit 0.70초, `GATE_TRACK_STALE` 사이클 약 70회, RELEASE 길이 = 0.7 + 0.1 + 0.3 = 1.1초로 orchestrator 한도 1.0초보다 길다. 분석 전체는 [RELEASE STEP_TIMEOUT 분석](../../records/reviews/release-step-timeout-2026-11-13.md)에 있다.

새 로그는 이렇게 보인다(사무실 PC WSL 실제 controller I0, 해제 전 vision heartbeat를 끊은 시험):

```text
[orchestrator_node]: SYNCHRONIZE 끝 feedback 없음: 대기 끝(제한시간·steady 상한·장애); 대기 ROS 0.282 s·steady 0.282 s; 마지막 feedback SYNCHRONIZE 0.016 s 전; 장애 HEARTBEAT_LOST
[orchestrator_node]: SYNCHRONIZE HEARTBEAT_LOST  [] 단계 0.352 s
[robot_controller_node]: DynamicPlace CANCELED CANCELED; 단계 APPROACH@0.000 SYNCHRONIZE@0.980 DESCEND@1.320; RELEASE 진입→commit -; 게이트 대기 -
```

---

### 덤: 로그 한 줄이 시험을 깨뜨린 이유

새 WARN 로그를 넣자 실제 controller I0의 동기 유지 상승 속도 검사가 23번 중 5번 실패했다(기준선과 로그 하나씩 끈 경우는 30번 중 0번). 실패는 늘 첫 상승 쌍 하나(0.023~0.025 m/s)였고 controller 자신의 속도 판정은 통과했다. 시험은 `robot/status`의 발행 시각과 위치로 속도를 재는데, 가짜 transport는 DynamicPlace가 끝나고 SafeRetreat가 시작하기까지(약 25 ms) 위치를 갱신하지 않는다. 그 틈에 발행된 상태의 위치는 발행 시각보다 오래됐고, 약 20 ms 오래되면 0.020 × 1.2 = 0.024가 된다. 로그 한 줄이 그 틈을 조금 늘려 확률을 키운 것으로 본다. 허용값은 그대로 두고, 앞 표본도 이미 올라가고 있는 쌍만 세게 고쳤다(10번 모두 통과).

## 3) replay 시험의 `_NODE_NAME_UNKNOWN_`

### 증상

CI run 43366819117에서 `test_replay_launch.py`의 synthetic_bag이 `/objects/detections` 퍼블리셔가 `vision_node`인지 검사하다가 `_NODE_NAME_UNKNOWN_`을 받아 실패했다.

### 원인 — DDS discovery와 노드 이름은 별개로 온다

ROS 2에서 **DDS endpoint가 먼저 발견(discovery)되고, 노드 이름은 별도로(ROS graph 정보를 통해) 온다.** 둘 사이의 짧은 창에서는 그 퍼블리셔의 노드 이름이 `_NODE_NAME_UNKNOWN_`으로 보인다. 시험은 이미지 퍼블리셔의 이름은 기다리고 있었지만 detections 퍼블리셔의 이름은 기다리지 않고 있었다. 즉 원래부터 있는 race인데, 그날 처음으로 그 창에 걸린 것이다.

### 해결

detections 퍼블리셔 목록이 `['vision_node']`와 같아질 때까지 최대 10초 기다리고, 타임아웃 메시지에 그 목록을 넣었다. 다른 노드가 같은 토픽을 내면 목록이 끝까지 같아지지 않으므로 원래 확인하려던 것(검출을 내는 것은 실제 vision 하나)은 그대로 확인한다.

---

## 4) 0.33초 멈추는 발행 — 작게 재현하고 native 스택으로 확인하기

### 증상

대역 노드와 실제 controller가 기동 2.5~3.25초 뒤 heartbeat를 0.31~0.43초 늦게 냈다. 같은 날 앞선 진단(faulthandler)으로 rclpy `Publisher.publish` 안에서 멈췄다는 것까지는 알았다([정지 진단 학습 문서](2026-11-13-stall-diagnosis-and-bench-stub.md)). rclpy `publish`는 GIL을 놓지 않으므로 발행이 막히면 프로세스의 모든 Python 스레드가 멈춘다.

### 작게 재현하기

전체 launch 대신 발행자 하나(100 Hz, heartbeat와 같은 QoS)와 새 참여자들만 띄우는 두 스크립트로 조건을 하나씩 바꿨다. 새 참여자를 시간 간격을 두고 붙이면(spaced) 3번 모두 멀쩡했고, launch처럼 6개를 한꺼번에 붙이면(burst) 3번 중 2번 `publish()` 한 번이 0.33초 걸렸다. 참여자가 생긴 약 3.4초 뒤였고, 크기와 시점이 실제 관찰과 같다. 같은 burst에서 비동기 발행(`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`)이나 공유 메모리를 끈 UDP 전용(`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`)은 각각 3번 중 0번이었다.

여기서 멈추면 안 된다. 재현은 발견 범위를 비운 기본 설정에서 한 것인데, I0 시험과 CI는 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 쓴다. 그 설정으로 같은 burst를 5번 돌리자 0번이었다. 그래서 "I0·CI 정지와 같은 원인"이라고는 말할 수 없고, 정지 시간·시점·막힌 곳(rclpy `publish` 안)이 같다는 데까지만 말한다. **재현 조건이 실제 조건과 같은지 마지막에 다시 확인하자.**

### native 스택

py-spy를 **부모 프로세스로** 띄워(WSL의 `ptrace_scope` 1에서는 남의 프로세스에 붙을 수 없다) `--native --idle`로 기록했다. 재현된 2번 모두, 멈춘 동안의 표본 전부가 Fast DDS 동기 전송 경로 `RTPSWriter::send_nts`의 `pthread_mutex_lock`에 있었다. 동기 발행은 호출한 스레드가 직접 보내므로, 다른 스레드가 그 mutex를 쥐고 있으면 노드가 통째로 멈춘다. 누가 쥐고 있었는지는 py-spy가 Python 스레드만 기록해서 모른다.

### 진단이 진단 대상을 멈출 때

같은 날 앞서 넣은 정지 진단(faulthandler)을 켠 대역 프로세스 시험이 CI에서 두 번 연달아 실패했고, 강제 종료된 대역의 서비스가 그래프에 남아 뒤 시험까지 줄줄이 실패했다. 원인을 찾다가 진단 자체의 함정을 하나 확인했다. 덤프는 stderr에 쓰는데, 출력을 읽지 않는 pipe(64 KiB)가 차면 감시 스레드의 쓰기가 막힌다. 다음 heartbeat에서 감시를 다시 거는 `dump_traceback_later`는 GIL을 쥔 채 그 스레드가 끝나기를 기다리므로 프로세스 전체가 멈춘다. 사무실 PC에서 출력을 읽지 않고 정지를 반복하자 약 30번째(65 KB)에 멈췄고, 출력을 계속 읽으면 40번도 멀쩡했다. 시험은 출력을 읽는 스레드를 두고 대역을 별도 namespace에 띄우게 고쳤다. 다만 이 경로는 CI 실패를 설명하지 못하고(그 시험은 5초 동안 덤프가 몇 번뿐이다) CI 실패는 PC에서 재현되지 않아, 시험을 `realtime_pc`(PC 판정)로 옮기고 원인은 남겨 두었다.

### 바꾸지 않은 것

현장 전이라 DDS 설정과 감시 판정 한도(0.3초)는 그대로 두고 후보만 기록했다. 공유 메모리를 끄면 카메라 영상이 느려지고, 비동기 발행은 지연을 다시 재야 한다. 자세한 표는 [발행 정지 재현 기록](../../records/reviews/publish-stall-repro-2026-11-13.md)에 있다.

## 5) 흔한 함정

이번 세 건에서 배운 것을, ROS 2가 처음인 팀원도 겪을 수 있는 형태로 정리한다.

- **종료 신호는 언제든, 어디든 떨어질 수 있다.** SIGINT가 오면 `KeyboardInterrupt`는 메인 스레드의 "다음 바이트코드"에서 발생한다. 그 자리가 정상 코드일 수도, C 라이브러리 경계일 수도, cleanup(`destroy_node`)일 수도 있다. "종료할 때만 나는 traceback"도 실패다 — 시험 검증(`'Traceback' not in log`)에 그대로 걸린다.
- **하나의 예외가 다른 예외를 가릴 수 있다.** C 함수 안에서 Python 예외가 끼어들면, 원래 예외가 사라지고 `RuntimeError: Unable to convert call argument ...` 같은 다른 예외가 로그에 남는다. 원인 파악이 어려워지므로, 종료 시점의 예외는 "종료 때문인가"를 먼저 판별하는 로직(guard의 `is_shutdown`)을 거치자.
- **rclpy SIGINT 핸들러를 덮어쓰지 마라.** `rclpy.init()` **후에** `signal.signal`로 Python 핸들러를 설치하면 rclpy 핸들러가 대체되고, idle executor는 깨지 못해 종료 신호를 놓친다. 커스텀 핸들러는 init **전에** 설치해서 rclpy의 체인에 끼워 넣자.
- **except 절을 한 예외에만 맞추지 마라.** planner가 `KeyboardInterrupt`만 잡고 있어서, SIGTERM(`ExternalShutdownException`) 경로도 traceback을 찍을 수 있었다. 종료 관련 예외 집합(KeyboardInterrupt, ExternalShutdownException, SIGINT 후의 RuntimeError)을 함께 다뤄야 한다.
- **같은 판정 값이 여러 원인을 가릴 수 있다.** `_phase_done`의 `None`은 "시간 초과"와 "컨트롤러 실패(사유 포함)"를 구분하지 못했다. 로그가 `[]`이면 사실 아무것도 알 수 없다. 판정 로직을 바꾸지 않더라도 **경로를 구분해 로그로 남기는 것**만으로 디버깅이 크게 나아진다.
- **의도된 대기가 다른 쪽의 짧은 한도와 겹치면 flake가 된다.** gate 대기 최대 2.0초(설계)와 orchestrator 스텝 한도 1.0초(제안값)처럼, 관련 값을 나란히 놓고 "이 조합이면 누가 먼저 끝나는가"를 계산해 보자. 그리고 그 관계는 open item으로 기록해 두자.
- **discovery 순서를 기다리는 시험을 작성하라.** DDS endpoint 발견과 노드 이름 확인 사이에는 `_NODE_NAME_UNKNOWN_` 창이 존재한다. 퍼블리셔의 노드 이름을 단정하는 시험은 반드시 그 목록이 기대값과 같아질 때까지 기다려야 한다.
- **큰 시스템의 드문 정지는 작은 재현으로 옮겨라.** 발행자 하나와 새 참여자만으로 0.33초 정지를 다시 만들자 설정(동기·비동기, 공유 메모리 유무)을 하나씩 바꿔 비교할 수 있었다. 이때 대조군(정지가 없어야 할 조건)을 함께 돌려야 "재현했다"는 말에 힘이 생긴다. 표본이 작으면(2/3 대 0/3) 그 크기를 그대로 적는다.
- **py-spy는 부모로 띄워라.** `ptrace_scope`가 1이면 이미 도는 프로세스에 붙지 못한다. `py-spy record -- python3 ...`처럼 자식으로 띄우고, 막혀 있는 스레드도 보려면 `--idle`, C 함수까지 보려면 `--native`를 준다.
- **아주 드문 race는 재현을 만들어 결정화하라.** 원래 드문 race를 소스 분석만으로 확신하기 어려울 때, "특정 프레임에서만 시그널을 raise" 같은 패치로 의도적으로 결정적으로 재현하면 원인 확정과 회귀 시험을 한 번에 만들 수 있다.
- **진단 도구도 대상 프로세스를 멈출 수 있다.** 덤프를 쓰는 출력이 막히면 진단이 프로세스를 세운다. 진단은 출력을 계속 읽는 곳(launch)에서만 켠다.
- **traceback이 없다고 깨끗하게 끝난 것은 아니다.** launch 로그의 `exit code`도 본다. −2는 SIGINT로 죽었다는 뜻이다.
- **시각과 값을 따로 읽는 측정은 그 사이가 벌어지는 순간에 틀린다.** 발행 시각(stamp)과 위치를 다른 때 읽으면, 위치를 갱신하지 않는 틈(넘겨주기 구간)에 발행된 표본이 속도를 틀리게 만든다. 판정 쪽(controller)이 쓰는 표본과 시험이 쓰는 표본이 다르면 같은 현상을 다르게 본다.
- **복사본 5개 전략을 쓸 때는 동일성을 시험으로 잠가라.** 패키지 의존성을 만들지 않기 위해 파일을 복제했다면, 그 복사본들이 바이트 단위로 같은지 static 검사(TC-SYS-007-02)로 강제하자.

---

## 6) 관련 파일


- 스펙
  - [../../specs/SPEC-000-system.md](../../specs/SPEC-000-system.md) — REQ-SYS-007 (ShutdownGuard, v0.7)
  - [../../specs/SPEC-100-orchestrator.md](../../specs/SPEC-100-orchestrator.md) — REQ-ORC-020 (phase 미스 로그, v0.17)
  - [../../specs/SPEC-500-robot-control.md](../../specs/SPEC-500-robot-control.md) — REQ-ROB-018 (DynamicPlace 요약 로그, v0.13; 섹션 9에 open item)
- 구현과 시험
  - [../../src/drop_mocks/drop_mocks/node_shutdown.py](../../src/drop_mocks/drop_mocks/node_shutdown.py) — ShutdownGuard
  - [../../tools/spec/test_node_shutdown.py](../../tools/spec/test_node_shutdown.py) — TC-SYS-007-02 (static AST 검사)
  - [../../src/drop_mocks/test/test_node_shutdown.py](../../src/drop_mocks/test/test_node_shutdown.py) — TC-SYS-007-01 (probe 프로세스 시험)
  - [../../src/orchestrator/test/test_phase_miss_log.py](../../src/orchestrator/test/test_phase_miss_log.py) — RELEASE 스텝 경고/피드백 나이 로그
  - [../../src/robot_control/test/test_dynamic_place_summary.py](../../src/robot_control/test/test_dynamic_place_summary.py) — TC-ROB-018-01 (track stale 0.7초 유지)
  - [../../src/drop_bringup/test/test_replay_launch.py](../../src/drop_bringup/test/test_replay_launch.py) — detections 퍼블리셔 이름 대기 수정
- 기록
  - [../../records/reviews/heartbeat-stall-evidence-2026-11-13.md](../../records/reviews/heartbeat-stall-evidence-2026-11-13.md) — 발행 정지 관찰 근거, RELEASE STEP_TIMEOUT 한 건의 로그
  - [../../records/reviews/release-step-timeout-2026-11-13.md](../../records/reviews/release-step-timeout-2026-11-13.md) — RELEASE STEP_TIMEOUT 분석
  - [../../records/reviews/publish-stall-repro-2026-11-13.md](../../records/reviews/publish-stall-repro-2026-11-13.md) — 발행 정지 미세 재현·native 스택(스크립트는 `records/reviews/evidence/publish-stall-2026-11-13/`)
  - [../../records/worklog/2026-11-13.md](../../records/worklog/2026-11-13.md) — 당일 작업 기록
- 용어
  - [glossary.md](glossary.md) — 용어 정리
