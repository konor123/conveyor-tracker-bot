# 멈춘 순간을 붙잡는 진단과 대역으로 측정 스크립트 시험하기 (2026-11-13)

ROS 2·Python을 막 배우는 팀원을 위한 설명 문서다. 실행 결과와 커밋은 [11월 13일 worklog](../../records/worklog/2026-11-13.md)에 있다.

## 1. 무엇을 했나

이번 묶음의 작업은 세 가지다.

1. **-inf 버그 수정**: mock 시나리오 엔진에서 정지 벨트 + 병뚜껑이 관측 구간 안에 있으면 `OverflowError`가 터졌다. 진입 시각을 0.0으로 두어 고쳤다.
2. **heartbeat 발행 지연 진단 추가**: mock controller의 heartbeat가 약 0.43 s 늦게 발행되던 문제를 가리는 진단 도구(`stall_watch`)를 넣었고, 사무실 PC에서 자연 발생 정지 한 건의 스택을 잡았다(발행 호출 안에서 멈춤).
3. **측정 스크립트 대역 시험**: `stream_bench.py`를 실기 없이 가짜 드라이버로 끝까지 돌려 보았다. 이 과정에서 빈 배열 `np.percentile` 결함을 찾아 고쳤다.

## 2. -inf가 정수 변환에서 터진 버그

### 무슨 일이었나

`src/drop_mocks/drop_mocks/scenario_engine.py`의 `time_at_s(s_target)`는 벨트 이동량이 `s_target`에 닿는 시각을 돌려준다.

초기 벨트 속도가 0이고 `s_target < 0`이면 `-inf`를 돌려준다. 시각 0 이전에는 어느 시각에도 닿지 않기 때문이다.

병뚜껑이 처음부터 관측 구간 안에 있으면 `frame()`에서 진입 시각 `t_enter`가 `-inf`가 된다. 그러면

```python
observation_count = math.floor((t - t_enter) * rate) + 1
```

에서 `math.floor(inf)`가 되고, 여기서 다음 오류가 난다.

```
OverflowError: cannot convert float infinity to integer
```

### 왜 이전에 재현이 안 됐나

이전 세션은 병뚜껑이 처음에 관측 구간 밖(`x0 = -0.05 m`)인 시나리오로 시험했다. 그래서 재현하지 못했다.

병뚜껑을 `x0 = 0.10 m`(관측 구간 0~0.45 m 안)에 두자 바로 재현됐다.

### 고친 방법

`t_enter`가 유한하지 않으면 `0.0`(시나리오 시작)으로 둔다. 시나리오 시각 0부터 관측했다고 보는 것이다.

다음은 그대로 둔다. 기존 시험이 의존하기 때문이다.

- `time_at_s`의 `-inf` 반환
- 움직이는 벨트의 음수 진입 시각

동작은 원래대로다. 정지 벨트에서 SelectTarget은 `FAILED BELT_STOPPED`, ComputeIntercept는 `TIME_INSUFFICIENT`다.

### 재현·확인 방법

시험 두 개로 확인한다.

- `TC-MCK-011-01`: 엔진 단위 시험(레벨 U)
- `TC-MCK-011-02`: 노드 시험(레벨 C). 시험 자료는 `src/drop_mocks/test/data/stopped_belt.yaml`이다.

재현 포인트는 병뚜껑 초기 위치를 관측 구간 안(예: `x0 = 0.10 m`)에 두는 것이다.

### 배운 것

- 재현이 안 될 때는 입력 조건을 바꿔 본다. 여기서는 병뚜껑 초기 위치였다.
- `inf`·`nan`이 `int()`, `math.floor()`, ROS Time 변환으로 흘러가는 경로를 의심한다.

## 3. heartbeat가 0.4 s 늦게 나간 문제를 가리는 진단

### 문제 상황

GitHub Actions 공유 러너에서 I0 통합 시험이 가끔 실패했다.

감시 노드(safety_supervisor)가 mock controller의 heartbeat를 0.3 s 넘게 못 받아 `CONTROLLER_HEARTBEAT_LOST`로 Stop을 냈다.

### 전달이 아니라 발행이 늦었다는 근거

앞 세션에서 넣은 진단 로그로 "발행 뒤 수신까지 0.001 s"임을 봤다. 그러면 전달이 아니라 발행이 약 0.43 s 늦었던 것이다.

그 순간은 사이클 전 기동 안정 구간이라 skill이 돌지 않았다. heartbeat를 보내는 경로는 타이머뿐이었다.

처음 의심한 것은 callback group이었다. mock controller의 heartbeat 타이머는 노드 기본 callback group(상호 배제)에 파라미터 서비스·상태 타이머와 함께 있다. 같은 group의 다른 콜백이 오래 돌면 타이머가 기다린다. 하지만 이것은 추측이었고, 추측만으로 group을 옮기면 원인을 모른 채 증상만 가릴 수 있다. 그래서 먼저 멈춘 순간을 보는 진단을 넣었다.

### 그래서 넣은 진단: faulthandler 기반 멈춤 덤프

환경 변수 `DROP_MOCK_STALL_DUMP_S`(예: `0.25`)를 주면 mock 노드 프로세스가 heartbeat 콜백에 들어올 때마다 `faulthandler.dump_traceback_later(0.25)`를 다시 건다.

동작 원리:

- 0.25 s 안에 다음 콜백 진입이 없으면 faulthandler의 C 감시 스레드가 모든 Python 스레드 스택을 stderr에 쓴다.
- 이 감시 스레드는 GIL 없이 돈다. 그래서 다른 스레드가 GIL을 쥔 채 C 호출에서 막혀 있어도 그 순간의 스택이 남는다.
- 재개되면 `STALL heartbeat 경로 공백 X s` 경고를 남긴다. 여기에 공백 시간 동안의 프로세스 CPU 시간(`resource.getrusage`, 사용자+커널)과 자발·비자발 문맥 전환 증가량을 함께 남긴다.

### 판독 방법

| 증상 | 해석 |
|---|---|
| 같은 group의 다른 콜백이 sleep 중 | CPU 증가가 작고, 스택에 그 콜백이 보인다 |
| GIL을 쥔 C 호출 | CPU 증가 ≈ 공백 시간 |
| 모든 스레드 대기 중 + CPU 증가 작음 | 프로세스가 CPU를 못 받은 것 |

### 사무실 PC에서의 검증

일부러 멈춰서 확인한 결과다.

- 기본 group 콜백에서 0.6 s sleep: 스택에 그 함수가 보였고, 공백 0.602 s, CPU 0.006 s.
- 별도 group에서 `sum(range(n))` 한 번(0.708 s, C 루프라 GIL을 놓지 않음): 스택에 그 함수가 보였고, 공백 0.719 s, CPU 0.722 s.

시험 설계 함정도 있었다. 첫 시도에서 짧은 `sum()`을 반복했더니 호출 사이에 GIL이 풀려 heartbeat가 계속 나갔다. 진단이 아무것도 못 잡았다.

### 진단이 실제로 잡은 것

진단을 켜고 사무실 PC에서 실제 controller I0를 돌리다가 자연 발생 정지를 세 번 잡았다(증거: [heartbeat-stall-evidence](../../records/reviews/heartbeat-stall-evidence-2026-11-13.md)).

- 장소: 셋 다 `mock_world_node`, 기동 2.5~3.2 s 뒤(다른 노드들이 올라오는 구간, CI 실패와 같은 구간). 공백 0.31~0.38 s, 그동안 CPU 약 0.01 s.
- 스택: 멈춘 스레드는 셋 다 rclpy `Publisher.publish` 안에 있었다. 둘은 heartbeat, 하나는 프레임 데이터 발행이었다. 다른 작업 스레드는 Python 코드 중간에 멈춰 있었고 CPU를 거의 쓰지 않았다.
- 이 노드의 heartbeat 타이머는 이미 자기 callback group에 있다. 그러니 group 경합이 아니라 **발행 호출 자체**가 멈춘 것이다. 처음 의심(group)대로 타이머를 옮겼다면 고치지 못했을 것이다.
- rclpy jazzy 소스의 `Publisher::publish`는 `rcl_publish`를 부르는 동안 GIL을 놓지 않는다(`py::gil_scoped_release` 없음). 그래서 발행이 막히면 그 프로세스의 모든 Python 스레드가 같이 멈춘다. 스레드 하나만 막힌 것처럼 보여도 프로세스 전체가 멈출 수 있다는 뜻이다.
- 아직 추정인 것: 멈춘 곳이 rmw·DDS 안이라는 것(Python 스택만 있고 native 스택은 없다), 기동 구간의 discovery와 관련 있다는 것, CI의 mock controller 공백도 같은 원인이라는 것. 그래서 판정 한도·callback group·DDS 설정은 바꾸지 않았다.
- 같은 날 실제 `robot_controller_node`도 기동 직후 같은 모양의 공백(발행 뒤 수신 0.000 s)을 냈다. 현장 운용 절차(기동 → 2 s 대기 → 사유를 남기고 해제)를 지키는 이유다.

### 운용

- 대역만 띄우는 ROS I0 시험(`test_i0_integration.py`)의 launch가 값이 없으면 `0.25`로 켠다. CI 워크플로 전체에는 켜지 않았다. 실제 controller I0를 켜고·끄고 번갈아 11번 돌렸더니 켠 6번 중 3번에 실패가 있었고(실제 노드의 종료 traceback 2번, RELEASE 시간 초과 1번) 끈 5번은 0번이었다. 진단이 실제 노드에 닿는 길은 찾지 못했지만, 원인 모를 차이를 CI 판정에 섞지 않는다.
- 기본(미설정)은 아무것도 하지 않는다. `beat()` 한 번은 약 100 µs다.
- 시험: `TC-MCK-012-01`(U, 하위 프로세스), `TC-MCK-012-02`(C, mock controller 프로세스를 SIGSTOP 0.6 s 뒤 SIGCONT하면 공백 경고).
- SIGSTOP은 감시 스레드도 멈추게 한다. 그래서 스택 출력은 재개 순간 경합이고 판정하지 않는다.
- 판정 한도 0.3 s와 콜백 group은 근거가 생기기 전에 바꾸지 않았다.

### 재현·확인 방법

- `DROP_MOCK_STALL_DUMP_S=0.25`를 주고 mock 노드를 launch로 띄운다. 멈춤이 있으면 launch 출력에 `Timeout (0:00:00.250000)!`과 스레드별 스택, 이어서 `STALL heartbeat 경로 공백` 경고가 나온다.
- `TC-MCK-012-01`, `TC-MCK-012-02`를 돌린다.
- `TC-MCK-012-02`는 mock controller 프로세스를 SIGSTOP 0.6 s 뒤 SIGCONT한다. 공백 경고가 나오면 통과다.

## 4. 측정 스크립트를 가짜 드라이버로 끝까지 돌리기

### 배경

`tools/bench/stream_bench.py`는 E0509에 speedl/servol 스트림으로 사인 궤적을 보낸다. joint_states + URDF 순기구학으로 실제 위치를 받아 지령 간격·지연·추종 오차를 낸다.

실기에서는 2026-11-07에 돌렸다.

Progress의 "mode:=virtual에서 동작 확인"은 DRCF 에뮬레이터(docker)가 필요한데, 쓸 수 있는 PC에 없었다. 그래서 실제 dsr_msgs2 형식의 **대역(stub) 드라이버**로 대신했다(`tools/bench/test_stream_bench_ros.py`, `TC-ROB-011-02`).

### 대역의 구성

- 합성 6축 URDF
- joint_states 100 Hz
- 지령 기준 위치를 순수 지연 50 ms로 따르는 가짜 팔
- `get_robot_state`, `get_last_alarm`, `get_current_posx`, `move_stop` 서비스

격리 방법: 스크립트는 하위 프로세스로 돌리고, 시험 전용 ROS 도메인, `--ns /bench_stub`, 임시 `--out-dir`를 쓴다. 실제 `/dsr01` 드라이버에 닿지 않게, 저장소 `records/bench`에 파일을 남기지 않게 하기 위함이다.

### 찾은 결함

안전 감시가 ramp(1 s) 안에서 중단하면 분석 함수가 빈 배열로 `np.percentile`을 불러 `IndexError`를 냈다. 요약 파일 없이 종료 코드 1로 끝났다.

고침: 표본이 없으면 추종 통계를 생략하고 사유를 남긴 뒤 종료 코드 3을 돌려준다.

### 대역을 만들며 겪은 함정 두 가지

**(a) 잘못된 적분 방식**: 처음 대역은 tick마다 속도를 적분해서, 지령이 늦게 오면 그 tick 전체를 0으로 처리했다. 그래서 추종 오차가 1 mm로 표류했다. 지령 수신 시각 기준 구간별 정확한 적분으로 고쳤다.

**(b) 콜백 직렬 실행 + GIL**: rclpy 노드의 콜백은 기본으로 한 상호 배제 group이라 MultiThreadedExecutor여도 하나씩 돈다. tick에서 수치 역기구학(FK 약 24회)이 GIL을 오래 잡아 지령 구독이 밀렸다. depth 10 큐가 넘쳐 601개 중 585~600개만 받았고, 추정 지연이 73~114 ms로 부풀었다.

고침: tick을 FK 1회 + 고정 야코비안 역행렬로 줄이고 SingleThreadedExecutor로 바꿨다. 그 결과 3회 모두 5 passed였다. 추정 지연 0.056 s(넣은 50 ms + 표본·전달 약 6 ms), 지연 보정 추종 오차 p95 약 8 µm였다.

### 배운 것

대역(가짜)이 느리면 측정 대상이 아니라 대역의 지연을 재게 된다.

### 확인하지 않는 것

- 제어기 수락
- 실제 지연·지터
- 실제 정지
- E0509 기구학
- 현장 드라이버 형식(OFF-2)

### 재현·확인 방법

- `dsr_msgs2`가 있는 환경(사무실 PC WSL)에서 `python3 -m pytest tools/bench/test_stream_bench_ros.py -rA`를 돌린다. 시험 안에서 스크립트가 하위 프로세스로 실행된다. `dsr_msgs2`가 없으면(CI·GCE) 사유와 함께 skip이고 통과로 세지 않는다.
- 지연 추정 시험(`test_speedl_analysis_recovers_injected_delay`)은 `realtime_pc`다. 공유 러너에서는 판정하지 않는다.
- 일찍 난 안전 중단은 `test_guard_abort_still_stops`(1.5 s 뒤 다른 축 8 mm 표류 → 종료 코드 3)와 순수 시험 `test_analyze_early_abort_keeps_interval_stats`(GCE에서도 돈다)로 본다.

## 5. 흔한 함정

1. **재현 안 됨 = 조건 문제일 수 있다.** 병뚜껑 초기 위치 하나 바꿔서 바로 재현됐다. 입력 조건을 바꿔 가며 시도한다.
2. **inf·nan은 정수 변환에서 터진다.** `int()`, `math.floor()`, ROS Time 변환으로 inf가 흘러가는지 경로를 따져 본다.
3. **GIL을 쥔 C 호출은 프로세스 전체를 막는다.** 긴 `sum()` 같은 C 루프나 GIL을 놓지 않는 rclpy `publish`가 막히면 다른 스레드의 콜백도 못 돈다. callback group을 나눠도 소용없다.
4. **짧은 연산 반복은 멈춤 재현이 안 된다.** 호출 사이에 GIL이 풀려 다른 콜백이 돌아간다. 재현에는 한 번의 긴 연산이 낫다.
5. **rclpy 콜백은 기본으로 직렬 실행이다.** MultiThreadedExecutor여도 기본 group이 상호 배제면 하나씩 돈다.
6. **느린 대역은 측정을 망친다.** 대역 지연이 측정값에 섞인다. 대역이 충분히 빠른지 먼저 본다.
7. **큐 depth는 유한하다.** 구독이 밀리면 큐가 넘쳐 메시지가 유실되고, 지연 추정이 부풀어진다.

## 6. 관련 파일

- 코드: [`scenario_engine.py`](../../src/drop_mocks/drop_mocks/scenario_engine.py), [`stall_watch.py`](../../src/drop_mocks/drop_mocks/stall_watch.py), [`ros_common.py`](../../src/drop_mocks/drop_mocks/ros_common.py)(`spin_mock`)
- 시험 harness: [`test_i0_integration.py`](../../src/drop_bringup/test/test_i0_integration.py)(`_launch`)
- 벤치: [`stream_bench.py`](../../tools/bench/stream_bench.py), [`test_stream_bench_ros.py`](../../tools/bench/test_stream_bench_ros.py)
- 명세: [SPEC-700](../../specs/SPEC-700-mocks.md) 4.8절·REQ-MCK-011·012, [SPEC-500](../../specs/SPEC-500-robot-control.md) REQ-ROB-011 표
- 기록: [worklog](../../records/worklog/2026-11-13.md), 증거 [heartbeat-stall-evidence](../../records/reviews/heartbeat-stall-evidence-2026-11-13.md)
- 시험 자료: [`stopped_belt.yaml`](../../src/drop_mocks/test/data/stopped_belt.yaml)

---

worklog가 사실 기록이고 이 문서는 설명이다. 수치·커밋은 worklog를 따른다.
