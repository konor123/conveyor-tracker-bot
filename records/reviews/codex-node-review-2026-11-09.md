# 노드 묶음 독립 코드 리뷰 — 2026-11-09

- 수행자: Codex, dispatched worker W2 (`task_ab8702c6edb2`, `ctx_ef5e679e77b7`), coordinator Claude Code.
- 장소·시각: GCE, 2026-11-09 05:06 UTC 이후(14:06 KST 이후).
- 기준 HEAD: `47308d8200fc65f7475042ed011fb1aba5639cd7`. 다른 worker의 문서·도구 수정이 같은 트리에서 진행 중이다. 아래 검토 대상 Python 파일은 리뷰 중 수정하지 않았다.
- 범위: orchestrator_node.py, safety_supervisor_node.py, record_logger_node.py, ros_qos.py 전체, cycle_runner.py, watchdogs.py, config/orchestrator.yaml, 지정 노드 시험 및 drop_bringup/test. PLAN.md·Progress.md·TEST_SPEC.md·당일 worklog·SPEC-100 v0.3·SPEC-110·SPEC-800·SPEC-010을 대조했다.
- 사용량·배정: coordinator가 지정한 독립 검수 작업을 수행했으며 재위임하지 않았다. 본 worker의 현재 사용량·잔여량·초기화 시각은 조회하지 않아 미확인이다. worklog 02:34 UTC의 Claude/Codex/Agy 수치는 과거 관측이며 현재 잔여량으로 해석하지 않았다.
- 제약: 저장소 파일 변경·commit·push·PC 접속·실장비 지령 없음. 이 보고서만 중앙 memory에 작성했다. 저장소 worklog·Progress 갱신과 최종 게이트 처리는 coordinator 소관이다.

## 발견(심각도 순)

### F1 — P0 안전: action 수락 응답이 늦으면 실행 중 goal을 잃고 cancel 성공으로 처리한다

- 위치: `src/orchestrator/orchestrator/orchestrator_node.py:165`~172, `:417`~419; `cycle_runner.py:473`~479.
- 시나리오: send_goal_async로 요청을 보낸 뒤 수락 future가 완료되기 전에 deadline 또는 장애로 `_wait`가 false를 반환한다. `_send`는 future나 goal id를 보관하지 않고 반환한다. 이어 cancel_active는 `_goal is None`이라 true를 반환하므로 실행기는 cancel ack를 받은 것으로 간주한다. 서버가 요청을 이미 수락하고 동작 중이거나 수락 응답만 늦은 경우, 이전 skill은 취소되지 않은 채 안전 자세·후퇴 action을 새로 요청할 수 있다. deadline 자체 중단이 있는 서버도 deadline 이전 장애 중단 경합은 방어하지 못한다.
- GCE AST 재현: server ready=true, send future done=false, 미래 result는 accepted handle로 설정. `_send` 뒤 `cancel_active(.01)` 실행 출력: `send result=None, cancel_active=True, cancel calls=0, stored handle=None`.
- 기준: SPEC-100 4.7절 action 무응답 cancel 및 SPEC-110 REQ-ABT-004. 단일 지령 권한은 클라이언트 주체뿐 아니라 이전 goal 종료 확인까지 보장해야 한다.
- 권고: pending-send future를 cycle별로 보관하고, 늦은 수락에도 반드시 cancel 및 결과 확인을 수행한다. 수락 여부 미확정은 cancel ack로 인정하지 않는다. 확정 전 대체 모션을 억제하고 제한시간 초과를 승격한다. 이전 cycle의 feedback도 cycle/goal token으로 격리한다.
- 시험 허점: 기존 robot_hang 시험은 수락된 goal의 결과 지연을 주로 검사한다. 수락 future 지연·장애 동시 발생·늦은 수락 후 실제 cancel 호출 및 후속 motion 부재를 검증해야 한다.

### F2 — P0 안전: supervisor가 Stop 전송 실패를 영구 latch하여 서비스 복구 후에도 재시도하지 않는다

- 위치: `src/orchestrator/orchestrator/safety_supervisor_node.py:68`~77, `:117`~128.
- 시나리오: 보호정지/통신단절 검출 시 monitor.check가 사유를 latch한다. 그 순간 Stop 서비스가 0.1 s 내 준비되지 않으면 RPC 없이 반환한다. 같은 장애가 지속하면 후속 tick은 None이므로 서비스가 복구되어도 Stop을 보내지 않는다. call_async의 실패나 `stopped=false`도 확인하지 않는다. 이 문제는 orchestrator가 멈춘 경우의 독립 안전 경로를 잃게 한다.
- GCE AST 재현: connected=false, 첫 tick service ready=false, 다음 tick ready=true. 출력: `attempted=1, actual RPC=0`.
- 권고: 장애 검출 latch와 Stop 전달·정지 확인 상태를 분리한다. `stopped=true` 확인 전에는 bounded retry하고 future 예외·거부를 기록한다. 연속 장애 동안 과도한 중복 RPC는 in-flight 제어로 방지한다.
- 시험 허점: test_safety_supervisor_node_calls_stop_only는 서비스를 먼저 준비한다. 서비스 지연 등장·호출 실패·정지 미확인 응답을 시험해야 한다.

### F3 — P1 정확성: 포트 대기 중 steady 사이클·중단 경로 상한, cancel, 종료가 작동하지 않는다

- 위치: `src/orchestrator/orchestrator/orchestrator_node.py:141`~157, `:617`~628; `cycle_runner.py:450`~457, `:490`~504.
- 시나리오: `_wait`는 pred, latch, ROS deadline만 검사한다. CycleTimer.expired와 cancel_requested는 상태 진입 경계에서만 검사한다. heartbeat는 정상이나 ROS /clock이 멈추고 service/action 응답이 없는 경우, steady timeout×2가 지나도 대기에서 나오지 않는다. L2/L3 경로도 ROS deadline에 묶인 포트 호출이 반환해야 AbortTracker.tick을 실행하므로 10/15 s steady 상한을 놓친다. ROS 시각이 정상이어도 cancel은 남은 step/pass deadline까지 늦어지고, 단계 timeout으로 처리되어 CANCELED 대신 FAILURE/SAFE_ABORT가 먼저 결정될 수 있다. `_wait`에 rclpy.ok/context shutdown 검사도 없어 SIGINT 시 실행 worker가 남거나 destroy된 node를 사용하는 경로가 있다.
- GCE AST 재현: ros_now=0 고정, is_cancel_requested=true, pred=false, deadline=1.0으로 worker thread를 시작했다. 40 ms 뒤 여전히 blocked였으며 hard fault를 주입해야 종료했다. 40 ms 관찰 자체는 90 s 상한 측정이 아니며, steady·cancel·shutdown 판정 부재는 해당 메서드 전체 소스로 확인했다. SIGINT 실제 ROS 종료 시험은 수행하지 않았다.
- 기준: REQ-ABT-003·005·008, SPEC-110 5절 호출 steady 제한시간, PLAN 0.1 모든 동작 제한시간.
- 권고: 포트 대기에서 cycle/abort steady 예산·cancel·shutdown을 검사하고 원인을 구분해 실행기로 전달한다. 해제 뒤 비하드 장애는 후퇴·검증을 계속하는 REQ-ORC-010 예외를 유지한다. abort 경로는 별도의 steady 제한시간으로 감시한다.
- 시험 허점: CycleTimer 단위 시험 및 test_watch_timers_ros는 각각 timer 판정/heartbeat 감시만 확인한다. heartbeat를 계속 보낸 채 실제 포트 future만 정지시키는 frozen-clock I0, 대기 중 cancel, L3 경로 frozen-clock, 진행 중 SIGINT 종료 시험이 필요하다.

### F4 — P1 정확성: 비동기 감시가 heartbeat·로봇 연결·보호정지 외의 필수 이벤트를 생성하지 않는다

- 위치: `src/orchestrator/orchestrator/orchestrator_node.py:518`~527; `watchdogs.py:33`~76; `config/orchestrator.yaml` watchdog 절.
- 시나리오: PRECHECK가 지난 뒤 gripper_error_bits≠0, gripper_response_s=0.501, gripper_alive=false가 되거나 belt.steady=false가 되어도 `_watch`는 아무 이벤트를 만들지 않는다. ClockMonitor/FreshnessMonitor/GripperMonitor는 모듈에 있으나 노드에서 연결하지 않는다. 관측 freshness·시각 역행 감시도 없다. 정상 heartbeat만으로 진행 중 skill 대기는 계속될 수 있다.
- GCE AST 재현: connected=true, protective_stop=false, required_nodes=[], gripper_alive=false, response=1.0, error_bits=1, belt.steady=false로 `_watch` 실행. 출력 `events=[]`.
- 기준: SPEC-100 4.7절 그리퍼 이상 등 비동기 이벤트, SPEC-110 4.3절 및 REQ-ABT-011~014. 컨트롤러의 해제 게이트가 일부 조건을 중복 방어할 수 있지만 orchestrator의 명시된 이벤트·중단 등급 계약을 대체하지 않는다.
- 권고: 상태별 watch를 순수 monitor에 연결한다. selected track의 epoch와 t_last_obs, ROS 시각 역행, 벨트 steady, 그리퍼 상태를 snapshot으로 판정한다. 인터페이스에는 alive boolean이 있으므로 이를 이용하거나 생존 카운터 입력 계약을 명시한다.
- 시험 허점: FakeSkillPort가 이미 계산된 Event를 반환하는 시험은 이벤트 생성 배선 누락을 잡지 못한다. 각 ROS 입력만 바꾼 실제 port/node 시험에서 latch·중단 등급·후속 skill 부재를 검증해야 한다.

### F5 — P1 정확성: logger의 파일 오류가 executor로 전파되고 실패한 trial을 재시도에서도 버린다

- 위치: `src/orchestrator/orchestrator/record_logger_node.py:236`~267, `:269`~280.
- 시나리오: 저장 디렉터리 권한 오류·디스크 부족·read-only filesystem에서 makedirs/open/write/flush의 OSError를 잡지 않는다. callback future 예외가 main spin으로 전파될 수 있으며 main은 KeyboardInterrupt/ExternalShutdownException만 처리한다. trial의 `_last_trial_id`는 파일 쓰기 성공 전에 설정되므로 오류 후 같은 trial 재전달도 중복으로 간주해 폐기한다.
- GCE AST 재현: records root=`/proc/conveyor-review-no-write`로 StepRecord callback 실행 시 `FileNotFoundError`가 callback 밖으로 전파되었다. 실제 disk-full과 ROS executor 종료는 미실행이다.
- 기준: SPEC-800 REQ-REC-002·003, 모든 시도·단계 기록 보존.
- 권고: 파일 오류를 명시적으로 처리하고 bounded queue/retry 또는 실패 상태를 노출한다. dedup commit은 durable append 성공 뒤에 한다. 보관 실패를 정상 기록 성공으로 보고하지 않는다.
- 시험 허점: 정상 tmp_path 쓰기만 확인한다. makedirs/open/flush 실패 및 복구 뒤 같은 trial 재전달을 검증해야 한다.

### F6 — P2 견고성: reentrant logger가 StepRecord 순서를 뒤집고 공유 metadata를 비원자적으로 읽는다

- 위치: `src/orchestrator/orchestrator/record_logger_node.py:160`~170, `:180`~213, `:240`~247, `:275`~280.
- 시나리오: MultiThreadedExecutor(4)에서 모든 구독·poll이 ReentrantCallbackGroup에 있다. 먼저 도착한 step callback이 파일 I/O에서 멈추면 다음 callback이 먼저 append한다. scenario/seed/assumed_params도 여러 callback에서 각각 바뀌고 trial 빌더는 snapshot 없이 읽으므로 서로 다른 시점의 metadata를 조합할 수 있다. written/trials_written의 compound 증가 및 `_poll_inflight` check-then-set도 직렬화되지 않는다.
- GCE 강제 교차 재현: seq1 callback의 makedirs 진입을 Event로 막고 seq2 callback을 먼저 완료한 뒤 해제했다. 실제 JSONL 순서 `[2, 1]`을 확인했다. 줄 단위 JSON 자체가 깨진다고 주장하는 것은 아니다.
- 권고: logger 상태·파일 쓰기를 단일 writer로 직렬화하거나 상호배제 callback group/lock과 metadata snapshot을 사용한다. downstream이 반드시 step_seq 정렬해야 하는 계약이라면 그 계약도 명시한다.
- 시험 허점: test_node_writes_jsonl_per_cycle는 순서를 기대하지만 강제 I/O 교차를 만들지 않는다. 실제 동시 callback·metadata 갱신 경합을 시험해야 한다.

### F7 — P2 견고성: JSON 문법이 유효한 잘못된 summary는 격리되지 않고 callback을 종료시킨다

- 위치: `src/orchestrator/orchestrator/record_logger_node.py:230`~249.
- 시나리오: json.loads만 try에 있다. `[]`/`null`은 summary.get에서, 잘못된 숫자 타입은 build_trial_record에서 예외를 일으켜 schema 검증·invalid.jsonl 보관까지 도달하지 않는다. JSONL schema 오류 격리 정책이 모든 유효 JSON에 적용되지 않는다.
- GCE AST 재현: `[]` → AttributeError, `{"trial_id":"x","cycle_time_s":"bad"}` → ValueError. 두 예외 모두 callback 밖으로 전파되었다.
- 권고: mapping 타입 및 각 필드를 변환 전에 검증하고, parse/build/schema 오류를 하나의 격리 경로로 처리한다. 그 후에도 같은 tid 수정본을 다시 받을 수 있어야 한다.
- 시험 허점: 정상 summary와 build 결과 schema 검증만 있고 잘못된 root/type 입력 시험은 없다.

### F8 — P2 견고성: 일반 install에서 schema를 찾지 못해도 검증 통과로 취급한다

- 위치: `src/orchestrator/orchestrator/record_logger_node.py:102`~117; `src/orchestrator/setup.py:11`~15.
- 시나리오: schema 후보는 Python 모듈에서 3단계 위 tools/eval 또는 CWD/tools/eval뿐이다. setup.py는 schema를 설치하지 않는다. symlink-install 원본 경로에서는 정상 동작하지만 일반 install 배포·저장소 밖 CWD에서는 두 후보가 없을 수 있고 이때 validate_trial_record는 `[]`를 반환한다. 실제 검증 없이 malformed record를 정상 trials.jsonl에 쓸 수 있다.
- 근거: 경로/설치 목록 정적 대조. 기존 external symlink-build 시나리오는 AST 실행에서 실제 schema를 찾아 `{}`의 required 오류 13개를 정상 반환했다. 이 정상 경로를 결함으로 분류하지 않았다. 일반 ROS install 배포 재현은 수행하지 않았다.
- 권고: schema를 패키지 share에 설치하고 ament 경로로 찾는다. schema 부재는 명시적 실패/상태로 취급한다.
- 시험 허점: external symlink-build 시험만으로 일반 copy install 경로를 보장하지 못한다.

## 다른 검토 결과와 한계

- 오늘 도입한 `_fault_lock`은 raise_fault의 우선순위 검사·저장, pending_fault 소비, reset, released flag를 같은 lock으로 보호한다. 기존 하드/비하드 동시 raise 손실은 이 코드에서 재발한다고 판단하지 않았다. drop_goal_handle 종료 대입은 lock 밖이지만 이 사실만으로 사용자 영향 있는 결함을 단정하지 않았다.
- orchestrator의 heartbeat와 RobotStatus writer는 hb_group으로 직렬화되나 runner는 별도 reentrant 실행이고 pending_fault에서 `_watch`를 직접 호출한다. 읽기에는 dict 전체 순회 중 구조 변경 같은 확정 예외가 보이지 않았다. robot_status 등 메시지 참조 대입은 개별 snapshot처럼 동작해도 여러 토픽의 동시 snapshot을 보장하지 않는다. 이를 모두 막연한 thread-unsafe 결함으로 올리지 않고 F4/F6의 실제 영향에 집중했다.
- supervisor는 기본 상호배제 그룹과 single-threaded rclpy.spin을 사용한다. unused ReentrantCallbackGroup/MultiThreadedExecutor import가 있다는 이유로 monitor thread race를 제기하지 않았다. 로봇 모션 action/stream을 소유하지 않고 Stop requester=SUPERVISOR만 보내는 점은 요구와 일치한다.
- RESULT_NAMES 순서는 DropTask.Result의 0~6과 일치한다. 알 수 없는 결과의 UNKNOWN fallback도 확인했다. release_error/stone_offset validity=false는 미측정 값을 유효하다고 보고하지 않는다.
- `step_durations_s` dict comprehension(orchestrator_node.py:590)은 재시도/여러 WAIT_CAP의 같은 이름을 마지막 한 건으로 덮는다. 총 단계 시간 통계로 사용하면 축소된다. 이 필드를 누적값/최종 시도값 중 무엇으로 정의할지 명세가 명확하지 않아 별도 차단 결함으로 세지 않았다.
- DropTask feedback publish_feedback 호출은 없다(SPEC-100 2절 출력과 SPEC-010 DropTask feedback 필드 참조). TaskStatus로만 진행 상황을 제공하며 action feedback 계약은 시험하지 않는다. action client가 진행 feedback에 의존해야 하는 범위는 coordinator 확인이 필요하다.
- 정상 DONE reason_code가 빈 문자열이 될 수 있고 VerifyDrop의 상세 reason_code·stone_offset·hold_time이 외부 결과까지 전달되지 않는 경로를 확인했다. 현재 명세 표의 빈 이유/미측정 값 처리와 구분해 후속 계약을 명시할 필요가 있다.
- ros_qos.py의 use_default_callbacks=False는 16행 전체와 생성 지점을 확인했다. SPEC-010 v0.3의 QoS 계약 시험으로 불일치를 검사한다는 정책과 일치한다. 조합 QoS 자체의 live 호환성 검사는 이 GCE에서 실행하지 않았다.
- StepRecord 경로 cycle_id의 `/`·`..` 거부는 Linux 기준 정상이다. date는 trial 경로에서 검사하지만 steps_path에서는 검사하지 않는다. date가 운영자가 주는 신뢰된 설정이므로 이를 외부 공격으로 단정하지 않았다.

## 검증 기록

### GCE 실행

```
PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py
```

결과: **591 passed, 2 skipped, 2.98 s**. skip은 rclpy가 필요한 test_ros_skill_port.py·test_watch_timers_ros.py이며 ignore한 두 파일은 실행 수에 포함하지 않았다. 따라서 591개 통과를 노드 계약 통과로 해석할 수 없다.

AST 재현은 현재 파일에서 클래스·함수 AST와 future annotations만 추출하고 Node=object, ROS clock/future/client를 SimpleNamespace로 대체했다. 파일 복사 구현이 아니라 실제 메서드를 실행했다. `_wait`는 daemon thread와 Event로 관찰 후 hard fault로 해제했고, logger 순서 역전은 실제 임시 디렉터리에 JSONL을 썼다. AST 첫 실행은 annotation용 sj 미정의로 NameError가 나서 실행하지 못했으며, future annotations AST를 보존한 다음 실행은 위 R1~R8 결과를 냈다. ROS transport·executor·장비의 실제 동작 재현과는 구분한다.

대표 AST 재현 구조(저장소 루트, ROS 불필요):

```python
import ast, threading, time
from types import SimpleNamespace as NS
from orchestrator.abort_policy import Event, HARD_FAULTS
from orchestrator.common import Mode
p = 'src/orchestrator/orchestrator/orchestrator_node.py'
t = ast.parse(open(p).read())
t.body = [n for n in t.body if
          isinstance(n, ast.ImportFrom) and n.module == '__future__' or
          isinstance(n, ast.ClassDef) and n.name == 'RosSkillPort']
env = dict(threading=threading, time=time, Mode=Mode, Event=Event,
           HARD_FAULTS=HARD_FAULTS, POLL_S=.001, _to_time=lambda x: x)
exec(compile(t, p, 'exec'), env)
n = NS(clock=NS(ros_now=lambda: 0.), cycle_id='c', step_id=1,
       get_logger=lambda: NS(warning=lambda msg: None), _watch=lambda: None)
port = env['RosSkillPort'](n)
port.reset(NS(is_cancel_requested=True))
done = threading.Event()
th = threading.Thread(target=lambda: (port._wait(lambda: False, 1.), done.set()), daemon=True)
th.start(); time.sleep(.04)
assert not done.is_set()  # cancel/steady/shutdown은 이 루프에서 검사하지 않는다
port.raise_fault(Event.PROTECTIVE_STOP); th.join(1)
port.reset(object())
cancels = []
gh = NS(accepted=True, cancel_goal_async=lambda: cancels.append(1),
        get_result_async=lambda: NS(done=lambda: False))
fut = NS(done=lambda: False, result=lambda: gh)
n.actions = {'pick': NS(server_is_ready=lambda: True,
                        send_goal_async=lambda *args, **kwargs: fut)}
port._wait = lambda pred, deadline: pred()
assert port._send('pick', NS(), 1.) is None
assert port.cancel_active(.01) is True
assert not cancels and port._goal is None
```

`PYTHONPATH=src/orchestrator python3`로 실행할 수 있다. 위의 짧은 관찰 시간은 장시간 timeout 실측값이 아니다.

### OCR

- `ocr delegate preview --repo .`: 実行時の差分は 0 reviewable / 0 total。全体レビューなので差分なしでも指定ファイル全体を読んだ。
- `ocr delegate rule src/orchestrator/orchestrator/orchestrator_node.py src/orchestrator/orchestrator/safety_supervisor_node.py src/orchestrator/orchestrator/record_logger_node.py`: system Python ruleを取得。precision優先、実行文脈のある競合のみ指摘、正確性/安全性はblocking、styleはnon-blockingという基準に従った。provider追加・認証情報設定なし。

### 既存 ROS 証拠

- `records/worklog/2026-11-09.md`に officepc WSLで build 10/10、orchestrator 617、drop_bringup 8通過等の記録、および CI run **42563966553 attempt 1・2 success**がある。
- このレビューでは PC・GitHubへの接続は行わず記録を読んだ。上記CIは既存I0が通る根拠であり、今回の遅延受理・Stop不可用・時計停止中のポート待機・記録I/O障害の通過根拠ではない。

## 노드 묶음 식별용 SHA-256

| ファイル（src/orchestrator/orchestrator/以下） | SHA-256 |
|---|---|
| orchestrator_node.py | b587d9c119216b257cb70ed3897fb2ac6b841a8a698a7070aa2102fddaef8520 |
| safety_supervisor_node.py | 169cf47a53a895495b29d2c226218c727368667858947cbc3222c5d96a263ccd |
| record_logger_node.py | 59f181599f3c1d7a462f131f0111a662ac1f5e44b3f705474eccefd9ad1ecf00 |
| ros_qos.py | 719b997a86c4f65933ea42b135d2a630b304381bf2dd9788ba2ed89c62b13a40 |
| cycle_runner.py | 93b48e78449c16c275f767f4af11e70c10a7fbd2084b0695ce8ac13be4e7ef1f |
| watchdogs.py | 0b9b854be97cebdc00c0b13e2bb46b8cbf2123b9a86fbe2527723a2787fa1c93 |

## 노드 묶음 독립 리뷰 통과 가능 여부

**현재 통과 불가.** P0 2건(F1·F2), P1 3건(F3·F4·F5), P2 3건(F6·F7·F8)을 확인했다. F8은 설치 목록·경로 정적 근거이며 일반 install ROS 환경 실증은 별도다.

통과 조건은 F1~F5 수정과 해당 실패 시나리오의 회귀 시험·독립 재검수다. F6~F8도 정상 기록 보장 조건으로 해결하거나 제한된 운용 범위·명시적인 후속 조건으로 coordinator가 기록해야 한다. ROS 환경에서 늦은 수락 cancel, Stop 서비스 지연·정지 응답 실패, heartbeat 정상 + frozen clock, 진행 중 cancel·SIGINT, gripper/관측/clock/belt 입력 장애 주입, logger I/O·동시 callback을 검증한다. 기존 I0와 REQ-ORC-010 lock 경합 시험도 함께 통과해야 한다. 실제 장비 지령이 필요한 검증은 이 리뷰에 포함하지 않는다.
