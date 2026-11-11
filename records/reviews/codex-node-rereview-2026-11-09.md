# W8 노드 묶음 독립 재검수 — 2026-11-09

- 판정: **노드 묶음 독립 리뷰 불통과**. F1·F2·F5·F6·F7의 원 지적은 수정됐고 F4 게이트 이관은 일관된다. 그러나 해제 후 만료된 사이클 deadline이 정상 후퇴를 거부하고 VERIFY를 생략하게 하는 P1 결함 N1이 남는다. F3는 부분 해결이며 F8은 fail-closed 완화만 완료다.
- 수행: Codex 독립 검수 worker W8, task `task_740aaa761f5c`, dispatch `ctx_f6e4cac3da6f`; coordinator Claude Code. GCE, 2026-11-09 06:10~06:14 UTC(15:10~15:14 KST, 보고서 작성 구간 포함).
- 기준: main HEAD `47308d8200fc65f7475042ed011fb1aba5639cd7`의 미커밋 통합 트리. 저장소 AGENTS.md, 1차 리뷰, W5/W7 보고서, production 4개 파일, 신규 회귀·기존 cycle/ROS port 시험, SPEC-100/110/800·TEST_SPEC·worklog를 대조했다. `.codegraph/`는 없어 CodeGraph를 사용하지 않았다.
- 제약 준수: 저장소·specs·records 변경, commit/push, PC·장비 접속 없음. 산출물은 이 중앙 memory 보고서뿐이다. pytest bytecode·cache 생성을 비활성화했다. 저장소 단계 기록·최종 게이트·CI 처리는 coordinator 소관이다.
- 사용량·배정: 재위임하지 않고 배정받은 독립 검수만 수행했다. 이 worker의 현재 사용량·잔여량·초기화 시각은 미확인이다. coordinator/fleet의 현재 잔여량도 이 검수에서 확인하지 않았으며 과거 보고 수치를 현재 값으로 간주하지 않았다.

## 새 발견 — 심각도 순

### N1 — P1 / 차단: 해제 뒤 사이클 deadline이 지나면 정상 후퇴 결과를 거부하고 VERIFY 없이 L4로 간다

근거 위치:

- `src/orchestrator/orchestrator/cycle_runner.py:219`~221, `:241`~255, `:454`~461: 해제 뒤에는 cycle timer guard와 비하드 중단을 억제한다.
- 같은 파일 `:275`~281: 그러나 RETREAT·VERIFY deadline은 해제 여부와 무관하게 `min(now+step_limit, ..., c.deadline)`이다.
- `src/orchestrator/orchestrator/orchestrator_node.py:153`~180: 만료된 deadline은 steady budget=0이며 `:173`의 cap 검사가 `:177`의 준비된 응답 검사보다 먼저 false를 반환한다. `_phase_done(:424)`와 `_result(:253)` 모두 이 대기를 사용한다.
- `cycle_runner.py:419`~425: 이 결과는 실제 로봇 후퇴 실패가 없어도 RETREAT_FAILED가 된다. `:233`~239, `:494`~519를 거쳐 Stop·WAIT_OPERATOR로 끝난다.
- 계약: SPEC-100 `:161`, SPEC-110 `:49`~56 및 REQ-ORC-010. 해제 뒤 비하드 이벤트(CYCLE_TIMEOUT 포함)에는 RETREAT·VERIFY를 계속해야 한다. 현재 SPEC-100의 일반 deadline 규칙과 cleanup 예외를 구현·명세 양쪽에서 정합화해야 한다.

GCE production AST 재현(성공 terminal과 RETREAT feedback가 이미 준비된 상태):

```
released=True, interrupting=False, ROS now=91.0, cycle deadline=90.0
CycleRunner._deadline(RETREAT)=90.0
RosSkillPort._wait(pred=lambda: True, deadline=90.0)=False
RosSkillPort.place_feedback('RETREAT', 90.0)=None
CycleRunner._do_retreat=(TIMEOUT/STEP_TIMEOUT, RETREAT_FAILED)
```

전체 production CycleRunner + 정상 FakeSkillPort 진행에 RETREAT만 실제 AST RosSkillPort로 연결한 추가 재현에서도, RETREAT 진입 때 ROS를 cycle deadline 뒤로 진행시키면:

```
ready RETREAT (now, deadline)=[(1099.7999999999997, 1090.0)]
result=PROTECTIVE_STOP, abort=4
end=['RETREAT', 'PROTECTIVE_STOP', 'WAIT_OPERATOR']
VerifyDrop calls=0, Stop calls=1
```

이는 실제 ROS transport 시험이 아니라 현재 production 메서드와 stub의 결합 재현이다. 첫 재현은 이미 준비된 성공 결과조차 거부됨을 직접 증명하고, 두 번째는 그 결과의 상태 전이를 증명한다. 기존 cycle deadline clipping은 이전부터 있던 잔여 문제이며, W5의 steady-cap-before-predicate 순서는 이미 준비된 응답까지 거부하는 경로를 강화했다. W7의 배율 2.0은 남은 시간이 0인 경우를 고치지 않는다.

권고: 해제 후 RETREAT·VERIFY에 독립적이고 유한한 cleanup 단계 예산을 부여하고, 만료된 원 사이클 deadline이 cleanup을 다시 중단시키지 않도록 한다. 하드 이상·실제 RETREAT 실패의 L4 우선순위와 frozen-clock steady 상한은 유지해야 한다. 단순히 guard를 제거하거나 predicate 순서만 바꾸면 응답이 조금 늦는 경우와 VERIFY 서비스 요청 억제는 남는다. 회귀 시험은 RELEASE 직후/RETREAT 도중/VERIFY 직전에 원 cycle ROS deadline을 실제로 넘기고, 성공 RETREAT·VERIFY 수행 및 실제 실패의 L4를 함께 검증해야 한다.

현재 `test_cycle_runner.py:384`의 모든 Event 주입 시험은 CYCLE_TIMEOUT라는 이벤트 값을 주입하지만 실제 clock을 deadline 뒤로 보내지 않는다. `test_ros_skill_port.py:54`의 release 후 비하드 대기 시험도 미래 deadline=10을 사용한다. 따라서 기존 green 결과와 N1은 양립한다.

## F1~F8 해결 판정

| 지적 | 판정 | 현재 production 근거와 시험 |
|---|---|---|
| F1 늦은 action 수락·cancel 오판 | **원 지적 해결** | `orchestrator_node.py:191`~250: pending future 보존, abandoned 상태·done callback에서 늦은 수락 cancel, 미확정 이전 goal이 있으면 새 action 차단. `:498`~531: pending 수락이나 terminal 예외를 ack로 보지 않고 status 4/5/6만 종료 확인. `cycle_runner.py:501`~516: cancel 미확인은 대체 모션 대신 L4. `test_node_failure_regressions.py:43,197,227`에서 늦은 callback, 후속 goal 차단, stale feedback token 격리, terminal 예외를 실행해 통과. 실제 ROS 수락 지연 transport 주입은 W8 미실행. |
| F2 Stop 전송 실패 영구 latch | **원 지적 해결** | `safety_supervisor_node.py:119`~160: active fault와 전달 상태 분리, 서비스 지연·stopped=false·future 예외 뒤 0.1 s 재시도. in-flight future가 끝나기 전 반환하여 동시 RPC 최대 1. 응답 영구 누락은 1 s warning을 유지하고 성공으로 간주하지 않음. 회귀 `:109,242` 통과. 기본 상호배제 callback group + single-thread spin이므로 새 lock 부재를 경합 결함으로 보지 않음. |
| F3 steady·cancel·shutdown 대기 | **부분 해결 / N1 때문에 불충분** | `orchestrator_node.py:153`~180: scaled finite cap, cycle/cleanup guard, cancel 원인·context 종료 검사. `cycle_runner.py:219,251,497`: 실행 중 guard 및 원인 회수·cleanup 별도 tracker. 신규 회귀 `:63,150,347` 통과(동결 ROS, shutdown, cancel, −0.8 s 역행 정상 완료 및 정지 cap). 다만 해제 후 만료 cycle deadline의 적용은 REQ-ORC-010과 충돌. SIGINT 실제 ROS 종료는 미실행. |
| F4 그리퍼·벨트·freshness·역행 ROS 감시 | **기능 미구현 / M0 게이트 이관은 일관됨** | `_watch`, `orchestrator_node.py:621`~630은 heartbeat·보호정지·연결만 감시하며 F4 입력은 여전히 연결하지 않음. SPEC-110 `:20,111`, TEST_SPEC `:297`~300 및 `:527`~537이 G0/G3/G4 -02 C 시험을 명시하고 M0에서 제외. 실제 경로 전 필수라는 범위 조건으로만 수용 가능. |
| F5 logger I/O 예외·재전달 손실 | **원 지적 해결, 운용 한계 있음** | `record_logger_node.py:135`~144, `:249,293,304`: callback 전체 OSError 격리·warning/storage_error, 정상 append/flush 뒤 dedup 확정. mkdir/open/flush 실패 뒤 같은 tid 재전달 회귀 `:78,314` 통과. 자동 재전달 queue는 없고 flush/close 부분 실패는 중복·부분 줄 가능; exactly-once나 전원 장애 내구성을 보장하지 않음. |
| F6 logger 순서·metadata 경합 | **원 지적의 강제 I/O 교차·공유 상태 문제 해결** | `record_logger_node.py:150,188,197,215,249,304`: 하나의 RLock으로 poll·reply·metadata·trial·step 전체 직렬화. `test_node_failure_regressions.py:169,280`에서 두 실제 step callback의 I/O 순서 [1,2], counter=2 및 동시 진입 차단 확인. lock은 callback이 lock을 얻은 순서를 보장하며 DDS 수신 시각/step_seq에 따른 재정렬 queue는 아님. |
| F7 유효 JSON 잘못된 summary 격리 | **원 지적 해결** | `record_logger_node.py:250`~274, `:297`: object 검사, parse/build 변환 예외 격리, invalid 입력은 정상 dedup 미확정. `[]`·잘못된 숫자와 I/O 복구 parameterized 회귀 `:78` 통과. invalid 경로의 OSError도 외곽 decorator로 격리. |
| F8 copy install schema 부재 | **fail-closed 완화 완료 / 배포 지원 미완료** | `record_logger_node.py:115`~118은 schema 부재를 오류로 반환하고 `:282`~289에서 invalid 파일로 보관. `test_node_failure_regressions.py:101` 통과. 그러나 `_find_schema_path(:104)`는 repo/CWD 탐색뿐이고 `src/orchestrator/setup.py:11`~15는 schema를 설치하지 않음. 정상 copy install·repo 밖 CWD에서 정상 trial을 기록하는 지원은 남아 있음. 현재 수용 범위는 schema가 실제 있는 저장소/symlink 배포에 한정. |

## 상호작용·스레드 검토

- 포트 자체 cap은 각 `_wait` 진입에서 남은 ROS 시간 ×2다. readiness와 response 대기는 각각 cap을 계산하지만 cycle timer guard와 L2/L3 tracker guard가 별도의 절대 상한을 검사하므로 frozen ROS에서 전체 대기 무한화로 연결되는 증거는 없었다. W7 fake-clock 회귀는 약 1.16 steady s 정상 phase와 2 steady s 정지 종료를 확인한다.
- L2/L3 cleanup은 사용자 cancel을 재적용하지 않고 tracker의 10/15 s 독립 cap을 적용한다. 이전 action terminal이 미확정이면 cleanup motion을 보내지 않는다. REQ-ORC-010의 비하드 latch 자체는 `_fault_lock` 및 released 처리로 유지되나 N1의 deadline 경로는 이를 우회한다.
- `_send_lock` RLock은 late-accept callback과 cancel의 handle 설치를 직렬화한다. reset은 미확정 motion을 지우지 않으며 feedback token을 갱신한다. 이 검토에서 추가적인 구체적 중복 motion 재현은 없었다.
- supervisor는 진행 중 RPC가 있으면 장애 복구/변경 여부와 무관하게 먼저 해당 응답을 회수한다. 응답 미확인 시 겹쳐 보내지 않는 정책은 명시적이다. 성공 뒤 동일 장애에 재호출하지 않으며 새로운 사유는 새 확인 구간으로 다룬다. permanent pending의 경고를 실제 Stop 성공으로 오인해서는 안 된다.
- logger는 상태 snapshot과 append를 lock 안에서 수행하지만 I/O가 장시간 정지하면 해당 logger callback들은 대기한다. 자동 replay·durable queue·step_seq 재정렬은 이번 수정 계약 밖이며 시험 기록 수와 실제 산출물을 대조해야 한다. storage_error는 node 내부 속성과 warning으로 노출되고 ROS diagnostic publish는 없다.

## F4 이관·M0 잔여 위험

SPEC-110 `:20`의 소유자와 실제 경로 사용 전 C/I 필수 문구, `:111`의 개별 시험 매핑은 TEST_SPEC `:297`~300의 입력·기대 및 `:527`~537 게이트 표와 일치한다:

- G0: ABT-014-02 그리퍼 오류·응답 지연 ROS 입력.
- G3: ABT-011-02 관측 freshness, ABT-012-02 역행 epoch/track/plan 무효화, ABT-013-02 벨트 이벤트.
- G4: ABT-011-02 재확인.
- M0: ABT-011~014 U 판정·이벤트 입력만 포함하고 위 -02 제외.

그러므로 M0 mock 완료를 해당 ROS 배선이나 실장비 안전성 완료라고 보고할 수 없다. 현재 노드에서 heartbeat 정상인 그리퍼 이상·불안정 벨트·오래된 관측이 PRECHECK 뒤 생기면 지정 이벤트로 중단시키지 못하는 위험은 그대로다. production 경로·현장 진입 전 위 게이트 차단이 실제로 집행되어야 한다. 게이트 이관 문서 자체 때문에 추가 불합격을 주지는 않으며 N1은 M0 내부 계약 문제라 이 이관으로 면제되지 않는다.

## 검증·증거

GCE, 저장소 루트:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/orchestrator:tools/spec python3 -m pytest -q -p no:cacheprovider src/orchestrator/test --ignore=src/orchestrator/test/test_safety_supervisor.py --ignore=src/orchestrator/test/test_record_logger.py
```

**617 passed, 2 skipped, 2.54 s**. skip은 ROS 의존 port/timer 파일이며 ignore한 supervisor/logger ROS 시험은 실행하지 않았다. 같은 환경의 `test_node_failure_regressions.py` 별도 실행은 **20 passed, 0.36 s**(위 617에 포함되어 중복 가산하지 않음). 이 파일은 현재 production AST를 추출하므로 원 지적의 ROS-free 시나리오 재실행 근거가 된다. N1 두 추가 재현은 Python stdin으로 실행했으며 저장소에 테스트 파일을 추가하지 않았다. trace.py는 생성물 변경 방지를 위해 실행하지 않았다.

PC 근거는 W7 `codex-ros-fixes-2026-11-09.md` 및 저장소 `records/worklog/2026-11-09.md:91`~95를 읽었다. 보고된 PC WSL 전체 **861 passed**, nominal **8/8·160/160 SUCCESS**는 W7의 기록에 근거하며 W8 재실행 결과가 아니다. 정상 nominal 성공은 deadline을 실제 넘긴 release 이후 cleanup이나 late-accept·Stop 복구 transport 경합을 직접 검증하지 않는다. W8은 PC·CI·실장비를 접속/실행하지 않았다.

검토 소스 SHA-256:

| 파일 | SHA-256 |
|---|---|
| orchestrator_node.py | `f762119fbf5a27b90a3d00c3d8d28c76ea1f0f310b158c1685ed7e54f3f895e1` |
| safety_supervisor_node.py | `25d2acbfc8895714ac0ed19a49db65878b189ee9a065beec29c8a76638b0c56f` |
| record_logger_node.py | `804f724a7940c4fb1a574c933c11f0e7695e9ad35c9894aaf41ddb1de7928911` |
| cycle_runner.py | `a1ca2dbba176be23959d1d83960d2ddf9e5a59aa0f4d08bff0eb16107fe16c04` |
| test_node_failure_regressions.py | `588640fb1bc0ca2e508e643236c7ab773759523cfb9dd350c2b34fc2b728a652` |

## 최종 판정과 재검수 조건

**불통과**. 통과 전 필수는 N1 수정·실제 clock/deadline 경계를 넘기는 회귀·독립 재확인이다. F4는 위 G0/G3/G4 C/I 필수 게이트, F8은 schema가 존재하는 저장소/symlink 운용 범위를 유지하거나 정상 install 패키징을 완료해야 한다. F5는 자동 재전달 없는 warning 기반 복구 계약 및 부분 쓰기 한계를 명시해 운용한다. 실제 ROS late-accept, Stop 지연/거부/응답 영구 누락, frozen-clock 포트, SIGINT, logger 저장 오류 주입은 일반 nominal 통과와 별도로 추적할 검증 한계다.

## N1 최소 재현 명령

저장소 루트에서 실행하며 파일을 쓰지 않는다. ROS 없는 기존 AST loader를 재사용하되 실행 대상은 현재 production 클래스다.

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/orchestrator:src/orchestrator/test:tools/spec python3 - <<'PYREPRO'
from test_node_failure_regressions import port
from orchestrator.cycle_runner import CycleRunner, OrchestratorCfg
from orchestrator.common import State
from orchestrator.abort_policy import Event
from types import SimpleNamespace as NS
import yaml
cfg = OrchestratorCfg.from_dict(yaml.safe_load(open('src/orchestrator/config/orchestrator.yaml')))
p = port()
p.clock.ros_now = lambda: 91.
p._released = True
p.raise_fault(Event.CYCLE_TIMEOUT)
p._feedback = [(90., NS(phase='RETREAT', phase_done=True, metric_keys=[], metric_values=[]))]
p._result_future = NS(done=lambda: True, result=lambda: NS(result=NS(result=0)))
c = NS(cap=None, deadline=90., released=True)
r = CycleRunner(p, p.clock, cfg)
d = r._deadline(c, State.RETREAT)
print(p._interrupting(), d, p._wait(lambda: True, d), r._do_retreat(c))
PYREPRO
```
