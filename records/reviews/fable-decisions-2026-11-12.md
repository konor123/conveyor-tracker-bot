# 남은 결정 사항 자문과 판단 (2026-11-12)

- 자문: Claude Fable 5.2(읽기 전용, 코드 읽기 근거, 시험·실기 실행 없음). 판단: coordinator(Claude Opus 5.5). 사용자가 결정 권한을 coordinator에 위임했다(2026-11-12 22:50 KST).
- 대상: DynamicPlace 구조, `BeltMotion.steady=false` 연결, Stop 해제 직후 goal 경합, Doosan 실제 backend, 도달 구간 설정 일치, I0 간헐 `STEP_TIMEOUT`.

## 판단

| 항목 | 채택 | 메모 |
|---|---|---|
| (a) DynamicPlace 구조 | 자문안 채택 | 순수 순서기 + 해제 게이트 + "해제 확정" 수준 transport 경계 + 가짜 backend. Doosan speedl transport는 실기 미검증으로 두고 노드 backend에 연결하지 않는다 |
| (b) `steady=false` | 자문안 채택 | 1.0 s(`belt_unsteady_hold_s`) 넘게 이어질 때만 `BELT_UNSTEADY`. 그보다 짧으면 해제 게이트만 닫는다. `BeltSynchronizer`는 `UNCERTAIN` 계획을 래치하지 않고 0.6 s(`uncertain_hold_s`) 동안 마지막 OK 계획을 따른다. PRECHECK도 같은 지속 규칙을 쓴다 |
| (c) Stop 해제 직후 goal 경합 | 자문안 채택 | goal 수락 뒤 새 controller heartbeat를 받은 다음 장애 래치를 무장한다. 새 heartbeat가 여전히 101이면 그대로 L4다. I0 시험의 기동 안정 대기는 `error_code` 0·`ready`를 확인한다 |
| (d) Doosan 실제 backend | 자문안 채택 | 주입형 client 어댑터와 가짜 dsr 서비스 시험까지. `DoosanClient`를 `robot_control`로 옮기는 정리는 11-14 현장 뒤로 미룬다 |
| (e) 도달 구간 설정 | 자문안 채택 | launch에서 세 노드(tracker·planner·orchestrator)에 같은 값을 주입하고 일치 시험을 둔다 |
| (f) I0 간헐 `STEP_TIMEOUT` | 자문안 채택 | 재현 전에는 callback group을 바꾸지 않는다 |

## 자문 원문

아래는 여섯 가지 결정에 대한 권고입니다. 전부 코드 읽기에 근거한 것이고, 시험이나 실기 실행은 하지 않았습니다. 표기는 [코드] = 소스 확인, [추론] = 근거 있는 추정, [미확인]입니다. 수치는 모두 제안값이며 OFF-2·OFF-5에서 확정해야 합니다.

## (a) DynamicPlace 구조

**권고: 지금 구현하되, 경계(seam)는 속도 지령이 아니라 "해제 확정(commit)" 수준에 둡니다.**

- **이유.** 단일 DRL 대안은 프로그램 시작 뒤 PC가 끼어들 수 없습니다(`docs/learn/2026-11-10-fast-static-cycle.md:43,68`). 경계를 `send_velocity()` 수준에 두면 스트림 방식만 맞고 대안으로 바꿀 수 없습니다.
- **순수 모듈 `robot_control/dynamic_place.py`.**
  - 단계 순서기: APPROACH → SYNCHRONIZE → DESCEND → RELEASE → RETREAT, 주입한 시계로 주기 실행.
  - 순수 함수 `release_gate(입력, 한도, horizon_s)`가 (ok, 사유, `gate_*` 지표)를 돌려줍니다.
  - XY는 기존 `BeltSynchronizer`를 그대로 씁니다.
- **transport 계약(Protocol).**
  - `capabilities`: `mid_course_update`, `commit_lead_s`, `keep_sync_retreat`.
  - 메서드: `update(plan)`, `commit_release(permit)`, `poll()`(TCP·시각·그리퍼 폭·released·fault), `abort(keep_sync)`.
  - 스트림 + 상주 루프: `commit_lead_s` ≈ 트리거 지연. commit은 `GripperSession.fire_open`입니다.
  - 단일 DRL: `commit_lead_s` = 하강 + 개방 시간. commit은 프로그램 시작이고 인자는 해제 시각·자세·속도입니다.
- **REQ-ROB-009.** 게이트는 commit 직전에 한 번, `commit_lead_s` 앞 예측값으로 판정합니다. 위반이면 commit 0회 + L3 요청입니다. 두 transport가 같은 함수를 쓰고 horizon만 다릅니다.
  - 단일 DRL에서는 horizon이 길어 0.5 mm 기준을 못 맞출 가능성이 큽니다 [추론].
  - 그 경우 `PLAN.md:905`대로 PREDICTED 수준으로 내려간다는 것을 SPEC-500에 적어 두십시오.
- **REQ-ROB-010.** `keep_sync` 후퇴는 순서기가 `poll()` 표본으로 XY 속도를 재서 ±1 mm/s를 판정합니다. `capabilities.keep_sync_retreat=false`인 경로는 성공으로 보고하지 않습니다.
- **가짜 backend.** 1차 지연(지연 29 ms, `Progress.md:103`) + 개방 지연 파라미터를 둡니다.
  - 시험: 게이트 항목별 위반 시 open 0회, 단계별 cancel·Stop·deadline, 개방 1회, feedback 키 계약(`mock_robot_controller_node.py:30`, `tools/testing/contracts/robot_controller_contract.py:42-49`).
- **Doosan transport.** "현장에서 처음 코드를 짜지 않는다" 규칙상 두 후보 모두 코드가 있어야 합니다.
  - speedl 발행기: 호출 형식은 `tools/bench/stream_bench.py:249`, `FreshnessWatchdog`를 붙입니다.
  - 단일 DRL 렌더러: `test/fakes/drl_env.py`로 본문을 실행해 시험합니다.
  - 둘 다 "실기 미검증"으로 표기하고, OFF-2 전에는 노드 `backend`에 연결하지 않습니다.
- **명세 변경.** SPEC-500 REQ-ROB-008·009·010 구체화, TC-ROB-008~010 추가.
- **위험.** 실기 TCP 피드백의 주기·나이(`max_tcp_age_s` 0.25 s, `belt_sync.py`)가 미측정입니다. OFF-2 측정 항목에 넣으십시오.

## (b) `steady=false` 연결

**권고: 한 번의 `steady=false`로는 중단하지 않습니다. 지속 시간으로만 중단하고, 그 사이는 "해제 게이트 닫힘 + 추종 유지"로 처리합니다.**

- **단순 연결의 비용.** 경보 11~18회/60 s, 동적 구간 3~4 s면 사이클의 약 45~70%가 중단됩니다(포아송 추정 [추론]).
- **더 급한 문제: 연결하지 않아도 지금 구조에서 경보가 사이클을 끊습니다 [코드+추론].**
  - 경보는 속도 공분산을 (0.05 m/s)²로 되돌립니다(`SPEC-300-tracking.md:88`).
  - 그러면 해제 시각 예측 std ≈ 0.05 × horizon이 되어, horizon이 65 ms만 넘어도 3.25 mm를 넘습니다.
  - planner는 track 메시지마다 revision을 내고(`intercept_planner_node.py:153`), 이때 `UNCERTAIN`(`intercept.py:164`)이 됩니다.
  - `BeltSynchronizer.accept`는 `status≠0`이면 `PLAN_INVALID`로 래치합니다(`belt_sync.py:118,127`).
- **동기화기 수정.** `UNCERTAIN` revision은 래치하지 않고 버립니다. 마지막 OK 계획을 `uncertain_hold_s`(제안 0.6 s = `recover_s` 0.5 + 여유) 동안 등속 외삽으로 따라갑니다.
  - 현재 `max_plan_age_s` 0.25 s는 재수렴 시간 약 0.3 s(`SPEC-300-tracking.md:50`)보다 짧아, 버리기만 하면 `PLAN_STALE`(`belt_sync.py:148`)로 끊깁니다.
  - 따라서 별도의 hold 상한이 필요하고, 그 상한을 넘으면 정지 → L3입니다.
  - REQ-ROB-013과 TC-ROB-013에 반영합니다.
- **모드별 동작.**
  - SYNCHRONIZED: 게이트가 다시 열릴 때까지 `sync.timeout_s`·`t_available` 안에서 기다립니다. 경보 한 번의 비용은 약 0.5 s 지연입니다.
  - PREDICTED: 해제 시각이 고정이라 기다릴 수 없습니다. 해제 시각에 게이트가 닫혀 있으면 열지 않고 L3입니다.
  - 두 모드 모두 따로 "속도 편차 Y" 규칙은 두지 않습니다. revision + 게이트의 `e_xy`·std가 그 역할을 합니다.
- **REQ-ABT-013 문구 제안.** "`steady=false`가 `belt_unsteady_hold_s`(제안 1.0 s = 2 × `recover_s`) 넘게 이어지면 `BELT_UNSTEADY`: 해제 전 L3, 해제 후 `CONTINUE_TO_RETREAT_VERIFY`. 그보다 짧으면 중단하지 않고 그동안 해제 게이트는 닫힌다."
- **시험.**
  - TC-ABT-013-01: 경계 1.000 / 1.001 s를 추가합니다.
  - TC-ABT-013-02(C): mock에서 0.5 s 지속이면 중단 없음·해제 지연, 2.0 s(기존 `unsteady_hold_s`)면 L3, 해제 후는 계속.
  - `SPEC-300-tracking.md:124`의 "동적 단계면 L3" 문장도 같이 고칩니다.
- **부수 항목.** PRECHECK의 `BELT_UNSTEADY`(`step_judges.py:95`)는 맥동 경보만으로 시작의 약 5%를 `NOT_ATTEMPTED`로 만듭니다 [추론]. PRECHECK에서 빼거나(WAIT_CAP이 이미 보류) 짧게 재시도하게 하십시오.
- **위험.** 20 mm/s 미만 변화와 감속은 원래 검출되지 않습니다(`SPEC-300-tracking.md:154`). `steady`는 안전망이 아니고 최종 방어는 게이트입니다. 임계값은 OFF-5 다이얼 조작 bag으로 확정해야 합니다.

## (c) Stop 해제 직후 goal 경합

**권고: orchestrator에서 고치고, 시험 절차도 함께 보강합니다.**

- **원인 [코드 추론, CI 로그로 확정한 것은 아님].**
  - `_run_cycle`이 `port.reset(goal_handle)`(`orchestrator_node.py:729`)로 장애 래치를 먼저 무장합니다.
  - `_watch`는 캐시된 `hb_error_code`가 101이면 `ROBOT_STOPPED`를 래치합니다(`:676`, `pending_fault`가 `:152`에서 호출).
  - heartbeat는 10 Hz라 캐시가 최대 0.1 s 낡습니다. 해제 0.03 s 뒤에 시작한 goal은 낡은 101을 봅니다.
- **부작용.** 이 L4는 `port.stop()`(`cycle_runner.py:525`)을 불러, 방금 해제한 controller를 다시 정지 유지로 만듭니다.
- **최소 변경.**
  - `_on_hb`에 controller heartbeat 수신 횟수(또는 stamp)를 저장합니다.
  - `_run_cycle`의 시작 대기 조건(`:732`)에 "goal 수락 뒤 받은 controller heartbeat 2개(또는 stamp ≥ 수락 시각)와 새 `RobotStatus`"를 추가합니다.
  - `port.reset`을 그 대기 뒤로 옮깁니다. 대기는 평소 0.2 s 이하, 상한은 기존 `startup_wait_s`입니다.
- **지켜야 할 것.** 101을 무시하는 것이 아닙니다. 새 heartbeat가 여전히 101이면 PRECHECK에서 L4여야 합니다(REQ-ORC-012, `test_i0_integration.py:611`). 이 시험은 그대로 통과해야 합니다.
- **시험 절차.** `_settle`(`test_i0_integration.py:108-121`)은 해제 뒤 `error_code == 0`과 `ready = true`를 본 다음 반환하게 합니다. 운용 절차(`Progress.md:141`)와 같은 순서입니다.
- **controller 쪽.** 해제 즉시 heartbeat를 발행해도 전송 지연이 남아 경합이 닫히지 않으므로 하지 않습니다.
- **명세·시험.** REQ-ORC-012에 한 문장 추가, 단위 시험 2건(낡은 101 → 새 0이면 진행, 새 101이면 L4).
- **남는 것.** 기동 약 3.2 s 뒤 Stop 자체의 원인은 여전히 미확인입니다.

## (d) `backend:=doosan`

**권고: `DoosanClient`를 import해 바로 연결하지 않습니다. 11-14 이후로 미루고, 지금은 어댑터와 가짜 서버만 만듭니다.**

- **막는 이유 세 가지 [코드].**
  - 패키지 순환: `static_pick_place/package.xml:14`가 `robot_control`에 의존하고, `doosan_client.py:13-14`가 `robot_control`을 import합니다.
  - `_call`이 `rclpy.spin_until_future_complete(self.node, …)`(`doosan_client.py:126`)를 씁니다. 이미 `MultiThreadedExecutor`가 돌리는 lifecycle 노드의 action 콜백 안에서 부르면 동작이 보장되지 않습니다 [추론].
  - `move_line`이 동기 블로킹이고 정지·취소 경로가 없습니다. REQ-ROB-007(cancel 0.5 s, 확인된 Stop)을 만족하지 못합니다.
- **빠진 부분이 곧 미검증 부분입니다.** 비동기 이동 + `move_stop` + 정지 확인은 현장에서 한 번도 검증되지 않았습니다.
- **지금 할 일.**
  - `robot_control/doosan_backend.py`: 주입받는 좁은 client Protocol → `StaticBackend` 어댑터. 전용 helper 노드와 executor 스레드를 씁니다.
  - 가짜 client로 단위 시험: 측정값 판정, cancel·Stop·deadline, DRL PLAY 중 이동 금지.
- **CI 제약.** CI에 `dsr_msgs2`가 없습니다(`ci.yml`에 설치 단계 없음, 기존 시험은 `sys.modules` 스텁: `test_doosan_client.py:18-23`). 가짜 dsr 서비스 서버로 `robot_controller_contract.py`를 돌리는 C 시험은 `needs_data`류 표시를 달고 PC(LEGION·officepc)에서 판정하십시오(`Progress.md:133-134`).
- **11-14 이후.** `DoosanClient`를 `robot_control`로 내리고 `static_pick_place`가 재수출하게 하는 정리를 사용자 승인 뒤 진행합니다.
- **위험.** 미루면 OFF-6에서 orchestrator 경유 pick → standby가 비게 됩니다. 그 전에는 `static_pick_place`로 파지한 뒤 DynamicPlace만 시험하는 절차로 대체할 수 있습니다.

## (e) 설정 일관성

**권고: launch 주입으로 단일 출처를 만들고, 일관성 시험을 함께 둡니다.**

- **mock 세계의 올바른 값은 0.20 / 0.40입니다 [코드].** 시나리오 zones가 기준이고(`nominal_20mms.yaml:18-19`), planner의 0.30은 launch가 0.40으로 덮어씁니다(`drop.launch.py:63-66`). mock에서는 지금 일치합니다.
- **실제 불일치는 두 곳입니다.**
  - 전부 실제 노드일 때: orchestrator 0.20 / 0.40 대 tracker·planner −0.05 / 0.30(`tracking.yaml:23`, `planner.yaml:4,14`). WAIT_CAP `TOO_EARLY`가 풀릴 때쯤 planner 기준으로는 이미 구간 밖이라 계획이 서지 않습니다 [추론].
  - zones가 다른 시나리오: `time_insufficient.yaml:12`는 `reach_start` 0.35인데 orchestrator는 0.20 고정입니다.
- **변경.**
  - orchestrator에 `budget.x_reach_start_m`·`budget.x_exit_m` ROS 파라미터를 추가합니다(미설정이면 yaml 값).
  - `_scenario_params`가 orchestrator용 값도 돌려주게 합니다.
  - 실제 배치용으로 `drop_bringup/config/system.yaml`(현재 빈 `{}`)에 배치 값을 한 번만 적고 세 노드에 주입합니다.
  - `test_real_bringup.py:81-86`의 파라미터 비교에 orchestrator를 추가합니다.
- **위험.** 실제 배치 값은 OFF-1·OFF-3 TBD입니다. 그때까지는 구조만 맞추는 작업입니다.

## (f) I0 간헐 `STEP_TIMEOUT`

**권고: 원인 대책으로는 옮기지 않습니다. 재발을 기다리고 Progress 항목은 그대로 둡니다.**

- **옮겨도 효과가 작습니다.** action client는 기본 그룹을 씁니다(`orchestrator_node.py:614`). 같이 쓰는 콜백은 setattr 수준이라, 유력 경로인 0.65 s 지연을 설명하지 못합니다 [추론].
- **"result 선처리"는 그룹과 무관합니다.** rclpy `ActionClient.execute`는 한 번 깨어날 때 result를 feedback보다 먼저 처리하는 것으로 기억합니다 [미확인: GCE에 rclpy가 없어 officepc에서 소스 확인 필요]. 이 경로는 이미 `FEEDBACK_DRAIN_S`(`:87`)가 막고 있습니다.
- **비용이 있습니다.** 재현 없이 동시성 구조를 바꾸면 새 진단(`377eb77`)의 해석이 흐려집니다.
- **옮기게 될 경우.** (c) 수정과 같은 I0 반복으로 검증하고, feedback 순서를 지키기 위해 Reentrant가 아닌 전용 MutuallyExclusive 그룹을 쓰십시오.

## 현장 전 오프라인 작업 우선순위

1. **(b) 동기화기 hold + REQ-ABT-013 개정.** 고치지 않으면 OFF-5·OFF-6에서 맥동 경보만으로 추종이 래치됩니다. 순수 Python이라 GCE에서 끝납니다.
2. **(a) 순서기 + `release_gate` + 가짜 backend.** 이어서 speedl 발행기와 단일 DRL 렌더러. OFF-2 결과가 나오면 바로 갈아 끼울 수 있습니다.
3. **(c) 새 heartbeat 대기 + `_settle` 보강.** 작은 변경이고 CI 간헐 실패 1건을 없앱니다.
4. **(e) launch 주입 + 일관성 시험.** 실제 노드 조합에서 계획이 안 서는 것을 현장 전에 막습니다.
5. **OFF-2 측정 절차서.** 다음 항목을 순서대로 적어 둡니다.
   - 상주 루프와 speedl 동시 동작.
   - TCP 피드백 주기·나이.
   - `move_stop` 정지 확인.
   - 기동 3.2 s 뒤 Stop 재현 여부.
   - LEGION 시계 계단.
6. **(d) Doosan 어댑터와 가짜 dsr 서버.** 11-14 이후, OFF-6 전에.
7. **(f)는 보류.** officepc에서 rclpy `execute` 순서만 확인해 기록합니다.