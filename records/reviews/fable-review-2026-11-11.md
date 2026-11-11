# conveyor-tracker-bot 커밋 검토 (`ec5df7f..b478556`, coordinator 수정 커밋)

- 검토자: Claude Code worker (Fable 5.2), 실행 장소 GCE, 읽기 전용
- 대상: 58582b5, 2b3bfe8, 44d113f, 306dd40, c2e60e5 (코드·명세) / 3264b55, 4088b77, b478556 (기록)
- 건수: **높음 0, 중간 4, 낮음 12**
- 저장소 작업 트리는 바꾸지 않았다(`git status --short` 빈 출력). 변이 실험은 scratchpad 사본(`/tmp/claude-1000/.../scratchpad/copy`)에서만 했다.

## 검증 방법

| 항목 | 결과 |
|---|---|
| `src/robot_control/test` + `src/static_pick_place/test` (저장소) | 118 passed, 2 skipped, 1 failed (`test_config`, cv2 없음, 기존 사항) |
| `tools` | 138 passed, 6 skipped |
| `tools/spec/trace.py --gate M0 --check` | REQ 50 / TC 94 / 누락 0 |
| 변이 실험 18건 (사본) | 사본을 고치지 않고 돌린 기준선은 2 failed, 117 passed(둘 다 `test_config`. 하나는 cv2, 다른 하나는 사본에 저장소 루트가 없어서로 추정). 변이 뒤 2 failed 그대로면 "살아남음" |
| 동작 실험 (가짜 제어기 `fakes/resident.py`, `_FakeRobot`) | 아래 각 발견에 적음 |

"확인함"은 코드 읽기와 가짜 제어기 실험으로 재현한 것이다. 실기 동작은 어느 것도 확인하지 않았다.

---

## 중간

### M-1. `DRL_NOT_STARTED`가 프로그램이 실제로 도는 중에도 나고, PC는 프로그램을 멈추지 않고 빠진다 (58582b5)

- 위치: `src/static_pick_place/static_pick_place/doosan_client.py:268-276`
- 문제: `DRL_NOT_STARTED` 판정이 `get_drl_state`만 보고, 레지스터 14·15가 보여 주는 "프로그램이 돌고 있다"는 증거(15가 0으로 지워짐, 14가 증가)를 쓰지 않는다.
- 시나리오(확인함, `_FakeRobot`): 레지스터 15 `250 → 0`, 14가 `1 → 2 → 3`으로 진행, `drl_state`는 계속 STOP(1) → 2.0 s에 `DRL_NOT_STARTED`. 그 시점까지 `open_before_pick`·`over_stone`을 "done"으로 이미 보고했다. `pick_place`는 `abort`를 기록하고 종료 코드 2로 끝난다. `drl_stop` 호출은 0회(`src` 전체에 `drl_stop` 사용 없음)이므로 제어기 프로그램은 남은 이동·그리퍼 단계를 끝까지 실행한다.
- 영향: PC는 "중단"이라고 보고했는데 팔은 계속 움직인다. 수정 전에는 이 경우 PC가 끝까지 감시했다.
- 확신도: 논리는 확인함. 실기에서 `drl_start` 뒤 2 s 넘게 PLAY가 안 보이는 일이 있는지는 **가능성**(worklog도 유예 2 s를 미검증으로 적음). `gripper()`의 2026-11-09 주석은 상태가 실시간으로 PLAY를 보인다고 하므로 가능성은 낮다고 본다.
- 제안: 실행 중 레지스터 15가 0으로 바뀌었거나 14가 **변한 것**을 보면 시작한 것으로 본다(`progress >= 1`만으로는 부족하다, L-12 참고). 중단 경로에서 프로그램이 아직 돌 수 있으면 `drl_stop`을 보내거나, 최소한 abort 로그에 "프로그램이 계속 돌 수 있음"을 남긴다.

### M-2. 지령 실행 전의 표본으로 완료를 판정해서, 루프 쓰기 실패가 성공으로 보고되고 다음 지령에서 터진다 (306dd40의 주장과 745ead1의 판정 논리)

- 위치: `src/robot_control/robot_control/gripper_adapter.py:623`(`_wait_position`), `:648-654`(`_wait_grasp`), `src/robot_control/robot_control/resident_transport.py:142-149`(`_check_command`)
- 문제: 성공 판정이 "레지스터 39 == 트리거한 seq"를 기다리지 않는다. `_check_command`는 실패를 볼 때만 쓰이고, 실행 전에 성공 조건이 맞으면 결과 40을 보지 않고 돌아온다.
- 시나리오 1(확인함): 공깃돌을 이미 쥔 상태(659 pulse, 전류 ≥ 한도의 80%)에서 `fail_writes=True`로 `session.grasp(100)` → 0.6 s 만에 `STALLED`·`holding=True` 반환, 그리퍼가 받은 목표 0회. 3 s 뒤 레지스터 39/40 = 2/242. 이어서 정상 상태로 보낸 다음 `grasp`가 `RESIDENT_WRITE_FAILED: seq=2`로 실패한다(실제로는 앞 지령의 실패).
- 시나리오 2(확인함): 그리퍼가 이미 600 pulse일 때 `fail_writes=True`로 `adapter.open(600 pulse 폭)` → 0.05 s 만에 `OK`. 실패가 없을 때도 같은 호출은 루프가 지령을 실행하기 전(`done_seq=0`, `last_seq=1`)에 `OK`를 돌려준다.
- 영향: (a) 전류 한도를 바꾼 재파지처럼 "이미 그 위치"인 지령의 쓰기 실패가 성공으로 나간다. (b) REQ-GRP-016의 "앞선 seq의 결과는 다음 지령에 영향을 주지 않는다"(`specs/SPEC-500-robot-control.md:79`)와 `docs/learn/2026-11-11-drl-stop-without-result.md:31`의 설명이 이 경우 성립하지 않는다. `open()`·`close()`가 트리거 전에 `_read_present_pulse()`로 `_check_command`를 부르기 때문이다.
- 책임 구분: 판정 논리는 팀원 커밋 745ead1에서 왔다. 306dd40은 실패 통로를 추가했지만 성공 경로가 그 통로를 확인하지 않는다.
- 확신도: 확인함(가짜 제어기). 실기 미확인.
- 등급 근거: 재현한 두 경우 모두 그리퍼의 실제 상태(쥐고 있음, 이미 열림)는 보고된 상태와 같아서 높음이 아니라 중간으로 두었다. 전류 한도 변경이 조용히 빠지는 점과 다음 지령이 잘못 실패하는 점을 더 무겁게 보면 올릴 수 있다.
- 제안: 성공을 돌려주기 전에 `done_seq == last_seq`이고 결과 0임을 요구하고, 그 뒤 올라온 새 표본으로만 도달·막힘을 판정한다.

### M-3. 306dd40의 DRL 논리는 시험이 문자열 포함 여부만 본다

- 위치: `src/robot_control/test/test_resident_transport.py:106`, `src/robot_control/test/fakes/resident.py:130-139`
- 문제: `gripper_resident.drl`을 실행하는 시험이 없다. 가짜 제어기가 같은 동작을 따로 구현하므로 DRL 본문을 틀리게 바꿔도 시험이 통과한다.
- 변이 결과(확인함, 셋 다 살아남음):
  - 39를 40보다 먼저 쓰도록 순서를 바꿈(`gripper_resident.drl:114-115`).
  - 전류 쓰기 실패 뒤에도 위치를 쓰게 함(`:111`).
  - 전류 쓰기 실패의 결과를 0으로 바꿈(`:110`).
- 영향: REQ-GRP-016의 "39보다 먼저 씀", "전류 쓰기에 실패하면 위치를 쓰지 않는다"는 문장을 검증하는 시험이 없다. TC-GRP-016-01의 "레지스터 번호 일치(결과 40 포함)"만 실제로 확인된다.
- 참고: `test_cycle_drl.py`의 `FakeController.run`은 `cycle.drl` 본문을 `exec`으로 실제 실행한다. 같은 방식을 상주 루프에도 쓸 수 있다(루프 종료는 레지스터 25로).
- 추가로 살아남은 변이: `present_current` 읽기에서 `_check_command`를 빼도 통과(`resident_transport.py:112-113`). 명세의 "위치·전류 읽기" 중 전류 쪽은 미검증.

### M-4. 제한시간 검사가 통과시키는 경계값으로는 정상 지령도 끝나지 않는다 (c2e60e5)

- 위치: `src/robot_control/robot_control/gripper_adapter.py:658-671`, `src/robot_control/test/test_resident_transport.py:131`, `TEST_SPEC.md:421`
- 문제: 검사 기준(응답 ≥ 3주기, 생존 ≥ 2주기)은 필요조건일 뿐인데, docstring은 "열기·닫기 1회 = 목표 쓰기 + 확인 표본"이라며 충분한 것처럼 설명하고, TC-GRP-016-04는 경계값에서 생성만 확인한다.
- 시나리오(확인함, 가짜 제어기 왕복 0.58 s, 세션 시작 뒤 첫 파지 `grasp(100)` = 전류 쓰기 포함, 트리거 위상 0·0.2·0.4 s 모두 같은 결과):

  | 응답 제한 | 생존 제한 | 결과 |
  |---|---|---|
  | 1.74 s (3주기) | 3.0 s | `GRIPPER_TIMEOUT: 1.750 s` |
  | 8.0 s | 1.16 s (2주기) | `GRIPPER_LOOP_DEAD: 1.200 s` |
  | 8.0 s | 1.74 s (3주기) | `STALLED` 정상 |
  | 3.0 s | 3.0 s | `STALLED` 정상 |

- 원인: 루프는 지령 실행 중(전류 쓰기 + 위치 쓰기) 생존 카운터를 올리지 않는다(`gripper_resident.drl:103-117`, 카운터는 `:128`·`:137`에서만 증가). 전류가 바뀌는 지령은 쓰기 2회 + 상태 읽기 1회 = 3주기 동안 카운터가 멈춘다. REQ-GRP-013의 "2배 주기" 기준을 상주 루프에 그대로 쓰면 전류가 바뀌는 지령(세션 첫 지령 포함)에서 `GRIPPER_LOOP_DEAD`가 난다. 그리퍼에 같은 전류가 이미 쓰여 있는 지령은 생존 1.16 s에서도 정상 완료됐다(확인함, 위상 3가지).
- 영향: 안전 쪽 실패(거짓 FAULT)다. worklog에 "판정이 불가능한 조합만 막는다"는 사용자 결정이 있으므로 코드 결함이라기보다, OFF-2 뒤 값을 정할 때 2주기·3주기를 하한으로 믿으면 안 된다는 점이 핵심이다.
- 확신도: 확인함(모델). 실기 주기는 미측정.
- 제안: 생존 하한을 3주기 이상으로 올리거나 루프가 쓰기 사이에도 카운터를 올리게 하고, docstring·REQ-GRP-016 문장을 "필요조건"으로 고친다.

---

## 낮음

### L-1. 실패 보고가 응답 제한시간보다 늦으면 다음 지령이 앞 지령의 실패를 받는다
- 위치: `resident_transport.py:142-149`
- 시나리오(확인함, 모델): 응답 제한 1.74 s, `fail_writes=True` → `GRIPPER_TIMEOUT`. 그 뒤 정상 상태의 다음 `grasp`가 `RESIDENT_WRITE_FAILED: seq=1`.
- 확신도: 실기에서는 **가능성**. 가짜 제어기는 실패에도 `(n_before+1)·rtt`를 쓰지만(`fakes/resident.py:142`) DRL은 전류 실패 시 위치 쓰기를 건너뛰고 `READ_TIMEOUT` 뒤 돌아오므로 실제 보고는 더 빠를 수 있다. 가짜와 DRL의 이 시간 차이 자체도 모델 불일치다.

### L-2. 실패 표지를 읽으면 STOP을 기다리지 않고 돌아온다 (기존, ed7e27a)
- 위치: `doosan_client.py:260-262`, docstring `:211`
- 시나리오(확인함): 표지 248, `drl_state`는 PLAY → `GRASP_EMPTY`를 즉시 올림. docstring은 "STOP이 된 뒤에만 돌아온다"고 한다. 성공 경로(`:254-257`)만 STOP을 기다린다.
- 영향: 현재 `pick_place`는 실패 시 바로 종료하므로 다음 지령은 없다. 재시도 논리를 붙이면 2026-11-09의 "종료 중 보낸 move_line은 움직이지 않고 끝난다" 문제가 다시 생긴다.

### L-3. `CYCLE_DRL_TIMEOUT`·`DRL_NOT_STOPPED`에서도 프로그램을 멈추지 않는다 (기존, ed7e27a)
- 위치: `doosan_client.py:256`, `:278`
- 시나리오(확인함): `drl_state` PLAY 유지, 제한시간 3 s → `CYCLE_DRL_TIMEOUT`, 정지 호출 0회. 제어기 프로그램은 계속 돈다. M-1과 같은 종류다.

### L-4. 시작 직후 바로 죽는 프로그램은 `DRL_STOPPED_NO_RESULT`가 아니라 2 s 뒤 `DRL_NOT_STARTED`로 나온다
- 위치: `doosan_client.py:266-276`
- 시나리오(가능성): `flange_serial_open` 예외처럼 수십 ms 안에 멈추면 PLAY를 한 번도 못 본다. 레지스터 15는 0으로 지워졌는데 이유는 "시작 안 함"으로 나온다. 실패로 닫히므로 안전하지만 현장 진단을 헷갈리게 한다. `read_timeout_s`가 0.01 s면 `init_gripper` 실패(241)도 0 표지를 못 보고 지나쳐 같은 이유로 나올 수 있다.

### L-5. belt_sync 시험이 이유 문자열 일부를 확인하지 않는다 (2b3bfe8)
- 위치: `src/robot_control/test/test_belt_sync.py:42`, `:78`
- 변이 결과(확인함, 살아남음): `PLAN_IDENTITY_CHANGED`를 다른 문자열로 바꿈 / 목표·TCP 영역 밖의 `WORKSPACE`를 다른 문자열로 바꿈 / 한 주기 예상 위치 검사(`belt_sync.py:144`)를 삭제.
- 영향: TEST_SPEC TC-ROB-013-04 기대값 `WORKSPACE`, REQ-ROB-013의 `PLAN_IDENTITY_CHANGED`·`PLAN_INVALID` 구분, "한 주기 예상 위치" 조건은 시험이 `not active`만 본다. TC-ROB-013-03의 "모두 비활성·속도 (0, 0)"도 `test_watchdogs` 4건에서만 속도를 확인한다.

### L-6. 같은 revision 시험이 부동소수 경계에 정확히 걸려 있다 (44d113f)
- 위치: `test_belt_sync.py:54`, `belt_sync.py:134`
- 내용: `reasons[25]`는 `10.0 + 25·0.01 − 10.0 <= 0.25`가 참이어야 통과한다. 지금은 정확히 0.25가 나와 통과하지만(확인함), `PLAN_STALE` 비교에는 `LOOP_GAP`의 `1e-12` 같은 여유가 없다. 시작 시각이나 주기를 바꾸면 깨질 수 있다. 한 칸 안쪽(24, 27)으로 옮기면 같은 것을 검증한다.

### L-7. 한 프로그램 사이클의 새 실패 이유에 REQ·TC가 없다 (58582b5)
- 위치: `src/static_pick_place/test/test_cycle_drl.py:286-326`
- 내용(확인함): `DRL_STOPPED_NO_RESULT`·`DRL_NOT_STARTED`는 `specs/`와 `TEST_SPEC.md` 어디에도 없다. 새 시험 4개는 `level('U')`만 달았고 이 파일 전체에 `req`·`tc` 표지가 0개다. AGENTS.md의 "REQ → TC → 시험" 순서와 표지 규칙에서 벗어난다(ed7e27a부터 이어진 상태).

### L-8. 이번에 고친 시험 폴더는 CI가 돌리지 않는다
- 위치: `.github/workflows/*.yml`
- 내용(확인함): 워크플로에 `src/robot_control/test`·`src/static_pick_place/test` 실행 단계가 없다. 기록의 "CI success"는 이 수정들의 시험 통과를 뜻하지 않는다(worklog 첫 절에 적혀 있음). GCE 수동 실행이 유일한 근거다.

### L-9. 명세 문장에 진행 상태 표현이 들어 있다 (c2e60e5)
- 위치: `specs/SPEC-500-robot-control.md:79`
- 내용: "OFF-2 실측으로 교체", "상주 루프용 제한시간 값은 OFF-2 실측 뒤 정한다"는 AGENTS.md의 "최종 내용만" 규칙 경계에 있다(판단, 위반으로 단정하지 않음). 또 `TEST_SPEC.md:421`의 TC-GRP-016-04가 016-02 앞에, `:386`의 TC-ROB-013이 012 앞에 놓여 번호 순서가 어긋난다.

### L-10. 학습 문서의 설명이 M-2 경우에 맞지 않는다 (4088b77)
- 위치: `docs/learn/2026-11-11-drl-stop-without-result.md:31`
- 내용: "앞 지령의 실패가 다음 지령까지 막지 않게"는 M-2·L-1 시나리오에서 반대로 동작한다.

### L-11. 루프를 다시 띄우면 실행 안 된 마지막 seq가 "실행됨·정상"으로 보인다
- 위치: `gripper_resident.drl:96-97`, `:142`
- 내용(코드 읽기, 가능성): 기동 때 39 = 레지스터 10, 40 = 0으로 쓴다. 트리거 뒤 루프가 죽어 다시 띄우면 39 == `_last_seq`, 결과 0이 된다. 지금 transport는 실패만 보므로 해가 없지만, M-2 제안대로 "39 == seq"를 성공 조건으로 쓰면 이 경우를 따로 막아야 한다.

### L-12. 지난 실행이 남긴 레지스터 14 때문에 실행 전 단계를 "done"으로 보고하고 멈춘 단계 이름이 틀린다 (기존 ed7e27a 논리, 58582b5의 `step=` 표시에 영향)
- 위치: `doosan_client.py:223`, `:247-250`, `:261`, `:271`
- 문제: 지난 실행이 결과 없이 멈추면 15 = 0, 14 ≥ 1이 남는다. 다음 실행은 시작 전 15 == 0이라 `reset_seen`이 처음부터 True이고, 프로그램이 14를 지우기 전에 읽은 값으로 `advance`한다. `current`는 뒤로 가지 않는다.
- 시나리오(확인함, `_FakeRobot`): 레지스터 14 `5 → 0 → 1 → 2`, 15 `0, 0, 0, 249` → 프로그램이 돌기 전에 `open_before_pick`·`over_stone`·`descend_grasp`·`close_grasp`를 done으로 보고(시간값도 무의미). 실제로는 2단계에서 실패했는데 `MOVE_FAILED ... step=lift_stone`으로 나온다.
- 영향: 58582b5가 다루는 바로 그 상황(결과 없는 정지) 다음 실행에서 로그와 실패 단계 이름이 틀린다. 성공·실패 판정 자체는 바뀌지 않는다.

---

## 문제없다고 판단한 항목

**관점 1 — 경쟁 조건·상태 전이**
- STOP 뒤 결과 재읽기: STOP을 처음 보면 sleep 없이 15를 다시 읽고, 두 번째에도 없을 때만 실패. 변이(재읽기 제거)에서 `test_run_cycle_rereads_...` 실패 확인.
- 지난 실행의 250을 이번 성공으로 읽지 않음: `reset_seen`이 0을 본 뒤에만 표지를 받는다. `DRL_STOPPED_NO_RESULT`도 PLAY를 본 뒤에만 낸다.
- 레지스터 39/40 순서: DRL은 40 → 39 순으로 쓰고 transport는 39 → 40 순으로 읽으므로, 39가 일치한 시점의 40은 그 seq의 결과다(DRL 본문 읽기로 확인, 실기 미확인).
- `next_seq` 순환: 1~127만 돌고 0과 직전 값을 내지 않는다. 기동 때의 0과 겹치지 않고, 오래된 39가 새 seq와 같아지려면 지령 126개가 실행되지 않아야 한다.
- 한 번만 알림(`_last_seq = None`)과 seq 일치 조건: 각각 변이에서 시험 실패 확인.

**관점 2 — 안전**
- `cycle.drl`: 그리퍼 준비 실패 시 팔 이동 없음, 단계 실패 시 즉시 return으로 다음 단계 없음, 빈 손 파지(248) 뒤 이동 없음. 레지스터 16~23 초기화는 결과 쓰기 전에 실행된다.
- `pick_place._run_one_drl`: `RobotError`는 모두 종료 코드 2 또는 4로 닫히고 `--cycles` 반복도 멈춘다. 성공 뒤 마지막 이동 목표 도달을 다시 확인한다.
- 상주 루프: 전류 쓰기 실패 시 `goal_current`를 갱신하지 않아 다음 지령에서 다시 쓴다.

**관점 3 — 시험의 실효성**
- 변이에서 시험이 실패한 것(= 실제로 검증함): PLAY 선행 조건, 유예 2 s, 레지스터 16~23 초기화, 같은 revision 재수신의 수신 감시(44d113f의 목적), 정지 래치, `position_std` 검사, steady 수신 나이 검사, 생존 2주기 하한.

**관점 4 — 명세 일치**
- 이유 문자열: `RESIDENT_WRITE_FAILED`, `TIMEOUT_BELOW_LOOP_PERIOD`, `RESIDENT_INIT`, belt_sync의 10개 이유가 SPEC-500·TEST_SPEC과 코드에서 같다.
- 값: 0.58 s, 3주기 1.74 s, 2주기 1.16 s, 결과 242, TC-ROB-013-01의 목표 (0.50, 0)·(0.49, 0) m이 코드·시험과 일치.
- `specs/TRACE.md`: 저장소 상태와 `--check` 결과 일치(누락 0).
- 단위 변환 규칙: `belt_sync_demo.py`는 m·m/s·s만 출력하고 `tools`의 규칙 시험이 통과한다. `belt_sync.py`·`belt_sync_preview.py`에 단위 변환 없음.
- `Progress.md` 추가 줄은 완료 표시 없이 남은 조건만 적었다.
- `belt_sync_preview`는 `/tracking_preview/cmd_vel`·`/tracking_preview/status`만 발행한다(코드 읽기).

**관점 5 — 기존 동작**
- 다섯 코드 커밋의 diff를 확인했다. 58582b5는 `run_cycle`에 상태 조회 분기를 더하기만 했고 기존 표지 판정은 그대로다. 306dd40·c2e60e5가 뺀 줄은 DRL의 무조건 `write_position`과 `trigger`의 한 줄 반환뿐이다. L-2·L-3·L-12의 "기존" 표시와 M-2의 745ead1 귀속은 이 diff에 근거한다.
- 기존 시험 전부 통과(위 표). `test_run_cycle_reports_each_step_...`의 상태 대본 변경은 반복마다 상태를 읽게 된 것에 맞춘 것이다.
- `sample_period_s` 검사로 기본 `GripperCfg` + `ResidentLoopTransport` 조합이 생성 때 거부되지만, 시험 밖에서 이 조합을 만드는 코드는 없다(grep). `sample_period_s`가 없는 transport는 검사하지 않는다.
- `one_drl_done` 로그에서 상태 0인 값을 뺀 것: `tools`·`src`에 이 필드를 읽는 코드가 없다(grep).
- 기록 전용 커밋(3264b55, 4088b77, b478556): 세 커밋 diff 전체를 비밀 값·계정·주소 패턴으로 훑어 0건, 미검증 항목을 미검증으로 적음. 설명 불일치는 L-10 하나.
