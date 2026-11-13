# 동적 투입 코드와 시험 환경의 함정

2026-11-12 밤 ~ 11-13, coordinator(Claude Opus 5.5)와 Claude 하위 작업자. 관련: [11월 12일 worklog](../../records/worklog/2026-11-12.md) 22:50 이후 절, [11월 13일 worklog](../../records/worklog/2026-11-13.md), [결정 기록](../../records/reviews/fable-decisions-2026-11-12.md).

## 무엇을 했나

- 남은 결정 6건을 다른 모델(Fable 5.2)에 자문하고 정했다: DynamicPlace 구조, 벨트 `steady=false` 처리, Stop 해제 직후 경합, Doosan backend, 도달 구간 설정, 간헐 `STEP_TIMEOUT`.
- 로봇 쪽 DynamicPlace를 구현했다(접근 → 동기 → 하강 → 해제 게이트 → 후퇴). 실제 `robot_controller_node`(가짜 로봇)가 mock과 같은 계약 시험을 통과한다.
- 시험이 없던 TC 6건에 시험을 붙이며 결함 2건을 찾았다(tracker stamp 21 ns 어긋남, calibration 불일치 이유 코드).
- 연속 제어 후보 조사([records/reviews/stream-candidates-2026-11-12.md](../../records/reviews/stream-candidates-2026-11-12.md))와 현장 안전 절차([docs/operations/safety-procedure.md](../operations/safety-procedure.md))를 썼다.

## 1. 경계를 어디에 두나: "해제 확정(commit)"

현장(OFF-2)에서 확인하기 전에는 두 방식 중 무엇을 쓸지 모른다.

- 스트림 + 상주 루프: PC가 10 ms마다 속도를 보내고, 그리퍼는 상주 DRL에 트리거를 보낸다. 해제 직전까지 목표를 고칠 수 있다.
- 한 DRL 프로그램: 하강·개방·후퇴를 제어기 프로그램 하나로 보낸다. 시작하면 PC가 끼어들 수 없다.

경계를 "속도 지령"에 두면 첫 방식에만 맞는다. 그래서 경계를 **해제를 확정하는 순간**에 두었다(`dynamic_place.PlaceTransport.commit_release`). transport는 `commit_lead_s`(확정 → 공깃돌이 손가락을 떠나는 시간)를 알려 주고, 해제 게이트는 그만큼 앞을 예측해 한 번 판정한다. 두 방식은 같은 게이트 함수를 쓰고 예측 거리만 다르다. 한 DRL 방식은 예측 거리가 길어 0.5 mm 기준을 맞추기 어려우므로 MODE_PREDICTED 수준까지만 쓴다(SPEC-500 4.5절).

## 2. 맥동 경보 한 번에 멈추지 않기

실제 벨트는 다이얼을 고정해도 속도 변화 경보가 관측 60 s당 11~18번 울린다. 경보가 나면 속도 공분산을 되돌리므로 그 순간의 계획은 `UNCERTAIN`이 된다. 경보 한 번에 추종을 멈추면 동적 구간(수 초)의 사이클 대부분이 끊긴다.

- 동기화기: `UNCERTAIN` 계획은 버리고 마지막 OK 계획을 최대 0.6 s 등속으로 따른다(`uncertain_hold_s`). 넘으면 멈춘다.
- 해제 게이트: `steady=false`인 동안 닫힌다. 안전은 "멈춤"이 아니라 "열지 않음"으로 지킨다.
- 오케스트레이터: `steady=false`가 1 s 넘게 이어질 때만 중단한다(`belt_unsteady_hold_s`).

## 3. 시각은 그대로 들고 다닌다

`sec + nanosec·1e-9`로 바꾼 float 초는 1.76e9 s 규모에서 분해능이 약 240 ns다. tracker가 검출 stamp를 float으로 바꿨다가 되살려 발행하자 21 ns가 어긋났다(REQ-SYS-003은 0 ns). 시각을 그대로 넘겨야 하는 곳은 원래 메시지의 stamp를 보관해 쓴다. 시험 장치의 시각이 100 s 근처이면 이 문제가 안 보인다. 실제 epoch 시각으로 시험해야 한다.

비슷한 순서 문제도 있었다. 주기 시각을 읽은 뒤 TCP 표본을 읽으면 표본이 µs 늦어 나이가 음수가 되고, `TCP_STALE`로 끝났다. 가짜 시계 단위 시험에서는 안 보이고 실제 노드에서 드러났다(TC-ROB-008-91).

## 4. 시험 환경의 시계: WSL은 시계를 뒤로 돌린다

집 PC WSL에서 실제 노드 계약 시험이 30번 중 2번쯤 `CLOCK_BACKWARD`로 실패했다. 코드에서는 한 스레드가 같은 시계로만 시각을 읽으므로 경합일 수 없다. 시스템 시각을 연속으로 읽어 보니 약 32.5 s마다 0.87 s씩 뒤로 갔다. WSL 안 `systemd-timesyncd`와 Windows 호스트 시간 동기가 충돌하는 것으로, 11-12에 사무실 PC에서 본 것과 같다.

```bash
python3 - <<'EOF'
import time
end = time.monotonic() + 120; prev = time.time(); n = 0
while time.monotonic() < end:
    t = time.time(); n += t < prev; prev = t
print('backward steps', n)
EOF
```

- 벽시계를 쓰는 노드(실제 tracker, 실제 controller)는 시각 역행을 장애로 처리한다(설계대로다). 시험이 이상하게 실패하면 먼저 시계부터 본다. `tools/bench/check_clock_steps.py`도 쓸 수 있다.
- 현장 장비(LEGION)도 시연 전에 같은 확인을 한다(Progress 7절 OFF-5 항목).

## 5. CI 공유 러너와 실시간 시험

10 Hz 주기·간격 0.3 s 같은 시험은 공유 러너가 바쁘면 실패한다. 실패한 것은 vision 계약 시험과 controller 상태 주기 시험이었다. 규칙(TEST_SPEC 0.3절)대로 `realtime_pc` 마커를 달아 CI에서는 실행·기록만 하고 판정은 PC에서 한다. 마커만 달고 끝내지 않는다. PC에서 그 시험을 실제로 돌려 판정한 결과를 worklog에 남긴다.

## 6. "0개"를 확인하는 시험은 양성 증거부터

replay launch 시험은 로봇 지령 경로가 그래프에 0개인지 본다. 그런데 노드 발견이 덜 끝났어도 "0개"는 통과해 버린다. 그래서 다음 순서로 확인했다.

1. 대상 노드의 ACTIVE heartbeat와 데이터 흐름이 보일 때까지 기다린다.
2. 금지 항목을 일부러 만든 탐침 노드로 조회가 실제로 그것을 잡는지 확인한다.
3. 그다음 0개를 판정한다.

`ros2 bag play`는 녹화된 토픽을 다 다시 낸다. 지령 경로를 끊으려면 허용 목록(`--topics`)을 쓴다.

함정 하나: `subprocess.PIPE`로 받은 launch 출력을 끝까지 안 읽으면 파이프(64 KB)가 차서 launch와 노드가 출력에서 멈출 수 있다. 출력은 스레드로 계속 비운다.

## 7. 다른 모델의 초안을 그대로 믿지 않는다

조사·안전 문서 초안은 OpenCode가 썼고, 수치와 인용은 원자료와 대조했다. 초안이 스스로 "인용 오류"라고 고친 것이 오히려 틀렸고(PLAN.md 11.3절에는 `amovel` 대안 행이 있다), "z 상한 미설정"도 틀렸다(`plan.py`가 `travel_z_m`을 상한으로 쓴다). 검수는 "그럴듯한가"가 아니라 "원본 파일에 그렇게 적혀 있는가"로 한다.

## 8. 장애 감시를 언제 켜나: 새 heartbeat를 본 다음

감시 노드가 Stop을 걸고 사람이 풀자마자 다음 사이클을 시작하면, 오케스트레이터가 들고 있던 controller heartbeat는 아직 "정지 유지(101)"다. heartbeat는 10 Hz라 캐시는 최대 0.1 s 낡았다. 그 낡은 값 때문에 방금 시작한 사이클이 L4로 끝나고, 방금 푼 로봇을 다시 정지시켰다. 고친 방법: goal을 받은 뒤 새로 들어온 heartbeat 2개와 `robot/status`를 확인한 다음 장애 latch를 켠다. 새 값이 여전히 101이면 그대로 L4다(REQ-ORC-012, TC-ORC-012-02).

## 9. 여러 스레드가 시각을 읽으면 없는 역행을 본다

시각 역행 검사를 여러 콜백이 각자 하면, 먼저 읽은 시각이 나중에 비교되는 순서 엇갈림 때문에 실제로 없는 역행을 본다. 그래서 steady 0.05 s 타이머 하나만 표본을 읽어 판정한다(REQ-ABT-012). 감시 타이머와 메시지 콜백이 같이 쓰는 (값, 시각)은 tuple 하나로 한꺼번에 바꿔, 읽는 쪽이 섞인 값을 보지 않게 한다.

## 10. 실기 연결 코드를 실기 없이 준비할 때

- `rclpy.spin_until_future_complete`를 action 콜백 안에서 부르면 이미 돌고 있는 executor와 엉킨다. 서비스 호출은 전용 helper 노드와 그 노드만 도는 executor 스레드로 하고, 기다림은 steady 시계로 잰다(`dsr_register_io.py`).
- 이동이 끝났다는 판정은 "명령이 받아들여졌다"가 아니라 측정 자세와 로봇 상태(STANDBY)로 한다. 11-09 현장에서 DRL이 PLAY인 동안 보낸 `move_line`은 받아들여졌지만 움직이지 않았다.
- 정지를 요청했는데 정지가 확인되지 않으면 깔끔한 취소가 아니라 FAULT로 보고한다. 팔이 아직 움직일 수 있다.
- rclpy 파라미터를 빈 목록 `[]`으로 선언하면 BYTE_ARRAY 형식이 되어 실수 배열로 덮어쓸 수 없다. `ParameterDescriptor(dynamic_typing=True)`로 선언한다.
- 이 프로젝트에서 그리퍼 아래 방향 자세는 quaternion `(0, 1, 0, 0)`이다. 단위 quaternion(identity)이 아니다. 기본값 Pose(원점, w=1)는 "설정 안 됨"으로 본다.

## 11. 짧은 경보는 "지속 시간"으로 판정하고, 적용 구간을 나눈다

벨트 `steady=false`는 PRECHECK·WAIT_CAP·동적 단계에서 뜻이 다르다. PRECHECK와 동적 단계에서는 1 s 넘게 이어질 때만 장애이고, WAIT_CAP에서는 그동안 기다린다. 새 감시 장애는 PRECHECK·중단 경로·WAIT_OPERATOR에서는 잡지 않는다. 그 구간에서 잡으면 로봇이 아직 안 움직였는데 L3 동작을 하거나, 이미 진행 중인 중단 경로를 L4로 올려 버린다(SPEC-110 4.4절).

## 12. 실제 controller를 통합 시험에 넣으면 대역이 숨기던 어긋남이 드러난다

`drop.launch.py use_mock_robot:=false robot_backend:=fake`는 대역 대신 실제 `robot_controller_node`를 띄운다. backend만 시뮬레이션이고 skill 순서기·해제 게이트·정지 유지는 실기와 같은 코드다. 이 구성으로 오케스트레이터 I0(TC-INT-014~018)를 돌리자 대역으로는 보이지 않던 결함 6건이 나왔다. 대역은 정해진 답만 내므로 SPEC과 코드가 어긋나도 통과한다.

- **같은 시간을 두 노드가 따로 재면 경계에서 실패한다.** controller는 SYNCHRONIZE를 0.3 s 유지하고 끝냈는데, 오케스트레이터가 feedback 수신 시각으로 다시 재면 0.2998 s가 되어 TIMEOUT이었다. 받는 쪽이 정확히 맞추기를 기대하지 말고, 보내는 쪽이 여유(`sync_hold_margin_s` 0.03 s)를 두고 연속 통과가 시작된 주기에도 feedback을 낸다.
- **cancel 직후의 속도는 아직 지령 속도가 아니다.** TCP가 1차 지연(시정수 29 ms)을 따르므로, 추종을 멈춘 직후에는 보정 속도가 남아 있다. 첫 표본부터 동기 유지 상승을 판정하면 `SYNC_SPEED` → L4가 된다. XY 속도가 계획 속도에 정착한 뒤(최대 0.2 s) 올린다.
- **SPEC과 코드의 상수가 다르면 통합에서 바로 걸린다.** 오케스트레이터는 PickStone을 150 mA로 보내는데 controller 코드 상한은 100 mA라 거부했다. SPEC의 1~150 mA로 맞췄다. PlaceStone의 `sync_held`가 0으로 고정돼 있던 것, `robot/status`의 그리퍼 필드가 거짓으로 고정돼 있던 것도 같은 종류다.
- **모르는 결과는 지어내지 않는다.** mock 세계는 실제 controller가 떨어뜨린 공깃돌의 위치를 모른다. 그래서 VerifyDrop은 SUCCESS가 아니라 UNKNOWN을 내고, 사이클 결과도 UNKNOWN이다(REQ-MCK-010). 시험이 통과하려고 성공을 만들어 내면 그 시험은 아무것도 증명하지 않는다.
- **action 결과는 제3자가 다시 읽을 수 있다.** skill 결과(`released`, `stone_placed_back`)는 오케스트레이터만 받지만, 시험은 action 서버의 `_action/get_result` 서비스로 같은 결과를 다시 읽어 확인한다.

## 13. 실시간 루프에 기대는 I0는 PC에서 판정한다

실제 controller의 DynamicPlace는 0.01 s 주기로 돌고, 주기 사이가 30 ms(`max_loop_gap_s`)를 넘으면 `LOOP_GAP`으로 멈춘다. 실기에서는 맞는 안전 동작이지만, 여러 노드를 한꺼번에 띄운 공유 CI 러너에서는 스케줄 지연만으로 넘을 수 있다. 그래서 이 루프를 쓰는 I0(TC-INT-014-01, TC-INT-017-01)에는 `realtime_pc`를 달고, CI는 별도 단계에서 실행·기록만 한다. 판정은 시계가 안정된 PC(사무실 PC WSL)에서 한다(TEST_SPEC 0.3절). 루프를 쓰지 않는 STATIC·통과 소진·bringup I0는 CI에서 그대로 판정한다.

## 14. "Traceback 없음" 검사가 잡은 종료 경합

`test_real_bringup.py`의 도달 구간 시험이 가끔 실패했다. 실패 로그를 남기도록 실행하자 원인이 보였다. 시험은 세 노드의 파라미터를 확인하자마자 launch를 SIGINT로 끝낸다. 그때 mock 노드는 아직 생성자 안에서 같은 역할의 실제 노드가 있는지 1 s 동안 그래프를 보고 있었고(`exclusive_check_s`), 그 `time.sleep`에서 `KeyboardInterrupt` traceback이 찍혔다. `spin_mock`은 실행 중 인터럽트는 조용히 처리했지만 생성 중 인터럽트는 처리하지 않았다. 생성 중에도 같은 방식으로 끝내도록 고치고 시험(TC-MCK-007-03)을 붙였다. 교훈은 두 가지다. 간헐 실패는 "다시 돌리니 통과"로 넘기지 말고 실패 출력을 파일로 남겨 다음에 원인을 보게 한다. 그리고 출력에 traceback이 없어야 한다는 단순한 검사가 종료 경로의 빈틈을 잡는다.

## 15. 확인되지 않는 Stop은 간격을 늘려 재시도한다

실기 연결이 없는 기본 bringup(`robot_backend` 기본 `disabled`)에 오케스트레이터를 켜면 Stop이 끝없이 반복됐다. controller가 `connected=false`를 내면 감시 노드가 `ROBOT_COMM_LOST`(L4)로 `robot/stop`을 부르고, controller는 정지 유지를 걸지만 확인할 backend가 없어 `stopped=false`로 답한다. 감시 노드는 0.1 s 뒤 다시 보낸다. 12 s 동안 Stop 60회, 경고 61줄이었다.

쉬워 보이는 해법은 disabled backend가 `stopped=true`로 답하는 것이다. 그러면 로봇을 볼 수 없는 쪽이 정지 확인을 지어내게 된다. `stopped`는 "정지를 확인했다"는 뜻이다(SPEC-010, REQ-ROB-007). 대신 재시도 간격을 늘렸다. `stopped=false`가 이어지면 0.1 s에서 두 배씩 늘려 1 s를 상한으로 하고, 경고는 처음과 5 s마다만 남긴다. 안전 효과는 줄지 않는다. controller는 Stop을 받는 즉시 정지 유지를 걸고(REQ-ROB-014), `stopped=false`는 Stop이 전달됐다는 뜻이기 때문이다. 전달 여부가 불확실한 서비스 부재·호출 예외는 0.1 s를 그대로 두고, 사유가 바뀌면 바로 보낸다. 판단 근거와 버린 선택지는 `records/reviews/stop-retry-decision-2026-11-13.md`에 있다.

## 16. 간헐 실패는 진단 로그를 넣고 다음 발생을 기다린다

CI에서 가끔 감시 노드가 사이클 도중 `CONTROLLER_HEARTBEAT_LOST`로 Stop을 냈다. 다시 돌리면 통과해서 원인을 알 수 없었다. 그래서 판정은 그대로 두고 로그만 늘렸다. 손실을 판정할 때 마지막 수신 나이, 받은 개수, 마지막 seq, 감시 노드 자신의 직전 주기 간격을 남기고, 다시 받았을 때 빠진 seq 개수를 남긴다. 다음 발생(2026-11-13 run 43366903740)에서 감시 주기 간격은 0.050 s로 정상이었고 seq는 빠짐 없이 18→19였다. 감시 노드가 느렸던 것도, controller가 메시지를 잃은 것도 아니다. heartbeat 하나가 0.43 s 늦게 왔다. 남은 질문은 "늦게 보냈나, 늦게 전달됐나"다. 메시지 header stamp는 보낸 시각이므로, 받은 시각과의 차이를 로그에 더하면 다음 발생 때 둘을 가를 수 있다. 진단은 질문 하나를 지우고 다음 질문을 좁히는 식으로 쌓는다.
