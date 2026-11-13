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
