# 정지 pick-and-place를 빠르게: 그리퍼 통신 왕복 줄이기 (2026-11-10)

## 무엇이 느렸나

11-09 3회차 실기는 한 사이클에 36.4 s였고 그중 그리퍼 3회가 19.8 s였다([실행 로그](../../records/trials/2026-11-09-static-pick-place/pp-run3.jsonl)). 손가락이 느린 것이 아니다. 그리퍼는 로봇 플랜지의 RS-485에 물려 있어 PC가 직접 말을 걸 수 없고, 제어기 안에서 도는 스크립트(DRL)가 Modbus로 대신 묻고 답을 기다린다. 이 왕복 한 번이 약 0.58 s이고, 명령 한 번에 확인·쓰기·읽기 왕복이 8~11번 들어 6~7 s가 된다.

`tools/eval/cycle_time_budget.py <로그>`로 단계별 시간과 "방식을 바꾸면 얼마가 되는지"를 계산할 수 있다. 이동 시간은 `거리/속도 + 속도/가속`에 구간마다 0.14 s를 더한 값과 7개 구간 모두 맞았다.

## 두 가지 길

| 방식 | 내용 | 상태 |
|---|---|---|
| 한 프로그램 사이클(`--one-drl`) | 이동(`movel`)과 그리퍼를 DRL 프로그램 하나에 넣어 보낸다. 그리퍼 확인은 처음 1회, 이미 열려 있으면 첫 열기를 쓰지 않는다 | 토요일 실기 대상 |
| 상주 루프(`robot_control/gripper_session.py`, `resident_transport.py`, `drl/gripper_resident.drl`) | DRL을 한 번 띄워 두고 PC는 레지스터로 명령한다. 예약 뒤 트리거로 해제 지연을 줄인다. 동적 투입에 필요하다 | 가짜 제어기 시험만 |

상주 루프를 바로 쓰지 않는 이유는 11-09 현장 기록에 있다. DRL이 PLAY인 동안 보낸 `move_line`이 움직이지 않고 끝났고, PLAY로 남은 DRL은 `drl_stop`이 듣지 않아 제어기를 재부팅했다. 그래서 먼저 **스스로 끝나는** 한 프로그램 방식을 쓴다. 상주 루프는 이 두 가지를 실기에서 확인한 뒤다.

## 가짜 제어기에서 본 시간

DRL은 Python 문법이라, 시험에서 `flange_serial_*`·`movel`·레지스터 함수를 가짜로 주고 **DRL 본문을 그대로 실행**한다(`src/static_pick_place/test/test_cycle_drl.py`). 문자열 검사보다 훨씬 많은 것을 잡는다. 다만 실기 측정은 아니다.

| 조건 | 그리퍼 | 이동 | 사이클 |
|---|---|---|---|
| 실기 측정(단계별 실행) | 19.76 s | 16.59 s | 36.35 s |
| 한 프로그램, 왕복 0.58 s | 6.30 s | 16.59 s | 22.89 s |
| 위와 같고 수직 100 mm/s | 6.30 s | 13.12 s | 19.42 s |
| 위와 같고 수평 200 mm/s | 6.30 s | 9.97 s | 16.27 s |

## 토요일(11-14) 실기 순서

사람 입회·비상정지 대기가 전제다(`[OFF]`). 한 단계가 기대와 다르면 다음으로 넘어가지 않는다.

1. 준비: 드라이버 `mode:=real`, 로봇 STANDBY·AUTONOMOUS, 다른 DRL 없음. `python3 -m pytest -q src/static_pick_place/test`.
2. 배치 확인: 11-09에 컨베이어·매트를 다시 놓았으므로 평면·보정 파일이 지금 배치와 맞는지 먼저 본다. `pick_place --config $SITE`(계획만)의 좌표가 눈으로 본 위치와 맞아야 한다.
3. 기존 방식 1회: `pick_place --config $SITE --execute`. 배치와 높이가 유효한지 검증된 경로로 확인한다.
4. 새 방식을 단계로 늘린다.
   - `--execute --one-drl --stop-after open_before_pick`: 팔은 움직이지 않는다. DRL이 끝나고 STOP으로 돌아오는지 본다.
   - `--execute --one-drl --stop-after over_stone`: DRL 안 `movel` 첫 실행(이동 높이). 여기서 움직이지 않거나 249가 나오면 중단한다.
   - `--execute --one-drl --stop-after close_grasp`: 하강과 파지. 돌을 쥔 채 멈추면 로그의 좌표로 `--execute --resume-from lift_stone --stone-xyz … --cap-xyz …`(기존 방식)로 잇는다.
   - `--execute --one-drl`: 전체.
5. 기록: 로그를 `records/trials/2026-11-14-static-pick-place/`에 두고 `cycle_time_budget.py`로 단계 시간을 비교한다.

멈추는 법: 프로그램 실행 중에는 PC에서 단계 사이에 끼어들 수 없다. 비상정지나 펜던트로 멈춘다. DRL이 PLAY로 남으면 드라이버를 다시 띄우지 말고 상태를 기록한 뒤 11-09 절차(제어기 재부팅)를 따른다.

## 흔한 함정

- 예약(prewrite)해 둔 목표는 다른 명령이 레지스터를 덮어쓰면 무효다. 그대로 트리거하면 엉뚱한 값이 실행된다(`gripper_adapter`에서 고침).
- 상주 루프가 올린 위치는 0.6 s마다 바뀐다. 같은 표본을 두 번 읽고 "멈췄다"고 판정하면 안 되므로 생존 카운터가 바뀐 표본끼리만 비교한다.
- 가짜 제어기의 시간은 모델이다. 왕복 0.58 s가 한 프로그램 안에서도 같은지, `movel`과 플랜지 시리얼을 한 프로그램에서 같이 쓸 수 있는지는 실기에서만 알 수 있다.

관련: [worklog](../../records/worklog/2026-11-10.md), `specs/SPEC-500-robot-control.md` REQ-GRP-014~016, `src/static_pick_place/static_pick_place/drl/cycle.drl`.
