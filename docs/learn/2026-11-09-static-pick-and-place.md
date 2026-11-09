# 정지 pick-and-place: 검출 좌표로 E0509가 공깃돌을 집어 뚜껑에 넣기 (2026-11-09)

카메라로 찾은 공깃돌을 E0509가 집어서 멈춰 있는 병뚜껑에 넣기까지의 과정을 처음 보는 팀원도 따라 할 수 있게 설명한다. 사실 기록은 [OFF-4 세션 기록](../../records/sessions/OFF-4-2026-11-09.md)과 [11월 9일 worklog](../../records/worklog/2026-11-09.md) 저녁 절, 원문 로그는 [records/trials/2026-11-09-static-pick-place](../../records/trials/2026-11-09-static-pick-place/)에 있다. 검출기 자체는 [공깃돌·병뚜껑 검출과 좌표](2026-11-09-stone-cap-detection.md)를 먼저 읽는다.

## 1. 한 사이클의 흐름

```
카메라 1프레임 ─▶ vision 검출(매트 공깃돌, 벨트 빈 뚜껑) ─▶ 평면 교차로 robot_base 좌표
   ─▶ 경유점 계획 + 역기구학 여유 확인 ─▶ (--execute일 때만) 실행
       열기(600) → 공깃돌 위(z 284) → 파지 높이로 하강 → 닫기(750, 150 mA)
       → 들기(z 284) → 뚜껑 위 → 해제 높이로 하강 → 열기(600) → 후퇴 → 대기 자세
   ─▶ 카메라 1프레임으로 뚜껑 안 공깃돌 확인(stone_inside)
```

코드는 [`src/static_pick_place`](../../src/static_pick_place/)다. 판단(어떤 돌·뚜껑을 고르고 어디로 갈지)은 ROS와 무관한 `plan.py`에 있고 단위 시험으로 고정했다. 로봇·그리퍼 호출은 `doosan_client.py`, 순서 실행은 `pick_place.py`다. 프로젝트 원칙(`PLAN.md` 0.2절)대로 판단과 입출력을 나눴기 때문에, 로봇 없이도 계획이 맞는지 시험할 수 있다.

이 패키지는 `robot_controller_node`에 실기 backend가 생기기 전까지 쓰는 **현장 도구**다. 설계상 로봇 지령 주체는 `robot_controller_node` 하나뿐이어서, SPEC-000 v0.3에 예외 범위(사람 입회, 오케스트레이터와 동시 실행 금지)를 적었다.

## 2. 실행 방법

```bash
cd ~/conveyor-tracker-bot
source /opt/ros/jazzy/setup.bash && source ~/doosan_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
       FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/drop_bringup/config/fastdds_large_shm.xml \
       PYTHONPATH=$PWD/src/static_pick_place:$PWD/src/vision:$PWD/src/robot_control:$PWD/src/calibration:$PYTHONPATH
SITE=records/calib/2026-11-09-planes/site.yaml   # 보정·평면 파일 경로. 파일 안 경로는 이 YAML 기준 상대 경로

python3 -m static_pick_place.pick_place --config $SITE                       # 계획만(로봇 지령 없음)
python3 -m static_pick_place.pick_place --config $SITE --image frame.png     # 저장된 영상으로 계획만
python3 -m static_pick_place.pick_place --config $SITE --execute             # 실기 1사이클([OFF], 입회 필수)
python3 -m static_pick_place.pick_place --config $SITE --execute --resume-from lift_stone \
        --stone-xyz 0.572 -0.0496 -0.0165 --cap-xyz 0.6032 0.0918 0.0839    # 중단된 사이클 잇기
```

출력은 한 줄에 JSON 하나다(`detections`, `targets`, `plan`, `ready`, `done`, `abort`, `verify`, `cycle_done`). 로그만 보고 무엇을 어디서 했는지 다시 짚을 수 있게 하려는 것이다. 종료 코드: 0 성공, 2 계획·로봇 오류, 3 작업자 중단, 4 빈 손 파지, 5 투입 확인 실패.

실행 전 확인할 것: 드라이버 연결(`dsr_bringup2 ... mode:=real`), 로봇 상태 STANDBY, **로봇 모드 AUTONOMOUS**(수동 모드에서는 그리퍼 DRL이 거부된다), 다른 DRL 없음. `check_ready()`가 이 셋을 보고 하나라도 어긋나면 시작하지 않는다.

## 3. 숫자는 어디서 왔나

TCP(공구 끝)를 제어기에 등록하지 않았으므로, 로봇이 보고하는 위치는 **플랜지**(손목 끝 원판) 위치다. 손가락 끝까지 길이를 모르니, 높이는 "플랜지가 대상 윗면보다 몇 mm 위인가"로 현장에서 직접 정했다.

| 값 | 어떻게 얻었나 |
|---|---|
| 파지: 플랜지 = 돌 윗면 + 162.0 mm | 플랜지 124.4 mm에서 사용자가 "너무 내려감", 20 mm 올린 144.4 mm에서 잡음. 돌 윗면은 검출 z −17.6 mm → 144.4 − (−17.6) |
| 해제: 플랜지 = 테두리 + 165.6 mm | 테두리 83.8 mm 위 244.4 mm에서 성공(160.6 mm), 이후 사용자 지시로 5 mm 올림 |
| 이동 높이 z 284.4 mm | 11-07 Hand-Eye 때 매트 위를 실제로 지나간 높이. 공깃돌을 든 채 테두리를 넘을 여유 |
| 돌 윗면 = 매트면 + 15 mm | 사용자 실측 공깃돌 높이. vision `h_stone_offset_m`에 넣으면 검출 위치 z가 곧 돌 윗면 |
| 테두리 = 벨트면 + 22 mm | 매트면에서 테두리 115 mm, 벨트면은 매트면 + 93 mm(작업대에서 벨트 100 mm, 매트 두께 7 mm) |
| 닫기 750 pulse·150 mA, 열기 600 pulse | 750은 11-07 사용자 지정 닫힘 끝점. 150 mA·600 pulse는 오늘 사용자 지시(작은 대상이라 많이 열지 않음) |
| 속도 수평 100 mm/s, 수직 50 mm/s | 처음 20/10 mm/s로 성공을 확인한 뒤 사용자 지시로 5배 |

매트 평면은 매트 위에 둔 ChArUco 보드의 PnP 자세로 구했다. depth는 좌표에 쓰지 않는다(같은 매트를 보드 PnP 618.6 mm, depth 598 mm로 재 3% 차이). 벨트 평면은 매트와 평행하다고 보고 실측 높이 차만 더했다. 평면 파일과 현장 설정은 [records/calib/2026-11-09-planes](../../records/calib/2026-11-09-planes/README.md)에 있다. `--config`로 준 YAML 안의 파일 경로가 상대 경로면 그 YAML이 있는 폴더 기준으로 푼다(`load_config`). 둘 다 **잠정값**이고, 외부 보정도 후보값이다. 그래서 첫 좌표는 높은 곳에서 XY만 맞추고 사람이 눈과 자로 확인한 뒤(1점, 2 mm 이하) 내려갔다.

## 4. 팔이 닿는지는 미리 계산한다

처음 벨트는 로봇 베이스에서 x ≈ 0.81 m에 있었다. 뚜껑 위로 가자 제어기가 "NOT REACHABLE"로 거절했다. `motion/ikin`(역기구학)으로 같은 자세의 관절각을 구해 보니 J3(팔꿈치)가 정확히 0°, 즉 팔을 끝까지 편 상태였다. 그리퍼를 수직으로 세우면 손목이 플랜지 위로 올라가서 수평으로 닿는 거리가 줄어든다. 높이를 160~284 mm로 바꿔도 J3는 0°였다.

선택지는 그리퍼를 바깥으로 20~25° 기울이거나(J3 26~42°로 닿음, 대신 해제 위치가 길이 추정 오차에 민감) 컨베이어를 옮기는 것이었고, 사용자가 컨베이어를 로봇 쪽으로 옮겼다(뚜껑 x ≈ 0.61 m, J3 53°).

그래서 실행기는 계획 단계에서 모든 경유점의 역기구학을 계산하고, J3 < 0.1745 rad(10°)이면 움직이기 전에 멈춘다(`NEAR_REACH_LIMIT`). 작업 공간 상자(x 0.30~0.75 m, y ±0.25 m) 밖 목표도 계획에서 거부한다.

## 5. 그리퍼 DRL: 왜 느리고, 어디서 막혔나

그리퍼는 PC가 아니라 제어기 플랜지의 RS-485에 붙어 있다. 그래서 PC가 직접 Modbus를 쓰지 못하고, **DRL**(제어기에서 도는 Python 비슷한 스크립트)을 `drl_start`로 보내 실행한다. DRL은 결과를 출력 레지스터(15번 = 결과 코드, 16~23번 = 위치·전류)에 쓰고, PC는 그 레지스터를 읽는다.

오늘 겪은 문제와 고친 방법:

| 증상 | 원인 | 대처 |
|---|---|---|
| `drl_state=3`이라 시작 거부 | 3 = `DRL_PROGRAM_STATE_LAST`, 드라이버 연결 뒤 아직 실행한 프로그램이 없을 때 값 | STOP(1)과 LAST(3)만 대기로 인정 |
| 결과 247 | 재기동 뒤 Torque Enable(RAM 256) = 0 | Operating Mode 5를 먼저 확인하고 RAM 256만 1로 씀(EEPROM은 쓰지 않음) |
| 실패한 DRL이 PLAY로 남고 `drl_stop`도 거부 | 실패 경로의 `exit()`가 프로그램을 끝내지 못함 | `exit()`를 쓰지 않고 결과 코드를 반환, 스크립트가 끝까지 실행되어 스스로 종료 |
| 돌을 잡았는데 60 s 시간 초과 | 막혀서 750에 못 가는데 "목표 도달"만 기다림 | 위치가 멈추면 파지로 판정(아래) |
| 파지 직후 들기 명령이 무시됨 | 결과 250을 쓴 순간 DRL은 아직 종료 중(PLAY). 그때 보낸 movel은 무시 | 결과를 받은 뒤 DRL 상태가 STOP이 될 때까지 기다림 |
| 지난 실행의 250을 이번 결과로 읽을 위험 | 레지스터는 실행 사이에 남는다 | DRL이 시작하며 15를 0으로 지우는 것을 본 뒤에만 결과를 받음 |
| 읽기 대기를 줄였더니 246·247 | 늦게 온 응답이 버퍼에 남아 다음 요청이 엉뚱한 응답을 읽음 | 시작할 때 수신 버퍼를 한 번 비움. CRC·모델·모드 확인 덕분에 잘못된 명령은 나가지 않음 |
| 이미 쥔 상태에서 다시 닫으면 판정 실패 | 위치가 안 변해서 "움직인 뒤 멈춤" 조건 불성립 | 정지 + (명령 전보다 10 pulse 이상 이동 **또는** 전류 ≥ 한도 80%) |

**느린 이유.** 이미 열린 그리퍼에 "열기"만 보내도 6~8 s가 걸렸다. 읽기 대기(timeout)를 0.5 → 0.25 s로 줄여도 그대로였고, 0.15 s 이하에서는 응답을 못 받았다. 읽기만 하는 진단 DRL로 재 보니 **Modbus 왕복 한 번이 약 0.58 s로 고정**이었다(대기 0.5 s와 0.2 s 모두 5회에 2.9 s). 바꿀 수 없는 경로 지연이므로 왕복 횟수를 줄였다: 모델·모드를 한 번에(0~5번), 토크·오류·목표 전류를 한 번에(256~275번), 루프에서는 전류와 위치를 한 번에(287~291번) 읽는다. 열린 상태 열기 7.2 → 5.2 s. 3회차에서 그리퍼 세 번이 19.7 s로 사이클 36.7 s의 절반 이상이다. 더 줄이려면 세션 중 확인 읽기를 생략하거나, 팔 동작까지 하나의 DRL에 넣는 방법(`PLAN.md` 11.3절의 대안)이 있다.

**"목표 위치에 못 감"이 성공일 수 있다.** 닫기 목표는 750 pulse지만 돌을 쥐면 손가락이 659~661에서 멈춘다. 이때 전류는 한도(150 mA) 근처인 149 mA다. 반대로 750까지 닫혔다면 손에 아무것도 없다는 뜻이라 실행기는 `GRASP_EMPTY`로 멈춘다.

## 6. 제어기·드라이버에서 배운 것

- **재부팅하면 상태가 바뀐다.** 로봇 모드가 수동(MANUAL)으로 돌아간다. 수동에서도 `move_line`은 감속해서 움직이지만 DRL은 거부된다. 펜던트가 자동·대기 상태가 아니면 ROS 드라이버의 제어 권한 요청도 거부된다(`ACCESS_CONTROL_DENY`).
- **지난 알람이 남는다.** 도달 불가 이동 거절(1206, level 2)이 "마지막 알람"으로 남아, level ≥ 2면 멈추는 그리퍼 도구가 계속 거부했다. 알람을 지우지 않고, 확인한 번호 하나만 넘기는 `--ack-alarm 1206`을 만들었다. 실행기는 지난 알람을 기록만 하고 상태(STANDBY)로 판단한다.
- **launch 종료가 내 셸을 죽인다.** 같은 프로세스 그룹에서 띄운 `ros2 launch`에 SIGINT를 보내면 그룹 전체가 정리되어 지령을 보내던 셸도 끝났다. 백그라운드 드라이버는 `setsid`로 따로 띄운다. `pgrep -f`로 찾을 때는 자기 명령줄과 겹치지 않게 `[r]os2` 같은 패턴을 쓴다.

## 7. 아직 증명되지 않은 것

- 반복 성공률(G2는 50회 ≥ 95%). 오늘은 4회 투입, 그중 2회는 중간에 재개했다. 끊김 없는 실행은 1회.
- 좌표 정확도. 수평 오차는 1점 확인뿐이고 외부 보정 20점 검증 전이다.
- 재배치 뒤 평면. 매트는 같은 작업대라 그대로 썼지만 다시 재지 않았다.
- 파지 전류 최소값, 개방 시간 `Δ_open`, 공구 무게 등록.
- 반복 모드(`--cycles`)는 dry-run만, 곡선 연속 이동(`move_blending`)은 계획만 있다.

## 관련 파일

- 코드: [`plan.py`](../../src/static_pick_place/static_pick_place/plan.py), [`doosan_client.py`](../../src/static_pick_place/static_pick_place/doosan_client.py), [`gripper_move.drl`](../../src/static_pick_place/static_pick_place/drl/gripper_move.drl), [`pick_place.py`](../../src/static_pick_place/static_pick_place/pick_place.py), [설정](../../src/static_pick_place/config/static_pick_place.yaml)
- 시험: [`test_plan.py`](../../src/static_pick_place/test/test_plan.py), [`test_doosan_client.py`](../../src/static_pick_place/test/test_doosan_client.py)(dsr_msgs2 없이 서비스 호출을 가짜로 바꿔 레지스터 처리·DRL 종료 대기를 확인)
- 명세: [SPEC-000](../../specs/SPEC-000-system.md) 4.1절(11개 패키지, 지령 주체 예외), [Progress](../../Progress.md) 5·6절
- 커밋: 849b7d3(패키지·첫 실기), ab0fd4b(속도·파지 판정), 4fa5b8d(Progress)
