# 고정 좌표계 TF와 RViz: 로봇·컨베이어·매트를 한 화면에 (2026-11-11)

G3(움직이는 병뚜껑 추적)의 "Moving Reference Frame" 중 장비 없이 할 수 있는 부분을 만들었다. 사실 기록은 [11월 11일 worklog](../../records/worklog/2026-11-11.md) 19:00 절, 규칙은 [SPEC-600 4.9절](../../specs/SPEC-600-calibration.md)이다.

## TF가 왜 필요한가

카메라는 "카메라에서 본 위치", 로봇은 "로봇 몸통 기준 위치", tracker는 "벨트 기준 위치"로 말한다. TF는 이 좌표계들 사이의 관계(이동 + 회전)를 나무 모양으로 모아 두어, RViz나 다른 노드가 아무 좌표계에서 다른 좌표계로 바꿀 수 있게 한다.

```
world ─ base_link(Doosan) ─ robot_base ─┬─ conveyor_frame ─ cap_<id>(tracker, 관측 위치)
                                        ├─ mat_frame
           base_link ─ camera_link(camera_tf_publisher) ─ 광학 프레임(RealSense)
```

- **고정(static) TF**는 변하지 않는 관계라 시작할 때 한 번 `/tf_static`으로 보낸다. 늦게 켠 노드도 받을 수 있게 "지난 메시지 보관(transient local)"으로 전달된다.
- **한 쌍은 한 노드만 발행한다.** `world → base_link`는 Doosan 드라이버(robot_state_publisher)가 이미 내므로 우리 노드는 드라이버가 없을 때(`publish_world:=true`)만 낸다. 두 노드가 같은 쌍을 내면 TF가 번갈아 바뀌며 흔들린다.
- **`robot_base`와 `base_link`**: 프로젝트 메시지는 `robot_base`, Doosan은 `base_link`라는 이름을 쓴다. 같은 좌표계이므로 항등 TF로 이어 RViz가 둘을 함께 그리게 했다.
- **미래 위치는 TF로 내지 않는다.** tf2는 미래 시각을 외삽하지 않는다. 예측 해제 위치는 planner가 `drop/predicted_marker`(Marker)로 따로 보낸다.

## 평면에서 좌표계 만들기(`plane_frame`)

매트 평면 보정 결과는 "법선 n과 거리 d"(`n·p = d`)뿐이다. 좌표계로 쓰려면 원점과 x축이 필요하다.

1. z축 = 법선. 아래를 향하면 뒤집어 위쪽으로 맞춘다(같은 평면).
2. x축 = 원하는 방향(`x_hint`)에서 법선 성분을 빼 평면 위로 눕힌 방향.
3. y축 = z × x(오른손 좌표계).
4. 원점 = 원하는 (x, y)를 지나는 수직선이 평면과 만나는 점: `z = (d − n_x·x − n_y·y) / n_z`.

## 시험에서 배운 것

- **각도는 `arccos`로 재지 않는다.** 같은 방향인데 내적이 반올림으로 0.9999999999999999가 되면 `arccos`가 1.5e-8 rad를 낸다. `atan2(|a×b|, a·b)`는 작은 각도에서도 정확하다. 처음 실패한 시험은 코드가 아니라 이 측정 방법이 틀렸던 것이다.
- **시험이 진짜 잡는지 확인**: 구현에 버그를 하나씩 넣어(법선 부호, 투영, 원점 높이) 시험이 실패하는지 보고 되돌렸다.
- **ID를 쓰기 전에 찾아본다**: 새 요구사항 번호를 010으로 붙였다가 기존 항목과 겹쳐 011·012로 바꿨다. `grep -o 'REQ-CAL-[0-9]*' specs/SPEC-600-calibration.md | sort -u`로 먼저 확인한다.

## 직접 해 보기(장비 없음)

```bash
source install/setup.bash
ros2 launch drop_bringup frames.launch.py publish_world:=true rviz:=true
ros2 run tf2_ros tf2_echo robot_base mat_frame          # 다른 터미널
```

mock과 함께 띄우면 tracker의 `cap_<id>`와 planner 예측 마커도 같은 화면에 보인다. `conveyor_frame`은 벨트 평면·진행 방향 보정(OFF-3) 전이라 아직 `robot_base`와 같은 자리(항등)다.

## 이어서: 규칙이 깨지지 않게 시험으로 묶기

"미래 시각을 TF로 묻지 않는다", "같은 TF를 두 노드가 내지 않는다"는 규칙은 지금 코드에서 지켜지고 있다. 하지만 누군가 나중에 planner에서 `lookup_transform`을 쓰면 아무도 모르게 깨진다. 그래서 [test_tf_usage.py](../../tools/spec/test_tf_usage.py)가 `src/` 코드를 훑어 TF 조회·발행이 허용된 파일에만 있는지 본다(REQ-SYS-006).

- **토큰으로 검사한다.** 파이썬 `tokenize`로 실제 코드 이름만 센다. 주석이나 문자열 안의 `lookup_transform`은 세지 않는다.
- **검사기 자체도 시험한다.** 임시 폴더에 일부러 위반 파일을 만들어 검사기가 정확히 1건을 잡는지 본다. 검사기가 항상 "문제없음"을 내는 버그를 막는다.
- **시계 역행**: 시뮬레이션·녹화 재생에서 ROS 시각이 뒤로 가면 이전 추적·계획은 믿을 수 없다. tracker(`test_clock_reset`, `test_reverse_stamp`)와 planner(I2 `clock_reset`)가 이미 이를 시험한다. 새로 짜기 전에 이미 있는 시험을 찾아 돌려 보는 것도 확인 작업이다.
