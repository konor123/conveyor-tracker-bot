# 실제 planner·tracker를 bringup에 연결하기 (2026-11-11)

G4 준비로, 시스템을 띄우는 launch 파일이 가짜(mock)가 아니라 실제 tracker·planner를 쓸 수 있게 했다. 사실 기록은 [11월 11일 worklog](../../records/worklog/2026-11-11.md) 20:10 절, 규칙은 [SPEC-400](../../specs/SPEC-400-planner.md) REQ-PLN-012다.

## 무엇이 문제였나

`drop.launch.py`는 역할마다 "실제 노드"와 "대역(mock)" 중 하나를 고른다(`use_mock_<역할>`). 그런데 planner 역할은 아예 없었고, tracker는 "아직 없음"으로 남아 있었다. 그래서 시스템을 띄우면 `mock_world_node`가 만든 **임시 계획**이 쓰였다. 실제 planner·tracker가 계약 시험을 통과했으니 이제 연결할 차례다.

## 무엇을 바꿨나

```bash
# 실제 planner, 나머지는 mock
ros2 launch drop_bringup drop.launch.py use_mock_vision:=true use_mock_tracking:=true \
    use_mock_robot:=true use_mock_planner:=false
# 실제 planner + 실제 tracker(검출은 mock)
ros2 launch drop_bringup drop.launch.py use_mock_vision:=true use_mock_tracking:=false \
    use_mock_robot:=true use_mock_planner:=false
```

- **한 역할에 한 노드**: 실제 planner를 띄우면 mock의 임시 계획(`publish_plan`)을 자동으로 끈다. 둘 다 계획을 내면 로봇이 어느 계획을 따를지 엉킨다.
- **lifecycle 자동 활성화**: 실제 노드는 lifecycle 노드라 처음엔 "설정 안 됨" 상태다. launch가 `configure` 이벤트를 보내고, 노드가 `inactive`가 되면 `activate`를 보내도록 이어 두었다(`LifecycleNode` + `OnStateTransition`). mock은 `autostart` 파라미터로 스스로 켜지는 것과 달리 실제 노드는 바깥에서 켠다.
- **기본값**: 모든 역할의 기본이 실제 노드다(2026-11-12부터 `use_mock_planner`도 기본 `false`). mock 임시 plan은 tracking 대역이 내므로 `use_mock_planner:=true`는 `use_mock_tracking:=true`와 함께만 쓸 수 있고, 아니면 launch가 거부한다. 전체 대역 `mock.launch.py`가 이 조합을 명시한다.

## 시험

[test_real_bringup.py](../../src/drop_bringup/test/test_real_bringup.py)가 launch를 실제로 띄우고 ROS 그래프를 조사한다.

- planner·tracker가 `ACTIVE`가 되는지(`get_state` 서비스)
- `planner/compute_intercept` 서비스를 제공하는 노드, `drop/intercept_plan`을 내는 노드가 **정확히 하나**(`intercept_planner_node`)인지
- 일부러 mock 계획을 끄지 않게 바꾸면 발행자가 2개가 되어 시험이 실패하는지(변이 시험)

기존 mock bringup·I0 통합 시험도 다시 돌려 기본 동작이 그대로인지 확인했다. 새 기능을 넣을 때 "기존 것이 그대로인가"를 같이 보는 것을 회귀 시험이라고 한다.

## 다음

실제 tracker·planner를 쓴 상태로 오케스트레이터 한 사이클(검출 → 추적 → 계획 → mock 로봇)이 끝까지 가는지 보는 것이 다음 확인이다.

## 한 사이클을 실제로 돌려 보니(탐색)

mock 카메라·로봇 + 실제 tracker·planner + 오케스트레이터로 DropTask 한 번을 보냈다. 끝까지 가지는 못했지만, 실패할 때마다 원인을 하나씩 찾았다.

1. **자동 켜기가 너무 자주 켜졌다.** launch가 "inactive가 될 때마다" 켜도록 되어 있어서, 사람이 일부러 끈 노드도 다시 켜졌다. "configure 직후 한 번만"으로 고쳤다. 안전 규칙(꺼진 노드가 있으면 시작 안 함)이 실제 노드에서도 지켜지려면 필요하다.
2. **세계와 노드의 지도가 달랐다.** mock 세계는 컨베이어를 로봇 기준 (0.30, −0.25)에 두는데, 실제 tracker는 "컨베이어 = 로봇 원점"으로 계산했다. 같은 뚜껑을 다른 자리로 보니 추적이 끊겼다. 관측·도달 구간 값도 달라서 tracker가 "도달 구간에 들어오기 전 뚜껑"을 고르지 못했다. mock 세계를 쓸 때는 launch가 시나리오의 위치·구간을 실제 노드에 넘기게 했다.
3. **멀리 내다볼수록 예측은 흐려진다.** 그 뒤 계획 단계까지 갔지만 계획이 모두 "불확실"이었다. 뚜껑이 도달 구간에 오기 8~10초 전에 계획을 세웠는데, 속도 오차 0.9 mm/s가 그동안 쌓여 위치 오차 약 7.6 mm가 됐다(기준 0.5 mm). 코드 버그가 아니라 "언제 계획을 세우느냐"의 설계 문제라 팀이 정할 일로 남겼다.

배운 점: 가짜 부품(mock)끼리는 서로 맞춰져 있어 잘 돌지만, 진짜 부품을 하나씩 바꿔 끼우면 숨어 있던 가정(좌표, 구간, 시점)이 드러난다. 이것이 통합 시험을 단계별로(I0 → I2 → 실기) 하는 이유다.

## 해결: 계획은 늦게, 기준은 둘로

로컬 실험(설정 임시 변경 → 원복)으로 원인을 나눠 본 뒤, 팀장 승인을 받아 정식으로 반영했다([SPEC-100](../../specs/SPEC-100-orchestrator.md) REQ-ORC-013·014).

| 실험 | 계획 기준 | 계획 시점 | 결과 |
|---|---|---|---|
| 원래 | 0.5 mm | 뚜껑을 처음 볼 때(해제 약 10 s 전) | 실패(오차 7.6 mm) |
| 기준만 완화 | 3.25 mm | 처음 볼 때 | 실패(7.5 mm) |
| **둘 다** | **3.25 mm** | **도달 구간 진입 3 s 전** | **4/4 성공(2.7 mm)** |

- **기준을 둘로 나눈다**: 계획할 때는 "뚜껑 안에 들어갈 가망"(3.25 mm)만 본다. 공깃돌을 놓는 순간에는 로봇이 뚜껑 위에서 함께 움직이고 계획이 계속 갱신되므로 엄격한 0.5 mm를 그대로 쓴다. 전에는 같은 숫자를 두 곳에 함께 써서 계획 단계가 불필요하게 막혔다.
- **계획을 늦춘다**: tracker가 알려 주는 "도달 구간 진입 예상 시각"까지 3 s가 남을 때까지 기다린다. 멀리 내다볼수록 속도 오차가 위치 오차로 쌓이기 때문이다(0.9 mm/s × 10 s ≈ 9 mm, × 3 s ≈ 2.7 mm).
- **원인을 나눠 실험하기**: 두 가지를 동시에 바꿔서 성공하면 무엇이 효과였는지 모른다. 하나씩 바꾼 실험(기준만, 추정만)을 함께 돌려 둘 다 필요하다는 것을 확인했다.
- **실측 근거가 있는 값은 건드리지 않는다**: tracker 칼만 필터의 `sigma_accel`을 낮춰도 통과했지만, 그 값은 실제 벨트 영상으로 정한 것이라 바꾸지 않았다.
