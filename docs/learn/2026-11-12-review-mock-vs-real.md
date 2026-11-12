# mock에서 통과한 기준을 실제 자료와 견주기 (2026-11-12)

G3·G4 준비 커밋을 검토하면서 확인한 내용이다. 사실 기록은 [worklog](../../records/worklog/2026-11-12.md), 발견 목록은 [검토 보고서](../../records/reviews/fable-g3-review-2026-11-12.md)에 있다.

## 무엇을 봤나

실제 tracker·planner를 mock vision·robot에 붙여 한 사이클이 끝나도록, 계획 시점을 도달 구간 진입 3 s 전으로 늦추고(`plan_lead_s`) 계획 단계의 위치 표준편차 기준을 0.5 → 3.25 mm로 푼 변경이다. I0 시험(TC-INT-009-01)은 두 모드 모두 4/4 성공한다.

## 통과한 시험이 말해 주는 것과 말해 주지 않는 것

- mock 검출은 잡음이 0이다. 그때 planner가 보고한 표준편차가 2.7~2.8 mm였고 기준은 3.25 mm다. 여유가 15%뿐인 기준은 "이 mock에서 통과하도록 고른 값"에 가깝다.
- 같은 필터 설정(`sigma_accel` 0.01)으로 실제 벨트 영상을 재생한 결과가 이미 저장소에 있다(`records/bench/belt-video-2026-11-07/replay-sigma-0.01.json`). 2.8 s 앞 예측의 실제 오차는 XY p95 22~37 mm다.
- 필터가 보고하는 표준편차(수 mm)와 실제 오차(수십 mm)가 한 자릿수 다르다. 등속 모델이 벨트 맥동을 담지 못하기 때문이다. 표준편차 기준은 "필터가 스스로 얼마나 확신하는가"를 볼 뿐 실제 정확도를 보장하지 않는다.
- 그래서 이 시험은 배선(계획 시점 판정, 기준이 어디로 전달되는가)의 근거이고, 수치의 근거는 OFF-5 bag이다.

## 확인 방법

```bash
python3 tools/eval/replay_belt_tracking.py records/bench/belt-video-2026-11-07/*.caps.csv --sigma-accel 0.01 --json <저장소 밖 경로>
# prediction_error → horizon별 along/xy p95
```

## 배운 점·흔한 함정

- **기준값을 바꿀 때는 그 값이 어디서 실제로 판정되는지 끝까지 따라간다.** 해제 게이트 0.5 mm는 orchestrator가 `DynamicPlace` goal에 실어 보내지만, mock 로봇은 고정 값을 돌려주고 실제 `DynamicPlace`는 아직 없다. 계획 단계 기준을 풀면 0.5 mm는 당분간 아무 데서도 걸리지 않는다.
- **시간으로 정한 값은 속도가 바뀌면 다시 계산한다.** `plan_lead_s` 3 s일 때 판정 시점의 가용 시간은 `3 + 도달 구간 길이 / 속도`다. 구간 0.20 m, 필요 시간 4 s + 여유 1 s면 100 mm/s부터 매 통과를 보낸다.
- **CI가 초록이어도 어떤 폴더를 돌렸는지 본다.** 현재 CI는 `src/robot_control/test`와 `src/calibration/test`를 돌리지 않는다. 그 폴더의 새 시험은 작성자 PC 결과만 근거다.
- **TC 번호는 TEST_SPEC의 그 행과 내용이 맞아야 한다.** 다른 내용의 시험에 기존 번호를 달면 추적표(`specs/TRACE.md`)가 틀린 근거를 보여 준다.
- **CI 실패는 로그의 실패 단계부터 본다.** 이번 4건은 apt 색인 내려받기 실패(exit 100)로 코드와 무관했다.
