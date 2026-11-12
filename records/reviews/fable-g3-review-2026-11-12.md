# G3·G4 준비 커밋 검토 (2026-11-12)

- 검토자: coordinator(Claude Fable 5.2), 실행 장소 GCE
- 대상: `47edc1d..2862228` 6개 커밋
  - cy: `3659c8d`(고정 좌표계 static TF, 실제 tracker·planner bringup, 계획 시점·기준 분리), `2862228`(TC-INT-009-01)
  - fmv3654: `eb595e2`(belt_sync 기준 분리·`sync_metrics`), `814175f`(CI DNS 진단), `db201b7`(`belt_sync.json`), `df095b9`(시험 2개)
- 성격: G3(8절)·G4(9절)의 `[ON]` 준비 작업이다. G3 게이트 판정은 OFF-5 측정(독립 GT)이 있어야 하며 이 커밋으로는 바뀌지 않는다.

## 검증 근거(세 층으로 구분)

| 층 | 범위 | 결과 |
|---|---|---|
| CI run 43117206997(`2862228`) | `tools/spec`·`tools/eval`, orchestrator, drop_mocks, tracking, intercept_planner, drop_bringup(TC-INT-009-01, TC-PLN-012-01 포함) | 전 단계 success, 10분 38초 |
| GCE 재실행(이 검토) | `tools` 140 passed·6 skipped, `src/robot_control/test` 101 passed·2 skipped, `src/calibration/test/test_frames.py` 4 passed, `src/intercept_planner/test` 97 passed·1 skipped, `trace.py --check` 명세에 없는 ID 0 | 통과 |
| 팀원 노트북만(재현하지 않음) | `test_static_frames.py`(TC-CAL-012-01), RViz 육안 확인 | worklog 기록만 있음 |

CI는 `src/robot_control/test`와 `src/calibration/test`를 돌리지 않는다. 따라서 TC-CAL-011·012와 새 `test_sync_*`는 CI 근거가 없다.

CI 실패 4건(`eb595e2`~`df095b9`)은 모두 "Install dependencies"의 apt 색인 내려받기 실패(exit 100)로, 코드 원인이 아니다.

## 발견

### A. 설계·판정(현장 전에 정해야 함)

**A-1. 계획 단계 기준 3.25 mm는 잡음 없는 mock에만 맞춘 값이다(높음).**
I0에서 PLAN_INTERCEPT `position_std_m`은 2.7~2.8 mm로 기준까지 여유가 약 15%다. mock 검출은 잡음 0이다. 실제 벨트 영상 재생(`records/bench/belt-video-2026-11-07/replay-sigma-0.01.json`, `sigma_accel` 0.01)에서 2.8 s 앞 예측의 실제 오차는 XY p95 22~37 mm, 1 s 앞도 10~28 mm다. 즉 ① 실기에서는 필터가 보고하는 표준편차가 3.25 mm를 넘어 매 통과가 `UNCERTAIN`이 될 수 있고 ② 넘지 않더라도 보고 표준편차가 실제 오차(벨트 맥동)보다 한 자릿수 작아 이 판정이 실제 정확도를 보장하지 않는다. TC-INT-009-01은 배선(계획 시점·기준 전달)이 맞는다는 근거이고 G4 수치 근거가 아니다. OFF-5 bag으로 3 s 앞 `position_std`와 실제 오차를 함께 재서 `plan_lead_s`·`plan.position_std_max_m`을 정한다.

**A-2. 해제 게이트 0.5 mm는 아직 어디서도 판정하지 않는다(높음, 기지 한계).**
orchestrator는 `DynamicPlace.Goal.position_std_max = 0.0005`를 보내지만(TC-ORC-014-01) mock 로봇은 고정 값(`gate_position_std_m` 0.0003)을 돌려주고, 실제 `DynamicPlace`는 없다. `BeltSynchronizer.release_uncertainty_ok()`는 진단용이라고 스스로 밝힌다. planner 기준을 0.5 → 3.25 mm로 풀었으므로, 실제 `DynamicPlace`가 해제 순간 0.5 mm를 판정하기 전까지는 0.5 mm 기준이 사이클 어디에도 걸리지 않는다. 10절 `DynamicPlace` 구현의 완료 조건에 넣는다.

**A-3. `plan_lead_s` 3.0 s 고정은 빠른 벨트에서 시간 여유와 부딪친다(중간).**
판정 시점의 `t_available ≈ plan_lead_s + (x_exit − x_reach_start)/v`다. `t_required` 3.6~4.0 s, `margin_s` 1.0 s면 도달 구간 0.20 m(mock 배치)에서 v > 약 100~125 mm/s, 0.35 m(planner.yaml)에서 v > 약 175~220 mm/s일 때 매 통과가 `PASS_SKIP/TIME_MARGIN`이 된다. OFF-1 최대 속도가 127 mm/s였다. 시연 속도(≤ 50 mm/s)에서는 문제없다. G6 3속도 조건을 정할 때 함께 확인한다. 자동 시험은 20 mm/s 하나뿐이다.

**A-4. `t_enter_window`는 대상 선택 때 한 번만 받는다(낮음).**
WAIT_CAP에서 `TOO_EARLY`로 기다리는 동안 벨트 속도가 바뀌어도 진입 예상 시각을 갱신하지 않는다. 속도 변화는 `BELT_UNSTEADY`로 따로 잡히므로 영향은 작다.

### B. fmv3654 커밋(`eb595e2`~`df095b9`) — 절차 누락

- **B-1. 명세·기록 없음.** `SyncConfig.max_position_std_m` 기본값 1 mm → 3.25 mm, `release_position_std_max_m`, `release_uncertainty_ok()`, `sync_metrics`·`sync_metrics_demo`, `--config`·`--csv` 인자가 SPEC-500·TEST_SPEC에 없다. worklog·학습 문서도 없다.
- **B-2. TC 번호가 내용과 다르다.** `test_sync_metrics.py`는 TC-ROB-013-05(정지와 명시적 재무장), `test_sync_revision.py`는 TC-ROB-013-02(계획 수락 조건과 래치)로 표시했지만 내용은 CSV 기록·요약과 해제 불확실성 분리다. `specs/TRACE.md`가 그 번호 아래에 이 시험들을 싣게 된다. 새 TC를 TEST_SPEC에 추가하고 마커를 고친다.
- **B-3. `config/belt_sync.json`은 설치되지 않고 읽는 코드도 없다.** `setup.py`가 `config/*.yaml`만 설치한다. `--config`로 직접 줄 때만 쓰이며 값은 코드 기본값과 같다. 0.5 / 3.25 mm가 orchestrator.yaml, planner.yaml, belt_sync.py 기본값, belt_sync.json 네 곳에 있고 일치 검사는 없다.
- **B-4. CI "Diagnose DNS" 단계가 남아 있다.** 진단하려던 apt 설치 뒤에 놓여 실패 시 실행되지도 않는다. 지운다.
- **B-5. `summarize()`의 reason 집계가 O(n²)이다.** 60 s × 100 Hz(6000행)에서 수 초 걸린다. `collections.Counter`로 바꾼다(낮음).

### C. cy 커밋(`3659c8d`, `2862228`)

- **C-1. `drop.launch.py` 기본 인자 조합은 계획을 내는 노드가 없다(중간).** 기본값이 vision·tracking 실제, planner mock이다. mock plan은 `mock_world_node`가 내는데 vision·tracking이 둘 다 실제면 `mock_world_node`가 뜨지 않는다. 실제 vision·tracker를 쓸 때는 `use_mock_planner:=false`가 사실상 필수다. 기본값을 맞추거나 이 조합을 launch에서 거부한다.
- **C-2. 시나리오 기하 전달 조건(낮음).** `_scenario_params`는 `use_mock_vision:=true`일 때만 넘긴다. 실제 vision + mock tracking + 실제 planner 조합이면 mock track은 시나리오 좌표, planner는 config(항등)로 어긋난다. 쓰지 않는 조합이면 launch에서 막는다.
- **C-3. `is_rigid` 허용오차 1e-9(낮음).** 보정 결과를 소수 6자리로 적은 `T_base_conveyor`는 직교성 오차가 1e-6 수준이라 발행이 거부된다. OFF-3 보정값을 넣을 때 걸린다. 허용오차를 1e-6으로 하거나 값을 전체 자릿수로 저장한다.
- **C-4. `frames.launch.py`의 `conveyor_frame`과 mock 세계가 다르다(낮음, 작성자도 기록).** `frames.launch.py`는 tracking.yaml(항등), mock 세계의 tracker는 시나리오 변환 (0.30, −0.25)를 써서 RViz에서 `cap_<id>`와 예측 Marker가 약 0.39 m 어긋난다. `frames.launch.py`에 시나리오 인자를 주거나 mock 확인 절차에 적는다.
- **C-5. `camera_link`는 `frames.launch.py`에 없다.** `camera_tf_publisher`를 따로 띄워야 8절 static TF 항목의 나무가 완성된다.
- **C-6. 관측 구간 끝을 지난 검출이 새 track을 만든다(중간, 작성자 발견·미수정).** mock(135개 track)과 실영상 maximum 조건(관측 1~2회짜리 4개) 모두에서 보인다. 관측 수 30 미만이라 대상 선택에는 걸리지 않지만 track id·TF가 불어난다. tracking 담당이 구간 밖 검출의 새 track 생성 여부를 정한다.
- **C-7. 실기 실행(실시간, 축척 1.0)에서 WAIT_CAP 대상 없음 1회 미재현, RETREAT `STEP_TIMEOUT` 2회.** I0는 `duration_scale_ratio` 0.12로만 돈다. mock skill 소요시간이 실시간일 때 step 제한시간과 맞는지 한 번 확인한다.
- **C-8. `release_tcp_z_m` 0.10 확인 필요(작성자 기록).** 플랜지 기준이면 테두리 위 약 8 mm로 충돌 높이다. 실제 `DynamicPlace`가 이 값을 쓰기 전에 팀장이 확인한다.

잘된 점: SPEC → TEST_SPEC → 실패 시험 → 구현 순서와 변이 확인(자동 재활성화, `publish_plan`, `plan_lead_s`)이 기록돼 있다. 자동 활성화가 사람이 끈 노드를 다시 켜던 문제를 스스로 찾아 고쳤다. TF 사용 범위를 정적 검사로 고정했다.

## Progress.md 반영

- 8절 "미래 시각 TF 조회 금지": 완료(TC-SYS-006-01, CI).
- 8절 "TF Timestamp 동기화·역행 무효화": 완료(`TimePointZero` 0건 TC-SYS-006-01, tracking·planner 시계 역행 시험, CI).
- 9절 "실제 planner 사용 시 `publish_plan` 미사용": 완료(TC-PLN-012-01, TC-INT-009-01, CI).
- 8절 "static TF"·"RViz2": 유지. 남은 조건은 C-4·C-5, 실제 `T_base_conveyor`(OFF-3), `cap_<id>`와 Marker의 위치 일치 확인이다.
