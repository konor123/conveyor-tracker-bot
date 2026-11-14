# 검토 대기 패치

`main`에 반영하지 않은 코드 변경을 검토용으로 보관한다. `src/`·`specs/`는 바뀌지 않는다. 반영하기로 하면 저장소 루트에서 아래처럼 적용하고, `AGENTS.md` 확인 명령(빌드·시험·`trace.py`)을 다시 돌린 뒤 커밋한다.

```bash
git apply --check records/patches/<파일>   # 적용 가능 여부만 확인
git am records/patches/<파일>              # 커밋 전부(작성자·메시지 포함)로 적용
# 일부만 쓸 때: git mailsplit -o<빈 폴더> records/patches/<파일> 뒤 git am <빈 폴더>/000N
```

| 파일 | 기준 커밋 | 내용 | 검증(적용 전 사본) | 근거 기록 | 반영 상태 |
|---|---|---|---|---|---|
| `2026-11-13-vision-crop-tracker-late-hover.patch` | `285c1e9` | (1) vision `process_frame`이 ROI 사각형 + 64 px만 `detect`에 넘김(REQ-VIS-011, TC-VIS-011-01·02, SPEC-200 v0.4) (2) tracker가 `t_state`보다 이른 검출도 수신으로 셈(REQ-TRK-015, TC-TRK-015-01, SPEC-300 v0.8) (3) 예측 해제 지점 위 비접촉 정지 시험 `tools/robot/hover_predicted.py`([OFF], 실기 `move_line` 사용)와 단위 시험 | 현장 노트북: 새 시험이 수정 전 실패·수정 뒤 통과, vision 144·tracking 139·static_pick_place 42 passed, `trace.py` TC 없는 REQ 0. CI 미실행 | `records/sessions/OFF-4-2026-11-13.md`, `records/trials/2026-11-13-hover-predicted/` | (1)·(2) 보강해 반영(커밋은 `records/worklog/2026-11-14.md`), (3) 반영 안 함(`records/reviews/patch-review-2026-11-14.md`) |
| `2026-11-14-g4-simple-demo.patch` | `4ef0eb9` | 간이 이동 뚜껑 투입 도구(`tools/robot/` 9개 파일, 커밋 18개): `g4_simple.py`(상주 루프로 파지 → 벨트 옆 대기 → 검출로 도착 시각 추정 → 비동기 `move_line`으로 해제 지점 → 도착 0.6 s 전 예약 개방 트리거, 해제 지점은 겨냥점보다 벨트 방향 +5 mm, `--count 0`은 park 복귀 없이 매트가 빌 때까지, [OFF] 실기 지령), `g4_ui.py`·`g4_ui.sh`(한 개 / 전부 / 원위치 / 중지 버튼 창), `run_recorder.py`(시도 영상·검출 녹화), `g4_analyze.py`(검출 기록으로 판정, 장면 모음), `hover_predicted.py`(11-13 패치의 0002·0003과 같은 내용)와 단위 시험 | 현장 노트북: `tools/robot` 단위 시험 29 passed, pyflakes 통과. 실기에서 놓은 돌 30개 연속 성공(run-13~28, 버튼 창). 중지 버튼 수정(대기 중 중지, 쥔 돌 내려놓기)은 계획만으로 확인, 실기 미실행. 버튼 창의 버튼 4개·속도 3단계·녹화 선택을 실기에서 실행. REQ·TC 없음, CI 미실행 | `records/sessions/OFF-2-2026-11-14.md`, `records/trials/2026-11-14-g4-simple/`, `records/media/2026-11-14/` | 반영 안 함(검토 대기) |

- (3)은 실기 로봇을 움직이는 스크립트다. 현장에서 처음 쓴 코드이며 사용자 승인 아래 2회 실행했다(세션 기록). 반영 전 검토가 필요하다.
- 기준 커밋 뒤 `main`이 바뀌면 `git apply --check`로 먼저 확인한다.
- `2026-11-13-vision-crop-tracker-late-hover.patch`의 첫 커밋은 `main`에 들어갔다(`2e50a6d`, 보강 `43e19b7`). 파일 전체에 `git am`을 쓰면 첫 커밋에서 실패하므로 남은 hover 커밋(0002·0003)만 나눠 적용한다.
- `2026-11-14-g4-simple-demo.patch`는 현장에서 쓴 실기 스크립트다(REQ → TC → 시험 순서를 따르지 않았다). 사용자 입회 아래 실행했다(세션 기록). 팔·그리퍼를 직접 움직이며 orchestrator·`robot_controller_node`·planner를 거치지 않는다. `main`의 `tools/robot/`에 같은 이름의 파일이 없을 때 `git am`으로 전체를 적용한다(11-13 패치의 hover 커밋을 이미 적용했다면 첫 두 커밋은 건너뛴다).
