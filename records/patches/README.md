# 검토 대기 패치

`main`에 반영하지 않은 코드 변경을 검토용으로 보관한다. `src/`·`specs/`는 바뀌지 않는다. 반영하기로 하면 저장소 루트에서 아래처럼 적용하고, `AGENTS.md` 확인 명령(빌드·시험·`trace.py`)을 다시 돌린 뒤 커밋한다.

```bash
git apply --check records/patches/<파일>   # 적용 가능 여부만 확인
git am records/patches/<파일>              # 커밋 3개(작성자·메시지 포함)로 적용
```

| 파일 | 기준 커밋 | 내용 | 검증(적용 전 사본) | 근거 기록 |
|---|---|---|---|---|
| `2026-11-13-vision-crop-tracker-late-hover.patch` | `285c1e9` | (1) vision `process_frame`이 ROI 사각형 + 64 px만 `detect`에 넘김(REQ-VIS-011, TC-VIS-011-01·02, SPEC-200 v0.4) (2) tracker가 `t_state`보다 이른 검출도 수신으로 셈(REQ-TRK-015, TC-TRK-015-01, SPEC-300 v0.8) (3) 예측 해제 지점 위 비접촉 정지 시험 `tools/robot/hover_predicted.py`([OFF], 실기 `move_line` 사용)와 단위 시험 | 현장 노트북: 새 시험이 수정 전 실패·수정 뒤 통과, vision 144·tracking 139·static_pick_place 42 passed, `trace.py` TC 없는 REQ 0. CI 미실행 | `records/sessions/OFF-4-2026-11-13.md`, `records/trials/2026-11-13-hover-predicted/` |

- (3)은 실기 로봇을 움직이는 스크립트다. 현장에서 처음 쓴 코드이며 사용자 승인 아래 2회 실행했다(세션 기록). 반영 전 검토가 필요하다.
- 기준 커밋 뒤 `main`이 바뀌면 `git apply --check`로 먼저 확인한다.
