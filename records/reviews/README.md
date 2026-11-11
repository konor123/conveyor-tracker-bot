# 검수·분석 보고서

AI 에이전트(Codex, Claude Fable)가 쓴 독립 검수·분석 보고서 원문이다. `Progress.md`·`records/worklog/`·`docs/learn/`이 근거로 가리킨다.

| 파일 | 날짜 | 수행 | 내용 |
|---|---|---|---|
| `codex-review-2026-11-09.md` | 11-09 | Codex | 미커밋 변경 독립 검수 |
| `codex-abort-fix-2026-11-09.md` | 11-09 | Codex | RETREAT 비동기 장애 중단 처리 수정 |
| `codex-m0-spec-review-2026-11-09.md` | 11-09 | Codex | M0 명세·시험 독립 리뷰(P1-1~P3-2) |
| `codex-m0-review-fixes-2026-11-09.md` | 11-09 | Codex | 위 지적 처리 |
| `codex-node-review-2026-11-09.md` | 11-09 | Codex | 노드 묶음 리뷰(F1~F8) |
| `codex-ros-fixes-2026-11-09.md` | 11-09 | Codex | ROS 통합 실패 수정과 PC 검증 |
| `codex-node-rereview-2026-11-09.md` | 11-09 | Codex | 노드 재검수(N1) |
| `codex-n1-fix-2026-11-09.md` | 11-09 | Codex | N1 수정 |
| `fable-review-2026-11-11.md` | 11-11 | Claude Fable | 커밋 검토(그리퍼 상주 루프·한 프로그램 사이클, M-1~M-4) |
| `m0-rereview-2026-11-11.md` | 11-11 | Codex | M0 독립 재검수(R1 발견) |
| `r1-review-2026-11-11.md` | 11-11 | Codex | R1(Stop 정지 유지) 수정 검수 |
| `fable-m0-risk-2026-11-11.md` | 11-11 | Claude Fable | M0 남은 위험 분석과 수정안 |

- `evidence/`: 보고서가 인용한 시험 로그·실행 스크립트(`w9-evidence-2026-11-09`, `codex-ros-evidence-2026-11-09`).
- 보고서 안의 `/tmp/...`, `~/ctb-...` 경로는 작성 당시 실행 장비의 임시 위치다. 저장소에는 없으며, 필요한 결과는 보고서 본문과 `evidence/`에 있다.
- 보고서는 작성 시점의 판단이다. 지적의 처리 결과는 같은 날 `records/worklog/`를 본다.
