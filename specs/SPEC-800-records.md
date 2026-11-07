# SPEC-800 records

- 버전: v0.1
- 근거: PLAN.md 10절
- REQ 영역: REC

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-11-07 | 시도 기록 JSON Schema, 성공률·Wilson 구간·분위수 규칙, 추적 행렬 REQ |

## 1. 목적·범위

시도·단계 기록의 형식과 게이트 보고서에 쓰는 통계 계산을 정한다. 기록 노드(`record_logger_node`)의 저장 동작은 그 노드를 만들 때 이 문서에 더한다.

## 2. 입력·출력(interface 버전)

SPEC-010 v0.2 기준.

| 기록 | 형식·위치 | 스키마·구현 |
|---|---|---|
| 시도 기록 | `records/trials/<date>/trials.jsonl` 한 줄 | `tools/eval/trial_schema.json`(JSON Schema 2020-12, PLAN.md 10.2절) |
| 단계 기록 | `records/trials/<date>/<cycle_id>.steps.jsonl` | `StepRecord` 필드 그대로 |
| 성능 측정 | `records/bench/<항목>-<date>.csv` + `.summary.txt` | `tools/bench/*.py` |
| 통계 | — | `tools/eval/stats.py`: `wilson_interval`, `success_rate`, `quantiles` |

## 3. 파라미터(단위·기본값·근거)

| 이름 | 값 | 근거 |
|---|---|---|
| Wilson 구간 z | 1.959963984540054(95%) | PLAN.md 13절 |
| 분위수 방식 | numpy `method="linear"` | TEST_SPEC.md TC-REC-005-01 |

## 4. 동작(상태·알고리즘)

- 시도 기록 필수 키: `trial_id`, `env`, `interface_version`, `scenario{name, seed}`, `assumed_params`, `gate`, `git_sha`, `mode`, `result`, `abort_level`(0~4), `reason_code`, `passes_skipped`(≥ 0), `cycle_time_s`. 그 밖의 키는 PLAN.md 10.2절 형식을 따르고 새 키를 더해도 된다.
- 성공률: 분자 = SUCCESS, 분모 = SUCCESS + FAILURE + UNKNOWN. CANCELED·SAFE_ABORT·PROTECTIVE_STOP·NOT_ATTEMPTED는 분모에 넣지 않고 따로 개수를 공개한다. n = 0이면 구간은 [0, 1].
- 분위수 보고: p50, p95, p99, 최대값.
- 실패·중단·미시도 기록을 지우지 않는다(PLAN.md 10.2절).

## 5. 시간 제약(주기·제한시간)

해당 없음(오프라인 분석). 기록 노드의 쓰기 지연은 노드 작성 시 정한다.

## 6. 실패 모드와 응답(중단 등급)

기록 실패는 중단 등급을 바꾸지 않는다. 스키마 검증에 실패한 시도 기록은 버리지 않고 `notes`에 오류를 적어 따로 보관한다(TBD: record_logger_node).

## 7. 성공 판정(측정 가능)

- REQ-REC-001: PLAN.md 10.2절 예시 기록은 스키마 검증을 통과하고, 필수 키 13개 중 하나라도 빠지거나 enum·범위를 벗어나면 검증이 실패한다(누락 검출 13/13).
- REQ-REC-002: 시간 예산 부족으로 통과 보내기를 소진한 사이클도 `result = NOT_ATTEMPTED`, `reason_code`, `passes_skipped`가 채워진 시도 기록 1줄을 남긴다.
- REQ-REC-003: 정상 1사이클은 실행한 단계 수와 같은 줄 수의 StepRecord JSONL을 남기고, 각 줄의 metric 키는 SPEC-100 4.4절 키와 같다.
- REQ-REC-004: Wilson 95% 구간은 45/50 → [0.7864, 0.9565], 50/50 → [0.9287, 1.0000], 0/20 → [0.0000, 0.1611](± 1e-4)이고, 성공률은 UNKNOWN을 분모에만 넣는다(SUCCESS 40, FAILURE 5, UNKNOWN 5 → 40/50).
- REQ-REC-005: 분위수 p50/p95/p99/최대값은 numpy `method="linear"`와 1e-12 이내로 같다.
- REQ-REC-006: `tools/spec/trace.py`는 TC 없는 REQ와 테스트 없는 TC를 모두 출력하고, 하나라도 있으면 종료 코드 ≠ 0이다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-REC-001-01(`tools/eval/test_trial_schema.py`), TC-REC-004-01·02(`tools/eval/test_stats.py`), TC-REC-005-01(같은 파일), TC-REC-006-01(`tools/spec/test_trace.py`) — U, M0. TC-REC-002-01·003-01은 I0(`record_logger_node`·`orchestrator_node` 작성 후).

## 9. 미결 사항

- `record_logger_node`: 저장 경로·파일 회전·스키마 검증 실패 처리.
- `trial_labeler.py`: 관찰자 판독 병합 형식.
