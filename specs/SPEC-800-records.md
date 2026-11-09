# SPEC-800 records

- 버전: v0.6
- 근거: PLAN.md 10절
- REQ 영역: REC

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.6 | 2026-11-09 | 일반 설치 schema 및 저장 오류 운용 계약 |
| v0.5 | 2026-11-09 | 단계 기록의 원본 ROS 시각 보존·역행 허용 명시 |
| v0.4 | 2026-11-09 | 기록 I/O 실패·직렬화·잘못된 summary 및 schema 부재 처리 |
| v0.3 | 2026-11-09 | M0 검증 범위·게이트 및 독립 계약 기대값 정합성 |
| v0.2 | 2026-11-09 | 일별 개발 지표 집계·미측정·날짜별 저장 규칙 |
| v0.1 | 2026-11-07 | 시도 기록 JSON Schema, 성공률·Wilson 구간·분위수 규칙, 추적 행렬 REQ |

## 1. 목적·범위

시도·단계 기록의 형식과 게이트 보고서에 쓰는 통계 계산을 정한다. 기록 노드(`record_logger_node`)의 JSONL 저장 경로와 오류 보관 규칙을 포함한다.

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 / runtime contract v0.4 기준.

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
- 저장 루트는 `records_dir`(기본 `DROP_RECORDS_DIR` 또는 실행 cwd의 `records/trials`)다. 날짜는 명시한 `date`, 생략 시 프로세스 호스트의 로컬 날짜를 기동 시 고정한다. 자정 자동 회전은 없다. 단계는 `task/step_record`, 시도는 orchestrator의 `last_trial_json` 파라미터 이벤트에서 기록하며 같은 직전 trial_id는 중복 저장하지 않는다.
- 단계 `t_start_s`·`t_end_s`는 수신 StepRecord의 원본 ROS 시각을 그대로 보존한다. ROS 역행 시 `t_end_s < t_start_s`도 유효하며, 기록을 단조 시각처럼 재정렬하거나 고치지 않는다.
- 실패·중단·미시도 기록을 지우지 않는다(PLAN.md 10.2절).

## 5. 시간 제약(주기·제한시간)

분석은 오프라인이다. 기록 노드는 이벤트 수신마다 한 줄을 append하고 flush한다. 쓰기 지연의 정량 상한은 규정하지 않는다.

## 6. 실패 모드와 응답(중단 등급)

기록 실패는 중단 등급을 바꾸지 않는다. 스키마 검증에 실패한 시도 기록은 버리지 않고 `notes`에 오류를 적어 `trials.invalid.jsonl`에 보관한다. JSON 파싱·타입·변환 실패도 원문과 오류를 invalid 파일에 보관한다. 잘못된 경로는 경고 후 해당 기록을 건너뛴다. 파일 I/O 오류는 `storage_error`와 경고로 노출하며 자동 재시도·복구 보장은 없다.

### 6.1 기록 실패와 직렬화 계약

Logger의 metadata 갱신과 JSONL append는 같은 lock으로 직렬화한다. 파일 생성·쓰기·flush 실패는 callback 밖으로 전파하지 않고 `storage_error`와 경고에 남긴다. trial 중복 확인 상태는 정상 append·flush 성공 뒤에만 확정하므로 실패한 trial의 재전달을 허용한다. 자동 재전달이나 전원 장애에 대한 exactly-once 보장은 없다.

summary의 root·타입·변환 오류는 원문과 오류를 `.invalid.jsonl`에 격리하고 정상 trial 중복 상태에 반영하지 않는다. schema 부재는 검증 오류로 처리하고 정상 trials 파일에 기록하지 않는다. 일반 install은 schema를 별도로 배치해야 하며, schema가 없는 실행은 검증 성공으로 간주하지 않는다.

### 6.2 설치·저장 오류 운용 계약

일반 copy install은 orchestrator share/schema/trial_schema.json을 사용하며 저장소·symlink 실행의 schema 탐색도 지원한다. 파일 저장 오류는 warning과 노드 내부 `storage_error` 속성으로 노출한다(ROS diagnostic 발행은 없음). 자동 재전달·durable queue는 없으므로 운용자가 원본과 기록 수를 대조하고 필요한 trial을 재전달한다. 정상 append·flush 뒤에만 dedup을 확정한다. flush·close 도중 오류나 전원 장애에서는 부분 줄·중복 기록이 생길 수 있으며 exactly-once와 전원 장애 내구성을 보장하지 않는다.

## 7. 성공 판정(측정 가능)

- REQ-REC-001: PLAN.md 10.2절 예시 기록은 스키마 검증을 통과하고, 필수 키 13개 중 하나라도 빠지거나 enum·범위를 벗어나면 검증이 실패한다(누락 검출 13/13).
- REQ-REC-002: 시간 예산 부족으로 통과 보내기를 소진한 사이클도 `result = NOT_ATTEMPTED`, `reason_code`, `passes_skipped`가 채워진 시도 기록 1줄을 남긴다.
- REQ-REC-003: 정상 1사이클은 실행한 단계 수와 같은 줄 수의 StepRecord JSONL을 남기고, 각 줄의 metric 키는 SPEC-100 4.4절 키와 같다.
- REQ-REC-004: Wilson 95% 구간은 45/50 → [0.7864, 0.9565], 50/50 → [0.9287, 1.0000], 0/20 → [0.0000, 0.1611](± 1e-4)이고, 성공률은 UNKNOWN을 분모에만 넣는다(SUCCESS 40, FAILURE 5, UNKNOWN 5 → 40/50).
- REQ-REC-005: 분위수 p50/p95/p99/최대값은 numpy `method="linear"`와 1e-12 이내로 같다.
- REQ-REC-006: `tools/spec/trace.py`는 TC 없는 REQ와 테스트 없는 TC를 모두 출력하고, 하나라도 있으면 종료 코드 ≠ 0이다.

- REQ-REC-007: 일별 개발 지표 도구는 여러 pytest JUnit XML의 testcase level 속성(U/C/I0/I1)을 기준으로 통과/전체를 집계한다. failure·error·skipped는 전체에 포함하되 통과에 포함하지 않는다. U는 unit, C는 contract, I0는 ros_i0, U와 I1 합계는 unit_replay이며 수준 없는 시험은 집계하지 않는다. 해당 수준의 입력 시험이 없으면 통과·전체는 모두 빈칸이다. REQ 연결률은 trace.build의 TC 연결 REQ 수/전체 REQ 수 × 100(소수 둘째 자리)이며 REQ가 없으면 빈칸이다. 날짜(기본 KST 오늘)·현재 git HEAD SHA와 gate_status·open_issues·robot_hours 인자를 PLAN.md 10.3절 헤더 순서로 기록하고 같은 날짜의 모든 행을 한 행으로 교체한다. integration_stage는 별도 명시한 I0~I5 값만 기록하며 생략한 운영 값은 빈칸이다. 잘못된 XML·날짜·음수 또는 비유한 운영 수치·기존 CSV 헤더 불일치는 출력 파일을 바꾸지 않고 실패한다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-REC-001-01(`tools/eval/test_trial_schema.py`), TC-REC-004-01·02(`tools/eval/test_stats.py`), TC-REC-005-01(같은 파일), TC-REC-006-01(`tools/spec/test_trace.py`) — U, M0. TC-REC-002-01·003-01은 I0(실제 `record_logger_node`·`orchestrator_node` 경로).

TC-REC-007-01·02·03·04·05(`tools/eval/test_devlog.py`) — U, M0.

## 9. 미결 사항

- `trial_labeler.py`: 관찰자 판독 병합 형식.
