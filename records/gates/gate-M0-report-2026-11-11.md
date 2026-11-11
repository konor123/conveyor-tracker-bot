# M0 게이트 보고서 — Spec·시험 명세·계약·골격 (2026-11-11)

## 판정

**M0 통과.** 합격 기준 6개 항목 모두 `통과`다. 남은 위험과 미검증 사항은 4절에 적었다.

## 1. 조건

| 항목 | 값 |
|---|---|
| 날짜 | 2026-11-11 (KST) |
| git SHA | 코드 `4ac036f`(R1 반영), 기록 `8da2de6` |
| env | mock(I0: `mock_world_node` + `mock_robot_controller_node` + orchestrator·safety_supervisor·record_logger) |
| 실행 장소 | GCE(순수 Python 시험), officepc WSL Ubuntu-24.04 / ROS 2 Jazzy(빌드·C·I0, 임시 폴더, 시험용 ROS_DOMAIN_ID, 로봇 지령 경로 없음), GitHub Actions CI |
| PC 시험 소스 | tar SHA-256 `6adba784ad483e77be807239ae4d2ad5c37c737e19c221242aecd8b7f6b5e427`(= `4ac036f` 작업 트리) |
| calibration_id | 해당 없음(대역) |
| 실측값 | 공깃돌 팔각기둥 17~18 mm·높이 15 mm, 병뚜껑 안쪽 지름 31 mm·바깥 32 mm·테두리 벨트면 위 22 mm(`e_xy_max` 3.25 mm) |
| 그리퍼 펌웨어 | 해당 없음(대역) |
| 시나리오 | `nominal_20mms`, `robot_hang`, `protective_stop`, `heartbeat_loss`, `time_insufficient` |

## 2. 합격 기준 대비 판정

| 기준 | 판정 | 근거 |
|---|---|---|
| SPEC 6종(000·010·100·110·700·800)과 M0 대상 시험 케이스 리뷰 완료 | 통과 | 2026-11-09 Codex 독립 리뷰(P1-1~P3-2) → 처리 → 2026-11-11 Codex 독립 재검수에서 해결 확인. 남은 SPEC-800 문구 2건은 v0.7로 정합(커밋 `3c56005`). 보고서: `records/reviews/m0-rereview-2026-11-11.md` |
| interface 빌드 | 통과 | officepc WSL `colcon build --symlink-install` 11개 패키지(49 s). wire schema v0.2 변경 없음(R1은 runtime contract SPEC-010 v0.6) |
| 단위 시험에서 장애 10종 이벤트 열마다 기대 중단 등급 | 통과 | `src/orchestrator/test`(abort_policy·deadlines·fault_transitions·cycle_runner 포함) officepc WSL 700 passed, GCE 순수 809 passed·9 skipped(ROS 의존 skip은 통과로 세지 않음) |
| I0 정상 시나리오 20/20 | 통과 | `test_i0_nominal_20_cycles_and_records` 20/20 SUCCESS(officepc WSL 최종 실행, CI run 42960553456). Wilson 95% [0.839, 1.000] |
| ROS I0 장애 시나리오 4종의 중단 등급 동작 | 통과 | `robot_hang`(cancel 0.5 s 뒤 L4·Stop), `protective_stop`(감시 노드 Stop → WAIT_OPERATOR L4, 재시도 거부, 사유 있는 재개), `heartbeat_loss`(vision L2·tracker L3·controller L4), `time_insufficient`(통과 보내기·NOT_ATTEMPTED 기록). 추가로 TC-INT-008-01(skill 없는 WAIT_CAP에서 감시 노드 Stop → L4, 해제 전 재시작 L4, 해제·재개 뒤 SUCCESS). officepc WSL I0 9 passed, CI 통과 |
| 오케스트레이터 REQ 연결률 100% | 통과 | `tools/spec/trace.py --gate M0 --check`: REQ 52 / TC 97 / TC 없는 REQ 0 / 테스트 없는 TC 0 / 명세에 없는 ID 0 |
| `records/`에 StepRecord·trial 기록 생성 | 통과 | I0 nominal에서 trial 20줄 JSON Schema 검증, 단계 기록 12단계 이름·순서·키·값을 수신 StepRecord와 대조(TC-REC-003-01). `tools` 시험 145 passed(PC), 138 passed·6 skipped(GCE) |

## 3. 결과 분포(I0, 최종 실행 1회)

| 시험 | N | SUCCESS | FAILURE | SAFE_ABORT | PROTECTIVE_STOP | NOT_ATTEMPTED |
|---|---:|---:|---:|---:|---:|---:|
| nominal | 20 | 20 | 0 | 0 | 0 | 0 |
| robot_hang | 1 | 0 | 0 | 0 | 1 | 0 |
| protective_stop | 1(+재시도 1) | 0 | 0 | 0 | 1 | 1(WAIT_OPERATOR) |
| heartbeat_loss | 3 | 0 | 1 | 1 | 1 | 0 |
| time_insufficient | 2 | 1 | 0 | 0 | 0 | 1 |
| supervisor Stop(R1) | 3 | 1 | 0 | 0 | 2 | 0 |

UNKNOWN 0건. 오차·지연 분포는 M0 범위가 아니다(대역 가정값).

## 4. 남은 위험·미검증

- **I0 간헐 실패.** 최종 소스 직전 빌드에서 I0 묶음 4회 중 2회는 nominal이 `STEP_TIMEOUT`(seed 7, 18)으로 실패했다. 원인은 조사하지 않았다(사용자 지시로 반복 측정 중단). CI와 최종 PC 실행은 통과다.
- **감시 노드 기동 과도.** 기동 직후 감시 노드가 controller heartbeat를 0.30~0.34 s 늦게 받아 `CONTROLLER_HEARTBEAT_LOST` Stop을 내는 경우가 있다(R1 이전에도 존재). 대역은 연속 발행했다. I0는 기동 1.5 s 안정 대기 중 이 Stop만 해제한다(TEST_SPEC 0.3절). 실기 확인은 OFF-2 항목이다.
- **실제 노드·실기.** `robot_controller_node`는 fake backend로만 시험했다. 실제 보호정지 입력 연결과 해제 거부는 미확인이다(G0).
- **감시 연결(F4).** 관측 신선도·시각 역행·그리퍼·벨트 이벤트의 ROS 연결은 G0·G3·G4 게이트 조건이다. M0 통과는 이 연결 완료를 뜻하지 않는다.
- **독립 재검수.** R1 독립 검수의 P1·P2 수정은 coordinator 확인만 했다.

## 5. 근거 위치

- 진행·완료 조건: `Progress.md` 1절
- 작업 기록: `records/worklog/2026-11-09.md` ~ `2026-11-11.md`
- 검수 보고서: `records/reviews/`(`codex-m0-spec-review-2026-11-09.md`, `m0-rereview-2026-11-11.md`, `r1-review-2026-11-11.md`)
- 추적 행렬: `specs/TRACE.md`
