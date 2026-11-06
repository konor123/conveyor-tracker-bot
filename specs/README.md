# specs

요구사항 명세. 작업 순서와 규칙은 `PLAN.md` 8절, 시험 케이스 원본은 저장소 루트 `TEST_SPEC.md`다.

## 파일

| 파일 | 범위 |
|---|---|
| `SPEC-000-system.md` | 범위, 용어, 좌표·시각·단위 규약 |
| `SPEC-010-interfaces.md` | msg/srv/action 필드·단위·의미, QoS, revision·epoch 처리, lifecycle 비활성 시 응답, 버전 |
| `SPEC-100-orchestrator.md` | 상태·전이·단계 판정 |
| `SPEC-110-abort-timeout.md` | 중단 등급·감시·제한시간·시계 정책 |
| `SPEC-200-vision.md` | 공깃돌·병뚜껑 검출, 광선-평면 교차, 로봇 마스크, 투입 판정 |
| `SPEC-300-tracking.md` | 칼만 필터, ID·재획득, 속도 변화 검출, 관측 TF |
| `SPEC-400-planner.md` | 예측, 시간 예산, 해제 시각·위치, 도달성 |
| `SPEC-500-robot-control.md` | Doosan 스트림·move_line 계약, 그리퍼 상주 루프 경로, 추종 제어, L0 |
| `SPEC-600-calibration.md` | 내부·외부·평면·TCP 보정, 검증 점 오차 |
| `SPEC-700-mocks.md` | mock 동작·시나리오·계약 테스트 |
| `SPEC-800-records.md` | 기록 스키마, 분석 지표 |
| `TRACE.md` | `tools/spec/trace.py`가 생성하는 REQ ↔ TC ↔ 테스트 행렬 |
| `snapshots/` | `ros2 interface show` 출력 스냅샷(TC-IF-001-01) |

## 템플릿

각 SPEC은 다음 9개 절을 모두 채운다. 정하지 못한 값은 `TBD(측정 G0)`처럼 결정 시점을 적는다.

1. 목적·범위 / 2. 입력·출력(interface 버전) / 3. 파라미터(단위·기본값·근거) / 4. 동작(상태·알고리즘) / 5. 시간 제약(주기·제한시간) / 6. 실패 모드와 응답(중단 등급) / 7. 성공 판정(측정 가능) / 8. 검증 방법(TC ID, 게이트) / 9. 미결 사항

## ID 체계

- 요구사항: `REQ-<영역>-<번호>`(번호 3자리). 정의는 줄 머리 `- REQ-ORC-012: <내용>` 형식으로 쓴다(`trace.py`가 이 형식을 읽는다). 요구사항마다 정량 기준을 하나 이상 둔다.
- 시험 케이스: `TC-<영역>-<REQ 번호>-<순번>`. `TC-ABT-012-01`은 `REQ-ABT-012`를 검증한다. 내용은 `TEST_SPEC.md`에만 쓰고 SPEC에서는 ID만 참조한다.
- 영역: `SYS` `IF` `ORC` `ABT` `VIS` `TRK` `PLN` `ROB` `GRP` `CAL` `MCK` `REC` `INT` `SIM` `BEN` `EXP`

## 버전

- SPEC마다 `v0.x` 버전과 변경 이력을 둔다. 구현이 SPEC과 달라져야 하면 SPEC부터 고친다.
- interface를 바꾸면 SPEC-010 버전을 올리고, `snapshots/`, mock, 계약 테스트를 같은 커밋에서 고친다.

## 추적 행렬

```bash
python3 tools/spec/trace.py              # specs/TRACE.md 갱신, 누락이 있으면 종료 코드 1
python3 tools/spec/trace.py --area ORC,ABT --check
```
