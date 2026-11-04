# 계획 검토 요약 (2026-11-04)

- 대상: `package_plan.md`, `progress_plan.md` 초안
- 산출물: 저장소 루트 `package_plan.md`, `progress_plan.md`(v2로 교체, 초안은 git 이력의 첫 커밋), Astra 자문 원문 `deliverables/astra-plan-review-2026-11-04.md`
- 검토: Claude Code(coordinator) 독립 검토 + Codex Astra(gpt-6-astra, high) 기술 자문. 두 검토의 핵심 지적은 서로 일치했다.
- coordinator 직접 확인: doosan-robot2 Jazzy `dade65d`의 `servol/speedl(_rt)_stream` 토픽과 메시지 필드(Header 없음), tf2 `cache.cpp`의 미래 외삽 오류 메시지.
- 미확인: 현장 설치 드라이버·제어기 버전, 실제 서보 주기·지연, 그리퍼·단추 치수, 벨트 속도 범위. 실기 시험은 하지 않았다.

## 핵심 지적과 v2 반영

| # | 원안의 문제 | v2 반영 |
|---|---|---|
| 1 | Moving TF로 미래 목표를 만든다고 읽힘. tf2는 미래를 외삽하지 않음 | TF는 관측·시각화 전용, 예측은 `TrackedButton`→planner가 소유 (package §0·3, progress §7) |
| 2 | 연속 제어 인터페이스를 9단계에서야 확정 | 1단계를 G0 게이트로 강화: 스트림 선정·지연 측정·stale/cancel 시험, 미달 시 범위 축소 |
| 3 | 적층 순간 팔 가림에 대한 대책 없음 | `OBSERVED/COASTING/LOST`, τ_max, 로봇 마스크, 해제 게이트 |
| 4 | 3D 위치 변환 방식 불명확, 10 mm 단추에 Depth 부적합 | RGB 중심 + 보정 평면 광선 교차, Depth는 감시용 |
| 5 | 2지 그리퍼로 10 mm 단추 파지 가능성 미검증 | 핑거 어댑터 검증을 4절 필수 선행으로, 흡착 대안 비교 |
| 6 | 위에서 본 영상으로는 2단 적층을 판정할 수 없음 | 관찰자 판독을 기준 정답으로, D455 상단 영상 자동 판정은 보조(보조 카메라 없음), SUCCESS/FAILURE/UNKNOWN |
| 7 | 두 번째 파지 중 첫 단추가 구간을 벗어날 위험, 판정식 없음 | 시간 예산 식, 두 단추 동시 사전 검출, 상류 배치 |
| 8 | 메시지 필드·시각·단위 미정 | msg/action 필드 초안, SI 내부·어댑터 변환 |
| 9 | 루트 `tests/`, setup.cfg·resource 누락 | 패키지별 `test/`, ament_python 구성 명시 |
| 10 | 안전·보호 정지·통신 손실 상태 없음 | SAFE_ABORT / PROTECTIVE_STOP / WAIT_OPERATOR, 자동 원점 복귀 금지 |
| 11 | 합격 기준·두 문서 단계 대응 없음 | G0~G7 게이트와 제안 수치, 두 문서 공통 ID |
| 추가 | Static/Predicted/Moving Frame 비교가 12절에만 있음 | 10A(예측 타이밍 적층)를 중간 단계로 신설 |

## 사용자 확인이 필요한 사항

1. 프로젝트 기간·시연 일자(게이트별 일정 배정에 필요)
2. 원본 1절 항목("성공")이 모두 완료 상태인지
3. 그리퍼 모델과 단추 실측 치수(두께·구멍)
4. ~~측면 판정용 보조 카메라 추가 사용 가능 여부~~ → 없음(2026-11-04 사용자 답변). `package_plan.md` 6.1절에 반영
5. 제안 합격 기준 수치를 과제 목표로 채택할지
6. ~~v2를 원본 파일에 덮어쓸지~~ → 원본을 먼저 커밋한 뒤 v2로 교체해 커밋·푸시(2026-11-04 사용자 지시)
