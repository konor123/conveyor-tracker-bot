# 전체 코드 설명 자료 교차 검토 (2026-11-14)

대상: [`docs/learn/2026-11-14-code-walkthrough.md`](../../docs/learn/2026-11-14-code-walkthrough.md). 작성은 coordinator(Claude Opus 5.5), 검토는 다른 모델 두 개에 맡겼다. 지적은 다수결이 아니라 저장소 코드·기록과 대조해 coordinator가 판정했다.

## 검토 방식

- 검토 고정본: 첫 초안(441줄, SHA-256 `42be220d2af1c1b9469111a6d9a23696dc2a511699b6f1fef16f713c898a5bf0`), 대조 커밋 `eed856a`.
- 검토자에게 준 것: 초안 본문, 저장소 위치, 할 일 네 가지(사실 대조, 상태 과장, 이해하기 어려운 곳, 빠진 것). coordinator가 의심하는 점은 적지 않았다. 사실 오류·상태 과장 지적에는 `파일:줄` 근거를 요구했다.

| 검토자 | 모델 | 상태 |
|---|---|---|
| OpenCode(읽기 전용 1회 호출) | `opencode/glm-5.3-flash-free` | 완료 |
| Antigravity(Orca worker) | `gemini-4-argon-high`. Opus 묶음 주간 한도 100%라 Opus 우선 규칙에서 Argon으로 대체 | 진행 중. 사용자 지시로 결과를 기다리지 않고 이 판을 먼저 냈다. 도착하면 다음 판에 반영한다 |

## OpenCode 지적과 판정

| # | 지적 | 판정 | 근거 | 조치 |
|---|---|---|---|---|
| A1 | "패키지 11개"를 README 근거로 인용했으나 README는 10개라고 씀 | 문서에는 해당 없음: 문서는 `src/`를 근거로 적었다. README 본문의 "ROS 2 패키지 10개"는 `src/` 11개와 맞지 않는 낡은 표기다 | `README.md` "구조" 절, `src/` | 문서 수정 없음. README 표기는 사용자에게 따로 알림 |
| A2 | "`next_state()` 43줄"은 함수가 아니라 파일 전체 줄 수 | 맞음 | `src/orchestrator/orchestrator/state_machine.py`(파일 43줄, `next_state`는 21줄부터) | 문장을 "한 파일(43줄)의 `next_state()`"로 고침 |
| B | 상태 과장 없음 | 동의 | `Progress.md`, `records/sessions/OFF-2-2026-11-14.md` | — |
| C1 | 해제 게이트 조건 13개가 한 문단에 몰려 있음 | 맞음 | `src/robot_control/robot_control/dynamic_place.py` `release_gate()` | 네 묶음 목록으로 풀고 이유 코드를 붙임 |
| C2 | 속도 표준편차·예측 앞선 시간·위치 표준편차 기준의 연결 설명이 없음 | 받아들이지 않음: 발표 자료의 범위를 넘는다 | — | — |
| C3 | CUSUM 옛값·새값을 섞어 쓰면 혼동 | 해당 없음: 문서는 현재 값(k 2, h 16)만 씀 | `src/tracking/config/tracking.yaml` | — |
| D1 | `belt_sync_preview` 그림자 노드 누락 | 맞음 | `src/robot_control/robot_control/belt_sync_preview.py` | 5.5절에 한 문장 추가 |
| D2 | `frames.launch.py` 누락 | 맞음 | `src/drop_bringup/launch/frames.launch.py` | 5.8절에 추가 |
| D3 | `drop_sim`이 빈 자리임을 표시 | 이미 표시함("설정 파일만, 장면 없음") | `src/drop_sim/` | — |
| D4 | 벨트 영상 예측 오차를 인용할 때 잠정 자료라는 단서 필요 | 맞음 | `records/bench/belt-video-2026-11-07/README.md`("후보 보정·압축 영상·단일 조명, G3 판정에 쓰지 않음") | 10절 답에 단서 추가 |
| D5 | CI의 `realtime_pc`는 기록만 함 | 이미 7.3절에 있음 | `.github/workflows/ci.yml` | — |
| E | 테스트 함수 약 780개, 커밋 `5dfc7cf`·`4ef0eb9`는 셸 없이 확인 못 함 | coordinator가 확인: `def test_` 780개(`src`·`tools`·`Hand-Eye-Calibration`), 두 커밋은 `main` 이력에 있음 | `git log` | — |

## coordinator가 작성 중 원문과 대조해 고친 것

- planner의 `revision` 갱신: 0.02 s마다가 아니라 새 `objects/tracked`가 올 때마다, 해제 시각은 고정한 채 갱신한다. 0.02 s 타이머는 만료 처리만 한다(`intercept_planner_node.py` `_tracked`·`_tick`, `plan_session.py` `update`).
- tracker 발행: 검출 메시지마다 내고, 입력이 0.2 s 넘게 끊겼을 때만 1/30 s마다 빈 프레임을 합성한다(`tracker_node.py` `_tick`).
- RELEASE·RETREAT 판정 조건 보완: 동기 유지, 그리퍼에 돌 없음, 작업영역 밖(`step_judges.py` `judge_release`·`judge_retreat`).

## 검토 뒤 바뀐 사실

검토를 맡긴 뒤 현장 세션이 기록을 두 번 더 올렸다(`eed856a` → `f654f62`, 14:19 KST). 간이 스크립트의 연속 성공이 17개에서 26개(벨트 33~56 mm/s)로 늘어 문서 수치를 고쳤다. 정식 경로 G4는 여전히 측정하지 않았다.
