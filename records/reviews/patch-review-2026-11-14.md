# 11-13 현장 실험 패치 검토 (2026-11-14)

- 대상: `records/patches/2026-11-13-vision-crop-tracker-late-hover.patch`(기준 `285c1e9`, 커밋 3개). (1) vision ROI crop(REQ-VIS-011) + tracker 늦은 검출 수신(REQ-TRK-015) (2)·(3) 예측 해제 지점 비접촉 정지 시험 `tools/robot/hover_predicted.py`.
- 검토: Codex(Orca worker, 읽기 전용), OpenCode(`oamp/auto`, 읽기 전용), coordinator(Claude Opus 5.5) 직접 읽기. 종합은 advisor. 실행 장소 GCE(읽기, `git apply --check` 통과). ROS·장비 시험은 하지 않았다(GCE에 cv2 없음).
- 근거 기록: `records/sessions/OFF-4-2026-11-13.md`, `records/trials/2026-11-13-hover-predicted/`.

## 1. 지적 사항

| 부분 | 지적 | 출처 | 판단 |
|---|---|---|---|
| tracker | 늦은 메시지마다 `frame_id` 확인 전에 `_last_received`를 갱신한다. stamp가 멈춘 흐름이나 다른 좌표계 흐름이 `input_timeout_s`보다 자주 오면 빈 프레임이 들어가지 않아 track이 늙지 않는다 | Codex P1 | 받아들임. `frame_id`를 먼저 보고, stamp가 앞으로 갈 때만 수신으로 센다 |
| tracker | `SelectTarget`에 현재 시각 기준 신선도 검사가 없어 남은 OBSERVED track을 고를 수 있다 | Codex P1 | 코드로 확인한 뒤 판단(반영 커밋 기록) |
| vision | ROI 꼭짓점을 반올림해 범위를 정하고, 실수 꼭짓점을 옮긴 뒤 `Roi.mask`가 다시 반올림한다. 짝수 반올림은 평행이동에 대해 불변이 아니어서 반 픽셀 꼭짓점과 홀수 offset에서 경계 픽셀이 바뀔 수 있다 | Codex P2 | 받아들임. 반올림한 정수 꼭짓점을 옮긴다 |
| vision | ROI가 영상 밖에 있으면 빈 영상이 `detect`로 가 예외가 날 수 있다. 범위는 양쪽 모두 영상 안으로 자른다 | Codex P2, OpenCode | 받아들임. 겹치는 영역이 없으면 `[]` |
| vision | crop 경계를 넘는 큰 물체(팔·손)는 전체 프레임에서는 큰 성분 하나라 크기 기준으로 빠지지만, 잘린 조각은 기준 안에 들어 가짜 검출이 될 수 있다 | OpenCode, Codex(같은 색 성분이 crop 밖으로 이어지는 경우) | 받아들임. 영상 경계가 아닌 crop 경계에 닿은 검출은 버린다(유효 대상물은 ROI 사각형 밖으로 64 px까지 닿을 수 없다). 실제 프레임에서 crop 켬·끔을 비교한다 |
| vision | 정적 투입(`static_pick_place`)도 `process_frame`을 쓴다. crop을 기본으로 켜면 11-13 실기에서 검증한 정적 경로가 바뀐다 | coordinator | crop은 `vision_node`만 켠다. 끄면 이전 `detect` 호출 그대로 |
| hover | 현재 자세→대기·정지 지점의 직선 경로, 이동 높이·속도·가속, park 끝점과 IK를 검사하지 않는다. 3번 커밋은 `NO_USABLE_PLAN`에만 park 복귀를 넣었고 로봇 오류·Ctrl+C·영상 변환 예외 뒤에는 돌아가지 않는다. 불확실한 지령 뒤 무조건 park 이동으로 고치면 안 된다 | Codex P1, OpenCode | 반영하지 않음. 실기 동작이라 원격 검증이 안 된다. 다시 쓸 때 조건을 `docs/operations/site-checklist-2026-11-14.md` 5절에 적었다 |
| hover | 통과 시각을 고른 track과 상관없이 모든 뚜껑 검출로 계산해, 뚜껑이 둘 이상이면 다른 뚜껑의 통과를 잴 수 있다. 단위 시험은 보간·이동 시간 계산만 본다 | Codex P2, OpenCode | 같음(반영하지 않음) |

## 2. 비접촉 시험 −0.81 s 해석

OpenCode가 `run-02.jsonl`로 계산했고 coordinator가 같은 값으로 확인했다. 계획 시각 속도 33.5 mm/s(`cap_v_mps`), 해제까지 2.8 s(`release_in_s`) → 남은 거리 약 94 mm. 실제 통과는 0.814 s 일찍(`cross_minus_release_s`)이었으므로 평균 약 47 mm/s다. 계획 0.21 s 전 같은 track의 계획이 `UNCERTAIN`으로 거절됐다(`plan_rejected`). 벨트를 손으로 다시 돌린 직후 속도가 바뀌는 중에 낸 계획으로 본다(1회, 검출 원시 표본은 남지 않음).

## 3. 결정

- 반영: (1)의 vision·tracker. 원 작성자 커밋을 그대로 적용하고 위 보강을 별도 커밋으로 둔다. 사무실 PC WSL 시험(vision·tracking·static_pick_place·replay·실제 controller I0)을 통과한 뒤에만 푸시한다.
- 반영 안 함: (2)·(3) hover 도구. 패치 파일에 그대로 둔다.
- 오늘 하지 않음: DDS 설정, RELEASE 제한, 나머지 진입점(`vision`·`calibration`·`static_pick_place`)의 종료 신호 처리. 11-13에 장치를 잡고 있던 카메라 노드는 RealSense C++ 드라이버(`camera.launch.py`가 `rs_launch.py`를 포함)라 Python 종료 처리와 관계없다.
