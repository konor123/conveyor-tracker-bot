# 발표 전 원격 정리: 검토 의견 종합, ROI 잘라 계산, tracker 입력 생존 판정

*ROS 2와 컴퓨터 비전이 처음인 팀원을 위한 학습 문서. 발표일(2026-11-14) 새벽, 장비 앞에 가기 전에 원격으로 진행한 검토·수정 내용을 "무엇을, 왜, 어떻게 확인했는지" 중심으로 정리한다.*

---

## 1) 세 의견을 어떻게 합쳤나

### 무엇을 했고 왜 했나

11-13 저녁 현장 점검([OFF-4-2026-11-13.md](../../records/sessions/OFF-4-2026-11-13.md))에서 라이브 트래킹 문제 두 가지가 발견됐고, 현장 실험 사본에만 임시로 수정한 뒤 검토 패치로 저장했다([2026-11-13-vision-crop-tracker-late-hover.patch](../../records/patches/2026-11-13-vision-crop-tracker-late-hover.patch)). 발표일(2026-11-14) 전에 장비 앞에 가기 전까지 코드를 검증된 상태로 만들기 위해, 코디네이터가 세 경로로 의견을 모았다:

- **Codex** (읽기 전용 리뷰) — 패치를 쪼개라고 했다. 그리고 각 부분의 숨은 문제를 지적했다.
- **OpenCode** (우선순위 + 리뷰) — 검토 뒤 패치를 반영하라고 했고, crop 경계에서 잘린 큰 물체 문제와 현장에서 본 hover 오차의 원인을 제시했다.
- **어드바이저** (종합) — 세 의견을 하나로 묶었다.

결과물이 [patch-review-2026-11-14.md](../../records/reviews/patch-review-2026-11-14.md)이고, 여기서 검증된 버전으로 병합했다.

### 의견이 갈리면 어떻게 결정했나 — 두 가지 실제 사례

**사례 1: hover 도구는 아예 병합하지 않기로 했다.** OpenCode는 hover 도구까지 반영하되 로봇 오류 경로에 park 복귀를 넣자고 했고, Codex는 도구가 현재 자세에서의 경로·park 자세를 검사하지 않으며 "결과가 불확실한 지령 뒤에 무조건 park로 움직이게 고치는 것"은 오히려 위험하다고 했다. 결론은 어느 쪽 고침을 고르는 대신, **이 도구는 실제 팔을 움직이고 원격으로 검증할 수 없으므로 병합하지 않는다**였다. 다시 쓸 때의 조건은 오늘 점검표 5절에 적었다. 참고로 OpenCode는 이 도구로 잰 −0.81 s 오차를 벨트 가속으로 설명했다(계획 33.5 mm/s, 실제 약 47 mm/s, 계획 0.21 s 전 같은 track의 계획이 `UNCERTAIN`으로 거절됨).

**사례 2: 종료 신호 처리 확장은 근거를 확인해 미뤘다.** shutdown 처리를 필드 노드들로 확장하자는 의견이 있었지만, 11-13에 남아 있던 프로세스는 RealSense C++ 드라이버였다(`camera.launch.py`가 `rs_launch.py`를 포함). C++ 드라이버 문제이므로 Python shutdown 처리와는 무관하다 — 확장 근거가 사라진다.

### 코디네이터가 추가로 찾아낸 것

두 리뷰어 모두 놓친 부분: `static_pick_place`도 `vision.pipeline.process_frame`을 호출한다. crop을 기본 켜기로 하면 **로봇에서 검증까지 마친 정적 데모 경로(e509c87)의 동작이 바뀐다**. 그래서 최종 설계는 crop을 기본 끄기로 하고, 설정이 있을 때만 켜지게 했다(2절). 검토 의견을 합치는 일은 리뷰어가 말한 것뿐 아니라 "이 변경이 누구에게 영향을 주는가"를 스스로 확인하는 일이다.

> **개념 정리 — 리뷰 의견 합치기:** 리뷰 의견은 (1) 받아들이기, (2) 근거를 확인하고 기각하기, (3) 절충이 아니라 "병합하지 않기"로 해결하기, (4) 리뷰어가 못 본 영향 범위를 직접 확인하기 — 네 가지로 정리된다. 갈림길에서 항상 물어야 할 질문: *이 변경을 원격으로 검증할 수 있는가? 검증된 경로를 바꾸지 않는가?*

---

## 2) vision ROI 잘라 계산

### 문제: 왜 느렸나

`detect`는 1280x800 프레임 **전체**에 numpy 연산을 한다. 현장에서 프레임당 약 160 ms, 사무실 PC에서 저장 프레임 기준 135 ms였다. 관심 영역(ROI) 밖은 계산할 필요가 없는데도 전부 계산하는 셈이다.

### 최종 설계 (REQ-VIS-011, [SPEC-200 v0.5](../../specs/SPEC-200-vision.md))

- **crop은 `SceneConfig.crop_margin_px`가 설정된 경우에만** 켠다. `vision_node`는 64 px로 설정한다.
- **기본은 끔.** 기본이 꺼져 있으면 `static_pick_place`는 예전의 전체 프레임 호출을 그대로 쓴다 — 로봇에서 검증된 데모 경로(e509c87)를 그대로 유지하기 위해서다.

### 하드닝: 자르기가 만드는 새로운 함정들

"ROI만 잘라서 계산하면 끝"이 아니라, 자르기 자체가 새 문제를 만든다. 패치에 들어간 방어 로직:

1. **반올림 먼저, 이동은 나중.** 이미 반올림된 정수 다각형을 **평행이동**한다. 절반 픽셀 좌표를 먼저 이동하고 반올림하면 안 된다(아래 반올림 함정 참고).
2. **이미지 경계로 클램프.** ROI를 이미지 밖으로 자르지 않게 눌러 담는다.
3. **이미지와 겹치지 않는 ROI는 검출 없음을 반환.** 빈 crop이 이상한 결과를 만들지 않게 한다.
4. **crop 경계(이미지 경계가 아닌)에 닿은 검출은 버린다.** ROI 중앙에 있는 정상 물체는 ROI 상자에서 64 px 더 나갈 수 없다. 그러므로 crop 경계에 닿은 컨투어는 **crop에 잘린 큰 컴포넌트**(팔, 손 같은 것)라고 볼 수 있다. OpenCode가 지적한 "crop 가장자리에서 잘린 큰 물체가 크기 필터를 통과할 수 있다"를 이 규칙으로 막았다.

> **개념 정리 — 반올림이 중요한 이유 (ties-to-even):** numpy의 `np.round`(Python `round`도 같다)는 .5를 짝수 쪽으로 반올림한다(banker's rounding). 그래서 `round(140.5)`는 140이지만, 같은 다각형을 37 px 옆으로 옮긴 `round(103.5) + 37`은 141이 된다. 즉 **반올림은 평행이동 불변이 아니다** — 같은 모양의 ROI라도 위치에 따라 1 px 크기가 달라질 수 있다. 그래서 "좌표를 옮긴 뒤 반올림"이 아니라 "반올림한 뒤 좌표를 옮긴다"가 규칙이다.

### 어떻게 확인했나

- **단위 테스트**([test_pipeline.py](../../src/vision/test/test_pipeline.py)): crop 켬/끔을 비교하며 ① 홀수 오프셋이 있는 절반 픽셀 정점, ② 이미지 네 변에 모두 닿는 ROI, ③ 이미지 밖 ROI, ④ ROI를 150 px 넘어 이어지는 캡 색 띠 — 네 경우를 검사한다.
- **사무실 PC WSL 시험**: vision 149, static_pick_place 42, replay 2, 실제 controller I0 8, 대역 I0 18 passed.
- **실제 프레임 82장 비교**: 저장해 둔 실제 1280x800 프레임 82장에 auto ROI(정적·vision_node 설정 양쪽)를 적용해 켬/끔을 비교했다. 검출이 144개로 동일했고, 평균 시간은 끔 135-136 ms vs 켬 26-27 ms(사무실 PC WSL). 비교 도구와 기록은 [vision-crop-2026-11-14/](../../records/reviews/evidence/vision-crop-2026-11-14/)(`compare_crop.py`, `frames.txt`, `result.txt`)에 있다.

> **함정 — "동일하다"고 주장할 때는 뭘 확인했는지 써야 한다.** 범용 동치 주장("crop해도 항상 같다")은 할 수 없다. SPEC-200은 일반 동치를 주장하는 대신, 검사한 케이스와 82장 프레임을 명시하도록 바뀌었다.

---

## 3) tracker 입력 수신 판정

### 현장에서 발견된 버그 (REQ-TRK-015, [SPEC-300 v0.9](../../specs/SPEC-300-tracking.md))

11-13 현장 점검에서: `input_timeout_s`보다 늦게 온 검출 때문에 `_tick`이 빈 프레임을 넣었고, 그 빈 프레임이 `t_state`를 앞으로 밀었다. 그러면 **이후의 모든 검출이 '늦은 것'이 되어 버려졌고**, `_last_received`는 갱신되지 않아 **트래커가 영영 회복하지 못했다**. 한 번 늦으면 영구 사망에 가까운 상태였다.

패치 초안의 수정: 늦은 검출도 "받은 것"으로 센다. 그런데 리뷰가 이 수정의 구멍을 찾았다.

### 리뷰가 찾은 구멍: 생존 판정을 속일 수 있다

늦은 입력을 무조건 "받은 것"으로 세면, **스탬프가 얼어 있는 스트림이나 잘못된 프레임 스트림도 트래커를 '살아 있음' 상태로 유지**할 수 있다. 게다가 `SelectTarget`에는 현재 시각 기준 신선도 검사가 없어서, 오래된 OBSERVED 트랙이 선택될 수도 있다. 즉 "입력이 오고 있다"(liveness)와 "데이터가 현재 시각에 맞다"(freshness)는 **별개의 문제**다.

### 최종 규칙

입력을 "받은 것"으로 세려면 **둘 다 만족**해야 한다:

1. 프레임이 `robot_base`이고,
2. 스탬프가 지금까지 본 어떤 입력보다 **나중**이다.

이 규칙은 늦은 입력 경로와 정상 경로 **양쪽**에 적용된다. 프레임 처리 자체는 바뀌지 않았다. 사무실 PC WSL에서 tracking 141 passed. 구현은 [tracker_node.py](../../src/tracking/tracking/tracker_node.py), 계약 테스트는 [test_tracker_node_contract.py](../../src/tracking/test/contract/test_tracker_node_contract.py).

> **개념 정리 — liveness vs freshness:** liveness는 "입력이 계속 도착하는가"이고, freshness는 "도착한 데이터가 지금 시각 기준으로 신선한가"다. frozen-stamp 스트림은 liveness만 채우고 freshness는 없다. stamp가 앞으로 가야만 수신으로 세면, 입력이 멈춘 것과 같은 흐름에서는 빈 프레임이 들어가 track이 늙고 OBSERVED가 풀린다. 그래서 `SelectTarget`에 현재 시각 신선도 검사가 없다는 점(Codex 지적, 코드로 확인)은 오늘 바꾸지 않았다.

### 테스트 교훈: 새 테스트는 반드시 옛 코드에서 실패해야 한다

- **첫 버전은 frozen-stamp 케이스에서 수정 적용 상태인데도 실패했다.** 원인: 언 스탬프가 `t_state`와 **같았기 때문에** 정상 경로로 들어갔고, 정상 경로는 여전히 liveness를 갱신했다. 수정은 "늦은 경로"만 고쳤는데 frozen 스탬프는 늦은 경로가 아니었던 것. 그래서 양쪽 경로 모두에 규칙을 적용하게 됐다.
- **wrong-frame 케이스는 0.3 s, 1 s 지연으로는 옛 코드에서도 통과했다.** 시험이 끝나기 전에 stamp가 `t_state`를 추월하면 빈 프레임이 한 번 들어가 옛 코드도 LOST에 닿았기 때문이다. 시험 내내 stamp가 `t_state` 뒤에 있도록 지연을 **2 s**로 올리자 옛 코드(패치만)는 두 경우 모두 실패하고 새 코드는 3번 모두 통과했다(사무실 PC WSL).

> **개념 정리 — 회귀 테스트의 자격 요건:** "새 코드에서 통과하는 테스트"는 아무 가치가 없다. **옛 코드에서 실패하는지** 먼저 확인해야 그 테스트가 버그를 잡는 힘이 있다는 증거가 된다. 그리고 테스트 파라미터(여기서는 지연 시간)가 너무 작으면 우연히 옛 코드도 통과할 수 있으니, 실패를 확인하면서 파라미터를 조정한다.

---

## 4) CI 서비스 발견 대기

### 어떻게 발견됐나

CI에서 `test_robot_inactive_rejects_goals`가 두 번 실행에서 모두 시간 초과됐다. 새 클라이언트가 `/robot_controller_node/change_state` 서비스를 **발견(discovery)하는 데 5 s를 기다리다가 죽은 것**인데, 그동안 노드는 살아 있었다. 같은 모듈의 뒤 시험이 통과해 노드는 살아 있었으므로 "서버가 없다"보다 "새 client의 매칭이 느리다"로 본다. 다만 추정이다.

### 수정 ([drop_ros.py](../../tools/testing/drop_ros.py))

- 서비스 **발견 대기를 최대 15 s로** 늘렸다. **응답 대기는 그대로**다 — 느려진 것은 발견이지 응답이 아니다.
- 시간 초과 오류가 **그래프에서 매칭되는 서비스와 노드 목록을 출력**하게 했다. 다음 실패에서 "서버가 아예 없는 것"과 "매칭이 느린 것"을 구별할 수 있다.

> **개념 정리 — ROS 2 서비스 발견:** ROS 2에서 클라이언트가 서비스를 호출하려면 먼저 그래프에서 서비스를 "발견"해야 한다. 이 발견은 즉각적이지 않을 수 있고, 노드가 살아 있어도 발견이 늦어질 수 있다. 그래서 발견 대기와 응답 대기는 별개의 타임아웃으로 다뤄야 하고, 실패 원인을 구별하려면 실패 시점의 그래프 상태를 오류 메시지에 남겨야 한다.


## 5) 남은 결정 여섯 건을 에이전트끼리 정한 방법

사용자가 남은 결정을 에이전트들이 논의해 정하라고 했다. coordinator가 결정마다 처음 안을 쓰고, Codex·OpenCode에게 같은 안을 보내 찬반과 근거를 받은 뒤 advisor가 종합했다. 전체는 [결정 기록](../../records/reviews/decisions-2026-11-14.md)에 있다.

- **의견이 갈리면 무엇으로 정하나.** hover 도구는 OpenCode가 찬성, Codex가 보류였다. 실제 팔을 움직이고 여기서 돌려 볼 수 없는 변경이라 보수적인 쪽을 따랐다. 오늘 가드를 넣으면 "고친 도구의 첫 실행"이 현장이 된다는 점이 결정적이었다.
- **값은 표본이 아니라 규칙과 최악 경우로 정한다.** RELEASE 단계 제한을 처음에는 측정 0.91 s의 1.5배로 1.5 s를 냈는데, OpenCode가 저장소 규칙 REQ-ABT-007(`max(ceil(1.5 × p99), p95 + 0.5)`, 정수 초로 올림)을 찾아 2.0 s라고 했다. 다시 설계값으로 따져 보니 PREDICTED는 하강이 빠르면 RELEASE가 `entry_lead_s` 1.3 + 확인 유지 0.3 = 1.6 s까지 가서 1.5 s로는 부족했다. 표본 4개(fake transport)만 보면 이런 경우를 놓친다.
- **두 값의 관계는 시험으로 묶는다.** controller의 게이트 대기 상한을 2.0 → 1.0 s로 내려 SYNCHRONIZED RELEASE가 1.4 s가 되게 했고, "정상·게이트 대기 경우의 RELEASE 길이 ≤ orchestrator RELEASE 제한"을 설정 일관성 시험(TC-ROB-019-01)으로 고정했다. 한쪽만 바꾸면 시험이 실패한다. 개방을 확인하지 못한 경우(약 2.1 s)는 여전히 넘으므로 남은 사례로 적었다.
- **대기 시간은 관찰한 범위를 덮게 정하되, 대기만으로 준비됐다고 보지 않는다.** 기동 직후 발행 정지가 2.5~3.9 s 뒤에 관찰되어 운용 기동 대기를 2 → 5 s, I0 시험 안정 대기를 1.5 → 5.0 s로 늘렸다. 대기 뒤 heartbeat 확인과 정지 유지 해제 절차는 그대로다(Codex 지적).
- **검증된 시연 경로는 결정 범위에서 뺀다.** 종료 신호 처리는 `vision_node`·calibration 노드로 넓혔지만, 단계마다 Enter 확인을 받는 정적 투입 CLI는 오늘 실기에서 검증한 경로라 넣지 않았다. 기존 동작(`camera_tf_publisher`가 노드의 종료 코드를 돌려주는 것, `static_frames`가 설정 오류에 1로 끝나는 것)은 그대로 두었다.
- **바꾸지 않는 것도 결정이다.** `SelectTarget` 신선도는 바꾸지 않았고, Codex가 짚은 남은 사례(오래 밀린 검출이 stamp 순서대로 계속 오면 track이 늙지 않는다)를 기록했다.
- **되돌리기 쉽게 나눈다.** 결정마다 커밋을 하나씩 만들어 현장에서 문제가 생기면 `git revert <커밋>` 한 줄로 그 결정만 되돌릴 수 있다.

---

## 6) 흔한 함정

이번 정리에서 실제로 만난(거의 만날 뻔한) 함정들:

1. **범용 동치 주장.** "crop해도 결과가 같다"고 말할 때는 무엇을 확인했는지 함께 써야 한다. 이번에는 케이스 목록 + 실제 프레임 82장으로 범위를 명시했다.
2. **반올림의 평행이동 불변 가정.** `round`는 ties-to-even이라 이동 후 반올림하면 위치에 따라 크기가 달라진다. 반올림 먼저, 이동은 나중.
3. **crop 가장자리에 잘린 큰 물체.** ROI 밖의 큰 컴포넌트(팔, 손)가 crop 경계에서 잘려 크기 필터를 통과할 수 있다. "이미지 경계가 아닌 crop 경계에 닿으면 버린다"로 막는다.
4. **이미지 밖 ROI.** ROI가 이미지와 안 겹치면 빈 crop이 되는데, 빈 결과를 조용히 통과시키지 말고 검출 없음으로 정리한다.
5. **생존 판정을 스탬프 없이 세는 것.** frozen-stamp나 wrong-frame 스트림이 liveness를 속일 수 있다. "프레임 + 스탬프 단조 증가"를 같이 봐야 한다.
6. **liveness와 freshness를 하나로 묶는 것.** 입력 수신 판정을 고쳐도 오래된 트랙 선택(SelectTarget의 freshness 검사 부재)은 남는다. 별개 문제다.
7. **옛 코드에서 통과하는 회귀 테스트.** frozen-stamp 케이스는 정상 경로로 들어가 수정 적용 상태에서도 실패했고, wrong-frame 케이스는 0.3 s·1 s 지연에서 옛 코드도 통과했다. 새 테스트는 옛 코드 실패를 확인하고, 파라미터가 판별력을 갖는지 확인한다.
8. **기본값 변경으로 검증된 경로를 건드리는 것.** crop 기본 켜기는 `static_pick_place`의 검증된 동작(e509c87)을 바꿀 뻔했다. 기본은 끄고, 설정된 곳만 켠다.
9. **원격으로 검증할 수 없는 것을 병합하는 것.** hover 도구는 실제 팔을 움직인다. 검증 불가면 논쟁에서 이겨도 넣지 않는다.
10. **CI 실패 원인을 미리 구별해 두지 않는 것.** 발견 지연과 서버 부재는 다른 문제다. 타임아웃 오류에 그래프 상태를 출력해 두면 다음 실패에서 바로 구별된다.

---

## 7) 관련 파일

| 내용 | 파일 |
|---|---|
| 11-13 현장 점검 기록 | [records/sessions/OFF-4-2026-11-13.md](../../records/sessions/OFF-4-2026-11-13.md) |
| 검토 패치 | [records/patches/2026-11-13-vision-crop-tracker-late-hover.patch](../../records/patches/2026-11-13-vision-crop-tracker-late-hover.patch) |
| 세 의견 종합 리뷰 | [records/reviews/patch-review-2026-11-14.md](../../records/reviews/patch-review-2026-11-14.md) |
| crop 비교 증거 | [records/reviews/evidence/vision-crop-2026-11-14/](../../records/reviews/evidence/vision-crop-2026-11-14/) (`compare_crop.py`, `frames.txt`, `result.txt`) |
| vision 파이프라인 / ROI crop 구현 | [src/vision/vision/pipeline.py](../../src/vision/vision/pipeline.py) |
| tracker 구현 | [src/tracking/tracking/tracker_node.py](../../src/tracking/tracking/tracker_node.py) |
| vision 단위 테스트 | [src/vision/test/test_pipeline.py](../../src/vision/test/test_pipeline.py) |
| tracker 계약 테스트 | [src/tracking/test/contract/test_tracker_node_contract.py](../../src/tracking/test/contract/test_tracker_node_contract.py) |
| vision 사양 (REQ-VIS-011) | [specs/SPEC-200-vision.md](../../specs/SPEC-200-vision.md) |
| tracking 사양 (REQ-TRK-015) | [specs/SPEC-300-tracking.md](../../specs/SPEC-300-tracking.md) |
| CI flake 수정 도구 | [tools/testing/drop_ros.py](../../tools/testing/drop_ros.py) |
| 남은 결정 여섯 건 | [records/reviews/decisions-2026-11-14.md](../../records/reviews/decisions-2026-11-14.md) |
| 오늘 현장 체크리스트 | [docs/operations/site-checklist-2026-11-14.md](../operations/site-checklist-2026-11-14.md) |
| 11-13 학습 문서 (이전 편) | [2026-11-13-site-check-camera-and-live-tracking.md](2026-11-13-site-check-camera-and-live-tracking.md) |
| 작업 로그 | [records/worklog/2026-11-14.md](../../records/worklog/2026-11-14.md) |
