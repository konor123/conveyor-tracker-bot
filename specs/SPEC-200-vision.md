# SPEC-200 vision

- 버전: v0.1
- 근거: PLAN.md 0.2-4, 4.5절
- REQ 영역: VIS

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-11-09 | 픽셀 검출(공깃돌 배경 색 거리·병뚜껑 색 범위+원 적합), `stone_inside`, 로봇 마스크 표시, 광선-평면 교차·공분산, 프레임 처리, DetectObjects 안정 창 |

## 1. 목적·범위

공깃돌·병뚜껑 검출, 광선-평면 교차, 로봇 마스크, 투입 판정. 판단 로직은 ROS와 무관한 순수 모듈(`detector.py`, `localizer.py`, `pipeline.py`, `stable_window.py`)이고 `vision_node`는 감싸기만 한다.

## 2. 입력·출력(interface 버전)

SPEC-010 wire schema v0.2 기준.

- 입력: 컬러 영상(D455 1280×800, bgr8/rgb8, header.stamp = 노출 시각), `camera_extrinsics.yaml`(robot_base ← camera_color_optical_frame, parent 이름은 `robot_base` 또는 Doosan 드라이버 이름 `base_link`), `mat_plane.yaml`·`belt_plane.yaml`(SPEC-600 2절), 로봇 마스크(0/1 영상, 선택).
- 출력: `objects/detections`(`ObjectDetectionArray`), `vision/detect_objects`(`DetectObjects`).
- 좌표: 픽셀은 왜곡 보정 전 (u 오른쪽, v 아래). 위치는 robot_base [m].

## 3. 파라미터(단위·기본값·근거)

픽셀 크기는 2026-11-09 OFF-1 정지 영상(카메라 높이 약 0.6 m, 약 0.95 mm/px)에서 정한 제안값이다. 카메라 높이를 바꾸면 비례해 고친다.

| 키 | 기본값 | 근거 |
|---|---|---|
| 공깃돌 `diameter_min_px`·`diameter_max_px` | 11 / 26 | 실영상 공깃돌 등가 지름 20~25 px |
| 공깃돌 `circularity_min` | 0.70 | 팔각형 원형도 0.95, 정사각형 0.79(반 픽셀 보정 후) |
| 공깃돌 `delta_e_min` / `delta_e_soft` | 30 / 15 | 배경(매트·벨트) 중앙값과의 Lab 거리. 경계 가중치는 15에서 0, 30에서 1 |
| 병뚜껑 `hsv_lo` / `hsv_hi` | (90,120,70) / (118,255,255) | 파란 뚜껑 실영상 H 97~100, S 200~230 |
| 병뚜껑 `diameter_min_px`·`diameter_max_px` | 28 / 52 | 실영상 바깥 지름 38~42 px |
| 병뚜껑 `ring_coverage_min` | 0.50 | 뚜껑 안 공깃돌로 끊긴 고리의 각도 덮임. 2026-11-09 로봇 투입 뒤 장면(하이라이트로 고리 일부 소실) 0.56 |
| `inner_diameter_ratio` | 0.80 | TBD(`D_in` 실측 OFF-1) |
| `center_std_px` | 0.5 | TBD(정지 영상 반복성) |
| `h_stone_offset_m` | 0.008 | TBD(공깃돌 높이 S03 실측) |
| `h_rim_m` / `floor_thickness_m` | TBD | 병뚜껑 실측 C03·C08 |
| 안정 창 `std_max_m` | 0.0005 | orchestrator `detect_stone.window_std_max_m` |
| 안정 창 `std_max_px` | 1.0 | 보정이 없을 때 대체 기준(제안값) |
| 안정 창 `presence_min_ratio` | 0.8 | 창 안 프레임 중 보인 비율 |
| 안정 창 `gate_m` / `gate_px` | 0.005 / 8 | 프레임 간 같은 대상물 |

## 4. 동작(상태·알고리즘)

### 4.1 공깃돌

공깃돌은 면마다 색이 다르다(노랑·분홍·보라·빨강·파랑). ROI 배경색(ROI 안 Lab 중앙값)과의 거리가 `delta_e_min` 이상인 픽셀을 전경으로 보고, 열림 연산 뒤 연결 성분마다 크기·원형도를 거른다. 면적이 공깃돌 최대 면적의 1.5배를 넘는 덩어리(손·ArUco 마커 테두리)는 볼록 껍질과 주변 `exclude_dilate_px`까지 제외한다. 중심은 성분 내부 가중치 1, 바깥 1 px 테두리는 배경 거리 비율 가중치로 구한 가중 중심이다(서브픽셀).

### 4.2 병뚜껑

색 범위 마스크를 닫고 구멍을 채운 성분이 크기·원형도를 넘으면 4.1과 같은 가중 중심을 쓴다. 넘지 못하면(뚜껑 안 공깃돌·하이라이트로 고리가 끊김) 바깥 경계 점(최소 외접원 반지름 ±2 px)에 대수적 원 적합을 하고, 바깥 경계가 덮는 각도 비율이 `ring_coverage_min` 이상이면 채택한다. 색 마스크는 ROI로 자르지 않고 중심이 ROI 안인 것만 쓴다(벨트 끝에 걸친 뚜껑). 채택한 뚜껑의 색 픽셀만 공깃돌 전경에서 뺀다(같은 색의 작은 공깃돌은 남긴다).

### 4.3 `stone_inside`·로봇 마스크

- 뚜껑 안쪽 반지름 `r_in = 0.5 · diameter_px · inner_diameter_ratio`. 공깃돌 중심과 뚜껑 중심의 거리 < `r_in`이면 안이다(안쪽 테두리 접촉 = `r_in − r_stone`, 바깥 테두리 접촉 = `r_out + r_stone`).
- 검출 성분(뚜껑은 원)이 `occlusion_dilate_px` 여유 안에서 로봇 마스크와 겹치면 `occluded_partial = true`. 가려진 뚜껑의 `stone_inside`는 판정 불가(`stone_inside_valid = false`).

### 4.4 평면 교차

`plumb_bob` 왜곡을 보정한 광선을 대상물별 평면과 교차한다. 매트 위 공깃돌 = 매트 + `h_stone_offset`, 병뚜껑 = 벨트 + `h_rim`, 뚜껑 안 공깃돌 = 벨트 + `floor_thickness` + `h_stone_offset`, 뚜껑 밖 벨트 위 공깃돌 = 벨트 + `h_stone_offset`. 높이는 robot_base +z 방향 이동이다. 위치 공분산은 픽셀 공분산의 1차 전파이며 K·외부 자세·평면 오차는 포함하지 않는다. 보정 파일이 없으면 `position_valid = false`로 픽셀 검출만 낸다.

### 4.5 DetectObjects 안정 창

프레임별 검출을 노출 시각과 함께 모은다. 최신 프레임의 (region, class, 가리지 않음) 검출을 기준으로, 창(`stable_window`) 안 각 프레임에서 게이트 안 가장 가까운 검출을 묶는다. 보인 비율이 `presence_min_ratio` 이상이고 축별 표준편차 최대값이 `std_max_m`(위치가 없으면 `std_max_px`) 이하인 것만 안정으로 본다. 창 길이가 차지 않았으면 판정하지 않고 다음 프레임을 기다린다. stamp가 역행·반복하면 창을 비운다. 안정 개수 < `min_count`이면 FAILED(`NOT_ENOUGH_STABLE`).

- REQ-VIS-001: 공깃돌(매트 배경, 5색)과 병뚜껑(벨트 배경)의 중심 오차 p95 ≤ 0.2 px, 공깃돌 등가 지름 오차 ≤ 1 px(TEST_SPEC 3.5절 합성 조건).
- REQ-VIS-002: 공깃돌·병뚜껑·흰 사각 표식이 섞인 장면에서 공깃돌은 STONE, 병뚜껑은 CAP으로 분류하고 표식은 검출하지 않는다(파란 공깃돌 포함).
- REQ-VIS-003: `stone_inside`는 뚜껑 안 중앙·안쪽 테두리 접촉에서 true, 바깥 테두리 접촉·빈 뚜껑에서 false, 뚜껑이 로봇 마스크로 가려지면 None이다.
- REQ-VIS-004: 왜곡 보정 광선과 robot_base 평면의 교차는 왜곡 계수 0·비0 합성 왕복 오차 1e-6 m 이하다.
- REQ-VIS-005: 4.4절 대상물별 평면을 쓴다.
- REQ-VIS-006: 픽셀 공분산 diag(0.25, 0.25) px²를 3×3 위치 공분산으로 전파하며 각 원소가 수치 야코비안 결과의 5% 이내다.
- REQ-VIS-007: 로봇 마스크와 겹친 검출은 `occluded_partial = true`로 표시하고 `DetectObjects` 개수에서 뺀다.
- REQ-VIS-008: `DetectObjects`는 4.5절 안정 창을 쓰며 지터 0.3 mm는 OK, 0.6 mm는 FAILED, 지난 deadline은 즉시 TIMEOUT이다.

## 5. 시간 제약(주기·제한시간)

영상 주기 30 Hz. 처리가 밀리면 최신 프레임만 처리한다. `DetectObjects`는 요청의 `deadline`을 넘기면 TIMEOUT이다.

## 6. 실패 모드와 응답(중단 등급)

| 상황 | 응답 |
|---|---|
| ROI(매트·벨트)를 찾지 못함 | 해당 ROI 검출 0개 |
| 보정 파일 없음 | `position_valid = false` |
| 광선이 평면과 평행·카메라 뒤 | 그 검출만 `position_valid = false` |
| 뚜껑 가림 | `stone_inside_valid = false` |

## 7. 성공 판정(측정 가능)

Progress.md 5절 G1 합격 기준.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

TC-VIS-001-01 ~ TC-VIS-007-01(U, `src/vision/test/test_detector.py`·`test_localizer.py`), TC-VIS-008-01·02(C), TC-VIS-010-01(I1, `off1_bags`). 게이트 G1.

## 9. 미결 사항

- 파란 공깃돌이 파란 뚜껑 안에 있으면 뚜껑 색 픽셀로 빠져 보이지 않는다(실영상 미확인).
- 매트 위 공깃돌 등가 지름이 실측(17~18 mm)보다 2~3 px 크게 나온다(그림자·경계 가중). 중심에는 영향이 작으나 지름으로 판정하지 않는다.
- `inner_diameter_ratio`, `h_stone_offset_m`, `h_rim_m`, `floor_thickness_m`: TBD(OFF-1 실측).
- ROI 자동 탐색은 확인용이다. 운용 ROI는 보정 뒤 고정값으로 둔다.
