# SPEC-600 calibration

- 버전: v0.4
- 근거: PLAN.md 2.3, 2.4, 2.5절, Progress.md 4절
- REQ 영역: CAL

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.4 | 2026-11-12 | `frames.launch.py scenario:=<이름>`: mock 시나리오의 컨베이어 변환으로 `conveyor_frame` 발행 |
| v0.3 | 2026-11-12 | `T_base_conveyor` 강체 판정 허용오차 1e-5(소수 6자리로 적은 보정 행렬 수용) |
| v0.2 | 2026-11-11 | 고정 좌표계 static TF(`base_link → robot_base → conveyor_frame·mat_frame`), 평면 좌표계 계산, RViz 설정(REQ-CAL-011, 012) |
| v0.1 | 2026-11-07 | 내부 파라미터는 D455 공장값 사용, ChArUco 보드 + TCP 접촉 eye-to-hand 절차, 평면·진행 방향·TCP 오프셋·보정 파일·검증 판정, static TF 발행, CameraInfo 비교 |

## 1. 목적·범위

고정 카메라(D455 컬러)와 `robot_base` 사이의 외부 보정, 매트·벨트 평면, 벨트 진행 방향, 파지한 공깃돌 중심 ↔ TCP 오프셋을 구하고 검증 점 오차로 합격을 판정한다. 결과 파일은 `vision`(광선-평면 교차)과 `orchestrator`(PRECHECK의 calibration_id 확인)가 읽는다.

내부 파라미터는 새로 보정하지 않고 D455 공장 보정값을 쓴다. 근거: 공장값으로 실제 ChArUco 영상(2026-11-07, 코너 24개)의 재투영 rms가 0.12 px였다. G1에서 재투영 rms가 0.5 px를 넘거나 검증 점 판정에 실패하면 내부 보정을 다시 검토한다(9절).

**K는 하나로 고정한다.** 보정과 vision은 모두 보정 파일 `camera_profile`의 K를 쓰고, 실행 중 CameraInfo의 fx·fy는 쓰지 않는다. 근거(2026-11-07 실측): D455F 펌웨어가 보고하는 fx·fy는 스트림 시작마다 644.14 ~ 646.86 px(0.4%)로 바뀌었다. 반면 같은 보드를 찍은 영상의 실제 배율 변화는 0.037%였다. 평면 위 물체를 광선-평면 교차로 찾는 이 시스템에서는 초점거리 오차를 PnP가 카메라 거리로 흡수한다. 그래서 절대값보다 보정과 운전에서 같은 K를 쓰는 것이 중요하다.

## 2. 입력·출력(interface 버전)

### 2.1 입력

- 카메라 프로파일 `config/d455_color_1280x800.yaml`: serial, firmware, 1280×800, fx, fy, cx, cy, `plumb_bob` 계수 5개. RealSense `inverse_brown_conrady`는 OpenCV `plumb_bob`과 같은 투영 모델이다.
- 보드 `config/charuco_board.yaml`: 5×7칸, 칸 30 mm, 마커 22 mm, `DICT_4X4_50`, 보드 두께. 보드 좌표계는 `calibration/charuco.py` 머리말을 따른다(코너 id는 왼쪽 위 안쪽 코너부터 행 우선, 한 행 4개, 0~23).
- 세션 YAML(OFF-3 기록, 형식 `records/templates/calib-session.yaml`): 장면마다 영상, 역할(`fit`·`verify`), 면(`belt`·`mat`·`spacer`), 접촉 코너 id → robot_base 좌표 [m]. 벨트 진행 방향용 연속 영상 목록 `belt_track`.

### 2.2 출력(`calibrate_camera --out <폴더>`)

| 파일 | 내용 |
|---|---|
| `camera_extrinsics.yaml` | `calibration_id`, `date`, `transform`(parent `base_link` ← child `camera_color_optical_frame`, translation_m, rotation_xyzw), `camera_profile`, `residuals`(방법, 장면·점 수, rms_px, 접촉 잔차, 검증 통계) |
| `<면>_plane.yaml` | `calibration_id`, `date`, frame, 단위 법선, `offset_m`(n·p = offset, 보드 두께 보정 후), 잔차. `belt_plane.yaml`에는 `direction`(진행 방향 단위 벡터)을 더한다 |
| `calib-verify-<date>.csv` | 검증 점별 XY 오차 [m] |
| `report.yaml` | 장면별 접촉 잔차, 제외한 장면, 검증 통계 |

## 3. 파라미터(단위·기본값·근거)

`src/calibration/config/calibration.yaml`이 원본이다.

| 키 | 기본값 | 근거 |
|---|---|---|
| `plane_ransac_thresh_m` | 0.001 | 접촉 잡음 σ 0.2 mm의 5배 |
| `touch_rms_max_m` | 0.001 | 장면 하나의 접촉점(4개) Kabsch 잔차. 넘으면 잘못 접촉한 장면으로 보고 제외한다 |
| `verify_min_points_count` | 20 | TC-BEN-012-01 |
| `verify_xy_p95_max_m` | 0.001 | TC-BEN-012-01 |
| `verify_xy_max_m` | 0.002 | TC-BEN-012-01 |
| 보드 `board_thickness_m` | 0.0001 | 종이 두께. 받침판을 쓰면 캘리퍼로 재서 고친다 |

## 4. 동작(상태·알고리즘)

### 4.1 평면 맞춤(`fit_plane`)

RANSAC(3점, 300회, 시드 고정) → 인라이어 최소제곱(SVD) → 인라이어 재선정 최대 3회. 법선은 `up`(robot_base +z) 쪽을 향한다. 인라이어 비율이 0.5 미만이면 `NO_CONSENSUS`를 낸다. 평면 높이는 `offset`이 아니라 `height_at(x, y)`로 비교한다(법선이 조금 기울어도 offset은 작업 영역의 z와 다르다).

### 4.2 벨트 진행 방향

- `fit_direction`: 시간 순서의 점 → (평면이 있으면 평면에 투영) → 주성분 → 처음 → 끝 방향으로 부호를 정한다.
- `board_track_direction`: 벨트 위 보드를 벨트로 옮기며 찍은 영상들 → 보드 자세 → 보드 **중심**(보드 원점이 아님)을 robot_base로 옮겨 `fit_direction`을 적용한다.

### 4.3 eye-to-hand

- `solve_eye_to_hand(cam_pts, base_pts)`: 3D-3D Kabsch(반사 보정).
- `solve_eye_to_hand_pnp(base_pts, uv, K, dist)`: SQPnP 초기값 → LM 정밀화. 모든 점이 카메라 앞에 있어야 한다.
- 퇴화: 점이 부족하거나(3D-3D 3개, PnP 6개 미만), 중심화한 점의 σ2/σ1이 1e-3 미만(한 직선)이면 `DEGENERATE`.

### 4.4 ChArUco + TCP 접촉 절차(`solve_from_board_views`)

1. 보드를 놓고 로봇이 시야를 가리지 않는 상태에서 영상을 찍는다(`tools/bench/capture_d455.py`).
2. `charuco_view`로 코너가 검출되는지 확인하고, 접촉할 코너 id를 표시 영상에서 확인한다(기본 0, 3, 20, 23).
3. 끝이 뾰족한 포인터를 쥐고 TCP를 포인터 끝으로 등록한 뒤, 저속 조그로 코너를 접촉하고 TCP 위치를 m 단위로 적는다. **접촉 정밀도 목표는 축별 σ 0.2 mm 이하**다.
4. 장면 배치: 면(벨트·매트·벨트 위 50 mm 받침)마다 fit 장면 4개를 시야 전체에 고르게 놓는다. 벨트 위 verify 장면 5개(접촉 4점씩, 20점)는 fit에 쓰지 않는다.
5. 계산: 장면마다 접촉점으로 robot_base ← board를 구하고(Kabsch, 잔차가 `touch_rms_max_m`를 넘는 장면은 제외), 검출 코너 전체를 robot_base로 옮겨 PnP 하나로 푼다. verify 장면의 접촉 코너는 픽셀을 접촉 높이의 수평면에 교차시켜 XY 오차를 낸다.
6. 평면: 면별로 fit 장면 코너(robot_base)에 4.1절을 적용하고 보드 두께를 뺀다.

배치 근거(합성 시험, seed 20개): 접촉 σ 0.3 mm에 면마다 2장면이면 검증 게이트 실패가 3/20이다. σ 0.2 mm에 면마다 4장면이면 실패 0/20, p95 최악 0.71 mm, 벨트 높이 오차 최악 0.11 mm다. 대응점 높이가 벨트·매트(20 mm 차이)뿐이면 기울기 성분이 약해 받침 높이를 더한다.

### 4.5 TCP 오프셋(`fit_tcp_offset`)

TCP 자세 (R_i, t_i)와 관측한 공깃돌 중심 p_i로 p_i = R_i·o + t_i를 최소제곱으로 푼다. 관측하지 않은 성분은 NaN으로 넣는다. 위에서 수직 접근(yaw만 변함)해 카메라로 XY만 관측하면 o_z는 풀 수 없으므로 `fixed={2: 값}`으로 고정한다. 정규방정식 조건수가 1e6을 넘으면 `DEGENERATE`.

### 4.6 검증(`pixel_to_plane`, `verify_xy`)

픽셀은 `undistortPointsIter`(50회, 1e-12)로 정규화한다. 기본 `undistortPoints`(5회)는 시야 가장자리에서 정확도가 부족하다. 정규화한 광선을 평면과 교차시킨다. `verify_xy`는 XY 오차의 p50·p95·최대와 판정(n ≥ 20, p95 ≤ 1 mm, 최대 ≤ 2 mm)을 낸다.

### 4.7 ChArUco 코너 정밀화

`interpolateCornersCharuco`(OpenCV 4.6)는 정밀화 창에 이웃 마커의 검은 테두리를 포함해, 이상적인 영상에서도 코너를 약 0.3 px 끌어당긴다. 마커 변 길이로 칸 모서리와 마커 사이 여백(px)을 구하고, 그 0.8배 창으로 `cornerSubPix`를 다시 적용한다.

### 4.8 ROS 연결

- `camera_tf_publisher`: `extrinsics_file`을 읽고, 드라이버 TF `camera_link` ← `camera_color_optical_frame`을 `wait_s`(기본 10 s) 동안 기다린다. 받으면 `base_link` ← `camera_link` = T_base_opt · T_link_opt⁻¹을 static TF로 한 번 발행한다. 기다리는 시간을 넘기면 종료 코드 1로 끝낸다. 광학 TF는 드라이버만 발행한다(PLAN.md 2.5절).
- `check_camera_info`: CameraInfo 한 개를 받아 프로파일과 비교한다. 크기·왜곡 모델은 일치해야 하고 cx·cy 차이는 0.5 px 이하여야 한다. 왜곡은 계수 차이가 아니라, 영상 전체 9×9 격자의 같은 광선을 두 계수로 투영한 픽셀 차이가 0.5 px 이하여야 한다. 불일치면 종료 코드 2다. fx·fy 차이는 경고만 낸다(1절).
- 드라이버 프레임(2026-11-07 realsense2_camera 4.58.4 확인): `camera_link` → `camera_color_optical_frame`, 접두 없음. 컬러 광학 원점은 `camera_link`의 y 0.059 m에 있다.
- 영상 전송: 1280×800 RGB(약 3 MB)는 FastDDS 기본 SHM 세그먼트(0.5 MB)를 넘어 UDP 조각으로 간다. best effort 구독은 0.2 Hz밖에 받지 못했다(카메라 28.7 fps, reliable 26 Hz). `drop_bringup/config/fastdds_large_shm.xml`(SHM 64 MB)을 쓰면 best effort 29.9 Hz, stamp 간격 최대 33.38 ms다. 영상을 받는 모든 노드가 같은 프로파일을 써야 한다(`drop_bringup/launch/camera.launch.py`).

### 4.9 고정 좌표계 TF(`static_frames_publisher`)

PLAN.md 2.5절·SPEC-000 4.2절의 고정 TF 중 카메라를 뺀 것을 발행한다. 카메라(`base_link → camera_link`)는 4.8절 `camera_tf_publisher`가, `world → base_link`와 로봇 링크는 Doosan `robot_state_publisher`가 발행한다(단일 발행 원칙).

```
world ─(robot_state_publisher, 또는 publish_world:=true일 때 이 노드)─ base_link
base_link ─ robot_base            항등. 프로젝트 메시지의 frame_id "robot_base"를 Doosan "base_link"에 잇는다
robot_base ─ conveyor_frame       tracking.yaml `T_base_conveyor`(planner와 같은 값)
robot_base ─ mat_frame            mat_plane.yaml에서 계산(아래)
```

- `plane_frame(normal, offset_m, x_hint, origin_xy)`: z축 = 평면 법선(위쪽, z 성분 > 0으로 맞춤), x축 = `x_hint`를 평면에 투영해 정규화, y = z × x, 원점 = robot_base `origin_xy`를 지나는 수직선과 평면의 교점. 평면은 `normal · p = offset_m`이다. `x_hint`가 법선과 평행(투영 길이 < 1e-6)이면 `ValueError`다.
- 노드는 시작할 때 한 번 `/tf_static`으로 발행한다. `T_base_conveyor`가 강체 변환(`RᵀR − I`의 각 성분과 `det − 1`이 1e-5 이내, 마지막 행 0 0 0 1)이 아니면 발행하지 않고 종료 코드 1로 끝낸다. 허용오차는 소수 6자리로 적은 보정 행렬(직교성 오차 약 1e-6)을 받기 위한 값이다. 같은 행렬을 읽는 `tracker_node`·`intercept_planner_node`도 configure에서 같은 허용오차로 판정한다.
- `drop_bringup/launch/frames.launch.py`가 `tracking.yaml`의 `T_base_conveyor`와 `records/calib/2026-11-09-planes/mat_plane.yaml`을 넘겨 노드를 띄운다. `rviz:=true`면 `drop_bringup/config/drop.rviz`(고정 프레임 `robot_base`, TF, `drop/predicted_marker`)로 RViz2를 띄운다. `publish_world:=true`는 로봇 드라이버 없이(mock·replay) 볼 때만 쓴다. `scenario:=<drop_mocks 시나리오>`를 주면 `tracking.yaml` 대신 그 시나리오 `world.frames`의 변환을 쓴다(`drop_mocks.scenario_engine.scenario_geometry`, `drop.launch.py`가 mock 세계일 때 실제 tracker·planner에 넘기는 값과 같다).

- REQ-CAL-011: `plane_frame()`의 원점은 평면 위(|normal·p − offset| ≤ 1e-12 m)이고 xy가 `origin_xy`와 같은 수직선 위다. 회전은 직교·det = 1이고 z축 = 정규화 법선, x축은 `x_hint`의 평면 투영 방향과 1e-9 rad 이내로 같다. 법선과 평행한 `x_hint`는 `ValueError`다.
- REQ-CAL-012: `static_frames_publisher`가 낸 `/tf_static`의 부모→자식 쌍은 `base_link→robot_base`, `robot_base→conveyor_frame`, `robot_base→mat_frame`(+ `publish_world`일 때 `world→base_link`)이고, 중복 쌍은 0이며, `robot_base→conveyor_frame` 변환은 정확한 강체 `T_base_conveyor`와 1e-9 이내, 소수 6자리로 적은 행렬과는 1e-5 이내로 같다. 강체가 아닌 `T_base_conveyor`면 아무것도 발행하지 않는다. 기울어진 회전을 소수 6자리로 적은 행렬은 받고 3자리로 적은 행렬과 배율 1.001은 거부한다. `tracker_node`·`intercept_planner_node`의 configure도 6자리 행렬에서 성공하고 3자리 행렬에서 실패한다. tracking.yaml과 planner.yaml의 `T_base_conveyor`는 같다. `frames.launch.py`는 `scenario`가 없으면 tracking.yaml의 변환을, 있으면 그 시나리오의 컨베이어 변환(mock 세계의 `conveyor_frame`과 1e-12 이내)을 노드에 넘긴다.

## 5. 시간 제약(주기·제한시간)

오프라인 도구라 실시간 제약이 없다. 보정 파일은 노드 기동 시 한 번 읽는다.

## 6. 실패 모드와 응답(중단 등급)

| 상황 | 응답 |
|---|---|
| 퇴화 배치·점 부족 | `CalibrationError('DEGENERATE')`, 결과 파일을 쓰지 않는다 |
| 평면 합의 부족 | `CalibrationError('NO_CONSENSUS')` |
| 장면 접촉 잔차 초과 | 그 장면을 제외하고 `bad_touch_views`에 남긴다 |
| 검증 불합격 | 파일은 쓰되 `residuals.verify.pass: false`, 종료 코드 2. PRECHECK에서 쓰면 안 된다 |
| 보정 파일 필수 항목 누락·값 오류 | `CalibrationFileError`, 노드는 기동하지 않는다 |
| vision과 orchestrator의 calibration_id 불일치 | PRECHECK FAILED(`CALIBRATION_MISMATCH`), 로봇 미동작 |
| 독립 검증 점 부족 / 중복 / fit 점 재사용 | `CalibrationError('INSUFFICIENT_POINTS' / 'DUPLICATE_POINTS' / 'FIT_POINT_REUSE')`, 검증 거부 |
| 입력 비유한값 / 불량 변환 / 단위 오류 | `CalibrationError('NONFINITE_INPUT' / 'MALFORMED_TRANSFORM' / 'INVALID_UNITS')`, 검증 거부 |
| 포인터 TCP 미정의 / 출처·프로파일 불일치 | `CalibrationError('UNKNOWN_POINTER_TCP' / 'MISSING_PROVENANCE' / 'CALIBRATION_MISMATCH' / 'PROFILE_MISMATCH')`, 검증 거부 |
| 합성 데이터를 물리 검증에 제출 | `CalibrationError('SYNTHETIC_DATA_REJECTED')`, 물리 게이트 합격 불가 |

## 7. 성공 판정(측정 가능)

- REQ-CAL-001: `fit_plane`은 잡음 0.2 mm 점 200개 + 이상점 10%에서 법선 오차 ≤ 0.1°, offset 오차 ≤ 0.1 mm, 법선 z > 0이다. 한 직선 위 점은 `DEGENERATE`로 거부한다.
- REQ-CAL-002: 진행 방향은 직선 이동 점(잡음 0.3 mm)에서 부호 포함 방향 오차 ≤ 0.1°, 보드 이동 영상(16 cm, 6장면, 픽셀 잡음 0.2 px)에서 ≤ 0.2°다.
- REQ-CAL-003: eye-to-hand는 3D-3D(잡음 축별 0.3 mm, 20점)에서 작업면 점 오차 ≤ 0.5 mm, 회전 오차 ≤ 0.1°, 카메라 원점 이동 오차 ≤ 1 mm(참고)이다. PnP(24점, 세 높이, 0.3 px)에서 벨트면 XY 최대 오차 ≤ 0.5 mm다. 4.4절 배치(접촉 σ 0.2 mm)에서 검증 게이트를 통과한다. 퇴화 배치는 `DEGENERATE`로 거부한다.
- REQ-CAL-004: TCP 오프셋 오차 ≤ 0.2 mm(전체 3축, 또는 z 고정 시 XY). z를 고정하지 않은 수직 접근 자세는 `DEGENERATE`로 거부한다.
- REQ-CAL-005: 보정 파일에 `calibration_id`·`date`·`residuals`·`camera_profile`·`transform` 중 하나라도 없거나 값이 비었거나, 프로파일 내부 항목이 없거나, quaternion이 단위가 아니면 로드에 실패한다. vision과 orchestrator의 calibration_id가 다르면 PRECHECK FAILED(`CALIBRATION_MISMATCH`)다.
- REQ-CAL-006: `pixel_to_plane`은 오차 없는 투영을 1 µm 이내로 되돌린다. `verify_xy`는 p95(선형 보간 백분위)·최대를 내고 n < 20이면 불합격이다.
- REQ-CAL-008: `base_link` ← `camera_link`에 드라이버 TF를 합성하면 보정 결과 `base_link` ← 광학 프레임과 1e-12 이내로 같다.
- REQ-CAL-009: `compare_camera_info`는 프로파일과 같은 CameraInfo(fx 차이 0.4 px 포함)에서 빈 목록을 낸다. 크기·왜곡 모델 불일치, cx 0.6 px 차이, 왜곡 투영 차이 > 0.5 px는 해당 항목을 담은 불일치로 낸다. `check_focal=False`면 fx·fy 차이(2.2 px)를 불일치로 내지 않는다.
- REQ-CAL-007: ChArUco 코너는 원근 렌더 영상에서 ≤ 0.2 px, 보드 자세는 이동 ≤ 1 mm·회전 ≤ 0.2°다. 보드 좌표는 인쇄 원본(PDF)의 배치와 같다.
- REQ-CAL-010: 후보 카메라 외부 보정의 독립 접촉점 검증 도구(CLI/라이브러리)는 common frame(`base_link`) 상에서 카메라 추정 위치와 포인터 보정된 로봇 실제 접촉점 간의 점별 XY 잔차를 계산하고, n>=20점 검증 시 제안 기준(p95 <= 1 mm, max <= 2 mm, 구성 가능)으로 합격 여부를 판정한다. 20점 미만, 중복/재사용 fit 점 또는 ID, 비유한값/불량 변환, 잘못된 단위/미확인 포인터 TCP, 출처(provenance) 누락 또는 카메라 프로파일/배치 불일치, 그리고 물리 게이트 판정에 합성 데이터를 제출하는 경우 즉시 거부하며, 생산 파일 자동 덮어쓰기나 영상 재투영 rms 기반 G1_pass를 엄격히 금지한다.

## 8. 검증 방법(`TEST_SPEC.md` TC ID, 게이트)

- U: TC-CAL-001-01, TC-CAL-002-01, TC-CAL-002-02, TC-CAL-003-01~04, TC-CAL-004-01, TC-CAL-005-01, TC-CAL-006-01, TC-CAL-007-01, TC-CAL-008-01, TC-CAL-009-01, TC-CAL-010-01~03, TC-CAL-011-01 (`src/calibration/test/`)
- C: TC-CAL-005-02(vision_node 구현 후), TC-CAL-012-01. U: TC-CAL-012-02(`drop_mocks/test/test_scenario_engine.py`). I2: TC-CAL-012-03(`drop_bringup/test/test_frames_launch.py`). I0: TC-INT-012-01(관측 TF와 예측 Marker의 좌표 일치, `test_i0_integration.py`)
- B: TC-BEN-012-01(OFF-3 검증 점)
- 게이트: G1

## 9. 미결 사항

- vision의 `localizer`가 보정 파일의 K를 쓰도록 SPEC-200에 반영한다(vision 구현 시).
- 시스템 전역 소켓 버퍼(`net.core.rmem_max` 4 MB)는 바꾸지 않았다. 다른 PC로 영상을 보내야 하면 다시 검토한다.
- 로봇 기준 프레임 이름(`base_link`)은 doosan-robot2 URDF로 확인한다.
- TCP 위치 기록 경로: 지금은 티치펜던트 값을 m로 옮겨 적는다. `motion_adapter`가 생기면 ROS로 TCP 자세를 읽어 세션 YAML을 자동으로 채운다.
- 접촉 정밀도 σ 0.2 mm는 가정이다. OFF-3에서 같은 코너를 5회 접촉해 실측하고, 미달이면 장면 수를 늘린다.
- 로봇 절대 위치 정확도(관절 보정 오차)는 모델에 넣지 않았다. 검증 점 오차에 그대로 나타난다.
- 내부 파라미터 재보정 여부: G1에서 재투영 rms > 0.5 px 또는 검증 불합격이면 ChArUco 다중 자세로 내부 보정을 다시 하고 공장값과 비교한다.
