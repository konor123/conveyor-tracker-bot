# handeye_calib.py — 고정 카메라 Hand-Eye 보정

`handeye_calib.py` 한 파일로 동작한다. ROS와 이 저장소의 다른 코드가 필요 없다.
로봇 TCP에 붙인 ArUco 표식을 여러 자세에서 찍은 영상과 그 자세의 로봇 좌표로 아래 두 변환을 구한다.

| 기호 | 방향 | 뜻 |
| --- | --- | --- |
| `Y` | `base <- camera` | 로봇 베이스에서 본 카메라(광학 프레임) |
| `X` | `tcp <- marker` | TCP에서 본 표식 |

카메라가 **고정**되어 있고 표식이 손끝에 붙은 배치(eye-to-hand)만 지원한다. 카메라가 손목에 붙은 배치는 식이 달라 쓸 수 없다.

## 준비

```bash
pip install numpy scipy pyyaml opencv-contrib-python   # cv2.aruco 필요. 4.6 ~ 4.x에서 확인
```

## 촬영 기준 (2026-11-07 세션에서 쓴 것)

- 자세 30개 이상 + 검증용 20개. 자세마다 로봇이 **멈춘 뒤** 로봇 좌표를 읽고 영상을 찍는다. 자세는 명령 목표가 아니라 제어기가 보고하는 **실제 값**을 쓴다.
- 손목을 크게 다르게 돌린다(자세 사이 최대 회전 30° 미만이면 경고). 표식이 항상 카메라를 보게 하되 기울기를 다양하게 한다.
- 한 자세에서 5장 정도 연속 촬영하면 코너를 평균해 지터를 줄인다.
- 표식 크기는 검은 테두리 **바깥** 한 변의 실측값이다. 틀리면 `X`·`Y` 병진이 그만큼 어긋난다.
- 영상 해상도는 내부 파라미터를 얻은 해상도와 같아야 한다.

## 입력

`poses.csv` — 한 줄이 한 자세다.

```csv
pose_id,images,x,y,z,a,b,c
pose-00,img/pose-00-*.png,635.96,-0.01,284.44,179.997,-165.076,89.998
pose-01,img/pose-01-*.png,612.30,40.52,301.10,175.120,-160.331,88.402
```

- `images`: 영상 경로. `;`로 여러 개, glob(`*`)도 된다. 상대 경로는 CSV 폴더(또는 `--image-dir`) 기준.
- `x,y,z`: TCP 위치. 기본 mm (`--length-unit m`으로 변경).
- 회전: `--rot-format`에 따라 열이 다르다.

  | `--rot-format` | 열 | 비고 |
  | --- | --- | --- |
  | `euler-zyz` (기본) | `a,b,c` (deg) | Doosan `posx`의 ZYZ. `R = Rz(a)·Ry(b)·Rz(c)` |
  | `euler-zyx`, `euler-xyz` | `a,b,c` (deg) | 내재(intrinsic) 오일러 |
  | `rotvec` | `a,b,c` (rad) | 회전벡터 |
  | `quat` | `qx,qy,qz,qw` | |
  | `matrix` | `r00,r01,...,r22` | 제어기가 회전행렬을 주면 이것을 권장 |

- `role`(선택): `fit` 또는 `verify`. 없으면 `--holdout-last N`으로 마지막 N자세를 검증용으로 쓴다. 검증용 자세는 추정에 쓰지 않는다.

`camera.yaml` — `fx, fy, cx, cy, distortion`(k1 k2 p1 p2 k3, plumb_bob)과 선택 `width, height`. ROS CameraInfo의 `k`(9개)·`d`도 받는다. 예: `records/calib/2026-11-07-final-layout/inputs/d455_color_1280x800.yaml`.

## 실행

```bash
python3 Hand-Eye-Calibration/handeye_calib.py \
    --poses poses.csv --camera camera.yaml --out result/ \
    --marker-id 25 --marker-size 0.040 --dict DICT_4X4_50 \
    --rot-format euler-zyz --holdout-last 20 --plot
```

프레임 이름은 `--base-frame`, `--tcp-frame`, `--camera-frame`으로 바꾼다(기본 `base_link`, `controller_tcp`, `camera_color_optical_frame`).

## 출력

- `handeye_result.yaml`: `Y`, `X`와 각각의 역행렬(4×4, 병진 m, quaternion xyzw), FIT/HOLDOUT 재투영 오차, 1σ 불확도, 조건수, 초기화 5종 비교, 경고, 입력 파일 SHA-256.
- `per_pose_errors.csv`: 자세별 검출 장수, 코너 지터, 재투영 오차.
- `residuals.png`: `--plot` 지정 시.

종료 코드 0 = 완료, 2 = 입력 오류. 경고는 stderr와 YAML `warnings`에 나온다.

## 결과 읽는 법

- **HOLDOUT 오차가 FIT보다 2배 넘게 크면** 과적합이거나 자세·영상 대응이 어긋난 것이다.
- 1σ와 조건수는 픽셀 잡음만 반영한다. 표식 크기 오차나 내부 파라미터 오차 같은 **체계 오차는 들어 있지 않다.**
- 재투영 오차가 작아도 `X`와 `Y`가 서로 보상해 실제 위치는 틀릴 수 있다. 결과는 항상 **후보값**이다. 운용에 쓰기 전에 실제 TCP를 알려진 점에 접촉시키는 식의 독립 검증이 필요하다(우리 팀 기준은 `TC-BEN-012-01`).

## 기준 재현

2026-11-07 세션 데이터로 같은 값이 나온다(`Y` 병진 [0.84658, 0.05628, 0.59248] m, FIT 0.977 px, HOLDOUT 1.147 px).
`records/calib/2026-11-07-final-layout/image_pose_index.csv`에 `x_mm,y_mm,z_mm`와 `A/B/C_ZYZ_deg`가 있으므로 자세별 한 줄로 줄여
`--rot-format euler-zyz --holdout-last 20 --marker-id 25 --marker-size 0.040`으로 실행하면 된다.

## 시험

```bash
python3 -m pytest -q Hand-Eye-Calibration
```

합성 ArUco 영상으로 알려진 `X`·`Y`를 복원하는지, 입력 오류를 거부하는지, Doosan ZYZ 규약이 실제 제어기 회전행렬과 맞는지 확인한다.
