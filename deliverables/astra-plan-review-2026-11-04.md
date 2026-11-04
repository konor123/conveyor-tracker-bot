# 컨베이어 동적 단추 적층 계획 기술 자문

- 검토일: 2026-11-04 UTC. 공식 자료·소스 확인: 약 10:33~10:40 UTC.
- 검토자: Orca에 배정된 Codex worker, 요청된 Astra 기술 자문 역할. 다른 worker는 호출하지 않았다.
- 대상: `/home/ubuntu/orca/projects/conveyer-tracking-bot/package_plan.md`, `progress_plan.md`.
- 범위: 계획 및 공개 공식 자료의 정적 검토. PC·카메라·로봇에 접속하거나 코드를 빌드·실행한 실기 검증은 하지 않았다. 저장소는 수정하지 않았다.
- 근거 구분: **확인함**은 문서·공식 소스에서 확인한 사실이며 실기 검증을 의미하지 않는다. **일반 지식**은 기하·제어·ROS 설계 원칙, **추정**은 아직 측정하지 않은 수치·제안 목표이다. 장비별 지원 여부는 별도로 **확인 필요**라고 표시한다.

## 검토 결론

정적 적층 → 영상 추적 → 예측 → 동기 추종의 큰 방향은 타당하다. 특히 매트와 벨트의 별도 보정 파일, 벨트가 수동 구동이므로 속도 명령을 만들지 않는 결정, 정적 적층을 먼저 검증하는 순서는 유지할 만하다. 다만 성공을 좌우하는 것은 Moving TF 자체가 아니라 **10 mm 물체의 실제 파지·해제 기하, 영상 가림 동안의 예측 오차, 제어 지연과 해제 지연의 합, 독립적인 적층 판정**이다.

필수 수정은 **11건(M01~M11)**이다. 이는 구현 결함이 이미 발견됐다는 뜻이 아니라, 현 계획을 구현 계약으로 사용할 때 실패·오판·실기 진행 불가를 막기 위해 명시해야 할 항목이다. 권장 5건, 선택 3건을 별도로 제안한다. 첫 목표는 저속·일정 속도에서의 정적 적층 및 예측 타이밍 적층으로 하고, 속도 동기 추종은 제어 인터페이스와 가림 허용 시간이 검증된 뒤 진행한다.

## 1. 분류별 지적 및 구체적 수정안

### 1.1 필수 수정: 11건

| ID | 위치 | 문제 및 근거 | 구체적 수정안 |
|---|---|---|---|
| M01 | `progress_plan.md` §7~9, 특히 73~76행; `package_plan.md` §4~5 | Moving TF를 미래 목표 생성·실시간 추종 수단으로 해석할 여지가 있다. tf2는 저장된 변환을 조회·보간하며 관측이 없는 미래를 운동 모델로 예측하지 않는다. TF에는 속도·불확실성·유효기간도 없다. **확인함/일반 지식**, [tf2 소스](https://github.com/ros2/geometry2/blob/jazzy/tf2/src/cache.cpp). | TF는 좌표 관계와 과거/현재 상태 표시, `TrackedButton`은 시각·속도·공분산, `InterceptPlan`은 미래 접촉 시각·예측·만료를 담당하도록 규정한다. 미래 시각의 `lookupTransform`을 제어 핵심에 넣지 않는다. §2.c 참조. |
| M02 | `progress_plan.md` §1의 11행과 §9의 93행; `package_plan.md` §4 마지막 부분 | 연속 제어를 1단계에서 단순 확인하고 9단계에서 확정하면 핵심 경로의 지원 여부를 너무 늦게 알게 된다. Jazzy에 스트림이 있어도 설치 버전·제어기·E0509 실기의 지원과 지연은 별개다. `move_line` 반복 호출은 연속 서보와 같지 않다. **확인함/확인 필요**, §2.b. | 1단계를 계약 검증 게이트로 강화: 버전 고정, 스트림 한 종류 선정, 단위·주기·피드백·중단·재시작 조건 확인. 실제 하강 없이 안전한 높이에서 지연·지터·통신 단절 시험을 먼저 한다. 목표 주기를 못 맞추면 예측 타이밍 적층으로 범위를 제한한다. |
| M03 | `package_plan.md` §5의 “관측 가능한 범위”; `progress_plan.md` §6·10·11 | 추적 손실을 언급하지만 적층 때 정상적으로 발생하는 팔·그리퍼 가림을 언제까지 허용할지 없다. 두 번째 단추를 첫 번째 단추로 오인할 수도 있다. 관측이 끊긴 뒤 마지막 좌표를 계속 쓰면 이동량만큼 빗나간다. **일반 지식**. | `OBSERVED/COASTING/LOST`와 `last_observed_stamp`, 최대 예측 시간, 공분산 제한을 정의한다. 해제 전 중단 조건과 해제 후 후퇴·검증 조건을 분리한다. robot mask, 보조 시점, §2.d의 시간 예산을 추가한다. |
| M04 | `progress_plan.md` §3~5; `package_plan.md` §3·4의 3D 위치 변환 | RGB→로봇 좌표 변환의 평면·높이·TCP 기준이 불명확하다. 얇은 단추의 단일 픽셀 Depth는 구멍·경계·배경을 읽기 쉽다. D455 최대 Depth 해상도 Min-Z는 약 520 mm라 400 mm에서 동일 설정 사용을 전제할 수 없다. **확인함/일반 지식**, [D455 사양](https://store.realsenseai.com/buy-realsense-depth-camera-d455.html). | RGB 중심+왜곡 보정+광선/보정 평면 교차를 기본으로 정하고, 매트·벨트 평면과 단추 윗면의 높이 차를 반영한다. 단추 두께·벨트 요동·TCP 오프셋을 측정하고, 보정에 쓰지 않은 점으로 오차를 검증한다. Depth는 넓은 평면 보정·이상 감시에 우선 사용한다. |
| M05 | `progress_plan.md` §4의 “필요시 핑거 어댑터”, §5·10; `package_plan.md` §2·4 robot control | 그리퍼 모델, 최소 폐쇄 폭, 핑거 두께, 단추 두께, 해제 여유가 없다. 일반 핑거가 매트에 먼저 닿거나 아래 단추를 밀 수 있으므로 단순 개폐 성공은 10 mm 단추 파지 가능성의 증거가 아니다. **일반 지식/확인 필요**. | 핑거 어댑터 검토를 선택 사항에서 초반 기계 검증으로 올린다. 측면 파지 위치·이탈 경로·열림 지연을 측정하고 정적 2단 적층을 통과해야 동적으로 진행한다. 측면 접근이 불가능하면 구멍을 피하는 흡착/씰 설계 등을 비교한다. §2.e 참조. |
| M06 | `package_plan.md` §5 `VERIFY_STACKING`; `progress_plan.md` §10의 114~115행 및 §12 | 위에서 흰 원 하나가 보이고 동반 이동하는 것으로 두 단추가 실제로 쌓였는지 확인할 수 없다. 단추 하나만 남은 경우와 완전 겹침을 구분하지 못한다. 이 상태로 성공률을 보고하면 지표가 무효다. **일반 지식**. | 측면 영상으로 2단 높이와 분리된 두 물체를 확인하거나 독립적인 높이 측정/수동 판독을 기준으로 정한다. `SUCCESS/FAILURE/UNKNOWN`을 사용하고 가림·미확인은 성공에서 제외한다. 중심 오차와 2단 존재·유지 시간을 별도로 기록한다. |
| M07 | `package_plan.md` §5 첫 번째 배치→두 번째 탐지·파지 순서; `progress_plan.md` §8·10 | 남은 시간 계산은 적혀 있으나 첫 번째 배치 직후부터 전체 작업이 가능한지 판정하는 식이 없다. 짧은 벨트에서는 두 번째 파지가 끝나기 전에 목표가 시야·도달 영역을 벗어난다. 물체 속도와 벨트 속도를 같다고 두는 것도 배치 직후 미끄러짐에서 실패한다. **일반 지식**. | `T_available > T_detect2 + T_pick2 + T_transit + T_sync + T_descend + T_release + T_clear + margin`을 첫 번째 투입 전·후와 각 상태 전환에서 검사한다. 가능한 경우 두 번째 위치를 미리 탐지하고 첫 단추를 상류에 놓는다. 실제 속도·가속·미끄러짐을 추적하고 시간 부족은 투입 보류/실패로 처리한다. |
| M08 | `package_plan.md` §4 `PoseArray 또는 Custom`, 메시지 이름만 명시 | 검출 식별자·측정 시각·상태 시각·공분산·유효기간·단위가 미정이다. `PoseArray`만으로 추적 ID와 관측 품질을 전파할 수 없다. ROS의 m/rad와 Doosan mm/deg, ROS quaternion과 Doosan 자세 표현을 혼동하면 실제 위험한 목표가 된다. **확인함/일반 지식**, §2.b·g. | 인터페이스 계약을 첫 통합 전에 확정한다. 프로젝트 내부는 SI·quaternion, 드라이버 경계에서만 변환하고 테스트한다. 관측/추정/예측 시각을 구분하고 ID·공분산·만료·취소 결과를 필수로 한다. |
| M09 | `package_plan.md` §1 Python 패키지 트리와 워크스페이스 루트 `tests/` | 실행 노드가 있는 ament_python 패키지에 `setup.cfg`, ament resource marker와 설치 규칙이 빠져 있다. 루트 tests는 그 위치만으로 `colcon test`의 패키지 테스트가 되지 않는다. 개략도에서 생략한 것이라면 그 사실을 밝혀야 한다. **확인함**, [ROS 패키지 문서 원본](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.rst). | 각 패키지에 `resource/<name>`, `setup.cfg`, entry points, config/launch 설치를 명시한다. 단위 테스트를 해당 패키지 `test/`에 두고 등록한다. 통합 테스트는 bringup의 `test/` 또는 별도 테스트 패키지에 둔다. 루트 tests를 남기면 별도 실행 명령을 문서화한다. |
| M10 | `progress_plan.md` §1·9·10·11; `package_plan.md` §5 실패→복구 | 안전·통신 손실·보호 정지·소유권 상실이 상태도에 없다. 일부 서보 계열은 운전 속도 조절/힘·컴플라이언스와 연동되지 않는다고 공식 문서에 적혀 있다. 수동 벨트를 ROS로 멈출 수 없으며 자동 복귀가 접촉을 악화시킬 수 있다. **확인함/일반 지식**, [servol](https://manual.doosanrobotics.com/en/api/1.28/publish/cdrflex-servol), §3. | 최초 실기 전에 장비별 안전 절차·속도/가속 제한·중단 경로·관찰자 역할을 정한다. `SAFE_ABORT`, `PROTECTIVE_STOP`, `WAIT_OPERATOR`와 재개 조건을 도입한다. 보호 정지 자동 리셋·무조건 원점 복귀를 금지하고, 통신 끊김 때의 실제 동작을 검증한다. |
| M11 | `progress_plan.md` 전 단계와 §12; `package_plan.md` §6의 6단계 대응 | “성공/오차 측정”만 있고 통과 기준·표본 수·조건이 없어 다음 단계로 갈 판단이 불가능하다. 6단계/12단계 간 대응도 문서화되지 않았다. **확인함(계획 내용)/일반 지식**. | §2.f의 정량 게이트를 초기 가설로 추가한다. 속도 범위·두께·조명·거리·버전을 기록하고, 정적 → 예측 타이밍 → 동기 추종을 따로 통과시킨다. 두 문서에 동일한 마일스톤 ID를 붙인다. |

### 1.2 권장: 5건

| ID | 위치 | 문제·근거 | 구체적 수정안 |
|---|---|---|---|
| R01 | `progress_plan.md` §2와 이후 모든 “Isaac Sim 검증” | 소형 접촉 물리 전체가 초기 병목이 될 수 있다. 시뮬레이터의 이상적인 Depth·접촉은 실기를 보증하지 않는다. **일반 지식**. | 초기에는 기하·TF·시각·추적·상태 전이를 검증하는 단순 시뮬레이션과 실제 그리퍼/비전 실험을 병행한다. 사실적인 접촉은 §3의 수렴 시험 후 확장한다. |
| R02 | `package_plan.md` §2 tracking/planner | tracking의 `motion_predictor.py`와 planner의 미래 예측이 중복될 수 있다. **확인함/설계 판단**. | tracker가 상태·공분산을 소유하고, 순수 함수 예측 모델을 공유하거나 planner가 상태를 받아 미래 접촉을 계산하도록 소유권을 하나로 정한다. 필터를 두 번 적용하지 않는다. |
| R03 | `progress_plan.md` §6·8·12 | 단일 카메라 추정값을 같은 카메라 결과와 비교하면 실측 정확도가 아닌 자기 일관성만 평가한다. **일반 지식**. | 고정 길이 기준자와 독립 영상/외부 측정으로 위치·속도를 검증한다. 시뮬레이터 GT와 실기 GT를 구분하고 프레임 누락·가림이 든 rosbag 회귀 자료를 남긴다. |
| R04 | `package_plan.md` §1의 전 제어 Python 구성 | 비전 처리와 Python executor가 고주기 지령을 공유하면 지터가 커질 수 있다. Python이라는 이유만으로 실패를 단정하지는 않는다. **일반 지식**. | 제어 루프와 비전을 분리하고 측정 결과가 요구 주기를 못 맞추면 드라이버 어댑터/서보 루프만 C++로 옮긴다. 상태 머신과 저속 시퀀스는 Python을 유지한다. |
| R05 | `package_plan.md` §4 절대 토픽명·`std_msgs/String`; §1 calibration | 네임스페이스 재사용, 상태 오류 코드, 보정 provenance가 약하다. 고정 카메라를 단순 eye-in-hand로 다루면 잘못된 변환이 된다. **일반 지식**. | 상대 토픽명+launch namespace, 타입화된 TaskStatus, 보정 ID/날짜/잔차/카메라 프로파일/단위 저장을 권장한다. eye-to-hand 관계와 로봇 base 기준을 명시한다. |

### 1.3 선택: 3건

| ID | 위치 | 문제·근거 | 구체적 수정안 |
|---|---|---|---|
| O01 | `progress_plan.md` §6~10 | 엔코더 없는 속도 추정의 상한이 존재한다. **일반 지식**. | 과제 조건이 허용하면 독립 벨트 마커·측면 카메라·엔코더를 비교 실험한다. 추가 하드웨어를 필수라고 가정하지 않으며, 물체 미끄러짐은 별도로 추정한다. |
| O02 | `package_plan.md` §1~2 | 9개 패키지는 작은 과제에 관리 부담일 수 있으나 ROS 관례 위반은 아니다. **설계 판단**. | 구현 인원이 적으면 task/planner를 하나의 배포 패키지 안 모듈로 시작할 수 있다. 인터페이스·드라이버 경계는 유지한다. 패키지 수만 줄이는 리팩터링은 필요 없다. |
| O03 | `progress_plan.md` §9·12 | 고급 모델 예측 제어는 가능한 확장이지만 초기 성공에 필수는 아니다. **설계 판단**. | 일정 속도 모델+제한된 피드백부터 시작한다. 가속 변화에서 예측 오차가 지배적이라는 측정 결과가 있을 때만 가속 모델/MPC/온라인 재계획을 추가한다. |

## 2. 요청 질문별 답변

### a. D455, 10 mm 단추: RGB 해상도·Depth와 위치 추정

**확인함:** 현재 공식 D455 제품 표는 RGB 1280×800/30 fps, RGB FOV 90°×65°, global shutter, Depth 최대 1280×720, 최대 해상도 Min-Z 약 52 cm, Depth 정확도 “4 m에서 2% 미만”을 명시한다. 과거 공식 소개에는 FOV 86°가 보이므로 실제 계산·보정은 연결된 프로파일의 `CameraInfo`를 사용해야 한다. [현재 D455 공식 사양](https://store.realsenseai.com/buy-realsense-depth-camera-d455.html), [과거 공식 소개](https://www.realsenseai.com/news-insights/which-device-is-right-for-you/).

**일반 지식에 따른 계산:** 카메라가 평면에 수직이고 광학 중심 근처라는 가정에서 수평 관측 폭 `W = 2 Z tan(HFOV/2)`, 단추 직경 픽셀 수 `d_px = 1280 × 0.010 / W`이다. 여기서 Z는 광학 깊이이며 경사진 설치의 단순 직선거리와 같지 않다.

| Z | HFOV 90°의 mm/px | 10 mm 단추 직경 | HFOV 86°일 때 직경(참고) |
|---|---:|---:|---:|
| 0.4 m | 0.625 | 16.0 px | 17.2 px |
| 0.6 m | 0.938 | 10.7 px | 11.4 px |
| 0.8 m | 1.250 | 8.0 px | 8.6 px |

8~16픽셀은 검출 가능성과 정밀 적층 가능성을 구별해야 하는 크기다. 단추 구멍, 하이라이트, 그림자, 일부 가림이 몇 픽셀만 바뀌어도 중심에 영향이 크다. 경사 촬영은 짧은 축을 더 줄인다. global shutter라도 노출 중 이동 흐림은 남는다. 예를 들어 50 mm/s에서 노출 10 ms면 물체가 0.5 mm 이동한다(계산 예시이며 실제 속도가 아니다).

**권고(일반 지식):** 왜곡 보정 RGB에서 외곽 원/타원과 마스크로 중심을 추정하고, 광선과 작업 평면의 교점을 구한다. 매트와 벨트에 각각 평면/호모그래피를 둔다. 원시 왜곡 픽셀에 단일 호모그래피를 바로 적용하지 않는다. 3D 외부 보정이 있으면 단추 두께를 더한 평면으로 광선을 교차시키는 방식이 높이 보정에 편리하다. 고정 카메라의 로봇 base 외부 보정과 TCP 보정을 서로 독립적으로 확인한다.

**추정:** 잘 조명되고 정지한 물체의 중심 검출이 0.2~0.5 px 정도라면 이미지 양자화/검출 성분만 약 0.13~0.63 mm이다. 이는 보장 정확도가 아니다. 보정·렌즈 잔차·평면 휨·로봇 절대 위치·TCP·파지 편차를 포함한 초기 총 XY 오차는 대략 1~3 mm 수준을 계획 가설로 놓되, 나쁜 조건에서는 더 클 수 있다. 2 mm 이하 적층 중심 오차를 원한다면 카메라 위치 오차에는 약 1 mm 이하의 예산을 배정하고 독립 검증으로 충족 여부를 판단한다. 두께 2 mm를 무시하고 카메라 높이 600 mm, 광축에서 200 mm 떨어진 점을 바닥 평면으로 투영하면 약 0.67 mm 편차가 생긴다(`r Δh/H`, 근사 계산).

**Depth 판단:** 4 m의 상대 정확도 사양을 0.4~0.8 m 단추의 ±8~16 mm 정확도나 sub-mm 정확도로 단순 환산할 수 없다. 넓은 평면의 평균 Depth는 안정화할 수 있어도 작은 경계 픽셀의 stereo matching·RGB 정렬·구멍 혼합 오차는 별개다. 단추 위 단일 Depth를 Z 정답으로 쓰지 않는다. 근거리 실제 오차는 **확인 필요**이며, 0.6/0.8 m에서 평면 및 단추 ROI의 유효 픽셀률·편향·시간 표준편차·경계 이상치를 기준 높이와 비교한다. 정밀도와 절대 정확도를 분리해 보고한다.

**확인함:** D400 데이터시트 표 4-11은 D455 Min-Z를 1280×720에서 520 mm, 848×480에서 350 mm로 명시한다. 따라서 0.4 m에서 RGB를 쓰는 것은 가능해도 최대 해상도 Depth가 유효하다고 가정하면 안 된다. 프로파일을 낮추면 최소 거리는 줄지만 공간 샘플링도 달라진다. [공식 데이터시트 91쪽](https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf#page=91).

### b. doosan-robot2 Jazzy 제어 인터페이스·주기·지연

**확인함:** 검토 시 공개 `jazzy` 브랜치 HEAD는 `dade65d3a99dd8b29d8eca3689a4ead74f228d77`이었다. 아래는 이 커밋에서 확인한 인터페이스이며, 현장 설치 커밋이나 E0509의 실제 제어기 지원을 확인한 것이 아니다. README는 제어기 3.x 사용 시 별도 빌드 옵션을 설명한다. 따라서 ROS 배포판 이름만으로 호환성이 확정되지 않는다. [고정 커밋 README](https://github.com/doosan-robotics/doosan-robot2/blob/dade65d3a99dd8b29d8eca3689a4ead74f228d77/README.md).

| 후보 | 확인한 ROS 경로/형식 | 주기·지연에 대해 말할 수 있는 것 | 용도·제약 |
|---|---|---|---|
| 일반 Cartesian 위치 stream | `<ns>/servol_stream`, `dsr_msgs2/msg/ServolStream`; pos[6], vel[2], acc[2], time | ROS 구독 callback이 DRFL `servol` 호출. 공식 일반 API 문서에 이 시스템의 보장 지령 주기·왕복 지연은 없다. **확인 필요**. 50~100 Hz를 초기 측정 후보로 삼는 것은 설계 제안일 뿐 지원 보장이 아니다. | 최근 목표를 속도·가속도 제한 내에서 추종. `time`은 도달 시간이지 이미지 timestamp가 아니다. 목표 시간이 제한 때문에 바뀔 수 있다. 일반 속도 조절 및 힘/강성 제어 연동을 가정하지 않는다. |
| 일반 Cartesian 속도 stream | `<ns>/speedl_stream`, `SpeedlStream`; vel[6], acc[2], time | 실제 안정 주기·지연 **확인 필요**. 문서의 통신 timeout 관련 서술은 버전 및 오탈자 확인이 필요하며, 카메라 프레임마다 한 번 보내는 설계를 기본으로 삼지 않는다. | `v_cmd = v_target + Kp (p_target - p_tcp)`처럼 위치 피드백을 바깥 루프에 둘 수 있다. 가속/속도/작업영역 제한이 필수. 속도 명령 중단 동작은 실기로 검증한다. |
| RT Cartesian 위치 stream | `<ns>/servol_rt_stream`, `ServolRtStream`; pos/vel/acc 각 6, time | API 1.31은 제어기 내부 주기 1 ms를 언급하지만 ROS end-to-end 1 kHz 보장이 아니다. 이 버전 문서는 통신 상태에 따른 jerk와 `time ≥20 ms` 권고도 명시한다. | `realtime/connect_rt_control`, 입출력 버전 조회·설정, start/stop 등의 초기화 확인 필요. 서보 목표 도달 시간과 발행 간격을 구분한다. 시간 인수 0의 의미·유효성 검사도 설치 소스와 대조한다. |
| RT Cartesian 속도 stream | `<ns>/speedl_rt_stream`, `SpeedlRtStream`; vel/acc 각 6, time | API 1.31은 내부 1 ms, 새 명령 없이 0.1 s 경과 시 timeout 정지를 설명한다. 이는 현장 지연이나 100 ms까지 안전하다는 보장이 아니다. | 위치 피드백을 추가한 유력 후보. 힘/컴플라이언스 및 Operation Speed [%] 연동 제한을 확인한다. 마지막 속도 유지 구간이 있으므로 별도 stale-command 대응 필요. |
| `move_line` / `amovel` | `<ns>/motion/move_line`, `MoveLine.srv`; sync_type=0은 movel, 그 외는 amovel 분기 | 이동 단위 서비스이므로 주기적 서보 재지령의 지연·덮어쓰기 보장이 없다. 동기 호출의 완료 지연과 비동기 요청 수락을 혼동하면 안 된다. | 정적 pick/place와 미리 계획한 저속 접근에 적합. 작은 선분을 반복 보내면 연속 동기 추종이 된다고 가정하지 않는다. 비동기도 기존 이동의 안전한 갱신을 자동 보장하지 않는다. |

ROS 경로와 callback, 큐 깊이는 [controller 소스 2331~2463행 및 501~518행](https://github.com/doosan-robotics/doosan-robot2/blob/dade65d3a99dd8b29d8eca3689a4ead74f228d77/dsr_controller2/src/dsr_controller2.cpp#L2331), 메시지 필드는 [ServolStream](https://github.com/doosan-robotics/doosan-robot2/blob/dade65d3a99dd8b29d8eca3689a4ead74f228d77/dsr_msgs2/msg/ServolStream.msg), [ServolRtStream](https://github.com/doosan-robotics/doosan-robot2/blob/dade65d3a99dd8b29d8eca3689a4ead74f228d77/dsr_msgs2/msg/ServolRtStream.msg), [MoveLine](https://github.com/doosan-robotics/doosan-robot2/blob/dade65d3a99dd8b29d8eca3689a4ead74f228d77/dsr_msgs2/srv/MoveLine.srv)에서 확인했다. 문서상 동작·제약 근거는 [일반 servol API 1.28](https://manual.doosanrobotics.com/en/api/1.28/publish/cdrflex-servol), [servol_rt API 1.31](https://manual.doosanrobotics.com/en/api/1.31/publish/cdrflex-servol_rt), [speedl_rt API 1.31](https://manual.doosanrobotics.com/en/api/1.31/publish/cdrflex-speedl_rt)이다. 서로 다른 문서 버전이므로 그대로 현장 동작이라고 단정하지 않는다.

**확인함:** `set_rt_control_output`의 통신 period는 API 1.31에서 0.001~1 s이며 이는 **로봇→외부 상태 출력 설정**이다. 이 값으로 외부→로봇 지령 주기나 ROS 지연을 보장할 수 없다. 해당 문서는 `loss` 인수가 현재 사용되지 않는다고도 명시한다. [RT 출력 설정](https://manual.doosanrobotics.com/en/api/1.31/publish/cdrflex-set_rt_control_output).

**확인함 + 설계 추론:** 검토 소스의 일반 servol/RT 구독은 깊이 20, 일반 speedl은 깊이 10이며 stream 메시지에는 Header가 없다. 오래된 지령을 구독 측에서 timestamp로 거절할 수 없는 계약이다. 발행 측 depth=1만으로 이미 수신된 큐를 지울 수는 없다. 따라서 설치 드라이버의 실제 QoS·executor 정체·callback 처리 시간을 측정하고, 최신 값만 전달하는 어댑터와 제어 권한 하나를 둔다. 드라이버 수정이 필요하면 별도 변경으로 검토한다.

**확인 방법(후속 현장 작업, 이번 검토에서 실행 안 함):**

1. 설치 git SHA, DRFL/API 라이브러리와 로봇 제어기 버전, real/sim 모드를 기록한다. `ros2 topic list -t`, `ros2 service list -t`, `ros2 interface show dsr_msgs2/msg/ServolStream`, `ros2 topic info --verbose <실제 토픽>`으로 계약을 확인한다.
2. 제조사 지원표/설치 매뉴얼에서 E0509와 제어기 조합의 RT 사용 가능 여부·제어 권한·모드 전환·패킷 주기를 확인한다. API가 존재하는 것과 장비가 수락하는 것은 별개다.
3. 안전한 높이·작은 진폭으로 실제 TCP 응답, 지령 송신/수신, 상태 측정 시각을 기록해 지연 p50/p95/p99·최대 간격·추종 오차를 산출한다. 서비스 응답 시간만 측정하지 않는다.
4. 비전 부하와 rosbag 기록을 켠 상태에서 반복한다. 통신 단절·stale·cancel을 시험하고 정지/재개를 관찰한다. 30 fps 비전과 서보 루프는 분리하고 서보가 측정 시각 기준 상태를 자체 예측한다.
5. ROS SI/quaternion → Doosan mm/deg 및 해당 API의 Euler 규약(예: servol_rt 문서의 ZYZ)을 전용 어댑터에서 변환한다. 속도 6성분의 회전 규약도 확인하고 초기 실험은 고정 자세로 제한한다. `time`에 절대 ROS 시각을 넣지 않는다.

### c. Moving Frame TF 설계

**확인함:** tf2 Jazzy cache는 요청 시각이 최신 동적 변환보다 미래면 extrapolation 오류를 반환한다. 조회 timeout은 데이터 도착을 기다릴 뿐 운동 예측을 추가하지 않는다. 정적 변환의 시간 독립성과 동적 변환의 제한은 구분해야 한다. [tf2 cache 구현](https://github.com/ros2/geometry2/blob/jazzy/tf2/src/cache.cpp).

**권고(일반 지식):** 고정 구조는 `world → robot_base → conveyor_frame`, `robot_base → camera_link → camera_color_optical_frame` 등 실제 URDF/드라이버 명칭과 일치시킨다. `conveyor_frame`은 벨트 장치에 고정된 좌표계이며 벨트 표면 속도 때문에 계속 이동시키지 않는다. 로봇 링크 TF는 joint state/robot_state_publisher, 카메라 optical TF는 카메라 드라이버의 발행권을 존중하고 중복 broadcaster를 만들지 않는다.

`button_<track_id>_frame`은 관측 또는 그 시각의 추정값을 시각화할 때만 발행한다. 물리적으로 원형 단추의 yaw는 관측 불가능할 수 있으므로 identity 또는 벨트 방향의 관례적 orientation임을 밝히고, 실제 파지 자세와 구분한다. TF를 제어 상태의 유일한 저장소로 삼지 않는다.

데이터 흐름은 다음과 같다.

1. 검출의 `header.stamp = 영상 측정 시각 t_obs`, frame=카메라 optical 또는 보정된 conveyor 기준.
2. `t_obs`에서 좌표 변환 후 필터에 넣는다. 카메라와 base가 고정이어도 로봇 mask에 쓰는 TCP/link 상태는 영상 시각에 맞춘다.
3. tracker는 상태 기준 시각 `t_state`, 마지막 실제 관측 `t_last_obs`, 위치·속도·공분산을 보낸다.
4. planner는 `t_contact`에 대해 `p_hat(t_contact) = p_hat(t_state) + v_hat Δt` 등을 계산한다. 공분산/가속 한계와 reachable 영역까지 검사한다.
5. controller는 base 기준 수치 목표와 유효기간을 사용한다. 미래 `button_frame` TF 조회를 하지 않는다. 미래 목표는 Marker/예측 메시지로 보여주고 관측 TF와 섞지 않는다.

`TimePointZero`로 항상 최신 TF를 가져오면 시각이 불일치한 데이터가 섞일 수 있다. 미래 TF를 미리 발행해 오류만 없애는 것도 권하지 않는다. 지연 영상의 TF 보존 기간을 확보하고, 재시작·`/clock` 역행 시 track/plan을 무효화한다.

### d. 팔 가림·추적 손실·타임스탬프 및 지연 보상

**일반 지식:** 가림은 예외라기보다 적층의 정상 단계다. robot URDF를 영상에 투영한 mask, 접근 방향 변경, 첫 단추의 상류 관측 구간 확보, 측면 보조 시점으로 가림을 줄인다. 마스크 밖의 벨트 텍스처/마커 속도는 보조값이며 단추 미끄러짐을 직접 관측하는 값은 아니다. 두 번째 단추와 첫 단추의 identity를 geometry·예측 gate·ROI·task phase로 분리한다.

가림 전 충분한 관측으로 위치/속도를 확정한 뒤 짧게 예측한다. 재검출 시 큰 innovation을 그대로 점프 목표로 쓰지 않고 association 및 이상치를 검사한다. 가림 중 사람이 다이얼을 조절하면 예측이 무너질 수 있으므로 초기 시연은 사이클 중 다이얼 고정 조건으로 제한하고, 속도 급변을 강건성 시험으로 따로 다룬다.

**추정식:** 축별 보수적 오차 예산을 `e_pred ≤ e_position + |δv| τ + 0.5 a_unmodelled τ² + |v| δt`로 둔다. 예를 들어 가림 τ=0.2 s, 속도 오차 2 mm/s, 미모델링 가속 20 mm/s²이면 가림 성분만 0.4+0.4=0.8 mm이다. τ=0.5 s이면 1+2.5=3.5 mm로 커진다. 이 값들은 장비 측정값이 아니라 허용 가림 시간을 산출하는 예시이다. 다른 오차를 합친 총 예산이 한계를 넘으면 해제 전에는 중단한다.

타임스탬프는 다음 시각을 분리한다: 센서 노출/측정, 호스트 수신, 검출 완료, 필터 상태 기준, 계획 생성, 지령 송신, 실제 TCP 상태 측정, 해제 요청, 물체가 실제 떨어진 시각. RealSense wrapper의 header가 어느 clock domain과 측정 지점을 나타내는지 확인하고 하드웨어 시계↔ROS 시간 offset/drift를 기록한다. 단순히 수신 시각을 영상 측정 시각으로 바꾸면 전송 지연이 숨는다.

`t_now - t_obs`가 이미 취득·처리·대기 지연을 포함하면 이를 예측에 다시 더하지 않는다. 현재에서 실제 접촉까지 남은 지령 전달·로봇 이동·하강·해제 지연을 추가하여 `t_contact`를 구한다. 평균 지연은 예측으로 보상할 수 있지만 지터는 불확실성으로 남는다. 50 mm/s에서 20 ms의 미보상 시간 오차는 1 mm이다(계산 예시).

실기 메시지/TF는 같은 ROS 시간 도메인을 쓰고, 프로세스 실행시간·watchdog에는 단조 시계를 쓴다. 시뮬레이션은 모든 참여 노드의 `use_sim_time`을 일치시키고 실기 clock과 섞지 않는다. 이미지 큐는 오래 쌓이지 않게 하고, 제어는 freshness·deadline·sequence를 검사한다. ROS QoS deadline 알림 자체는 로봇을 멈추는 안전 장치가 아니다.

### e. 2지 그리퍼·해제와 실제 적층 판정

**일반 지식/확인 필요:** 단추 직경만으로 파지 가능성을 판단할 수 없다. 실제 두께·곡률·구멍·질량, 그리퍼 최소 폭·평행도·해상도·최소 힘·응답 시간, 핑거 측면 접촉 높이와 매트 여유가 필요하다. 옆면을 잡는 얇은 핑거는 파지할 수 있어도 하단이 아래 단추 높이까지 내려오면 적층 때 충돌한다. 단추를 들어 올릴 때 부착/이중 파지, 이동 중 slip, 열릴 때 한쪽 핑거에 붙는 현상까지 본다.

핑거 설계는 좁은 접촉 돌기·교체 가능한 연질 패드·작은 V홈 등을 후보로 삼되 실제 단추를 끼워 보는 시험으로 정한다. 단추를 파지하는 동안 하부가 핑거보다 충분히 돌출되는지, 해제 시 양쪽 핑거가 아래 단추와 벨트를 건드리지 않는지 단면 치수로 확인한다. 필요한 force를 모르므로 수치 파지력을 여기서 지정하지 않는다.

흡착은 측면 충돌을 줄이는 대안이지만 단추 구멍 때문에 진공이 새고 노즐이 지름 10 mm보다 커질 수 있다. 구멍을 피하는 작은 컵, 환형 씰, 진공 감지와 실제 해제 지연을 시험한다. 공기 blow-off가 아래 단추를 밀지 않는지도 확인한다. 이미 장착된 2지 그리퍼의 성능을 보지 않고 흡착을 정답으로 단정하지 않는다.

동적 해제는 XY를 맞추는 순간만으로 끝나지 않는다. 접촉 전후의 상대 XY 속도·수직 속도를 작게 하고, 필요한 동안 동기 이동을 유지한 채 손가락을 연 다음 충분한 여유를 확보해 후퇴한다. 해제 요청부터 물체 이탈까지의 p95 지연을 측정한다. 1 mm 자유낙하만으로도 약 14 ms가 걸리며, 수평 속도 차 50 mm/s면 약 0.7 mm 상대 이동이 추가된다(이상화된 계산). 따라서 낮은 release gap과 속도 일치가 중요하다.

**성공 판정 권고:** 측면 고정 영상 또는 독립 높이 계측에서 두 층 존재, 벨트 위 안정 자세, 단추 중심 차 ≤제안 허용값을 확인한다. 도구가 물러난 뒤 최소 1 s 또는 정해진 이동 거리 동안 유지되어야 성공으로 본다. 단추 두께를 h라 하면 판정 센서가 h와 2h를 분리할 수 있는 불확실성을 가져야 하며, D455 Depth로 가능하다고 사전 가정하지 않는다. 초기에는 동기 촬영한 측면 영상의 수동 판독을 기준 정답으로 사용해도 된다. 최상단 RGB 원 하나·그리퍼 열림 신호·로봇 접촉력만으로 두 층을 증명할 수 없다.

### f. 정량 목표·마일스톤 순서

아래 수치는 모두 **추정/제안 합격 기준**이다. 실제 벨트의 최저 안정 속도·단추 두께·그리퍼가 확인되면 조정해야 하며, 제품 사양이나 실험 결과가 아니다. 최초 동적 운전 범위는 가능하다면 10~30 mm/s의 일정 속도로 제한하고, 실제 최소 속도가 더 높으면 관측/도달 구간과 시간 예산을 다시 계산한다.

| 권고 게이트 | 기존 단계 대응 | 정량 합격 기준 초안 및 검증 |
|---|---|---|
| G0 계약·기계·안전 타당성 | 1 + 4 일부, 11의 중단 규칙 선행 | 실제 그리퍼·단추 치수 기록, 단위 변환 검증, 제어권 단일화. 통신/추적 손실 및 cancel을 모사한 20회에서 정의된 중단/운전자 대기 전환 20/20, 예기치 않은 재개 0회. 안전거리·허용 중단 시간은 현장 위험 평가로 별도 결정. |
| G1 비전/보정 | 3 + 4 검출 부분 | 보정에 안 쓴 20개 이상 위치에서 XY p95 ≤1 mm, 최대 ≤2 mm를 목표로 한다. 카메라 높이·조명 조건마다 검출 recall ≥99%, 잘못된 목표 선택 ≤1%/시험 프레임으로 측정. 프레임 상관을 고려해 위치·사이클 단위 결과도 보고. |
| G2 정적 파지·적층 | 4~5 | 50회에서 단일 단추 파지 ≥95%, 정적 2단 적층 ≥90%, 독립 계측 중심 오차 p95 ≤2 mm. 성공·실패·판정 불가 수 및 1 s 유지 여부 기록. |
| G3 추적·시각 계약 | 6~7, 8 지연 측정 선행 | 속도 조건별 60 s 이상 관측, 독립 GT 대비 XY p95 ≤1 mm, 속도 오차 p95 ≤2 mm/s. timestamp 역행 0회, 명시한 clock 오차 목표 ≤5 ms. 가능한 관측 구간에서 유효 track 비율 ≥99%; 가림 구간은 따로 집계. |
| G4 예측·타이밍 적층 | 8 + 새 중간 게이트 | 50/100/200 ms 예측 horizon별 독립 미래 관측과 대조, 접촉 horizon의 XY 예측 오차 p95 ≤1.5 mm. 고정된 intercept 위치/시각의 저속 적층 30회에서 성공 ≥80%를 초반 목표로 설정. 최소 체공·해제 지연을 포함하고, 완전 무추종인 경우 상대 속도로 정해지는 허용 시간창을 충족해야 한다. |
| G5 비접촉 동기 추종 | 9 | 안전 높이에서 조건별 60 s, TCP-목표 상대 XY p95 ≤1 mm, 상대 속도 p95 ≤2 mm/s. 제어 간격/지연 분포 및 freshness 위반을 기록. 예: 100 Hz를 선정했다면 주기 10 ms, 간격 p99 ≤15 ms를 목표로 측정하되 제조사 지원을 먼저 확인. |
| G6 동적 적층 | 10 | 실제 가능한 저/중/고 3속도에서 각 50회, 초기 ≥80%, 개선 목표 ≥90%. 중심 오차 p95 ≤2 mm, 1 s 이상 유지. 해제 직전 위치/속도/가림 gate 통과 때만 실행. 부적합 조건에서 시도하지 않은 횟수도 공개. |
| G7 통합·복구·비교 | 11~12 | 100개 전체 사이클에서 성공·실패·안전 중단·판정 불가·cycle time 분포 공개. 위치 오차는 성공 사례만 고르지 않고 측정 가능한 실패도 포함. 속도 변경·가림·영상 누락·제어 지연을 주입하여 각각의 중단 조건 확인. |

G1의 1 mm와 G5의 1 mm를 통과해도 총 적층 오차 2 mm가 자동 보장되지는 않는다. 예측·TCP·보정·해제 성분의 상관과 편향이 있어 단순 RSS가 부적절할 수 있으므로 최종 end-to-end 오차를 별도로 측정한다. 정적 위치 오차·추종 오차·해제 편차의 예산을 할당하고 가장 큰 항목부터 개선한다.

순서는 `G0 → G1/G2 → G3 → G4 → G5 → G6 → G7`을 권장한다. 시뮬레이션 2단계는 병행하며 G0~G2를 모두 막는 선행 조건으로 만들지 않는다. G4의 예측 타이밍은 실현 가능한 작은 시간창/저속에서의 기준선이며, 2지 그리퍼 특성상 정지 TCP로 적층이 불가능하면 짧은 사전 계획 속도 일치 구간을 허용하고 그 조건을 공개한다. G4를 통과했다는 이유로 G5를 생략하지 않는다.

비교 실험은 세 방법의 제어 품질 차이로 정의한다: 고정 좌표(지연/운동 보상 없음), 미래 목표+계획된 접촉 시각, 지속 갱신된 목표+속도 동기 추종. TF를 쓰는지 여부만 비교 변인으로 삼지 않는다. 동일 속도·조명·그리퍼·실패 정의에서 비교하고 성공률에 표본 수와 Wilson 등 이항 신뢰구간을 붙인다. 50회 중 45회 성공은 90% 관측값이지 90% 이상의 신뢰성 보증은 아니다.

### g. 9개 패키지와 ROS 2 인터페이스·테스트

**일반 지식/설계 판단:** 9개는 허용 가능한 책임 분리다. 패키지는 노드 한 개와 동의어가 아니며 초기부터 9개 완성이 필요하지 않다는 원 계획도 타당하다. 별도 벨트 명령 패키지를 만들 필요는 없다. 부족한 것은 패키지 수가 아니라 보정·검증·시간·안전 감시·드라이버 어댑터의 책임 정의다. `button_robot_control`은 Doosan adapter와 gripper adapter, task manager는 orchestration과 결과 판정, bringup은 launch/공통 설정을 소유하도록 한다. verify 모듈을 vision/task에 두거나 규모가 커질 때만 패키지로 분리한다.

인터페이스 초안(프로젝트 내부 m, rad, s; 위치와 속도의 frame을 일치시킴):

| 형식 | 최소 필드와 의미 |
|---|---|
| `ButtonDetectionArray` | `Header`(원 영상 측정 시각·frame), `image_sequence`, `calibration_id`, detections[]. |
| `ButtonDetection` | 관측 ID(영속 track ID와 구분), 영역 enum(MAT/BELT), `u/v`, pixel diameter/ellipse, confidence, 2D center covariance, validity; bbox/contour는 필요할 때만. |
| 위치 관측 Custom | `Header`, detection ID, `Point`+3×3 covariance, plane/calibration ID, localization method enum. 추정할 수 없는 yaw를 정확한 Pose처럼 주지 않는다. |
| `TrackedButton` | `Header.stamp=t_state`, 고유 `track_id` 및 reset epoch, `last_observed_stamp`, position/velocity와 그 교차항을 포함한 상태 공분산(예: belt 2D 상태 [x,y,vx,vy]의 4×4), `OBSERVED/COASTING/LOST`, observation count, `valid_until`. |
| `BeltMotion` | `Header`, 기준 frame의 방향·속도/선택 가속도, covariance, 추정 window, source enum(BELT_TEXTURE/BUTTON/기타), last observation, valid. 단추 한 개에서 추정하면 belt speed를 직접 측정한 것처럼 명명하지 않는다. |
| `InterceptPlan` | plan ID/revision, target track ID/epoch, 생성 Header, source state stamp, `contact_time`, `valid_until`, base-frame target TCP pose, target linear velocity, release command time 또는 지연 모델 ID, uncertainty, reachable/status/reason. 절대 시각과 duration을 명확히 구분. |
| `StackingTask.action` | Goal: mode(STATIC/PREDICTED/SYNCHRONIZED), 목표/영역, 허용 오차·timeout. Feedback: state enum, target/plan ID, 추정 오차·남은 시간·품질. Result: SUCCESS/FAILURE/UNKNOWN/CANCELED, reason code, 중심 오차/유지 시간과 validity, 검증 수단. 취소는 실제 정지·후퇴 결과를 확인 후 완료. |
| `TaskStatus`/diagnostics | 상태 enum, error code, command owner, freshness, 그리퍼/로봇 연결·보호 정지 상태. `std_msgs/String`은 사람용 로그에는 가능하나 제어 계약은 타입화. |

Doosan stream이 Header 없는 배열이라고 해서 프로젝트 전 구간도 배열·무시각 데이터로 만들지 않는다. adapter가 freshness와 유효성을 검사하고 마지막 단계에서 제조사 메시지로 변환한다. 로봇 TCP의 피드백도 상태 측정 시각과 목표 계산 시각을 맞춰 사용한다. pose에는 geometry_msgs 표준을 활용하되 원형 물체의 미관측 orientation과 위치 공분산을 문서화한다.

**확인함/관례:** ament_python 패키지에는 resource marker, 실행 파일 검색을 위한 setup.cfg, console_scripts, package.xml 및 config 설치 규칙이 필요하다. interface 패키지는 ament_cmake/rosidl 생성, `rosidl_default_generators`, `rosidl_default_runtime`, 의존 메시지와 `rosidl_interface_packages` 그룹을 선언한다. sim/bringup은 `ament_package()`와 자원 설치를 포함한다. [ROS 2 패키지 생성 문서 원본](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.rst).

테스트 위치 권고:

- `button_vision/test/`: 왜곡/평면 변환, 알려진 타깃 위치, 단추 구멍·그림자·경계 사례.
- `button_tracking/test/`: 시간 간격 변화, frame drop, out-of-order, ID 전환, 가림·재획득 및 공분산 증가.
- `button_stacking_planner/test/`: 접촉 시간 feasibility, 시야/도달영역 이탈, 공분산/만료, 속도 변경·멈춤.
- `button_robot_control/test/`: m↔mm, quaternion↔제조사 자세, command ownership, stale·cancel·gripper 실패의 fake-driver 시험. 하드웨어 동작 테스트는 별도 명시적 실행으로 분리.
- `button_stacking_bringup/test/`: launch_testing 및 rosbag replay로 end-to-end 계약·시뮬레이션 시간·노드 재시작 검증.

각 test를 package.xml/build backend에 등록하고 `colcon test --packages-select ...`, `colcon test-result --verbose`로 발견 여부·실패를 확인한다. 루트 `tests/` 자체가 금지되는 것은 아니지만 패키지 밖 파일이 자동 등록된다는 가정은 잘못이다. 테스트 도구가 아직 없는 현재 저장소에 대해 통과했다고 주장할 수는 없다.

## 3. 계획에 없는 위험과 대응

| 위험 | 영향 | 대응 및 확인할 증거 |
|---|---|---|
| 협동로봇이라는 이유로 작은 물체 접촉이 안전하다고 가정 | 손가락 끼임, 벨트 닙 포인트, 날카로운 핑거, 물체 튀김. 작은 단추 접촉은 로봇 충돌 감지의 검출 한계보다 작을 수 있음. | 현장 안전 담당자가 배치·가드·안전거리·제한을 확인. 충돌 감지를 단추 접촉 센서나 적층 성공 센서로 대체하지 않는다. 자율 제어와 무관한 정지 수단 확보. |
| 보호 정지·통신 끊김 이후 자동 복귀 | 그리퍼가 단추/벨트와 접촉한 상태에서 복귀해 손상. 수동 벨트는 계속 움직임. | 정지 원인·물체 보유 여부·접촉 상태를 확인하기 전 재개 금지. 해제 전/후 중단 경로를 구분하고 현장 운전자 확인이 필요한 상태를 둔다. |
| 서보 명령이 일반 속도 slider/force mode와 같은 제한을 따른다고 오해 | 예상보다 빠른 움직임, 힘 제어와 충돌, 지연된 목표 실행. | 설치 버전의 서보 제한·안전기능 적용 범위를 확인하고 명령 자체 제한 및 freshness watchdog을 둔다. API 운전 속도 조절 비연동과 안전 정격 기능의 작동 여부를 혼동하지 않는다. |
| 컨베이어 미끄러짐·맥동·진동·표면 이음매 | 일정 속도 가정과 평면 높이가 무너짐. 벨트 속도와 단추 속도 불일치. | 배치 직후 안정화 구간, 벨트 위치별 Z/XY 변동 조사, 실제 물체 추적 유지. 속도 오차/가속 한계 초과 시 해제 gate 차단. |
| 시야·도달범위·IK·특이점·케이블 | 좌표는 도달 가능해도 자세/속도/후퇴 경로는 불가능. | 작업영역을 TCP pose와 joint/velocity 한계까지 포함해 보수적으로 설정. 접촉 후 같은 속도로 이동 가능한 여유와 안전 후퇴를 먼저 확인. |
| 로봇 반복정밀도와 절대 정확도 혼동 | 스펙이 좋아도 카메라 base 보정·TCP 오차로 계속 빗나감. | 다점 절대 위치 확인과 실제 파지한 단추 중심→TCP 오프셋 측정. 툴 교체/충돌/카메라 이동 후 재보정. |
| 조명·노출·USB·CPU/GPU 부하 | 검출 변동, frame drop, 처리 지연·지터. | 조명 고정, 허용 흐림에 맞춘 노출, USB 프로파일 확인, 기록·시뮬레이션 동시 부하 시험. 품질 저하 시 해제 보류. |
| 데이터·clock 재시작 | 이전 track/계획 재사용, 미래/과거 timestamp 혼합. | epoch/sequence와 expiry, 재시작 시 모두 무효화. real/sim namespace·clock을 격리. rosbag 재생이 실제 로봇 command 경로에 연결되지 않게 launch 구분. |
| Isaac Sim의 10 mm rigid body 접촉 | 기본 contact/rest offset, 시간 간격, 마찰·질량·관성으로 튐·관통·가짜 안정 적층. | 단추 실제 두께/질량과 stage 단위 확인. convex collider와 실물 형상 차를 기록. 접촉/정지 offset을 물체 크기에 맞추고 solver·substep/CCD를 필요에 따라 조정. 임의로 질량이나 접착을 늘려 성공시키지 않는다. |
| 가상 벨트 구현 착오 | 벨트 mesh를 순간 이동시키거나 단추 위치를 직접 덮어써 물리 검증이 무의미해짐. | 물리 모드에서는 표면 속도/접촉 마찰로 운반되게 구현하고 slip을 측정. 알고리즘 전용 kinematic 모드는 별도 라벨로 운영하고 물리 성공률과 섞지 않는다. |
| 이상적 가상 카메라/그리퍼 | 시뮬레이션만 성공하고 실물에서는 구멍·광택·가림·해제 지연으로 실패. | 실물 영상 replay, 노이즈/가림/지연 주입, 측정한 gripper delay·마찰 분포로 검증. sim GT를 제어 입력으로 몰래 사용하지 않는다. |
| 검증 불가능한 성공률 | 실패를 UNKNOWN/중단으로 숨기거나 성공 사례만 오차 산출. | 전체 투입 시도·해제 시도·중단·실패·UNKNOWN을 분모와 함께 공개. 자동 판정과 독립 수동 판정의 혼동행렬 작성. |

Isaac Sim 5.1 공식 문서는 작은 contact offset이 접촉 지연·jitter·관통을 만들 수 있으며 CCD와 시간 간격 조정이 영향을 준다고 설명한다. contact offset은 접촉 생성 거리, rest offset은 정지 간격을 정한다. [5.1 Physics Simulation Fundamentals](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html), [5.1 offset API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.experimental.objects/docs/index.html).

**추정/시험 제안:** physics 120~240 Hz를 출발 후보로 비교하되 특정 값이 정답은 아니다. step을 절반으로 줄인 뒤 정적 적층 유지·최대 관통·미끄러짐·동적 성공률이 허용 범위 안에서 수렴하는지 본다. 관통 허용값은 예컨대 단추 두께의 10% 미만부터 검토한다. 렌더 FPS와 physics rate를 분리하고, solver iteration·contact offset·마찰 변화에 대한 민감도를 기록한다. 시뮬레이션에서 100번 성공한 것이 실기 100번 성공을 보장하지 않는다.

## 4. 검토 증거와 남은 확인

검토 중 기록한 저장소 HEAD는 `afd4e163644b8f8a268e8a3d3ed8555a20cdb66c`이다. 최종 `git status --short`에는 두 계획이 untracked(`??`)로 표시되므로 저장소가 clean하다고 주장하지 않는다. 이 worker가 원문을 변경하지 않았음은 읽기 전후 SHA-256 일치로 확인했다. 계획의 SHA-256은 다음과 같다.

- `package_plan.md`: `d2637eb0b0f6d6b4556bc55ae7e71e15f18c57c4eb2f62c94acf090326f23035`
- `progress_plan.md`: `1cd7d632dae647024725aa3dae8a2069a01fbf38bd3ec51bff35a930453d78c4`

공식 GitHub Jazzy 원문과 제조사 매뉴얼·사양은 온라인으로 확인했다. ROS 문서 웹페이지는 접근 차단이 있어 공식 `ros2_documentation` 및 `geometry2` GitHub 원문을 대조했다. 현장 설치 드라이버 SHA, E0509 제어기/API 버전, 실제 안정 서보 주기·지연, 단추 두께/그리퍼 형상, 벨트 속도 범위·유효 작업 길이, 위치·적층 성공률은 모두 **미확인**이다. 보고서의 수치 목표는 이를 측정하기 위한 제안이다.

이 worker는 재분배를 수행하지 않았고 계정/한도·설정을 조회하거나 변경하지 않았다. 다른 에이전트의 사용량 확인과 작업 배정 판단은 coordinator 범위이며, 이 보고서는 그 상태를 확인했다고 주장하지 않는다. 계획 파일 수정·PC 및 로봇 실기 실행은 후속 작업으로 남는다.

최종 문서 검증(2026-11-04 10:46 UTC, GCE): 보고서 존재, 필수 수정 ID 11개, 질문 a~g 전부, 위험 대응 절을 확인했다. 두 원문 SHA-256은 최초 기록과 일치했다. 코드 테스트·장비 성능 시험은 검토 범위 밖이므로 수행/통과로 기록하지 않았다.
