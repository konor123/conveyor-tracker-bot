# 프로젝트 구조와 개발 흐름

## 무엇을 만들고 있나

컨베이어 위 병뚜껑의 위치를 카메라로 보고, 로봇이 공깃돌을 집어 그 병뚜껑 안에 넣는다. 장비는 Doosan E0509, ROBOTIS RH-P12-RN(A), RealSense D455이며 ROS 2 Jazzy로 연결한다([개요](../../README.md)). 정지한 물체에 놓기부터 시작하여 예측 시각에 해제하는 방식, 계속 움직임을 맞추는 방식으로 확장한다.

초기 설계의 핵심은 **판단과 장비 통신을 분리하는 것**이다. 단계 판정·시간 예산·중단 정책은 ROS 없는 Python 함수로 만들고, ROS 노드는 요청과 메시지를 변환하는 역할을 한다. 그래야 현장 장비를 기다리지 않고 실패 조건을 시험할 수 있다([PLAN 0.2·2절](../../PLAN.md)).

배운 점: 로봇 프로젝트의 진척은 코드 양보다 어떤 위험을 어떤 증거로 줄였는지로 읽는다. 흔한 함정: 카메라가 보이고 로봇이 움직였다는 사실을 전체 투입 성공으로 해석하는 것이다.

직접 해 보기(저장소 루트, 읽기 전용):

```bash
sed -n '1,100p' Progress.md
git log --since=2026-11-05 --oneline --stat
```

## 게이트와 현장 세션은 무엇이 다른가

게이트는 합격 조건이고 OFF-n은 현장에서 수행할 작업 묶음이다. 한 세션이 여러 게이트에 자료를 줄 수 있으며, 자료 수집만으로 게이트가 통과하지 않는다. 아래는 [Progress의 게이트·세션 표](../../Progress.md)를 읽기 쉽게 정리한 것이다.

| 게이트 | 확인하는 능력 | 주 통합 단계·현장 세션 |
|---|---|---|
| M0 | 명세·인터페이스·오케스트레이터 골격·최소 대역 | I0, 장비 없이 수행 |
| G0 | 제어·그리퍼·안전 계약과 치수 | I3, OFF-1·2 |
| G1 | D455 검출·좌표 보정 | I1, OFF-1 자료·OFF-3 검증 |
| G2 | 정적 파지·정지 벨트 투입 | I5, OFF-4 |
| G3 | 추적·속도·관측 좌표계·지연 | I1·I4, OFF-1 자료·OFF-5 |
| G4 | 예측 타이밍 투입 | I2→I5, OFF-6 |
| G5 | 비접촉 동기 추종 | I3→I5, OFF-5 |
| G6 | 동기 추종하며 동적 투입 | I5, OFF-6 |
| G7 | 통합·복구·비교 평가 | I5·시뮬레이션, OFF-7 |

트랙 A는 로봇·제어(G0→G2 로봇측→G5), B는 비전·보정(G1→G3), C는 계획·오케스트레이션(시간 예산·예측→G4 로직), D는 시뮬레이션이다. 인터페이스·fixture·bag으로 연결하므로 서로의 구현 완료를 모두 기다릴 필요가 없다. 시뮬레이션은 실기 게이트를 막지 않는다([PLAN 11절](../../PLAN.md)).

배운 점: 일정 단위와 합격 단위를 구분하면 “현장 다녀옴”을 “검증 완료”로 잘못 보고하지 않는다. 흔한 함정: OFF를 원격 접속 여부로 나누는 것이다. 실제 지령·실시간 센서·물리량 측정은 원격이어도 OFF다.

직접 해 보기:

```bash
rg -n '합격 기준|OFF-[1-7]' Progress.md
```

## 저장소와 TDD를 연결하기

저장소 루트 자체가 colcon 워크스페이스다. [src](../../src/)는 ROS 패키지, [specs](../../specs/)는 SPEC·REQ·인터페이스 스냅샷, [tools](../../tools/)는 시험·측정·분석 도구, [records](../../records/)는 결과를 담는다. [AGENTS](../../AGENTS.md)는 작업·기록 규칙이다.

```mermaid
flowchart LR
  S[SPEC의 REQ] --> T[TEST_SPEC의 TC]
  T --> R[실패하는 시험]
  R --> I[최소 구현]
  I --> G[통과와 리팩터링]
  G --> X[trace로 연결 검사]
```

REQ는 지켜야 할 계약, TC는 그 계약을 확인할 구체적 입력·기대 결과다. 테스트 함수에는 req·tc·level 마커를 붙인다. 경계는 직전·정확한 경계·직후를 확인하고 FakeClock으로 ROS 시각과 steady 시각을 독립 조작한다. 공통 pytest 훅은 패키지 test/conftest.py가 가져온다([TEST_SPEC 0·1절](../../TEST_SPEC.md), [공통 훅](../../tools/testing/drop_pytest.py)).

배운 점: trace는 “어떤 시험이 요구사항에 연결됐는가”를 검사한다. 시험 assertion의 정확성·실행 통과까지 보증하지 않는다. 흔한 함정: 구현의 상수를 그대로 정답으로 가져와 구현과 시험이 같은 실수를 공유하는 것이다. 11월 9일에는 [독립 metric oracle](../../tools/testing/spec_metrics.py)을 두어 이 문제를 줄였다.

직접 해 보기(GCE, 문서·순수 Python 확인):

```bash
python3 tools/spec/trace.py --gate M0 --check
python3 -m pytest -q tools/spec/test_trace.py
```

## 대역과 계약 시험, I0~I5

오케스트레이터가 중심인 구조여도 입력·응답·장애를 만들어 줄 상대가 필요하다. ROS 대역은 세계 모델인 mock_world와 로봇 계약을 제공하는 mock_robot_controller 두 종류다. 작은 함수 시험은 fake, 실영상 회귀는 rosbag, 예측 계산은 fixture로 역할을 나눈다([PLAN 7절](../../PLAN.md), [SPEC-700](../../specs/SPEC-700-mocks.md)). 계약 시험은 대역과 실제 구현에 같은 외부 기대값을 적용해 교체 가능성을 확인한다. 대역의 예쁜 동작이 실제 로봇의 동작을 증명하지는 않는다.

| 단계 | 구성과 확인 대상 |
|---|---|
| I0 | 세계·로봇 대역 + 실제 오케스트레이터: 정상 전이·중단·제한시간 |
| I1 | bag replay + 실제 검출·추적: 실데이터 인식 |
| I2 | 추적 fixture 또는 replay + 실제 planner: 예측·시간 예산 |
| I3 | 합성 관측 + 실제 planner·controller: 안전 높이의 실기 skill·추종 |
| I4 | 실제 카메라→추적→계획 + 로봇 대역: 로봇 지령 없는 인식 경로 |
| I5 | 전체 실제 경로: 투입·복구·게이트 실험 |

배운 점: 통합은 실물 비율을 한 번에 올리는 대신 검증할 경계를 하나씩 바꾼다. 흔한 함정: I0 성공을 I3 안전성이나 I5 성공률로 해석하는 것이다.

직접 해 보기(ROS 설치 PC의 별도 임시 mock 워크스페이스에서만):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=183
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest -q src/drop_bringup/test
```

현장 장비 도메인·현장 저장소 사본을 사용하지 않는다. GCE는 편집·순수 Python, PC는 ROS 빌드·통합·반복 측정, CI는 재현 가능한 자동 회귀를 맡는다([실행 규칙](../../AGENTS.md)).

## 기록은 서로 다른 질문에 답한다

| 기록 | 답하는 질문 |
|---|---|
| [worklog](../../records/worklog/2026-11-09.md) | 누가 어디서 무엇을 바꾸고 어떤 명령으로 확인했나? |
| [devlog.csv](../../records/devlog.csv) | 하루의 시험 수준별 통과 수·REQ 연결률·게이트 상태는? |
| [param_changes.csv](../../records/param_changes.csv) | 어떤 N·통계·파일 근거로 제안값을 실측값으로 바꿨나? |
| [bags.csv](../../records/bags.csv) | 저장소 밖 원본 bag의 경로와 해시는? |
| [sessions](../../records/sessions/)·[bench](../../records/bench/) | 현장 조건·원시 측정·미완 항목은? |

배운 점: 결론은 원시 자료까지 거슬러 갈 수 있어야 한다. 흔한 함정: 빈 미결 이슈 수를 0으로 읽거나 skip을 pass로 더하는 것이다. [11월 9일 devlog](../../records/devlog.csv)의 open_issues 빈칸은 미측정이며 robot_hours는 그날 지표다.

직접 해 보기:

```bash
cat records/devlog.csv
python3 tools/eval/devlog.py --help
```

기록을 생성할 때는 최종 XML·실행 장소·소스 SHA를 함께 보존한다. 학습 문서는 결과 표를 복제하기보다 그 결과를 어떻게 해석하는지 설명한다.
