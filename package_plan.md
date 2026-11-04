# ROS 2 패키지 구조 — Button Stacking Project

개발 환경: Ubuntu 24.04 Native / ROS 2 Jazzy / Isaac Sim 5.1  
로봇: Doosan E0509 + 2-Finger Gripper  
비전: Intel RealSense D455 (작업대 고정)  
작업: 검은색 매트 위 10mm 흰색 단추 파지 → 녹색 컨베이어벨트 위 배치 → 이동 속도 측정 → 두 번째 단추 동적 적층

## 1. 전체 디렉터리 구조

```
button_stacking_ws/
│
├── src/
│   │
│   ├── button_stacking_interfaces/
│   │   ├── msg/
│   │   │   ├── ButtonDetection.msg
│   │   │   ├── ButtonDetectionArray.msg
│   │   │   ├── TrackedButton.msg
│   │   │   ├── BeltMotion.msg
│   │   │   └── InterceptPlan.msg
│   │   ├── action/
│   │   │   └── StackingTask.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── button_vision/
│   │   ├── button_vision/
│   │   │   ├── __init__.py
│   │   │   ├── detector_node.py
│   │   │   ├── localizer_node.py
│   │   │   └── image_processing.py
│   │   ├── config/
│   │   │   └── vision.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_tracking/
│   │   ├── button_tracking/
│   │   │   ├── __init__.py
│   │   │   ├── tracker_node.py
│   │   │   ├── speed_estimator.py
│   │   │   └── motion_predictor.py
│   │   ├── config/
│   │   │   └── tracking.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_stacking_planner/
│   │   ├── button_stacking_planner/
│   │   │   ├── __init__.py
│   │   │   ├── intercept_planner_node.py
│   │   │   ├── trajectory_generator.py
│   │   │   └── reachability_checker.py
│   │   ├── config/
│   │   │   └── planner.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_robot_control/
│   │   ├── button_robot_control/
│   │   │   ├── __init__.py
│   │   │   ├── robot_controller_node.py
│   │   │   ├── gripper_controller.py
│   │   │   ├── pick_sequence.py
│   │   │   └── dynamic_place.py
│   │   ├── config/
│   │   │   └── robot_control.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_task_manager/
│   │   ├── button_task_manager/
│   │   │   ├── __init__.py
│   │   │   ├── task_manager_node.py
│   │   │   ├── state_machine.py
│   │   │   └── recovery.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_calibration/
│   │   ├── button_calibration/
│   │   │   ├── __init__.py
│   │   │   ├── camera_calibration.py
│   │   │   ├── plane_calibration.py
│   │   │   └── hand_eye_calibration.py
│   │   ├── config/
│   │   │   ├── camera_extrinsics.yaml
│   │   │   ├── mat_homography.yaml
│   │   │   └── belt_homography.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── button_stacking_bringup/
│   │   ├── launch/
│   │   │   ├── vision.launch.py
│   │   │   ├── robot.launch.py
│   │   │   ├── stacking.launch.py
│   │   │   └── simulation.launch.py
│   │   ├── config/
│   │   │   └── system.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── button_stacking_sim/
│       ├── usd/
│       │   └── button_stacking_scene.usd
│       ├── scripts/
│       │   └── conveyor_simulation.py
│       ├── config/
│       │   └── simulation.yaml
│       ├── CMakeLists.txt
│       └── package.xml
│
├── tests/
│   ├── test_detection.py
│   ├── test_tracking.py
│   ├── test_speed_estimation.py
│   └── test_intercept.py
│
└── README.md
```

총 9개 ROS 2 패키지로 구성합니다. RealSense ROS 2 Wrapper 및 Doosan ROS 2 드라이버는 별도 의존성으로 설치합니다.

## 2. 패키지별 기능


| 패키지                          | 핵심 역할               | 주요 노드                             |
| ---------------------------- | ------------------- | --------------------------------- |
| `button_stacking_interfaces` | 사용자 정의 메시지·액션       | —                                 |
| `button_vision`              | 단추 검출 및 로봇 좌표 변환    | `detector_node`, `localizer_node` |
| `button_tracking`            | 단추 추적 및 속도 추정       | `tracker_node`                    |
| `button_stacking_planner`    | 미래 위치 예측, 적층 궤적 계획  | `intercept_planner_node`          |
| `button_robot_control`       | E0509 및 2지 그리퍼 제어   | `robot_controller_node`           |
| `button_task_manager`        | 작업 상태 관리, 실패 복구     | `task_manager_node`               |
| `button_calibration`         | 카메라·작업평면·로봇 좌표 보정   | 보정 도구                             |
| `button_stacking_bringup`    | 전체 노드 실행 및 설정       | Launch                            |
| `button_stacking_sim`        | Isaac Sim 5.1 시뮬레이션 | 시뮬레이션 스크립트                        |


## 3. 주요 ROS 2 데이터 흐름

RealSense D455

RGB Image / Camera Info

button_vision

검출 → 위치 변환

매트 영역

정적 파지 좌표

벨트 영역

이동 단추 관측

button_tracking

위치·속도 추정

button_stacking_planner

미래 접촉 위치·시점 및 궤적 계획

button_robot_control

Doosan E0509 + Gripper

button_task_manager가 전체 작업 시퀀스를 관리하며, 매트의 정적 파지 좌표도 로봇 제어에 전달

## 4. 주요 메시지 및 토픽


| 토픽                         | 메시지 형식                 | 내용             |
| -------------------------- | ---------------------- | -------------- |
| `/buttons/detections`      | `ButtonDetectionArray` | 매트·벨트 단추 검출 결과 |
| `/buttons/positions`       | `PoseArray` 또는 Custom  | 단추의 3D 작업 좌표   |
| `/buttons/tracked`         | `TrackedButton`        | 첫 번째 단추의 추적 상태 |
| `/belt/measured_motion`    | `BeltMotion`           | 영상 기반 속도 추정    |
| `/stacking/intercept_plan` | `InterceptPlan`        | 적층 위치·시간·속도    |
| `/task/state`              | `std_msgs/String`      | 현재 작업 상태       |


로봇에는 별도의 드라이버 인터페이스를 사용하며, 그리퍼 구동 인터페이스는 실제 장착 모델에 맞춰 연결합니다.

컨베이어벨트는 수동 다이얼 조절식이므로 ROS 제어 패키지 및 속도 명령 토픽을 포함하지 않습니다.

## 5. 최종 작업 상태(State Machine)

```
IDLE
  │
  ▼
DETECT_FIRST_BUTTON
  │
  ▼
PICK_FIRST_BUTTON
  │
  ▼
PLACE_FIRST_ON_BELT
  │
  ▼
TRACK_FIRST_BUTTON
  │
  ├───────────────┐
  │               │
  ▼               │
DETECT_SECOND     │ 위치·속도
  │               │ 지속 갱신
  ▼               │
PICK_SECOND       │
  │               │
  ▼               │
PLAN_INTERCEPT ◄──┘
  │
  ▼
SYNCHRONIZE_VELOCITY
  │
  ▼
DYNAMIC_STACK
  │
  ▼
VERIFY_STACKING
  │
  ├── SUCCESS → DONE
  │
  └── FAILURE → RECOVERY
```

적층이 진행되는 동안에도 `button_tracking`은 관측 가능한 범위에서 위치와 속도 추정을 계속 수행합니다. `button_stacking_planner`는 최신 추정값과 로봇 상태를 반영해 목표를 갱신합니다.

## 6. 개발 단계별 패키지 적용


| 단계  | 개발 목표            | 우선 구현 패키지                                  |
| --- | ---------------- | ------------------------------------------ |
| 1   | D455 영상 수신·단추 검출 | `button_vision`, `button_stacking_bringup` |
| 2   | 카메라-로봇 좌표 변환     | `button_calibration`                       |
| 3   | E0509 정적 파지 및 배치 | `button_robot_control`                     |
| 4   | 이동 단추 추적·속도 추정   | `button_tracking`                          |
| 5   | 미래 위치 예측 및 동적 적층 | `button_stacking_planner`                  |
| 6   | 전체 시퀀스 통합·실패 복구  | `button_task_manager`                      |
| 병행  | 가상 환경 테스트        | `button_stacking_sim`                      |


`button_stacking_interfaces`는 메시지 통신이 필요한 시점부터 구현하고, 처음부터 전체 패키지를 모두 완성할 필요는 없습니다.

핵심 구성은 `Vision → Tracking → Planning → Robot Control`이며, Task Manager가 전체 순서를 관리하는 구조입니다.