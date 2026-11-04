# 미니프로젝트 1 — Moving Frame TF 기반 단추 적층 프로젝트

## 1. E0509 컨트롤러와 ROS 2 연결

- Ubuntu PC와 E0509 제어기 TCP 통신 성공
- doosan-robot2 Jazzy 빌드 성공
- `model:=e0509 / mode:=real` 연결 성공
- 실제 관절 Joint State 수신
- RViz2와 실제 관절 상태 동기화
- ROS 2에서 저속 단일 관절 이동 성공
- Cartesian Motion 및 연속 제어 인터페이스 확인
- 2지 그리퍼 개폐 성공

## 2. Isaac Sim 시뮬레이션 환경 구축

- Isaac Sim 5.1에 E0509 모델 Import
- Articulation Root 및 Joint Control 설정
- 2지 그리퍼 모델 구성 및 개폐 제어
- 직경 10 mm 단추 Rigid Body 모델링
- 검은색 매트 및 교육용 미니 컨베이어 구현
- 컨베이어 이동 및 단추 접촉 물리 구현
- 고정형 RGB-D 가상 카메라 설치
- Isaac Sim ROS 2 Bridge 연결
- ROS 2에서 가상 로봇 관절 제어 성공

## 3. RealSense D455 연결 및 캘리브레이션

- D455 ROS 2 연결 및 RGB·Depth 영상 수신
- D455 CameraInfo 확인
- 카메라 왜곡 보정
- 기준점 또는 Calibration Target 배치
- 카메라 좌표와 로봇 작업 좌표의 대응점 취득
- Extrinsic Transform 계산
- 바닥과 벨트의 평면 위치·높이 측정
- 실제 좌표 변환 오차 평가

## 4. 단추 인식 및 파지

- 검은색 매트 위 직경 10 mm 흰색 단추 검출
- OpenCV 기반 단추 중심 좌표 계산
- 픽셀 좌표를 Robot Base Frame으로 변환
- 단추 Grasp Pose 계산
- 2지 그리퍼로 단추 파지 및 Lift
- 필요시 교체형 핑거 어댑터 적용
- Isaac Sim에서 파지 동작 검증
- 실제 E0509에서 단추 파지 성공
- 단추 파지 성공률 및 위치 오차 측정

## 5. 정적 Pick &amp; Place 및 적층

- 정지한 컨베이어 위에 첫 번째 단추 배치
- 바닥에서 두 번째 단추 인식 및 파지
- 첫 번째 단추 위 적층 목표 좌표 계산
- 두 번째 단추 하강 및 그리퍼 해제
- Isaac Sim에서 정적 적층 검증
- 실제 E0509에서 정지 상태 2단 적층 성공
- 반복 작업 성공률 측정

## 6. 컨베이어 위 이동 단추 추적

- 녹색 벨트 위 첫 번째 흰색 단추 검출
- 영상 프레임별 단추 중심 좌표 측정
- 연속 위치 변화로 이동 속도 계산
- Linear Regression 또는 Kalman Filter 적용
- 수동 다이얼 조작에 따른 속도 변화 감지
- Isaac Sim Ground Truth와 추정 결과 비교
- 실제 벨트에서 위치·속도 추정 오차 측정
- 영상 처리 지연 및 추적 손실 조건 확인

## 7. Moving Reference Frame 구현

- `world / robot_base / camera / conveyor` TF 구성
- 첫 번째 단추의 `button_1_frame` 생성
- Vision 기반 Dynamic TF 갱신
- TF Timestamp 동기화
- Moving Frame을 Robot Base Frame으로 변환
- RViz2에서 이동 좌표계 시각화
- Isaac Sim Ground Truth 기반 TF 정확도 검증
- 실제 단추 위치와 TF 위치 비교 검증

## 8. 이동 목표 위치 예측

- 카메라 영상 취득 및 처리 지연 측정
- ROS 2 통신 및 로봇 제어 지연 측정
- 첫 번째 단추의 미래 위치 예측
- 로봇 이동 시간과 남은 작업 가능 시간 계산
- 벨트 속도 변화에 따른 예측값 갱신
- Isaac Sim에서 예측 알고리즘 검증
- 실제 벨트에서 예측 위치 오차 평가

## 9. Moving Frame 기반 로봇 추종 제어

- E0509 연속 위치·속도 제어 인터페이스 확정
- 첫 번째 단추 기준 TCP 목표 자세 계산
- 움직이는 목표 위치를 로봇이 실시간 추종
- TCP와 단추의 상대 속도 동기화
- Position Feedback 및 Velocity Feedforward 적용
- Isaac Sim에서 추종 제어 검증
- 실제 E0509에서 단추 추종 성공
- 벨트 속도 변화에 대한 추종 성능 확인
- TCP 위치·속도 추종 오차 측정

## 10. 이동 중 단추 적층

- 첫 번째 단추를 움직이는 벨트 위에 배치
- 첫 번째 단추 추적을 유지하며 두 번째 단추 파지
- 첫 번째 단추의 남은 작업 가능 시간 확인
- 두 번째 단추의 적층 목표 Pose 계산
- TCP를 첫 번째 단추의 이동 속도에 동기화
- 첫 번째 단추 위로 두 번째 단추 하강
- 그리퍼 해제 및 충돌 없는 후퇴
- Isaac Sim에서 동적 적층 검증
- 실제 E0509에서 동적 적층 성공
- 적층 후 두 단추의 동반 이동 확인
- 적층 중심 오차 및 성공률 측정

## 11. 전체 시스템 통합

- ROS 2 노드 및 Launch 파일 통합
- 작업 상태 관리(State Machine) 구현
- 첫 번째 단추 배치부터 두 번째 단추 적층까지 자동화
- 단추 검출·파지 실패 처리
- 이동 단추 추적 손실 및 속도 급변 대응
- 작업 가능 시간 초과 및 도달 불가능 목표 처리
- Isaac Sim에서 전체 자동화 검증
- 실제 장비에서 반복 작업 및 자동 복귀 성공

## 12. 성능 평가 및 시연

- 정지 벨트에서 적층 성공률 측정
- 벨트 속도별 적층 성공률 비교
- Vision 위치·속도 추정 오차 측정
- Moving Frame TF 정확도 평가
- TCP 추종 위치·속도 오차 측정
- Static Target / Predicted Target / Moving Frame 제어 비교
- 작업 Cycle Time 및 안정성 평가
- 전체 프로젝트 시연 영상 제작

