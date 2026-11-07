# conveyor-tracker-bot

Doosan E0509 + RH-P12-RN(A) + RealSense D455로 컨베이어 위를 지나가는 병뚜껑에 공깃돌을 넣는 ROS 2 Jazzy 프로젝트. 저장소 루트가 colcon 워크스페이스(`drop_ws`)다.

- 계획·구조·판정 규칙: `PLAN.md`
- 진행·게이트·현장 세션: `Progress.md`
- 시험 명세: `TEST_SPEC.md`
- 요구사항 명세: `specs/`

## 구조

```
src/        ROS 2 패키지 10개(PLAN.md 2.3절)
specs/      SPEC 문서, TRACE.md, interface 스냅샷
records/    기록(trials, bench, sessions, param_changes.csv, devlog.csv, datasets.yaml)
tools/      eval(녹화·분석), bench(현장 측정), spec(추적 행렬), testing(공통 pytest 규칙)
```

## 빌드·시험

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
python3 tools/spec/trace.py          # REQ ↔ TC ↔ 테스트 추적 행렬
```

## 외부 의존성

| 항목 | 버전 |
|---|---|
| ROS 2 | Jazzy |
| doosan-robot2 | Jazzy 브랜치, 설치 커밋 SHA는 `records/bench/versions-*.txt`(OFF-1)에서 옮겨 적는다 |
| E0509 제어기·DRFL | `records/bench/versions-*.txt`, 티치펜던트 |
| realsense-ros | 4.58.4(librealsense2 2.58.4, apt), `records/bench/d455-check-*.txt`. 영상 수신 노드는 `FASTRTPS_DEFAULT_PROFILES_FILE=src/drop_bringup/config/fastdds_large_shm.xml` |
| RH-P12-RN(A) 펌웨어 | Modbus-RTU 사용 시 V12 이상 |
