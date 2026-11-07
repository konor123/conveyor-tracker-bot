# AGENTS.md — conveyor-tracker-bot 작업 규칙

이 저장소에서 작업하는 사람과 AI 에이전트가 따르는 규칙이다. 설계 원본은 `PLAN.md`, 진행·게이트는 `Progress.md`, 시험 명세는 `TEST_SPEC.md`다.

## 저장소

- 저장소 루트가 colcon 워크스페이스(`drop_ws`)다. 패키지는 `src/`, 명세는 `specs/`, 기록은 `records/`, 도구는 `tools/`.
- `main`에 직접 커밋한다. 작업 브랜치를 만들지 않는다. 푸시 전에 `git fetch`로 원격을 확인하고, 새 커밋이 있으면 `git rebase origin/main` 후 푸시한다.
- 작업 단계가 완료되면 검증 근거에 맞춰 `Progress.md`의 완료 여부와 남은 조건을 갱신하고, 해당 단계의 코드·시험·기록과 함께 커밋하여 GitHub에 푸시한다. 부분 수행이나 실패한 시험을 완료로 표시하지 않는다.
- `deliverables/`는 `.gitignore` 대상이다. 커밋하지 않고 `git add -f`로 강제 추가하지 않는다.
- rosbag은 저장소 밖에 둔다. 경로와 SHA-256만 `records/bags.csv`에 남긴다(`tools/eval/record_trial.sh`).
- GitHub에 공유할 영상은 ffmpeg로 압축한 RGB·깊이 미리보기와 원래 프레임 시각 CSV를 `records/media/<date>/`에 둔다. 원본 rosbag은 보존하며, 압축 영상의 크기·프레임 수·SHA-256 및 원본 대응표를 함께 기록한다.

## 문서

- `PLAN.md`·`Progress.md`·`TEST_SPEC.md`·`specs/`에는 최종 내용만 쓴다. "반영했다/변경/추가/아직/TODO" 같은 진행 보고, 조언 반영표, 사용자에게 묻는 질문 목록은 넣지 않는다. 그런 내용은 대화나 작업 보고로 전한다.
- 수치는 제안값이다. 실측으로 바꾸면 `records/param_changes.csv`에 근거(파일·N·통계값)와 함께 남긴다.
- 제안값이 바뀌면 `TEST_SPEC.md` 기대값도 같은 커밋에서 고친다.

## 개발 순서(TDD)

```
SPEC의 REQ → TEST_SPEC.md의 TC → 실패하는 테스트 → 구현 → 통과 → trace.py
```

- 테스트에 `@pytest.mark.req("REQ-…")`, `@pytest.mark.tc("TC-…")`, `@pytest.mark.level("U")`를 단다. `[ON*]` 시험은 `@pytest.mark.needs_data("<자료 이름>")`(자료 위치는 `records/datasets.yaml`).
- 각 패키지 `test/conftest.py`가 `tools/testing/drop_pytest.py`를 가져온다. 새 패키지도 같은 conftest를 둔다.
- 판단 로직은 ROS와 무관한 순수 Python 모듈로 만들고, 노드는 감싸기만 한다(`PLAN.md` 0.2절).
- 단위 변환은 `robot_control/motion_adapter.py`(mm/deg/ZYZ)와 `gripper_adapter.py`(pulse)에서만 한다.

## interface

- 원본은 `src/drop_interfaces/`. 바꾸면 SPEC-010 버전을 올리고 스냅샷을 갱신한다.

```bash
colcon build --packages-select drop_interfaces && source install/setup.bash
tools/spec/snapshot_interfaces.sh          # 갱신
tools/spec/snapshot_interfaces.sh --check  # 비교(CI)
```

## 확인 명령

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
python3 -m pytest -q tools
python3 tools/spec/trace.py
```

colcon이 없으면 시스템을 바꾸지 않고 venv에 설치해 쓴다: `python3 -m venv --system-site-packages ~/.venvs/colcon && ~/.venvs/colcon/bin/pip install colcon-common-extensions`.

## 장비·안전

- 실제 로봇·그리퍼에 지령을 보내는 일은 `[OFF]`다. 원격 접속은 온라인 작업이 아니다(`PLAN.md` 11.4절).
- 현장에서 처음 코드를 짜지 않는다. 현장에서 고친 코드는 같은 날 커밋하고 `[ON]` 시험을 다시 돌린다.
- 그리퍼는 ROBOTIS RH-P12-RN(A)이다. 하드웨어 Goal Position은 0~1150 pulse이며, 참고 구현 서비스 `/dsr01/gripper/cmd`의 0~750 반전 규약을 프로젝트 코드에 쓰지 않는다. Modbus-RTU는 펌웨어 V12 이상.
- 그리퍼 배선(E0509 플랜지 RS-485 + 제어기 DRL)은 유지한다. PC 쪽으로 옮기는 방안은 사용자 승인 후에만 검토한다.
- 그리퍼 펌웨어·통신 설정(Protocol Type, ID, 통신 속도)은 바꾸지 않는다.
