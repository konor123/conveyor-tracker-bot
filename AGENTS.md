# AGENTS.md — conveyor-tracker-bot 작업 규칙

이 저장소에서 작업하는 사람과 AI 에이전트가 따르는 규칙이다. 설계 원본은 `PLAN.md`, 진행·게이트는 `Progress.md`, 시험 명세는 `TEST_SPEC.md`다.

## 저장소

- 저장소 루트가 colcon 워크스페이스(`drop_ws`)다. 패키지는 `src/`, 명세는 `specs/`, 기록은 `records/`, 도구는 `tools/`.
- `main`에 직접 커밋한다. 작업 브랜치를 만들지 않는다. 푸시 전에 `git fetch`로 원격을 확인하고, 새 커밋이 있으면 `git rebase origin/main` 후 푸시한다.
- 작업 단계가 완료되면 검증 근거에 맞춰 `Progress.md`의 완료 여부와 남은 조건을 갱신하고, 해당 단계의 코드·시험·기록과 함께 커밋하여 GitHub에 푸시한다. 부분 수행이나 실패한 시험을 완료로 표시하지 않는다.
- `deliverables/`는 `.gitignore` 대상이다. 커밋하지 않고 `git add -f`로 강제 추가하지 않는다.
- rosbag은 저장소 밖에 둔다. 경로와 SHA-256만 `records/bags.csv`에 남긴다(`tools/eval/record_trial.sh`).
- GitHub에 공유할 영상은 ffmpeg로 압축한 RGB·깊이 미리보기와 원래 프레임 시각 CSV를 `records/media/<date>/`에 둔다. 원본 rosbag은 보존하며, 압축 영상의 크기·프레임 수·SHA-256 및 원본 대응표를 함께 기록한다.

## 작업 단계 기록·보고

작업 단계(Progress.md 항목 하나, 버그 수정 하나, 현장 측정 묶음 하나처럼 검증해서 끝낼 수 있는 단위)가 끝날 때마다 다음을 남긴 뒤 다음 단계로 넘어간다. 실패하거나 중단한 단계도 같은 방법으로 남긴다.

1. **저장소 기록**: `records/worklog/<YYYY-MM-DD>.md`에 단계마다 한 절을 덧붙인다. 항목은 시각(KST), 수행자(사람 또는 에이전트·모델), 실행 장소(GCE / PC 이름 / CI), 단계 이름과 관련 Progress.md 항목, 바꾼 파일, 검증 명령과 결과(통과·실패 개수, CI run ID), 실패·미검증 사항, 남은 일, 커밋 SHA다. 현장 세션은 `records/sessions/OFF-<n>-<date>.md`를 함께 쓴다.
2. **진행표·커밋**: 검증 근거에 맞춰 `Progress.md`를 갱신하고 코드·시험·기록과 함께 커밋해 푸시한다(위 저장소 규칙). 푸시 뒤 CI 결과를 확인해 기록에 남긴다.
3. **보고**: 사용자에게 단계 보고를 보낸다. 내용은 한 일, 검증 결과(실행 장소 포함), 실패·미검증 사항, 다음 단계다. 보고 내용은 `PLAN.md`·`Progress.md`·`TEST_SPEC.md`·`specs/`에 넣지 않는다.

4. **팀 학습 문서**: 작업한 날마다 `docs/learn/<YYYY-MM-DD>-<주제>.md`를 쓰거나 같은 날 문서를 보강하고, `docs/learn/README.md` 목록에 한 줄을 추가한다. 팀원이 처음 읽어도 따라올 수 있게 무엇을 왜 했는지, 문제를 어떻게 찾고 확인했는지(재현 방법, 측정값, 명령), 배운 개념(ROS 2·DDS·시계·시험 기법 등)과 흔한 함정, 관련 파일·커밋·기록 위치를 설명한다. worklog가 사실 기록이라면 학습 문서는 설명이다. 같은 내용을 복사하지 말고 링크한다. 비밀 값과 개인 계정 정보는 쓰지 않는다.

GCE 중앙 작업 공간에서 일하는 에이전트는 같은 내용을 `/srv/cloud-workspace/memory/projects/conveyor-tracker-bot/`에도 남긴다(사용량 확인·작업 배정 근거 포함). 인증 정보와 비밀 값은 어느 기록에도 쓰지 않는다.

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
python3 tools/spec/trace.py --gate M0 --check
python3 tools/eval/devlog.py --junit /tmp/drop-tests.xml --gate M0:진행 --open-issues 0 --robot-hours 0
```

colcon이 없으면 시스템을 바꾸지 않고 venv에 설치해 쓴다: `python3 -m venv --system-site-packages ~/.venvs/colcon && ~/.venvs/colcon/bin/pip install colcon-common-extensions`.

실행 장소: GCE 중앙 작업 공간은 자원이 작아 편집·순수 Python 단위 시험·조정에만 쓴다. ROS 빌드·통합 시험·반복 측정은 PC에서 한다. 실제 구동 장비는 LEGION(`academylegion` 런타임, 로봇·카메라·그리퍼 연결)이고, 꺼져 있으면 사무실 PC WSL(`officepc`)을 쓴다. LEGION에서는 현장 저장소 사본과 분리된 임시 폴더와 시험용 `ROS_DOMAIN_ID`로 mock 시험만 돌리고 로봇 지령 경로는 건드리지 않는다. 결과 기록에는 실행 장소를 적는다.

## 장비·안전

- 실제 로봇·그리퍼에 지령을 보내는 일은 `[OFF]`다. 원격 접속은 온라인 작업이 아니다(`PLAN.md` 11.4절).
- 현장에서 처음 코드를 짜지 않는다. 현장에서 고친 코드는 같은 날 커밋하고 `[ON]` 시험을 다시 돌린다.
- 그리퍼는 ROBOTIS RH-P12-RN(A)이다. 하드웨어 Goal Position은 0~1150 pulse이며, 참고 구현 서비스 `/dsr01/gripper/cmd`의 0~750 반전 규약을 프로젝트 코드에 쓰지 않는다. Modbus-RTU는 펌웨어 V12 이상.
- 그리퍼 배선(E0509 플랜지 RS-485 + 제어기 DRL)은 유지한다. PC 쪽으로 옮기는 방안은 사용자 승인 후에만 검토한다.
- 그리퍼 펌웨어·통신 설정(Protocol Type, ID, 통신 속도)은 바꾸지 않는다.
