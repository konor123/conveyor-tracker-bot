# 그리퍼 개폐 인계 시험 — 2026-11-07

- 실행: 이 호스트 academylegion, Codex. 현장 비상정지·관찰: 사용자(대화에서 준비 확인).
- 드라이버 기존 1개 재사용. 로봇 팔 지령 없음. 상주 DRL 없음; 일회성 DRL 사용.
- EEPROM·통신 설정·펌웨어·Operating Mode 변경 없음. 기존 Torque Enable=1 유지, Goal Current=200 mA 설정.
- 시작 조회: robot_state=1, drl_state=1, 마지막 alarm level=1/group=1/index=2007. 그리퍼 Mode=5, Hardware Error=0, Present Position=0.
- 기준: TEST_SPEC TC-BEN-003-01의 pulse 방향 확인에 해당하는 예비 시험. 간격 실측·전체 범위·지연 분포 시험 아님.

| 시험 | 지령 pulse | 실제 위치 pulse | 읽은 전류 | 현장 관찰 |
|---|---|---|---|---|
| 1 | 0 → 100 → 0 | 100 → 0 | 8 → 8 mA | 사용자: 움직임 없음. 물리적 개폐 성공 판정 안 함 |
| 2 | 0 → 500 → 0 | 500 → 0 | 8 → 7 mA | 사용자가 더 큰 움직임 시험 지시 후 실제 그리퍼 움직임 확인 |

각 목표 도달 확인 뒤 2초 결과 페이지 유지. 두 시험 모두 DRL 완료 마커 250, postflight robot_state=1, alarm level=1/index=2007.
명령·원시 결과: `records/bench/gripper-takeover-2026-11-07/step-{100,500}-open.{drl,jsonl}`.
DRL 구문 및 TEST_SPEC CRC 표준 벡터 3개를 실행 전 확인. 응답 CRC·주소·FC·길이를 검사하고, 모델·모드·토크·오류·시작 위치를 확인한 뒤 동작.
기존 도구 문제: gripper_cli.py는 자신을 exec하는 셸 스크립트, gripper_cmd.py는 잘못된 서비스 경로와 금지된 750 반전 fallback, gripper_direct.py는 상주 루프를 가정. 해당 파일들은 타 세션 미커밋 파일이라 이번 시험에서 수정하지 않음.

## 터미널 명령

`~/.local/bin/gripper` → `tools/bench/gripper_cli.py`의 자기 호출을 제거하고 `gripper_once.py`와 `drl/gripper_once.drl`을 사용한다. `gripper open` 실기 실행으로 Present Position=0, DRL 완료, STANDBY를 확인했다.
사용자 지정 현장 닫힘 끝점은 750 pulse이다. 터미널 CLI 기본 close=750, 목표 허용 범위 0~750, 전류 상한 200 mA. 750 끝점은 사용자 지정값이며 이번 세션에서 직접 구동·간격 실측한 값은 아니다. 제조사 전체 주소표의 범위와 현장 어댑터의 운용 끝점을 구분한다.

검증: CLI 오프라인 시험 7개 통과. GRP trace 명세 밖 ID 0, 미구현 TC 15개로 종료 코드 1. 전체 G0 통과 판정 없음.

## 전류 100 mA 지정

사용자 지시: "전부 100mA로 줄인다." 개폐·위치·측정 도구(`gripper_once.py`, `gripper_cmd.py`, `gripper_direct.py`, `gripper_pulse_table.py`)의 기본값 및 허용 상한은 100 mA이다. 기존 실행 기록의 200 mA 값은 당시 값으로 보존한다. SPEC-500 및 TC-GRP-004-01 기대값도 100 mA이다.
검증: 관련 오프라인 테스트 25개 통과. GRP trace는 명세 밖 ID 0, 기존 미구현 TC 15개로 종료 코드 1.
초기 회귀 시험 중 구형 direct 함수가 101 mA를 검증 없이 PLC 레지스터로 전송했다. 이 경로에 전송 전 검증을 추가했으며, 회귀 시험의 소켓을 차단해 오프라인 시험에 네트워크 접근이 없도록 했다. 이 전송만으로 실제 그리퍼 Goal Current가 바뀌었다는 증거는 없다.
실제 그리퍼 Goal Current=100만 쓰고 읽는 DRL은 `records/bench/gripper-takeover-2026-11-07/set-current-100.drl`이다. 첫 실행은 robot_state=1, alarm level=1/index=2007, drl_state=0(실행 중)에서 기동 전에 거부되어 장비 설정 변경 확인은 보류했다. 다른 DRL을 중단하거나 위치 지령을 보내지 않았다.
