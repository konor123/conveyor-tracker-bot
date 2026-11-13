# 확인되지 않는 Stop의 재시도 간격 결정 (2026-11-13)

- 자문: Claude 상위 검토 모델(coordinator 세션의 advisor 도구, 대화 기록과 코드 읽기에 근거, 실행 없음). 판단: coordinator(Claude Opus 5.5). 사용자가 남은 결정을 coordinator에 위임했다(2026-11-12 22:50 KST).
- 관련: SPEC-110 8절(Stop 전달 확인과 재시도), SPEC-500 REQ-ROB-014(정지 유지), `orchestrator/safety_supervisor_node.py`.

## 문제

실기 연결이 없는 기본 bringup(`use_mock_robot:=false`, `robot_backend` 기본 `disabled`)에서 오케스트레이터를 켜면 다음 순서로 Stop이 끝없이 반복된다.

1. `robot_controller_node`가 `robot/status`에 `connected=false`를 낸다.
2. 감시 노드가 `ROBOT_COMM_LOST`(L4)를 검출하고 `robot/stop`을 부른다.
3. controller는 정지 유지를 걸지만, 정지를 확인할 backend가 없어 `stopped=false`로 응답한다.
4. 감시 노드는 `stopped=true`가 올 때까지 0.1 s 뒤 다시 보낸다.

사무실 PC WSL에서 12 s 동안 Stop 60회와 `Stop rejected` 경고 61줄을 확인했다(약 5회/s). controller의 `stop_history_json`(최근 100건)도 약 20 s 만에 같은 항목으로 찬다.

## 선택지와 판단

| 선택지 | 판단 | 이유 |
|---|---|---|
| disabled backend의 Stop을 `stopped=true`로 응답 | 채택하지 않음 | `stopped`는 정지를 확인했다는 뜻이다(SPEC-010, REQ-ROB-007, `SkillExecutor.stop`). 로봇을 볼 수 없는 backend가 확인을 지어내게 된다 |
| 감시 노드가 disabled backend를 따로 처리 | 채택하지 않음 | `RobotStatus`에는 backend 종류가 없어 interface(SPEC-010)를 바꿔야 한다. `connected=false`에 대한 L4 자체는 맞는 반응이다 |
| `stopped=false`가 이어지면 재시도 간격을 늘림 | **채택** | SPEC-110의 재시도 간격은 "0.1 s 이상"이라 계약 안이다. 아래 근거로 안전 효과가 줄지 않는다 |
| controller Stop 이력에서 같은 요청자·사유의 연속 항목 합치기 | 채택하지 않음 | 시험이 수신 시각으로 Stop을 거르는데, 합치면 이후 Stop이 앞 항목에 묻힐 수 있다. 간격을 늘리면 100건은 약 100 s 분량이고, 사유가 바뀔 때의 로그(`STOP_LATCHED`)는 처음 한 번 남는다 |

간격을 늘려도 안전 효과가 줄지 않는 근거:

- controller는 Stop을 받는 즉시 정지 유지를 건다(REQ-ROB-014). `stopped=false` 응답은 Stop이 전달됐다는 뜻이고, 그 뒤의 재시도는 정지 확인을 다시 묻는 것일 뿐이다.
- 서비스 부재와 호출 예외는 전달되지 않았을 수 있으므로 0.1 s 간격을 그대로 둔다.
- 장애 사유가 바뀌면(예: `ROBOT_COMM_LOST` → `PROTECTIVE_STOP`) 늘어난 간격을 기다리지 않고 바로 보낸다.

## 결정

- `stopped=false` 응답이 이어지면 다음 재시도까지의 간격을 0.1 s에서 두 배씩 늘리고 1 s를 상한으로 한다(0.1, 0.2, 0.4, 0.8, 1.0 s, …). `stopped=true`, 장애 해소, 사유 변경 때 처음으로 돌아간다.
- 경고는 첫 거부 때와 그 뒤 5 s마다 사유·연속 횟수·다음 간격을 붙여 남긴다.
- 진행 중 Stop RPC 최대 1개, 응답 없는 RPC 1 s 경고, Stop만 호출하는 단일 지령 권한은 그대로 둔다.
- 이 변경은 실제 controller 통합(작업자 5 진행분)과 다른 커밋으로 넣었다. 감시 노드 Stop이 얽힌 CI 간헐 실패를 진단 중이어서(`9309618`) 원인을 섞지 않기 위해서다.

## 확인

- 단위 시험 `test_supervisor_backs_off_while_stop_stays_unconfirmed`(TC-ABT-009-03): 0.05 s 감시 주기에서 12 s 동안 거부가 이어질 때 재시도 간격이 늘어나기만 하고 1.05 s(상한 + 1주기)에서 멈춘다. 재시도는 16회 이하(고정 0.1 s면 약 80회), 경고는 3회다. 사유를 바꾸면 그 주기에 바로 보낸다. 수정 전 코드에서는 실패한다.
- 기존 TC-ABT-009-03 시험(지연·거부·예외 뒤 0.1 s 재시도, in-flight 1개)은 그대로 통과한다.
- 사무실 PC WSL: 기본 bringup(`use_mock_robot:=false`, `robot_backend` disabled) + 오케스트레이터에서 감시 노드의 Stop 거부는 15.7 s 동안 18회, 경고는 4줄이었다(수정 전 12 s에 60회·61줄). 같은 실행에서 orchestrator 791 passed, 실제 controller I0 8 passed.
