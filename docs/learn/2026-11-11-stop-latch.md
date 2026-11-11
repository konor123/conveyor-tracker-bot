# 멈춘 로봇은 사람이 풀 때까지 멈춰 있어야 한다: Stop 정지 유지 (2026-11-11)

## 문제

감시 노드(safety_supervisor)는 보호정지·통신 단절·controller heartbeat 손실을 보면 `robot/stop`을 부른다. 지금까지 이 Stop은 **실행 중인 skill**만 끝냈다. 검출을 기다리거나(WAIT_CAP) 단계와 단계 사이처럼 skill이 없을 때 Stop이 오면 로봇은 아무 표시도 남기지 않았고, orchestrator는 몰랐다. 다음 단계의 이동 지령도 그대로 받았다(M0 재검수 R1).

## 해결: 로봇이 상태를 들고 있게 한다

- controller가 Stop을 받으면 **정지 유지**(latch)한다. 유지 중에는 `RobotStatus.ready=false`, heartbeat `error_code=101`이다. 보호정지(100)가 함께 있으면 100이 우선이다.
- 유지 중에 들어온 goal은 받아들이되 로봇에 아무 지령도 보내지 않고 `CANCELED`(`STOPPED`)로 끝낸다. 거부하지 않는 이유: orchestrator가 거부를 "단계 실패"로 보고 안전 자세 이동(L2)을 다시 시도하기 때문이다. `STOPPED`로 끝내면 기존 REQ-ORC-011 경로로 곧바로 L4가 된다.
- orchestrator는 heartbeat의 `error_code`를 감시 주기마다 본다. 101이면 어느 상태든 L4로 끝낸다.
- 해제는 사람만 한다: `ros2 param set /robot_controller_node stop_release_reason "<사유>"` → orchestrator `resume_reason`. 순서가 바뀌면 다음 사이클이 PRECHECK에서 다시 L4다.

원본: `specs/SPEC-010-interfaces.md` 4.5절, 코드 `src/robot_control/robot_control/stop_latch.py`.

## 흔한 함정(이번에 실제로 밟은 것)

- **rclpy는 파라미터를 선언할 때도 설정 callback을 부른다.** 대역 노드는 callback이 읽는 `_lock`을 선언 뒤에 만들어 기동 자체가 실패했다. callback이 읽는 상태는 `declare_parameter`보다 먼저 만든다.
- **수락과 실행 사이의 틈.** ROS action은 `goal_callback`(수락)과 `execute_callback`(실행)이 따로다. 수락 때 "실행 중"을 예약하지 않으면 그 틈에 작업자 해제가 끼어들어, 멈춰 있을 때 받은 goal이 해제 뒤 움직인다(독립 검수 P1).
- **"아무것도 안 함"은 정말 아무것도 하지 않아야 한다.** 유지 중이 아닐 때의 해제가 backend 정지까지 풀고 있었다(P2).
- **상태를 남기면 숨어 있던 문제가 드러난다.** 감시 노드가 기동 직후 controller heartbeat를 잠깐 늦게 받아 보내던 가짜 Stop은 예전부터 있었지만 흔적이 없었다. 정지 유지 뒤로는 다음 시험을 L4로 만들었다. 원인을 가리는 법: 같은 실행에서 (1) 다른 구독자가 받은 heartbeat stamp 공백, (2) 감시 노드 판정 순간의 타이머 간격과 마지막 수신 stamp를 같이 찍는다. 보내는 쪽은 연속이고 감시 노드 타이머도 정상인데 마지막 수신만 오래됐다면 수신 쪽 지연이다.
- **WSL은 SSH 세션이 끝나면 백그라운드 프로세스를 정리한다.** 긴 시험은 제한시간 안의 단계로 나눠 돌린다.

관련: [worklog](../../records/worklog/2026-11-11.md), `Progress.md` 1절 노드 항목, 검수 보고서(`records/reviews/r1-review-2026-11-11.md`).
