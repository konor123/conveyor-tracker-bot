# 남은 결정 여섯 건 (2026-11-14)

- 사용자 지시(11-14 01:2x KST): 에이전트들이 논의해 결정하고 모두 진행한다.
- 의견: Codex(Orca worker, 읽기 전용, 결정당 한 줄), OpenCode(`oamp/auto`, 읽기 전용, 저장소 근거·영향 검색), advisor(종합). 결정·구현은 coordinator(Claude Opus 5.5).
- 오늘의 제약: 실기에서 검증한 정적 투입 시연 경로(`static_pick_place`, `e509c87`)는 바꾸지 않는다. 실기 지령은 `[OFF]`(현장, 사용자 승인). 바꾼 수치는 `records/param_changes.csv`에 근거와 함께 남긴다.

| 결정 | 처음 안 | Codex | OpenCode | 결정 |
|---|---|---|---|---|
| D1 hover 도구 | 거절만 하는 가드를 넣어 반영 | 보류: 같은 줄의 뚜껑 둘은 횡방향 범위로 하나를 고를 수 없고, park 위치 허용오차만으로는 자세·경로·설정 위험이 닫히지 않는다 | 찬성 | **반영하지 않음**. 실제 팔을 움직이고 여기서 실행해 볼 수 없는 변경이라 보수적인 의견을 따른다. 오늘 가드를 넣으면 고친 도구의 첫 실행이 현장이 된다. 다시 쓰는 방법과 조건은 `docs/operations/site-checklist-2026-11-14.md` 5절 그대로 |
| D2 RELEASE 시간 | orchestrator RELEASE 1.0 → 1.5 s, controller 게이트 대기 2.0 → 1.0 s | 잠정값으로 받아들이되 경계 시험과 기록, 사유 전달·1.4 s 상한을 보장한다고 쓰지 말 것 | 구조 찬성, 값은 REQ-ABT-007 `max(ceil(1.5 × p99), p95 + 0.5)`로 2.0 s | **RELEASE 2.0 s, 게이트 대기 1.0 s**(아래 1절) |
| D3 DDS·기동 대기 | DDS 그대로, 운용 기동 대기 2 → 5 s, I0 기동 안정 대기 1.5 → 5.0 s | 찬성, 대기만으로 준비됐다고 보지 말고 heartbeat 확인·해제 절차 유지 | 찬성 | **진행**. I0 안정 대기 변경은 사무실 PC I0가 통과할 때만 넣는다 |
| D4 종료 처리 확장 | `vision_node`·calibration 진입점에 `ShutdownGuard`, `static_pick_place` 제외 | 찬성, SIGINT·SIGTERM·반복 종료 확인 | 찬성 | **진행**(별도 커밋). 일회성 CLI `check_camera_info`는 제외 |
| D5 TC-MCK-012-02 | `realtime_pc` 유지 | 찬성, PC 시험은 계속 돌리고 보고 | 찬성 | **유지**. CI에서만 멈추는 원인은 미해결로 둔다 |
| D6 `SelectTarget` 신선도 | 바꾸지 않음 | 오늘은 찬성. stamp가 계속 앞으로 가는 오래된 backlog는 stamp 조건으로 막지 못한다 | 찬성 | **바꾸지 않음**. 남은 사례(아래 3절)를 기록한다 |

## 1. D2 값

- 규칙: REQ-ABT-007(`specs/SPEC-110-abort-timeout.md`) `max(ceil(1.5 × p99), p95 + 0.5)` s.
- 입력: 실제 controller I0(fake transport) 정상 사이클 RELEASE 0.88~0.91 s(`records/reviews/release-step-timeout-2026-11-13.md` 4절, 4건) → `ceil(1.5 × 0.91)` = 2, 0.91 + 0.5 = 1.41 → **2.0 s**.
- 설계상 최악 경우와 맞춰 봄(`DynamicPlaceCfg` 기본값):
  - PREDICTED: 하강 끝에서 commit(해제 0.1 s 앞)까지 최대 `entry_lead_s` 1.3 − 0.1 = 1.2 s, 이어서 개방 0.1 s·확인 유지 0.3 s → 1.6 s. 1.5 s로는 하강이 빠를 때 넘고 2.0 s는 덮는다.
  - SYNCHRONIZED: 게이트 대기 상한을 1.0 s로 내리면 1.0 + 0.1 + 0.3 = 1.4 s.
  - 남는 경우: 개방을 확인하지 못하면 확인 유지 대신 `release_timeout_s` 1.0 s를 기다려 최대 1.0 + 0.1 + 1.0 = 2.1 s로 2.0 s를 넘는다.
- 결과 분류는 바꾸지 않는다. controller 결과가 RELEASE 끝 feedback보다 먼저 와도 orchestrator 판정은 `STEP_TIMEOUT`이고 사유는 로그(REQ-ORC-020)에만 남는다. 스케줄·feedback 지연이 더해지므로 위 길이는 상한 보장이 아니다.
- 값은 fake transport 기준 잠정값이며 G0 실측 뒤 다시 정한다. 두 값의 관계(orchestrator RELEASE ≥ 정상·게이트 대기 경우의 RELEASE 길이)는 설정 일관성 시험으로 고정한다.

## 2. D3 근거

정지 관찰 시점은 기동 2.5~3.9 s 뒤(`records/reviews/publish-stall-repro-2026-11-13.md`, `records/reviews/heartbeat-stall-evidence-2026-11-13.md`), CI 실패는 3.0~3.25 s 뒤 Stop이었다. 5 s 대기는 관찰 범위를 덮는다. 대기 뒤 controller heartbeat 확인과 정지 유지 해제 절차는 그대로 한다. DDS 설정은 미세 재현이 I0·CI 설정(`LOCALHOST`)에서 0/5였고 비동기·UDP 전용에 비용이 있어 바꾸지 않는다.

## 3. D6 남은 사례

stamp가 앞으로 갈 때만 입력으로 세는 규칙은 stamp가 멈춘 흐름과 다른 좌표계 흐름을 막는다. 그러나 오래 밀린 검출(backlog)이 stamp 순서대로 계속 들어오면 입력으로 세여, track은 늙지 않은 채 현재 시각보다 뒤처질 수 있다. `SelectTarget`은 현재 시각 신선도를 보지 않으므로, 이 경우는 planner의 관측 신선도·시간 예산 판정에 맡긴다.
