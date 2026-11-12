# 연속 제어 인터페이스 후보 조사 (Doosan E0509, doosan-robot2 Jazzy)

- 작성: OpenCode(oamp/auto) 초안, coordinator(Claude Opus 5.5) 검수(2026-11-12). 저장소 수치는 `records/bench/`·`records/sessions/` 원자료와 대조했다. doosan-robot2 인용은 공개 저장소 jazzy 브랜치 원문 기준이며 설치 커밋과의 일치는 4절 7번에서 확인한다.

대상: `Progress.md` 2절 항목 "후보 조사·문서화: `servol_stream` / `speedl_stream`(일반), `servol_rt_stream` / `speedl_rt_stream`(RT, `realtime/connect_rt_control` 등 초기화 필요). 단위(mm, deg)·자세 규약·`time` 인수 의미·timeout 동작".

조사 원문: doosan-robot2 공개 저장소 jazzy 브랜치(2026-11-12 웹 원문 기준). 현장 설치 드라이버는 jazzy 커밋 `dade65d`·DRCF `GF02100300`·DRFL `GL013304`(`records/bench/versions-2026-11-07-170841.txt`, `records/sessions/OFF-2-2026-11-07.md`). 웹 원문과 설치 커밋의 필드 일치 여부는 별도 확인 대상(확인 필요). 저장소 내 계약 위치는 `PLAN.md` 3.3절(연속 제어 토픽 행)과 `specs/SPEC-010-interfaces.md` 227행(TBD)이다.

## 1. 후보 목록

### 1.1 표

| 이름 | 토픽·메시지 형식 | 필드(원문 그대로)·단위·좌표계 | `time` 인수 의미 | 제어기 쪽 timeout·정지 동작 | RT 초기화 |
|---|---|---|---|---|---|
| `servol_stream` | `<ns>/servol_stream`, `dsr_msgs2/msg/ServolStream` | `float64[6] pos` / `float64[2] vel` / `float64[2] acc` / `float64 time` (`dsr_msgs2/msg/ServolStream.msg`). 단위: repo에 단위 문서 없음(확인 필요: 제어기 규약 mm·deg). 이 프로젝트는 mm/deg로 송신해 동작 확인(`tools/bench/stream_bench.py` m→mm 변환, FK−posx 차 0.00 mm: `records/bench/loaded-stream-run-2026-11-07.json`). `pos`는 `ref=0`(DR_BASE) 작업공간 6요소(xyz+ZYZ 각도), `vel`·`acc`는 [선속도/가속도, 회전 속도/가속도] 2요소. 자세 규약 ZYZ `R=Rz(a)Ry(b)Rz(c)`(`src/robot_control/robot_control/motion_adapter.py`) | 도달 목표 시간(초). repo에 비-RT servol의 `time` 문서 없음(확인 필요). 드라이버는 검증 없이 `Drfl->servol(pos, vel, acc, time)`로 전달(`dsr_controller2/src/dsr_controller2.cpp` servol 콜백). 실측: `time=0`에서 제어기 경고 "[ServoL] Can't keep target time due to limit velocity or acceleration"(시간 제약이 속도·가속도 한계와 상호작용, `records/sessions/OFF-2-2026-11-07.md`) | stale 지령 시 timeout 문서 없음(문서에 없음). 드라이버 쪽 watchdog 없음(콜백은 전달만). stale 정지 동작은 미실시(확인 필요, 4절). 정지는 `/dsr01/motion/move_stop`(stop_mode 2=DR_SSTOP, `tools/bench/stream_bench.py`, DR_SSTOP 값은 `dsr_common2/imp/DSR_ROBOT2.py`) | 불필요 |
| `speedl_stream` | `<ns>/speedl_stream`, `dsr_msgs2/msg/SpeedlStream` | `float64[6] vel` / `float64[2] acc` / `float64 time` (`dsr_msgs2/msg/SpeedlStream.msg`). 단위·좌표계 동일(위 servol 행). `vel`은 6축 작업공간 속도, `acc`는 [선, 회전] 2요소 | 속도 지령 유지 시간(초). 이 프로젝트 도구 주석: "DRFL time 인수(servol 도달 시간 / speedl 유지 시간)"(`tools/bench/stream_bench.py:303`). 지령이 끊기면 마지막 `time`이 지난 뒤 제어기가 timeout으로 스스로 멈춤(아래 timeout 열) | 제어기 경고 "[SpeedL] speedl() generates time-out error if it is called for 0.1 [sec]" 관측 → 0.1 s 무지령 timeout으로 자체 정지(실측 근거: `records/bench/stop-2026-11-07-172032.summary.txt`, `records/sessions/OFF-2-2026-11-07.md`). repo 공개 문서에는 해당 동작 기술 없음(문서에 없음). 드라이버 watchdog 없음(콜백 전달만, `dsr_controller2/src/dsr_controller2.cpp` speedl 콜백) | 불필요 |
| `servoj_stream` | `<ns>/servoj_stream`, `dsr_msgs2/msg/ServojStream` | `float64[6] pos` / `float64[6] vel` / `float64[6] acc` / `float64 time` / `int8 mode` — "servoj mode; 0:DR_SERVO_OVERRIDE, 1:DR_SERVO_QUEUE"(`dsr_msgs2/msg/ServojStream.msg`). 관절공간(단위 deg, 확인 필요) | `Drfl->servoj(pos, vel, acc, time, mode)` 전달(`dsr_controller2/src/dsr_controller2.cpp` servoj 콜백). 의미 repo 문서 없음(확인 필요) | 문서에 없음(확인 필요) | 불필요 |
| `speedj_stream` | `<ns>/speedj_stream`, `dsr_msgs2/msg/SpeedjStream` | `float64[6] vel` / `float64[6] acc` / `float64 time` (`dsr_msgs2/msg/SpeedjStream.msg`). 관절공간(deg/s 등, 확인 필요) | `Drfl->speedj(vel, acc, time)` 전달(`dsr_controller2/src/dsr_controller2.cpp`). 의미 repo 문서 없음(확인 필요) | 문서에 없음(확인 필요) | 불필요 |
| `servol_rt_stream` | `<ns>/servol_rt_stream`, `dsr_msgs2/msg/ServolRtStream` | `float64[6] pos` / `float64[6] vel` / `float64[6] acc` / `float64 time` — 주석 "target time [s]; 0 means bound by vel/acc, whose first element must then be positive"(`dsr_msgs2/msg/ServolRtStream.msg`). 단위·좌표계는 servol_stream과 같은 작업공간 규약(확인 필요) | 위 주석 그대로. 드라이버 검증: `time<0`(단, DR_COND_NONE=-10000 제외) 무시, `time==0`이면 `vel[0]>0`·`acc[0]>0` 필요, 아니면 무시(`dsr_controller2/src/dsr_controller2.cpp` `rt_stream_task_is_valid`) | RT 지령 무지령 시 동작 문서에 없음(확인 필요). 참고: RT 읽기 스트림(`read_data_rt`) 정체 감시는 0.1 s(`dsr_hardware2/src/dsr_hw_interface2.cpp` `g_k_rt_stale_timeout_s`), 이것은 joint_states 신선도 감시이지 지령 timeout이 아님 | 필요(아래 1.2) |
| `speedl_rt_stream` | `<ns>/speedl_rt_stream`, `dsr_msgs2/msg/SpeedlRtStream` | `float64[6] vel` / `float64[6] acc` / `float64 time` — 주석 동일(`dsr_msgs2/msg/SpeedlRtStream.msg`) | servol_rt와 동일 규칙(`dsr_controller2/src/dsr_controller2.cpp` speedl_rt 콜백·`rt_stream_task_is_valid`) | 문서에 없음(확인 필요) | 필요(아래 1.2) |

그 외 존재하지만 이번 검토에서 제외한 것: `servoj_rt_stream`·`speedj_rt_stream`·`torque_rt_stream`·`alter_motion_stream`(토픽 구독 목록: `dsr_controller2/src/dsr_controller2.cpp`, 메시지 `dsr_msgs2/msg/*RtStream.msg`).

### 1.2 RT 초기화 절차와 현재 상태

- 서비스 순서: `/dsr01/realtime/connect_rt_control`(요청 `ip_address`(string), `port`(uint32)) → `/dsr01/realtime/set_rt_control_output`(요청 `version`(string), `period`(float64), `loss`(int32)) → `/dsr01/realtime/start_rt_control`(요청 없음). 모두 응답 `bool success`(`dsr_msgs2/srv/ConnectRtControl.srv`, `dsr_msgs2/srv/SetRtControlOutput.srv`, `dsr_msgs2/srv/StartRtControl.srv`; 서비스 등록 이름: `dsr_controller2/src/dsr_controller2.cpp` `realtime/*` 접두).
- 단, real 모드에서 `dsr_hw_interface2`가 bringup 시 이미 수행한다: `connect_rt_control(rt_ip)`(UDP 12347) → `set_rt_control_output("v1.0", 0.001, 4)` → `start_rt_control()` → `set_velj_rt`/`set_accj_rt`(한계 70) (`dsr_hardware2/src/dsr_hw_interface2.cpp`). 실기 로그에도 "Connected RT control stream" 기록(`records/bench/loaded-driver-2026-11-07.log`). 따라서 RT 스트림 토픽은 별도 초기화 없이 바로 시험 가능성이 있다(중복 `connect_rt_control` 호출은 경고 대상: `dsr_common2/imp/DSR_ROBOT2.py` "RT service connection may already have been established by dsr_hw_interface; Need caution using this function again.").
- virtual 모드는 RT 연결을 지원하지 않는다("Virtual controller doesn't support real time connection.", `dsr_hardware2/src/dsr_hw_interface2.cpp`). `mode:=virtual` 확인은 RT 계열에 적용 불가.

### 1.3 QoS·토픽 관측값

- 드라이버 구독 QoS 기본값, depth: `servol_stream` 20, `speedl_stream` 10, RT 스트림 4종 20(`dsr_controller2/src/dsr_controller2.cpp` 구독 생성부). 실측 도구는 `servol` depth 20·`speedl` depth 10으로 송신(`tools/bench/stream_bench.py`). 메시지에 Header가 없어 신선도 검사는 어댑터 몫(`PLAN.md` 3.3절).
- 실기 토픽 확인: `/dsr01/servol_stream`, `/dsr01/speedl_stream`(`records/bench/versions-2026-11-07-170841.txt`).

## 2. 이 프로젝트에서 실측한 것

조건: OFF-2 2026-11-07, E0509 실기, `mode:=real`, 네임스페이스 `/dsr01`, 피드백은 joint_states(100 Hz)+URDF 순기구학, FK−제어기 posx 차 0.00 mm. 지연 = 지령↔실제 위치 최소제곱 시차, 추종 오차는 앞뒤 ramp 2 s 제외 구간(`records/sessions/OFF-2-2026-11-07.md`, `tools/bench/stream_bench.py`).

### 2.1 로봇 단독(60 s, x ±10 mm, 0.5 Hz)

| iface | Hz | `time` | 지령 간격 p99 | 지연 | 추종 p95(원시/시차보정) | 근거 파일 |
|---|---|---|---:|---:|---|---|
| servol | 50 | 0 | 20.2 ms | 113 ms | 3.9 / 0.71 mm | `records/bench/stream-2026-11-07-171449-servol-50hz.*` |
| servol | 100 | 0 | 10.2 ms | 102 ms | 3.5 / 0.61 mm | `records/bench/stream-2026-11-07-171601-servol-100hz.*` |
| speedl | 50 | 0.04 s | 20.1 ms | 52 ms | 1.6 / 0.15 mm | `records/bench/stream-2026-11-07-171726-speedl-50hz.*` |
| speedl | 100 | 0.02 s, 끝 ramp-down | 10.2 ms | 32 ms | 0.97 / 0.18 mm, 끝 위치 중심 ±0.05 mm | `records/bench/stream-2026-11-07-171911-speedl-100hz.*` |

(10 s 사전 점검 servol 50 Hz ±5 mm: 간격 p99 20.1 ms, 지연 74 ms, `records/bench/stream-2026-11-07-171326-servol-50hz.*`.)

### 2.2 참조 부하 조건(D455 RGB+depth 스트리밍, RGB/CameraInfo rosbag 녹화, ArUco 참조 처리; 생산 vision 부하 아님)

| iface | Hz | 간격 p99 ms | 지연 ms | 추종 p95 mm(원시/보정) | 실제 X 범위 mm | 판정 | 근거 |
|---|---:|---:|---:|---|---:|---|---|
| speedl | 100 | 10.146 | 29 | 0.903 / 0.062 | 20.0736 | 합격 | `records/bench/loaded-stream-comparison-2026-11-07.csv` |
| speedl | 50 | 20.130 | 49 | 1.528 / 0.054 | 19.9946 | 합격 | 같음 |
| servol | 100 | 10.225 | 100 | 3.424 / 0.654 | 20.0942 | 합격 | 같음 |
| servol | 50 | 20.601 | 496 | 9.956 / 9.965 | 0.0036 | 실제 이동 없음, 불합격 | 같음 |

- servol 50 Hz 무응답 회차: 마지막 알람 level 1, group 2, index 1216("[ServoL] Can't keep target time due to limit velocity or acceleration"), 원인 미확정, 자동 reset·`move_resume`·드라이버 재시작 없음(`records/bench/loaded-stream-run-2026-11-07.json`, `records/sessions/OFF-2-2026-11-07.md`).
- 부하 조건 10 s 예비 speedl 100 Hz: 간격 p99 10.138 ms, 지연 29 ms, 보정 후 p95 0.064 mm(`records/bench/stream-2026-11-07-195518-speedl-100hz.*`).

### 2.3 stale 지령 정지(speedl만)

- speedl 100 Hz, ±20 mm/s × 0.3 s 후 지령 중단, 20회: 20/20 정지, 재개 0회, 정지까지 p50 0.144 s / p95 0.149 s / 최대 0.150 s, 마지막 지령 뒤 이동 최대 2.34 mm(`records/bench/stop-2026-11-07-172032.summary.txt`, `records/bench/stop-2026-11-07-172032.csv`). `time`은 2 지령주기(0.02 s) 사용.
- 부하 조건의 stale 재시험은 무효(실제 이동 0.000148 mm, "움직이지 않은 로봇의 정지" 판정 방지, exit 2)(`records/sessions/OFF-2-2026-11-07.md`, `records/bench/loaded-stale-raw-2026-11-07.json`).

### 2.4 미측정

- RT 계열 전부(`servol_rt_stream`·`speedl_rt_stream` 및 joint RT): 미측정.
- `servol_stream` stale 지령(무지령 시 정지 여부): 미측정.
- `servoj_stream`·`speedj_stream`: 미측정.
- 통신 단절·cancel·프로세스 종료 시 정지: 미측정(`records/sessions/OFF-2-2026-11-07.md` 미완 항목).

## 3. 선택안과 근거

1. **1순위: `speedl_stream`, 100 Hz, `time` = 지령주기 2배, 궤적 끝 ramp-down.** 근거: 부하 조건에서 지연 29 ms·시차보정 추종 p95 0.062 mm로 4조건 중 최소, 로봇 단독 조건에서도 지연 32 ms·p95 0.18 mm로 최소(2.1·2.2 표). 제어기 쪽 0.1 s 무지령 timeout이 L0 정지 경로로 성립(20/20, p95 149 ms, 이동 ≤ 2.34 mm, 2.3절). 끝 ramp-down 없이 끊으면 시작 위치가 회차마다 밀림(누적 −7 mm) — ramp-down으로 해결(`records/sessions/OFF-2-2026-11-07.md`). `Progress.md` 2절과 동일한 방향.
2. **2순위: `servol_stream` 100 Hz.** 지연 100 ms·보정 p95 0.654 mm으로 speedl 대비 열위(2.2표)이나 위치 지령이라 적분 표류가 없다. 단 servol 50 Hz 무응답 사건(알람 1216)의 원인 규명 전에는 제어 경로 채택 보류(확인 필요).
3. **RT 계열(`speedl_rt_stream` 등): 실측 전 순위 판단 불가.** 초기화는 real 모드에서 드라이버가 이미 수행(1.2절)하므로 시험 장벽은 낮다. RT 계열은 `time=0` 규약이 msg 주석으로 명시되고 드라이버 입력 검증이 있다(`dsr_msgs2/msg/SpeedlRtStream.msg`, `dsr_controller2/src/dsr_controller2.cpp`). 단 virtual 모드 확인 불가(1.2절).
4. 보조 근거·주의: speedl은 속도 지령 적분 기반이라 표류 감시가 필요 — 측정 도구는 중심 대비 이탈 guard를 둔다(`tools/bench/stream_bench.py`). 단위 변환은 `robot_control/motion_adapter.py`에서만 한다(`PLAN.md` 0.2절 규칙). 수치는 제안값이며, 실측 확정 시 `records/param_changes.csv`와 `TEST_SPEC.md` 기대값을 같은 커밋에서 갱신한다(저장소 규칙).

## 4. OFF-2에서 확인할 것

1. RT 계열 실측: `speedl_rt_stream`·`servol_rt_stream`을 100/50 Hz로 동일 사인 프로토콜 측정(`tools/bench/stream_bench.py` RT 확장 필요). 별도 `connect_rt_control` 초기화 없이 바로 토픽 지령이 받아들여지는지(1.2절 관측 확인), 중복 연결 시 경고 동작 확인.
2. `servol_stream` stale 동작: 지령 중단 후 제어기가 스스로 멈추는지, 정지 시간·이동(`speedl`만 측정됨, 2.3절). `speedj`·`servoj`도 대상이면 함께.
3. servol 50 Hz 부하 무응답 원인(알람 group 2, index 1216) 재현·검증: `records/bench/loaded-stream-run-2026-11-07.json`·`records/bench/loaded-driver-2026-11-07.log`의 보존 자료로 원인 검토 후 재시험(`records/sessions/OFF-2-2026-11-07.md`).
4. 통신 단절·`cancel`·`robot_controller_node` 프로세스 종료 시 스트림 정지 시간·이동 거리(드라이버 1세션 제한 유지, `records/sessions/OFF-2-2026-11-07.md` 교훈).
5. `time` 인수 경계: speedl `time`을 지령주기 1배/3배로 바꿀 때 정지 시간·추종 변화, servol `time=0` 경고(알람 "[ServoL] Can't keep target time…") 발생 조건.
6. 그리퍼 상주 DRL 루프 동시 동작 중 스트림 추종 유지(`Progress.md` 2절 그리퍼 항목, `PLAN.md` 11.3절 대안 비교 재료).
7. 설치 드라이버 커밋 `dade65d`와 웹 jazzy 원문의 msg 필드·토픽 이름 일치 확인(1장 전제, 확인 필요).
