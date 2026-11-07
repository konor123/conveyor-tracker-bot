# TEST_SPEC — 이동 병뚜껑 공깃돌 투입 시험 명세

- 확정: 2026-11-05 / 구조·판정 규칙: `PLAN.md` / 진행·게이트: `Progress.md`
- 이 문서는 **테스트 코드를 구현보다 먼저 쓰기 위한 명세**다. 순수 모듈 API(2절)와 시험 케이스(3절)만 보고 실패하는 테스트를 쓸 수 있어야 한다.
- 수치는 `PLAN.md`의 제안값을 따른다. 제안값이 바뀌면 이 문서의 기대값도 같은 커밋에서 고친다. 테스트 코드는 기대값을 하드코딩하지 않고 `config/*.yaml`이나 시나리오 파일에서 읽되, 경계값 시험은 이 문서의 값을 fixture로 고정한다.

## 0. 사용법

### 0.1 TDD 순서

```
SPEC의 REQ 확인 → 이 문서의 TC 선택 → fake·fixture 준비(1절) → 테스트 작성
 → 실패 확인(구현 없음 또는 NotImplementedError) → 최소 구현 → 통과 → 리팩터링 → trace.py 갱신
```

- 실패 확인 없이 통과한 테스트는 테스트가 아무것도 검증하지 않는다는 뜻이므로 기대값이나 입력을 다시 본다.
- 한 TC는 하나의 판단만 검증한다. 경계값 TC는 `임계 직전 / 경계 / 직후` 세 점을 `pytest.mark.parametrize`로 넣는다.
- 버그를 고칠 때는 그 버그를 재현하는 TC를 먼저 추가한다(`TC-<영역>-<REQ>-9xx` 번호대).

### 0.2 ID 체계

- 시험 케이스: `TC-<영역>-<REQ 번호>-<순번>`. `TC-ABT-003-02`는 `REQ-ABT-003`의 두 번째 케이스다.
- 이 문서의 각 표 "검증 내용"은 해당 REQ의 초안이다. SPEC 문서를 쓸 때 같은 번호로 REQ를 옮기고, 수치는 SPEC을 기준으로 맞춘다.
- 영역: `SYS`(규약) · `IF`(인터페이스) · `ORC`(상태 머신·단계 판정) · `ABT`(중단·제한시간·감시) · `VIS`(비전) · `TRK`(추적) · `PLN`(계획) · `ROB`(로봇 제어) · `GRP`(그리퍼) · `CAL`(보정) · `MCK`(대역) · `REC`(기록·분석) · `INT`(ROS 통합) · `SIM`(시뮬레이션) · `BEN`(현장 측정) · `EXP`(게이트 실험)

### 0.3 시험 수준과 장소

| 수준 | 내용 | 실행 수단 | 장소 | CI |
|---|---|---|---|---|
| U | 순수 Python 모듈 단위 시험(ROS 불필요) | `pytest` | `[ON]` | 매 커밋 |
| C | 노드 하나의 외부 계약 시험(대역 또는 장비 없는 실제 노드) | `pytest` + `rclpy`, `test/contract/` | `[ON]` | 매 커밋 |
| I0 | mock 2종 + 오케스트레이터 ROS 통합 | `launch_testing` | `[ON]` | 매 커밋 |
| I1·I2 | replay·fixture 기반 인식·계획 통합 | `launch_testing` + rosbag | `[ON*]`·`[ON]` | 매 커밋(자료가 있을 때) |
| S | Isaac Sim 통합 | Isaac Sim python + ROS 2 Bridge | `[ON]`(RTX GPU PC) | 수동 |
| I3·I4·I5 | 실제 로봇·카메라가 들어가는 통합 | launch + 절차서 | `[OFF]` | 아님 |
| B | 현장 측정 절차(물리량·지연 실측) | 측정 스크립트 + 절차서 | `[OFF]` | 아님 |
| E | 게이트 실험(반복 시도, 관찰자 판독) | `DropTask` + `trial_labeler.py` | `[OFF]` | 아님 |

- `[ON*]` 시험은 필요한 자료가 없으면 **skip**으로 끝나야 하며 pass로 세지 않는다(`needs_data` 마커, 0.4절).
- `[OFF]` 시험은 pytest로 돌리지 않는다. 절차·합격 기준·기록 위치를 이 문서에 두고, 결과는 `records/`와 게이트 보고서에 남긴다.

### 0.4 pytest 규칙

```python
# 각 패키지 test/conftest.py가 공통 conftest를 import 한다
@pytest.mark.req("REQ-ABT-003")
@pytest.mark.tc("TC-ABT-003-01")
@pytest.mark.level("U")                 # U / C / I0 / I1 / I2
@pytest.mark.needs_data("off1_bags")    # [ON*] 전용: 자료가 없으면 skip
def test_l2_path_timeout_escalates_to_l4(fake_clock): ...
```

- U 시험은 실제 시간에 의존하지 않는다. `time.sleep`·`time.time` 대신 `FakeClock`을 쓴다. U 시험 하나는 1 s 안에 끝나야 한다.
- 난수는 seed를 고정하고(`seed` fixture, 기본 0~19), 통계 기준을 쓰는 시험은 seed 20개 모두에서 통과해야 한다.
- 부동소수점 비교는 `pytest.approx(abs=...)`로 하고 허용오차를 이 문서에 적힌 값으로 쓴다.
- `tools/spec/trace.py`는 `req`·`tc` 마커를 모아 REQ ↔ TC ↔ 테스트 함수 행렬을 만들고, TC가 없는 REQ와 테스트가 없는 TC를 출력한다.

## 1. 시험 기반 (테스트보다 먼저 만든다)

| 이름 | 위치 | 역할 | 최소 기능 |
|---|---|---|---|
| `FakeClock` | `orchestrator/test/fakes/clock.py`(다른 패키지의 fake clock도 같은 인터페이스를 따른다) | ROS 시각과 steady 시각을 따로 조작 | `ros_now()`, `steady_now()`, `advance(dt)`(둘 다 진행), `advance_ros(dt)`, `freeze_ros()`, `jump_ros(dt)`(음수 허용) |
| `FakeSkillPort` | `orchestrator/test/fakes/skills.py` | skill 호출을 대본대로 응답 | skill별 `duration`, `result`, `hang`, `feedback` 열, 호출 기록 `calls` |
| `FakeTransport` | `robot_control/test/fakes/transport.py` | Doosan 스트림·move_line 대역 | 송신 기록(steady 시각 포함), 지연·누락·단절 주입, 현재 TCP 상태 1차 지연 모델(선택) |
| `FakeGripper` | `robot_control/test/fakes/gripper.py` | 제어기 레지스터 + 상주 루프 DRL + RH-P12-RN(A) 대역 | 제어기 레지스터 쓰기 기록, 루프 주기(기본 20 ms) 모사, 생존 카운터, 그리퍼 Modbus 주소 맵, 응답 지연 분포, 무응답, Hardware Error 비트, pulse 실측표 기반 위치 응답, 파지 시 전류 응답, Model Number 35074 |
| `SyntheticCamera` | `vision/test/fakes/camera.py` | 합성 영상·투영 | 내부 행렬·왜곡·외부 자세, `project(point)`, `render(objects, background)`(매트 검정·벨트 녹색·원형 물체·잡음·그림자 선택) |
| `TrajectoryFixture` | `tracking/test/fixtures/` | 합성 병뚜껑 궤적 → `ObjectDetectionArray` 열 | 등속·속도 계단·가림 구간·프레임 누락·역순 stamp |
| `ScenarioEngine` | `drop_mocks` | I0·C 시험용 세계 모델 | 7.3절 시나리오 YAML, seed |
| 자료 세트 | 저장소 밖(경로만 `records/datasets.yaml`) | `[ON*]` 시험 입력 | `off1_bags`(속도 3종 × 조명 2종), `off1_static_images`(라벨 포함), `off5_bags` |

자료 세트 라벨 형식(`labels.csv`): `file, frame, object_class, u, v, stone_inside, verdict, labeler`. 라벨은 두 사람이 독립으로 달고, 불일치 프레임은 제외하지 않고 `disputed`로 표시한다.

## 2. 순수 모듈 API (TDD 기준 시그니처)

모든 길이는 m, 시간은 s, 각도는 rad다(그리퍼 레지스터 값은 예외로 pulse·mA). 시각 인자는 float 초(ROS 시각)로 받고 노드에서 `builtin_interfaces/Time`으로 바꾼다.

```python
# ---------- orchestrator ----------
class Result(IntEnum): OK=0; FAILED=1; TIMEOUT=2; PRECONDITION=3; CANCELED=4; FAULT=5
class AbortLevel(IntEnum): NONE=0; L1=1; L2=2; L3=3; L4=4; CONTINUE_TO_RETREAT_VERIFY=10
@dataclass(frozen=True)
class Judgement: result: Result; reason_code: str; metrics: dict[str, float]; detail: str = ""
# 위 세 타입은 common.py. reason "WAIT" = 판정 보류(같은 단계에서 관측 계속), detail = 세부 원인

# step_judges.py — 단계마다 하나. m = 측정값 dataclass, p = 판정 파라미터 dataclass
def judge_precheck(m: PrecheckMeas, p: PrecheckParams) -> Judgement
def judge_detect_stone(m: DetectMeas, p: DetectParams) -> Judgement
def judge_pick_stone(m: PickMeas, p: PickParams) -> Judgement
def judge_move_standby(m: StandbyMeas, p: StandbyParams) -> Judgement
def judge_wait_cap(m: WaitCapMeas, p: WaitCapParams) -> Judgement      # reason "PASS_SKIP" 가능
def judge_plan_intercept(m: PlanMeas, p: PlanParams) -> Judgement
def judge_synchronize(samples: Sequence[SyncSample], p: SyncParams) -> Judgement
def judge_descend(m: DescendMeas, p: DescendParams) -> Judgement
def judge_release(m: ReleaseMeas, p: ReleaseParams) -> Judgement
def judge_retreat(m: RetreatMeas, p: RetreatParams) -> Judgement
def judge_approach(m: ApproachMeas, p: ApproachParams) -> Judgement
def judge_predicted_entry(m: EntryMeas, p: EntryParams) -> Judgement   # MODE_PREDICTED의 SYNCHRONIZE
def judge_static_place(m: StaticPlaceMeas, p: StaticPlaceParams) -> Judgement   # MODE_STATIC의 DESCEND
def judge_verify(m: VerifyMeas, p: VerifyParams) -> Judgement
METRIC_KEYS: dict[str, frozenset[str]]                                  # 단계별 기록 키(SPEC-100 4.4절)

# state_machine.py
def next_state(state: State, mode: Mode, j: Judgement, abort: AbortLevel, ctx: CycleCtx) -> State

# abort_policy.py
def decide(event: Event, ctx: AbortCtx) -> AbortLevel
def escalate(current: AbortLevel, new: AbortLevel) -> AbortLevel
class AbortTracker:                        # 진행 중 등급 + 중단 경로·cancel 제한시간(steady)
    def __init__(self, p: AbortParams, clock: SteadyClock): ...
    def on_event(self, event: Event, ctx: AbortCtx) -> AbortLevel
    def on_cancel_sent(self) -> None
    def on_cancel_ack(self) -> None
    def on_path_done(self) -> None         # 중단 경로 완료 → WAIT_OPERATOR
    def tick(self) -> AbortLevel           # 제한시간 초과 시 승격

# watchdogs.py — 감시(SPEC-110 4.3절). 시계는 생성자에서 받는다
class HeartbeatMonitor:   beat(node), lost(node) -> bool                  # steady
class FreshnessMonitor:   stale(t_now, t_last_obs) -> bool                # ROS
class ClockMonitor:       update(t_ros) -> bool                           # True = 역행
class GripperMonitor:     update(error_bits, response_s, alive_counter), fault() -> bool   # steady
class CycleTimer:         expired() -> bool                               # ROS deadline 또는 steady 상한

# deadlines.py
def effective_deadline(now: float, step_limit: float, dynamic: float | None, cycle: float) -> float
def derive_step_timeout(p95: float, p99: float) -> float          # max(ceil(1.5*p99), p95+0.5)

# cycle_runner.py — 한 사이클 실행(SPEC-100 4.2절). skill은 SkillPort, 시계는 FakeClock 인터페이스
class CycleRunner:
    def __init__(self, port: SkillPort, clock: Clock, cfg: OrchestratorCfg): ...
    def run(self, goal: DropGoal) -> CycleOutcome   # 상태 열, StepRecord 열, 결과, passes_skipped
    def resume(self, reason: str) -> bool          # WAIT_OPERATOR 재개, 빈 사유는 거부

# ---------- intercept_planner ----------
@dataclass(frozen=True)
class Budget: t_available: float; t_required: float; ok: bool; components: dict[str, float]
def evaluate_budget(x_cap: float, v_cap: float, x_exit: float,
                    components: dict[str, float], margin: float) -> Budget
def predict(track: TrackState, t: float) -> tuple[np.ndarray, np.ndarray]   # (pos[2], cov[2x2])
def compute_plan(track: TrackState, now: float, req: PlanRequest, p: PlannerParams) -> Plan
def tau_max(e_allow: float, e_pos: float, dv: float, a: float, v: float, dt: float) -> float

# ---------- tracking ----------
class KalmanCV:
    def __init__(self, q_accel: float, x0: np.ndarray, P0: np.ndarray): ...
    def predict(self, t: float) -> None
    def update(self, t: float, z: np.ndarray, R: np.ndarray) -> float   # 반환: NIS
def associate(tracks: Sequence[TrackState], dets: Sequence[Det], gate_chi2: float) -> Assignment
class SpeedChangeDetector:
    def update(self, t: float, innovation_v: float) -> bool              # True = 변화 검출

# ---------- vision ----------
def detect(image_bgr: np.ndarray, roi: Roi, classes: dict[int, ColorModel]) -> list[Blob]
def pixel_to_plane(u: float, v: float, cam: CameraModel, T_base_cam: np.ndarray,
                   plane: Plane) -> tuple[np.ndarray, np.ndarray]         # (point[3], cov[3x3])
def stone_inside(cap: Blob, stones: Sequence[Blob], p: InsideParams) -> bool | None  # None = 판정 불가
def verify(frames: Sequence[FrameObs], cap_track_id: int, p: VerifyParams) -> Verdict

# ---------- robot_control ----------
def pose_si_to_doosan(pos_m: np.ndarray, quat_xyzw: np.ndarray) -> list[float]   # [mm×3, deg ZYZ×3]
def pose_doosan_to_si(p: Sequence[float]) -> tuple[np.ndarray, np.ndarray]
class FreshnessWatchdog:
    def __init__(self, period: float, clock: SteadyClock): ...
    def on_command(self) -> None
    def check(self) -> bool                                              # False = L0
def tracking_command(p_target, v_ff, p_tcp, kp: float, v_max: float, a_max: float,
                     v_prev, dt: float) -> np.ndarray

class GripperAdapter:                      # 그리퍼 지령은 이 클래스만 보낸다
    def __init__(self, transport: GripperTransport, table: PulseTable, cfg: GripperCfg,
                 clock: SteadyClock): ...
    def startup_check(self) -> None        # Model Number·펌웨어·Hardware Error 확인, 실패 시 예외
    def close(self, current_limit_ma: int) -> GripResult
    def open(self, width_m: float) -> OpenResult
    def read_state(self) -> GripperState    # width_m, current_ma, error_bits, t_read
class PulseTable:
    def width_to_pulse(self, w: float) -> int
    def pulse_to_width(self, p: int) -> float

# ---------- calibration (SPEC-600) ----------
def fit_plane(points: np.ndarray, ransac_thresh: float, up=(0, 0, 1), rng=None) -> Plane   # Plane.height_at(x, y)
def fit_direction(points: np.ndarray, plane: Plane | None = None) -> np.ndarray
def solve_eye_to_hand(cam_pts: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, float]  # (T, rms [m])
def solve_eye_to_hand_pnp(base_pts, uv, K, dist) -> tuple[np.ndarray, float]                  # (T, rms [px])
def solve_from_board_views(views: list[BoardView], spec: BoardSpec, K, dist) -> tuple[np.ndarray, dict]
def board_track_direction(frames, spec, K, dist, T, plane=None) -> np.ndarray
def pixel_to_plane(uv, K, dist, T, plane: Plane) -> np.ndarray
def verify_xy(true_pts, est_pts) -> dict                     # n, xy_p50_m, xy_p95_m, xy_max_m, pass
def fit_tcp_offset(R, t, p, fixed: dict | None = None) -> tuple[np.ndarray, float]
def detect_charuco(image, spec, K, dist) -> tuple[np.ndarray, np.ndarray]   # (코너 id, uv)
def board_pose(ids, uv, spec, K, dist) -> tuple[np.ndarray, float]          # (camera ← board, rms [px])
def load_extrinsics(path) -> Extrinsics                     # 누락 시 CalibrationFileError
```

## 3. 시험 케이스

표의 "장소"는 `[ON]`/`[ON*]`/`[OFF]`(`PLAN.md` 11.4절). 기대값 수치는 `PLAN.md` 제안값 기준이다.

### 3.1 SYS — 규약

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-SYS-001-01 | 프로젝트 패키지 이름은 underlay와 겹치지 않는다 | U | `[ON]` | workspace를 source하지 않은 셸에서 10개 패키지마다 `ros2 pkg prefix <name>` | 모두 "not found" |
| TC-SYS-002-01 | 설정 파일의 수치 키는 단위 접미사를 가진다 | U | `[ON]` | 모든 `config/*.yaml`, `scenarios/*.yaml` 파싱 | 수치 키가 모두 `_m`, `_s`, `_mps`, `_mps2`, `_rad`, `_hz`, `_ma`, `_pulse`, `_count`, `_ratio` 중 하나로 끝남(예외 목록은 파일 안 `unitless:`) |
| TC-SYS-003-01 | 관측 메시지 stamp는 영상 노출 시각이다 | C | `[ON]` | `vision_node`에 stamp T인 합성 영상 입력, 처리 지연 50 ms 주입 | `ObjectDetectionArray.header.stamp == T`(정확히 일치) |
| TC-SYS-003-02 | track stamp는 상태 시각이다 | C | `[ON]` | `tracker_node`에 stamp T 검출 입력, 수신은 T+80 ms | `TrackedObject.header.stamp == t_state == T` |
| TC-SYS-004-01 | 단위 변환은 adapter 두 곳에만 있다 | U | `[ON]` | 소스 정적 검사: `* 1000`, `math.degrees`, `/ 1000` 패턴 | `motion_adapter.py`, `gripper_adapter.py`, 테스트 외에는 0건 |

### 3.2 IF — 인터페이스

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-IF-001-01 | 모든 msg·srv·action 정의가 SPEC-010 스냅샷과 같다 | U | `[ON]` | `ros2 interface show` 출력과 `specs/snapshots/*.txt` 비교 | 차이 0. 차이가 있으면 SPEC-010 버전을 올려야 통과 |
| TC-IF-002-01 | 모든 srv 요청·action goal은 공통 요청 필드로 시작한다(외부 진입점 `DropTask` 제외) | U | `[ON]` | 생성된 Python 클래스의 필드 순서 조사 | 처음 세 필드가 `cycle_id`, `step_id`, `deadline` |
| TC-IF-003-01 | 상수 값이 명세와 같다 | U | `[ON]` | `ObjectDetection`, `TrackedObject`, `BeltMotion`, `InterceptPlan`, `DropTask` 상수 | `CLASS_STONE=1`, `CLASS_CAP=2`, `CLASS_BELT_MARKER=3`, `OBSERVED=0`, `COASTING=1`, `LOST=2`, `NOT_ATTEMPTED=6` 등 3장 값과 일치 |
| TC-IF-004-01 | 발행 QoS가 3.3절 표와 같다 | C | `[ON]` | 노드별 실행 후 `get_publishers_info_by_topic` | reliability·durability·depth 일치 |
| TC-IF-004-02 | QoS가 맞지 않는 구독은 계약 시험에서 실패로 잡힌다 | C | `[ON]` | best effort 발행자 + reliable 구독자 구성 | 계약 시험이 `QOS_MISMATCH`로 실패(오류 없는 무수신을 놓치지 않음) |

### 3.3 ORC — 상태 머신·단계 판정

경계 시험 공통 파라미터: `d_stone_min = 0.017 m`, `d_stone_max = 0.018 m`, `Δ_open = 0.004 m`, `e_xy_max = 0.00275 m`, `v_rel_max = 0.005 m/s`, `h_clear = 0.002 m`.

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-ORC-001-01 | MODE_SYNCHRONIZED 정상 상태 열 | U | `[ON]` | 모든 판정 OK | `IDLE→PRECHECK→DETECT_STONE→PICK_STONE→MOVE_STANDBY→WAIT_CAP→PLAN_INTERCEPT→APPROACH→SYNCHRONIZE→DESCEND→RELEASE→RETREAT→VERIFY→DONE` |
| TC-ORC-001-02 | MODE_STATIC은 PLAN_INTERCEPT·SYNCHRONIZE를 건너뛰고 WAIT_CAP을 정지 뚜껑 검출로 바꾼다 | U | `[ON]` | mode = STATIC, 모든 판정 OK | PLAN_INTERCEPT·SYNCHRONIZE 없음, WAIT_CAP 단계에서 `DetectObjects(REGION_BELT, CLASS_CAP)` 호출, `PlaceStone` 사용 |
| TC-ORC-001-03 | MODE_PREDICTED는 SYNCHRONIZE를 계획 속도 일치 구간 진입 판정으로 바꾼다 | U | `[ON]` | mode = PREDICTED | SYNCHRONIZE 판정 입력이 계획 구간 진입 시각·속도 오차 |
| TC-ORC-002-01 | 판정 실패 뒤 다음 단계 skill을 부르지 않는다 | U | `[ON]` | 각 단계를 하나씩 FAILED로 대본 작성(단계 수만큼 parametrize) | `FakeSkillPort.calls`에 다음 단계 skill 없음 |
| TC-ORC-003-01 | PRECHECK heartbeat 나이 경계 | U | `[ON]` | 나이 0.299 / 0.300 / 0.301 s | OK / OK / FAILED(`HEARTBEAT_STALE`) |
| TC-ORC-003-02 | PRECHECK 그리퍼 응답·상태 | U | `[ON]` | 응답 0.5 s / 0.501 s, error_bits ≠ 0, 개방 아님 | OK / FAILED(`GRIPPER_SLOW`) / FAILED(`GRIPPER_ERROR`) / FAILED(`GRIPPER_NOT_OPEN`) |
| TC-ORC-003-03 | DETECT_STONE 판정 | U | `[ON]` | confidence 0.89 / 0.90, 창 표준편차 0.5 / 0.51 mm, 개수 0 | FAILED / OK, OK / FAILED, FAILED(`NO_STONE`) |
| TC-ORC-003-04 | PICK_STONE 핑거 간격 경계 | U | `[ON]` | 간격 15.9 / 16.0 / 19.0 / 19.1 mm(`[d_stone_min − 1, d_stone_max + 1] mm`) | FAILED / OK / OK / FAILED(`GRIP_WIDTH`) |
| TC-ORC-003-05 | PICK_STONE 전류·매트 확인 | U | `[ON]` | 전류 < 하한 / 매트 ROI에 공깃돌 남음 | FAILED(`GRIP_CURRENT_LOW`) / FAILED(`STONE_ON_MAT`) |
| TC-ORC-003-06 | MOVE_STANDBY 자세 오차 | U | `[ON]` | 2.0 / 2.1 mm, holding false | OK / FAILED, FAILED(`DROPPED`) |
| TC-ORC-003-07 | WAIT_CAP 추적 안정 경계 | U | `[ON]` | observation_count 29 / 30, speed_std 1.00 / 1.01 mm/s, `steady=false` | 통과 대기 / OK, OK / 대기, 대기 |
| TC-ORC-003-08 | WAIT_CAP 빈 뚜껑·시간 예산 | U | `[ON]` | `stone_inside=true` / `stone_inside_valid=false` / margin −0.01 s / margin 0 | FAILED(`CAP_NOT_EMPTY`) / reason `PASS_SKIP`(`CAP_STATE_UNKNOWN`) / reason `PASS_SKIP` / OK |
| TC-ORC-003-09 | PLAN_INTERCEPT 판정 | U | `[ON]` | time_margin 0.99 / 1.00 s, position_std 0.5 / 0.51 mm, `valid_until = now + T_approach` | FAILED / OK, OK / FAILED, FAILED(경계는 `>`) |
| TC-ORC-003-10 | SYNCHRONIZE 연속 0.3 s | U | `[ON]` | 100 Hz 표본, 0.25 s 시점에 e_xy 1회 초과 | 초과 시점부터 다시 셈, 초과 후 0.3 s 연속 만족한 시점에 OK, 2 s 안에 못 채우면 TIMEOUT |
| TC-ORC-003-11 | DESCEND 핑거 끝 여유 | U | `[ON]` | 핑거 끝 − 테두리 = 1.9 / 2.0 mm | FAILED(`RIM_CLEARANCE`) / OK |
| TC-ORC-003-12 | RELEASE 개방 확인 | U | `[ON]` | 간격 20.9 / 21.0 mm(`d_stone_max+Δ_open−1 mm` = 21 mm) | FAILED / OK |
| TC-ORC-003-13 | RETREAT 판정 | U | `[ON]` | z < 안전 높이 / 그리퍼에 공깃돌 보임 | FAILED → L4 / FAILED → L4 |
| TC-ORC-003-14 | VERIFY 제한시간 | U | `[ON]` | `VerifyDrop` 3 s 무응답 | 결과 UNKNOWN, reason `VERIFY_TIMEOUT` |
| TC-ORC-004-01 | 판정에 쓴 값이 모두 StepRecord에 남는다 | U | `[ON]` | 각 judge 호출 | `Judgement.metrics` 키가 단계별 명세 키 집합과 같고, StepRecord `metric_keys`에 그대로 복사 |
| TC-ORC-005-01 | 선행조건 불충족이면 실행하지 않는다 | U | `[ON]` | PICK_STONE 검출 시각 1.01 s 전 | `PRECONDITION`, skill 호출 0회, 로봇 미동작 |
| TC-ORC-006-01 | 재시도 횟수 | U | `[ON]` | DETECT_STONE 2회 실패 후 OK / 3회 실패 | attempt 3에서 진행 / L2 |
| TC-ORC-006-02 | 재시도는 선행조건부터 다시 본다 | U | `[ON]` | PICK_STONE 실패 후 재시도 시 검출 시각이 1 s 지남 | DETECT_STONE부터 다시 실행 |
| TC-ORC-007-01 | 통과 보내기 | U | `[ON]` | max_passes = 2, 1번째 통과 margin 부족, 2번째 OK | `passes_skipped = 1`, PLAN_INTERCEPT 진행 |
| TC-ORC-007-02 | 통과 보내기 소진 | U | `[ON]` | max_passes = 2, 세 통과 모두 부족 | L3, `SafeRetreat(keep_sync=false, place_back_pose=매트)` 호출, 결과 `NOT_ATTEMPTED` |
| TC-ORC-008-01 | 대기 자세에서 계획 실패는 다음 통과로 넘긴다 | U | `[ON]` | PLAN_INTERCEPT가 동적 deadline까지 실패, 남은 통과 있음 | APPROACH 호출 0회, WAIT_CAP 복귀 |
| TC-ORC-009-01 | WAIT_OPERATOR 재개 | U | `[ON]` | `resume` 사유 없음 / 사유 있음 | 거부 / PRECHECK부터 재시작, 진입 시각·사유 기록 |

### 3.4 ABT — 중단·제한시간·감시

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-ABT-001-01 | `decide()` 판정 순서 전수 | U | `[ON]` | 모든 상태 × {HARD_FAULT, 단계 실패, cancel} × holding {T,F} × released {T,F} × 재시도 {남음, 없음} | `PLAN.md` 5.2절 순서와 일치(기대표는 `test/data/decide_table.csv`, 이 표가 명세) |
| TC-ABT-002-01 | 높은 등급이 낮은 등급을 덮어쓴다 | U | `[ON]` | L3 진행 중 L4 / L4 진행 중 L2 | L4 / L4 유지 |
| TC-ABT-003-01 | L2 중단 경로 제한시간 | U | `[ON]` | steady 10.000 / 10.001 s 경과 | 유지 / L4 |
| TC-ABT-003-02 | L3 중단 경로 제한시간 | U | `[ON]` | steady 15.000 / 15.001 s 경과 | 유지 / L4 |
| TC-ABT-004-01 | cancel 무응답 승격 | U | `[ON]` | cancel 후 0.5 s 응답 없음 | 한 단계 승격 |
| TC-ABT-005-01 | 들고 있을 때 사용자 cancel은 L3 | U | `[ON]` | holding = true, 사용자 cancel | L3 |
| TC-ABT-006-01 | 실제 deadline은 세 값의 최솟값 | U | `[ON]` | 단계·동적·사이클이 각각 최소인 3경우, 동적 = None | 최솟값 / None은 무시 |
| TC-ABT-007-01 | 측정 기반 단계 제한시간 | U | `[ON]` | (p95, p99) = (1.0, 1.2) / (2.0, 2.1) | 2.0 / 4.0(`max(ceil(1.5·p99), p95+0.5)`) |
| TC-ABT-008-01 | `/clock`이 멈춰도 사이클은 끝난다 | U | `[ON]` | `freeze_ros()`, steady만 `goal.timeout × 2` 진행 | 사이클 중단(L2, 들고 있으면 L3) |
| TC-ABT-009-01 | 감시 타이머는 steady 시계를 쓴다 | U | `[ON]` | `jump_ros(−5 s)` 후 heartbeat 정상 / `freeze_ros()` 후 heartbeat 0.3 s 끊김 | 오검출 없음 / 검출 |
| TC-ABT-010-01 | heartbeat 3회 누락 | U | `[ON]` | 누락 0.299 / 0.300 s, 대상 vision(동적 구간) / vision(비동적) / robot_controller | 유지 / 검출 → L3 / L2 / L4 |
| TC-ABT-011-01 | 동적 구간 관측 신선도 | U | `[ON]` | `t_now − t_last_obs` = 0.300 / 0.301 s, 해제 전 | 유지 / L3 |
| TC-ABT-012-01 | ROS 시각 역행 | U | `[ON]` | 역행 1회 | track·plan 무효화, 들고 있으면 L3 아니면 L2 |
| TC-ABT-013-01 | 벨트 속도 급변 | U | `[ON]` | `steady=false`, 해제 전 / 해제 후 | L3 / 계속(RETREAT→VERIFY) |
| TC-ABT-014-01 | 그리퍼 감시 | U | `[ON]` | error_bits ≠ 0 또는 응답 0.501 s, holding / 비holding | L3 / L2 |
| TC-ABT-015-01 | 장애 10종 이벤트 열 | U | `[ON]` | `PLAN.md` 7.3절 10종을 이벤트 열로(시나리오마다 1 TC, 순번 01~10) | 시나리오별 기대 등급(`test/data/fault_expectations.yaml`) |
| TC-ABT-016-01 | WAIT_OPERATOR는 제한시간이 없다 | U | `[ON]` | steady 1 h 경과 | 상태 유지, 승격 없음 |

### 3.5 VIS — 비전

합성 조건 기본값: 0.55 mm/px(높이 0.35 m 수직, 컬러 1280×800·HFOV 90°), 공깃돌 팔각형(면-면 17 mm, 꼭짓점-꼭짓점 18 mm, 약 33 px), 병뚜껑 바깥 지름 32 mm(약 58 px), 회색 잡음 σ = 2.

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-VIS-001-01 | 공깃돌 중심 검출 정확도 | U | `[ON]` | 매트 배경, 무작위 위치 100개 × seed 20 | 중심 오차 p95 ≤ 0.2 px, 지름 오차 ≤ 1 px |
| TC-VIS-001-02 | 병뚜껑 중심 검출 정확도 | U | `[ON]` | 벨트 배경, 같은 조건 | 중심 오차 p95 ≤ 0.2 px |
| TC-VIS-002-01 | 대상물 분류 | U | `[ON]` | 공깃돌·병뚜껑·표식 혼합 장면 200개 | 오분류 0 |
| TC-VIS-003-01 | `stone_inside` 판정 | U | `[ON]` | 뚜껑 안 중앙 / 안쪽 테두리 접촉 / 테두리 밖 접촉 / 빈 뚜껑 / 그리퍼 마스크로 50% 가림 | true / true / false / false / None |
| TC-VIS-004-01 | 광선-평면 교차 왕복 | U | `[ON]` | 알려진 평면 위 점 투영 후 역투영, 왜곡 계수 0과 비0 | 오차 ≤ 1e-6 m |
| TC-VIS-005-01 | 대상물별 평면 높이를 쓴다 | U | `[ON]` | 같은 픽셀, class STONE(매트) / CAP(벨트) / 뚜껑 안 STONE | 각각 `mat + h_stone_offset`, `belt + h_rim`, `belt + 바닥 두께 + h_stone_offset` 평면과 교차 |
| TC-VIS-006-01 | 위치 공분산 전파 | U | `[ON]` | 픽셀 공분산 diag(0.25, 0.25) | 수치 야코비안 결과와 각 원소 5% 이내 |
| TC-VIS-007-01 | 로봇 마스크 겹침 | U | `[ON]` | 검출이 마스크와 겹침 | `occluded_partial = true`, `DetectObjects` 개수에서 제외 |
| TC-VIS-008-01 | `DetectObjects` 안정 창 | C | `[ON]` | fixture 검출 지터 0.3 mm / 0.6 mm, `stable_window = 0.5 s` | OK / FAILED |
| TC-VIS-008-02 | `DetectObjects` deadline | C | `[ON]` | 이미 지난 deadline / 처리 중 경과 | 즉시 `TIMEOUT` / `TIMEOUT` |
| TC-VIS-009-01 | 자동 투입 판정(합성) | U | `[ON]` | 정상 투입 / 튕겨 나가 벨트 위 / 그리퍼에 붙음 / 가림으로 미확인 / 0.9 s 안에 있다가 나감 | SUCCESS / FAILURE / FAILURE / UNKNOWN / FAILURE |
| TC-VIS-009-02 | 자동 투입 판정(실영상) | U | `[ON*]` | `off1_static_images` 라벨 세트(`disputed` 제외 집계, 별도 공개) | `stone_inside` 정확도 ≥ 99%, verdict 혼동행렬 출력 |
| TC-VIS-010-01 | 실데이터 검출 회귀 | I1 | `[ON*]` | `off1_bags` replay | 라벨 프레임 recall ≥ 99%, 오선택 ≤ 1%(class별) |

### 3.6 TRK — 추적

합성 조건 기본값: 30 Hz, 측정 잡음 σ = 0.3 mm, 속도 20 mm/s, `q_accel`은 config 값.

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-TRK-001-01 | 등속 추적 정확도 | U | `[ON]` | 60 s, seed 20 | 관측 30회 이후 위치 오차 p95 ≤ 0.3 mm, 속도 오차 p95 ≤ 0.5 mm/s |
| TC-TRK-001-02 | 필터 일관성 | U | `[ON]` | 같은 조건 | 평균 NEES가 χ²(4) 95% 구간 안 |
| TC-TRK-002-01 | 역순 stamp는 버린다 | U | `[ON]` | t_state보다 이른 검출 입력 | 상태 불변, 폐기 카운트 +1 |
| TC-TRK-003-01 | 가림 중 COASTING | U | `[ON]` | 0.5 s 관측 없음 | `COASTING`, 공분산 trace 단조 증가 |
| TC-TRK-003-02 | 재획득 게이팅 | U | `[ON]` | 가림 후 게이트 안 / 밖 관측 | 같은 track_id `OBSERVED` / 새 track |
| TC-TRK-004-01 | LOST 전환 | U | `[ON]` | 관측 없음이 coast 한도(config, 초기 1.0 s) 초과 | `LOST` |
| TC-TRK-005-01 | 통과마다 새 track | U | `[ON]` | 뚜껑이 FOV를 나갔다가 `T_lap` 뒤 다시 들어옴 | 새 track_id, epoch 불변 |
| TC-TRK-006-01 | 속도 변화 검출 | U | `[ON]` | t = 10 s에 20 → 30 mm/s | 1.0 s 안에 `steady=false` |
| TC-TRK-006-02 | 속도 변화 오검출 없음 | U | `[ON]` | 60 s 등속, seed 20 | 오검출 0회 |
| TC-TRK-007-01 | 프레임 10% 누락 | U | `[ON]` | 무작위 10% 누락 | 유효 추적 ≥ 99%, 오차가 TC-TRK-001-01 기준의 1.1배 이내 |
| TC-TRK-008-01 | 시각 역행 시 초기화 | U | `[ON]` | `jump_ros(−1 s)` | 모든 track 제거, epoch +1 |
| TC-TRK-009-01 | `ResetTracking` | C | `[ON]` | 호출 | `new_epoch = 이전 + 1`, track 비움 |
| TC-TRK-010-01 | `SelectTarget` | C | `[ON]` | 도달 구간 상류의 뚜껑 1개 / 없음 / 이미 구간 안 | track 반환, 진입 시각 오차 ≤ 50 ms / `PRECONDITION` / 다음 통과 대상 없음 `PRECONDITION` |
| TC-TRK-011-01 | `BeltMotion` | C | `[ON]` | 등속 입력 | `source = SOURCE_CAP`, `|direction| = 1 ± 1e-9`, speed 오차 ≤ 0.5 mm/s |
| TC-TRK-012-01 | 관측 TF | C | `[ON]` | 추적 중 | `cap_<id>` stamp = t_state, 미래 stamp 0건, orientation identity |
| TC-TRK-013-01 | 실데이터 추적 회귀 | I1 | `[ON*]` | `off1_bags`(정상·가림·누락·속도 변화) | 유효 추적 ≥ 99%(가림 구간 별도), 속도 변화 검출 지연 ≤ 1.0 s |

### 3.7 PLN — 계획

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-PLN-001-01 | 시간 예산 식 | U | `[ON]` | `x_exit = 0.30`, `x_cap = 0.0`, `v = 0.02`, 성분 합 4.0, margin 1.0 | `t_available = 15.0`, `t_required = 4.0`(margin 미포함), ok |
| TC-PLN-001-02 | 예산 경계 | U | `[ON]` | `t_available − t_required = margin` / `margin − 1e-6` | ok / not ok |
| TC-PLN-001-03 | 벨트 정지·역방향 | U | `[ON]` | 동적 모드에서 `v ≤ v_min` 또는 진행 방향 반대 | not ok, reason `BELT_STOPPED` / `BELT_REVERSED` |
| TC-PLN-002-01 | 예측은 t_state 기준이다(지연 중복 보상 없음) | U | `[ON]` | t_state = now − 0.1 s, v = 20 mm/s, t_release = now + 2 s | 예측 위치 = p + v·2.1 s(±1e-9) |
| TC-PLN-003-01 | 예측 공분산 전파 | U | `[ON]` | 임의 P, dt | `σ² = P_xx + 2·dt·P_xv + dt²·P_vv`(축별)와 일치 |
| TC-PLN-004-01 | 파지 오프셋 반영 | U | `[ON]` | tcp_offset (1, −0.5) mm | `target_tcp_pose` XY = 뚜껑 중심 − 오프셋 |
| TC-PLN-005-01 | 도달 불가 | U | `[ON]` | 해제점이 작업영역 밖 | status `UNREACHABLE` |
| TC-PLN-006-01 | revision·만료 | C | `[ON]` | 같은 plan을 3회 갱신 / `valid_until` 경과 | revision 1, 2, 3 단조 증가, plan_id 불변 / `EXPIRED` |
| TC-PLN-007-01 | epoch 불일치 | C | `[ON]` | 요청 epoch ≠ 현재 epoch | result FAILED, reason `EPOCH_MISMATCH` |
| TC-PLN-008-01 | 불확실성 | U | `[ON]` | position_std 0.51 mm | status `UNCERTAIN` |
| TC-PLN-009-01 | earliest_release 준수 | U | `[ON]` | earliest_release가 계산 시각보다 늦음 | `release_time ≥ earliest_release` |
| TC-PLN-010-01 | τ_max 계산 | U | `[ON]` | e_allow 2.75 mm, e_pos 0.3 mm, δv 0.5 mm/s, a 0, v 20 mm/s, δt 5 ms | 식을 만족하는 최대 τ(해석해와 1e-6 s 이내) |
| TC-PLN-011-01 | I2 예측·예산 통합 | I2 | `[ON]` | fixture tracked → 실제 planner: 정상, `time_insufficient`, `UNCERTAIN`, 만료·epoch 불일치·clock reset | 시나리오별 status·reason이 기대표와 일치 |

### 3.8 ROB — 로봇 제어

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-ROB-001-01 | 자세 변환 기준값 | U | `[ON]` | identity, x = 0.1 m, z축 90° 회전 | `[0,0,0,0,0,0]`, `[100,0,0,…]`, ZYZ 기대값(`test/data/pose_pairs.csv`) |
| TC-ROB-001-02 | 자세 변환 왕복 | U | `[ON]` | 무작위 quaternion 1000개 + 짐벌 특이(β = 0, π) | 왕복 회전 오차 ≤ 1e-9 rad(특이 자세는 같은 회전이면 통과) |
| TC-ROB-002-01 | 속도 단위 변환 | U | `[ON]` | 0.02 m/s, 0.1 rad/s | 20 mm/s, 5.7296 deg/s |
| TC-ROB-003-01 | freshness 감시(L0) | U | `[ON]` | 주기 10 ms, 지령 간격 20.000 / 20.001 ms | 유지 / 속도 0 지령 1회 송신, 이후 지령 거부(재무장 전까지) |
| TC-ROB-004-01 | 지령 범위 제한 | U | `[ON]` | 작업영역 밖 목표, 속도 한도 초과 | 송신 안 함, L0, reason `RANGE` |
| TC-ROB-005-01 | 지령 주체 단일 | C | `[ON]` | 전체 launch 후 그래프 조사 | 스트림 토픽·move_line·그리퍼 호출 경로의 클라이언트가 `robot_controller_node` 하나 |
| TC-ROB-006-01 | 추종 지령 식과 제한 | U | `[ON]` | 오차 0 / 오차 큼 / 이전 속도와 차이 큼 | `v = v_ff` / `|v| = v_max` / `|Δv| ≤ a_max·dt` |
| TC-ROB-007-01 | skill 계약 | C | `[ON]` | skill 5종 각각: 수락·feedback·결과 코드 | 3.2절 명세와 일치 |
| TC-ROB-007-02 | cancel·deadline·Stop | C | `[ON]` | cancel / deadline 경과 / 실행 중 `Stop` | 0.5 s 안 CANCELED / `TIMEOUT` 자체 종료 / 실행 goal 선점, 정지 확인 뒤 `stopped = true` 응답 |
| TC-ROB-008-01 | DynamicPlace plan 추종 | U | `[ON]` | 낮은 revision / epoch 변경 / 만료 | 무시 / FAILED `PLAN_INVALID` / FAILED `PLAN_INVALID` |
| TC-ROB-009-01 | 해제 게이트 미통과 시 그리퍼를 열지 않는다 | U | `[ON]` | 게이트 항목을 하나씩 위반(5항목) | `FakeGripper` open 호출 0회, 결과 L3 요청 |
| TC-ROB-010-01 | 동기 유지 후퇴 | U | `[ON]` | `SafeRetreat(keep_sync=true)` | z가 안전 높이에 닿을 때까지 XY 지령 속도 = 계획 속도 ± 1 mm/s |
| TC-ROB-011-01 | virtual 모드 형식 확인 | C | `[ON]` | Doosan `mode:=virtual`(지원 시) | 선정 stream·`move_line` 요청이 거부 없이 수락. 미지원이면 stub로 대체하고 skip 사유 기록 |
| TC-ROB-012-01 | Heartbeat | C | `[ON]` | 10 s 관찰 | 10 Hz ± 10%, seq 연속 |

### 3.9 GRP — 그리퍼(RH-P12-RN(A), 상주 루프 DRL 경유)

`FakeGripper`의 pulse 실측표 기본값은 선형(0 pulse = 106 mm 개방, 1150 pulse = 0 mm)으로 두고, OFF-1 실측표가 나오면 그 표로 바꾼다. 방향(0이 개방인지 닫힘인지)도 실측표에서 확정한다.

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-GRP-001-01 | pulse ↔ 간격 변환 | U | `[ON]` | 표 안 임의 점 1000개 왕복 | 간격 오차 ≤ 0.05 mm |
| TC-GRP-001-02 | 표 검증 | U | `[ON]` | 비단조 표 / 범위 밖 요청 | 로드 실패 / 예외(`OUT_OF_RANGE`), 클램프하지 않음 |
| TC-GRP-002-01 | 시작 확인 | U | `[ON]` | Model Number 35074 / 다른 값 / Hardware Error ≠ 0 | 통과 / 예외(구형 펌웨어 표 사용 방지) / 예외 |
| TC-GRP-003-01 | 레지스터 주소(프로토콜별) | U | `[ON]` | config 주소표, protocol = dynamixel2 / modbus | dynamixel2: Torque Enable 512, Hardware Error Status 518, Goal Current 550, Profile Velocity 560, Goal Position 564, Present Current 574, Present Position 580 / modbus: Torque Enable 256, Hardware Error Status 259(하위 바이트), Goal Current 275, Goal Position 282, Present Current 287, Present Position 290. 두 표를 섞어 쓰면 로드 실패 |
| TC-GRP-003-02 | EEPROM 쓰기는 토크 꺼진 상태에서만 | U | `[ON]` | 토크 켜진 상태에서 Operating Mode 쓰기 요청 | 쓰기 0건, 예외 |
| TC-GRP-004-01 | 닫기 순서 | U | `[ON]` | `close(current_limit_ma)` | 쓰기 순서: (필요 시) 모드 5 확인 → Goal Current → Goal Position. 한도(config 최대) 초과 전류는 거부 |
| TC-GRP-005-01 | 부분 개방 | U | `[ON]` | `open(0.019)` | Goal Position = `width_to_pulse(0.019)`, Present Position이 허용오차 안에 들면 OK |
| TC-GRP-006-01 | 파지 판정 | U | `[ON]` | 간격·전류 조합(경계 3점씩) | TC-ORC-003-04·05와 같은 결과 |
| TC-GRP-007-01 | 응답 제한시간 | U | `[ON]` | 응답 0.500 / 0.501 s, 무응답 | OK / `FAULT`(`GRIPPER_TIMEOUT`) / `FAULT`. steady 시계 사용 |
| TC-GRP-008-01 | 오류 비트 해석 | U | `[ON]` | Hardware Error Status 비트별 | reason에 비트 이름 포함, `FAULT` |
| TC-GRP-009-01 | 지연 계측 기록 | U | `[ON]` | open 1회 | `t_cmd`, `t_ack`, `t_motion_start`(Present Position 첫 변화), `t_done`이 단조 증가로 기록 |
| TC-GRP-010-01 | 그리퍼 transport는 adapter만 쓴다 | U | `[ON]` | 소스 정적 검사 | `GripperTransport` import가 `gripper_adapter.py`와 테스트 외 0건 |
| TC-GRP-011-01 | Modbus-RTU 프레임 | U | `[ON]` | CRC 표준 벡터 `01 03 00 00 00 01` / FC06 Torque Enable=1(slave 1, 주소 256) / FC16 Goal Position=300(주소 282, 2워드) / FC03 Present Current(주소 287, 1워드) | CRC `84 0A` / `01 06 01 00 00 01 49 F6` / `01 10 01 1A 00 02 04 01 2C 00 00 BF 79` / `01 03 01 1F 00 01 B4 30` |
| TC-GRP-012-01 | 제어기 레지스터 값 인코딩 | U | `[ON]` | 7비트 분할 인코딩(참고 구현의 레지스터 전달 특성 `(x & 0xFF) << 24` 대응) 0~16383 전수 | 인코딩 → DRL 쪽 복원 왕복 일치. 범위 밖은 예외 |
| TC-GRP-012-02 | 트리거 순서 | U | `[ON]` | `open()` 호출 | 목표값 레지스터 쓰기가 모두 끝난 뒤 트리거(seq) 레지스터를 마지막에 1회 씀. RELEASE 전에 목표값을 미리 쓴 경우 해제 순간 쓰기는 트리거 1회뿐 |
| TC-GRP-013-01 | 상주 루프 생존 확인 | U | `[ON]` | 생존 카운터가 0.100 / 0.101 s 동안 변하지 않음 | 유지 / `FAULT`(`GRIPPER_LOOP_DEAD`) |
| TC-GRP-013-02 | 루프 기동 조건 | U | `[ON]` | 로봇 상태 STANDBY 아님에서 기동 요청 | `drl_start` 호출 0회, `PRECONDITION` |

### 3.10 CAL — 보정

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-CAL-001-01 | 평면 맞춤 | U | `[ON]` | 잡음 0.2 mm 점 200개 + 이상점 10%, 시드 20개 / 한 직선 위 점 | 법선 오차 ≤ 0.1°, 높이(offset) 오차 ≤ 0.1 mm, 법선 z > 0 / 예외(`DEGENERATE`) |
| TC-CAL-002-01 | 벨트 진행 방향 | U | `[ON]` | 직선 이동 점 90개(0.3 m), 잡음 0.3 mm, 임의 yaw. 높이가 변하는 궤적 + 평면 | 부호 포함 방향 오차 ≤ 0.1° / 평면 위 방향만 냄 |
| TC-CAL-002-02 | 보드 이동 영상으로 진행 방향 | U | `[ON]` | 벨트 위 보드 16 cm 이동 6장면, 픽셀 잡음 0.2 px | 방향 오차 ≤ 0.2° |
| TC-CAL-003-01 | eye-to-hand(3D-3D) | U | `[ON]` | 알려진 변환 + 잡음 축별 σ 0.3 mm, 대응점 20개(벨트·매트·받침 세 높이) | 작업면 점 30개 오차 최대 ≤ 0.5 mm, 회전 오차 ≤ 0.1°, 카메라 원점 이동 오차 ≤ 1 mm(참고), rms 반환 |
| TC-CAL-003-02 | 퇴화 배치 거부 | U | `[ON]` | 한 직선 위 대응점(3D-3D·PnP), PnP 점 5개 | 예외(`DEGENERATE`) |
| TC-CAL-003-03 | eye-to-hand(PnP, 접촉점 + 픽셀) | U | `[ON]` | D455 공장 내부 파라미터, 24점(세 높이), 픽셀 잡음 0.3 px | 재투영 rms < 0.5 px, 벨트면 검증 30점 XY 최대 ≤ 0.5 mm |
| TC-CAL-003-04 | ChArUco + TCP 접촉 세션 | U | `[ON]` | SPEC-600 4.4절 배치(면마다 fit 4장면, 접촉 4점 σ 0.2 mm, verify 5장면 20점), 픽셀 잡음 0.2 px / 한 접촉점 4 mm 오류 / 합성 영상 17장으로 `calibrate_camera` 실행 | 검증 게이트 합격, 회전 ≤ 0.2°(참고), 벨트·매트 높이 오차 ≤ 0.3 mm / 그 장면 `bad_touch_views`에 표시 / 결과 파일 5종 생성, 종료 코드 0 |
| TC-CAL-004-01 | TCP 오프셋 추정 | U | `[ON]` | 알려진 오프셋 + 잡음 0.2 mm, 30자세: 기울기 ±30° 3축 / 수직 접근 yaw만 + XY 관측 + z 고정 | 오차 ≤ 0.2 mm / XY 오차 ≤ 0.2 mm, z 미고정 시 `DEGENERATE` |
| TC-CAL-005-01 | 보정 파일 필수 항목 | U | `[ON]` | 항목 누락 파일, 빈 값, 프로파일 내부 항목 누락, 단위가 아닌 quaternion | 로드 실패(calibration_id·날짜·잔차·카메라 프로파일·transform), 정상 파일 왕복 일치 |
| TC-CAL-005-02 | calibration_id 불일치 | C | `[ON]` | vision과 orchestrator의 id 다름 | PRECHECK FAILED(`CALIBRATION_MISMATCH`) |

### 3.11 MCK — 대역

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-MCK-001-01 | 결정성 | U | `[ON]` | 같은 시나리오·seed 2회 | 출력 열 해시 동일 |
| TC-MCK-002-01 | 역할 분리 일관성 | C | `[ON]` | `role:=vision`, `role:=tracking` 별도 프로세스 | 같은 시각의 뚜껑 위치 차 ≤ 1e-9 m |
| TC-MCK-003-01 | mock_world 계약 | C | `[ON]` | vision·tracker 계약 시험 | 통과 |
| TC-MCK-004-01 | mock_robot_controller 계약 | C | `[ON]` | robot_controller 계약 시험 | 통과 |
| TC-MCK-005-01 | 장애 주입 시각 | C | `[ON]` | 시나리오 이벤트 10종 | 지정 시각 ± 1 프레임에 발생 |
| TC-MCK-006-01 | 가정값 표기 | U | `[ON]` | 시나리오 파일 | 실측이 아닌 값이 `assumed:` 목록에 있고 trial 기록 `assumed_params`로 복사 |
| TC-MCK-007-01 | 실제·대역 동시 기동 금지 | I0 | `[ON]` | `use_mock_vision:=true`와 실제 vision 동시 지정 | bringup이 기동 거부 |

### 3.12 REC — 기록·분석

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-CAL-006-01 | 광선-평면 교차·검증 통계 | U | `[ON]` | 오차 없는 투영 50점(시야 가장자리 5 px까지) / XY 오차 0~1.9 mm 20점, z 차이 10 mm / 10점 | 왕복 오차 ≤ 1 µm / p95·최대 일치, 불합격(p95 > 1 mm), 절반 크기면 합격, z 무시 / 불합격(n < 20) |
| TC-CAL-007-01 | ChArUco 검출·자세 | U | `[ON]` | 원근 렌더(4배 초표본) 보드 5자세, 거리 0.37 m, 기울기 ≤ 20° | 코너 24개, 코너 오차 ≤ 0.2 px, 자세 이동 ≤ 1 mm·회전 ≤ 0.2°, 보드 좌표가 인쇄 원본 배치와 일치 |
| TC-CAL-008-01 | camera_link static TF 합성 | U | `[ON]` | 보정 결과 + RealSense형 camera_link ← 광학 TF | 다시 합성한 base ← 광학이 보정 결과와 1e-12 이내, det = 1 |
| TC-CAL-009-01 | CameraInfo ↔ 프로파일 비교 | U | `[ON]` | 같은 값, fx +0.4 px / 높이 720, 왜곡 모델 다름, cx +0.6 px, k3 +0.01, 왜곡 0 / fx·fy +2.2 px | 불일치 없음 / 해당 항목 불일치 / fx·fy만 불일치, `check_focal=False`면 없음 |
| TC-REC-001-01 | 시도 기록 스키마 | U | `[ON]` | `PLAN.md` 10.2절 JSON Schema로 검증 | 필수 필드 누락 시 실패 |
| TC-REC-002-01 | 시도하지 않은 경우도 기록 | I0 | `[ON]` | `time_insufficient` 소진 | `result = NOT_ATTEMPTED`, reason, `passes_skipped` 기록 |
| TC-REC-003-01 | 단계 기록 | I0 | `[ON]` | 정상 1사이클 | 단계 수만큼 StepRecord JSONL, metric 키 포함 |
| TC-REC-004-01 | Wilson 95% 구간 | U | `[ON]` | k/n = 45/50, 50/50, 0/20 | [0.7864, 0.9565], [0.9287, 1.0000], [0.0000, 0.1611](±1e-4) |
| TC-REC-004-02 | UNKNOWN 처리 | U | `[ON]` | SUCCESS 40, FAILURE 5, UNKNOWN 5 | 성공률 40/50(UNKNOWN은 분모에만) |
| TC-REC-005-01 | 분위수 보고 | U | `[ON]` | 알려진 표본 | p50/p95/p99/최대값이 numpy `method="linear"`와 일치 |
| TC-REC-006-01 | 추적 행렬 | U | `[ON]` | TC 없는 REQ 1개, 테스트 없는 TC 1개 | 두 목록 출력, 종료 코드 ≠ 0 |

### 3.13 INT — ROS 통합

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-INT-001-01 | I0 정상 | I0 | `[ON]` | `nominal_20mms`, 20회 | 20/20 SUCCESS(mock 판정), 상태 열 TC-ORC-001-01과 같음 |
| TC-INT-002-01 | 로봇 무응답 | I0 | `[ON]` | `robot_hang` | cancel 0.5 s 무응답 → 승격, 정의된 등급 |
| TC-INT-003-01 | 보호정지 | I0 | `[ON]` | `protective_stop` | supervisor → `Stop` → WAIT_OPERATOR, 자동 재개 0회 |
| TC-INT-004-01 | heartbeat 손실 | I0 | `[ON]` | vision·tracker·controller 각각 끊음 | L2/L3/L4(TC-ABT-010-01과 같음) |
| TC-INT-005-01 | 시간 부족 | I0 | `[ON]` | 첫 통과 부족 → 다음 통과 / 전부 부족 | 두 번째 통과에서 진행 / `NOT_ATTEMPTED` |
| TC-INT-006-01 | lifecycle 비활성 | I0 | `[ON]` | 노드 하나 `INACTIVE` | PRECHECK FAILED, skill 호출 0회 |
| TC-INT-007-01 | replay는 지령 경로가 없다 | I1 | `[ON]` | `replay.launch.py` | 스트림 토픽 발행자·robot action 클라이언트·그리퍼 호출 0개 |
| TC-INT-020-01 | I3 실기 skill·감시 | I3 | `[OFF]` | 안전 높이: deadline 자체 중단, stale 지령, `Stop`, plan 갱신 중단, 추종 중 그리퍼 개방 | 각 20회, 정의된 동작 20/20, 예기치 않은 재개 0회 |
| TC-INT-021-01 | I4 인식 경로 | I4 | `[OFF]` | 실영상 → 계획, 로봇 mock | stale·추적 손실·계획 유효성 판정이 기대표와 일치 |

### 3.14 SIM — 시뮬레이션

| TC | 검증 내용(REQ 초안) | 수준 | 장소 | 조건 | 기대 결과 |
|---|---|---|---|---|---|
| TC-SIM-001-01 | sim GT는 제어에 쓰지 않는다 | S | `[ON]` | 그래프 조사 | GT 토픽 구독자는 평가 도구뿐 |
| TC-SIM-002-01 | `use_sim_time` 일괄 적용 | S | `[ON]` | `simulation.launch.py` | 모든 노드 `use_sim_time = true` |
| TC-SIM-003-01 | 병뚜껑은 벨트와 함께 움직인다 | S | `[ON]` | 벨트 표면 속도 20 mm/s | 뚜껑 속도 오차 ≤ 1% |
| TC-SIM-004-01 | 접촉 수렴 | S | `[ON]` | 120 / 240 Hz, 정지 뚜껑 낙하 50회씩 | 안착 비율 차 ≤ 5%p, 최대 관통 < 공깃돌 높이의 10% |

### 3.15 BEN — 현장 측정 절차 (전부 `[OFF]`)

| TC | 측정 내용 | 세션 | 절차 | 기록 | 합격 기준(제안) |
|---|---|---|---|---|---|
| TC-BEN-001-01 | 현장 실측 | OFF-1 | `Progress.md` 0절 항목, 항목마다 3회 측정 | `site-survey-<date>.csv` | 모든 항목 기록, 3회 범위 공개 |
| TC-BEN-002-01 | 벨트 기준 속도 | OFF-1, OFF-5 | 다이얼 설정마다 병뚜껑 한 바퀴 시간 10회 | `belt-speed-<date>.csv` | 설정별 평균·표준편차, 맥동 공개 |
| TC-BEN-003-01 | pulse ↔ 간격 실측표 | OFF-1 | 하드웨어 Goal Position 0~1150 pulse(참고 구현 서비스의 0~750 반전 규약이 아닌 하드웨어 값)를 100 pulse 간격 + 공깃돌 지름 ±10 mm 구간은 25 pulse 간격, 닫는 방향 3회·여는 방향 1회 캘리퍼 측정(`tools/bench/gripper_pulse_table.py`) | `gripper-table-<date>.csv` | 단조, 히스테리시스 공개, 0 pulse가 개방인지 확인 |
| TC-BEN-004-01 | 스트림 성능 | OFF-2 | 안전 높이, 진폭 10 mm·0.5 Hz 사인 60 s, 후보 인터페이스 × 주기(50/100 Hz), 비전·rosbag 부하 포함 | `stream-<date>.csv` | 지령 간격 p99 ≤ 1.5 × 주기, 지연 p50/p95/p99 보고 |
| TC-BEN-005-01 | 정지 동작 | OFF-2 | 통신 단절·stale·cancel 모사 각 20회 | `stop-<date>.csv` | 20/20 정지, 예기치 않은 재개 0회 |
| TC-BEN-006-01 | 그리퍼 지연(정지 상태) | OFF-2 | 개방(`Δ_open`)·닫기 각 100회 | `gripper-<date>.csv` | `t_cmd → t_motion_start`, `→ t_done` p50/p95/p99 보고 |
| TC-BEN-007-01 | 상주 루프 + 스트림 동시 동작 | OFF-2 | 상주 루프를 띄운 상태에서 TC-BEN-004 조건 반복, 추종 중 개방 트리거 20회 | `gripper-stream-<date>.csv` | 스트림 지령 간격 최대값 변화, 추종 오차 p95 증가 ≤ 0.5 mm, 개방 지연이 정지 상태 p95의 1.2배 이내. 미달 시 `PLAN.md` 11.3절 |
| TC-BEN-008-01 | 파지 전류 | OFF-2 | 전류를 단계적으로 올리며 계획 최대 가속으로 10회 왕복, 미끄러짐(TCP 오프셋 변화 > 0.5 mm) 확인 | `grip-current-<date>.csv` | 미끄러짐 0인 최소 전류 × 1.5를 설정값으로 |
| TC-BEN-009-01 | 낙하 튕김 | OFF-1(예비), OFF-4 | 해제 높이 2~3단계 × 의도 오차 0/1/2/3 mm × 10회 이상, 정지 뚜껑 | `drop-matrix-<date>.csv` | 조건별 안착률·Wilson 구간, 안착률 ≥ 95%인 최대 오차로 `e_xy_max` 확정 |
| TC-BEN-010-01 | 가림 시간 | OFF-5 | 대기·접근·하강 동작을 뚜껑 위에서 재현, 검출 누락 구간 측정 | `occlusion-<date>.csv` | 단계별 가림 시간 분포, τ_max와 비교 |
| TC-BEN-011-01 | 지연 구성 | OFF-5 | 단계별 시각 기록 + 고속 촬영(240 fps 이상)으로 공깃돌 이탈 시각 확인 | `latency-chain-<date>.csv` | 노출 → 공깃돌 이탈까지 구성 성분별 p50/p95 |
| TC-BEN-012-01 | 보정 검증 | OFF-3 | 보정에 쓰지 않은 20개 이상 위치에 TCP 접촉 | `calib-verify-<date>.csv` | XY p95 ≤ 1 mm, 최대 ≤ 2 mm |

### 3.16 EXP — 게이트 실험 (전부 `[OFF]`)

공통 절차:

- 시도 전: 병뚜껑이 비었는지, 공깃돌이 매트 ROI 안 무작위 위치(사전에 만든 위치표 순서)에 있는지 확인한다.
- 조건(속도·모드) 순서는 블록 무작위로 섞어 시간에 따른 변화(조명·발열)가 한 조건에 몰리지 않게 한다.
- 관찰자는 자동 판정 결과를 보지 않고 판독한다. 판독 불가는 `UNKNOWN`으로 기록한다.
- 중단·NOT_ATTEMPTED도 시도 수에 포함해 기록한다. 시도를 다시 하려고 기록을 지우지 않는다.

| TC | 게이트 | 조건 | N | 합격 기준 |
|---|---|---|---|---|
| TC-EXP-G2-01 | G2 | MODE_STATIC, 정지 벨트 | 파지 50, 투입 50 | `Progress.md` G2 |
| TC-EXP-G3-01 | G3 | 속도 3종 × 60 s 관측 | 조건당 1회 이상 | `Progress.md` G3 |
| TC-EXP-G4-01 | G4 | MODE_PREDICTED, 저속 | 30 | `Progress.md` G4 |
| TC-EXP-G5-01 | G5 | 비접촉 추종, 속도 3종 × 60 s | 조건당 1회 이상 | `Progress.md` G5 |
| TC-EXP-G6-01 | G6 | MODE_SYNCHRONIZED, 저·중·고 | 속도당 50 | `Progress.md` G6 |
| TC-EXP-G7-01 | G7 | 통합 + 고장 주입 | 100 사이클 | `Progress.md` G7 |
| TC-EXP-CMP-01 | 13절 | Static Target / Predicted / Moving Frame, 같은 속도·조명 | 방식당 30 이상 | 같은 실패 정의로 성공률·Wilson 구간·해제 오차 분포 비교 |

## 4. 게이트별 필수 시험 묶음

| 게이트 | `[ON]` 필수(모두 통과) | `[ON*]` 필수 | `[OFF]` 필수 |
|---|---|---|---|
| M0 | SYS 전부, IF 전부, ORC 전부, ABT 전부, MCK 전부, REC 전부, INT-001~007 | — | — |
| G0 | ROB-001~012, GRP 전부(실측표 전에는 선형 표) | GRP-001(OFF-1 실측표로 재실행) | BEN-001, 003~008, INT-020 |
| G1 | VIS-001~008, CAL 전부 | VIS-010 | BEN-012 |
| G2 | ORC(MODE_STATIC 경로), VIS-009-01 | VIS-009-02 | BEN-009, EXP-G2 |
| G3 | TRK-001~012 | TRK-013 | BEN-002, 010, INT-021, EXP-G3 |
| G4 | PLN 전부 | — | BEN-011, EXP-G4 |
| G5 | ROB-006, 008~010 | — | EXP-G5 |
| G6 | ROB 전부, PLN 전부 | — | EXP-G6 |
| G7 | 위 전부 + SIM(선택) | 위 전부 | EXP-G7, EXP-CMP |

- 앞 게이트의 `[ON]` 시험은 뒤 게이트에서도 계속 통과해야 한다(CI 회귀).
- 시험 케이스를 추가·삭제하면 이 표와 `tools/spec/trace.py` 결과를 같은 커밋에서 맞춘다.
