"""시나리오 엔진: 시나리오 YAML 하나로 정의한 단일 세계 모델.

ROS에 의존하지 않는다. 시각은 시나리오 시각 t [s](t = t_ros − t0_s)이고, 프레임 출력은
(seed, 프레임 k, 대상)에서 만든 난수로만 정해지므로 역할별 프로세스가 같은 값을 낸다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# drop_interfaces 상수(ObjectDetection, TrackedObject, 공통 결과 코드)
REGION_UNKNOWN, REGION_MAT, REGION_BELT = 0, 1, 2
CLASS_STONE, CLASS_CAP = 1, 2
OBSERVED, COASTING, LOST = 0, 1, 2
RESULT = {'OK': 0, 'FAILED': 1, 'TIMEOUT': 2, 'PRECONDITION': 3, 'CANCELED': 4, 'FAULT': 5}
VERDICT = {'SUCCESS': 0, 'FAILURE': 1, 'UNKNOWN': 2}
MODE_STATIC, MODE_PREDICTED, MODE_SYNCHRONIZED = 0, 1, 2

EVENT_TYPES = ('occlusion', 'track_lost', 'epoch_increment', 'speed_change', 'frame_drop',
               'heartbeat_stop', 'clock_jump', 'stone_in_cap', 'protective_stop', 'gripper_delay')
ROLES = ('vision', 'tracking', 'robot_controller')
SKILLS = ('PickStone', 'MoveToPose', 'PlaceStone', 'DynamicPlace', 'SafeRetreat')
GRIPPER_SKILLS = ('PickStone', 'PlaceStone', 'DynamicPlace')

_SKILL_DEFAULTS = {
    'duration_s': 1.0, 'result': 'OK', 'reason_code': '', 'hang': False, 'cancel_ack': True,
    'protective_stop_at_ratio': None,
    'grip_width_m': 0.0175, 'grip_current_ma': 150.0, 'pose_error_m': 0.0005,
    'release_error_m': 0.0005, 'gate_e_xy_m': 0.001, 'gate_v_rel_mps': 0.002,
    'gate_t_since_obs_s': 0.05, 'gate_position_std_m': 0.0003,
}
PHASE_METRIC_DEFAULTS = {
    'plan_valid': 1.0, 'pose_error_m': 0.0005, 'e_xy_m': 0.001, 'v_rel_mps': 0.002,
    'entry_time_error_s': 0.01, 'sync_ok': 1.0, 'rim_clearance_m': 0.0025,
    'height_error_m': 0.0002, 'target_error_m': 0.0005, 'open_width_m': 0.022,
    'sync_held': 1.0, 'tcp_z_m': 0.12, 'outside_workspace': 1.0, 'stone_in_gripper': 0.0,
}
PHASE_METRIC_KEYS = {
    ('DynamicPlace', 'APPROACH', False): ('plan_valid', 'pose_error_m'),
    ('DynamicPlace', 'APPROACH', True): ('plan_valid', 'pose_error_m'),
    ('DynamicPlace', 'SYNCHRONIZE', False): ('e_xy_m', 'v_rel_mps'),
    ('DynamicPlace', 'SYNCHRONIZE', True): ('entry_time_error_s', 'v_rel_mps'),
    ('DynamicPlace', 'DESCEND', False): ('sync_ok', 'rim_clearance_m', 'height_error_m'),
    ('DynamicPlace', 'DESCEND', True): ('sync_ok', 'rim_clearance_m', 'height_error_m'),
    ('DynamicPlace', 'RELEASE', False): ('open_width_m', 'sync_held'),
    ('DynamicPlace', 'RELEASE', True): ('open_width_m', 'sync_held'),
    ('DynamicPlace', 'RETREAT', False): ('tcp_z_m', 'outside_workspace', 'stone_in_gripper'),
    ('DynamicPlace', 'RETREAT', True): ('tcp_z_m', 'outside_workspace', 'stone_in_gripper'),
    ('PlaceStone', 'APPROACH', False): ('plan_valid', 'pose_error_m'),
    ('PlaceStone', 'DESCEND', False): ('target_error_m', 'rim_clearance_m', 'height_error_m'),
    ('PlaceStone', 'RELEASE', False): ('open_width_m', 'sync_held'),
    ('PlaceStone', 'RETREAT', False): ('tcp_z_m', 'outside_workspace', 'stone_in_gripper'),
}
DEFAULT_TCP_POSE = (0.40, 0.0, 0.20, 0.0, 0.0, 0.0, 1.0)
_STREAM = {'cap': 1, 'stone': 2, 'drop': 3}


@dataclass(frozen=True)
class Detection:
    region: int
    object_class: int
    object_index: int               # 시나리오의 caps/stones 순번
    u: float
    v: float
    diameter_px: float
    center_cov_px: tuple
    position: tuple                 # robot_base [m]
    position_cov: tuple             # 3x3 row-major [m^2]
    stone_inside: bool = False
    stone_inside_valid: bool = False


@dataclass(frozen=True)
class Track:
    track_id: int
    epoch: int
    status: int
    position: tuple                 # conveyor_frame [m]
    velocity: tuple                 # conveyor_frame [m/s]
    state_cov: tuple                # [x, y, vx, vy] 4x4 row-major
    last_observed_t: float          # 시나리오 시각 [s]
    observation_count: int
    valid_for_s: float


@dataclass(frozen=True)
class Belt:
    speed: float
    speed_std: float
    window_s: float
    steady: bool
    last_change_t: float | None


@dataclass(frozen=True)
class Frame:
    k: int
    t: float
    stamp_offset: float             # clock_jump 누적 [s]
    dropped: bool
    detections: tuple
    tracks: tuple
    belt: Belt


@dataclass(frozen=True)
class DetectReply:
    result: int
    reason_code: str
    detections: tuple
    position_std: tuple


@dataclass(frozen=True)
class SelectReply:
    result: int
    reason_code: str
    track_id: int = 0
    epoch: int = 0
    t_enter_window: float = 0.0     # 시나리오 시각 [s]


@dataclass(frozen=True)
class PlanReply:
    result: int
    reason_code: str
    status: int
    track_id: int = 0
    epoch: int = 0
    source_t: float = 0.0
    release_t: float = 0.0
    valid_until_t: float = 0.0
    target_xyz: tuple = (0.0, 0.0, 0.0)
    target_vel: tuple = (0.0, 0.0, 0.0)
    position_std: float = 0.0
    time_margin_s: float = 0.0


@dataclass(frozen=True)
class VerifyReply:
    verdict: int
    stone_inside: bool
    stone_offset: float
    stone_offset_valid: bool


@dataclass(frozen=True)
class SkillReply:
    name: str
    duration_s: float
    result: int
    reason_code: str
    hang: bool
    cancel_ack: bool
    protective_stop_at_ratio: float | None
    values: dict = field(default_factory=dict)
    phase_metrics: dict = field(default_factory=dict)

    def metrics_for(self, phase: str, predicted: bool = False) -> tuple[list[str], list[float]]:
        keys = PHASE_METRIC_KEYS.get((self.name, phase, bool(predicted) if self.name == 'DynamicPlace' else False), ())
        overrides = self.phase_metrics.get(phase) or {}
        vals = [float(overrides[k]) if k in overrides else float(PHASE_METRIC_DEFAULTS[k]) for k in keys]
        return list(keys), vals


# ------------------------------------------------------------------ 가정값 분류
def _numeric_leaves(node, path=''):
    """(키 경로, 값) 목록. 목록 첨자는 경로에서 뺀다. 수치 목록(xyz_m 등)은 잎 하나로 센다."""
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _numeric_leaves(v, f'{path}.{k}' if path else k)
    elif isinstance(node, list):
        if node and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in node):
            yield path, node
        else:
            for x in node:
                yield from _numeric_leaves(x, path)
    elif isinstance(node, (int, float)) and not isinstance(node, bool):
        yield path, node


def classify_params(scenario: dict) -> tuple[dict[str, str], list[str]]:
    """world·robot 수치 값을 assumed/measured로 분류한다. (분류, 오류 목록)."""
    assumed = list(scenario.get('assumed') or [])
    measured = dict(scenario.get('measured') or {})
    out: dict[str, str] = {}
    errors: list[str] = []
    for section in ('world', 'robot'):
        for path, _ in _numeric_leaves(scenario.get(section) or {}, section):
            a = any(path == p or path.startswith(p + '.') for p in assumed)
            m = path in measured
            if a and m:
                errors.append(f'{path}: assumed와 measured에 모두 있음')
            elif not (a or m):
                errors.append(f'{path}: 분류 없음')
            else:
                out[path] = 'measured' if m else 'assumed'
    for p in measured:
        if p not in out:
            errors.append(f'{p}: measured 키가 수치 값이 아님')
    return out, errors


# ------------------------------------------------------------------ 엔진
def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def scenario_geometry(world: dict) -> dict:
    """시나리오 world.frames·world.zones → mock 세계와 같은 기하(실제 노드 파라미터·고정 TF용).

    T_base_conveyor: robot_base ← conveyor_frame row-major 16개, zones: conveyor_frame x 구간 [m].
    """
    (x, y, z), R = world['frames']['conveyor_xyz_m'], _rot(float(world['frames'].get('conveyor_yaw_rad', 0.0)))
    T = [float(R[0, 0]), float(R[0, 1]), 0.0, float(x), float(R[1, 0]), float(R[1, 1]), 0.0, float(y),
         0.0, 0.0, 1.0, float(z), 0.0, 0.0, 0.0, 1.0]
    return {'T_base_conveyor': T, 'zones': {k: float(v) for k, v in world['zones'].items()}}


class ScenarioEngine:
    def __init__(self, scenario: dict, seed: int | None = None):
        self.scenario = scenario
        self.name = scenario.get('name', '')
        self.seed = int(scenario.get('seed', 0) if seed is None or seed < 0 else seed)
        self.rate = float(scenario.get('frame_rate_hz', 30.0))
        w = scenario.get('world') or {}
        self._belt = w.get('belt') or {}
        self._zones = w['zones']
        fr = w['frames']
        self._conv_xyz = np.asarray(fr['conveyor_xyz_m'], float)
        self._conv_R = _rot(float(fr.get('conveyor_yaw_rad', 0.0)))
        self._mat_xyz = np.asarray(fr['mat_xyz_m'], float)
        self._mat_R = _rot(float(fr.get('mat_yaw_rad', 0.0)))
        cam = w['camera']
        self._cam = np.asarray(cam['xyz_m'], float)
        self._w = int(cam['width_count'])
        self._h = int(cam['height_count'])
        self._f = (self._w / 2.0) / math.tan(float(cam['hfov_rad']) / 2.0)
        self._pix_std = float(cam.get('pixel_std_ratio', 0.0))
        self._obj = w['objects']
        self._pos_std = float((w.get('noise') or {}).get('position_std_m', 0.0))
        self._valid_for = float((w.get('tracking') or {}).get('valid_for_s', 0.3))
        self._caps = list(w.get('caps') or [])
        self._stones = list(w.get('stones') or [])
        self._verify = list(w.get('verify') or [{}])
        r = scenario.get('robot') or {}
        self._scale = float(r.get('duration_scale_ratio', 1.0))
        self._skills = r.get('skills') or {}
        self._robot_status = r.get('status') or {}
        self.events = sorted((dict(e) for e in scenario.get('events') or []), key=lambda e: e['t_s'])
        for e in self.events:
            if e['type'] not in EVENT_TYPES:
                raise ValueError(f"알 수 없는 이벤트: {e['type']}")
        self._speed_steps = [(0.0, float(self._belt['speed_mps']))] + [
            (float(e['t_s']), float(e['speed_mps'])) for e in self.events if e['type'] == 'speed_change']
        _, errors = classify_params(scenario)
        if errors:
            raise ValueError('가정값 분류 오류: ' + '; '.join(errors))

    @classmethod
    def from_file(cls, path: str, seed: int | None = None) -> 'ScenarioEngine':
        import yaml
        with open(path, encoding='utf-8') as fh:
            return cls(yaml.safe_load(fh), seed)

    def assumed_params(self) -> list[str]:
        cls, _ = classify_params(self.scenario)
        return sorted(p for p, kind in cls.items() if kind == 'assumed')

    # -------------------------------------------------------------- 시각·이벤트
    def frame_index(self, t: float) -> int:
        return math.floor(t * self.rate + 1e-9)

    def frame_time(self, k: int) -> float:
        return k / self.rate

    def _active(self, etype, t, **match):
        for e in self.events:
            if e['type'] != etype or any(e.get(k) != v for k, v in match.items()):
                continue
            end = e['t_s'] + e['duration_s'] if 'duration_s' in e else math.inf
            if e['t_s'] <= t < end:
                return e
        return None

    def _count(self, etype, t):
        return sum(1 for e in self.events if e['type'] == etype and e['t_s'] <= t)

    def _rng(self, k, stream, index=0):
        return np.random.default_rng([self.seed, k + 1_000_000, _STREAM[stream], index])

    # -------------------------------------------------------------- 벨트
    def speed(self, t: float) -> float:
        v = self._speed_steps[0][1]
        for ts, vs in self._speed_steps:
            if ts <= t:
                v = vs
        return v

    def belt_s(self, t: float) -> float:
        """시각 0부터 t까지 벨트 이동량 [m]. t < 0은 초기 속도로 외삽."""
        if t <= 0.0:
            return self._speed_steps[0][1] * t
        s, prev_t, v = 0.0, 0.0, self._speed_steps[0][1]
        for ts, vs in self._speed_steps[1:]:
            if ts >= t:
                break
            s += v * (ts - prev_t)
            prev_t, v = ts, vs
        return s + v * (t - prev_t)

    def time_at_s(self, s_target: float) -> float:
        """belt_s(t) = s_target인 t. 속도 0 구간에서 닿지 않으면 inf."""
        v0 = self._speed_steps[0][1]
        if s_target <= 0.0:
            return s_target / v0 if v0 > 0 else (0.0 if s_target == 0 else -math.inf)
        s, prev_t, v = 0.0, 0.0, v0
        for ts, vs in self._speed_steps[1:]:
            seg = v * (ts - prev_t)
            if s + seg >= s_target and v > 0:
                return prev_t + (s_target - s) / v
            s += seg
            prev_t, v = ts, vs
        return prev_t + (s_target - s) / v if v > 0 else math.inf

    def _belt_state(self, t):
        changes = [e['t_s'] for e in self.events if e['type'] == 'speed_change' and e['t_s'] <= t]
        last = changes[-1] if changes else None
        hold = float(self._belt.get('unsteady_hold_s', 2.0))
        return Belt(speed=self.speed(t), speed_std=float(self._belt.get('speed_std_mps', 0.0)),
                    window_s=float(self._belt.get('window_s', 1.0)),
                    steady=last is None or t >= last + hold, last_change_t=last)

    # -------------------------------------------------------------- 병뚜껑
    def cap_belt_pose(self, i: int, t: float) -> tuple[int, float, float]:
        """(lap, x, y): conveyor_frame 벨트 좌표(참값)."""
        L = float(self._belt['lap_length_m'])
        u = float(self._caps[i]['x0_m']) + self.belt_s(t)
        lap = math.floor(u / L)
        return lap, u - lap * L, float(self._caps[i].get('y_m', 0.0))

    def _visible(self, x):
        return self._zones['obs_start_m'] <= x <= self._zones['obs_end_m']

    def _t_enter_view(self, i, lap):
        L = float(self._belt['lap_length_m'])
        return self.time_at_s(lap * L + self._zones['obs_start_m'] - float(self._caps[i]['x0_m']))

    def track_id(self, i: int, lap: int) -> int:
        return 100 * (lap + 2) + i + 1

    def epoch(self, t: float, resets: int = 0) -> int:
        return self._count('epoch_increment', t) + resets

    def conveyor_to_base(self, p) -> np.ndarray:
        p = np.asarray(p, float)
        xy = self._conv_R @ p[:2] + self._conv_xyz[:2]
        return np.array([xy[0], xy[1], self._conv_xyz[2] + p[2]])

    def base_to_conveyor(self, p) -> np.ndarray:
        p = np.asarray(p, float)
        xy = self._conv_R.T @ (p[:2] - self._conv_xyz[:2])
        return np.array([xy[0], xy[1], p[2] - self._conv_xyz[2]])

    # -------------------------------------------------------------- 프레임
    def _pixel(self, p_base, diameter, rng):
        dz = self._cam[2] - p_base[2]
        u = self._w / 2.0 + self._f * (p_base[0] - self._cam[0]) / dz
        v = self._h / 2.0 - self._f * (p_base[1] - self._cam[1]) / dz
        sd = self._pix_std
        if sd > 0:
            u, v = u + rng.normal(0, sd), v + rng.normal(0, sd)
        return u, v, self._f * diameter / dz, (sd * sd, 0.0, 0.0, sd * sd)

    def _pos_cov(self):
        s2 = self._pos_std ** 2
        return (s2, 0.0, 0.0, 0.0, s2, 0.0, 0.0, 0.0, 0.0)

    def _noise(self, rng):
        return rng.normal(0.0, self._pos_std, 2) if self._pos_std > 0 else np.zeros(2)

    def frame(self, k: int, resets: int = 0) -> Frame:
        t = self.frame_time(k)
        jump = sum(float(e['offset_s']) for e in self.events if e['type'] == 'clock_jump' and e['t_s'] <= t)
        drop = self._active('frame_drop', t)
        dropped = bool(drop) and self._rng(k, 'drop').random() < float(drop.get('drop_ratio', 1.0))
        belt = self._belt_state(t)
        occluded = self._active('occlusion', t) is not None
        lost = self._active('track_lost', t) is not None
        in_cap = self._active('stone_in_cap', t) is not None
        epoch = self.epoch(t, resets)
        dets, tracks = [], []
        for i, st in enumerate(self._stones):
            rng = self._rng(k, 'stone', i)
            xy = self._mat_R @ [float(st['x_m']), float(st['y_m'])] + self._mat_xyz[:2] + self._noise(rng)
            p = (xy[0], xy[1], self._mat_xyz[2] + float(self._obj['stone_height_m']))
            u, v, d, ccov = self._pixel(p, float(self._obj['stone_diameter_m']), rng)
            dets.append(Detection(REGION_MAT, CLASS_STONE, i, u, v, d, ccov, p, self._pos_cov()))
        v_belt = belt.speed
        sd = self._pos_std
        cov4 = (sd * sd, 0, 0, 0, 0, sd * sd, 0, 0, 0, 0, belt.speed_std ** 2, 0, 0, 0, 0, belt.speed_std ** 2)
        for i in range(len(self._caps)):
            lap, x, y = self.cap_belt_pose(i, t)
            if not self._visible(x):
                continue
            rng = self._rng(k, 'cap', i)
            n = self._noise(rng)
            p_conv = np.array([x + n[0], y + n[1], float(self._obj['cap_rim_height_m'])])
            t_enter = self._t_enter_view(i, lap)
            if not math.isfinite(t_enter):
                # 초기 속도 0인 벨트에서 처음부터 관측 구간 안: 진입 시각이 −inf다. 관측은 시나리오 시각 0부터 센다
                t_enter = 0.0
            if occluded:
                occ = self._active('occlusion', t)
                last_obs = max(t_enter, self.frame_time(self.frame_index(occ['t_s']) - 1))
            else:
                last_obs = t
                p_base = self.conveyor_to_base(p_conv)
                u, v, d, ccov = self._pixel(p_base, float(self._obj['cap_outer_diameter_m']), rng)
                dets.append(Detection(REGION_BELT, CLASS_CAP, i, u, v, d, ccov, tuple(p_base),
                                      self._pos_cov(), stone_inside=in_cap, stone_inside_valid=True))
            status = LOST if lost else (COASTING if occluded else OBSERVED)
            tracks.append(Track(
                track_id=self.track_id(i, lap), epoch=epoch, status=status, position=tuple(p_conv),
                velocity=(v_belt, 0.0, 0.0), state_cov=cov4, last_observed_t=last_obs,
                observation_count=max(1, math.floor((t - t_enter) * self.rate + 1e-9) + 1),
                valid_for_s=self._valid_for))
        return Frame(k=k, t=t, stamp_offset=jump, dropped=dropped, detections=tuple(dets),
                     tracks=tuple(tracks), belt=belt)

    # -------------------------------------------------------------- 서비스 응답
    def heartbeat_alive(self, role: str, t: float) -> bool:
        return self._active('heartbeat_stop', t, role=role) is None

    def detect(self, t: float, region: int, object_class: int, min_count: int) -> DetectReply:
        fr = self.frame(self.frame_index(t))
        dets = tuple(d for d in fr.detections if d.region == region and d.object_class == object_class)
        std = tuple(self._pos_std for _ in dets)
        if len(dets) < min_count:
            return DetectReply(RESULT['FAILED'], 'NOT_ENOUGH_OBJECTS', dets, std)
        return DetectReply(RESULT['OK'], '', dets, std)

    def select_target(self, t: float, min_observation_count: int, resets: int = 0) -> SelectReply:
        fr = self.frame(self.frame_index(t), resets)
        reach = self._zones['reach_start_m']
        best = None
        for tr in fr.tracks:
            x = tr.position[0]
            if tr.status != OBSERVED or x >= reach or tr.observation_count < min_observation_count:
                continue
            if best is None or x > best.position[0]:
                best = tr
        if best is None:
            return SelectReply(RESULT['FAILED'], 'NO_TARGET')
        v = fr.belt.speed
        t_enter = fr.t + (reach - best.position[0]) / v if v > 0 else math.inf
        return SelectReply(RESULT['OK'], '', best.track_id, best.epoch, t_enter)

    def compute_intercept(self, t: float, target_track_id: int, target_epoch: int,
                          resets: int = 0, t_required_s: float = 3.6) -> PlanReply:
        fr = self.frame(self.frame_index(t), resets)
        tr = next((x for x in fr.tracks if x.track_id == target_track_id), None)
        if tr is None or tr.status != OBSERVED:
            return PlanReply(RESULT['FAILED'], 'NO_TRACK', 2)
        if tr.epoch != target_epoch:
            return PlanReply(RESULT['FAILED'], 'EPOCH_MISMATCH', 4, tr.track_id, tr.epoch)
        v = fr.belt.speed
        x_exit = float(self._zones['reach_end_m'])
        t_avail = (x_exit - tr.position[0]) / v if v > 0 else 0.0
        margin = t_avail - t_required_s
        dt_rel = min(t_required_s, max(0.0, t_avail))
        x_rel = tr.position[0] + v * dt_rel
        p_base = tuple(float(c) for c in self.conveyor_to_base([x_rel, tr.position[1], tr.position[2]]))
        v_xy = self._conv_R @ [v, 0.0]
        v_base = (float(v_xy[0]), float(v_xy[1]), 0.0)
        valid_until_t = fr.t + max(t_avail, 5.0)
        if margin < 1.0 - 1e-9:
            return PlanReply(RESULT['FAILED'], 'TIME_INSUFFICIENT', 1, tr.track_id, tr.epoch,
                             fr.t, fr.t + dt_rel, valid_until_t, p_base, v_base, self._pos_std, margin)
        return PlanReply(RESULT['OK'], '', 0, tr.track_id, tr.epoch, fr.t, fr.t + dt_rel,
                         valid_until_t, p_base, v_base, self._pos_std, margin)

    def verify(self, call_index: int, scripted: bool = True) -> VerifyReply:
        """`world.verify` 대본의 판정. `scripted=False`(실제 controller와 함께 띄운 mock 세계)면 대본 대신 UNKNOWN:
        mock 세계는 실제 controller가 떨어뜨린 공깃돌의 위치를 모르므로 투입 여부를 지어내지 않는다."""
        if not scripted:
            return VerifyReply(VERDICT['UNKNOWN'], False, 0.0, False)
        e = self._verify[min(call_index, len(self._verify) - 1)] or {}
        verdict = VERDICT[e.get('verdict', 'SUCCESS')]
        inside = bool(e.get('stone_inside', verdict == VERDICT['SUCCESS']))
        offset = e.get('stone_offset_m', 0.002 if inside else None)
        return VerifyReply(verdict, inside, float(offset or 0.0), offset is not None)

    # -------------------------------------------------------------- 로봇
    def robot_status_pose(self) -> tuple[float, float, float, float, float, float, float]:
        s = self._robot_status
        if not s:
            return DEFAULT_TCP_POSE
        xyz = s.get('tcp_xyz_m') or s.get('xyz_m')
        if xyz is not None:
            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        else:
            x = float(s.get('x_m', s.get('x', DEFAULT_TCP_POSE[0])))
            y = float(s.get('y_m', s.get('y', DEFAULT_TCP_POSE[1])))
            z = float(s.get('z_m', s.get('z', DEFAULT_TCP_POSE[2])))
        quat = s.get('orientation_xyzw_ratio') or s.get('orientation')
        if quat is not None:
            qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        else:
            qx = float(s.get('qx_ratio', s.get('qx', 0.0)))
            qy = float(s.get('qy_ratio', s.get('qy', 0.0)))
            qz = float(s.get('qz_ratio', s.get('qz', 0.0)))
            qw = float(s.get('qw_ratio', s.get('qw', 1.0)))
        return (x, y, z, qx, qy, qz, qw)

    def skill(self, name: str, call_index: int) -> SkillReply:
        if name not in SKILLS:
            raise ValueError(f'알 수 없는 skill: {name}')
        seq = self._skills.get(name) or [{}]
        e = {**_SKILL_DEFAULTS, **(seq[min(call_index, len(seq) - 1)] or {})}
        values = {k: e[k] for k in _SKILL_DEFAULTS if k.endswith(('_m', '_ma', '_mps'))
                  or k == 'gate_t_since_obs_s'}
        for k in ('holding', 'released'):
            if k in e:
                values[k] = bool(e[k])
        raw_pm = e.get('phase_metrics') or {}
        phase_metrics = {
            str(ph): {str(mk): float(mv) for mk, mv in (md or {}).items()}
            for ph, md in raw_pm.items()
        }
        raw_dur = float(e['duration_s'])
        scaled_dur = raw_dur * self._scale
        if name == 'DynamicPlace' and raw_dur >= 1.8 and scaled_dur < 1.8:
            scaled_dur = 1.8
        return SkillReply(name=name, duration_s=scaled_dur,
                          result=RESULT[e['result']], reason_code=e['reason_code'],
                          hang=bool(e['hang']), cancel_ack=bool(e['cancel_ack']),
                          protective_stop_at_ratio=e['protective_stop_at_ratio'],
                          values=values, phase_metrics=phase_metrics)

    def protective_stop(self, t: float) -> bool:
        return self._active('protective_stop', t) is not None

    def gripper_delay(self, t: float) -> float:
        e = self._active('gripper_delay', t)
        return float(e['delay_s']) if e else 0.0
