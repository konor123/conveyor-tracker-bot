"""stream_bench.py를 Doosan 드라이버 대역에 붙여 끝까지 돌린다(SPEC-500 REQ-ROB-011, TEST_SPEC.md TC-ROB-011-02).

`mode:=virtual`(DRCF 에뮬레이터, docker)을 쓸 수 있는 PC가 없어 실제 `dsr_msgs2` 형식의 대역으로 대신한다.
대역은 스크립트가 쓰는 표면만 연다: `robot_description`(transient local), `joint_states` 100 Hz,
`system/get_robot_state`·`system/get_last_alarm`·`aux_control/get_current_posx`·`motion/move_stop`,
`speedl_stream`·`servol_stream` 구독. 팔은 기준 위치를 지령으로 움직이고(speedl은 속도 적분·`time` 동안 유지,
servol은 위치) 실제 위치를 기준의 `DELAY_S` 전 값으로 둔 순수 지연 모델이다. 관절값은 합성 6축 URDF의
수치 역기구학으로 낸다(E0509 기구학이 아님).

확인하지 않는 것: 제어기 수락·거부 조건, 실제 지연·지터·추종 오차, speedl 무지령 정지와 move_stop의 실제 정지,
E0509 URDF 순기구학과 posx의 일치(2026-11-07 실기 실행에서 확인), 현장 드라이버 형식과의 일치(OFF-2).

스크립트는 하위 프로세스로 띄우고, 시험 전용 도메인·namespace(`/bench_stub`)와 저장소 밖 임시 기록 폴더를 준다.
`dsr_msgs2`가 없으면 skip이다(CI·GCE). 판정은 `dsr_msgs2`가 있는 PC에서 한다.
"""
from __future__ import annotations

import bisect
import glob
import importlib.util
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import stream_bench as sb  # noqa: E402

HAVE_DSR = importlib.util.find_spec('rclpy') is not None and importlib.util.find_spec('dsr_msgs2') is not None
pytestmark = pytest.mark.skipif(not HAVE_DSR, reason='rclpy·dsr_msgs2 없음: Doosan 메시지 패키지가 있는 PC에서 판정한다. '
                                'virtual 모드는 DRCF 에뮬레이터(docker)가 필요하다(SPEC-500 4.6절)')

SCRIPT = os.path.join(os.path.dirname(__file__), 'stream_bench.py')
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
NS = '/bench_stub'
DOMAIN = 150 + os.getpid() % 50
DELAY_S = 0.050                    # 대역 팔의 순수 지연
TICK_S = 0.01                      # joint_states 100 Hz
ORIENT_DEG = (0.0, 180.0, 0.0)

# 합성 6축(base_link → link_6). 관절 1~3으로 위치를 맞추고 4~6은 고정한다.
URDF = """<robot name="bench_stub">
  <link name="base_link"/><link name="link_1"/><link name="link_2"/><link name="link_3"/>
  <link name="link_4"/><link name="link_5"/><link name="link_6"/>
  <joint name="joint_1" type="revolute"><parent link="base_link"/><child link="link_1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="joint_2" type="revolute"><parent link="link_1"/><child link="link_2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/></joint>
  <joint name="joint_3" type="revolute"><parent link="link_2"/><child link="link_3"/>
    <origin xyz="0 0 0.40" rpy="0 0 0"/><axis xyz="0 1 0"/></joint>
  <joint name="joint_4" type="revolute"><parent link="link_3"/><child link="link_4"/>
    <origin xyz="0.35 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/></joint>
  <joint name="joint_5" type="revolute"><parent link="link_4"/><child link="link_5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/></joint>
  <joint name="joint_6" type="revolute"><parent link="link_5"/><child link="link_6"/>
    <origin xyz="0 0 -0.10" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
</robot>"""
Q0 = {'joint_1': 0.0, 'joint_2': 0.6, 'joint_3': 0.9, 'joint_4': 0.0, 'joint_5': 0.0, 'joint_6': 0.0}


class BenchDriverStub:
    """stream_bench가 쓰는 Doosan 드라이버 표면의 대역(위 모듈 설명). 받은 지령·요청을 그대로 남긴다."""

    def __init__(self, robot_state: int = 1, drift_after_s: float | None = None):
        import rclpy
        from dsr_msgs2.msg import ServolStream, SpeedlStream
        from dsr_msgs2.srv import GetCurrentPosx, GetLastAlarm, GetRobotState, MoveStop
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        self._JointState, self._F64 = JointState, Float64MultiArray
        self.robot_state = robot_state
        self.drift_after_s = drift_after_s
        self.chain = sb.UrdfChain(URDF)
        self.q = dict(Q0)
        self._J0inv = self._jacobian_inverse()
        self.ref = self.chain.position(self.q)
        self.center = self.ref.copy()
        t_start = time.monotonic()
        self.hist_t = [t_start - 1.0, t_start]          # 기준 위치 표본(tick마다), 순수 지연 보간용
        self.hist_r = [self.ref.copy(), self.ref.copy()]
        self.v_cmd = np.zeros(3)
        self.t_cmd = time.monotonic()      # 지금 속도 지령을 받은 시각(그때까지 적분해 ref에 넣음)
        self.hold_until = 0.0
        self.first_cmd = None
        self.speedl, self.servol, self.move_stop, self.calls = [], [], [], []
        self._lock = threading.Lock()
        self.ctx = rclpy.Context()
        self.ctx.init(domain_id=DOMAIN)
        self.node = rclpy.create_node('bench_driver_stub', namespace=NS, context=self.ctx)
        n = self.node
        n.create_publisher(String, 'robot_description',
                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)).publish(String(data=URDF))
        self.js_pub = n.create_publisher(JointState, 'joint_states', 50)
        n.create_subscription(SpeedlStream, 'speedl_stream', self._on_speedl, 10)
        n.create_subscription(ServolStream, 'servol_stream', self._on_servol, 20)
        n.create_service(GetRobotState, 'system/get_robot_state', self._robot_state)
        n.create_service(GetLastAlarm, 'system/get_last_alarm', self._last_alarm)
        n.create_service(GetCurrentPosx, 'aux_control/get_current_posx', self._posx)
        n.create_service(MoveStop, 'motion/move_stop', self._move_stop)
        n.create_timer(TICK_S, self._tick)
        self.exec = SingleThreadedExecutor(context=self.ctx)
        self.exec.add_node(n)
        self._thread = threading.Thread(target=self.exec.spin, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ 지령
    def _ref_at(self, now):
        """받은 시각 기준 구간별 상수 속도의 정확한 적분: 마지막 지령의 속도를 `time` 동안 유지, 그 뒤 0."""
        return self.ref + self.v_cmd * max(0.0, min(now, self.hold_until) - self.t_cmd)

    def _set_velocity(self, now, v, hold_s):
        self.ref = self._ref_at(now)
        self.v_cmd, self.t_cmd, self.hold_until = v, now, now + hold_s

    def _on_speedl(self, m):
        with self._lock:
            now = time.monotonic()
            self.first_cmd = self.first_cmd or now
            self.speedl.append((now, list(m.vel), list(m.acc), float(m.time)))
            self._set_velocity(now, np.array(m.vel[:3]) / 1000.0, float(m.time))

    def _on_servol(self, m):
        with self._lock:
            now = time.monotonic()
            self.first_cmd = self.first_cmd or now
            self.servol.append((now, list(m.pos), list(m.vel), list(m.acc), float(m.time)))
            self._set_velocity(now, np.zeros(3), 0.0)
            self.ref = np.array(m.pos[:3]) / 1000.0

    def _move_stop(self, req, res):
        with self._lock:
            self.move_stop.append(int(req.stop_mode))
            self._set_velocity(time.monotonic(), np.zeros(3), 0.0)
        res.success = True
        return res

    # ------------------------------------------------------------ 상태
    def _robot_state(self, req, res):
        self.calls.append('get_robot_state')
        res.robot_state, res.success = self.robot_state, True
        return res

    def _last_alarm(self, req, res):
        self.calls.append('get_last_alarm')
        res.log_alarm.level, res.log_alarm.group, res.log_alarm.index = 0, 0, 0
        res.success = True
        return res

    def _posx(self, req, res):
        self.calls.append(f'get_current_posx(ref={req.ref})')
        with self._lock:
            p = self._actual(time.monotonic())
        res.task_pos_info = [self._F64(data=[*(p * 1000.0), *ORIENT_DEG, 2.0])]
        res.success = True
        return res

    def _actual(self, now):
        """기준 위치의 DELAY_S 전 값(순수 지연) + 표류 주입."""
        tq = now - DELAY_S
        i = min(max(bisect.bisect_right(self.hist_t, tq), 1), len(self.hist_t) - 1)
        t0, t1 = self.hist_t[i - 1], self.hist_t[i]
        w = min(max((tq - t0) / (t1 - t0), 0.0), 1.0) if t1 > t0 else 1.0
        p = (1.0 - w) * self.hist_r[i - 1] + w * self.hist_r[i]
        if self.drift_after_s is not None and self.first_cmd is not None and now - self.first_cmd > self.drift_after_s:
            p = p + np.array([0.0, 0.008, 0.0])          # 다른 축 8 mm: 스크립트 안전 감시(5 mm) 밖
        return p

    IK_JOINTS = ('joint_1', 'joint_2', 'joint_3')

    def _jacobian_inverse(self):
        """Q0에서 관절 1~3 위치 야코비안의 역행렬(한 번 계산)."""
        p0 = self.chain.position(Q0)
        J = np.zeros((3, 3))
        for j, name in enumerate(self.IK_JOINTS):
            dq = dict(Q0)
            dq[name] += 1e-7
            J[:, j] = (self.chain.position(dq) - p0) / 1e-7
        return np.linalg.inv(J)

    def _ik(self, target):
        """고정 야코비안 반복 1회(tick마다): 한 tick의 이동은 0.2 mm 미만이고 수 mm 범위에서 야코비안 변화가 작아
        직전 관절값에서 1회로 µm 단위까지 맞는다. tick을 가볍게 해 지령 구독이 GIL을 기다리지 않게 한다."""
        step = self._J0inv @ (target - self.chain.position(self.q))
        for j, name in enumerate(self.IK_JOINTS):
            self.q[name] += float(step[j])
        return dict(self.q)

    def _tick(self):
        with self._lock:
            now = time.monotonic()
            self.hist_t.append(now)
            self.hist_r.append(self._ref_at(now))
            if len(self.hist_t) > 400:
                del self.hist_t[:100], self.hist_r[:100]
            p = self._actual(now)
        q = self._ik(p)
        m = self._JointState(name=list(q), position=[q[k] for k in q])
        m.header.stamp = self.node.get_clock().now().to_msg()
        self.js_pub.publish(m)

    closed = False

    def close(self):
        self.closed = True
        self.exec.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        self.ctx.try_shutdown()


def _bench(tmp_path, *extra, timeout=60.0):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    before = set(glob.glob(os.path.join(REPO, 'records', 'bench', 'stream-*')))
    r = subprocess.run([sys.executable, SCRIPT, '--ns', NS, '--out-dir', str(tmp_path), *extra],
                       capture_output=True, text=True, timeout=timeout, env=env)
    assert set(glob.glob(os.path.join(REPO, 'records', 'bench', 'stream-*'))) == before   # 저장소에 쓰지 않음
    return r


def _summary(tmp_path) -> dict:
    files = glob.glob(os.path.join(str(tmp_path), 'stream-*.summary.txt'))
    assert len(files) == 1, files
    out = {}
    for ln in open(files[0], encoding='utf-8'):
        if ': ' in ln and not ln.startswith('#'):
            k, v = ln.rstrip('\n').split(': ', 1)
            out[k] = v
    return out


@pytest.fixture
def stub_factory():
    made = []

    def make(**kw):
        s = BenchDriverStub(**kw)
        made.append(s)
        time.sleep(0.3)
        return s
    yield make
    for s in made:
        if not s.closed:
            s.close()


@pytest.mark.req('REQ-ROB-011')
@pytest.mark.tc('TC-ROB-011-02')
@pytest.mark.level('C')
def test_dry_run_and_refusal_send_nothing(tmp_path, stub_factory):
    """dry-run은 사전 점검만 하고 끝난다. STANDBY가 아니면 지령 없이 거부(종료 코드 2), move_stop도 부르지 않는다."""
    s = stub_factory()
    r = _bench(tmp_path, '--iface', 'speedl', '--rate-hz', '100', '--dry-run')
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'dry-run' in r.stdout and 'FK−posx 차=0.00 mm' in r.stdout
    assert {'get_robot_state', 'get_last_alarm', 'get_current_posx(ref=0)'} <= set(s.calls)
    s.close()                                  # 같은 namespace에 대역 둘을 함께 두지 않는다
    s2 = stub_factory(robot_state=2)
    r = _bench(tmp_path, '--iface', 'speedl', '--rate-hz', '100')
    assert r.returncode == 2 and 'robot_state 2 ≠ 1(STANDBY)' in r.stdout, r.stdout + r.stderr
    for st in (s, s2):
        assert st.speedl == [] and st.servol == [] and st.move_stop == []
    assert glob.glob(os.path.join(str(tmp_path), 'stream-*')) == []


@pytest.mark.req('REQ-ROB-011')
@pytest.mark.tc('TC-ROB-011-02')
@pytest.mark.level('C')
def test_speedl_run_streams_stops_and_records(tmp_path, stub_factory):
    """speedl 100 Hz 6 s: 지령 형식(`vel` mm/s 6개·`acc` [mm/s², deg/s²]·`time` 2주기), 끝에 속도 0 지령 1회와
    `move_stop`(`stop_mode` 2), 임시 폴더에 CSV·요약. 중심 대비 움직임은 지령 축(x)만."""
    s = stub_factory()
    r = _bench(tmp_path, '--iface', 'speedl', '--rate-hz', '100', '--amp-m', '0.005', '--freq-hz', '0.5',
               '--duration-s', '6', '--ramp-s', '1')
    assert r.returncode == 0, r.stdout + r.stderr
    sm = _summary(tmp_path)
    assert sm['aborted_by_guard'] == 'False'
    n = int(sm['commands'])
    assert 540 <= n <= 610 and len(s.speedl) == n + 1                    # 지령 + 끝의 속도 0 1회
    for _, vel, acc, t in s.speedl[:-1]:
        assert len(vel) == 6 and vel[1:] == [0.0] * 5 and abs(vel[0]) <= 2 * math.pi * 0.5 * 5.0 + 1e-9
        assert acc == [200.0, 60.0] and t == pytest.approx(0.02)
    assert s.speedl[-1][1] == [0.0] * 6
    assert s.move_stop == [2]
    peak = max(abs(v[0]) for _, v, _, _ in s.speedl)
    assert peak == pytest.approx(2 * math.pi * 0.5 * 5.0, rel=0.02)     # 최대 속도 15.7 mm/s
    end = s._ref_at(time.monotonic())
    assert abs(end[1] - s.center[1]) < 1e-9 and abs(end[2] - s.center[2]) < 1e-9
    assert len(glob.glob(os.path.join(str(tmp_path), 'stream-*-speedl-100hz.csv'))) == 1


@pytest.mark.req('REQ-ROB-011')
@pytest.mark.tc('TC-ROB-011-02')
@pytest.mark.level('C')
@pytest.mark.realtime_pc
def test_speedl_analysis_recovers_injected_delay(tmp_path, stub_factory):
    """대역 팔의 순수 지연 50 ms를 분석이 찾는다. 속도 적분·100 Hz 표본 때문에 반 주기 안팎이 더해진다:
    추정 지연 50~70 ms, 지연 보정 뒤 추종 오차 p95 ≤ 0.2 mm, 지령 간격 p99 ≤ 1.5 × 주기."""
    stub_factory()
    r = _bench(tmp_path, '--iface', 'speedl', '--rate-hz', '100', '--amp-m', '0.005', '--freq-hz', '0.5',
               '--duration-s', '6', '--ramp-s', '1')
    assert r.returncode == 0, r.stdout + r.stderr
    sm = _summary(tmp_path)
    lag = float(sm['lag_s'])
    assert 0.050 <= lag <= 0.070, sm
    comp = eval(sm['tracking_error_lag_comp_m'], {})                   # noqa: S307 - 스크립트가 쓴 dict 문자열
    assert comp['p95'] <= 0.0002, sm
    assert sm['cmd_interval_p99_ok'] == 'True', sm


@pytest.mark.req('REQ-ROB-011')
@pytest.mark.tc('TC-ROB-011-02')
@pytest.mark.level('C')
def test_guard_abort_still_stops(tmp_path, stub_factory):
    """다른 축이 5 mm 넘게 벗어나면 안전 중단(종료 코드 3)하고도 속도 0 지령과 `move_stop`을 보낸다."""
    s = stub_factory(drift_after_s=1.5)
    r = _bench(tmp_path, '--iface', 'speedl', '--rate-hz', '100', '--amp-m', '0.005', '--freq-hz', '0.5',
               '--duration-s', '6', '--ramp-s', '1')
    assert r.returncode == 3 and '안전 중단' in r.stdout, r.stdout + r.stderr
    assert _summary(tmp_path)['aborted_by_guard'] == 'True'
    assert s.speedl[-1][1] == [0.0] * 6 and s.move_stop == [2]
    assert len(s.speedl) < 300                                             # 6 s 전체(600개)를 보내기 전에 멈춤


@pytest.mark.req('REQ-ROB-011')
@pytest.mark.tc('TC-ROB-011-02')
@pytest.mark.level('C')
def test_servol_run_message_format(tmp_path, stub_factory):
    """servol 50 Hz 3 s: `pos`는 중심 ± 진폭(mm, 자세는 posx 그대로), `vel` [50 mm/s, 30 deg/s], `acc` [200, 60],
    `time` 0. 끝에 `move_stop`(2)."""
    s = stub_factory()
    r = _bench(tmp_path, '--iface', 'servol', '--rate-hz', '50', '--amp-m', '0.005', '--freq-hz', '0.5',
               '--duration-s', '3', '--ramp-s', '1')
    assert r.returncode == 0, r.stdout + r.stderr
    c = s.center * 1000.0
    assert 140 <= len(s.servol) <= 152
    for _, pos, vel, acc, t in s.servol:
        assert len(pos) == 6 and abs(pos[0] - c[0]) <= 5.0 + 0.05
        assert pos[1] == pytest.approx(c[1], abs=0.05) and pos[2] == pytest.approx(c[2], abs=0.05)
        assert pos[3:] == list(ORIENT_DEG)
        assert vel == [50.0, 30.0] and acc == [200.0, 60.0] and t == 0.0
    assert s.speedl == [] and s.move_stop == [2]
