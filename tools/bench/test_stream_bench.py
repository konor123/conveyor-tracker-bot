"""stream_bench.py의 순수 함수 단위 시험(순기구학, 사인 궤적, 지연·오차 분석)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import stream_bench as sb  # noqa: E402

URDF = """<robot name="t">
  <link name="base_link"/><link name="l1"/><link name="l2"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="f" type="fixed"><parent link="l2"/><child link="tip"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/></joint>
</robot>"""


@pytest.mark.level('U')
def test_fk_planar_two_link():
    ch = sb.UrdfChain(URDF, 'base_link', 'tip')
    assert ch.names() == ['j1', 'j2']
    p = ch.position({'j1': math.pi / 2, 'j2': -math.pi / 2})
    assert p == pytest.approx([0.3, 0.2, 0.1], abs=1e-12)


@pytest.mark.level('U')
def test_fk_rejects_missing_chain():
    with pytest.raises(ValueError):
        sb.UrdfChain(URDF, 'base_link', 'nowhere')


@pytest.mark.level('U')
def test_sine_ramp_starts_at_rest_and_reaches_amplitude():
    d0, v0 = sb.sine_target(0.0, 0.01, 0.5, 2.0)
    assert d0 == 0.0 and v0 == 0.0
    d, v = sb.sine_target(2.5, 0.01, 0.5, 2.0)
    assert d == pytest.approx(0.01 * math.sin(math.pi * 2.5))
    assert v == pytest.approx(0.01 * math.pi * math.cos(math.pi * 2.5))


@pytest.mark.level('U')
def test_analyze_recovers_known_lag_and_intervals():
    period, lag = 0.02, 0.085
    t_cmd = np.arange(0, 20, period)
    x_cmd = 0.01 * np.sin(np.pi * t_cmd)
    t_fb = np.arange(0.005, 20, 0.01)
    x_fb = 0.01 * np.sin(np.pi * (t_fb - lag))
    rows = [{'kind': 'cmd', 't_steady': t, 'x_m': x} for t, x in zip(t_cmd, x_cmd)]
    rows += [{'kind': 'fb', 't_steady': t, 'x_m': x} for t, x in zip(t_fb, x_fb)]
    s = sb.analyze(rows, period, ramp_s=2.0)
    assert s['lag_s'] == pytest.approx(lag, abs=0.001)
    assert s['cmd_interval_s']['p99'] == pytest.approx(period)
    assert s['cmd_interval_p99_ok']
    assert s['tracking_error_lag_comp_m']['max'] < 5e-5 < s['tracking_error_raw_m']['max']


@pytest.mark.level('U')
def test_sine_ramps_down_to_rest_at_center():
    T = 60.0
    d, v = sb.sine_target(T, 0.01, 0.5, 2.0, T)
    assert d == 0.0 and v == 0.0
    d, v = sb.sine_target(T - 1e-6, 0.01, 0.5, 2.0, T)
    assert abs(d) < 1e-8 and abs(v) < 1e-4
    d, _ = sb.sine_target(T - 2.5, 0.01, 0.5, 2.0, T)                 # ramp 전에는 원래 진폭
    assert d == pytest.approx(0.01 * math.sin(math.pi * (T - 2.5)))


@pytest.mark.level('U')
def test_stop_metrics_on_synthetic_decel():
    import stop_bench as stb
    t = np.arange(0, 2, 0.01)
    t_last, v0, a = 0.5, 0.02, 0.2                      # 0.5 s에 지령 끊김, 0.1 s 뒤 감속 시작
    x = np.where(t < 0.6, v0 * t, v0 * 0.6 + v0 * np.minimum(t - 0.6, 0.1) - 0.5 * a * np.minimum(t - 0.6, 0.1) ** 2)
    t_stop, d = stb.stop_metrics(t, x, t_last)
    assert t_stop == pytest.approx(0.2, abs=0.02)
    assert d == pytest.approx(v0 * 0.1 + v0 * 0.1 - 0.5 * a * 0.01, abs=2e-4)
