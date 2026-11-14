"""moving_drop.py 순수 함수 단위 시험."""
import numpy as np
import pytest

from tools.robot.moving_drop import arrival_estimate, crossing, judge, last_pass_model, move_time_s


def track(v=0.034, x0=0.57, t0=100.0, n=30, hz=25.0, y=0.085, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    return [(t0 + k / hz, x0 + v * k / hz + noise * rng.standard_normal(), y) for k in range(n)]


def test_constant_speed_gives_the_arrival_time_and_speed():
    pts = track()
    now = pts[-1][0] + 0.05
    e = arrival_estimate(pts, 0.66, 0.085, now)
    x_last = pts[-1][1]
    assert e['v'] == pytest.approx(0.034, abs=1e-6)
    assert e['t_arrive'] == pytest.approx(pts[-1][0] + (0.66 - x_last) / 0.034, abs=1e-4)


def test_noise_of_point_three_mm_keeps_the_arrival_within_fifty_ms():
    errs = []
    for seed in range(20):
        pts = track(noise=0.0003, seed=seed)
        e = arrival_estimate(pts, 0.66, 0.085, pts[-1][0] + 0.05)
        errs.append(e['t_arrive'] - (pts[0][0] + (0.66 - 0.57) / 0.034))
    assert max(abs(v) for v in errs) < 0.05


def test_too_few_stale_or_short_tracks_give_nothing():
    pts = track()
    assert arrival_estimate(pts[:5], 0.66, 0.085, pts[4][0] + 0.05) is None            # 검출 5개, 지난 속도 없음
    assert arrival_estimate(pts, 0.66, 0.085, pts[-1][0] + 0.5) is None                # 0.5 s 묵음
    assert arrival_estimate(track(n=8, x0=0.57), 0.66, 0.085, 100.0 + 8 / 25.0) is None    # 0.28 s 구간


def test_points_past_the_release_point_or_off_the_belt_line_are_ignored():
    pts = track()
    other = [(t, 0.75, 0.085) for t, _, _ in pts]              # 이미 지나간 뚜껑
    off = [(t, x, 0.14) for t, x, _ in pts]                    # 벨트 선 밖
    now = pts[-1][0] + 0.05
    assert arrival_estimate(pts + other, 0.66, 0.085, now)['v'] == pytest.approx(0.034, abs=1e-6)
    assert arrival_estimate(off, 0.66, 0.085, now) is None


def test_stop_time_freezes_the_fit_before_the_arm_blocks_the_view():
    pts = track()
    shifted = pts[:20] + [(t, x + 0.004, y) for t, x, y in pts[20:]]     # 가려지며 중심이 4 mm 밀림
    now = pts[-1][0] + 0.05
    clean = arrival_estimate(shifted, 0.66, 0.085, now, t_stop=pts[20][0] - 1e-6)
    assert clean['v'] == pytest.approx(0.034, abs=1e-6)
    assert clean['t_last'] == pytest.approx(pts[19][0])


def test_scattered_points_or_out_of_range_speed_give_nothing():
    assert arrival_estimate(track(noise=0.006), 0.66, 0.085, 101.2) is None
    assert arrival_estimate(track(v=0.004), 0.66, 0.085, 101.2) is None
    assert arrival_estimate(track(v=-0.03, x0=0.6), 0.66, 0.085, 101.2) is None


def test_first_detections_of_a_pass_use_the_previous_pass_speed():
    pts = track(n=4)
    e = arrival_estimate(pts, 0.66, 0.085, pts[-1][0] + 0.05, v_prior=0.034)
    assert e['prior'] is True and e['n'] == 4
    t_true = pts[0][0] + (0.66 - 0.57) / 0.034
    assert e['t_arrive'] == pytest.approx(t_true, abs=0.02)
    assert arrival_estimate(pts[:2], 0.66, 0.085, pts[1][0] + 0.05, v_prior=0.034) is None      # 검출 2개


def test_prior_speed_is_used_with_the_current_position_unless_the_belt_changed():
    pts = track(v=0.036, noise=0.0002, seed=3)                     # 이번 통과의 국소 속도는 36, 지난 통과 평균은 34
    now = pts[-1][0] + 0.05
    e = arrival_estimate(pts, 0.66, 0.085, now, v_prior=0.034)
    assert e['prior'] is True and e['v'] == pytest.approx(0.034)
    x_now = 0.57 + 0.036 * (len(pts) - 3) / 25.0                   # 마지막 다섯의 중앙
    assert e['t_arrive'] == pytest.approx(pts[-3][0] + (0.66 - x_now) / 0.034, abs=0.02)
    fast = track(v=0.052)                                          # 지난 통과보다 18 mm/s 빠르다: 맞춤을 쓴다
    e = arrival_estimate(fast, 0.66, 0.085, fast[-1][0] + 0.05, v_prior=0.034)
    assert e['prior'] is False and e['v'] == pytest.approx(0.052, abs=1e-6)


def test_time_on_the_roller_is_added_for_a_cap_that_just_appeared():
    pts = [(100.0 + k / 25.0, 0.546 + 0.030 * k / 25.0, 0.09) for k in range(4)]     # 롤러 위, 30 mm/s
    e = arrival_estimate(pts, 0.67, 0.09, pts[-1][0] + 0.05, v_prior=0.050)
    x_ref, t_ref = np.median([q[1] for q in pts]), np.median([q[0] for q in pts])
    expect = t_ref + (0.56 - x_ref) / 0.030 + (0.67 - 0.56) / 0.050
    assert e['t_arrive'] == pytest.approx(expect, abs=1e-6)


def test_last_pass_model_takes_the_newest_complete_pass():
    old = [(t, x, y, 0.084) for t, x, y in track(v=0.030, x0=0.55, t0=100.0, n=160)]
    new = [(t, x, y, 0.0835) for t, x, y in track(v=0.052, x0=0.55, t0=120.0, n=100, y=0.091)]
    m = last_pass_model(old + new)
    assert m['v'] == pytest.approx(0.052, abs=1e-6) and m['y'] == pytest.approx(0.091) and m['z'] == pytest.approx(0.0835)
    assert last_pass_model(old + new[:12])['v'] == pytest.approx(0.030, abs=1e-6)      # 새 통과가 아직 짧으면 지난 통과
    assert last_pass_model([]) is None


def cap_flags(t0, n, inside, x0=0.70, v=0.04, hz=25.0, valid=True):
    return [(t0 + k / hz, x0 + v * k / hz, inside, valid) for k in range(n)]


def test_judge_counts_only_frames_after_the_trigger_and_past_the_gripper():
    flags = cap_flags(99.0, 20, False) + cap_flags(100.5, 60, True) + cap_flags(100.5, 5, False, x0=0.60)
    out = judge(flags, 100.0, 0.69)
    assert out == {'result': 'SUCCESS', 'si_true': 60, 'si_false': 0}
    assert judge(cap_flags(100.5, 60, False), 100.0, 0.69)['result'] == 'MISS'
    assert judge(cap_flags(100.5, 3, True), 100.0, 0.69)['result'] == 'UNKNOWN'            # 프레임이 모자람
    assert judge(cap_flags(100.5, 60, True, valid=False), 100.0, 0.69)['result'] == 'UNKNOWN'


def test_judge_ignores_the_empty_cap_of_the_next_lap():
    flags = cap_flags(100.5, 60, True) + cap_flags(114.0, 80, False)                       # 다음 바퀴: 빈 뚜껑
    assert judge(flags, 100.0, 0.69) == {'result': 'SUCCESS', 'si_true': 60, 'si_false': 0}
    assert judge(flags, 100.0, 0.69, window_s=30.0)['result'] == 'UNKNOWN'                 # 창이 없으면 섞인다


def test_crossing_interpolates_time_and_y():
    samples = [(10.0, 0.60, 0.080), (10.1, 0.62, 0.082), (10.2, 0.64, 0.084)]
    t, y = crossing(samples, 0.63)
    assert t == pytest.approx(10.15)
    assert y == pytest.approx(0.083)
    assert crossing([(0.0, 0.5, 0.0), (1.0, 0.55, 0.0)], 0.6) is None
    assert crossing([(1.0, 0.7, 0.0), (0.0, 0.5, 0.0)], 0.6)[0] == pytest.approx(0.5)      # 시각 순서가 섞여도 된다


def test_move_time_trapezoid_and_triangle():
    assert move_time_s(0.35, 0.1, 0.3) == pytest.approx(0.35 / 0.1 + 0.1 / 0.3)
    short = 0.01            # 0.1²/0.3 = 0.033 m보다 짧으면 최고 속도에 닿지 않는다
    assert move_time_s(short, 0.1, 0.3) == pytest.approx(2 * (short / 0.3) ** 0.5)
