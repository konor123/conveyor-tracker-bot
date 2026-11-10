"""cycle_time_budget.py 단위 시험: 로그에서 단계 시간을 읽고 정책별 추정이 모델 식과 맞는지 확인한다."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cycle_time_budget as ctb  # noqa: E402

DURATIONS = [('open_before_pick', 6.0), ('over_stone', 3.0), ('descend_grasp', 3.0), ('close_grasp', 7.0),
             ('lift_stone', 3.0), ('over_cap', 2.0), ('descend_release', 1.0), ('open_release', 7.0),
             ('retreat', 1.0), ('park', 3.0)]


def write_log(path: Path, durations=DURATIONS, extra_cycle=False) -> Path:
    t = 100.0
    rows = [{'t': 99.0, 'event': 'plan', 'step': 'over_stone'}, {'t': t, 'event': 'ready'}]
    for name, sec in durations:
        t += sec
        rows.append({'t': t, 'event': 'done', 'step': name})
    rows.append({'t': t + 0.3, 'event': 'cycle_done'})
    if extra_cycle:
        rows += [{'t': t + 1.0, 'event': 'ready'}, {'t': t + 99.0, 'event': 'done', 'step': 'open_before_pick'}]
    path.write_text('\n'.join(json.dumps(r) for r in rows) + '\n', encoding='utf-8')
    return path


def by_name(rows):
    return {r['policy']: r for r in rows}


def test_reads_step_durations_of_first_cycle(tmp_path):
    steps = ctb.read_steps(write_log(tmp_path / 'run.jsonl', extra_cycle=True))
    assert [s.name for s in steps] == [n for n, _ in DURATIONS]
    assert [round(s.seconds, 6) for s in steps] == [d for _, d in DURATIONS]
    assert [s.name for s in steps if s.gripper] == ['open_before_pick', 'close_grasp', 'open_release']


def test_log_without_steps_is_an_error(tmp_path):
    p = tmp_path / 'empty.jsonl'
    p.write_text(json.dumps({'t': 1.0, 'event': 'dry_run_done'}) + '\n', encoding='utf-8')
    with pytest.raises(ValueError):
        ctb.read_steps(p)


def test_policy_estimates_follow_the_model(tmp_path):
    steps = ctb.read_steps(write_log(tmp_path / 'run.jsonl'))
    rows = by_name(ctb.table(steps, rtt_s=0.5, open_polls=1, grasp_polls=2))

    assert rows['measured'] == {'policy': 'measured', 'gripper_s': 20.0, 'move_s': 16.0, 'total_s': 36.0,
                                'saved_s': 0.0, 'estimated': False}
    assert rows['skip_first_open']['total_s'] == 30.0                       # 첫 열기 6 s만 빠진다
    # 상주 루프: 열기 0.5·(1+1) = 1.0 두 번, 파지 0.5·(1+2) = 1.5
    assert rows['resident_loop']['gripper_s'] == 3.5
    assert rows['resident_loop+skip_first_open']['gripper_s'] == 2.5
    # 첫 열기 1.0 s는 다음 이동 3.0 s보다 짧아 기다리는 시간이 없다
    assert rows['resident_loop+overlap_first_open']['gripper_s'] == 2.5
    assert all(r['move_s'] == 16.0 for r in rows.values())                  # 이동은 바꾸지 않는다
    assert all(r['estimated'] for name, r in rows.items() if name != 'measured')


def test_overlap_waits_only_for_the_excess(tmp_path):
    steps = ctb.read_steps(write_log(tmp_path / 'run.jsonl'))
    out = ctb.apply(steps, ctb.Policy('x', resident_loop=True, overlap_first_open=True),
                    rtt_s=2.0, open_polls=1, grasp_polls=2)                 # 열기 4.0 s > 이동 3.0 s
    assert out[0].seconds == pytest.approx(1.0)


def test_cli_prints_every_policy(tmp_path, capsys):
    assert ctb.main([str(write_log(tmp_path / 'run.jsonl')), '--rtt', '0.58', '0.1']) == 0
    text = capsys.readouterr().out
    assert text.count('resident_loop+skip_first_open') == 2 and '(추정)' in text


def test_trapezoid_time_and_short_move():
    assert ctb.trapezoid_s(0.139, 0.05, 0.15) == pytest.approx(0.139 / 0.05 + 0.05 / 0.15)
    assert ctb.trapezoid_s(0.001, 0.1, 0.3) == pytest.approx(2 * (0.001 / 0.3) ** 0.5)   # 최고 속도에 못 미친다
    assert ctb.trapezoid_s(0.0, 0.1, 0.3) == 0.0


def test_retime_keeps_measured_times_at_base_speed_and_shortens_vertical(tmp_path):
    plan = [('over_stone', (0.5, 0.0, 0.3)), ('descend_grasp', (0.5, 0.0, 0.15)), ('lift_stone', (0.5, 0.0, 0.3)),
            ('over_cap', (0.6, 0.1, 0.3)), ('descend_release', (0.6, 0.1, 0.25)), ('retreat', (0.6, 0.1, 0.3)),
            ('park', (0.4, 0.0, 0.4))]
    steps = ctb.read_steps(write_log(tmp_path / 'run.jsonl'))
    same, residuals = ctb.retime_moves(steps, plan)
    assert [s.seconds for s in same] == pytest.approx([s.seconds for s in steps])     # 같은 속도면 측정값 그대로
    assert set(residuals) == {name for name, _ in plan}

    fast, _ = ctb.retime_moves(steps, plan, vertical=(0.1, 0.3))
    by = {s.name: s.seconds for s in fast}
    base = {s.name: s.seconds for s in steps}
    saved = ctb.trapezoid_s(0.15, 0.05, 0.15) - ctb.trapezoid_s(0.15, 0.1, 0.3)
    assert base['descend_grasp'] - by['descend_grasp'] == pytest.approx(saved)
    assert by['over_cap'] == pytest.approx(base['over_cap'])                           # 수평 구간은 그대로
    assert by['close_grasp'] == base['close_grasp']
